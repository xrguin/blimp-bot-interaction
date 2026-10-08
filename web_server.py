"""Offline localhost browser GUI. Run: python web_server.py --no-browser."""
from __future__ import annotations

import argparse
import asyncio
from contextlib import asynccontextmanager, suppress
import json
import os
from pathlib import Path
import threading
from urllib.parse import urlsplit
import webbrowser

import anyio
from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, Response, JSONResponse
from fastapi.staticfiles import StaticFiles
from starlette.middleware.trustedhost import TrustedHostMiddleware

from sim.web_runtime import SimulationRuntime
from sim.web_sessions import SessionRegistry, SessionExpired, SessionCapacity

WEB_ROOT = Path(__file__).resolve().parent / "web"
LOOPBACK_HOSTS = {"localhost", "127.0.0.1", "::1"}


def allowed_origin(websocket):
    origin = websocket.headers.get("origin")
    if not origin:  # Local scripts/test clients have no browser Origin header.
        return True
    try:
        parsed = urlsplit(origin)
        port = parsed.port or (443 if parsed.scheme == "https" else 80)
        server_port = websocket.url.port or (443 if websocket.url.scheme == "wss" else 80)
        return parsed.scheme in ("http", "https") and parsed.hostname in LOOPBACK_HOSTS and port == server_port and not parsed.username and not parsed.password
    except ValueError:
        return False


def create_app(runtime=None):
    runtime = runtime or SimulationRuntime()

    @asynccontextmanager
    async def lifespan(app):
        runtime.start()
        try:
            yield
        finally:
            await asyncio.to_thread(runtime.stop)

    app = FastAPI(lifespan=lifespan, docs_url=None, redoc_url=None, openapi_url=None)
    app.state.runtime = runtime
    app.state.controller = None
    app.add_middleware(TrustedHostMiddleware, allowed_hosts=["localhost", "127.0.0.1", "[::1]"])

    @app.get("/health")
    async def health():
        return {"status": "ok"}

    @app.get("/api/config")
    async def config():
        return runtime.config()

    @app.post("/api/session")
    async def local_session():
        return JSONResponse({"mode": "shared", "token": None, "config": runtime.config(), "expires_at": None},
                            headers={"Cache-Control": "no-store"})

    @app.get("/api/log.npz")
    async def export():
        data = await asyncio.wrap_future(runtime.export())
        return Response(data, media_type="application/octet-stream", headers={"Content-Disposition": 'attachment; filename="blimp-session.npz"', "Cache-Control": "no-store"})

    @app.get("/api/camera/recording")
    async def export_camera():
        try:
            data = await asyncio.wrap_future(runtime.export_camera())
        except (ValueError, RuntimeError) as exc:
            return JSONResponse({"detail": str(exc)}, status_code=409, headers={"Cache-Control": "no-store"})
        return Response(data, media_type="application/json", headers={"Content-Disposition": 'attachment; filename="camera-recording.json"', "Cache-Control": "no-store"})

    @app.get("/")
    async def index():
        return FileResponse(WEB_ROOT / "index.html", headers={"Cache-Control": "no-cache"})

    @app.websocket("/ws")
    async def websocket_endpoint(ws: WebSocket):
        if not allowed_origin(ws):
            await ws.close(code=1008)
            return
        await ws.accept()
        token = object()
        has_control = app.state.controller is None
        if has_control:
            app.state.controller = token
        send_lock = asyncio.Lock()

        async def send(payload):
            async with send_lock:
                await asyncio.wait_for(ws.send_json(payload), timeout=2.0)

        async def receive_commands():
            while True:
                message = None
                try:
                    event = await ws.receive()
                    if event["type"] == "websocket.disconnect":
                        raise WebSocketDisconnect(event.get("code", 1000))
                    raw = event.get("text")
                    if raw is None:
                        raise ValueError("Commands must use JSON text messages")
                    if len(raw) > 8192:
                        raise ValueError("Command is too large")
                    message = json.loads(raw)
                    if not has_control:
                        raise ValueError("This tab is read-only. Close the controlling tab and reconnect to take control.")
                    await asyncio.wrap_future(runtime.submit(message))
                    await send({"type": "ack", "id": message["id"], "ok": True})
                except (ValueError, TypeError, RuntimeError) as exc:
                    command_id = message.get("id") if isinstance(message, dict) else None
                    if not isinstance(command_id, str):
                        command_id = None
                    await send({"type": "error", "id": command_id, "message": str(exc)})

        async def publish_states():
            while True:
                await send(runtime.snapshot())
                await asyncio.sleep(0.05)

        tasks = []
        try:
            await send({"type": "hello", "has_control": has_control})
            tasks = [asyncio.create_task(receive_commands()), asyncio.create_task(publish_states())]
            done, _ = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
            for task in done:
                task.result()
        except (WebSocketDisconnect, RuntimeError, OSError, asyncio.TimeoutError, asyncio.CancelledError):
            pass
        finally:
            # Starlette/server shutdown can cancel the connection task. Complete
            # control release before making ownership available to another tab.
            with anyio.CancelScope(shield=True):
                for task in tasks:
                    task.cancel()
                await asyncio.gather(*tasks, return_exceptions=True)
                if app.state.controller is token:
                    with suppress(RuntimeError, asyncio.CancelledError):
                        await asyncio.wrap_future(runtime.disconnect())
                    app.state.controller = None
                with suppress(RuntimeError, WebSocketDisconnect):
                    await ws.close()

    app.mount("/static", StaticFiles(directory=WEB_ROOT, check_dir=False), name="static")
    return app


def _public_origin(value):
    try:
        parsed = urlsplit(value)
        if (parsed.scheme != "https" or not parsed.hostname or parsed.username or parsed.password or
                parsed.path not in ("", "/") or parsed.query or parsed.fragment or parsed.port not in (None, 443)):
            raise ValueError
        if parsed.hostname in LOOPBACK_HOSTS or "*" in parsed.hostname:
            raise ValueError
        return f"https://{parsed.hostname}", parsed.hostname
    except (ValueError, TypeError):
        raise ValueError("PUBLIC_ORIGIN must be one exact public HTTPS origin, such as https://example.onrender.com") from None


def create_public_app(public_origin=None, registry=None):
    """Isolated visitors; intentionally one process/worker, with no persistent files."""
    public_origin = public_origin or os.environ.get("PUBLIC_ORIGIN")
    if not public_origin:
        hostname = os.environ.get("RENDER_EXTERNAL_HOSTNAME")
        if not hostname:
            raise ValueError("Set PUBLIC_ORIGIN or RENDER_EXTERNAL_HOSTNAME in public mode")
        public_origin = f"https://{hostname}"
    origin, hostname = _public_origin(public_origin)
    registry = registry or SessionRegistry(max_sessions=int(os.environ.get("BLIMP_MAX_SESSIONS", "2")))

    @asynccontextmanager
    async def lifespan(app):
        async def cleanup():
            while True:
                await asyncio.sleep(1)
                await asyncio.to_thread(registry.cleanup)
        cleanup_task = asyncio.create_task(cleanup())
        try:
            yield
        finally:
            cleanup_task.cancel()
            await asyncio.gather(cleanup_task, return_exceptions=True)
            await asyncio.to_thread(registry.close)

    app = FastAPI(lifespan=lifespan, docs_url=None, redoc_url=None, openapi_url=None)
    app.state.sessions = registry
    app.state.public_origin = origin
    # Render's internal HTTP health probe may use localhost; only /health is
    # served for those hosts. Public app requests still require the exact host.
    app.add_middleware(TrustedHostMiddleware, allowed_hosts=[hostname, "localhost", "127.0.0.1", "[::1]"])

    @app.middleware("http")
    async def public_headers(request, call_next):
        if request.url.hostname != hostname and request.url.path != "/health":
            return JSONResponse({"detail": "Invalid host"}, status_code=400)
        response = await call_next(request)
        response.headers["X-Content-Type-Options"] = "nosniff"
        response.headers["Referrer-Policy"] = "no-referrer"
        if request.url.path.startswith("/api/"):
            response.headers["Cache-Control"] = "no-store"
        return response

    @app.get("/health")
    async def health():
        return {"status": "ok"}

    @app.get("/api/config")
    async def metadata():
        return {"mode": "isolated", "public_limits": registry.limits()}

    @app.post("/api/session")
    async def new_session(request: Request):
        if request.headers.get("origin") != origin:
            return JSONResponse({"error": "invalid_origin", "detail": "Open this simulator from its own website to start a flight."}, status_code=403)
        try:
            session = await asyncio.to_thread(registry.create)
        except SessionCapacity as exc:
            return JSONResponse({"error": "capacity", "detail": str(exc)}, status_code=503, headers={"Retry-After": "10"})
        config = session.runtime.config()
        config["public_limits"] = registry.limits()
        return {"mode": "isolated", "token": session.token, "config": config, "expires_at": session.expires_at}

    async def session_export(request, camera=False):
        authorization = request.headers.get("authorization", "")
        token = authorization[7:] if authorization.startswith("Bearer ") else None
        session = None
        try:
            session = registry.begin_export(token)
            future = session.runtime.export_camera() if camera else session.runtime.export()
            data = await asyncio.wrap_future(future)
            return Response(data, media_type="application/json" if camera else "application/octet-stream",
                            headers={"Content-Disposition": 'attachment; filename="camera-recording.json"' if camera else 'attachment; filename="blimp-session.npz"'})
        except SessionExpired as exc:
            return JSONResponse({"error": "session_expired", "detail": str(exc)}, status_code=401)
        except OverflowError as exc:
            return JSONResponse({"error": "export_limit", "detail": str(exc)}, status_code=413)
        except (ValueError, RuntimeError) as exc:
            return JSONResponse({"error": "export_unavailable", "detail": str(exc)}, status_code=409)
        finally:
            if session is not None:
                registry.end_export(session)

    @app.get("/api/log.npz")
    async def export(request: Request):
        return await session_export(request)

    @app.get("/api/camera/recording")
    async def export_camera(request: Request):
        return await session_export(request, camera=True)

    @app.get("/")
    async def index():
        return FileResponse(WEB_ROOT / "index.html", headers={"Cache-Control": "no-cache"})

    @app.websocket("/ws")
    async def socket(ws: WebSocket):
        if ws.headers.get("origin") != origin or ws.url.hostname != hostname:
            await ws.close(code=1008)
            return
        await ws.accept()
        token, lease = None, None
        tasks = []
        send_lock = asyncio.Lock()

        async def send(payload):
            async with send_lock:
                await asyncio.wait_for(ws.send_json(payload), timeout=2)

        async def session_error(code, message, close_code):
            await send({"type": "session_error", "code": code, "message": message})
            await ws.close(code=close_code)

        def assert_owner():
            if not registry.ownership(token, lease):
                raise SessionExpired("This connection was replaced by another connection.")

        async def receive_commands():
            while True:
                message = None
                try:
                    event = await ws.receive()
                    if event["type"] == "websocket.disconnect":
                        raise WebSocketDisconnect(event.get("code", 1000))
                    assert_owner()
                    raw = event.get("text")
                    if raw is None:
                        raise ValueError("Commands must use JSON text messages")
                    if len(raw) > 8192:
                        raise ValueError("Command is too large")
                    message = json.loads(raw)
                    await asyncio.wrap_future(registry.submit(token, lease, message))
                    await send({"type": "ack", "id": message["id"], "ok": True})
                except SessionExpired:
                    raise
                except (ValueError, TypeError, RuntimeError) as exc:
                    command_id = message.get("id") if isinstance(message, dict) else None
                    await send({"type": "error", "id": command_id if isinstance(command_id, str) else None, "message": str(exc)})

        async def publish_states(runtime):
            while True:
                assert_owner()
                await send(runtime.snapshot())
                await asyncio.sleep(0.05)

        try:
            event = await asyncio.wait_for(ws.receive(), timeout=5)
            raw = event.get("text")
            try:
                if not isinstance(raw, str) or len(raw) > 512:
                    raise ValueError
                attach = json.loads(raw)
                if not isinstance(attach, dict) or set(attach) != {"type", "token"} or attach["type"] != "attach":
                    raise ValueError
            except (ValueError, TypeError):
                await session_error("invalid_attach", "Attach a valid flight session before sending commands.", 1008)
                return
            token = attach["token"]
            session, lease = registry.claim(token)
            await send({"type": "hello", "has_control": True, "mode": "isolated", "expires_at": session.expires_at})
            tasks = [asyncio.create_task(receive_commands()), asyncio.create_task(publish_states(session.runtime))]
            done, _ = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
            for task in done:
                task.result()
        except SessionExpired as exc:
            superseded = "replaced" in str(exc)
            with suppress(RuntimeError, WebSocketDisconnect, asyncio.TimeoutError, OSError):
                await session_error("session_superseded" if superseded else "session_expired", str(exc), 4002 if superseded else 4001)
        except asyncio.TimeoutError:
            with suppress(RuntimeError, WebSocketDisconnect, asyncio.TimeoutError, OSError):
                if lease is None:
                    await session_error("invalid_attach", "Flight attachment timed out.", 1008)
                else:
                    # A slow/lost connection may reconnect with the same token;
                    # it is not an invalid or expired session attachment.
                    await ws.close(code=1011)
        except (WebSocketDisconnect, RuntimeError, OSError, asyncio.CancelledError):
            pass
        finally:
            with anyio.CancelScope(shield=True):
                for task in tasks:
                    task.cancel()
                await asyncio.gather(*tasks, return_exceptions=True)
                if lease is not None:
                    pending = registry.release(token, lease)
                    if pending is not None:
                        with suppress(RuntimeError, asyncio.CancelledError):
                            await asyncio.wrap_future(pending)
                with suppress(RuntimeError, WebSocketDisconnect):
                    await ws.close()

    app.mount("/static", StaticFiles(directory=WEB_ROOT, check_dir=False), name="static")
    return app


def create_deployment_app():
    """Uvicorn factory; public exposure is always an explicit deployment choice."""
    return create_public_app() if os.environ.get("BLIMP_PUBLIC") == "1" else create_app()


def main():
    parser = argparse.ArgumentParser(description="Offline localhost blimp and rover control panel")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--n", type=int, default=4, help="Number of rovers (1–32)")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--mode", choices=["teleop", "auto"], default="teleop")
    parser.add_argument("--rovers", choices=["circle", "idle"], default="circle")
    parser.add_argument("--no-browser", action="store_true")
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("Port must be between 1 and 65535")
    try:
        runtime = SimulationRuntime(n=args.n, seed=args.seed, mode=args.mode, rovers_mode=args.rovers)
    except ValueError as exc:
        parser.error(str(exc))
    url = f"http://127.0.0.1:{args.port}"
    print(f"Blimp control panel: {url}\nStarts paused. Press Run in the browser. Ctrl+C stops the server.")
    timer = None
    if not args.no_browser:
        timer = threading.Timer(1.0, lambda: webbrowser.open(url))
        timer.daemon = True
        timer.start()
    try:
        import uvicorn
        uvicorn.run(create_app(runtime), host="127.0.0.1", port=args.port, ws_max_size=8192)
    finally:
        if timer is not None:
            timer.cancel()


if __name__ == "__main__":
    main()
