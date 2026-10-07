"""Browser GUI server. Local use: python web_server.py --no-browser. Public use: see DEPLOY.md.

Every visitor gets a private simulation (own worker thread, seed, rovers, cameras), identified
by an HttpOnly cookie, capped in number and stopped after an idle period; `--shared` restores the
single world everyone sees. The server always binds to the loopback interface; a tunnel or
reverse proxy (Cloudflare Tunnel, Tailscale, nginx) publishes it, and `--public-host` tells the
server which public hostnames to accept. `--access-token` adds an application-level shared secret.
"""
from __future__ import annotations

import argparse
import asyncio
from contextlib import asynccontextmanager, suppress
import json
from pathlib import Path
import secrets
import threading
from urllib.parse import urlsplit
import webbrowser

# Several visitor sessions render cameras from their own threads. GLFW's X11 layer is not
# thread-safe and aborts the process when contexts are created concurrently, so the server
# renders through EGL even when a display is present (override with MUJOCO_GL=glfw if needed).
import os
os.environ.setdefault("MUJOCO_GL", "egl")

import anyio
from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse, RedirectResponse, Response
from fastapi.staticfiles import StaticFiles
from starlette.middleware.trustedhost import TrustedHostMiddleware

from sim.web_runtime import SimulationRuntime
from sim.web_sessions import SessionLimitError, SessionManager, parse_session_options

WEB_ROOT = Path(__file__).resolve().parent / "web"
LOOPBACK_HOSTS = {"localhost", "127.0.0.1", "::1"}
TOKEN_COOKIE = "blimp_token"
WS_NO_SESSION, WS_UNAUTHORIZED = 4404, 4401       # application close codes the page reacts to


def cameras_enabled(rover_backend: str, flag) -> bool:
    """Camera streams default to on with the MuJoCo backend; --cameras/--no-cameras override."""
    return rover_backend == "mujoco" if flag is None else bool(flag)


def allowed_origin(websocket, public_hosts=()):
    """Accept same-origin loopback pages, or https pages served from a configured public host."""
    origin = websocket.headers.get("origin")
    if not origin:  # Local scripts/test clients have no browser Origin header.
        return True
    try:
        parsed = urlsplit(origin)
        if parsed.username or parsed.password or parsed.scheme not in ("http", "https"):
            return False
        if parsed.hostname in public_hosts:
            return parsed.scheme == "https"
        port = parsed.port or (443 if parsed.scheme == "https" else 80)
        server_port = websocket.url.port or (443 if websocket.url.scheme == "wss" else 80)
        return parsed.hostname in LOOPBACK_HOSTS and port == server_port
    except ValueError:
        return False


def is_public_request(request_or_ws, public_hosts) -> bool:
    host = (request_or_ws.headers.get("host") or "").split(":")[0].lower()
    return host in public_hosts


def create_app(runtime=None, *, factory=None, public_hosts=(), access_token=None, max_sessions=8, idle_timeout=600.0, max_rovers=8):
    """Build the FastAPI app.

    `runtime`: one shared SimulationRuntime for every visitor (original behaviour, used by tests).
    `factory(**options)`: per-visitor sessions; options come from the page's query string
    (n, seed, mode, rovers) validated by `parse_session_options`.
    """
    public_hosts = {h.lower() for h in public_hosts}
    if runtime is None and factory is None:
        runtime = SimulationRuntime()
    sessions = (SessionManager(shared_runtime=runtime) if runtime is not None
                else SessionManager(factory=factory, max_sessions=max_sessions, idle_timeout=idle_timeout))

    @asynccontextmanager
    async def lifespan(app):
        sessions.start()
        try:
            yield
        finally:
            await asyncio.to_thread(sessions.stop)

    app = FastAPI(lifespan=lifespan, docs_url=None, redoc_url=None)
    app.state.sessions = sessions
    app.add_middleware(TrustedHostMiddleware, allowed_hosts=["localhost", "127.0.0.1", "[::1]", *sorted(public_hosts)])

    # ------------------------------------------------------------ helpers
    def authorized(request_or_ws) -> bool:
        return access_token is None or secrets.compare_digest(request_or_ws.cookies.get(TOKEN_COOKIE, ""), access_token)

    def set_cookie(response, request, name, value):
        response.set_cookie(name, value, httponly=True, samesite="lax", secure=is_public_request(request, public_hosts),
                            max_age=7 * 24 * 3600, path="/")

    def current_session(request_or_ws):
        return sessions.get(request_or_ws.cookies.get(SessionManager.COOKIE))

    def no_session():
        return JSONResponse({"error": "no session", "hint": "load /api/config first"}, status_code=404,
                            headers={"Cache-Control": "no-store"})

    # ------------------------------------------------------------ routes
    @app.get("/health")
    async def health():
        return {"status": "ok", "sessions": sessions.count(), "max_sessions": sessions.max_sessions}

    @app.get("/")
    async def index(request: Request):
        token = request.query_params.get("token")
        if access_token is not None and token is not None:
            # Visiting /?token=... stores the secret in a cookie and drops it from the address bar.
            if not secrets.compare_digest(token, access_token):
                return Response("Invalid access token", status_code=401)
            response = RedirectResponse("/", status_code=303)
            set_cookie(response, request, TOKEN_COOKIE, token)
            return response
        return FileResponse(WEB_ROOT / "index.html", headers={"Cache-Control": "no-cache"})

    @app.get("/api/config")
    async def config(request: Request):
        """Create (or resume) the visitor's session and return its configuration."""
        if not authorized(request):
            return JSONResponse({"error": "access token required"}, status_code=401)
        try:
            options = parse_session_options(request.query_params, max_rovers=max_rovers)
        except ValueError as exc:
            return JSONResponse({"error": str(exc)}, status_code=400)
        try:
            session, created = await asyncio.to_thread(sessions.get_or_create, request.cookies.get(SessionManager.COOKIE), options)
        except SessionLimitError as exc:
            return JSONResponse({"error": str(exc)}, status_code=409, headers={"Retry-After": "60"})
        payload = session.runtime.config()
        payload["session"] = sessions.describe(session)
        response = JSONResponse(payload, headers={"Cache-Control": "no-store"})
        if created or request.cookies.get(SessionManager.COOKIE) != session.id:
            set_cookie(response, request, SessionManager.COOKIE, session.id)
        return response

    @app.get("/api/log.npz")
    async def export(request: Request):
        if not authorized(request):
            return Response(status_code=401)
        session = current_session(request)
        if session is None:
            return no_session()
        data = await asyncio.wrap_future(session.runtime.export())
        return Response(data, media_type="application/octet-stream",
                        headers={"Content-Disposition": 'attachment; filename="blimp-session.npz"', "Cache-Control": "no-store"})

    @app.get("/api/camera/{name}")
    async def camera(name: str, request: Request):
        """Latest frame of one vehicle camera (JPEG when an encoder is installed, else PNG)."""
        if not authorized(request):
            return Response(status_code=401)
        session = current_session(request)
        if session is None:
            return no_session()
        runtime = session.runtime
        if name not in runtime.camera_names():
            return Response(status_code=404)
        frame = await asyncio.to_thread(runtime.camera_frame, name)
        if frame is None:                                     # worker has not rendered its first frame yet
            return Response(status_code=503, headers={"Retry-After": "1", "Cache-Control": "no-store"})
        frame_id, sim_time, data, media = frame
        etag = f'"{runtime.generation}-{frame_id}"'
        headers = {"Cache-Control": "no-store", "ETag": etag, "X-Frame-Id": str(frame_id), "X-Sim-Time": f"{sim_time:.3f}"}
        if request.headers.get("if-none-match") == etag:
            return Response(status_code=304, headers=headers)
        return Response(data, media_type=media, headers=headers)

    @app.websocket("/ws")
    async def websocket_endpoint(ws: WebSocket):
        if not allowed_origin(ws, public_hosts):
            await ws.close(code=1008)
            return
        if not authorized(ws):
            await ws.close(code=WS_UNAUTHORIZED)
            return
        session = current_session(ws)
        if session is None:
            await ws.close(code=WS_NO_SESSION)
            return
        runtime = session.runtime
        await ws.accept()
        sessions.connect(session)
        token = object()
        has_control = runtime.controller is None
        if has_control:
            runtime.controller = token
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
            await send({"type": "hello", "has_control": has_control, "session": sessions.describe(session)})
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
                if runtime.controller is token:
                    with suppress(RuntimeError, asyncio.CancelledError):
                        await asyncio.wrap_future(runtime.disconnect())
                    runtime.controller = None
                sessions.disconnect(session)
                with suppress(RuntimeError, WebSocketDisconnect):
                    await ws.close()

    app.mount("/static", StaticFiles(directory=WEB_ROOT, check_dir=False), name="static")
    return app


def main():
    parser = argparse.ArgumentParser(description="Blimp and rover control panel server (loopback; publish through a tunnel, see DEPLOY.md)")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--n", type=int, default=4, help="Default number of rovers (1–32); visitors may pass ?n= up to --max-rovers")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--mode", choices=["teleop", "auto"], default="teleop")
    parser.add_argument("--rovers", choices=["circle", "idle"], default="circle")
    parser.add_argument("--rovers-backend", choices=["ideal", "mujoco"], default="ideal",
                        help="ideal unicycle rovers (default) or contact-based MuJoCo rovers (needs the mujoco package)")
    parser.add_argument("--cameras", dest="cameras", action="store_true", default=None,
                        help="render the rover/blimp cameras and stream them to the browser (default with --rovers-backend mujoco)")
    parser.add_argument("--no-cameras", dest="cameras", action="store_false", help="disable the camera streams")
    parser.add_argument("--camera-size", default="640x480", metavar="WxH",
                        help="camera resolution for every session (default 640x480; 320x240 quarters render, encode and bandwidth cost)")
    parser.add_argument("--shared", action="store_true", help="one simulation shared by all visitors instead of a private one per visitor")
    parser.add_argument("--max-sessions", type=int, default=8, help="cap on concurrent private simulations")
    parser.add_argument("--max-rovers", type=int, default=8, help="largest rover count a visitor may request with ?n=")
    parser.add_argument("--idle-timeout", type=float, default=600.0, help="seconds without an open connection before a private simulation is stopped")
    parser.add_argument("--public-host", action="append", default=[], metavar="HOST",
                        help="public hostname the server is reached through (tunnel/proxy); repeatable. Enables https origins and secure cookies for it")
    parser.add_argument("--access-token", default=None, help="shared secret visitors must present once as /?token=... (stored in a cookie)")
    parser.add_argument("--no-browser", action="store_true")
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("Port must be between 1 and 65535")
    if not 1 <= args.max_sessions <= 64:
        parser.error("--max-sessions must be between 1 and 64")
    if not 1 <= args.max_rovers <= 32:
        parser.error("--max-rovers must be between 1 and 32")
    cameras = cameras_enabled(args.rovers_backend, args.cameras)
    try:
        camera_size = tuple(int(v) for v in args.camera_size.lower().split("x"))
        if len(camera_size) != 2:
            raise ValueError
    except ValueError:
        parser.error("--camera-size must look like 640x480")
    defaults = dict(n=args.n, seed=args.seed, mode=args.mode, rovers_mode=args.rovers, rover_backend=args.rovers_backend,
                    cameras=cameras, camera_size=camera_size)

    def factory(**options):
        return SimulationRuntime(**{**defaults, **options})

    try:
        if args.shared:
            app = create_app(factory(), public_hosts=args.public_host, access_token=args.access_token)
        else:
            factory()                                    # fail early (missing mujoco, bad options) before serving
            app = create_app(factory=factory, public_hosts=args.public_host, access_token=args.access_token,
                             max_sessions=args.max_sessions, idle_timeout=args.idle_timeout, max_rovers=args.max_rovers)
    except (ValueError, ImportError) as exc:
        parser.error(str(exc))
    url = f"http://127.0.0.1:{args.port}"
    print(f"Blimp control panel: {url}\nStarts paused. Press Run in the browser. Ctrl+C stops the server.")
    if args.public_host:
        print("Public hostnames accepted: " + ", ".join(args.public_host) + ("  (access token required)" if args.access_token else ""))
    print("Sessions: " + ("one shared simulation" if args.shared else f"private per visitor, up to {args.max_sessions}, idle timeout {args.idle_timeout:.0f} s"))
    timer = None
    if not args.no_browser:
        timer = threading.Timer(1.0, lambda: webbrowser.open(url))
        timer.daemon = True
        timer.start()
    try:
        import uvicorn
        uvicorn.run(app, host="127.0.0.1", port=args.port, ws_max_size=8192, proxy_headers=True)
    finally:
        if timer is not None:
            timer.cancel()


if __name__ == "__main__":
    main()
