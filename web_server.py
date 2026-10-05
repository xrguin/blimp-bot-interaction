"""Offline localhost browser GUI. Run: python web_server.py --no-browser."""
from __future__ import annotations

import argparse
import asyncio
from contextlib import asynccontextmanager, suppress
import json
from pathlib import Path
import threading
from urllib.parse import urlsplit
import webbrowser

import anyio
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, Response
from fastapi.staticfiles import StaticFiles
from starlette.middleware.trustedhost import TrustedHostMiddleware

from sim.web_runtime import SimulationRuntime

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

    app = FastAPI(lifespan=lifespan, docs_url=None, redoc_url=None)
    app.state.runtime = runtime
    app.state.controller = None
    app.add_middleware(TrustedHostMiddleware, allowed_hosts=["localhost", "127.0.0.1", "[::1]"])

    @app.get("/health")
    async def health():
        return {"status": "ok"}

    @app.get("/api/config")
    async def config():
        return runtime.config()

    @app.get("/api/log.npz")
    async def export():
        data = await asyncio.wrap_future(runtime.export())
        return Response(data, media_type="application/octet-stream", headers={"Content-Disposition": 'attachment; filename="blimp-session.npz"', "Cache-Control": "no-store"})

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
