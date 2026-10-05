"""Public visitor isolation, token/Origin boundaries, lifetime and memory caps."""
import io
import json
import time
import unittest
from unittest.mock import patch

import numpy as np
from fastapi.testclient import TestClient
from starlette.websockets import WebSocketDisconnect

from sim.web_runtime import SimulationRuntime, validate_command
from sim.web_sessions import SessionRegistry, SessionExpired, SessionCapacity
from web_server import create_app, create_public_app, create_deployment_app


ORIGIN = "https://blimp-test.onrender.com"


def command(action, **values):
    return {"type": "command", "id": "public-test", "action": action, **values}


def receive_kind(socket, kind):
    for _ in range(100):
        message = socket.receive_json()
        if message["type"] == kind:
            return message
    raise AssertionError(f"Did not receive {kind}")


class Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


class RegistryTests(unittest.TestCase):
    def test_disconnected_idle_and_hard_expiry_stop_workers(self):
        for reason in ("disconnected", "idle", "hard"):
            with self.subTest(reason=reason):
                clock = Clock()
                registry = SessionRegistry(clock=clock, disconnected_grace=5, idle_seconds=10, hard_seconds=20)
                try:
                    session = registry.create()
                    if reason != "disconnected":
                        _, lease = registry.claim(session.token)
                    if reason == "disconnected":
                        clock.now += 5
                    elif reason == "idle":
                        # Empty key heartbeats must not prevent inactivity expiry.
                        clock.now += 9
                        registry.submit(session.token, lease, command("keys", keys=[])).result(timeout=1)
                        clock.now += 1
                    else:
                        for _ in range(3):
                            clock.now += 5
                            registry.submit(session.token, lease, command("gain", value=0.4)).result(timeout=1)
                        clock.now += 5
                    with self.assertRaises(SessionExpired):
                        registry.claim(session.token)
                    self.assertEqual(registry.cleanup(), 1)
                    self.assertFalse(session.runtime._thread.is_alive())
                finally:
                    registry.close()

    def test_capacity_reclaim_and_safe_lease_release(self):
        clock = Clock()
        registry = SessionRegistry(max_sessions=1, clock=clock)
        try:
            session = registry.create()
            with self.assertRaises(SessionCapacity):
                registry.create()
            _, old = registry.claim(session.token)
            _, new = registry.claim(session.token)
            self.assertIsNone(registry.release(session.token, old))
            registry.submit(session.token, new, command("pause", value=False)).result(timeout=1)
            self.assertFalse(session.runtime.snapshot()["paused"])
            registry.release(session.token, new).result(timeout=1)
            self.assertTrue(session.runtime.snapshot()["paused"])
            clock.now += 59
            same, _ = registry.claim(session.token)
            self.assertIs(same.runtime, session.runtime)
            clock.now += 1800
            next_session = registry.create()
            self.assertNotEqual(next_session.token, session.token)
            self.assertFalse(session.runtime._thread.is_alive())
        finally:
            registry.close()

    def test_cleanup_stops_outside_registry_lock(self):
        clock = Clock()
        registry = SessionRegistry(clock=clock)
        session = registry.create()
        original_stop = session.runtime.stop
        def stop():
            acquired = registry._lock.acquire(blocking=False)
            self.assertTrue(acquired)
            if acquired:
                registry._lock.release()
            original_stop()
        with patch.object(session.runtime, "stop", side_effect=stop):
            clock.now += 60
            registry.cleanup()


class PublicRuntimeTests(unittest.TestCase):
    def test_log_cap_pauses_and_preserves_recording_until_reset(self):
        runtime = SimulationRuntime(max_log_steps=3)
        runtime._apply(validate_command(command("camera_record_start")), time.monotonic())
        for _ in range(5):
            runtime._step()
        runtime._publish()
        self.assertEqual(runtime.sim.k, 3)
        self.assertEqual(len(runtime.sim.log["t"]), 3)
        self.assertTrue(runtime.snapshot()["paused"])
        self.assertIn("Reset", runtime.snapshot()["limit_message"])
        recording = json.loads(runtime._export_camera())
        self.assertEqual(recording["reason"], "log_limit")
        self.assertEqual(recording["frames"][-1]["control_step"], 3)
        with self.assertRaises(ValueError):
            runtime._apply(validate_command(command("pause", value=False)), time.monotonic())
        runtime._apply(validate_command(command("reset")), time.monotonic())
        runtime._apply(validate_command(command("pause", value=False)), time.monotonic())
        runtime._step()
        self.assertEqual(runtime.sim.k, 1)
        self.assertIsNone(runtime.limit_message)
        self.assertEqual(json.loads(runtime._export_camera()), recording)
        self.assertIsNone(SimulationRuntime().max_log_steps)

    def test_export_size_bound(self):
        runtime = SimulationRuntime(max_export_bytes=10)
        with self.assertRaises(OverflowError):
            runtime._export_npz()
        runtime._apply(validate_command(command("camera_record_start")), time.monotonic())
        runtime._apply(validate_command(command("camera_record_stop")), time.monotonic())
        with self.assertRaises(OverflowError):
            runtime._export_camera()


class PublicHTTPTests(unittest.TestCase):
    def setUp(self):
        self.registry = SessionRegistry()
        self.client = TestClient(create_public_app(ORIGIN, self.registry), base_url=ORIGIN)
        self.client.__enter__()

    def tearDown(self):
        self.client.__exit__(None, None, None)

    def new_session(self):
        response = self.client.post("/api/session", headers={"origin": ORIGIN})
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.headers["cache-control"], "no-store")
        return response.json()

    def socket(self, token):
        return self.client.websocket_connect(ORIGIN.replace("https", "wss") + "/ws", headers={"origin": ORIGIN})

    def test_bootstrap_origin_host_and_filesystem_exposure(self):
        for origin in (None, "null", "https://evil.example", ORIGIN + "/", "http://blimp-test.onrender.com"):
            headers = {} if origin is None else {"origin": origin}
            self.assertEqual(self.client.post("/api/session", headers=headers).status_code, 403)
        session = self.new_session()
        self.assertEqual(session["mode"], "isolated")
        self.assertEqual(len(session["token"]), 43)
        self.assertTrue(session["expires_at"].endswith("Z"))
        self.assertEqual(session["config"]["public_limits"]["max_log_steps"], 12000)
        self.assertNotIn(session["token"], self.client.get("/api/config").text)
        self.assertEqual(self.client.get("/health", headers={"host": "localhost"}).status_code, 200)
        self.assertEqual(self.client.get("/", headers={"host": "localhost"}).status_code, 400)
        self.assertEqual(self.client.get("/health", headers={"host": "evil.example"}).status_code, 400)
        for path in ("/docs", "/openapi.json", "/redoc", "/results/circle.npz", "/AGENTS.md", "/.git/config", "/static/../README.md", "/static/%2e%2e/README.md"):
            with self.subTest(path=path):
                self.assertEqual(self.client.get(path).status_code, 404)

    def test_two_visitors_mutations_recordings_and_exports_are_isolated(self):
        first, second = self.new_session(), self.new_session()
        self.assertNotEqual(first["token"], second["token"])
        with self.socket(first["token"]) as a, self.socket(second["token"]) as b:
            a.send_json({"type": "attach", "token": first["token"]})
            b.send_json({"type": "attach", "token": second["token"]})
            self.assertTrue(receive_kind(a, "hello")["has_control"])
            self.assertTrue(receive_kind(b, "hello")["has_control"])
            for action, values in (("param", {"key": "task.blimp_height", "value": 1.7}), ("camera_record_start", {}), ("camera_record_stop", {})):
                a.send_json(command(action, **values))
                self.assertTrue(receive_kind(a, "ack")["ok"])
            first_runtime = self.registry._sessions[first["token"]].runtime
            second_runtime = self.registry._sessions[second["token"]].runtime
            self.assertEqual(first_runtime.snapshot()["desired_altitude"], 1.7)
            self.assertEqual(second_runtime.snapshot()["desired_altitude"], 0)
            first_auth = {"authorization": f"Bearer {first['token']}"}
            second_auth = {"authorization": f"Bearer {second['token']}"}
            recording = self.client.get("/api/camera/recording", headers=first_auth)
            self.assertEqual(recording.status_code, 200)
            self.assertEqual(len(recording.json()["frames"]), 1)
            self.assertEqual(self.client.get("/api/camera/recording", headers=second_auth).status_code, 409)
            for headers in ({}, {"authorization": "Bearer missing"}):
                self.assertEqual(self.client.get("/api/log.npz", headers=headers).status_code, 401)
                self.assertEqual(self.client.get("/api/camera/recording", headers=headers).status_code, 401)
            self.assertEqual(self.client.get(f"/api/log.npz?token={first['token']}").status_code, 401)
            with np.load(io.BytesIO(self.client.get("/api/log.npz", headers=first_auth).content)) as data:
                self.assertEqual(data["n_rovers"], 4)

    def test_missing_origin_bad_attach_and_bad_token_rejected(self):
        with self.assertRaises(WebSocketDisconnect):
            with self.client.websocket_connect(ORIGIN.replace("https", "wss") + "/ws"):
                pass
        for attach, code in ((command("reset"), 1008), ({"type": "attach", "token": "bad"}, 4001)):
            with self.socket(None) as socket:
                socket.send_json(attach)
                self.assertEqual(receive_kind(socket, "session_error")["code"], "invalid_attach" if code == 1008 else "session_expired")
                with self.assertRaises(WebSocketDisconnect) as caught:
                    socket.receive_json()
                self.assertEqual(caught.exception.code, code)

    def test_new_lease_does_not_pause_when_old_socket_closes(self):
        session = self.new_session()
        first_context = self.socket(session["token"])
        first = first_context.__enter__()
        first.send_json({"type": "attach", "token": session["token"]})
        receive_kind(first, "hello")
        with self.socket(session["token"]) as second:
            second.send_json({"type": "attach", "token": session["token"]})
            receive_kind(second, "hello")
            second.send_json(command("pause", value=False))
            receive_kind(second, "ack")
            self.assertEqual(receive_kind(first, "session_error")["code"], "session_superseded")
            first_context.__exit__(None, None, None)
            runtime = self.registry._sessions[session["token"]].runtime
            self.assertFalse(runtime.snapshot()["paused"])
        self.assertTrue(runtime.snapshot()["paused"])

    def test_capacity_response_is_retryable(self):
        self.assertEqual(self.registry.max_sessions, 2)
        for _ in range(2):
            self.new_session()
        response = self.client.post("/api/session", headers={"origin": ORIGIN})
        self.assertEqual(response.status_code, 503)
        self.assertEqual(response.json()["error"], "capacity")
        self.assertEqual(response.headers["retry-after"], "10")

    def test_reconnect_retains_flight_and_expiry_rejects_socket_and_exports(self):
        clock = Clock()
        self.registry._clock = clock
        session = self.new_session()
        with self.socket(session["token"]) as first:
            first.send_json({"type": "attach", "token": session["token"]})
            receive_kind(first, "hello")
            first.send_json(command("param", key="task.blimp_height", value=1.2))
            receive_kind(first, "ack")
        clock.now += 59
        with self.socket(session["token"]) as second:
            second.send_json({"type": "attach", "token": session["token"]})
            receive_kind(second, "hello")
            state = receive_kind(second, "state")
            self.assertEqual(state["desired_altitude"], 1.2)
            self.assertTrue(state["paused"])
            clock.now += 600
            self.assertEqual(receive_kind(second, "session_error")["code"], "session_expired")
            with self.assertRaises(WebSocketDisconnect) as caught:
                second.receive_json()
            self.assertEqual(caught.exception.code, 4001)
        auth = {"authorization": f"Bearer {session['token']}"}
        for path in ("/api/log.npz", "/api/camera/recording"):
            response = self.client.get(path, headers=auth)
            self.assertEqual(response.status_code, 401)
            self.assertEqual(response.json()["error"], "session_expired")
        self.registry.cleanup()

    def test_attach_deadline_and_http_export_limit(self):
        with self.socket(None) as socket:
            self.assertEqual(receive_kind(socket, "session_error")["code"], "invalid_attach")
            with self.assertRaises(WebSocketDisconnect) as caught:
                socket.receive_json()
            self.assertEqual(caught.exception.code, 1008)
        session = self.new_session()
        self.registry._sessions[session["token"]].runtime.max_export_bytes = 10
        response = self.client.get("/api/log.npz", headers={"authorization": f"Bearer {session['token']}"})
        self.assertEqual(response.status_code, 413)
        self.assertEqual(response.json()["error"], "export_limit")


class DeploymentFactoryTests(unittest.TestCase):
    def test_local_bootstrap_and_explicit_public_environment(self):
        with TestClient(create_app(), base_url="http://127.0.0.1") as client:
            response = client.post("/api/session")
            self.assertEqual(response.status_code, 200)
            self.assertEqual(response.json()["mode"], "shared")
            self.assertIsNone(response.json()["token"])
        with patch.dict("os.environ", {"BLIMP_PUBLIC": "1", "PUBLIC_ORIGIN": ORIGIN}, clear=True):
            app = create_deployment_app()
            self.assertEqual(app.state.public_origin, ORIGIN)
            self.assertEqual(app.state.sessions.max_sessions, 2)
        for invalid in ("http://example.com", "https://example.com/path", "https://user:pass@example.com", "https://*.example.com", "https://localhost"):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                create_public_app(invalid)


if __name__ == "__main__":
    unittest.main()
