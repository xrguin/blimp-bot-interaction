"""Per-visitor sessions, caps, idle reaping, cookies, access token and public-host handling."""
from __future__ import annotations

import unittest

from fastapi.testclient import TestClient
from starlette.websockets import WebSocketDisconnect

from sim.params import SimParams
from sim.web_runtime import SimulationRuntime
from sim.web_sessions import SessionLimitError, SessionManager, parse_session_options
from web_server import WS_NO_SESSION, WS_UNAUTHORIZED, allowed_origin, create_app


class FakeRuntime:
    def __init__(self, **options):
        self.options, self.started, self.stopped = options, False, False

    def start(self):
        self.started = True

    def stop(self):
        self.stopped = True


class SessionManagerTests(unittest.TestCase):
    def test_options_parsing(self):
        self.assertEqual(parse_session_options({"n": "6", "seed": "3", "mode": "auto", "rovers": "idle", "junk": "x"}),
                         {"n": 6, "seed": 3, "mode": "auto", "rovers_mode": "idle"})
        for bad in ({"n": "0"}, {"n": "9"}, {"n": "two"}, {"seed": "-1"}, {"mode": "fly"}, {"rovers": "square"}):
            with self.assertRaises(ValueError):
                parse_session_options(bad, max_rovers=8)

    def test_create_cap_reap_and_shared(self):
        clock = [100.0]
        manager = SessionManager(factory=FakeRuntime, max_sessions=2, idle_timeout=60.0, clock=lambda: clock[0])
        a = manager.create({"n": 2})
        b = manager.create({})
        self.assertTrue(a.runtime.started and b.runtime.started)
        self.assertEqual(a.runtime.options, {"n": 2})
        self.assertIsNot(manager.get(a.id), manager.get(b.id))
        self.assertIsNone(manager.get("nope"))
        with self.assertRaises(SessionLimitError):
            manager.create({})
        manager.connect(a)                                   # a has an open connection, b is idle
        clock[0] += 61.0
        self.assertEqual(manager.reap(), [b.id])
        self.assertTrue(b.runtime.stopped and not a.runtime.stopped)
        self.assertEqual(manager.count(), 1)
        manager.disconnect(a)
        clock[0] += 30.0                                     # touched at disconnect: not idle yet
        self.assertEqual(manager.reap(), [])
        c = manager.create({})                               # room again after the reap
        session, created = manager.get_or_create(c.id, {})
        self.assertIs(session, c)
        self.assertFalse(created)
        info = manager.describe(c)
        self.assertEqual((info["shared"], info["count"], info["max"], len(info["id"])), (False, 2, 2, 8))
        manager.stop()
        self.assertTrue(a.runtime.stopped and c.runtime.stopped)
        shared = SessionManager(shared_runtime=FakeRuntime())
        self.assertIs(shared.get(None), shared.get("anything"))
        self.assertIs(shared.create({}), shared.get(None))
        self.assertEqual(shared.reap(), [])
        shared.start()
        self.assertTrue(shared.get(None).runtime.started)
        with self.assertRaises(ValueError):
            SessionManager()


class OriginTests(unittest.TestCase):
    def test_public_host_origins(self):
        class WS:
            def __init__(self, origin, scheme="ws", port=8000):
                self.headers = {"origin": origin} if origin else {}
                self.url = type("U", (), {"scheme": scheme, "port": port})()

        self.assertTrue(allowed_origin(WS(None)))
        self.assertTrue(allowed_origin(WS("http://127.0.0.1:8000")))
        self.assertFalse(allowed_origin(WS("http://127.0.0.1:9000")))
        self.assertFalse(allowed_origin(WS("https://blimp.example.org")))
        self.assertTrue(allowed_origin(WS("https://blimp.example.org"), {"blimp.example.org"}))
        self.assertFalse(allowed_origin(WS("http://blimp.example.org"), {"blimp.example.org"}))   # public hosts must be https
        self.assertFalse(allowed_origin(WS("https://evil.example.org"), {"blimp.example.org"}))


class PerVisitorAppTests(unittest.TestCase):
    """Real SimulationRuntimes (ideal rovers, one rover each) behind the per-visitor app."""

    @staticmethod
    def factory(**options):
        return SimulationRuntime(**{"n": 1, "seed": 0, "mode": "auto", **options})

    def test_sessions_are_private_cookies_and_cap(self):
        app = create_app(factory=self.factory, max_sessions=2, idle_timeout=600.0, max_rovers=3)
        # One client owns the app lifespan; further visitors are plain clients with their own cookie jars.
        with TestClient(app, base_url="http://127.0.0.1") as alice:
            bob = TestClient(app, base_url="http://127.0.0.1")
            self.assertEqual(alice.get("/api/camera/blimp").status_code, 404)        # no session yet
            config = alice.get("/api/config?n=2&seed=7").json()
            self.assertEqual((config["n_rovers"], config["seed"], config["session"]["shared"]), (2, 7, False))
            self.assertIn(SessionManager.COOKIE, alice.cookies)
            self.assertEqual(alice.get("/api/config").json()["session"]["id"], config["session"]["id"])   # resumed, not recreated
            self.assertEqual(alice.get("/health").json()["sessions"], 1)
            other = bob.get("/api/config").json()
            self.assertNotEqual(other["session"]["id"], config["session"]["id"])
            self.assertEqual(other["n_rovers"], 1)
            self.assertEqual(bob.get("/health").json()["sessions"], 2)
            self.assertEqual(alice.get("/api/config?n=9").status_code, 400)           # above --max-rovers
            carol = TestClient(app, base_url="http://127.0.0.1")
            full = carol.get("/api/config")
            self.assertEqual(full.status_code, 409)
            self.assertIn("full", full.json()["error"])
            # Each visitor controls their own simulation; a second tab of the same visitor spectates.
            with alice.websocket_connect("ws://127.0.0.1/ws") as ws1, alice.websocket_connect("ws://127.0.0.1/ws") as ws2, \
                    bob.websocket_connect("ws://127.0.0.1/ws") as ws3:
                hello1, hello2, hello3 = ws1.receive_json(), ws2.receive_json(), ws3.receive_json()
                self.assertEqual((hello1["has_control"], hello2["has_control"], hello3["has_control"]), (True, False, True))
                self.assertEqual(hello1["session"]["id"], config["session"]["id"])
                self.assertEqual(hello3["session"]["id"], other["session"]["id"])
                ws1.send_json({"type": "command", "id": "run", "action": "pause", "value": False})
                for _ in range(40):
                    state = ws1.receive_json()
                    if state.get("type") == "state" and state["t"] > 0.1:
                        break
                self.assertGreater(state["t"], 0.1)
                bob_state = None
                for _ in range(5):
                    bob_state = ws3.receive_json()
                    if bob_state.get("type") == "state":
                        break
                self.assertEqual(bob_state["t"], 0.0)                                   # Bob's world is untouched
                ws1.send_json({"type": "command", "id": "stop", "action": "pause", "value": True})
            self.assertEqual(alice.get("/api/log.npz").status_code, 200)
            self.assertEqual(bob.get("/api/camera/blimp").status_code, 404)           # ideal backend: no cameras

    def test_websocket_without_session_and_access_token(self):
        app = create_app(factory=self.factory, access_token="s3cret", public_hosts=["blimp.example.org"])
        with TestClient(app, base_url="http://127.0.0.1") as client:
            self.assertEqual(client.get("/api/config").status_code, 401)
            with self.assertRaises(WebSocketDisconnect) as ctx:
                with client.websocket_connect("ws://127.0.0.1/ws"):
                    pass
            self.assertEqual(ctx.exception.code, WS_UNAUTHORIZED)
            self.assertEqual(client.get("/?token=wrong", follow_redirects=False).status_code, 401)
            redirect = client.get("/?token=s3cret", follow_redirects=False)
            self.assertEqual((redirect.status_code, redirect.headers["location"]), (303, "/"))
            self.assertIn("blimp_token", client.cookies)
            with self.assertRaises(WebSocketDisconnect) as ctx:                     # authorised but no session yet
                with client.websocket_connect("ws://127.0.0.1/ws"):
                    pass
            self.assertEqual(ctx.exception.code, WS_NO_SESSION)
            self.assertEqual(client.get("/api/config").status_code, 200)
            with client.websocket_connect("ws://127.0.0.1/ws") as ws:
                self.assertTrue(ws.receive_json()["has_control"])
            # Public hostname accepted by the host middleware; unknown hosts rejected.
            self.assertEqual(client.get("/health", headers={"host": "blimp.example.org"}).status_code, 200)
            self.assertEqual(client.get("/health", headers={"host": "evil.example.org"}).status_code, 400)


if __name__ == "__main__":
    unittest.main()
