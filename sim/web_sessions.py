"""Bounded in-memory public visitor sessions, owned by one server process.

Tokens are bearer capabilities: never include them in URLs or logs. This registry
is deliberately process-local; deploy exactly one web worker.
"""
from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import secrets
import threading
import time

from .web_runtime import SimulationRuntime


class SessionExpired(ValueError):
    pass


class SessionCapacity(RuntimeError):
    pass


@dataclass
class VisitorSession:
    token: str
    runtime: SimulationRuntime
    created: float
    last_activity: float
    disconnected_at: float | None
    expires_at: str
    lease: object | None = None
    exporting: bool = False


class SessionRegistry:
    def __init__(self, max_sessions=2, disconnected_grace=60, idle_seconds=600, hard_seconds=1800,
                 max_log_steps=12000, max_export_bytes=32 * 1024 * 1024, clock=time.monotonic,
                 runtime_factory=None):
        if type(max_sessions) is not int or not 1 <= max_sessions <= 8:
            raise ValueError("Public session limit must be between 1 and 8")
        if min(disconnected_grace, idle_seconds, hard_seconds) <= 0:
            raise ValueError("Session lifetimes must be positive")
        self.max_sessions = max_sessions
        self.disconnected_grace, self.idle_seconds, self.hard_seconds = disconnected_grace, idle_seconds, hard_seconds
        self.max_log_steps, self.max_export_bytes = max_log_steps, max_export_bytes
        self._clock = clock
        self._runtime_factory = runtime_factory or (lambda: SimulationRuntime(max_log_steps=max_log_steps, max_export_bytes=max_export_bytes))
        self._sessions = {}
        self._lock = threading.Lock()

    def limits(self):
        return {"max_log_steps": self.max_log_steps, "max_recording_seconds": 30,
                "idle_seconds": self.idle_seconds, "disconnected_grace_seconds": self.disconnected_grace,
                "hard_seconds": self.hard_seconds}

    def _expired(self, session, now):
        return (now - session.created >= self.hard_seconds or now - session.last_activity >= self.idle_seconds or
                (session.disconnected_at is not None and now - session.disconnected_at >= self.disconnected_grace))

    def _get(self, token):
        # Call only under lock; fixed-length opaque tokens bound lookup input.
        if not isinstance(token, str) or len(token) != 43:
            raise SessionExpired("This flight expired. Start a new flight.")
        session = self._sessions.get(token)
        if session is None or self._expired(session, self._clock()):
            raise SessionExpired("This flight expired. Start a new flight.")
        return session

    def create(self):
        self.cleanup()
        with self._lock:
            if len(self._sessions) >= self.max_sessions:
                raise SessionCapacity("All flight slots are busy. Please try again shortly.")
            now = self._clock()
            token = secrets.token_urlsafe(32)
            runtime = self._runtime_factory()
            expires = datetime.fromtimestamp(time.time() + self.hard_seconds, timezone.utc).isoformat().replace("+00:00", "Z")
            session = VisitorSession(token, runtime, now, now, now, expires)
            runtime.start()
            self._sessions[token] = session
            return session

    def claim(self, token):
        with self._lock:
            session = self._get(token)
            session.lease = object()
            session.disconnected_at = None
            session.last_activity = self._clock()
            return session, session.lease

    def ownership(self, token, lease):
        with self._lock:
            session = self._get(token)
            return session.lease is lease

    def submit(self, token, lease, message):
        with self._lock:
            session = self._get(token)
            if session.lease is not lease:
                raise SessionExpired("This connection was replaced by another connection.")
            future = session.runtime.submit(message)
            # Empty key heartbeats and outgoing telemetry do not extend idle time.
            if message.get("action") != "keys" or message.get("keys"):
                session.last_activity = self._clock()
            return future

    def release(self, token, lease):
        with self._lock:
            session = self._sessions.get(token)
            if session is None or session.lease is not lease:
                return None
            session.lease = None
            session.disconnected_at = self._clock()
            # Enqueue before releasing the ownership lock: a newly claimed
            # socket's commands then follow this disconnect in worker order.
            return session.runtime.disconnect()

    def begin_export(self, token):
        with self._lock:
            session = self._get(token)
            if session.exporting:
                raise RuntimeError("A download is already being prepared for this flight")
            session.exporting = True
            session.last_activity = self._clock()
            return session

    def end_export(self, session):
        with self._lock:
            session.exporting = False

    def cleanup(self):
        with self._lock:
            now = self._clock()
            expired = [key for key, value in self._sessions.items() if self._expired(value, now)]
            sessions = [self._sessions.pop(key) for key in expired]
        for session in sessions:
            session.runtime.stop()
        return len(sessions)

    def close(self):
        with self._lock:
            sessions = list(self._sessions.values())
            self._sessions.clear()
        for session in sessions:
            session.runtime.stop()
