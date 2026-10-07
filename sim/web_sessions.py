"""Per-visitor simulation sessions for the web server.

Each visitor (identified by an HttpOnly cookie) gets a private `SimulationRuntime` with its own
worker thread, seed, rovers and cameras. The manager caps the number of concurrent sessions,
stops sessions that have had no open connection for `idle_timeout` seconds, and can also run in
*shared* mode where every visitor sees one runtime (the original single-world behaviour).
"""
from __future__ import annotations

import secrets
import threading
import time


class SessionLimitError(RuntimeError):
    """Raised when a new session is requested but the server is at its session cap."""


class Session:
    __slots__ = ("id", "runtime", "options", "created", "last_seen", "connections", "started")

    def __init__(self, session_id, runtime, options, now, started=False):
        self.id = session_id
        self.runtime = runtime
        self.options = dict(options)
        self.created = now
        self.last_seen = now
        self.connections = 0            # open WebSockets; a session with connections is never reaped
        self.started = started          # simulation thread running; only started sessions count toward the cap


def parse_session_options(query, max_rovers=8):
    """Visitor options from the page's query string; unknown keys are ignored, bad values rejected."""
    options = {}
    if "n" in query:
        try:
            n = int(query["n"])
        except (TypeError, ValueError):
            raise ValueError("n must be an integer") from None
        if not 1 <= n <= max_rovers:
            raise ValueError(f"n must be between 1 and {max_rovers}")
        options["n"] = n
    if "seed" in query:
        try:
            seed = int(query["seed"])
        except (TypeError, ValueError):
            raise ValueError("seed must be an integer") from None
        if not 0 <= seed < 2**32:
            raise ValueError("seed must be between 0 and 4294967295")
        options["seed"] = seed
    if "mode" in query:
        if query["mode"] not in ("teleop", "auto"):
            raise ValueError("mode must be teleop or auto")
        options["mode"] = query["mode"]
    if "rovers" in query:
        if query["rovers"] not in ("circle", "idle"):
            raise ValueError("rovers must be circle or idle")
        options["rovers_mode"] = query["rovers"]
    return options


class SessionManager:
    COOKIE = "blimp_session"
    REAP_INTERVAL = 15.0

    def __init__(self, factory=None, *, shared_runtime=None, max_sessions=4, idle_timeout=180.0, unconnected_timeout=60.0,
                 clock=time.monotonic):
        """`factory(**options)` builds a (not yet started) SimulationRuntime for a new visitor.

        A page load only *reserves* a session (cookie, runtime object, no thread); the simulation
        starts when the page's WebSocket connects (`ensure_started`). Crawlers and scanners that
        fetch /api/config therefore cost memory for `unconnected_timeout` seconds, never a slot.
        With `shared_runtime`, every visitor is attached to that one runtime instead.
        """
        if (factory is None) == (shared_runtime is None):
            raise ValueError("give either a runtime factory or a shared runtime")
        self.factory = factory
        self.shared = shared_runtime is not None
        self.max_sessions = 1 if self.shared else int(max_sessions)
        self.max_reserved = 8 * self.max_sessions           # bound on reserved-but-idle sessions (memory)
        self.idle_timeout = float(idle_timeout)
        self.unconnected_timeout = float(unconnected_timeout)
        self.clock = clock
        self._sessions = {}
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._reaper = None
        if self.shared:
            self._shared = Session("shared", shared_runtime, {}, self.clock(), started=True)
            self._sessions[self._shared.id] = self._shared

    # ------------------------------------------------------------ lookup / creation
    def get(self, session_id):
        """Session for a cookie value (always the shared one in shared mode), touching it."""
        with self._lock:
            session = self._shared if self.shared else self._sessions.get(session_id)
            if session is not None:
                session.last_seen = self.clock()
            return session

    def create(self, options=None):
        """Reserve a session for a new visitor (runtime built, thread not started)."""
        if self.shared:
            return self.get(None)
        options = dict(options or {})
        with self._lock:
            self._reap_locked(self.clock())
            if len(self._sessions) >= self.max_reserved:
                raise SessionLimitError("The server is busy. Try again in a minute.")
            runtime = self.factory(**options)
            session = Session(secrets.token_urlsafe(24), runtime, options, self.clock())
            self._sessions[session.id] = session
        return session

    def running(self):
        with self._lock:
            return sum(1 for s in self._sessions.values() if s.started)

    def ensure_started(self, session):
        """Start the session's simulation on first real use; enforces the running-session cap."""
        with self._lock:
            if session.started:
                return
            self._reap_locked(self.clock())
            if sum(1 for s in self._sessions.values() if s.started) >= self.max_sessions:
                raise SessionLimitError(f"The server is full ({self.max_sessions} simulations running). Try again in a few minutes.")
            session.started = True
            session.last_seen = self.clock()
        session.runtime.start()

    def get_or_create(self, session_id, options=None):
        session = self.get(session_id)
        return (session, False) if session is not None else (self.create(options), True)

    def connect(self, session):
        with self._lock:
            session.connections += 1
            session.last_seen = self.clock()

    def disconnect(self, session):
        with self._lock:
            session.connections = max(0, session.connections - 1)
            session.last_seen = self.clock()

    def count(self):
        with self._lock:
            return len(self._sessions)

    def describe(self, session):
        return {"id": session.id[:8], "shared": self.shared, "count": self.running(), "max": self.max_sessions,
                "idle_timeout": self.idle_timeout, "started": session.started, "options": session.options}

    # ------------------------------------------------------------ lifecycle
    def _reap_locked(self, now):
        """Stop and drop sessions with no open connection for longer than idle_timeout (shared mode never reaps)."""
        if self.shared:
            return []
        expired = [s for s in self._sessions.values()
                   if s.connections == 0 and now - s.last_seen > (self.idle_timeout if s.started else self.unconnected_timeout)]
        for s in expired:
            del self._sessions[s.id]
        return expired

    def reap(self):
        with self._lock:
            expired = self._reap_locked(self.clock())
        for s in expired:
            if s.started:
                s.runtime.stop()
        return [s.id for s in expired]

    def _reaper_loop(self):
        while not self._stop.wait(self.REAP_INTERVAL):
            try:
                self.reap()
            except Exception:
                pass

    def start(self):
        if self.shared:
            self._shared.runtime.start()
            return
        if self._reaper is None or not self._reaper.is_alive():
            self._stop.clear()
            self._reaper = threading.Thread(target=self._reaper_loop, name="blimp-session-reaper", daemon=True)
            self._reaper.start()

    def stop(self):
        self._stop.set()
        if self._reaper is not None:
            self._reaper.join(timeout=5)
        with self._lock:
            sessions = list(self._sessions.values())
            if not self.shared:
                self._sessions.clear()
        for s in sessions:
            if s.started:
                s.runtime.stop()
