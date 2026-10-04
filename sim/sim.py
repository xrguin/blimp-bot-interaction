"""TeamSim: N rovers + blimp, 100 Hz physics / 20 Hz control, npz logging.

Gym-like: reset() -> obs, step() -> obs. Controllers are plugged in via `set_controllers`
so later learned planners can replace them without touching the plant.
"""
from __future__ import annotations

import numpy as np

from .blimp import Blimp
from .controllers import BlimpPD, CircleTracker
from .params import SimParams
from .rover import Rover


class TeamSim:
    def __init__(self, P: SimParams | None = None):
        self.P = P or SimParams()
        self.blimp = Blimp(self.P.blimp)
        self.blimp.floor_alt, self.blimp.ceiling_alt = self.P.floor_alt, self.P.ceiling_alt
        self.rovers = [Rover(self.P.rover) for _ in range(self.P.task.n_rovers)]
        self.rng = np.random.default_rng(self.P.seed)
        self.t = 0.0
        self.k = 0
        self.decim = max(1, int(round(self.P.dt_ctrl / self.P.dt_phys)))
        self.set_controllers(CircleTracker(self.P.task, len(self.rovers)), BlimpPD(self.P.task, dt=self.P.dt_ctrl))
        self.log = {"t": [], "blimp_eta": [], "blimp_nu": [], "blimp_u": [], "blimp_thrust": [], "blimp_lift": [],
                    "rover_q": [], "rover_u": [], "rover_ref": []}

    def set_controllers(self, rover_ctrl, blimp_ctrl):
        self.rover_ctrl, self.blimp_ctrl = rover_ctrl, blimp_ctrl

    def set_pid(self, enabled: bool):
        """Live PID on/off (GUI PID button, H key). Controllers may capture a hold reference here."""
        self.P.task.pid_enabled = bool(enabled)
        if hasattr(self.blimp_ctrl, "on_pid_toggle"):
            self.blimp_ctrl.on_pid_toggle(bool(enabled))

    # ------------------------------------------------------------------ API
    def reset(self, blimp_eta0=None):
        tk = self.P.task
        self.t, self.k = 0.0, 0
        self.rng = np.random.default_rng(self.P.seed)
        self.blimp.rebuild()
        for c in (self.rover_ctrl, self.blimp_ctrl):
            if hasattr(c, "reset"):
                c.reset()
        if blimp_eta0 is None:
            blimp_eta0 = np.array([tk.circle_center[0] + 1.0, tk.circle_center[1] - 1.0, -0.6, 0.0, 0.0, 0.0])
        self.blimp.reset(eta=blimp_eta0)
        for i, r in enumerate(self.rovers):
            pos, vel = self.rover_ctrl.reference(i, 0.0)
            heading = np.arctan2(vel[1], vel[0])
            # start slightly off the circle so the tracker has something to do
            r.reset(np.array([pos[0] * 0.8, pos[1] * 0.8, heading]))
        for v in self.log.values():
            v.clear()
        return self.observe()

    def observe(self):
        return {"t": self.t, "blimp_eta": self.blimp.eta.copy(), "blimp_nu": self.blimp.nu.copy(),
                "rover_q": np.array([r.q for r in self.rovers])}

    def step(self):
        """One control step (dt_ctrl): controllers -> `decim` physics steps."""
        tk = self.P.task
        u_r = np.array([self.rover_ctrl.command(i, r, self.t) for i, r in enumerate(self.rovers)])
        u_b = self.blimp_ctrl.command(self.blimp)
        refs = np.array([self.rover_ctrl.reference(i, self.t)[0] for i in range(len(self.rovers))])
        self._log(u_b, u_r, refs)
        for _ in range(self.decim):
            self.blimp.step(self.P.dt_phys, rng=self.rng)
            for r in self.rovers:
                r.step(self.P.dt_phys)
            self.t += self.P.dt_phys
        self.k += 1
        return self.observe()

    def run(self, T=None, callback=None):
        T = self.P.T_end if T is None else T
        n = int(round(T / self.P.dt_ctrl))
        for _ in range(n):
            self.step()
            if callback is not None:
                callback(self)
        return self.logs()

    # ------------------------------------------------------------------ logging
    def _log(self, u_b, u_r, refs):
        L = self.log
        L["t"].append(self.t); L["blimp_eta"].append(self.blimp.eta.copy()); L["blimp_nu"].append(self.blimp.nu.copy())
        L["blimp_u"].append(np.asarray(u_b).copy()); L["blimp_thrust"].append(self.blimp.thrust.copy()); L["blimp_lift"].append(float(self.blimp.lift))
        L["rover_q"].append(np.array([r.q for r in self.rovers])); L["rover_u"].append(u_r.copy()); L["rover_ref"].append(refs)

    def logs(self):
        return {k: np.asarray(v) for k, v in self.log.items()}

    def save_npz(self, path):
        """Underwater-repo convention per agent: X, U, X_next, Eta (+ team arrays)."""
        L = self.logs()
        nu, eta, ub = L["blimp_nu"], L["blimp_eta"], L["blimp_u"]
        q, ur = L["rover_q"], L["rover_u"]
        np.savez(path, t=L["t"][:-1],
                 X=nu[:-1], U=ub[:-1], X_next=nu[1:], Eta=eta[:-1], Eta_next=eta[1:],
                 rover_X=q[:-1], rover_U=ur[:-1], rover_X_next=q[1:], rover_ref=L["rover_ref"][:-1],
                 blimp_thrust=L["blimp_thrust"][:-1], blimp_lift=L["blimp_lift"][:-1], dt=self.P.dt_ctrl, n_rovers=len(self.rovers), seed=self.P.seed)
