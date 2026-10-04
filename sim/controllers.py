"""v1 controllers for data generation.

Rovers: track equally spaced points on a circle through the hand point (feedback linearization).
Blimp: PD on NED position (circle centre, fixed altitude) and yaw, mapped through the
6-thruster allocator. Pitch/roll (swing) are left uncontrolled, as on the vehicle.
"""
from __future__ import annotations

import numpy as np

from .blimp import Blimp, R_zyx
from .params import TaskParams
from .rover import Rover


class CircleTracker:
    def __init__(self, task: TaskParams, n: int):
        self.task = task
        self.n = n
        self.phase0 = 2 * np.pi * np.arange(n) / n

    def reference(self, i, t):
        tk = self.task
        c = np.asarray(tk.circle_center, float)
        R, s = tk.circle_radius, tk.rover_speed
        ang = self.phase0[i] + (s / max(R, 1e-6)) * t
        pos = c + R * np.array([np.cos(ang), np.sin(ang)])
        vel = s * np.array([-np.sin(ang), np.cos(ang)])
        return pos, vel

    def command(self, i, rover: Rover, t):
        h_ref, hd_ref = self.reference(i, t)
        h = rover.hand()
        h_dot = hd_ref + self.task.k_hand * (h_ref - h)
        v, w = rover.hand_velocity_to_u(h_dot)
        rover.command(v, w)
        return np.array([v, w])


class BlimpPD:
    """PID on the CENTRE OF MASS, not the centre of volume (integral term optional, ki_pos = 0 -> PD).

    The CV swings with the pendulum (horizontal excursion d_VM sin(theta)); feeding CV velocity
    back through thrusters mounted below CV pumps the swing (observed in v0: pitch grew from 3 to
    7 deg in 40 s). The CM is the pendulum pivot and does not swing, so controlling it leaves the
    swing to decay on its own damping b, as on the real vehicle without a swing controller.
    """
    def __init__(self, task: TaskParams, use_cm: bool = True, dt: float = 0.05):
        self.task = task
        self.use_cm = use_cm
        self.dt = dt
        self.e_int = np.zeros(3)
        self.p_ref_override = None      # NED position of the CONTROLLED point (CM if use_cm) or None -> task reference
        self.yaw_ref_override = None
        self.last_wrench = np.zeros(6)

    def reset(self):
        self.e_int[:] = 0.0

    def on_pid_toggle(self, enabled: bool):
        self.e_int[:] = 0.0

    def controlled_point(self, blimp: Blimp):
        """Position and NED velocity of the controlled point (CM by default)."""
        eta, nu = blimp.eta, blimp.nu
        R = R_zyx(*eta[3:6])
        r_g = np.array([0.0, 0.0, blimp.p.d_VM]) if self.use_cm else np.zeros(3)
        return eta[:3] + R @ r_g, R @ (nu[:3] + np.cross(nu[3:], r_g)), R

    def reference(self, blimp: Blimp):
        tk = self.task
        if self.p_ref_override is not None:
            p_ref = np.asarray(self.p_ref_override, float)
        else:
            zoff = blimp.p.d_VM if self.use_cm else 0.0
            p_ref = np.array([tk.circle_center[0], tk.circle_center[1], -tk.blimp_height + zoff])
        yaw_ref = tk.blimp_yaw_ref if self.yaw_ref_override is None else self.yaw_ref_override
        return p_ref, yaw_ref

    def wrench(self, blimp: Blimp) -> np.ndarray:
        """PID body wrench [Fx, Fy, Fz, 0, 0, Tz] without commanding the thrusters."""
        tk = self.task
        p, p_dot, R = self.controlled_point(blimp)
        p_ref, yaw_ref = self.reference(blimp)
        e = p_ref - p
        self.e_int += e * self.dt
        if tk.ki_pos > 0:                                   # anti-windup: clamp the integral force
            lim = tk.i_max_N / tk.ki_pos
            self.e_int = np.clip(self.e_int, -lim, lim)
        F_ned = tk.kp_pos * e - tk.kd_pos * p_dot + tk.ki_pos * self.e_int
        F_body = R.T @ F_ned
        yaw = blimp.eta[5]
        yaw_err = np.arctan2(np.sin(yaw_ref - yaw), np.cos(yaw_ref - yaw))
        tau_z = tk.kp_yaw * yaw_err - tk.kd_yaw * blimp.nu[5]
        self.last_wrench = np.array([F_body[0], F_body[1], F_body[2], 0.0, 0.0, tau_z])
        return self.last_wrench

    def command(self, blimp: Blimp):
        if not self.task.pid_enabled:                       # PID off: thrusters idle, integrator cleared
            self.e_int[:] = 0.0
            blimp.command(np.zeros(6))
            return np.zeros(6)
        u = blimp.allocate(self.wrench(blimp))
        blimp.command(u)
        return u
