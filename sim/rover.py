"""Differential-drive ground robot as an exact unicycle (perfect-knowledge v1).

State  q = [x, y, theta]  on the floor plane (NED x north, y east; theta from x toward y).
Input  u = [v, omega]     forward speed and yaw rate, achieved instantly (onboard loop perfect).
The hand point h = p + d (cos theta, sin theta) is holonomic: any desired hand velocity maps to a
unique (v, omega) (feedback linearization), so planners may treat rovers as particles at h.
"""
from __future__ import annotations

import numpy as np

from .params import RoverParams


class Rover:
    def __init__(self, p: RoverParams, q0=(0.0, 0.0, 0.0)):
        self.p = p
        self.q = np.asarray(q0, float).copy()
        self.u = np.zeros(2)

    def reset(self, q0):
        self.q = np.asarray(q0, float).copy()
        self.u[:] = 0.0

    def command(self, v, w):
        self.u = np.array([np.clip(v, -self.p.v_max, self.p.v_max), np.clip(w, -self.p.w_max, self.p.w_max)])

    def step(self, dt):
        x, y, th = self.q
        v, w = self.u
        if abs(w) < 1e-9:
            x += v * np.cos(th) * dt
            y += v * np.sin(th) * dt
        else:  # exact arc
            x += (v / w) * (np.sin(th + w * dt) - np.sin(th))
            y -= (v / w) * (np.cos(th + w * dt) - np.cos(th))
            th += w * dt
        self.q = np.array([x, y, th])
        return self.q

    # ------------------------------------------------------------ hand point
    def hand(self):
        x, y, th = self.q
        d = self.p.d_hand
        return np.array([x + d * np.cos(th), y + d * np.sin(th)])

    def hand_velocity_to_u(self, h_dot):
        """Invert  h_dot = [[cos, -d sin], [sin, d cos]] [v, w]."""
        th = self.q[2]
        d = self.p.d_hand
        v = np.cos(th) * h_dot[0] + np.sin(th) * h_dot[1]
        w = (-np.sin(th) * h_dot[0] + np.cos(th) * h_dot[1]) / d
        return v, w
