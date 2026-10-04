"""6-DoF lighter-than-air blimp in Fossen form, parameters from the GT-MAB paper.

    M nu_dot + C(nu) nu + D(nu) nu + g(eta) = tau_thrusters + tau_disturbance
    eta_dot = J(eta) nu

eta = [x, y, z, phi, theta, psi]  world pose (NED, z down), Euler ZYX
nu  = [u, v, w, p, q, r]          body velocity at the centre of volume (CV)
The centre of mass sits d_VM below CV; buoyancy acts at CV, weight at CM, neutral trim.
With no thrust and no added mass this reduces exactly to the paper's pendulum
    I_CM theta_dd = -b theta_d - m g d_VM sin(theta)
(test: tests/test_blimp.py).
"""
from __future__ import annotations

import numpy as np

from .params import BlimpParams, G, skew


# ---------------------------------------------------------------- kinematics
def R_zyx(phi, theta, psi):
    cph, sph, cth, sth, cps, sps = np.cos(phi), np.sin(phi), np.cos(theta), np.sin(theta), np.cos(psi), np.sin(psi)
    return np.array([
        [cps * cth, -sps * cph + cps * sth * sph, sps * sph + cps * cph * sth],
        [sps * cth, cps * cph + sph * sth * sps, -cps * sph + sth * sps * cph],
        [-sth, cth * sph, cth * cph],
    ])


def T_euler(phi, theta):
    cph, sph, cth, tth = np.cos(phi), np.sin(phi), np.cos(theta), np.tan(theta)
    cth = cth if abs(cth) > 1e-6 else 1e-6
    return np.array([[1.0, sph * tth, cph * tth], [0.0, cph, -sph], [0.0, sph / cth, cph / cth]])


def coriolis(M: np.ndarray, nu: np.ndarray) -> np.ndarray:
    """Fossen (2011) eq. 3.46: C(nu) for any symmetric mass matrix M (rigid body or added)."""
    M11, M12, M21, M22 = M[:3, :3], M[:3, 3:], M[3:, :3], M[3:, 3:]
    nu1, nu2 = nu[:3], nu[3:]
    S1 = skew(M11 @ nu1 + M12 @ nu2)
    S2 = skew(M21 @ nu1 + M22 @ nu2)
    C = np.zeros((6, 6))
    C[:3, 3:] = -S1
    C[3:, :3] = -S1
    C[3:, 3:] = -S2
    return C


class Blimp:
    def __init__(self, p: BlimpParams):
        self.p = p
        self.eta = np.zeros(6)
        self.nu = np.zeros(6)
        self.thrust = np.zeros(6)          # realised thrust per thruster (N)
        self.thrust_cmd = np.zeros(6)
        self._draught = np.zeros(2)
        self.floor_alt, self.ceiling_alt = 0.05, 4.0     # set by TeamSim from SimParams
        self._lift_drift = 0.0            # accumulated random-walk part of B - W (N)
        self.rebuild()

    # parameters may change live (sliders) -> recompute cached matrices
    def rebuild(self):
        p = self.p
        self.M = p.M_RB() + p.M_A()
        self.M_inv = np.linalg.inv(self.M)
        self.pos, self.axes = p.thruster_geometry()
        # 6x6 wrench map: tau = B @ thrust
        self.B = np.zeros((6, 6))
        for i in range(6):
            self.B[:3, i] = self.axes[i]
            self.B[3:, i] = np.cross(self.pos[i], self.axes[i])
        # Controllable wrench components are [Fx, Fy, Fz, tau_z]; roll/pitch torques are induced by
        # the thrust plane sitting d_VT below CV and are NOT allocated (swing uncontrolled in v1).
        self.CTRL_IDX = [0, 1, 2, 5]
        self.B_red_pinv = np.linalg.pinv(self.B[self.CTRL_IDX, :])

    def reset(self, eta=None, nu=None):
        self.eta = np.zeros(6) if eta is None else np.asarray(eta, float).copy()
        self.nu = np.zeros(6) if nu is None else np.asarray(nu, float).copy()
        self.thrust[:] = 0.0
        self.thrust_cmd[:] = 0.0
        self._draught[:] = 0.0
        self._lift_drift = 0.0

    # ---------------------------------------------------------------- forces
    def restoring(self, eta):
        """g(eta) with CM at (0,0,d_VM) below CV and buoyancy B = W + lift at CV (Fossen eq. 4.6).

        W - B = -lift enters the force rows; the torque rows use z_g W (z_b = 0)."""
        p = self.p
        W = p.m * G
        WmB = -self.lift                 # property: slider value + drift, read every physics step
        phi, theta = eta[3], eta[4]
        cph, sph, cth, sth = np.cos(phi), np.sin(phi), np.cos(theta), np.sin(theta)
        g = np.zeros(6)
        g[0] = WmB * sth
        g[1] = -WmB * cth * sph
        g[2] = -WmB * cth * cph
        g[3] = p.d_VM * W * cth * sph
        g[4] = p.d_VM * W * sth
        return g

    def damping(self, nu):
        p = self.p
        D = np.array([p.d_lin, p.d_lin, p.d_lin, p.b_xy, p.b_xy, p.b_z]) * nu
        D[:3] += np.array([p.d_quad_xy, p.d_quad_xy, p.d_quad_z]) * np.abs(nu[:3]) * nu[:3]
        D[5] += p.d_quad_yaw * abs(nu[5]) * nu[5]
        return D

    def wrench(self, thrust):
        return self.B @ thrust

    def nu_dot(self, eta, nu, tau):
        C = coriolis(self.p.M_RB(), nu) + coriolis(self.p.M_A(), nu)
        rhs = tau - C @ nu - self.damping(nu) - self.restoring(eta)
        return self.M_inv @ rhs

    def eta_dot(self, eta, nu):
        R = R_zyx(eta[3], eta[4], eta[5])
        T = T_euler(eta[3], eta[4])
        return np.concatenate([R @ nu[:3], T @ nu[3:]])

    # ---------------------------------------------------------------- stepping
    def command(self, u6):
        """Normalised thruster commands in [-1, 1]^6 -> thrust targets."""
        u = np.clip(np.asarray(u6, float), -1.0, 1.0)
        self.thrust_cmd = self.p.T_max * self.p.batt_scale * u

    def allocate(self, wrench_des):
        """Desired body wrench -> normalised commands (pseudo-inverse, then clip)."""
        t = self.B_red_pinv @ np.asarray(wrench_des, float)[self.CTRL_IDX]
        return np.clip(t / (self.p.T_max * self.p.batt_scale), -1.0, 1.0)

    def step(self, dt, rng=None, extra_wrench=None):
        p = self.p
        # thruster lag
        alpha = dt / max(p.tau_p, 1e-6)
        self.thrust += np.clip(alpha, 0.0, 1.0) * (self.thrust_cmd - self.thrust)
        tau = self.wrench(self.thrust)
        if p.net_lift_drift > 0.0 and rng is not None:
            self._lift_drift += p.net_lift_drift * np.sqrt(dt) * rng.standard_normal()
        if p.draught_N > 0.0 and rng is not None:
            a = dt / p.draught_tau
            self._draught += -a * self._draught + np.sqrt(2 * a) * p.draught_N * rng.standard_normal(2)
            R = R_zyx(*self.eta[3:6])
            tau[:3] += R.T @ np.array([self._draught[0], self._draught[1], 0.0])
        if extra_wrench is not None:
            tau = tau + extra_wrench
        # RK4 on (eta, nu)
        x = np.concatenate([self.eta, self.nu])

        def f(xx):
            e, n = xx[:6], xx[6:]
            return np.concatenate([self.eta_dot(e, n), self.nu_dot(e, n, tau)])

        k1 = f(x); k2 = f(x + 0.5 * dt * k1); k3 = f(x + 0.5 * dt * k2); k4 = f(x + dt * k3)
        x = x + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)
        self.eta, self.nu = x[:6], x[6:]
        # floor / ceiling: inelastic stop on the vertical world axis (z down)
        alt = -self.eta[2]
        if alt < self.floor_alt or alt > self.ceiling_alt:
            self.eta[2] = -np.clip(alt, self.floor_alt, self.ceiling_alt)
            R = R_zyx(*self.eta[3:6]); v_w = R @ self.nu[:3]
            if (alt < self.floor_alt and v_w[2] > 0) or (alt > self.ceiling_alt and v_w[2] < 0):
                v_w[2] = 0.0; self.nu[:3] = R.T @ v_w
        return self.eta, self.nu

    # ---------------------------------------------------------------- helpers
    @property
    def lift(self):
        """B - W in N: the live parameter (slider) plus the accumulated drift."""
        return self.p.net_lift_N + self._lift_drift

    @property
    def altitude(self):
        return -self.eta[2]

    def cm_position_world(self):
        R = R_zyx(*self.eta[3:6])
        return self.eta[:3] + R @ np.array([0.0, 0.0, self.p.d_VM])
