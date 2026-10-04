"""Verification against the GT-MAB paper and basic sanity.

Run:  python -m sim.tests.test_blimp   (from the repo root)
"""
import numpy as np

from ..blimp import Blimp, R_zyx
from ..params import BlimpParams, G, SimParams
from ..rover import Rover
from ..params import RoverParams


def paper_pendulum(theta0, T=10.0, dt=0.001, f=0.0, p=None):
    """Integrate paper Eq. (10)/(21): I_CM th_dd = -b th_d - m g d_VM sin th + (d_VT - d_VM) f."""
    p = p or BlimpParams()
    th, thd = theta0, 0.0
    out = []
    for k in range(int(T / dt)):
        out.append(th)
        def acc(th_, thd_):
            return (-p.b_xy * thd_ - p.m * G * p.d_VM * np.sin(th_) + (p.d_VT - p.d_VM) * f) / p.I_CM_xy
        # RK4
        k1 = thd, acc(th, thd)
        k2 = thd + 0.5 * dt * k1[1], acc(th + 0.5 * dt * k1[0], thd + 0.5 * dt * k1[1])
        k3 = thd + 0.5 * dt * k2[1], acc(th + 0.5 * dt * k2[0], thd + 0.5 * dt * k2[1])
        k4 = thd + dt * k3[1], acc(th + dt * k3[0], thd + dt * k3[1])
        th += dt / 6 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0])
        thd += dt / 6 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1])
    return np.array(out)


def nrmse_fit(y, yhat):
    return 100.0 * (1 - np.linalg.norm(y - yhat) / np.linalg.norm(y - y.mean()))


def test_free_swing_matches_paper():
    """No thrust, no added mass, no translational drag: 6-DoF free swing == paper pendulum."""
    p = BlimpParams(k_add_xy=0.0, k_add_z=0.0, k_add_rot=0.0, d_lin=0.0, d_quad_xy=0.0, d_quad_z=0.0)
    b = Blimp(p)
    th0 = np.deg2rad(15.0)
    b.reset(eta=[0, 0, -1.0, 0, th0, 0])
    dt, T = 0.001, 10.0
    th = []
    for k in range(int(T / dt)):
        th.append(b.eta[4]); b.step(dt)
    th = np.array(th)
    ref = paper_pendulum(th0, T, dt, p=p)
    fit = nrmse_fit(ref, th)
    # paper: omega_n = sqrt(20.43) = 4.52 rad/s -> period 1.39 s
    zc = np.where(np.diff(np.sign(th)) > 0)[0]
    period = np.mean(np.diff(zc)) * dt if len(zc) > 2 else np.nan
    print(f"[free swing] NRMSE fit vs paper pendulum: {fit:.2f} %   period {period:.3f} s (paper 1.390 s)")
    assert fit > 99.0, fit
    assert abs(period - 2 * np.pi / np.sqrt(20.4284)) < 0.01


def test_lever_arm():
    """Constant surge thrust from rest: q_dot(0) = (d_VT - d_VM) f / I_CM."""
    p = BlimpParams(k_add_xy=0.0, k_add_z=0.0, k_add_rot=0.0, tau_p=1e-9)
    b = Blimp(p)
    b.reset(eta=[0, 0, -1.0, 0, 0, 0])
    f = 0.05
    tau = b.wrench(np.array([f / 4 / np.sqrt(0.5)] * 4 + [0, 0]))   # four horizontal at 45 deg -> Fx = f
    assert abs(tau[0] - f) < 1e-9 and abs(tau[1]) < 1e-9 and abs(tau[5]) < 1e-9
    nd = b.nu_dot(b.eta, b.nu, tau)
    expected_qdot = (p.d_VT - p.d_VM) * f / p.I_CM_xy
    # about CV the rigid body sees the coupled (m, m*d) system; q_dot from the 6x6 solve must equal the CM result
    print(f"[lever arm] q_dot = {nd[4]:.4f} rad/s^2, expected {expected_qdot:.4f}; u_dot = {nd[0]:.4f} m/s^2 (f/m = {f/p.m:.4f})")
    assert abs(nd[4] - expected_qdot) < 1e-6
    # CM acceleration must equal f/m (pure translation of the CM)
    a_cm = nd[0] + p.d_VM * nd[4]        # a_cm,x = a_cv,x + z_g * q_dot
    assert abs(a_cm - f / p.m) < 1e-6, a_cm


def test_allocator_roundtrip():
    b = Blimp(BlimpParams())
    for w in (np.array([0.05, 0, 0, 0, 0, 0]), np.array([0, 0.03, 0, 0, 0, 0]), np.array([0, 0, -0.04, 0, 0, 0]), np.array([0, 0, 0, 0, 0, 0.002])):
        t = b.B_red_pinv @ w[b.CTRL_IDX]
        w2 = b.B @ t
        # only the commanded components are required to match; pitch/roll torques are the unavoidable coupling
        assert np.allclose(w2[[0, 1, 2, 5]], w[[0, 1, 2, 5]], atol=1e-9), (w, w2)
    print("[allocator] surge/sway/heave/yaw round-trip OK; induced pitch torque for 0.05 N surge =",
          np.round((b.B @ (b.B_red_pinv @ np.array([0.05, 0, 0, 0])))[4], 5), "N m")


def test_unicycle_exact():
    r = Rover(RoverParams(v_max=10, w_max=10))
    r.reset([0, 0, 0]); r.command(0.5, 1.0)
    T = 2 * np.pi / 1.0
    n = 600
    for _ in range(n):
        r.step(T / n)
    print(f"[unicycle] after one full turn: pos error {np.linalg.norm(r.q[:2]):.2e} m, heading error {abs(r.q[2]-2*np.pi):.2e} rad")
    assert np.linalg.norm(r.q[:2]) < 1e-9 and abs(r.q[2] - 2 * np.pi) < 1e-9


def test_rotation_orthonormal():
    R = R_zyx(0.3, -0.2, 1.1)
    assert np.allclose(R @ R.T, np.eye(3))


if __name__ == "__main__":
    test_rotation_orthonormal(); test_unicycle_exact(); test_allocator_roundtrip(); test_lever_arm(); test_free_swing_matches_paper()
    print("all tests passed")
