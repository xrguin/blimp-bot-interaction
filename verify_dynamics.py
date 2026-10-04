"""Headless checks that the simulated blimp behaves the way the README claims.

    python verify_dynamics.py            # prints PASS/FAIL lines, writes results/verify_dynamics.png

Panels (all with the default parameters unless stated):
  a  free swing from 15 deg vs the paper's pendulum  I_CM th'' = -b th' - m g d_VM sin th
  b  thrusters off: altitude for B - W = +0.02, 0, -0.02 N (positive lift must rise)
  c  heavy blimp (B - W = -0.03 N): PID ON holds 1 m with upward thrust; OFF at 50 s -> sinks; ON at 70 s -> recovers
     (c2: a trim heavier than the heave authority 2 T_max = 0.06 N cannot be held at all)
  d  1 s surge burst excites the swing, which then decays with the paper's damping (envelope b / 2 I_CM)
  e  PD on the centre of volume pumps the swing; PID on the centre of mass does not
  f  teleop: W held for 10 s then released, hold assist ON vs OFF -> altitude
"""
import os
import numpy as np

from sim.params import SimParams, G
from sim.sim import TeamSim
from sim.controllers import BlimpPD
from sim.keyboard import KeyboardBlimpController


def pendulum_paper(P, th0, T, dt=0.01):
    """Paper pendulum (Eq. 19-21 of Tao et al. 2018) integrated with RK4 for reference."""
    p = P.blimp
    I, b, mgd = p.I_CM_xy, p.b_xy, p.m * G * p.d_VM

    def f(x):
        return np.array([x[1], (-b * x[1] - mgd * np.sin(x[0])) / I])
    x = np.array([th0, 0.0]); out = [th0]
    for _ in range(int(T / dt)):
        k1 = f(x); k2 = f(x + 0.5 * dt * k1); k3 = f(x + 0.5 * dt * k2); k4 = f(x + dt * k3)
        x = x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4); out.append(x[0])
    return np.arange(len(out)) * dt, np.array(out)


class Idle:
    def __init__(self, task, n): self.task, self.n = task, n
    def reference(self, i, t): return np.zeros(2), np.zeros(2)
    def command(self, i, rover, t): rover.command(0.0, 0.0); return np.zeros(2)


class Off:
    """Blimp controller that leaves the thrusters idle."""
    def command(self, blimp): blimp.command(np.zeros(6)); return np.zeros(6)
    def reset(self): pass


def run(sim, T, hook=None):
    t, eta, nu, Fup = [], [], [], []
    while sim.t < T - 1e-9:
        if hook: hook(sim)
        sim.step()
        t.append(sim.t); eta.append(sim.blimp.eta.copy()); nu.append(sim.blimp.nu.copy())
        Fup.append(-sim.blimp.wrench(sim.blimp.thrust)[2])      # body Fz is down; report "up" (small angles)
    return np.array(t), np.array(eta), np.array(nu), np.array(Fup)


def main():
    checks = []
    P0 = SimParams(); P0.task.n_rovers = 1
    # ---------------------------------------------------------------- a: free swing vs paper
    P = SimParams(); P.task.n_rovers = 1
    P.blimp.k_add_xy = P.blimp.k_add_z = P.blimp.k_add_rot = 0.0; P.blimp.d_lin = P.blimp.d_quad_xy = P.blimp.d_quad_z = 0.0
    sim = TeamSim(P); sim.set_controllers(Idle(P.task, 1), Off())
    sim.reset(blimp_eta0=np.array([0, 0, -1.0, 0, np.deg2rad(15), 0]))
    ta, ea, _, _ = run(sim, 10.0)
    tp, thp = pendulum_paper(P, np.deg2rad(15), 10.0)
    th_sim = np.interp(tp, ta, ea[:, 4])
    nrmse = 1 - np.linalg.norm(th_sim - thp) / np.linalg.norm(thp - thp.mean())
    checks.append(("a free swing matches paper pendulum (fit > 99 %)", nrmse > 0.99, f"fit {100 * nrmse:.2f} %"))
    # ---------------------------------------------------------------- b: lift sign, thrusters off
    alts = {}
    for lift in (+0.02, 0.0, -0.02):
        P = SimParams(); P.task.n_rovers = 1; P.blimp.net_lift_N = lift
        sim = TeamSim(P); sim.set_controllers(Idle(P.task, 1), Off())
        sim.reset(blimp_eta0=np.array([0, 0, -1.0, 0, 0, 0]))
        tb, eb, _, _ = run(sim, 8.0); alts[lift] = (tb, -eb[:, 2])
    checks.append(("b +0.02 N lift rises, -0.02 N sinks, 0 stays", alts[0.02][1][-1] > 1.2 and alts[-0.02][1][-1] < 0.8 and abs(alts[0.0][1][-1] - 1) < 1e-3,
                   f"final alt {alts[0.02][1][-1]:.2f} / {alts[0.0][1][-1]:.2f} / {alts[-0.02][1][-1]:.2f} m"))
    # ---------------------------------------------------------------- c: heavy blimp, PID on/off/on
    P = SimParams(); P.task.n_rovers = 1; P.blimp.net_lift_N = -0.03
    sim = TeamSim(P); sim.set_controllers(Idle(P.task, 1), BlimpPD(P.task, dt=P.dt_ctrl))
    sim.reset(blimp_eta0=np.array([0, 0, -1.0, 0, 0, 0]))
    state = {"off": False, "on": False}

    def hook_c(s):
        if s.t >= 50.0 and not state["off"]:
            s.set_pid(False); state["off"] = True
        if s.t >= 70.0 and not state["on"]:
            s.set_pid(True); state["on"] = True
    tc, ec, _, Fc = run(sim, 100.0, hook_c)
    alt_c = -ec[:, 2]
    i1 = np.searchsorted(tc, 49.0); i2 = np.searchsorted(tc, 69.0); i3 = len(tc) - 1
    on1 = tc < 50.0
    outside = np.where(on1 & (np.abs(alt_c - 1.0) > 0.02))[0]
    t_settle = tc[outside[-1]] if len(outside) else 0.0
    checks.append(("c heavy blimp: PID holds 1 m, thrust up = |B-W| (steady state)", abs(alt_c[i1] - 1) < 0.02 and abs(Fc[i1] - 0.03) < 0.003,
                   f"alt {alt_c[i1]:.3f} m, Fz(up) {Fc[i1]:.3f} N at 49 s; dip to {alt_c[on1].min():.2f} m, settled (2 cm) at {t_settle:.0f} s"))
    checks.append(("c PID OFF -> sinks to the floor; ON -> back to 1 m", alt_c[i2] < 0.1 and abs(alt_c[i3] - 1) < 0.05,
                   f"alt {alt_c[i2]:.2f} m at 69 s, {alt_c[i3]:.3f} m at 100 s"))
    # heave authority: two vertical thrusters -> 2 T_max = 0.06 N is the largest |B - W| the PID can hold
    P = SimParams(); P.task.n_rovers = 1; P.blimp.net_lift_N = -0.07
    sim = TeamSim(P); sim.set_controllers(Idle(P.task, 1), BlimpPD(P.task, dt=P.dt_ctrl))
    sim.reset(blimp_eta0=np.array([0, 0, -1.0, 0, 0, 0]))
    tg, eg, _, Fg = run(sim, 30.0)
    checks.append(("c2 |B-W| above 2 T_max cannot be held (thrust saturates)", -eg[-1, 2] < 0.1 and abs(Fg[-1] - 2 * P.blimp.T_max) < 1e-3,
                   f"B-W = -0.07 N: alt {-eg[-1, 2]:.2f} m, Fz(up) {Fg[-1]:.3f} N = 2 T_max {2 * P.blimp.T_max:.3f} N"))
    # ---------------------------------------------------------------- d: surge burst, swing decay
    P = SimParams(); P.task.n_rovers = 1
    sim = TeamSim(P); kb = KeyboardBlimpController(sim.blimp, gain=0.6, key_timeout=0.0)
    sim.set_controllers(Idle(P.task, 1), kb)
    sim.reset(blimp_eta0=np.array([0, 0, -1.0, 0, 0, 0]))

    def hook_d(s):
        kb.pressed = {"w"} if s.t < 1.0 else set()
    td, ed, _, _ = run(sim, 40.0, hook_d)
    pitch = np.degrees(ed[:, 4])
    sigma = P.blimp.b_xy / (2 * P.blimp.I_CM_xy)                                 # paper damping: envelope exp(-sigma t)
    # fit the decay rate from the peaks after the burst
    seg = td > 2.0; pk = [i for i in range(1, len(td) - 1) if seg[i] and pitch[i] > pitch[i - 1] and pitch[i] > pitch[i + 1] and pitch[i] > 0.2]
    rate = -np.polyfit(td[pk], np.log(pitch[pk]), 1)[0] if len(pk) > 3 else np.nan
    checks.append(("d swing decay rate matches b / 2 I_CM (within 25 %)", abs(rate - sigma) / sigma < 0.25, f"fitted {rate:.3f} 1/s vs paper {sigma:.3f} 1/s"))
    env_t = td[td > 1.0]; env = np.max(np.abs(pitch[td > 1.0])) * np.exp(-sigma * (env_t - env_t[0]))
    # ---------------------------------------------------------------- e: PD on CV vs PID on CM
    res_e = {}
    for use_cm, label in ((False, "PD on centre of volume"), (True, "PID on centre of mass")):
        P = SimParams(); P.task.n_rovers = 1
        if not use_cm:
            P.task.ki_pos = 0.0
        sim = TeamSim(P); sim.set_controllers(Idle(P.task, 1), BlimpPD(P.task, use_cm=use_cm, dt=P.dt_ctrl))
        sim.reset(blimp_eta0=np.array([1.0, -1.0, -0.6, 0, np.deg2rad(3), 0]))
        te, ee, _, _ = run(sim, 60.0); res_e[label] = (te, np.degrees(ee[:, 4]))
    amp = {k: (np.max(np.abs(v[1][(v[0] > 5) & (v[0] < 15)])), np.max(np.abs(v[1][v[0] > 50]))) for k, v in res_e.items()}
    checks.append(("e PD on CV grows the swing, PID on CM lets it decay", amp["PD on centre of volume"][1] > amp["PD on centre of volume"][0] and amp["PID on centre of mass"][1] < amp["PID on centre of mass"][0],
                   "early/late |pitch| " + ", ".join(f"{k}: {a:.1f}/{b:.1f} deg" for k, (a, b) in amp.items())))
    # ---------------------------------------------------------------- f: teleop hold assist
    res_f = {}
    for hold_on, label in ((True, "hold ON"), (False, "hold OFF")):
        P = SimParams(T_end=1e9); P.task.n_rovers = 1; P.task.pid_enabled = hold_on
        sim = TeamSim(P); hold = BlimpPD(P.task, dt=P.dt_ctrl)
        kb = KeyboardBlimpController(sim.blimp, gain=0.4, key_timeout=0.0, hold=hold)
        sim.set_controllers(Idle(P.task, 1), kb)
        sim.reset(blimp_eta0=np.array([0, 0, -1.0, 0, 0, 0]))

        def hook_f(s):
            kb.pressed = {"w"} if s.t < 10.0 else set()
        tf, ef, _, _ = run(sim, 25.0, hook_f); res_f[label] = (tf, -ef[:, 2], ef[:, 0])
    dev = {k: np.max(np.abs(v[1] - 1.0)) for k, v in res_f.items()}
    checks.append(("f hold ON keeps altitude within 0.15 m while driving", dev["hold ON"] < 0.15, f"max |alt - 1| ON {dev['hold ON']:.3f} m, OFF {dev['hold OFF']:.3f} m"))

    # ---------------------------------------------------------------- report
    ok_all = True
    for name, ok, info in checks:
        ok_all &= bool(ok); print(f"  [{'PASS' if ok else 'FAIL'}] {name:58s} {info}")
    print("verify_dynamics", "PASSED" if ok_all else "FAILED")

    # ---------------------------------------------------------------- figure
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(2, 3, figsize=(15, 8))
    ax = axs[0, 0]
    ax.plot(ta, np.degrees(ea[:, 4]), label="simulator"); ax.plot(tp, np.degrees(thp), "--", label="paper pendulum")
    ax.set_title("free swing"); ax.set_xlabel("t (s)"); ax.set_ylabel("pitch (deg)"); ax.legend(frameon=False)
    ax = axs[0, 1]
    for lift, (tb, alt) in alts.items():
        ax.plot(tb, alt, label=f"B − W = {lift:+.2f} N")
    ax.set_title("thrusters off"); ax.set_xlabel("t (s)"); ax.set_ylabel("altitude (m)"); ax.legend(frameon=False)
    ax = axs[0, 2]
    ax.plot(tc, alt_c, label="altitude (m)"); ax.plot(tc, Fc / 0.03, label="thrust up / |B − W|")
    ax.axvspan(50, 70, color="#e6e5e1", label="PID OFF")
    ax.set_title("heavy blimp, B − W = −0.03 N"); ax.set_xlabel("t (s)"); ax.legend(frameon=False)
    ax = axs[1, 0]
    ax.plot(td, pitch, label="pitch"); ax.plot(env_t, env, "--", label="exp(−b t / 2 I_CM)"); ax.plot(env_t, -env, "--", color="C1")
    ax.set_title("1 s surge burst"); ax.set_xlabel("t (s)"); ax.set_ylabel("pitch (deg)"); ax.legend(frameon=False)
    ax = axs[1, 1]
    for k, (te, pe) in res_e.items():
        ax.plot(te, pe, lw=0.8, label=k)
    ax.set_title("scenario 1 from 3 deg pitch"); ax.set_xlabel("t (s)"); ax.set_ylabel("pitch (deg)"); ax.legend(frameon=False)
    ax = axs[1, 2]
    for k, (tf, alt, x) in res_f.items():
        ax.plot(tf, alt, label=f"altitude, {k}")
    ax.axvspan(0, 10, color="#e6e5e1", label="W held")
    ax.set_title("teleop surge, gain 0.4"); ax.set_xlabel("t (s)"); ax.set_ylabel("altitude (m)"); ax.legend(frameon=False)
    fig.tight_layout()
    os.makedirs("results", exist_ok=True)
    fig.savefig("results/verify_dynamics.png", dpi=140)
    return ok_all


if __name__ == "__main__":
    raise SystemExit(0 if main() else 1)
