"""Fly the blimp with an adapted BlueROV teleop key map (see sim/keyboard.py).

GUI:
    python teleop_blimp.py                      # rovers idle
    python teleop_blimp.py --rovers circle      # rovers run their circle meanwhile
    python teleop_blimp.py --gain 0.6 --npz results/teleop.npz

Headless self-test of the key-sign conventions (no keyboard):
    python teleop_blimp.py --self-test

Keyboard focus must be on the matplotlib window. Hold-to-move: W/S surge, A/D sway (A = left),
Q/E heave (Q = up), F/R yaw (F = left, R = right), +/- gain, Space panic, P screenshot, Esc quit.
"""
import argparse
import os
import time

import numpy as np

from sim.params import SimParams
from sim.sim import TeamSim
from sim.keyboard import KeyboardBlimpController, AXIS_NAMES
from sim.controllers import BlimpPD


class IdleRovers:
    def __init__(self, task, n): self.task, self.n = task, n
    def reference(self, i, t):
        ang = 2 * np.pi * i / max(self.n, 1)
        return np.array([self.task.circle_radius * np.cos(ang), self.task.circle_radius * np.sin(ang)]), np.zeros(2)
    def command(self, i, rover, t):
        rover.command(0.0, 0.0); return np.zeros(2)


def self_test():
    """Scripted bursts instead of keys; checks each key moves the blimp the piloted way."""
    P = SimParams(); P.task.n_rovers = 1
    sim = TeamSim(P)
    kb = KeyboardBlimpController(sim.blimp, gain=0.6, key_timeout=0.0)
    sim.set_controllers(IdleRovers(P.task, 1), kb)
    results = []

    def burst(key, seconds=3.0):
        sim.reset(blimp_eta0=sim.blimp.pose_at_altitude(1.0))
        kb.pressed = {key}
        for _ in range(int(seconds / P.dt_ctrl)):
            sim.step()
        kb.pressed = set()
        return sim.blimp.eta.copy(), sim.blimp.nu.copy()

    e, n = burst("w"); results.append(("W -> surge forward (x > 0)", e[0] > 0.05, f"x={e[0]:+.3f} m, u={n[0]:+.3f}"))
    e, n = burst("s"); results.append(("S -> surge back (x < 0)", e[0] < -0.05, f"x={e[0]:+.3f} m"))
    e, n = burst("a"); results.append(("A -> left as piloted (y_world < 0 at yaw 0)", e[1] < -0.05, f"y={e[1]:+.3f} m"))
    e, n = burst("d"); results.append(("D -> right (y_world > 0)", e[1] > 0.05, f"y={e[1]:+.3f} m"))
    e, n = burst("q"); results.append(("Q -> up (clearance > 1 m)", sim.blimp.altitude > 1.05, f"clearance={sim.blimp.altitude:.3f} m"))
    e, n = burst("e"); results.append(("E -> down (clearance < 1 m)", sim.blimp.altitude < 0.95, f"clearance={sim.blimp.altitude:.3f} m"))
    e, n = burst("f"); results.append(("F -> yaw left (psi < 0)", e[5] < -0.05, f"psi={np.degrees(e[5]):+.1f} deg"))
    e, n = burst("r"); results.append(("R -> yaw right (psi > 0)", e[5] > 0.05, f"psi={np.degrees(e[5]):+.1f} deg"))
    # swing check: a surge burst must excite pitch (thrusters below CV)
    e, n = burst("w", 1.0); results.append(("W excites pitch (swing)", abs(e[4]) > np.deg2rad(0.5), f"pitch={np.degrees(e[4]):+.2f} deg after 1 s"))
    ok_all = True
    for name, ok, info in results:
        ok_all &= bool(ok); print(f"  [{'OK' if ok else 'FAIL'}] {name:46s} {info}")
    print("authority per axis [surge, sway, heave, yaw]:", np.round(kb.authority, 4), "(N, N, N, N m)")
    print("self-test", "PASSED" if ok_all else "FAILED")
    return ok_all


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rovers", choices=["idle", "circle"], default="idle")
    ap.add_argument("--gain", type=float, default=0.4)
    ap.add_argument("--n", type=int, default=4)
    ap.add_argument("--npz", default=None)
    ap.add_argument("--self-test", action="store_true")
    ap.add_argument("--key-timeout", type=float, default=0.75)
    ap.add_argument("--hold", action="store_true", help="start with the PID hold assist ON (toggle with H or the PID button)")
    ap.add_argument("--heave-keys", choices=["qe", "eq"], default="qe",
                    help="qe: Q up / E down (default); eq: E up / Q down (as in EDMDc/teleop_tank.py)")
    a = ap.parse_args()
    if a.self_test:
        raise SystemExit(0 if self_test() else 1)

    import matplotlib
    import matplotlib.pyplot as plt
    for k in list(plt.rcParams):                     # free every key for the teleop
        if k.startswith("keymap."):
            plt.rcParams[k] = []
    from sim.viewer import Viewer

    P = SimParams(T_end=1e9); P.task.n_rovers = a.n
    P.task.pid_enabled = bool(a.hold)
    sim = TeamSim(P)
    hold = BlimpPD(P.task, dt=P.dt_ctrl)
    kb = KeyboardBlimpController(sim.blimp, gain=a.gain, key_timeout=a.key_timeout, heave_keys=a.heave_keys, hold=hold)
    rover_ctrl = sim.rover_ctrl if a.rovers == "circle" else IdleRovers(P.task, a.n)
    sim.set_controllers(rover_ctrl, kb)
    viewer = Viewer(sim, gui=True)
    fig = viewer.fig
    # Default reset starts at the origin with the gondola bottom on the ground.
    viewer.on_reset.append(kb.reset)
    up_key, down_key = ("Q", "E") if a.heave_keys == "qe" else ("E", "Q")

    def teleop_lines():
        c = kb.cmd4
        return [
            f"TELEOP   Surge {c[0]:+.2f}   Sway {c[1]:+.2f}   Heave {c[2]:+.2f}   Yaw {c[3]:+.2f}"
            f"     Gain {kb.gain:.1f} (+/−)     Keys [{' '.join(sorted(kb.pressed)) or '-'}]",
            f"W/S surge   A/D sway   {up_key}/{down_key} up/down   F/R yaw left/right     H hold   Space panic   P screenshot   Esc quit",
        ]
    viewer.extra_lines = teleop_lines           # shown in the telemetry block, not over the 3D axes

    def on_press(ev):
        if viewer.text_input_active:
            kb.pressed.clear()
            return
        if (ev.key or "").lower() == "h":
            viewer.set_pid(not P.task.pid_enabled); return
        kb.on_press(ev.key)
        if not kb.running:
            plt.close(fig)

    def on_release(ev):
        kb.on_release(ev.key)

    fig.canvas.mpl_connect("key_press_event", on_press)
    fig.canvas.mpl_connect("key_release_event", on_release)

    # sliders may change T_max -> authority must follow
    for s in viewer.sliders:
        s.on_changed(lambda _v: kb.rebuild())

    orig_frame = viewer._frame

    def frame(i):
        if viewer.text_input_active:
            kb.pressed.clear()
        viewer.hud.cmd4 = kb.cmd4
        arts = orig_frame(i)
        if kb.capture_requested:
            kb.capture_requested = False
            os.makedirs("results", exist_ok=True)
            path = time.strftime("results/teleop_%Y%m%d_%H%M%S.png")
            fig.savefig(path, dpi=120); print("[teleop] screenshot ->", path)
        return arts

    viewer._frame = frame
    print(f"[teleop] ready — WASD/{up_key}{down_key}, F/R yaw left/right, gain {kb.gain:.1f} (+/- to adjust), H or the green/grey "
          f"button = PID hold on/off, Space panic, P screenshot, Esc quit.")
    viewer.run_gui()                            # resets to reset_eta0; --hold already set task.pid_enabled
    if a.npz:
        os.makedirs(os.path.dirname(a.npz) or ".", exist_ok=True)
        sim.save_npz(a.npz); print("[teleop] log ->", a.npz)


if __name__ == "__main__":
    main()
