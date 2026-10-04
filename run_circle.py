"""Scenario 1: rovers run a circle; the blimp flies to the circle centre and holds 1 m altitude.

GUI (live sliders):      python run_circle.py --gui
Headless video + plots:  python run_circle.py --mp4 results/circle.mp4 --plots results/circle.png --npz results/circle.npz
"""
import argparse
import os

import numpy as np

from sim.params import SimParams
from sim.sim import TeamSim


def summary_plots(L, path, P):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    t = L["t"]; eta = L["blimp_eta"]; tk = P.task
    fig, axs = plt.subplots(2, 3, figsize=(14, 7))
    ax = axs[0, 0]
    ax.plot(t, eta[:, 0], label="x"); ax.plot(t, eta[:, 1], label="y"); ax.plot(t, -eta[:, 2], label="altitude")
    ax.axhline(tk.circle_center[0], color="k", ls=":", lw=0.8); ax.axhline(tk.blimp_height, color="k", ls=":", lw=0.8)
    ax.set_title("blimp position (m)"); ax.set_xlabel("t (s)"); ax.legend(frameon=False)
    ax = axs[0, 1]
    ax.plot(t, np.degrees(eta[:, 3]), label="roll"); ax.plot(t, np.degrees(eta[:, 4]), label="pitch"); ax.plot(t, np.degrees(eta[:, 5]), label="yaw")
    ax.set_title("blimp attitude (deg)"); ax.set_xlabel("t (s)"); ax.legend(frameon=False)
    ax = axs[0, 2]
    for i in range(6):
        ax.plot(t, L["blimp_thrust"][:, i], lw=1, label=f"T{i}")
    ax.set_title("thruster forces (N)"); ax.set_xlabel("t (s)"); ax.legend(frameon=False, ncol=3, fontsize=8)
    ax = axs[1, 0]
    q = L["rover_q"]; ref = L["rover_ref"]
    for i in range(q.shape[1]):
        ax.plot(q[:, i, 0], q[:, i, 1], lw=1.2, label=f"rover {i}")
    ang = np.linspace(0, 2 * np.pi, 200)
    ax.plot(tk.circle_center[0] + tk.circle_radius * np.cos(ang), tk.circle_center[1] + tk.circle_radius * np.sin(ang), "k--", lw=0.8)
    ax.plot(eta[:, 0], eta[:, 1], color="#2a78d6", lw=1, label="blimp (ground track)")
    ax.set_aspect("equal"); ax.set_title("top view (m)"); ax.legend(frameon=False, fontsize=8)
    ax = axs[1, 1]
    d = P.rover.d_hand
    hand = q[:, :, :2] + d * np.stack([np.cos(q[:, :, 2]), np.sin(q[:, :, 2])], axis=2)
    err = np.linalg.norm(hand - ref, axis=2)
    for i in range(q.shape[1]):
        ax.plot(t, err[:, i], lw=1, label=f"rover {i}")
    ax.set_title("rover tracking error, hand point to reference (m)"); ax.set_xlabel("t (s)"); ax.legend(frameon=False, fontsize=8)
    ax = axs[1, 2]
    nu = L["blimp_nu"]
    ax.plot(t, nu[:, 0], label="u"); ax.plot(t, nu[:, 1], label="v"); ax.plot(t, nu[:, 2], label="w")
    ax.set_title("blimp body velocity (m/s)"); ax.set_xlabel("t (s)"); ax.legend(frameon=False)
    fig.suptitle("Scenario 1 — rovers on a circle, blimp to centre at 1 m")
    fig.tight_layout(); fig.savefig(path, dpi=150)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gui", action="store_true")
    ap.add_argument("--mp4", default=None)
    ap.add_argument("--plots", default=None)
    ap.add_argument("--npz", default=None)
    ap.add_argument("--T", type=float, default=40.0)
    ap.add_argument("--n", type=int, default=4)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--net-lift", type=float, default=0.0, help="B - W in N (negative = heavy blimp)")
    ap.add_argument("--T-max", type=float, default=None, help="override thrust per thruster (N)")
    a = ap.parse_args()

    P = SimParams(T_end=a.T, seed=a.seed)
    P.task.n_rovers = a.n
    P.blimp.net_lift_N = a.net_lift
    if a.T_max is not None:
        P.blimp.T_max = a.T_max
    sim = TeamSim(P)

    if a.gui:
        import matplotlib.pyplot as plt
        for k in list(plt.rcParams):                 # free H / space / R from matplotlib's own key bindings
            if k.startswith("keymap."):
                plt.rcParams[k] = []
        from sim.viewer import Viewer
        viewer = Viewer(sim, gui=True)
        viewer.fig.canvas.mpl_connect("key_press_event", viewer.on_key)
        print("[gui] PID button / H = PID on-off,  RESET button / R = reset,  PAUSE button / space = pause")
        viewer.run_gui()
        return
    if a.mp4:
        import matplotlib
        matplotlib.use("Agg")
        from sim.viewer import Viewer
        os.makedirs(os.path.dirname(a.mp4) or ".", exist_ok=True)
        L = Viewer(sim, gui=False).render_video(a.mp4, T=a.T, every=2)
    else:
        sim.reset(); L = sim.run(a.T)
    if a.plots:
        os.makedirs(os.path.dirname(a.plots) or ".", exist_ok=True)
        summary_plots(L, a.plots, P)
    if a.npz:
        os.makedirs(os.path.dirname(a.npz) or ".", exist_ok=True)
        sim.save_npz(a.npz)
    eta = L["blimp_eta"]
    print(f"final blimp pos {eta[-1,:2].round(3)} alt {-eta[-1,2]:.3f} m, max |pitch| {np.degrees(np.abs(eta[:,4]).max()):.1f} deg")


if __name__ == "__main__":
    main()
