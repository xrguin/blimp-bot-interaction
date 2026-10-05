"""Effect of a non-neutral trim (net lift) on altitude hold: PD vs PID, and a drifting trim.

    python exp_net_lift.py --out results/net_lift.png
"""
import argparse
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from sim.params import SimParams
from sim.sim import TeamSim

C = dict(blue="#2a78d6", orange="#eb6834", aqua="#1baf7a", ink2="#52514e")


def run(net_lift, ki, drift=0.0, T=80.0, seed=0):
    P = SimParams(T_end=T, seed=seed)
    P.blimp.net_lift_N = net_lift
    P.blimp.net_lift_drift = drift
    P.task.ki_pos = ki
    sim = TeamSim(P)
    sim.reset()
    L = sim.run(T)
    return L["t"], L["blimp_bottom_altitude"], L["blimp_thrust"][:, 4] + L["blimp_thrust"][:, 5], L["blimp_lift"]


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--out", default="results/net_lift.png"); a = ap.parse_args()
    cases = [("neutral trim, PD", 0.0, 0.0, 0.0, C["ink2"]),
             ("+0.01 N lift, PD", 0.01, 0.0, 0.0, C["orange"]),
             ("+0.01 N lift, PID", 0.01, 0.004, 0.0, C["blue"]),
             ("drifting lift, PID", 0.0, 0.004, 0.002, C["aqua"])]
    fig, axs = plt.subplots(1, 3, figsize=(14, 4))
    for name, nl, ki, dr, col in cases:
        t, alt, Fz, lift = run(nl, ki, dr)
        axs[0].plot(t, alt, color=col, label=name)
        axs[1].plot(t, Fz, color=col, label=name)
        axs[2].plot(t, lift, color=col, label=name)
    axs[0].axhline(1.0, color="k", ls=":", lw=0.8); axs[0].set_ylabel("gondola clearance (m)")
    axs[1].set_ylabel("vertical thrust, sum of both props (N)")
    axs[2].set_ylabel("net lift B − W (N)")
    for ax in axs:
        ax.set_xlabel("t (s)"); ax.legend(frameon=False, fontsize=8)
    fig.suptitle("Altitude hold with non-neutral trim")
    fig.tight_layout(); fig.savefig(a.out, dpi=150)
    print("saved", a.out)


if __name__ == "__main__":
    main()
