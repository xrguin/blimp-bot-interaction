"""Keyboard teleop for the blimp with the SAME key map as the BlueROV teleop
(underwater-manipulation-isaac-6/EDMDc/teleop_tank.py): hold-to-move, 4-axis command
[surge, sway, heave, yaw] in [-1, 1], scaled by a gain.

    W / S      surge forward / back
    A / D      sway left / right          (A = +sway = LEFT as piloted)
    Q / E      heave up / down            (Q = +heave = UP; --heave-keys eq swaps to E up / Q down)
    R / F      yaw left / right           (F = +yaw = RIGHT, i.e. r > 0)
    + / -      command gain up / down (0.1 .. 1.0)
    Space      panic: clear all held keys (zero command)
    P          screenshot of the viewer
    Esc        quit

The piloted meaning is preserved, not the body-axis sign: the ROV body frame is x forward,
y LEFT, z UP (Isaac), the blimp body frame is x forward, y RIGHT, z DOWN (paper/Fossen), so
+sway maps to body -y and +heave to body -z here.
"""
from __future__ import annotations

import time
import numpy as np

from .blimp import Blimp
from .controllers import BlimpPD

# (axis, key, sign) — surge/sway/yaw as teleop_tank.AXIS_KEYS; heave: Q = up, E = down (user's choice,
# 2026-10-04; teleop_tank.py itself has E up / Q down -> --heave-keys eq)
AXIS_KEYS = (
    (0, "w", +1.0), (0, "s", -1.0),     # surge
    (1, "a", +1.0), (1, "d", -1.0),     # sway  (A = left)
    (2, "q", +1.0), (2, "e", -1.0),     # heave (Q = up)
    (3, "f", +1.0), (3, "r", -1.0),     # yaw   (F = right)
)


def axis_keys(heave_keys: str = "qe"):
    """'qe' -> Q up / E down (default);  'eq' -> E up / Q down (as in EDMDc/teleop_tank.py)."""
    if heave_keys == "qe":
        return AXIS_KEYS
    if heave_keys == "eq":
        return tuple((a, {"e": "q", "q": "e"}.get(k, k), s) for a, k, s in AXIS_KEYS)
    raise ValueError(heave_keys)
AXIS_NAMES = ("surge", "sway", "heave", "yaw")


def cmd4_to_body_wrench_dir(cmd4: np.ndarray) -> np.ndarray:
    """Piloted 4-axis command -> [Fx, Fy, Fz, 0, 0, Tz] direction in the blimp body frame."""
    surge, sway, heave, yaw = cmd4
    return np.array([surge, -sway, -heave, 0.0, 0.0, yaw])


class KeyboardBlimpController:
    """Drop-in replacement for BlimpPD: `command(blimp)` reads the held keys.

    Authority: per-axis wrench that drives at least one thruster to T_max when that axis is
    commanded alone at gain 1 (same idea as StaticArduSubAllocator.authority). Combined axes are
    scaled globally so no thruster exceeds T_max (BlueROV allocator behaviour), never clipped
    per thruster.
    """

    def __init__(self, blimp: Blimp, gain: float = 0.4, key_timeout: float = 0.75, heave_keys: str = "qe",
                 hold: BlimpPD | None = None):
        self.blimp = blimp
        self.hold = hold                  # optional PID used as position/altitude/yaw HOLD when task.pid_enabled
        self.axis_keys = axis_keys(heave_keys)
        self.gain = float(np.clip(gain, 0.1, 1.0))
        self.key_timeout = key_timeout
        self.pressed: set = set()
        self.last_activity = time.monotonic()
        self.running = True
        self.capture_requested = False
        self.cmd4 = np.zeros(4)
        self.rebuild()

    def rebuild(self):
        b = self.blimp
        self.authority = np.zeros(4)
        for axis in range(4):
            e = np.zeros(4); e[axis] = 1.0
            w = cmd4_to_body_wrench_dir(e)[b.CTRL_IDX]
            t = b.B_red_pinv @ w
            self.authority[axis] = b.p.T_max * b.p.batt_scale / np.max(np.abs(t))

    # ----- key events (names as matplotlib delivers them) ---------------------------
    def on_press(self, key: str):
        self.last_activity = time.monotonic()
        k = (key or "").lower()
        if k == "escape":
            self.running = False
        elif k == " " or k == "space":
            self.pressed.clear()
        elif k in ("+", "="):
            self.gain = min(1.0, round(self.gain + 0.1, 2)); print(f"[teleop] gain = {self.gain:.1f}")
        elif k == "-":
            self.gain = max(0.1, round(self.gain - 0.1, 2)); print(f"[teleop] gain = {self.gain:.1f}")
        elif k == "p":
            self.capture_requested = True
        else:
            self.pressed.add(k)

    def on_release(self, key: str):
        self.last_activity = time.monotonic()
        self.pressed.discard((key or "").lower())

    def reset(self):
        """Sim reset: drop held keys and the captured hold reference (re-captured on the next command)."""
        self.pressed.clear()
        self.cmd4 = np.zeros(4)
        if self.hold is not None:
            self.hold.reset()
            self.hold.p_ref_override = None
            self.hold.yaw_ref_override = None

    def on_pid_toggle(self, enabled: bool):
        """Capture 'hold where you are' when the PID assist is switched on."""
        if self.hold is None:
            return
        self.hold.reset()
        if enabled:
            p, _, _ = self.hold.controlled_point(self.blimp)
            self.hold.p_ref_override = p.copy()
            self.hold.yaw_ref_override = float(self.blimp.eta[5])

    # ----- controller interface -----------------------------------------------------
    def current_cmd4(self) -> np.ndarray:
        if self.pressed and self.key_timeout > 0 and time.monotonic() - self.last_activity > self.key_timeout:
            print("[teleop] key timeout -> zero command"); self.pressed.clear()   # lost KEY_RELEASE guard
        cmd = np.zeros(4)
        for axis, key, sign in self.axis_keys:
            if key in self.pressed:
                cmd[axis] += sign
        return np.clip(cmd, -1.0, 1.0) * self.gain

    def command(self, blimp: Blimp, cmd4=None):
        self.cmd4 = self.current_cmd4() if cmd4 is None else np.asarray(cmd4, float)
        wrench = cmd4_to_body_wrench_dir(self.cmd4) * np.array([self.authority[0], self.authority[1], self.authority[2], 0, 0, self.authority[3]])
        if self.hold is not None and self.hold.task.pid_enabled:
            # PID hold assist: axes with a key held are manual (and their hold reference follows the
            # vehicle); idle axes are held by the PID at the last captured reference.
            p, _, R = self.hold.controlled_point(blimp)
            if self.hold.p_ref_override is None:
                self.hold.p_ref_override = p.copy(); self.hold.yaw_ref_override = float(blimp.eta[5])
            manual_xy = abs(self.cmd4[0]) > 0 or abs(self.cmd4[1]) > 0
            if manual_xy:
                self.hold.p_ref_override[:2] = p[:2]
            if abs(self.cmd4[2]) > 0:
                self.hold.p_ref_override[2] = p[2]
            if abs(self.cmd4[3]) > 0:
                self.hold.yaw_ref_override = float(blimp.eta[5])
            w_pid = self.hold.wrench(blimp)
            if not manual_xy:
                wrench[0], wrench[1] = w_pid[0], w_pid[1]
            if abs(self.cmd4[2]) == 0:
                wrench[2] = w_pid[2]
            if abs(self.cmd4[3]) == 0:
                wrench[5] = w_pid[5]
        t = blimp.B_red_pinv @ wrench[blimp.CTRL_IDX]
        u = t / (blimp.p.T_max * blimp.p.batt_scale)
        m = np.max(np.abs(u))
        if m > 1.0:                       # global scaling, as the ArduSub allocator does
            u = u / m
        blimp.command(u)
        return u
