"""MuJoCo's interactive viewer running the team scenario on the MuJoCo rover backend.

    python run_mujoco_gui.py                        # teleop blimp (hold assist on), 4 rovers on the circle
    python run_mujoco_gui.py --mode auto            # blimp autopilot to the circle centre at 1 m
    python run_mujoco_gui.py --no-camera-window     # viewer only (camera streams stay inside the viewer)
    python run_mujoco_gui.py --dump-mjcf scene.xml  # also save the model (python -m mujoco.viewer --mjcf scene.xml)

Windows
  * MuJoCo viewer: MuJoCo's own GUI. Left panel: rendering/visualization flags (contact points
    and forces, camera frustums...), physics options, watch. Right panel: joint sliders and the
    wheel-actuator Control sliders. Rendering > Camera switches the main view to a rover or blimp
    camera. Camera streams are overlaid inside the view: a strip of live thumbnails along the
    bottom and the selected camera enlarged at the bottom right; a HUD shows the status and keys.
    Physics is stepped by this script at the fixed 100 Hz / 20 Hz rates, so the viewer's own
    Run/speed controls are inactive. Ctrl + right-drag pushes a rover.
  * Vehicle cameras (OpenCV window, needs opencv-python): the same streams as a grid, and the
    place to type WASD: MuJoCo reserves every letter key for its own toggles (W wireframe,
    S shadows, Q camera frustums, ...), so letters are only read in this window. Its two
    trackbars re-aim the cameras live: blimp camera tilt (90 = straight down, 0 = straight
    ahead) and rover camera pitch (-30..60 deg below the horizon).

Keys and mouse
  The viewer reports key presses only (no release), so a flight key is a PULSE: one tap applies
  the command for --pulse seconds (default 0.5 s), tapping again renews it, the opposite key
  replaces it and Space releases every axis. Holding a key in the camera window works like
  holding it in the browser: the key auto-repeat keeps renewing the command, which then stops
  about 0.1 s after release. With hold assist on, idle axes are held by the PID
  (as in the browser GUI). For continuous control, drag with the left mouse button in the camera
  window like a pair of thumbsticks: a drag started in the left half = surge/sway, in the right
  half = up-down/yaw, proportional to the drag distance (--stick-radius pixels = full command);
  releasing the button zeroes the command. (Do not right-click there: OpenCV's Qt window grabs it.)
    In the MuJoCo viewer:  Up/Down surge · Left/Right sway · PgUp/PgDn up/down · Home/End yaw
                           Tab hold assist / PID on-off · Del manual wheels (Control sliders drive them)
    In the camera window:  W/S surge · A/D sway · Q/E up/down · F/R yaw · H hold · M manual wheels · P pause
                           left-half drag surge/sway · right-half drag up-down/yaw
    In both:               Space release all · Enter pause/run · Backspace reset · +/- manual gain
                           1-9 select a camera, 0 none/grid · Esc quits the viewer

The blimp dynamics stay in sim/blimp.py and are mirrored into the mocap body; its physical
parameters are not editable here (use the browser GUI's sliders for that).
"""
from __future__ import annotations

import argparse
import os
import time

import numpy as np

# The viewer owns a GLFW window on its own thread; render the vehicle cameras through EGL so the
# two never share GLFW state. Override with MUJOCO_GL=glfw if EGL is unavailable on your machine.
os.environ.setdefault("MUJOCO_GL", "egl")

from sim.controllers import BlimpPD  # noqa: E402
from sim.keyboard import KeyboardBlimpController  # noqa: E402
from sim.params import SimParams  # noqa: E402
from sim.sim import TeamSim  # noqa: E402

# GLFW key codes
KEY_SPACE, KEY_ENTER, KEY_TAB, KEY_BACKSPACE, KEY_DELETE = 32, 257, 258, 259, 261
KEY_RIGHT, KEY_LEFT, KEY_DOWN, KEY_UP, KEY_PAGE_UP, KEY_PAGE_DOWN, KEY_HOME, KEY_END = 262, 263, 264, 265, 266, 267, 268, 269
KEY_GAIN_UP, KEY_GAIN_DOWN = (61, 334), (45, 333)                     # '=' / keypad +, '-' / keypad -
AXIS_PARTNER = {"w": "s", "s": "w", "a": "d", "d": "a", "q": "e", "e": "q", "f": "r", "r": "f"}
VIEWER_AXIS_KEYS = {KEY_UP: "w", KEY_DOWN: "s", KEY_LEFT: "a", KEY_RIGHT: "d",
                    KEY_PAGE_UP: "q", KEY_PAGE_DOWN: "e", KEY_HOME: "f", KEY_END: "r"}
MARKER_RGBA = (0.45, 0.55, 0.60, 0.85)
REPEAT_GAP, HOLD_TAIL = 0.08, 0.10                # key events closer than REPEAT_GAP s are auto-repeat: a held key
MARGIN = 8                                        # overlay spacing (px)
THUMB_STEPS, LARGE_STEPS = (4, 5, 8), (2, 4)      # integer subsampling of the 640x480 frames: 160/128/80 px and 320/160 px wide


def cv_key_to_glfw(code: int) -> int | None:
    """Map an OpenCV waitKey code to the GLFW code used by GuiSession.on_key."""
    if code is None or code < 0:
        return None
    code &= 0xFF
    if 97 <= code <= 122:                      # lowercase letters -> GLFW uppercase codes
        return code - 32
    return {8: KEY_BACKSPACE, 13: KEY_ENTER, 9: KEY_TAB, 127: KEY_DELETE}.get(code, code)


def stick_from_drag(origin, point, pad, radius):
    """Thumbstick drag -> cmd4 contribution [surge, sway(+left), heave(+up), yaw(+right)] in [-1, 1].

    Screen y grows downward, so dragging up is positive surge/heave. The "left" pad (drag begun in
    the left half of the window) gives surge/sway, the "right" pad heave/yaw. `radius` pixels of
    drag give a full command.
    """
    dx = (point[0] - origin[0]) / float(radius)
    dy = (origin[1] - point[1]) / float(radius)
    cmd = np.zeros(4)
    if pad == "left":
        cmd[0], cmd[1] = dy, -dx
    elif pad == "right":
        cmd[2], cmd[3] = dy, dx
    return np.clip(cmd, -1.0, 1.0)


class StickKeyboard(KeyboardBlimpController):
    """Keyboard controller that adds an analog joystick vector to the pulsed keys."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.stick = np.zeros(4)

    def current_cmd4(self) -> np.ndarray:
        cmd = np.zeros(4)
        for axis, key, sign in self.axis_keys:
            if key in self.pressed:
                cmd[axis] += sign
        cmd += self.stick
        return np.clip(cmd, -1.0, 1.0) * self.gain

    def reset(self):
        super().reset()
        self.stick[:] = 0.0


def pip_layout(viewport, names, frames, selected):
    """Picture-in-picture rectangles for the viewer: (left, bottom, width, height, rgb) per overlay.

    Thumbnails run along the bottom edge from the left; the selected camera is enlarged at the
    bottom right when it fits. Rectangles are in framebuffer pixels (MuJoCo's bottom-left origin).
    """
    out = []
    if not frames or viewport is None:
        return out
    left, bottom, width, height = viewport.left, viewport.bottom, viewport.width, viewport.height
    present = [name for name in names if name in frames]
    if not present:
        return out
    h0, w0 = frames[present[0]].shape[:2]
    # Thumbnail size: the largest step at which every camera fits across the viewport, else the smallest.
    step = next((s for s in THUMB_STEPS if len(present) * (w0 // s + MARGIN) + MARGIN <= width), THUMB_STEPS[-1])
    thumbs = [(name, np.ascontiguousarray(frames[name][::step, ::step])) for name in present]
    th, tw = thumbs[0][1].shape[:2]
    large = None
    if selected in frames:
        for ls in LARGE_STEPS:
            lh, lw = h0 // ls, w0 // ls
            if lw + 2 * MARGIN <= width and lh + th + 3 * MARGIN <= height:
                img = np.ascontiguousarray(frames[selected][::ls, ::ls])
                large = (left + width - lw - MARGIN, bottom + th + 2 * MARGIN, lw, lh, img)
                break
    x = left + MARGIN
    for name, img in thumbs:
        if x + tw > left + width - MARGIN or th + 2 * MARGIN > height:
            break
        if name == selected:                    # highlight the enlarged camera's thumbnail
            img = img.copy()
            img[:2], img[-2:], img[:, :2], img[:, -2:] = (69, 217, 210), (69, 217, 210), (69, 217, 210), (69, 217, 210)
        out.append((x, bottom + MARGIN, tw, th, img))
        x += tw + MARGIN
    if large is not None:
        out.append(large)
    return out


class CameraWindow:
    """Grid of all vehicle cameras (or one enlarged) composed as an RGB image; shown with OpenCV."""
    TILE = (240, 320)                           # rows, cols of each grid tile (half resolution)
    TITLE = "Vehicle cameras  -  focus here for WASD teleop  (1-9 enlarge, 0 grid)"
    TRACKBARS = {"blimp cam tilt (deg)": ("blimp_tilt_deg", 0.0, 90.0), "rover cam pitch (deg, -30..60)": ("rover_pitch_deg", -30.0, 60.0)}

    def __init__(self, names, stick_radius: int = 80, blimp_tilt_deg: float = 90.0, rover_pitch_deg: float = 10.0):
        self.names = list(names)
        self.stick_radius = int(stick_radius)
        self.drag = None                        # (origin (x, y), point (x, y), "left" | "right" pad) while the button is held
        self.width = 2 * self.TILE[1]           # width of the last composed image (pad split at the middle)
        self.angles = {"blimp_tilt_deg": float(blimp_tilt_deg), "rover_pitch_deg": float(rover_pitch_deg)}
        self.pending_angles = {}                # trackbar changes, applied by the main loop under the viewer lock
        self._open = False
        self._closed_by_user = False

    # -- camera trackbars
    def on_trackbar(self, name, position):
        """Trackbar callback (positions are integers from 0): queue the new angle for the main loop."""
        key, low, _ = self.TRACKBARS[name]
        value = float(low + position)
        if value != self.angles[key]:
            self.angles[key] = value
            self.pending_angles[key] = value

    def take_pending_angles(self):
        pending, self.pending_angles = self.pending_angles, {}
        return pending

    # -- mouse thumbsticks (left button only: the right button freezes OpenCV's Qt window)
    def on_mouse(self, event, x, y, flags=0, userdata=None):
        import cv2
        if event == cv2.EVENT_LBUTTONDOWN:
            self.drag = ((x, y), (x, y), "left" if x < self.width / 2 else "right")
        elif event == cv2.EVENT_MOUSEMOVE and self.drag is not None:
            self.drag = (self.drag[0], (x, y), self.drag[2])
        elif event == cv2.EVENT_LBUTTONUP:
            self.drag = None

    @property
    def stick(self) -> np.ndarray:
        if self.drag is None:
            return np.zeros(4)
        origin, point, pad = self.drag
        return stick_from_drag(origin, point, pad, self.stick_radius)

    def compose(self, frames, selected=None) -> np.ndarray:
        if not frames:
            img = np.zeros((*self.TILE, 3), dtype=np.uint8)
        elif selected in frames:
            img = self._label(frames[selected].copy(), f"{selected}  (0 = grid)")
        else:
            tiles = []
            for i, name in enumerate(self.names):
                frame = frames.get(name)
                tile = frame[::2, ::2] if frame is not None else np.zeros((*self.TILE, 3), dtype=np.uint8)
                tiles.append(self._label(np.ascontiguousarray(tile), f"{i + 1}: {name}"))
            cols = min(3, len(tiles))
            rows = -(-len(tiles) // cols)
            blank = np.zeros_like(tiles[0])
            tiles += [blank] * (rows * cols - len(tiles))
            img = np.vstack([np.hstack(tiles[r * cols:(r + 1) * cols]) for r in range(rows)])
        self.width = img.shape[1]
        return self._draw_stick(img)

    def _draw_stick(self, img):
        """Pad hints along the bottom edge; while dragging, the origin ring, deflection line and knob."""
        try:
            import cv2
        except ImportError:
            return img
        h, w = img.shape[:2]
        cv2.line(img, (w // 2, h - 26), (w // 2, h), (120, 120, 120), 1)
        for text, x in (("drag: surge / sway", 8), ("drag: up-down / yaw", w // 2 + 8)):
            cv2.putText(img, text, (x, h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1, cv2.LINE_AA)
        if self.drag is None:
            return img
        origin, point, pad = self.drag
        color = (69, 217, 210) if pad == "left" else (255, 172, 91)
        cv2.circle(img, origin, self.stick_radius, color, 1, cv2.LINE_AA)
        cv2.line(img, origin, point, color, 2, cv2.LINE_AA)
        cv2.circle(img, point, 7, color, -1, cv2.LINE_AA)
        cmd = self.stick
        text = (f"stick surge {cmd[0]:+.2f} sway {cmd[1]:+.2f}" if pad == "left" else f"stick up {cmd[2]:+.2f} yaw {cmd[3]:+.2f}")
        cv2.putText(img, text, (max(4, origin[0] - 90), min(h - 30, origin[1] + self.stick_radius + 18)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
        return img

    @staticmethod
    def _label(img, text):
        try:
            import cv2
            cv2.rectangle(img, (0, 0), (8 + 9 * len(text), 22), (0, 0, 0), -1)
            cv2.putText(img, text, (4, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        except ImportError:
            pass
        return img

    def show(self, frames, selected=None) -> list:
        """Display the composed image; returns the GLFW-style key codes pressed in this window.

        All queued key events are drained each call (key auto-repeat can outpace the 20 Hz loop),
        so a held key's command ends right after the physical release.
        """
        if self._closed_by_user:
            return []
        import cv2
        if self._open and cv2.getWindowProperty(self.TITLE, cv2.WND_PROP_VISIBLE) < 1:
            self._closed_by_user = True
            return []
        if not self._open:
            cv2.namedWindow(self.TITLE, cv2.WINDOW_AUTOSIZE)
            cv2.setMouseCallback(self.TITLE, self.on_mouse)
            for name, (key, low, high) in self.TRACKBARS.items():
                cv2.createTrackbar(name, self.TITLE, int(round(self.angles[key] - low)), int(round(high - low)),
                                   lambda position, name=name: self.on_trackbar(name, position))
        cv2.imshow(self.TITLE, self.compose(frames, selected)[..., ::-1])
        self._open = True
        keys = []
        for _ in range(16):
            key = cv_key_to_glfw(cv2.waitKey(1))
            if key is None:
                break
            keys.append(key)
        return keys

    def close(self):
        if self._open:
            import cv2
            cv2.destroyAllWindows()
            self._open = False


class GuiSession:
    """Scenario stepping, key handling, HUD text and overlays, independent of the viewer window."""

    def __init__(self, P: SimParams, speed: float = 1.0, mode: str = "teleop", cameras: bool = True, camera_window: bool = False,
                 pulse: float = 0.5, stick_radius: int = 80):
        if P.rover_backend != "mujoco":
            raise ValueError("run_mujoco_gui needs SimParams.rover_backend == 'mujoco'")
        if mode not in ("teleop", "auto"):
            raise ValueError("mode must be 'teleop' or 'auto'")
        if not pulse > 0:
            raise ValueError("pulse must be positive")
        self.P, self.speed, self.mode, self.pulse = P, float(speed), mode, float(pulse)
        P.cameras = bool(cameras)
        self.sim = TeamSim(P)
        self.keyboard = None
        self.pulse_until = {}                   # flight key -> simulation time when its pulse ends
        self.last_tap = {}                      # flight key -> wall-clock time of its last key event
        self.clock = time.monotonic             # replaceable in tests
        if mode == "teleop":
            self.keyboard = StickKeyboard(self.sim.blimp, key_timeout=0, hold=BlimpPD(P.task, dt=P.dt_ctrl))
            self.sim.set_controllers(self.sim.rover_ctrl, self.keyboard)
        self.camera_names = list(self.world.frame_names) if cameras else []
        self.selected_camera = None
        self.camera_window = (CameraWindow(self.camera_names, stick_radius, P.mujoco.blimp_cam_tilt_deg, P.mujoco.rover_cam_pitch_deg)
                              if cameras and camera_window else None)
        self.paused = False
        self.status = ""
        self.reset()

    @property
    def world(self):
        return self.sim.world

    def reset(self):
        self.pulse_until.clear()
        self.last_tap.clear()
        if self.camera_window is not None:
            self.camera_window.drag = None
        self.sim.reset()
        if self.keyboard is not None:
            self.keyboard.rebuild()
            self.keyboard.on_pid_toggle(self.P.task.pid_enabled)   # arm the hold reference at the start pose

    # ------------------------------------------------------------ keys
    def tap(self, key: str):
        """Flight key event with press-only input.

        A tap starts (or renews) a pulse of `pulse` simulated seconds; the partner key replaces
        it. Key events arriving within REPEAT_GAP of the previous one are the OS auto-repeat of a
        held key: they renew only a short HOLD_TAIL, so the command follows the physical hold and
        stops shortly after release instead of accumulating pulses.
        """
        if self.keyboard is None:
            return
        wall = self.clock()
        repeat = wall - self.last_tap.get(key, float("-inf")) < REPEAT_GAP
        self.last_tap[key] = wall
        self.pulse_until.pop(AXIS_PARTNER[key], None)
        self.pulse_until[key] = self.sim.t + (HOLD_TAIL if repeat else self.pulse)
        self.refresh_keys()

    def refresh_keys(self):
        """Drop expired pulses and push the active keys (and the joystick) into the controller."""
        if self.keyboard is None:
            return
        now = self.sim.t
        self.pulse_until = {k: u for k, u in self.pulse_until.items() if u > now}
        self.keyboard.pressed = set(self.pulse_until)
        if self.camera_window is not None:
            self.keyboard.stick = self.camera_window.stick
        self.keyboard.cmd4 = self.keyboard.current_cmd4()

    def release_all(self):
        self.pulse_until.clear()
        self.last_tap.clear()
        if self.camera_window is not None:
            self.camera_window.drag = None
        if self.keyboard is not None:
            self.keyboard.pressed.clear()
            self.keyboard.stick[:] = 0.0
            self.keyboard.cmd4 = np.zeros(4)

    def select_camera(self, digit: int):
        """1-9 pick a camera (if it exists), 0 clears the enlarged view / returns the window to its grid."""
        if digit == 0:
            self.selected_camera = None
        elif 1 <= digit <= len(self.camera_names):
            self.selected_camera = self.camera_names[digit - 1]

    def toggle_hold(self):
        self.sim.set_pid(not self.P.task.pid_enabled)

    def toggle_manual_wheels(self):
        self.world.manual_ctrl = not self.world.manual_ctrl
        if not self.world.manual_ctrl:                      # hand the wheels back to the tracker cleanly
            self.world.data.ctrl[:] = 0.0

    def on_key(self, keycode: int, source: str = "viewer"):
        """Key press from the MuJoCo viewer or the camera window (GLFW key codes).

        Letters are honoured only from the camera window: in the viewer every letter also
        toggles one of MuJoCo's own flags, so there the arrow/PgUp/PgDn/Home/End layout is used.
        """
        letter = chr(keycode).lower() if 65 <= keycode <= 90 else None
        if keycode == KEY_SPACE:
            self.release_all()
        elif keycode == KEY_BACKSPACE:
            self.reset()
        elif keycode == KEY_ENTER or (letter == "p" and source == "window"):
            self.paused = not self.paused
        elif keycode in VIEWER_AXIS_KEYS:
            self.tap(VIEWER_AXIS_KEYS[keycode])
        elif letter in AXIS_PARTNER and source == "window":
            self.tap(letter)
        elif keycode == KEY_TAB or (letter == "h" and source == "window"):
            self.toggle_hold()
        elif keycode == KEY_DELETE or (letter == "m" and source == "window"):
            self.toggle_manual_wheels()
        elif keycode in KEY_GAIN_UP and self.keyboard is not None:
            self.keyboard.gain = min(1.0, round(self.keyboard.gain + 0.1, 2))
        elif keycode in KEY_GAIN_DOWN and self.keyboard is not None:
            self.keyboard.gain = max(0.1, round(self.keyboard.gain - 0.1, 2))
        elif 48 <= keycode <= 57:
            self.select_camera(keycode - 48)

    # ------------------------------------------------------------ stepping
    def active_keys_text(self):
        """Pulsed keys with their remaining time, plus the joystick vector when deflected."""
        parts = [f"{k.upper()} {max(0.0, u - self.sim.t):.1f}s" for k, u in sorted(self.pulse_until.items())]
        if self.keyboard is not None and np.any(self.keyboard.stick):
            parts.append("stick " + " ".join(f"{v:+.2f}" for v in self.keyboard.stick))
        return " ".join(parts) or "-"

    def step(self):
        self.refresh_keys()
        if not self.paused:
            self.sim.step()
        flags = [f"t = {self.sim.t:6.2f} s", "paused" if self.paused else "running"]
        if self.keyboard is not None:
            flags.append(f"teleop {self.active_keys_text()}")
            flags.append(f"hold {'on' if self.P.task.pid_enabled else 'off'}")
            flags.append(f"gain {self.keyboard.gain:.1f}")
        else:
            flags.append(f"autopilot PID {'on' if self.P.task.pid_enabled else 'off'}")
        if self.world.manual_ctrl:
            flags.append("manual wheels")
        if self.selected_camera:
            flags.append(f"camera {self.selected_camera}")
        self.status = " | ".join(flags)

    def hud_lines(self):
        """(label column, value column) for the viewer's compact top-left text overlay."""
        b = self.sim.blimp
        labels = "t\npos\nclr\nkeys\nhold\nmode\nhelp"
        if self.keyboard is not None:
            keys = self.active_keys_text() + f"  gain {self.keyboard.gain:.1f}"
            hold = ("on" if self.P.task.pid_enabled else "off") + "  (Tab)"
            help_line = f"taps pulse {self.pulse:.1f}s | WASD + mouse stick in cam window"
        else:
            keys = "autopilot"
            hold = ("PID on" if self.P.task.pid_enabled else "PID off") + "  (Tab)"
            help_line = "Enter pause | Bksp reset | Del wheels | 1-9 cams"
        M = self.P.mujoco
        mode = ("paused" if self.paused else "running") + (" | manual wheels" if self.world.manual_ctrl else "") \
            + (f" | cam {self.selected_camera}" if self.selected_camera else "") \
            + f" | blimp cam {M.blimp_cam_tilt_deg:.0f} deg, rover cam {M.rover_cam_pitch_deg:.0f} deg"
        values = (f"{self.sim.t:.2f} s\n{b.eta[0]:+.2f}, {b.eta[1]:+.2f} m  yaw {np.degrees(b.eta[5]):+.0f}\n"
                  f"{b.altitude:.2f} m\n{keys}\n{hold}\n{mode}\n{help_line}")
        return labels, values

    def draw_markers(self, scene):
        """Rover reference points on the floor, into the viewer's user scene."""
        import mujoco
        from sim.mujoco_world import ned_to_mj_pos
        scene.ngeom = 0
        for i in range(len(self.sim.rovers)):
            if scene.ngeom >= scene.maxgeom:
                break
            ref, _ = self.sim.rover_ctrl.reference(i, self.sim.t)
            p = ned_to_mj_pos([ref[0], ref[1], 0.0])
            mujoco.mjv_initGeom(scene.geoms[scene.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE, np.array([0.02, 0.0, 0.0]),
                                np.array([p[0], p[1], 0.02]), np.eye(3).flatten(), np.array(MARKER_RGBA, dtype=np.float32))
            scene.ngeom += 1

    def overlays(self, viewport):
        return pip_layout(viewport, self.camera_names, self.sim.frames, self.selected_camera)

    def close(self):
        if self.camera_window is not None:
            self.camera_window.close()
        if self.world is not None:
            self.world.close()


def run(session: GuiSession, viewer, duration: float | None = None):
    """Real-time paced loop against a (passive) viewer: step under its lock, overlay, sync, cameras, sleep."""
    import mujoco
    dt = session.P.dt_ctrl / session.speed
    next_tick = start = time.monotonic()
    while viewer.is_running() and (duration is None or time.monotonic() - start < duration):
        with viewer.lock():
            session.step()
            session.draw_markers(viewer.user_scn)
            labels, values = session.hud_lines()
            viewer.set_texts((None, mujoco.mjtGridPos.mjGRID_TOPLEFT, labels, values))
            images = [(mujoco.MjrRect(x, y, w, h), img) for x, y, w, h, img in session.overlays(viewer.viewport)]
            if images:
                viewer.set_images(images)
            else:
                viewer.clear_images()
        viewer.sync()
        if session.camera_window is not None:
            keys = session.camera_window.show(session.sim.frames, session.selected_camera)
            angles = session.camera_window.take_pending_angles()
            if keys or angles:
                with viewer.lock():
                    for key in keys:
                        session.on_key(key, source="window")
                    if angles:
                        session.world.set_camera_angles(**angles)
        next_tick += dt
        now = time.monotonic()
        if next_tick < now - 0.5:                           # fell far behind: do not replay the backlog
            next_tick = now
        time.sleep(max(0.0, next_tick - now))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--mode", choices=["teleop", "auto"], default="teleop", help="blimp: pulsed keyboard/joystick teleop with hold assist, or autopilot")
    ap.add_argument("--n", type=int, default=4)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--speed", type=float, default=1.0, help="simulation speed relative to real time")
    ap.add_argument("--gain", type=float, default=0.4, help="initial manual gain (0.1-1)")
    ap.add_argument("--pulse", type=float, default=0.5, help="seconds of command per flight-key tap (simulated time)")
    ap.add_argument("--stick-radius", type=int, default=80, help="mouse drag distance (px) in the camera window for a full command")
    ap.add_argument("--rover-speed", type=float, default=None, help="override the circle speed (m/s)")
    ap.add_argument("--circle-radius", type=float, default=None, help="override the circle radius (m)")
    ap.add_argument("--blimp-cam-tilt", type=float, default=None, help="blimp camera tilt in degrees: 90 straight down (default), 0 straight ahead")
    ap.add_argument("--rover-cam-pitch", type=float, default=None, help="rover camera pitch below the horizon in degrees (default 10)")
    ap.add_argument("--no-cameras", dest="cameras", action="store_false", help="do not render the vehicle cameras at all")
    ap.add_argument("--camera-window", dest="camera_window", action="store_true", default=None,
                    help="open the OpenCV camera/WASD window (default when opencv-python is installed)")
    ap.add_argument("--no-camera-window", dest="camera_window", action="store_false")
    ap.add_argument("--T", type=float, default=None, help="quit after this many wall-clock seconds (demos/tests)")
    ap.add_argument("--dump-mjcf", default=None, help="write the generated MJCF model to this file")
    a = ap.parse_args()

    camera_window = a.camera_window
    if camera_window is None or camera_window:
        try:
            import cv2  # noqa: F401
            camera_window = a.cameras
        except ImportError:
            if camera_window:
                raise
            camera_window = False
            print("opencv-python not installed: no camera/WASD window; use the arrow-key layout in the viewer")

    P = SimParams(seed=a.seed, rover_backend="mujoco")
    P.task.n_rovers = a.n
    if a.rover_speed is not None:
        P.task.rover_speed = a.rover_speed
    if a.circle_radius is not None:
        P.task.circle_radius = a.circle_radius
    if a.blimp_cam_tilt is not None:
        P.mujoco.blimp_cam_tilt_deg = float(np.clip(a.blimp_cam_tilt, 0.0, 90.0))
    if a.rover_cam_pitch is not None:
        P.mujoco.rover_cam_pitch_deg = float(np.clip(a.rover_cam_pitch, -30.0, 60.0))
    session = GuiSession(P, speed=a.speed, mode=a.mode, cameras=a.cameras, camera_window=camera_window,
                         pulse=a.pulse, stick_radius=a.stick_radius)
    if session.keyboard is not None:
        session.keyboard.gain = float(np.clip(a.gain, 0.1, 1.0))
    if a.dump_mjcf:
        with open(a.dump_mjcf, "w") as f:
            f.write(session.world.xml)
        print(f"MJCF written to {a.dump_mjcf}")

    import mujoco
    import mujoco.viewer
    print(__doc__.split("Keys and mouse\n")[1].split("\n\n")[0].strip())
    try:
        with mujoco.viewer.launch_passive(session.world.model, session.world.data, key_callback=session.on_key) as viewer:
            with viewer.lock():
                viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CAMERA] = True   # show the vehicle camera frustums
                viewer.cam.lookat[:] = (0.0, 0.0, 0.3)
                viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = 6.0, 135.0, -30.0
            run(session, viewer, duration=a.T)
    finally:
        session.close()
        eta = session.sim.blimp.eta
        print(f"final blimp position ({eta[0]:+.2f}, {eta[1]:+.2f}) m, clearance {session.sim.blimp.altitude:.2f} m, t = {session.sim.t:.1f} s")


if __name__ == "__main__":
    main()
