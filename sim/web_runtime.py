"""Thread-owned simulation for the offline browser UI (no Matplotlib imports)."""
from __future__ import annotations

import copy
from collections import deque
from concurrent.futures import Future
from dataclasses import asdict
import io
import json
import math
import queue
import threading
import time

import numpy as np

from .blimp import R_zyx
from .camera import (CameraRecording, camera_intrinsics, camera_pose,
                     default_camera_profile, empty_recording_status, validate_camera_profile)
from .controllers import BlimpPD, CircleTracker
from .keyboard import KeyboardBlimpController
from .params import G, SLIDERS, SimParams
from .sim import TeamSim


PARAMETER_LABELS = {
    "T_max": ("Maximum thruster force", "N"), "tau_p": ("Thruster response time", "s"),
    "d_quad_yaw": ("Quadratic yaw drag", "N m s²/rad²"), "d_lin": ("Linear drag", "N s/m"),
    "d_quad_xy": ("Horizontal quadratic drag", "N s²/m²"), "d_quad_z": ("Vertical quadratic drag", "N s²/m²"),
    "k_add_xy": ("Horizontal added mass", ""), "k_add_z": ("Vertical added mass", ""),
    "I_z": ("Yaw inertia", "kg m²"), "b_z": ("Yaw damping", "N m s/rad"),
    "b_xy": ("Swing damping", "N m s/rad"), "draught_N": ("Draught strength", "N"),
    "net_lift_g": ("Net lift", "g equiv."), "net_lift_drift": ("Lift drift", "N/√s"),
    "kp_pos": ("Position proportional gain", "N/m"), "kd_pos": ("Position derivative gain", "N s/m"),
    "ki_pos": ("Position integral gain", "N/(m s)"), "i_max_N": ("Integral force limit", "N"),
    "circle_radius": ("Circle radius", "m"), "rover_speed": ("Rover speed", "m/s"),
    "blimp_height": ("Desired altitude", "m"), "force_scale": ("Force arrow scale", "m/N"),
}
PARAMETER_DESCRIPTIONS = {
    "net_lift_g": "Positive lifts upward; negative adds downward load. Equivalent weight only.",
}
PARAMETER_BOUNDS = {f"{group}.{name}": (group, name, low, high) for group, name, low, high in SLIDERS}
MATRIX_PARAMETERS = {"blimp.k_add_xy", "blimp.k_add_z", "blimp.I_z"}
MOVEMENT_KEYS = frozenset("wasdqerf")


class IdleRovers:
    def __init__(self, task, n):
        self.task, self.n = task, n

    def reference(self, i, t):
        angle = 2 * np.pi * i / self.n
        return np.asarray(self.task.circle_center) + self.task.circle_radius * np.array([np.cos(angle), np.sin(angle)]), np.zeros(2)

    def command(self, i, rover, t):
        rover.command(0.0, 0.0)
        return np.zeros(2)


def bounded_number(value, low, high):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("Enter a number")
    if not low <= value <= high or not math.isfinite(value):
        raise ValueError(f"Value must be between {low:g} and {high:g}")
    return float(value)


def validate_command(message):
    """Return a sanitized command; never accept arbitrary attribute assignments."""
    if not isinstance(message, dict) or message.get("type") != "command":
        raise ValueError("Expected a command object")
    command_id = message.get("id")
    if not isinstance(command_id, str) or not 1 <= len(command_id) <= 128:
        raise ValueError("Command id must be a nonempty string of at most 128 characters")
    action = message.get("action")
    result = {"id": command_id, "action": action}
    if action == "param":
        key = message.get("key")
        if not isinstance(key, str) or key not in PARAMETER_BOUNDS:
            raise ValueError("Unknown parameter")
        _, _, low, high = PARAMETER_BOUNDS[key]
        result.update(key=key, value=bounded_number(message.get("value"), low, high))
    elif action == "keys":
        keys = message.get("keys")
        if not isinstance(keys, list) or len(keys) > 8 or any(not isinstance(key, str) or key not in MOVEMENT_KEYS for key in keys):
            raise ValueError("Keys must be a list containing only w, a, s, d, q, e, r, f")
        result["keys"] = sorted(set(keys))
    elif action == "gain":
        result["value"] = bounded_number(message.get("value"), 0.1, 1.0)
    elif action in ("pid", "pause"):
        if type(message.get("value")) is not bool:
            raise ValueError("Expected true or false")
        result["value"] = message["value"]
    elif action in ("mode", "rovers"):
        choices = ("teleop", "auto") if action == "mode" else ("circle", "idle")
        if message.get("value") not in choices:
            raise ValueError(f"Choose {' or '.join(choices)}")
        result["value"] = message["value"]
    elif action == "camera_config":
        result["value"] = validate_camera_profile(message.get("value"))
    elif action not in ("reset", "stop", "camera_record_start", "camera_record_stop", "camera_record_clear"):
        raise ValueError("Unknown command action")
    return result


class SimulationRuntime:
    """One worker owns the mutable simulator; HTTP/WS only see copied snapshots.

    The scheduler never changes integration dt or accumulates browser frame work.
    If computation falls behind, simulation time runs slower than wall time.
    """
    KEY_TIMEOUT = 0.35

    def __init__(self, n=4, seed=0, mode="teleop", rovers_mode="circle", max_log_steps=None, max_export_bytes=None):
        if isinstance(n, bool) or not isinstance(n, int) or not 1 <= n <= 32:
            raise ValueError("Rover count must be between 1 and 32")
        if isinstance(seed, bool) or not isinstance(seed, int) or not 0 <= seed < 2**32:
            raise ValueError("Seed must be an integer between 0 and 4294967295")
        if mode not in ("teleop", "auto") or rovers_mode not in ("circle", "idle"):
            raise ValueError("Invalid simulation mode")
        if max_log_steps is not None and (type(max_log_steps) is not int or max_log_steps < 1):
            raise ValueError("Log limit must be a positive integer")
        if max_export_bytes is not None and (type(max_export_bytes) is not int or max_export_bytes < 1):
            raise ValueError("Export limit must be a positive integer")
        self.max_log_steps, self.max_export_bytes = max_log_steps, max_export_bytes
        self.limit_message = None
        self.P = SimParams(seed=seed)
        self.camera_profile = default_camera_profile(self.P.blimp)
        self.camera_recording = None
        self.P.task.n_rovers = n
        self.sim = TeamSim(self.P)
        self.keyboard = KeyboardBlimpController(self.sim.blimp, key_timeout=0, hold=BlimpPD(self.P.task, dt=self.P.dt_ctrl))
        self.autopilot = BlimpPD(self.P.task, dt=self.P.dt_ctrl)
        self.mode, self.rovers_mode = mode, rovers_mode
        self.paused = True
        self.generation = 0
        self.seq = 0
        self.error = None
        self.realtime_factor = 0.0
        self._realtime_intervals = deque(maxlen=20)
        self._realtime_last_step = None
        self._last_keys = float("-inf")
        self._commands = queue.Queue(maxsize=256)
        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._stop = threading.Event()
        self._thread = None
        self._choose_controllers()
        self._reset()
        self._publish()

    def _choose_controllers(self):
        rovers = CircleTracker(self.P.task, len(self.sim.rovers)) if self.rovers_mode == "circle" else IdleRovers(self.P.task, len(self.sim.rovers))
        self.sim.set_controllers(rovers, self.keyboard if self.mode == "teleop" else self.autopilot)

    def _reset(self, recording_reason="reset"):
        if self.camera_recording is not None:
            self.camera_recording.finish(recording_reason)
        self._clear_keys()
        self.limit_message = None
        self.keyboard.reset()
        self.autopilot.reset()
        self.autopilot.p_ref_override = None
        self.autopilot.yaw_ref_override = None
        self.autopilot.last_wrench[:] = 0.0
        self.keyboard.hold.last_wrench[:] = 0.0
        self.sim.reset()
        self.keyboard.rebuild()
        if self.mode == "teleop":
            self.keyboard.on_pid_toggle(self.P.task.pid_enabled)
        self.paused = True
        self.error = None
        self._reset_realtime_factor()
        self.generation += 1

    def _reset_realtime_factor(self):
        self._realtime_intervals.clear()
        self._realtime_last_step = None
        self.realtime_factor = 0.0

    def _record_step_timing(self, completed_at):
        """Measure simulated/wall elapsed time, avoiding reciprocal-average bias."""
        if self.paused:
            self._reset_realtime_factor()
            return
        if self._realtime_last_step is not None:
            self._realtime_intervals.append(max(completed_at - self._realtime_last_step, 1e-6))
            self.realtime_factor = len(self._realtime_intervals) * self.P.dt_ctrl / sum(self._realtime_intervals)
        self._realtime_last_step = completed_at

    def _clear_keys(self):
        self.keyboard.pressed.clear()
        self.keyboard.cmd4[:] = 0.0
        self._last_keys = float("-inf")

    def _expire_keys(self, now):
        if now - self._last_keys > self.KEY_TIMEOUT:
            self._clear_keys()

    def _apply(self, command, now):
        action = command["action"]
        if action == "param":
            key, value = command["key"], command["value"]
            group, name, _, _ = PARAMETER_BOUNDS[key]
            setattr(getattr(self.P, group), name, value)
            if key in MATRIX_PARAMETERS:
                self.sim.blimp.rebuild()
            if key == "blimp.T_max":
                self.keyboard.rebuild()
            if key == "task.blimp_height":
                self.keyboard.set_altitude_target(value)
        elif action == "keys":
            # Paused/automatic views cannot leave a manual command armed for later.
            self.keyboard.pressed = set(command["keys"]) if self.mode == "teleop" and not self.paused else set()
            self._last_keys = now
            self.keyboard.cmd4 = self.keyboard.current_cmd4()
        elif action == "gain":
            self.keyboard.gain = command["value"]
        elif action == "pid":
            self.sim.set_pid(command["value"])
        elif action == "pause":
            if self.error and not command["value"]:
                raise ValueError("Reset the simulation before resuming after an error")
            if self.limit_message and not command["value"]:
                raise ValueError(self.limit_message)
            if self.paused != command["value"] or command["value"]:
                self._reset_realtime_factor()
            self.paused = command["value"]
            self._clear_keys()
        elif action == "reset":
            self._reset()
        elif action == "mode":
            if self.mode != command["value"]:
                self.mode = command["value"]
                self._clear_keys()
                self.keyboard.reset()
                self.autopilot.reset()
                self._choose_controllers()
                if self.mode == "teleop":
                    self.keyboard.on_pid_toggle(self.P.task.pid_enabled)
        elif action == "rovers":
            self.rovers_mode = command["value"]
            self._choose_controllers()
        elif action == "stop":
            self._clear_keys()
        elif action == "disconnect":
            if self.camera_recording is not None:
                self.camera_recording.finish("disconnect")
            self._clear_keys()
            self.paused = True
            self._reset_realtime_factor()
        elif action == "camera_config":
            if self.camera_recording is not None and self.camera_recording.active:
                raise ValueError("Stop camera recording before changing its profile")
            self.camera_profile = copy.deepcopy(command["value"])
        elif action == "camera_record_start":
            if self.camera_recording is not None:
                raise ValueError("Clear the previous camera recording before starting another")
            if self.error:
                raise ValueError("Reset the simulation before starting a camera recording")
            if self.limit_message:
                raise ValueError(self.limit_message)
            self._make_state()  # Validate the starting state before retaining it.
            config = self.config()
            metadata = {"profile": self.camera_profile, "geometry": config["geometry"],
                        "arena": self.P.arena, "seed": self.P.seed, "generation": self.generation,
                        "n_rovers": len(self.sim.rovers), "dt_ctrl": self.P.dt_ctrl, "dt_phys": self.P.dt_phys,
                        "K_pixels": camera_intrinsics(self.camera_profile),
                        "initial_parameters": asdict(self.P),
                        "timing": "Each transition captures pre-state, applies blimp_U and rover_U for dt_ctrl, then captures post-state. Frames are sampled at exact control boundaries; the terminal frame can close a partial camera interval.",
                        "frames_convention": "World NED; body forward/right/down at CV; optical right/down/forward. Eta angles roll/pitch/yaw in radians; altitude is gondola-bottom clearance in metres.",
                        "limitations": "Ideal rendering with configurable pinhole/Brown-Conrady optics; no calibrated sensor noise, exposure, rolling shutter, compression, or hardware latency. Live browser previews may skip states; recording retains every control transition."}
            self.camera_recording = CameraRecording(metadata, self._camera_state())
        elif action == "camera_record_stop":
            if self.camera_recording is not None:
                self.camera_recording.finish("user_stop")
        elif action == "camera_record_clear":
            if self.camera_recording is not None and self.camera_recording.active:
                raise ValueError("Stop the camera recording before clearing it")
            self.camera_recording = None

    def _camera_state(self):
        b = self.sim.blimp
        return {"t": self.sim.t, "control_step": self.sim.k, "eta": b.eta.tolist(), "nu": b.nu.tolist(),
                "thrust": b.thrust.tolist(), "net_lift": float(b.lift), "altitude": b.altitude,
                "rover_q": [r.q.tolist() for r in self.sim.rovers],
                "camera": camera_pose(b.eta, self.camera_profile)}

    def _step(self):
        if self.max_log_steps is not None and self.sim.k >= self.max_log_steps:
            self._reach_log_limit()
            return
        recording = self.camera_recording
        capture = recording is not None and recording.active
        if capture:
            pre, parameters = self._camera_state(), asdict(self.P)
            context = {"mode": self.mode, "rovers_mode": self.rovers_mode, "gain": self.keyboard.gain,
                       "pid_enabled": self.P.task.pid_enabled}
        self.sim.step()
        self._make_state()  # Invalid/partial steps must never enter a recording.
        if capture:
            # CircleTracker returns an unconstrained request; Rover.command
            # saturates it before integration. Record the actual applied pair.
            recording.append(pre, self._camera_state(), np.clip(self.sim.log["blimp_u"][-1], -1, 1),
                             [r.u for r in self.sim.rovers], parameters, context,
                             self.sim.log["rover_u"][-1], self.sim.blimp.thrust_cmd)
        if self.max_log_steps is not None and self.sim.k >= self.max_log_steps:
            self._reach_log_limit()

    def _reach_log_limit(self):
        self.paused = True
        self._reset_realtime_factor()
        self._clear_keys()
        self.limit_message = f"This flight reached its {self.max_log_steps:,}-step recording limit. Download your data, then Reset to fly again."
        if self.camera_recording is not None:
            self.camera_recording.finish("log_limit")

    def _make_state(self):
        b = self.sim.blimp
        force = R_zyx(*b.eta[3:]) @ b.wrench(b.thrust)[:3]
        state = {
            "type": "state", "seq": self.seq + 1, "generation": self.generation,
            "control_step": self.sim.k,
            "t": self.sim.t, "paused": self.paused, "mode": self.mode, "rovers_mode": self.rovers_mode,
            "pid_enabled": self.P.task.pid_enabled, "gain": self.keyboard.gain,
            "eta": b.eta.tolist(), "nu": b.nu.tolist(), "thrust": b.thrust.tolist(),
            "command": self.keyboard.current_cmd4().tolist() if self.mode == "teleop" else [0.0] * 4,
            "rover_q": [rover.q.tolist() for rover in self.sim.rovers], "force_world": force.tolist(),
            "net_lift": float(b.lift), "net_lift_g": float(1000.0 * b.lift / G),
            "altitude": b.altitude, "cv_altitude": b.cv_altitude,
            "desired_altitude": self.keyboard.desired_altitude if self.mode == "teleop" else self.P.task.blimp_height,
            "params": {key: float(getattr(getattr(self.P, group), name)) for key, (group, name, _, _) in PARAMETER_BOUNDS.items()},
            "realtime_factor": self.realtime_factor if not self.paused else 0.0,
            "error": self.error,
            "limit_message": self.limit_message,
            "camera": {"profile": copy.deepcopy(self.camera_profile), "K_pixels": camera_intrinsics(self.camera_profile),
                       **camera_pose(b.eta, self.camera_profile)},
            "camera_recording": self.camera_recording.status() if self.camera_recording is not None else empty_recording_status(),
        }
        # Check the actual wire values, including derived force and altitude,
        # before they can reach JSON or become a browser's latest snapshot.
        numeric_keys = ("t", "gain", "eta", "nu", "thrust", "command", "rover_q",
                        "force_world", "net_lift", "net_lift_g", "altitude", "cv_altitude", "desired_altitude", "realtime_factor")
        values = [state[key] for key in numeric_keys]
        values += [list(state["params"].values()), b.thrust_cmd, b._draught,
                   self.keyboard.hold.e_int, self.autopilot.e_int]
        if not all(np.all(np.isfinite(value)) for value in values):
            raise FloatingPointError("Non-finite simulation state")
        return state

    def _publish(self):
        state = self._make_state()
        self.seq = state["seq"]
        with self._lock:
            self._snapshot = state

    def snapshot(self):
        with self._lock:
            return copy.deepcopy(self._snapshot)

    def config(self):
        state = self.snapshot()
        positions, axes = self.P.blimp.thruster_geometry()
        return {
            "parameters": [{"key": key, "group": group, "label": PARAMETER_LABELS[name][0],
                            "min": low, "max": high, "value": state["params"][key], "unit": PARAMETER_LABELS[name][1],
                            **({"description": PARAMETER_DESCRIPTIONS[name]} if name in PARAMETER_DESCRIPTIONS else {})}
                           for key, (group, name, low, high) in PARAMETER_BOUNDS.items()],
            "dt_phys": self.P.dt_phys, "dt_ctrl": self.P.dt_ctrl,
            "geometry": {"r_env": self.P.blimp.r_env, "h_env": self.P.blimp.h_env,
                         "d_VM": self.P.blimp.d_VM, "d_VT": self.P.blimp.d_VT,
                         "gondola_size": self.P.blimp.gondola_size.tolist(),
                         "thruster_length": self.P.blimp.thruster_length, "thruster_radius": self.P.blimp.thruster_radius,
                         "thruster_positions": positions.tolist(), "thruster_axes": axes.tolist(), "rover_length": self.P.rover.body_len},
            "arena": self.P.arena, "seed": self.P.seed, "n_rovers": len(self.sim.rovers),
            "camera": state["camera"]["profile"], "camera_intrinsics_pixels": state["camera"]["K_pixels"],
        }

    def submit(self, message):
        return self._enqueue(validate_command(message))

    def disconnect(self):
        return self._enqueue({"action": "disconnect"})

    def export(self):
        return self._enqueue({"action": "export"})

    def export_camera(self):
        return self._enqueue({"action": "camera_export"})

    def _export_camera(self):
        if self.camera_recording is None:
            raise ValueError("No camera recording is available")
        if self.camera_recording.active:
            raise ValueError("Stop camera recording before downloading it")
        return self._bounded_export(json.dumps(self.camera_recording.data, allow_nan=False, separators=(",", ":")).encode("utf-8"))

    def _bounded_export(self, data):
        if self.max_export_bytes is not None and len(data) > self.max_export_bytes:
            raise OverflowError("Export exceeds the server download size limit")
        return data

    def _enqueue(self, command):
        future = Future()
        if self._thread is None or not self._thread.is_alive() or self._stop.is_set():
            future.set_exception(RuntimeError("Simulation is not running"))
            return future
        try:
            self._commands.put_nowait((command, future))
        except queue.Full:
            future.set_exception(RuntimeError("Too many pending commands; try again"))
        self._wake.set()
        return future

    def _export_npz(self):
        # Match TeamSim.save_npz, giving zero/one-sample logs useful empty shapes.
        log = self.sim.logs()
        n = len(self.sim.rovers)
        widths = {"t": (), "blimp_nu": (6,), "blimp_eta": (6,), "blimp_u": (6,),
                  "blimp_thrust": (6,), "blimp_lift": (), "blimp_bottom_altitude": (),
                  "rover_q": (n, 3), "rover_u": (n, 2), "rover_ref": (n, 2)}
        for key, shape in widths.items():
            log[key] = log[key].reshape((-1,) + shape)
        output = io.BytesIO()
        np.savez(output, t=log["t"][:-1], X=log["blimp_nu"][:-1], U=log["blimp_u"][:-1], X_next=log["blimp_nu"][1:],
                 Eta=log["blimp_eta"][:-1], Eta_next=log["blimp_eta"][1:], rover_X=log["rover_q"][:-1],
                 rover_U=log["rover_u"][:-1], rover_X_next=log["rover_q"][1:], rover_ref=log["rover_ref"][:-1],
                 blimp_thrust=log["blimp_thrust"][:-1], blimp_lift=log["blimp_lift"][:-1],
                 blimp_bottom_altitude=log["blimp_bottom_altitude"][:-1], blimp_bottom_altitude_next=log["blimp_bottom_altitude"][1:],
                 eta_reference="CV_NED", altitude_reference="gondola_bottom", gondola_size=self.P.blimp.gondola_size,
                 thruster_length=self.P.blimp.thruster_length, thruster_radius=self.P.blimp.thruster_radius, d_VT=self.P.blimp.d_VT,
                 dt=self.P.dt_ctrl, n_rovers=n, seed=self.P.seed)
        return self._bounded_export(output.getvalue())

    def start(self):
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="blimp-physics", daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._wake.set()
        if self._thread is not None:
            self._thread.join(timeout=5)
        while not self._commands.empty():
            _, future = self._commands.get_nowait()
            if not future.done():
                future.set_exception(RuntimeError("Simulation stopped"))

    def _run(self):
        next_step = time.monotonic() + self.P.dt_ctrl
        self._reset_realtime_factor()
        while not self._stop.is_set():
            self._wake.clear()
            now = time.monotonic()
            self._expire_keys(now)
            for _ in range(64):
                try:
                    command, future = self._commands.get_nowait()
                except queue.Empty:
                    break
                if not future.set_running_or_notify_cancel():
                    continue
                try:
                    if command["action"] == "export":
                        result = self._export_npz()
                    elif command["action"] == "camera_export":
                        result = self._export_camera()
                    else:
                        self._apply(command, time.monotonic())
                        self._publish()
                        result = None
                    future.set_result(result)
                except Exception as exc:
                    future.set_exception(exc)
            now = time.monotonic()
            if self.paused:
                next_step = now + self.P.dt_ctrl
                self._reset_realtime_factor()
            elif now >= next_step:
                try:
                    self._step()
                    self._record_step_timing(time.monotonic())
                    self._make_state()
                except Exception:
                    # A step changes clocks, rovers, RNG, actuator/disturbance
                    # states, controller memory and logs. Reset them together
                    # instead of publishing a partially rolled-back session.
                    self._reset(recording_reason="numerical_error")
                    self.error = "Simulation reset and paused after a numerical error; session log cleared. Press Reset to continue."
                # Preserve the fixed step; avoid replaying an unbounded backlog.
                next_step += self.P.dt_ctrl
                if next_step < time.monotonic():
                    next_step = time.monotonic()
            self._publish()
            self._wake.wait(max(0.001, min(0.05, next_step - time.monotonic())))
