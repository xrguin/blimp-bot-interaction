"""Calibratable pinhole camera geometry and exact simulation-clock recording.

Optical axes follow OpenCV: x right, y down, z forward. Intrinsics refer to
pixel centres at integer coordinates. No measured OV2640 calibration is implied.
"""
from __future__ import annotations

import copy
import math

import numpy as np

from .blimp import R_zyx


RESOLUTIONS = {(320, 240), (640, 480), (800, 600), (1600, 1200)}


def default_camera_profile(blimp_parameters):
    focal = 640 / (2 * math.tan(math.radians(60) / 2))
    return {
        "schema_version": "xiao-camera-v1", "sensor": "OV2640",
        "calibration_status": "estimated", "width": 640, "height": 480, "fps": 10,
        "reference_width": 640, "reference_height": 480,
        "K": [[focal, 0.0, 319.5], [0.0, focal, 239.5], [0.0, 0.0, 1.0]],
        "D": [0.0] * 5,
        "translation_body_m": [0.0, 0.0, blimp_parameters.d_VT + blimp_parameters.gondola_size[2] / 2 + 0.002],
        "rotation_optical_to_body": [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
        "near_m": 0.001, "far_m": 50.0,
        "notes": "Uncalibrated placeholder: 60 degree horizontal field of view, zero distortion. Optical centre 2 mm below the gondola box; image top is body forward. Replace with measured intrinsics and mounting pose.",
    }


def _array(value, shape, name, limit=1e6):
    try:
        raw = np.asarray(value)
        if raw.shape != shape or raw.dtype.kind not in "iuf" or any(isinstance(v, (bool, np.bool_)) for v in np.asarray(value, dtype=object).flat):
            raise ValueError
        result = raw.astype(float)
        if not np.all(np.isfinite(result)) or np.any(np.abs(result) > limit):
            raise ValueError
        return result
    except (ValueError, TypeError, OverflowError):
        raise ValueError(f"Camera {name} must contain finite numbers with shape {shape}") from None


def _integer(value, low, high, name):
    if isinstance(value, bool) or not isinstance(value, int) or not low <= value <= high:
        raise ValueError(f"Camera {name} must be an integer between {low} and {high}")
    return value


def _number(value, low, high, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not low <= value <= high:
        raise ValueError(f"Camera {name} must be between {low} and {high}")
    return float(value)


def validate_camera_profile(profile):
    if not isinstance(profile, dict):
        raise ValueError("Camera profile must be a JSON object")
    required = {"schema_version", "sensor", "calibration_status", "width", "height", "fps",
                "reference_width", "reference_height", "K", "D", "translation_body_m",
                "rotation_optical_to_body", "near_m", "far_m"}
    if not required <= profile.keys() or profile.keys() - required - {"notes"}:
        raise ValueError("Camera profile has missing or unsupported fields")
    p = copy.deepcopy(profile)
    if p["schema_version"] != "xiao-camera-v1" or p["sensor"] != "OV2640":
        raise ValueError("Expected an xiao-camera-v1 OV2640 profile")
    if p["calibration_status"] not in ("estimated", "calibrated"):
        raise ValueError("Camera calibration_status must be estimated or calibrated")
    width = _integer(p["width"], 1, 1600, "width")
    height = _integer(p["height"], 1, 1200, "height")
    if (width, height) not in RESOLUTIONS:
        raise ValueError("Choose 320x240, 640x480, 800x600, or 1600x1200")
    if _integer(p["fps"], 5, 20, "fps") not in (5, 10, 20):
        raise ValueError("Choose 5, 10, or 20 simulated frames per second")
    rw = _integer(p["reference_width"], 16, 4096, "reference_width")
    rh = _integer(p["reference_height"], 16, 4096, "reference_height")
    if width * rh != height * rw:
        raise ValueError("Camera calibration and output must have the same aspect ratio; cropped modes need their own calibration")
    K = _array(p["K"], (3, 3), "K")
    if not np.allclose(K[2], [0, 0, 1], rtol=0, atol=1e-12) or K[0, 1] != 0 or K[1, 0] != 0:
        raise ValueError("Camera K must have zero skew and final row [0, 0, 1]")
    if not 0.1 * rw <= K[0, 0] <= 20 * rw or not 0.1 * rh <= K[1, 1] <= 20 * rh:
        raise ValueError("Camera focal lengths are outside supported bounds")
    if not -0.5 <= K[0, 2] <= rw - 0.5 or not -0.5 <= K[1, 2] <= rh - 0.5:
        raise ValueError("Camera principal point must be inside the reference image")
    D = _array(p["D"], (5,), "D", limit=5)
    translation = _array(p["translation_body_m"], (3,), "translation_body_m", limit=2)
    rotation = _array(p["rotation_optical_to_body"], (3, 3), "rotation_optical_to_body", limit=1.001)
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6, rtol=0) or not np.isclose(np.linalg.det(rotation), 1, atol=1e-6):
        raise ValueError("Camera rotation must be an orthonormal, right-handed rotation matrix")
    p["near_m"] = _number(p["near_m"], 0.0001, 0.1, "near_m")
    p["far_m"] = _number(p["far_m"], 0.2, 1000, "far_m")
    notes = p.get("notes", "")
    if not isinstance(notes, str) or len(notes) > 2000:
        raise ValueError("Camera notes must be text of at most 2000 characters")
    # Reject clearly folded Brown-Conrady maps over the sensor field. This
    # sampled check is a supported-calibration guard, not calibration evidence.
    x, y = np.meshgrid(np.linspace((-0.5 - K[0, 2]) / K[0, 0], (rw - 0.5 - K[0, 2]) / K[0, 0], 33),
                       np.linspace((-0.5 - K[1, 2]) / K[1, 1], (rh - 0.5 - K[1, 2]) / K[1, 1], 25))
    k1, k2, p1, p2, k3 = D
    r2 = x * x + y * y
    radial = 1 + k1 * r2 + k2 * r2**2 + k3 * r2**3
    derivative = k1 + 2 * k2 * r2 + 3 * k3 * r2**2
    a = radial + 2 * x * x * derivative + 2 * p1 * y + 6 * p2 * x
    b = 2 * x * y * derivative + 2 * p1 * x + 2 * p2 * y
    d = radial + 2 * y * y * derivative + 6 * p1 * y + 2 * p2 * x
    if np.any(a <= 0.05) or np.any(d <= 0.05) or np.any(a * d - b * b <= 0.01):
        raise ValueError("Camera distortion folds or collapses the image; use a valid pinhole calibration")
    p.update(K=K.tolist(), D=D.tolist(), translation_body_m=translation.tolist(),
             rotation_optical_to_body=rotation.tolist(), notes=notes)
    return p


def camera_intrinsics(profile):
    """Scale around pixel edges, retaining the pixel-centre convention."""
    K = np.array(profile["K"], dtype=float)
    scale = profile["width"] / profile["reference_width"]
    K[0, 0] *= scale
    K[1, 1] *= scale
    K[0, 2] = scale * (K[0, 2] + 0.5) - 0.5
    K[1, 2] = scale * (K[1, 2] + 0.5) - 0.5
    return K.tolist()


def camera_pose(eta, profile):
    body_to_ned = R_zyx(*eta[3:])
    return {"position_ned_m": (np.asarray(eta[:3]) + body_to_ned @ np.asarray(profile["translation_body_m"])).tolist(),
            "rotation_optical_to_ned": (body_to_ned @ np.asarray(profile["rotation_optical_to_body"])).tolist()}


class CameraRecording:
    """Keeps all control transitions, sampling frames only at exact tick boundaries."""
    MAX_SECONDS = 30

    def __init__(self, metadata, state):
        self.data = copy.deepcopy(metadata)
        self.data.update(schema_version="xiao-recording-v1", frames=[], transitions=[], reason=None)
        ratio = 1 / (metadata["profile"]["fps"] * metadata["dt_ctrl"])
        if not math.isclose(ratio, round(ratio), abs_tol=1e-9) or ratio < 1:
            raise ValueError("Camera fps must evenly divide the control clock")
        self.stride = round(ratio)
        self.max_steps = round(self.MAX_SECONDS / metadata["dt_ctrl"])
        self.active = True
        self.initial_step = state["control_step"]
        self.last_state = copy.deepcopy(state)
        self._frame(state)

    def _frame(self, state):
        self.data["frames"].append({"frame_index": len(self.data["frames"]), **copy.deepcopy(state)})

    def append(self, pre, post, blimp_command, rover_command, parameters, context,
               rover_command_requested, thrust_target):
        if not self.active:
            return
        self.data["transitions"].append({"step": pre["control_step"], "t": pre["t"], "t_next": post["t"],
                                          "pre": copy.deepcopy(pre), "post": copy.deepcopy(post),
                                          "blimp_U": np.asarray(blimp_command).tolist(), "rover_U": np.asarray(rover_command).tolist(),
                                          "rover_U_requested": np.asarray(rover_command_requested).tolist(),
                                          "thrust_target_N": np.asarray(thrust_target).tolist(),
                                          "parameters": copy.deepcopy(parameters), "context": copy.deepcopy(context)})
        self.last_state = copy.deepcopy(post)
        if (post["control_step"] - self.initial_step) % self.stride == 0:
            self._frame(post)
        if len(self.data["transitions"]) >= self.max_steps:
            self.finish("duration_limit")

    def finish(self, reason):
        if not self.active:
            return
        # Even an early stop retains the final partial camera interval. The
        # timestamp is authoritative; do not relabel this frame as a full period.
        if self.data["frames"][-1]["control_step"] != self.last_state["control_step"]:
            self._frame(self.last_state)
        self.data["reason"] = reason
        self.active = False

    def status(self):
        return {"status": "recording" if self.active else "ready", "frames": len(self.data["frames"]),
                "transitions": len(self.data["transitions"]),
                "seconds": self.last_state["t"] - self.data["frames"][0]["t"],
                "reason": self.data["reason"], "max_seconds": self.MAX_SECONDS}


def empty_recording_status():
    return {"status": "empty", "frames": 0, "transitions": 0, "seconds": 0.0,
            "reason": None, "max_seconds": CameraRecording.MAX_SECONDS}
