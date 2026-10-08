"""Camera geometry, calibration validation, and exact recording contracts."""
import copy
import json
import time
import unittest
from unittest.mock import patch

import numpy as np
from fastapi.testclient import TestClient

from sim.blimp import Blimp, R_zyx
from sim.camera import camera_intrinsics, camera_pose, default_camera_profile, validate_camera_profile
from sim.params import BlimpParams
from sim.rover import Rover
from sim.web_runtime import SimulationRuntime, validate_command
from web_server import create_app


def command(action, **values):
    return {"type": "command", "id": "camera-test", "action": action, **values}


def apply(runtime, action, **values):
    runtime._apply(validate_command(command(action, **values)), time.monotonic())
    runtime._publish()


class CameraProfileTests(unittest.TestCase):
    def test_default_pose_and_calibrated_pixel_projection(self):
        runtime = SimulationRuntime()
        p = validate_camera_profile(default_camera_profile(runtime.P.blimp))
        pose = camera_pose(np.array([0, 0, -1, 0, 0, 0]), p)
        self.assertEqual(p["calibration_status"], "estimated")
        self.assertAlmostEqual(pose["position_ned_m"][2], -0.688)
        rotation = np.asarray(pose["rotation_optical_to_ned"])
        np.testing.assert_allclose(rotation[:, 2], [0, 0, 1])
        np.testing.assert_allclose(-rotation[:, 1], [1, 0, 0])  # Image top = forward.
        ground_point = np.array([0.2, 0.1, 0])
        optical = rotation.T @ (ground_point - pose["position_ned_m"])
        K = np.asarray(camera_intrinsics(p))
        pixel = (K @ optical)[:2] / optical[2]
        self.assertGreater(pixel[0], K[0, 2])  # Body right is image right.
        self.assertLess(pixel[1], K[1, 2])
        angles = [0.1, -0.2, 0.5]
        tilted = camera_pose(np.r_[1, 2, -3, angles], p)
        np.testing.assert_allclose(tilted["rotation_optical_to_ned"], R_zyx(*angles) @ rotation)
        np.testing.assert_allclose(tilted["position_ned_m"], [1, 2, -3] + R_zyx(*angles) @ np.array(p["translation_body_m"]))

    def test_resolution_scaling_preserves_rays(self):
        p = default_camera_profile(BlimpParams())
        old_K = np.array(p["K"])
        p.update(width=320, height=240)
        K = np.array(camera_intrinsics(validate_camera_profile(p)))
        for old_pixel in ([0, 0], [319.5, 239.5], [639, 479]):
            old_pixel = np.array(old_pixel)
            new_pixel = (old_pixel + 0.5) * 0.5 - 0.5
            old_ray = np.linalg.inv(old_K) @ np.r_[old_pixel, 1]
            new_ray = np.linalg.inv(K) @ np.r_[new_pixel, 1]
            np.testing.assert_allclose(new_ray, old_ray)

    def test_invalid_calibration_is_rejected(self):
        cases = [("width", 641), ("fps", 12), ("reference_height", 500), ("sensor", "OV5640"),
                 ("K", [[100, 1, 0], [0, 100, 0], [0, 0, 1]]), ("D", [0, 0]),
                 ("D", [-5, 0, 0, 0, 0]), ("D", [float("nan"), 0, 0, 0, 0]),
                 ("translation_body_m", [0, 0, True]), ("translation_body_m", [0, 0, 4]),
                 ("rotation_optical_to_body", [[1, 0, 0], [0, 1, 0], [0, 0, -1]]),
                 ("near_m", float("nan")), ("notes", 7)]
        for key, value in cases:
            p = default_camera_profile(BlimpParams())
            p[key] = value
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                validate_camera_profile(p)
        p = default_camera_profile(BlimpParams())
        p["extra_field"] = True
        with self.assertRaises(ValueError):
            validate_camera_profile(p)


class CameraRecordingTests(unittest.TestCase):
    def test_all_action_transitions_and_final_partial_frame_replay(self):
        runtime = SimulationRuntime(n=2)
        runtime.sim.blimp.reset(runtime.sim.blimp.pose_at_altitude(1))
        apply(runtime, "pid", value=False)
        apply(runtime, "pause", value=False)
        apply(runtime, "camera_record_start")
        for index in range(5):
            apply(runtime, "keys", keys=["q"] if index < 3 else ["e", "r"])
            runtime._step()
            runtime._publish()
        apply(runtime, "camera_record_stop")
        data = json.loads(runtime._export_camera())
        self.assertEqual([f["control_step"] for f in data["frames"]], [0, 2, 4, 5])
        self.assertEqual(len(data["transitions"]), 5)
        self.assertEqual(runtime.snapshot()["camera_recording"]["status"], "ready")
        self.assertGreater(runtime.seq, runtime.sim.k)  # Wire seq is never used as time.
        for index, tr in enumerate(data["transitions"]):
            np.testing.assert_array_equal(tr["blimp_U"], runtime.sim.log["blimp_u"][index])
            np.testing.assert_array_equal(tr["rover_U_requested"], runtime.sim.log["rover_u"][index])
            np.testing.assert_array_equal(tr["rover_U"], np.clip(runtime.sim.log["rover_u"][index],
                                          [-runtime.P.rover.v_max, -runtime.P.rover.w_max], [runtime.P.rover.v_max, runtime.P.rover.w_max]))
            self.assertAlmostEqual(tr["t_next"] - tr["t"], 0.05)
            self.assertEqual(tr["pre"]["control_step"], index)
            self.assertEqual(tr["post"]["control_step"], index + 1)
            if index:
                self.assertEqual(tr["pre"], data["transitions"][index - 1]["post"])
            blimp = Blimp(copy.deepcopy(runtime.P.blimp))
            blimp.reset(eta=np.array(tr["pre"]["eta"]), nu=np.array(tr["pre"]["nu"]))
            blimp.thrust = np.array(tr["pre"]["thrust"])
            blimp.command(tr["blimp_U"])
            rovers = [Rover(runtime.P.rover, q) for q in tr["pre"]["rover_q"]]
            for rover, action in zip(rovers, tr["rover_U"]):
                rover.command(*action)
            for _ in range(5):
                blimp.step(0.01)
                for rover in rovers:
                    rover.step(0.01)
            np.testing.assert_allclose(blimp.eta, tr["post"]["eta"], atol=1e-14, rtol=0)
            np.testing.assert_allclose(blimp.nu, tr["post"]["nu"], atol=1e-14, rtol=0)
            np.testing.assert_allclose(blimp.thrust, tr["post"]["thrust"], atol=1e-14, rtol=0)
            np.testing.assert_allclose([r.q for r in rovers], tr["post"]["rover_q"], atol=1e-14, rtol=0)

    def test_paused_start_profile_lock_parameter_provenance_and_no_overwrite(self):
        runtime = SimulationRuntime()
        apply(runtime, "camera_record_start")
        self.assertEqual(runtime.snapshot()["camera_recording"]["frames"], 1)
        self.assertTrue(runtime.paused)
        with self.assertRaises(ValueError):
            apply(runtime, "camera_config", value=runtime.camera_profile)
        with self.assertRaises(ValueError):
            runtime._export_camera()
        apply(runtime, "param", key="blimp.net_lift_g", value=2)
        runtime._step()
        apply(runtime, "camera_record_stop")
        saved = json.loads(runtime._export_camera())
        self.assertAlmostEqual(saved["transitions"][0]["parameters"]["blimp"]["net_lift_N"], 0.01962)
        with self.assertRaises(ValueError):
            apply(runtime, "camera_record_start")
        new_profile = copy.deepcopy(runtime.camera_profile)
        new_profile.update(width=320, height=240)
        apply(runtime, "camera_config", value=new_profile)
        self.assertEqual(json.loads(runtime._export_camera()), saved)
        apply(runtime, "camera_record_clear")
        self.assertEqual(runtime.snapshot()["camera_recording"]["status"], "empty")
        apply(runtime, "camera_record_start")
        with self.assertRaises(ValueError):
            apply(runtime, "camera_record_clear")

    def test_reset_disconnect_and_error_preserve_only_successful_episode(self):
        for reason in ("reset", "disconnect", "numerical_error"):
            with self.subTest(reason=reason):
                runtime = SimulationRuntime()
                apply(runtime, "camera_record_start")
                runtime._step()
                if reason == "disconnect":
                    runtime._apply({"action": "disconnect"}, time.monotonic())
                elif reason == "reset":
                    apply(runtime, "reset")
                else:
                    original = runtime.sim.step

                    def corrupt_step():
                        original()
                        runtime.sim.blimp.eta[0] = float("nan")

                    with patch.object(runtime.sim, "step", side_effect=corrupt_step):
                        with self.assertRaises(FloatingPointError):
                            runtime._step()
                    runtime._reset(recording_reason="numerical_error")
                data = json.loads(runtime._export_camera())
                self.assertEqual(data["reason"], reason)
                self.assertEqual(data["generation"], 1)
                if reason != "disconnect":
                    self.assertGreater(runtime.generation, data["generation"])
                self.assertEqual(len(data["transitions"]), 1)
                self.assertEqual([f["control_step"] for f in data["frames"]], [0, 1])
                runtime._step()
                self.assertEqual(runtime._export_camera(), json.dumps(data, allow_nan=False, separators=(",", ":")).encode())

    def test_applied_rover_action_is_saturated_request(self):
        runtime = SimulationRuntime(n=1)
        runtime.sim.rovers[0].reset([-100, -100, 0])
        apply(runtime, "camera_record_start")
        runtime._step()
        apply(runtime, "camera_record_stop")
        tr = json.loads(runtime._export_camera())["transitions"][0]
        self.assertGreater(abs(tr["rover_U_requested"][0][0]), runtime.P.rover.v_max)
        self.assertGreater(abs(tr["rover_U_requested"][0][1]), runtime.P.rover.w_max)
        self.assertEqual(tr["rover_U"], [[runtime.P.rover.v_max, runtime.P.rover.w_max]])
        rover = Rover(runtime.P.rover, tr["pre"]["rover_q"][0])
        rover.command(*tr["rover_U"][0])
        for _ in range(5):
            rover.step(runtime.P.dt_phys)
        np.testing.assert_array_equal(rover.q, tr["post"]["rover_q"][0])

    def test_worker_pauses_do_not_duplicate_frames_and_failure_keeps_last_valid_step(self):
        runtime = SimulationRuntime()
        runtime.start()
        try:
            runtime.submit(command("camera_record_start")).result(timeout=1)
            time.sleep(0.12)
            self.assertEqual(runtime.snapshot()["camera_recording"]["frames"], 1)
            self.assertEqual(runtime.snapshot()["camera_recording"]["transitions"], 0)
            successful_step = runtime.sim.step
            steps = 0

            def fail_second_step():
                nonlocal steps
                successful_step()
                steps += 1
                if steps == 2:
                    runtime.sim.blimp.eta[0] = float("nan")

            with patch.object(runtime.sim, "step", side_effect=fail_second_step):
                runtime.submit(command("pause", value=False)).result(timeout=1)
                deadline = time.monotonic() + 2
                while runtime.snapshot()["error"] is None and time.monotonic() < deadline:
                    time.sleep(0.01)
            self.assertIsNotNone(runtime.snapshot()["error"])
            data = json.loads(runtime.export_camera().result(timeout=1))
            self.assertEqual(data["reason"], "numerical_error")
            self.assertEqual(len(data["transitions"]), 1)
            self.assertEqual([f["control_step"] for f in data["frames"]], [0, 1])
            self.assertEqual(runtime.snapshot()["control_step"], 0)
            json.dumps(data, allow_nan=False)
        finally:
            runtime.stop()

    def test_duration_cap_and_configured_frame_rates(self):
        for fps in (5, 10, 20):
            runtime = SimulationRuntime(n=1)
            p = copy.deepcopy(runtime.camera_profile)
            p["fps"] = fps
            apply(runtime, "camera_config", value=p)
            apply(runtime, "camera_record_start")
            for _ in range(601):
                runtime._step()
            data = json.loads(runtime._export_camera())
            self.assertEqual(data["reason"], "duration_limit")
            self.assertEqual(len(data["transitions"]), 600)
            self.assertEqual(len(data["frames"]), 30 * fps + 1)
            self.assertAlmostEqual(data["frames"][-1]["t"], 30)

    def test_http_export_serialized_on_worker(self):
        runtime = SimulationRuntime()
        with TestClient(create_app(runtime), base_url="http://127.0.0.1") as client:
            self.assertEqual(client.get("/api/camera/recording").status_code, 409)
            runtime.submit(command("camera_record_start")).result(timeout=1)
            self.assertEqual(client.get("/api/camera/recording").status_code, 409)
            runtime.submit(command("camera_record_stop")).result(timeout=1)
            response = client.get("/api/camera/recording")
            self.assertEqual(response.status_code, 200)
            self.assertEqual(response.json()["schema_version"], "xiao-recording-v1")
            self.assertEqual(len(response.json()["frames"]), 1)
            self.assertEqual(response.json()["transitions"], [])
            self.assertEqual(response.headers["cache-control"], "no-store")


if __name__ == "__main__":
    unittest.main()
