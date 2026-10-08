"""Browser-backend regression checks: python -m unittest sim.tests.test_web."""
import io
import json
import time
import unittest
from unittest.mock import patch

import numpy as np
from fastapi.testclient import TestClient
from starlette.websockets import WebSocketDisconnect

from sim.controllers import BlimpPD
from sim.keyboard import KeyboardBlimpController
from sim.params import G, SimParams, SLIDERS
from sim.sim import TeamSim
from sim.web_runtime import SimulationRuntime, validate_command
from web_server import create_app


def command(action, **values):
    return {"type": "command", "id": "test", "action": action, **values}


def apply(runtime, action, **values):
    runtime._apply(validate_command(command(action, **values)), time.monotonic())
    runtime._publish()


def receive_kind(ws, kind):
    for _ in range(100):
        message = ws.receive_json()
        if message["type"] == kind:
            return message
    raise AssertionError(f"No {kind} response")


class ValidationTests(unittest.TestCase):
    def test_parameter_metadata_and_bounds(self):
        runtime = SimulationRuntime()
        metadata = runtime.config()["parameters"]
        self.assertEqual(len(metadata), len(SLIDERS))
        for parameter in metadata:
            for value in (parameter["min"], parameter["max"]):
                valid = validate_command(command("param", key=parameter["key"], value=value))
                self.assertEqual(valid["value"], value)
            for value in (parameter["min"] - 1, parameter["max"] + 1, 10 ** 1000, float("nan"), float("inf"), "0.1", True):
                with self.assertRaises(ValueError):
                    validate_command(command("param", key=parameter["key"], value=value))

    def test_malformed_messages(self):
        messages = [None, [], "bad", {}, {"type": "command", "id": "", "action": "reset"},
                    command("param", key="blimp.__dict__", value=0), command("param", key=[], value=0),
                    command("keys", keys="w"), command("keys", keys=["escape"]), command("keys", keys=[None]),
                    command("pause", value=1), command("pid", value="true"), command("mode", value="oops"),
                    command("gain", value=0), command("unknown")]
        for message in messages:
            with self.subTest(message=message), self.assertRaises(ValueError):
                validate_command(message)

    def test_immutable_snapshot(self):
        runtime = SimulationRuntime()
        state = runtime.snapshot()
        state["eta"][0] = 100
        state["params"]["blimp.T_max"] = 200
        self.assertEqual(runtime.snapshot()["eta"][0], 0)
        self.assertEqual(runtime.snapshot()["params"]["blimp.T_max"], 0.03)


class RuntimeTests(unittest.TestCase):
    def test_realtime_factor_measures_elapsed_time_and_clears_pause_history(self):
        runtime = SimulationRuntime()
        apply(runtime, "pause", value=False)
        completed_at = 100.0
        runtime._record_step_timing(completed_at)
        self.assertEqual(runtime.realtime_factor, 0.0)  # First completion anchors the clock.
        for interval in [0.01, 0.09] * 10:
            completed_at += interval
            runtime._record_step_timing(completed_at)
        # 20 control intervals advance one simulated second in one wall second.
        # Averaging per-interval speed would incorrectly report about 2.78x.
        self.assertAlmostEqual(runtime.realtime_factor, 1.0)
        for _ in range(20):
            completed_at += 0.1
            runtime._record_step_timing(completed_at)
        self.assertAlmostEqual(runtime.realtime_factor, 0.5)
        for stop_action in ("pause", "disconnect", "reset"):
            if stop_action == "disconnect":
                runtime._apply({"action": "disconnect"}, completed_at)
            else:
                apply(runtime, stop_action, **({"value": True} if stop_action == "pause" else {}))
            self.assertEqual(runtime.realtime_factor, 0.0)
            self.assertEqual(len(runtime._realtime_intervals), 0)
            apply(runtime, "pause", value=False)
            completed_at += 1000  # Paused time must not enter the resumed reading.
            runtime._record_step_timing(completed_at)
            self.assertEqual(runtime.realtime_factor, 0.0)
            completed_at += 0.05
            runtime._record_step_timing(completed_at)
            self.assertAlmostEqual(runtime.realtime_factor, 1.0)

    def test_net_lift_grams_endpoints_metadata_and_unchanged_mass(self):
        runtime = SimulationRuntime()
        metadata = {entry["key"]: entry for entry in runtime.config()["parameters"]}
        setting = metadata["blimp.net_lift_g"]
        self.assertNotIn("blimp.net_lift_N", metadata)
        self.assertEqual((setting["min"], setting["max"]), (-10.0, 10.0))
        self.assertEqual(setting["unit"], "g equiv.")
        self.assertEqual(setting["label"], "Net lift")
        self.assertIn("Equivalent weight only", setting["description"])
        original_mass = runtime.P.blimp.m
        original_inertia = (runtime.P.blimp.I_CM_xy, runtime.P.blimp.I_z)
        original_matrix = runtime.sim.blimp.M.copy()
        with patch.object(runtime.sim.blimp, "rebuild", wraps=runtime.sim.blimp.rebuild) as rebuild:
            for grams in (-10.0, 0.0, 10.0):
                apply(runtime, "param", key="blimp.net_lift_g", value=grams)
                self.assertAlmostEqual(runtime.P.blimp.net_lift_N, grams * G / 1000)
                self.assertAlmostEqual(runtime.P.blimp.net_lift_g, grams)
                self.assertAlmostEqual(runtime.sim.blimp.lift, grams * G / 1000)
                self.assertAlmostEqual(runtime.sim.blimp.restoring(np.zeros(6))[2], grams * G / 1000)
                self.assertAlmostEqual(runtime.snapshot()["params"]["blimp.net_lift_g"], grams)
                self.assertAlmostEqual(runtime.snapshot()["net_lift_g"], grams)
            self.assertEqual(rebuild.call_count, 0)
        for invalid in (-10.01, 10.01):
            with self.assertRaises(ValueError):
                validate_command(command("param", key="blimp.net_lift_g", value=invalid))
        with self.assertRaises(ValueError):
            validate_command(command("param", key="blimp.net_lift_N", value=0.01))
        self.assertEqual(runtime.P.blimp.m, original_mass)
        self.assertEqual((runtime.P.blimp.I_CM_xy, runtime.P.blimp.I_z), original_inertia)
        np.testing.assert_array_equal(runtime.sim.blimp.M, original_matrix)

    def test_net_lift_grams_drift_reset_and_newton_exports(self):
        runtime = SimulationRuntime(seed=7)
        apply(runtime, "param", key="blimp.net_lift_g", value=-4.5)
        apply(runtime, "param", key="blimp.net_lift_drift", value=0.001)
        for _ in range(4):
            runtime.sim.step()
        runtime._publish()
        state = runtime.snapshot()
        self.assertAlmostEqual(state["params"]["blimp.net_lift_g"], -4.5)
        self.assertNotAlmostEqual(state["net_lift_g"], -4.5)
        self.assertAlmostEqual(state["net_lift"], runtime.sim.blimp.lift)
        self.assertAlmostEqual(state["net_lift_g"], state["net_lift"] * 1000 / G)
        with np.load(io.BytesIO(runtime._export_npz())) as data:
            np.testing.assert_array_equal(data["blimp_lift"], runtime.sim.log["blimp_lift"][:-1])
            self.assertAlmostEqual(data["blimp_lift"][0], -4.5 * G / 1000)
        apply(runtime, "reset")
        self.assertAlmostEqual(runtime.P.blimp.net_lift_N, -4.5 * G / 1000)
        self.assertAlmostEqual(runtime.snapshot()["params"]["blimp.net_lift_g"], -4.5)
        self.assertAlmostEqual(runtime.snapshot()["net_lift_g"], -4.5)
        self.assertEqual(runtime.sim.blimp._lift_drift, 0.0)

    def test_ground_start_reset_and_zero_altitude_target(self):
        for mode in ("teleop", "auto"):
            with self.subTest(mode=mode):
                runtime = SimulationRuntime(mode=mode)
                initial = runtime.snapshot()
                self.assertEqual(initial["eta"][:2], [0.0, 0.0])
                self.assertAlmostEqual(initial["altitude"], 0.0)
                self.assertAlmostEqual(initial["cv_altitude"], 0.325)
                geometry = runtime.config()["geometry"]
                np.testing.assert_array_equal(geometry["gondola_size"], runtime.P.blimp.gondola_size)
                self.assertEqual(geometry["thruster_length"], runtime.P.blimp.thruster_length)
                self.assertEqual(geometry["thruster_radius"], runtime.P.blimp.thruster_radius)
                apply(runtime, "param", key="task.blimp_height", value=0.0)
                self.assertEqual(runtime.snapshot()["desired_altitude"], 0.0)
                runtime.sim.blimp.reset(runtime.sim.blimp.pose_at_altitude(1.2, x=1, y=-1, angles=[0.1, 0.2, 0]))
                runtime._publish()
                self.assertAlmostEqual(runtime.snapshot()["altitude"], 1.2)
                apply(runtime, "reset")
                self.assertEqual(runtime.snapshot()["eta"], initial["eta"])
                self.assertAlmostEqual(runtime.snapshot()["altitude"], 0.0)
                self.assertEqual(runtime.snapshot()["desired_altitude"], 0.0)
                json.dumps(runtime.snapshot(), allow_nan=False)

    def test_step_matches_existing_simulator(self):
        runtime = SimulationRuntime(seed=17)
        reference = TeamSim(SimParams(seed=17))
        keyboard = KeyboardBlimpController(reference.blimp, key_timeout=0, hold=BlimpPD(reference.P.task, dt=reference.P.dt_ctrl))
        reference.set_controllers(reference.rover_ctrl, keyboard)
        reference.reset()
        keyboard.on_pid_toggle(True)
        apply(runtime, "param", key="blimp.draught_N", value=0.02)
        reference.P.blimp.draught_N = 0.02
        apply(runtime, "pause", value=False)
        apply(runtime, "keys", keys=["w", "q"])
        keyboard.pressed = {"w", "q"}
        for _ in range(12):
            runtime.sim.step()
            reference.step()
        np.testing.assert_array_equal(runtime.sim.blimp.eta, reference.blimp.eta)
        np.testing.assert_array_equal(runtime.sim.blimp.nu, reference.blimp.nu)
        self.assertAlmostEqual(runtime.sim.t, 12 * 0.05)

    def test_altitude_hold_and_manual_override(self):
        runtime = SimulationRuntime()
        apply(runtime, "param", key="task.blimp_height", value=1.7)
        self.assertEqual(runtime.keyboard.desired_altitude, 1.7)
        xy = runtime.keyboard.hold.p_ref_override[:2].copy()
        apply(runtime, "pid", value=False)
        apply(runtime, "reset")
        self.assertEqual(runtime.keyboard.desired_altitude, 1.7)
        self.assertFalse(runtime.P.task.pid_enabled)
        apply(runtime, "pid", value=True)
        np.testing.assert_array_equal(xy, runtime.keyboard.hold.p_ref_override[:2])
        runtime.sim.step()
        self.assertLess(runtime.sim.blimp.nu[2], 0)  # NED: negative heave moves up.
        apply(runtime, "pause", value=False)
        apply(runtime, "keys", keys=["e"])
        runtime.sim.step()
        self.assertIsNone(runtime.keyboard._altitude_target)

    def test_key_expiry_pause_reset_and_modes(self):
        runtime = SimulationRuntime()
        apply(runtime, "keys", keys=["w"])
        self.assertFalse(runtime.keyboard.pressed)
        apply(runtime, "pause", value=False)
        apply(runtime, "keys", keys=["w"])
        self.assertTrue(runtime.keyboard.pressed)
        runtime._expire_keys(runtime._last_keys + 0.351)
        self.assertFalse(runtime.keyboard.pressed)
        for action, values in [("pause", {"value": True}), ("mode", {"value": "auto"}), ("reset", {}), ("stop", {})]:
            apply(runtime, "mode", value="teleop")
            apply(runtime, "pause", value=False)
            apply(runtime, "keys", keys=["w"])
            apply(runtime, action, **values)
            self.assertFalse(runtime.keyboard.pressed)
        self.assertEqual(runtime.sim.t, 0)

    def test_idle_rovers_remain_stationary(self):
        runtime = SimulationRuntime(rovers_mode="idle")
        before = np.array([r.q.copy() for r in runtime.sim.rovers])
        for _ in range(5):
            runtime.sim.step()
        np.testing.assert_array_equal(before, np.array([r.q for r in runtime.sim.rovers]))

    def test_matrix_rebuild_only_when_required(self):
        runtime = SimulationRuntime()
        with patch.object(runtime.sim.blimp, "rebuild", wraps=runtime.sim.blimp.rebuild) as rebuild:
            apply(runtime, "param", key="view.force_scale", value=80)
            apply(runtime, "param", key="task.kp_pos", value=0.04)
            apply(runtime, "param", key="blimp.d_lin", value=0.1)
            self.assertEqual(rebuild.call_count, 0)
            apply(runtime, "param", key="blimp.k_add_xy", value=0.4)
            self.assertEqual(rebuild.call_count, 1)
        old_authority = runtime.keyboard.authority.copy()
        apply(runtime, "param", key="blimp.T_max", value=0.06)
        np.testing.assert_allclose(runtime.keyboard.authority, 2 * old_authority)

    def test_empty_and_existing_export_compatibility(self):
        runtime = SimulationRuntime(n=2)
        with np.load(io.BytesIO(runtime._export_npz())) as data:
            self.assertEqual(data["X"].shape, (0, 6))
            self.assertEqual(data["rover_X"].shape, (0, 2, 3))
            self.assertEqual(data["rover_U"].shape, (0, 2, 2))
            self.assertEqual(data["blimp_bottom_altitude"].shape, (0,))
            self.assertEqual(data["blimp_bottom_altitude_next"].shape, (0,))
            self.assertEqual(str(data["eta_reference"]), "CV_NED")
            self.assertEqual(str(data["altitude_reference"]), "gondola_bottom")
        for _ in range(5):
            runtime.sim.step()
        legacy = io.BytesIO()
        runtime.sim.save_npz(legacy)
        legacy.seek(0)
        with np.load(legacy) as expected, np.load(io.BytesIO(runtime._export_npz())) as actual:
            self.assertEqual(set(expected.files), set(actual.files))
            for key in expected.files:
                np.testing.assert_array_equal(expected[key], actual[key])

    def test_worker_advances_without_browser_and_pauses_on_disconnect(self):
        runtime = SimulationRuntime()
        runtime.start()
        try:
            runtime.submit(command("pause", value=False)).result(timeout=1)
            deadline = time.monotonic() + 2
            while runtime.snapshot()["t"] < 0.1 and time.monotonic() < deadline:
                time.sleep(0.01)
            self.assertGreaterEqual(runtime.snapshot()["t"], 0.1)
            runtime.submit(command("keys", keys=["w"])).result(timeout=1)
            time.sleep(0.4)
            self.assertEqual(runtime.snapshot()["command"], [0.0] * 4)
            runtime.disconnect().result(timeout=1)
            state = runtime.snapshot()
            self.assertTrue(state["paused"])
            time.sleep(0.07)
            self.assertEqual(runtime.snapshot()["t"], state["t"])
        finally:
            runtime.stop()
        self.assertFalse(runtime._thread.is_alive())

    def test_failed_step_resets_complete_session_and_json_recovers(self):
        # Cover an exception during a partially mutated step and an apparently
        # successful step whose hidden lift/rover state contains NaN/Infinity.
        for raises in (True, False):
            with self.subTest(raises=raises):
                runtime = SimulationRuntime(seed=9)
                initial = runtime.snapshot()
                original_step = runtime.sim.step

                def failed_step():
                    original_step()
                    runtime.sim.blimp._lift_drift = float("nan")
                    runtime.sim.blimp.thrust_cmd[:] = float("inf")
                    runtime.sim.rovers[0].q[:] = float("nan")
                    runtime.sim.rovers[0].u[:] = float("inf")
                    runtime.keyboard.hold.e_int[:] = float("nan")
                    runtime.keyboard.hold.last_wrench[:] = float("nan")
                    runtime.sim.rng.random(5)
                    runtime.sim.log["blimp_lift"].append(float("nan"))
                    if raises:
                        raise FloatingPointError("injected partial-step failure")

                runtime.start()
                try:
                    with patch.object(runtime.sim, "step", side_effect=failed_step):
                        runtime.submit(command("pause", value=False)).result(timeout=1)
                        deadline = time.monotonic() + 2
                        while not runtime.snapshot()["error"] and time.monotonic() < deadline:
                            time.sleep(0.01)
                    state = runtime.snapshot()
                    self.assertTrue(state["paused"])
                    self.assertIn("session log cleared", state["error"])
                    self.assertGreater(state["generation"], initial["generation"])
                    self.assertEqual(state["t"], 0)
                    self.assertEqual(runtime.sim.k, 0)
                    json.dumps(state, allow_nan=False)
                    np.testing.assert_array_equal(state["eta"], initial["eta"])
                    np.testing.assert_array_equal(state["rover_q"], initial["rover_q"])
                    np.testing.assert_array_equal(runtime.sim.blimp.thrust_cmd, np.zeros(6))
                    np.testing.assert_array_equal(runtime.keyboard.hold.e_int, np.zeros(3))
                    np.testing.assert_array_equal(runtime.keyboard.hold.last_wrench, np.zeros(6))
                    self.assertTrue(all(not entries for entries in runtime.sim.log.values()))
                    expected_rng = np.random.default_rng(9).bit_generator.state
                    self.assertEqual(runtime.sim.rng.bit_generator.state, expected_rng)
                    with self.assertRaises(ValueError):
                        runtime.submit(command("pause", value=False)).result(timeout=1)
                    runtime.submit(command("reset")).result(timeout=1)
                    self.assertIsNone(runtime.snapshot()["error"])
                    runtime.submit(command("pause", value=False)).result(timeout=1)
                    deadline = time.monotonic() + 2
                    while runtime.snapshot()["t"] == 0 and time.monotonic() < deadline:
                        time.sleep(0.01)
                    self.assertGreater(runtime.snapshot()["t"], 0)
                    json.dumps(runtime.snapshot(), allow_nan=False)
                finally:
                    runtime.stop()


class WebTests(unittest.TestCase):
    def setUp(self):
        self.runtime = SimulationRuntime(n=2)
        self.client = TestClient(create_app(self.runtime), base_url="http://127.0.0.1")
        self.client.__enter__()

    def tearDown(self):
        self.client.__exit__(None, None, None)

    def connect(self, **kwargs):
        return self.client.websocket_connect("ws://127.0.0.1/ws", **kwargs)

    def test_http_config_export_and_host_restriction(self):
        self.assertEqual(self.client.get("/health").json(), {"status": "ok"})
        self.assertEqual(len(self.client.get("/api/config").json()["parameters"]), 22)
        response = self.client.get("/api/log.npz")
        self.assertEqual(response.status_code, 200)
        with np.load(io.BytesIO(response.content)) as data:
            self.assertEqual(data["n_rovers"], 2)
        self.assertEqual(self.client.get("/health", headers={"host": "evil.example"}).status_code, 400)

    def test_command_ack_state_and_rejected_input(self):
        with self.connect(headers={"origin": "http://127.0.0.1"}) as ws:
            self.assertTrue(receive_kind(ws, "hello")["has_control"])
            ws.send_json(command("param", key="task.blimp_height", value=1.5))
            self.assertTrue(receive_kind(ws, "ack")["ok"])
            self.assertEqual(self.runtime.snapshot()["desired_altitude"], 1.5)
            ws.send_json(command("param", key="task.blimp_height", value=50))
            self.assertIn("between", receive_kind(ws, "error")["message"])
            ws.send_text("not json")
            self.assertIsNone(receive_kind(ws, "error")["id"])
            ws.send_bytes(b"bad binary")
            self.assertIn("JSON text", receive_kind(ws, "error")["message"])
            ws.send_json(command("gain", value=0.7))
            receive_kind(ws, "ack")
            self.assertEqual(self.runtime.snapshot()["gain"], 0.7)

    def test_spectator_and_controller_disconnect(self):
        with self.connect() as owner:
            self.assertTrue(receive_kind(owner, "hello")["has_control"])
            with self.connect() as spectator:
                self.assertFalse(receive_kind(spectator, "hello")["has_control"])
                spectator.send_json(command("pause", value=False))
                self.assertIn("read-only", receive_kind(spectator, "error")["message"])
            owner.send_json(command("pause", value=False))
            receive_kind(owner, "ack")
            self.assertFalse(self.runtime.snapshot()["paused"])
        self.assertTrue(self.runtime.snapshot()["paused"])
        with self.connect() as next_owner:
            self.assertTrue(receive_kind(next_owner, "hello")["has_control"])

    def test_foreign_origin_is_rejected(self):
        for origin in ("https://example.com", "http://127.0.0.1:9999", "null"):
            with self.subTest(origin=origin), self.assertRaises(WebSocketDisconnect):
                with self.connect(headers={"origin": origin}):
                    pass


if __name__ == "__main__":
    unittest.main()
