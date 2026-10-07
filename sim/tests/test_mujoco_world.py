"""MuJoCo rover backend and vehicle cameras: frames, kinematics, contact, determinism, logging.

Skipped when the optional `mujoco` package is not installed. Run from the repository root:
    python -m pytest sim/tests/test_mujoco_world.py
"""
from __future__ import annotations

import importlib.util
import io
import os
import struct
import tempfile
import unittest
import zlib

import numpy as np

from sim.params import RoverParams, SimParams
from sim.rover import Rover

HAS_MUJOCO = importlib.util.find_spec("mujoco") is not None


def make_sim(n=1, seed=1, cameras=False, **mj):
    P = SimParams(seed=seed, rover_backend="mujoco", cameras=cameras)
    P.task.n_rovers = n
    for key, value in mj.items():
        setattr(P.mujoco, key, value)
    from sim.sim import TeamSim
    sim = TeamSim(P)
    sim.reset()
    return sim


def drive(sim, rover, v, w, T):
    """Hold one (v, w) command on a MuJoCo rover for T seconds of physics."""
    for _ in range(int(round(T / sim.P.dt_phys))):
        rover.command(v, w)
        sim.world.step(sim.P.dt_phys)


@unittest.skipUnless(HAS_MUJOCO, "mujoco package not installed")
class FrameConventionTests(unittest.TestCase):
    def test_ned_mujoco_round_trip_and_yaw_sign(self):
        from sim.blimp import R_zyx
        from sim.mujoco_world import mj_to_ned_pos, mj_to_ned_rot, ned_to_mj_pos, ned_to_mj_rot
        rng = np.random.default_rng(0)
        for _ in range(20):
            p = rng.normal(size=3)
            R = R_zyx(*rng.uniform(-1, 1, 3))
            np.testing.assert_allclose(mj_to_ned_pos(ned_to_mj_pos(p)), p, atol=1e-12)
            Rm = ned_to_mj_rot(R)
            np.testing.assert_allclose(Rm @ Rm.T, np.eye(3), atol=1e-12)
            self.assertAlmostEqual(np.linalg.det(Rm), 1.0, places=12)
            np.testing.assert_allclose(mj_to_ned_rot(Rm), R, atol=1e-12)
        # NED z is down, so a positive NED yaw is a negative rotation about MuJoCo's z-up axis.
        Rm = ned_to_mj_rot(R_zyx(0.0, 0.0, 0.3))
        self.assertAlmostEqual(np.arctan2(Rm[1, 0], Rm[0, 0]), -0.3, places=12)

    def test_blimp_mocap_follows_eta(self):
        from sim.blimp import R_zyx
        from sim.mujoco_world import ned_to_mj_pos, ned_to_mj_rot
        import mujoco
        sim = make_sim(n=1)
        eta = np.array([0.4, -0.7, -1.3, 0.1, -0.2, 0.9])
        sim.blimp.eta = eta.copy()
        sim.world.sync_blimp(sim.blimp.eta)
        sim.world.forward()
        bid = mujoco.mj_name2id(sim.world.model, mujoco.mjtObj.mjOBJ_BODY, "blimp")
        np.testing.assert_allclose(sim.world.data.xpos[bid], ned_to_mj_pos(eta[:3]), atol=1e-9)
        np.testing.assert_allclose(sim.world.data.xmat[bid].reshape(3, 3), ned_to_mj_rot(R_zyx(*eta[3:])), atol=1e-9)


@unittest.skipUnless(HAS_MUJOCO, "mujoco package not installed")
class RoverDynamicsTests(unittest.TestCase):
    def test_straight_drive_tracks_speed_with_lag(self):
        sim = make_sim(n=1)
        r = sim.rovers[0]
        r.reset(np.zeros(3))
        speeds = []
        for _ in range(300):
            r.command(0.2, 0.0)
            sim.world.step(sim.P.dt_phys)
            speeds.append(r.v[0])
        speeds = np.array(speeds)
        t63 = sim.P.dt_phys * np.argmax(speeds >= 0.2 * (1 - np.exp(-1)))
        self.assertTrue(0.05 <= t63 <= 0.25, f"speed time constant {t63:.3f} s outside 0.05-0.25 s")
        self.assertLess(speeds.max(), 0.2 * 1.2, "speed overshoot above 20 %")
        self.assertAlmostEqual(speeds[-50:].mean(), 0.2, delta=0.004)
        self.assertAlmostEqual(r.q[2], 0.0, delta=0.01)          # heading held
        self.assertAlmostEqual(r.q[1], 0.0, delta=0.01)          # no lateral drift
        self.assertGreater(r.q[0], 0.2 * 3.0 - 0.2 * 0.3)        # distance consistent with the lag

    def test_turn_matches_ideal_unicycle_direction_and_rate(self):
        sim = make_sim(n=1)
        mj = sim.rovers[0]
        mj.reset(np.zeros(3))
        ideal = Rover(RoverParams())
        ideal.reset(np.zeros(3))
        for _ in range(400):
            mj.command(0.2, 0.5)
            ideal.command(0.2, 0.5)
            sim.world.step(sim.P.dt_phys)
            ideal.step(sim.P.dt_phys)
        self.assertGreater(mj.q[2], 0)                            # same turn direction as the unicycle
        self.assertAlmostEqual(mj.q[2], ideal.q[2], delta=0.1)   # 2.0 rad after 4 s, minus the lag
        self.assertLess(np.linalg.norm(mj.q[:2] - ideal.q[:2]), 0.08)
        self.assertAlmostEqual(mj.v[0], 0.2, delta=0.006)
        self.assertAlmostEqual(mj.v[1], 0.5, delta=0.02)

    def test_command_limits_apply(self):
        sim = make_sim(n=1)
        r = sim.rovers[0]
        r.command(5.0, -20.0)
        np.testing.assert_allclose(r.u, [sim.P.rover.v_max, -sim.P.rover.w_max])

    def test_rovers_collide_instead_of_passing_through(self):
        sim = make_sim(n=2)
        a, b = sim.rovers
        a.reset(np.array([-0.3, 0.0, 0.0]))                       # facing +x
        b.reset(np.array([0.3, 0.0, np.pi]))                      # facing -x
        gap = []
        for _ in range(400):
            a.command(0.2, 0.0)
            b.command(0.2, 0.0)
            sim.world.step(sim.P.dt_phys)
            gap.append(np.linalg.norm(a.q[:2] - b.q[:2]))
        gap = np.array(gap)
        # Ideal unicycles would cross (relative travel 1.6 m > 0.6 m); contact keeps the bodies apart.
        # Each chassis reaches 0.04 m ahead of its axle, so nose-to-nose contact leaves ~0.08 m between axles.
        front = sim.P.mujoco.chassis_offset[0] + sim.P.mujoco.chassis_size[0]
        self.assertGreater(gap.min(), 2 * front - 0.01)
        self.assertLess(gap.min(), 2 * front + 0.05)
        self.assertGreater(gap[-1], 2 * front - 0.01)
        self.assertLess(abs(a.v[0]) + abs(b.v[0]), 0.02)        # both stopped against each other

    def test_reset_restores_identical_trajectories(self):
        sim = make_sim(n=3, seed=5)
        for _ in range(40):
            sim.step()
        first = sim.logs()
        sim.reset()
        for _ in range(40):
            sim.step()
        second = sim.logs()
        for key in ("rover_q", "rover_v", "rover_wheel_w", "blimp_eta"):
            np.testing.assert_array_equal(first[key], second[key], err_msg=key)


@unittest.skipUnless(HAS_MUJOCO, "mujoco package not installed")
class TeamAndCameraTests(unittest.TestCase):
    def test_circle_scenario_tracks_and_logs(self):
        sim = make_sim(n=4, seed=2)
        for _ in range(int(15 / sim.P.dt_ctrl)):
            sim.step()
        L = sim.logs()
        q, ref = L["rover_q"], L["rover_ref"]
        d = sim.P.rover.d_hand
        hand = q[:, :, :2] + d * np.stack([np.cos(q[:, :, 2]), np.sin(q[:, :, 2])], axis=2)
        err = np.linalg.norm(hand - ref, axis=2)
        self.assertLess(err[-100:].max(), 0.03, "hand-point tracking error above 3 cm in the last 5 s")
        self.assertAlmostEqual(L["rover_v"][-100:, :, 0].mean(), sim.P.task.rover_speed, delta=0.01)
        self.assertAlmostEqual(L["rover_v"][-100:, :, 1].mean(), sim.P.task.rover_speed / sim.P.task.circle_radius, delta=0.01)
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "circle.npz")
            sim.save_npz(path)
            data = np.load(path)
            n = len(L["t"]) - 1
            self.assertEqual(data["rover_V"].shape, (n, 4, 2))
            self.assertEqual(data["rover_V_next"].shape, (n, 4, 2))
            self.assertEqual(data["rover_wheel_W"].shape, (n, 4, 2))
            self.assertEqual(str(data["rover_backend"]), "mujoco")
            self.assertEqual(float(data["wheel_radius"]), sim.P.mujoco.wheel_radius)
            np.testing.assert_array_equal(data["rover_X_next"], L["rover_q"][1:])

    def test_ideal_backend_unchanged_and_cameras_require_mujoco(self):
        from sim.sim import TeamSim
        sim = TeamSim(SimParams(seed=3))
        sim.reset()
        self.assertIsNone(sim.world)
        self.assertNotIn("rover_v", sim.log)
        self.assertEqual(sim.frames, {})
        arrays = sim.npz_arrays()
        self.assertEqual(str(arrays["rover_backend"]), "ideal")
        self.assertNotIn("rover_V", arrays)
        with self.assertRaises(ValueError):
            TeamSim(SimParams(cameras=True))

    def test_cameras_render_with_aligned_metadata(self):
        sim = make_sim(n=2, cameras=True, cam_width=320, cam_height=240)
        self.assertEqual(set(sim.frames), {"rover0", "rover1", "blimp"})
        for img in sim.frames.values():
            self.assertEqual(img.shape, (240, 320, 3))
            self.assertEqual(img.dtype, np.uint8)
            self.assertGreater(img.std(), 1.0, "camera image is flat")
        self.assertEqual(sim.frame_meta["k"], 0)
        self.assertEqual(sim.frame_meta["t"], 0.0)
        sim.step()
        self.assertEqual(sim.frame_meta["k"], sim.k)
        self.assertAlmostEqual(sim.frame_meta["t"], sim.t)
        K = sim.frame_meta["intrinsics"]
        self.assertAlmostEqual(K["fx"], 120 / np.tan(np.radians(45)), places=9)
        self.assertEqual((K["cx"], K["cy"]), (160.0, 120.0))
        cam = sim.frame_meta["cameras"]["blimp"]
        # Downward camera sits below the CV (NED z down => larger z) by the gondola clearance plus offset.
        expected_depth = sim.world.bottom_offset + sim.P.mujoco.blimp_cam_offset
        self.assertAlmostEqual(cam["position"][2] - sim.blimp.eta[2], expected_depth, places=6)
        # OpenCV z axis (forward) points down in NED for the level blimp camera.
        np.testing.assert_allclose(cam["R_cv"][:, 2], [0.0, 0.0, 1.0], atol=1e-9)
        np.testing.assert_allclose(cam["R_cv"] @ cam["R_cv"].T, np.eye(3), atol=1e-9)
        rover_cam = sim.frame_meta["cameras"]["rover0"]
        heading = np.array([np.cos(sim.rovers[0].q[2]), np.sin(sim.rovers[0].q[2]), 0.0])
        # Forward camera: OpenCV z axis is the heading tilted down by the configured pitch, up to
        # the small attitude the chassis takes on its contacts (well under 1 degree).
        pitch = np.radians(sim.P.mujoco.rover_cam_pitch_deg)
        self.assertAlmostEqual(float(rover_cam["R_cv"][:, 2] @ heading), np.cos(pitch), delta=0.01)
        self.assertAlmostEqual(float(rover_cam["R_cv"][2, 2]), np.sin(pitch), delta=0.02)
        self.assertAlmostEqual(float(rover_cam["R_cv"][:, 0] @ heading), 0.0, delta=0.02)   # image x axis is lateral

    def test_camera_tilt_is_live_and_geometrically_right(self):
        from sim.mujoco_world import camera_axes
        sim = make_sim(n=1, cameras=True, cam_width=160, cam_height=120)
        world = sim.world
        # Blimp level at yaw 0: the OpenCV forward axis of the blimp camera is the tilt direction in NED.
        for tilt in (90.0, 45.0, 0.0, 20.0):
            world.set_camera_angles(blimp_tilt_deg=tilt)
            cam = world.camera_pose_ned("blimp")
            np.testing.assert_allclose(cam["R_cv"][:, 2], [np.cos(np.radians(tilt)), 0.0, np.sin(np.radians(tilt))], atol=1e-9)
            np.testing.assert_allclose(cam["R_cv"][:, 0], [0.0, 1.0, 0.0], atol=1e-9)   # image x = blimp's right (NED +y)
            self.assertEqual(sim.P.mujoco.blimp_cam_tilt_deg, tilt)
        # Rover camera pitch follows the same convention relative to the rover heading.
        r = sim.rovers[0]
        r.reset(np.array([0.0, 0.0, 0.5]))
        heading = np.array([np.cos(0.5), np.sin(0.5), 0.0])
        for pitch in (0.0, 30.0, -20.0):
            world.set_camera_angles(rover_pitch_deg=pitch)
            cam = world.camera_pose_ned("rover0")
            self.assertAlmostEqual(float(cam["R_cv"][:, 2] @ heading), np.cos(np.radians(pitch)), delta=0.01)
            self.assertAlmostEqual(float(cam["R_cv"][2, 2]), np.sin(np.radians(pitch)), delta=0.01)
        # The change is visible in the next rendered frame without rebuilding the model.
        world.set_camera_angles(blimp_tilt_deg=90.0)
        down, _ = world.render(0, 0.0)
        world.set_camera_angles(blimp_tilt_deg=0.0)
        ahead, _ = world.render(0, 0.0)
        self.assertGreater(np.abs(down["blimp"].astype(int) - ahead["blimp"].astype(int)).mean(), 5.0)
        np.testing.assert_allclose(camera_axes(90.0) @ camera_axes(90.0).T, np.eye(3), atol=1e-12)
        world.set_camera_angles(blimp_tilt_deg=90.0)

    def test_frame_recorder_writes_png_and_index(self):
        from sim.mujoco_world import FrameRecorder
        sim = make_sim(n=1, cameras=True, cam_width=160, cam_height=120)
        with tempfile.TemporaryDirectory() as tmp:
            rec = FrameRecorder(tmp, every=2)
            rec(sim)
            for _ in range(4):
                sim.step()
                rec(sim)
            count = rec.close()
            self.assertEqual(count, 3 * 2)                       # k = 0, 2, 4 for two cameras
            files = sorted(f for f in os.listdir(tmp) if f.endswith(".png"))
            self.assertEqual(files, ["blimp_000000.png", "blimp_000002.png", "blimp_000004.png",
                                     "rover0_000000.png", "rover0_000002.png", "rover0_000004.png"])
            with open(os.path.join(tmp, files[0]), "rb") as f:
                png = f.read()
            self.assertEqual(png[:8], b"\x89PNG\r\n\x1a\n")
            width, height = struct.unpack(">II", png[16:24])
            self.assertEqual((width, height), (160, 120))
            with open(os.path.join(tmp, "index.csv")) as f:
                lines = f.read().strip().splitlines()
            self.assertEqual(lines[0], "k,t,camera,file,px,py,pz,qw,qx,qy,qz")
            self.assertEqual(len(lines), 7)
            self.assertTrue(os.path.exists(os.path.join(tmp, "cameras.json")))

    def test_png_writer_round_trip(self):
        from sim.png import write_png
        rng = np.random.default_rng(0)
        img = rng.integers(0, 256, size=(5, 7, 3), dtype=np.uint8)
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "x.png")
            write_png(path, img)
            with open(path, "rb") as f:
                data = f.read()
        # Decode the single IDAT chunk by hand: filter byte 0 per row, then raw RGB.
        pos, chunks = 8, {}
        while pos < len(data):
            length, tag = struct.unpack(">I4s", data[pos:pos + 8])
            chunks[tag] = data[pos + 8:pos + 8 + length]
            pos += 12 + length
        raw = zlib.decompress(chunks[b"IDAT"])
        rows = np.frombuffer(raw, dtype=np.uint8).reshape(5, 1 + 7 * 3)
        self.assertTrue(np.all(rows[:, 0] == 0))
        np.testing.assert_array_equal(rows[:, 1:].reshape(5, 7, 3), img)

    def test_web_runtime_accepts_backend(self):
        from sim.web_runtime import SimulationRuntime
        runtime = SimulationRuntime(n=2, seed=4, mode="auto", rover_backend="mujoco")
        self.assertEqual(runtime.config()["rover_backend"], "mujoco")
        for _ in range(5):
            runtime.sim.step()
        state = runtime._make_state()
        self.assertEqual(len(state["rover_q"]), 2)
        data = np.load(io.BytesIO(runtime._export_npz()))
        self.assertEqual(data["rover_V"].shape[1:], (2, 2))
        self.assertEqual(str(data["rover_backend"]), "mujoco")
        with self.assertRaises(ValueError):
            SimulationRuntime(rover_backend="webots")


@unittest.skipUnless(HAS_MUJOCO, "mujoco package not installed")
class ManualWheelsAndGuiTests(unittest.TestCase):
    def test_manual_ctrl_hands_wheels_to_external_sliders(self):
        sim = make_sim(n=1)
        r = sim.rovers[0]
        r.reset(np.zeros(3))
        sim.world.manual_ctrl = True
        sim.world.data.ctrl[list(r.act)] = 6.0                  # as the MuJoCo viewer's Control sliders would
        for _ in range(200):
            r.command(0.0, 0.0)                                 # the tracker's command must not override
            sim.world.step(sim.P.dt_phys)
        np.testing.assert_allclose(sim.world.data.ctrl[list(r.act)], 6.0)
        self.assertAlmostEqual(r.v[0], 6.0 * sim.P.mujoco.wheel_radius, delta=0.01)
        np.testing.assert_array_equal(r.u, [0.0, 0.0])
        sim.world.manual_ctrl = False
        r.command(0.0, 0.0)
        for _ in range(200):
            sim.world.step(sim.P.dt_phys)
        self.assertLess(abs(r.v[0]), 0.01)                      # tracker in charge again

    def test_gui_session_pulses_stick_and_key_sources(self):
        import mujoco
        from run_mujoco_gui import (KEY_BACKSPACE, KEY_DELETE, KEY_END, KEY_ENTER, KEY_PAGE_UP, KEY_SPACE, KEY_TAB, KEY_UP,
                                    GuiSession, StickKeyboard, cv_key_to_glfw, run, stick_from_drag)
        P = SimParams(seed=3, rover_backend="mujoco")
        P.task.n_rovers = 3
        session = GuiSession(P, speed=50.0, mode="teleop", cameras=True, camera_window=False, pulse=0.5)
        wall = [0.0]
        session.clock = lambda: wall[0]                          # deterministic wall clock for repeat detection
        self.assertIsInstance(session.keyboard, StickKeyboard)
        self.assertTrue(P.task.pid_enabled)                      # hold assist armed at start
        # Viewer layout: arrows/PgUp/End pulse flight axes; letters from the viewer are ignored (MuJoCo toggles).
        session.on_key(KEY_UP)
        wall[0] = 1.0
        session.on_key(KEY_PAGE_UP)
        session.on_key(ord("D"))                                 # viewer letter: must not act
        self.assertEqual(session.keyboard.pressed, {"w", "q"})
        for _ in range(6):                                       # 0.3 s: still inside the 0.5 s pulse
            session.step()
        self.assertEqual(session.keyboard.pressed, {"w", "q"})
        self.assertGreater(abs(session.keyboard.cmd4).sum(), 0)
        wall[0] = 10.0                                           # a deliberate second tap (not auto-repeat) renews the surge pulse
        session.on_key(KEY_UP)
        self.assertAlmostEqual(session.pulse_until["w"], session.sim.t + 0.5)
        for _ in range(6):                                       # 0.6 s total: heave expired, surge renewed
            session.step()
        self.assertEqual(session.keyboard.pressed, {"w"})
        for _ in range(12):
            session.step()
        self.assertEqual(session.keyboard.pressed, set())        # everything expired; hold assist in charge
        np.testing.assert_array_equal(session.keyboard.cmd4, np.zeros(4))
        # A held key arrives as auto-repeat (events a few ms apart): the command follows the hold and
        # ends HOLD_TAIL after the last event instead of accumulating one pulse per event.
        from run_mujoco_gui import HOLD_TAIL
        wall[0] = 20.0
        for _ in range(40):                                      # 40 repeats at 30 ms = 1.2 s of holding
            session.on_key(ord("W"), source="window")
            wall[0] += 0.03
            session.step()
        # the last repeat was stamped one control step before the current simulation time
        self.assertAlmostEqual(session.pulse_until["w"], session.sim.t - session.P.dt_ctrl + HOLD_TAIL, places=9)
        self.assertEqual(session.keyboard.pressed, {"w"})
        for _ in range(6):                                       # 0.3 s > HOLD_TAIL: the command has ended
            session.step()
        self.assertEqual(session.keyboard.pressed, set())        # released shortly after the last repeat
        wall[0] = 99.0
        session.on_key(KEY_END)
        wall[0] += 1.0
        session.on_key(ord("D"), source="window")                # camera window letters do act
        wall[0] += 1.0
        session.on_key(ord("F"), source="window")                # partner key replaces yaw direction
        self.assertEqual(session.keyboard.pressed, {"d", "f"})
        session.on_key(KEY_ENTER)                                # paused: pulses do not drain (simulation time)
        for _ in range(30):
            session.step()
        self.assertEqual(session.keyboard.pressed, {"d", "f"})
        self.assertIn("paused", session.status)
        session.on_key(ord("P"), source="window")
        session.on_key(ord("P"))                                 # P only pauses from the camera window
        self.assertFalse(session.paused)
        session.on_key(KEY_SPACE)
        self.assertEqual(session.keyboard.pressed, set())
        self.assertEqual(session.pulse_until, {})
        # Joystick vector adds to the command and is zeroed by Space.
        session.keyboard.stick[:] = stick_from_drag((100, 100), (100, 60), "left", 80)
        np.testing.assert_allclose(session.keyboard.stick, [0.5, 0.0, 0.0, 0.0])
        np.testing.assert_allclose(session.keyboard.current_cmd4(), [0.5 * 0.4, 0, 0, 0])
        np.testing.assert_allclose(stick_from_drag((0, 0), (40, 0), "left", 80), [0.0, -0.5, 0.0, 0.0])   # drag right = sway right
        np.testing.assert_allclose(stick_from_drag((0, 0), (80, -160), "right", 80), [0.0, 0.0, 1.0, 1.0])  # clipped, up/yaw right
        session.on_key(KEY_SPACE)
        np.testing.assert_array_equal(session.keyboard.stick, np.zeros(4))
        session.on_key(KEY_TAB)
        self.assertFalse(P.task.pid_enabled)
        session.on_key(ord("H"), source="window")
        self.assertTrue(P.task.pid_enabled)
        session.on_key(KEY_DELETE)
        self.assertTrue(session.world.manual_ctrl)
        session.on_key(ord("M"), source="window")
        self.assertFalse(session.world.manual_ctrl)
        session.on_key(61)                                        # '=' raises the gain
        self.assertAlmostEqual(session.keyboard.gain, 0.5)
        session.on_key(45)
        self.assertAlmostEqual(session.keyboard.gain, 0.4)
        session.on_key(ord("2"))
        self.assertEqual(session.selected_camera, "rover1")
        session.on_key(ord("9"))                                  # no ninth camera: selection unchanged
        self.assertEqual(session.selected_camera, "rover1")
        session.on_key(ord("0"))
        self.assertIsNone(session.selected_camera)
        session.on_key(KEY_UP)
        session.on_key(KEY_BACKSPACE)
        self.assertEqual(session.sim.k, 0)
        self.assertEqual(session.pulse_until, {})                 # reset clears pulses
        self.assertEqual(cv_key_to_glfw(ord("w")), ord("W"))
        self.assertEqual(cv_key_to_glfw(13), KEY_ENTER)
        self.assertEqual(cv_key_to_glfw(8), KEY_BACKSPACE)
        self.assertIsNone(cv_key_to_glfw(-1))
        labels, values = session.hud_lines()
        self.assertEqual(len(labels.split("\n")), len(values.split("\n")))
        scene = mujoco.MjvScene(session.world.model, maxgeom=50)
        session.draw_markers(scene)
        self.assertEqual(scene.ngeom, 3)

        class FakeViewer:                                         # drives run() without a window
            def __init__(self):
                self.calls, self.user_scn = 0, mujoco.MjvScene(session.world.model, maxgeom=50)
                self.viewport = mujoco.MjrRect(0, 0, 1000, 700)
                self.texts, self.images, self.cleared = None, None, 0

            def is_running(self):
                self.calls += 1
                return self.calls <= 5

            def lock(self):
                import contextlib
                return contextlib.nullcontext()

            def sync(self):
                pass

            def set_texts(self, texts):
                self.texts = texts

            def set_images(self, images):
                self.images = images

            def clear_images(self):
                self.cleared += 1

        fake = FakeViewer()
        session.on_key(ord("1"))
        run(session, fake)
        self.assertEqual(session.sim.k, 5)
        self.assertEqual(len(fake.images), 4 + 1)                 # four thumbnails (3 rovers + blimp) plus the enlarged rover0 view
        self.assertEqual(fake.texts[1], mujoco.mjtGridPos.mjGRID_TOPLEFT)
        auto = GuiSession(SimParams(seed=1, rover_backend="mujoco"), mode="auto", cameras=False)
        self.assertIsNone(auto.keyboard)
        auto.on_key(KEY_UP)                                       # no teleop in auto mode
        auto.step()
        self.assertIn("autopilot", auto.status)
        with self.assertRaises(ValueError):
            GuiSession(SimParams())
        with self.assertRaises(ValueError):
            GuiSession(SimParams(rover_backend="mujoco"), pulse=0.0)

    def test_pulse_tap_matches_browser_key_hold(self):
        """A 0.5 s tap in the MuJoCo GUI must reproduce holding W for 0.5 s in the browser runtime exactly."""
        from run_mujoco_gui import KEY_UP, GuiSession
        from sim.web_runtime import SimulationRuntime
        rt = SimulationRuntime(n=1, seed=0, mode="teleop")
        web = []
        for k in range(80):
            rt.keyboard.pressed = {"w"} if k * 0.05 < 0.5 else set()
            rt.keyboard.cmd4 = rt.keyboard.current_cmd4()
            rt.sim.step()
            web.append(rt.sim.blimp.eta.copy())
        P = SimParams(seed=0, rover_backend="mujoco")
        P.task.n_rovers = 1
        gs = GuiSession(P, mode="teleop", cameras=False, pulse=0.5)
        gs.on_key(KEY_UP)
        mj = []
        for _ in range(80):
            gs.step()
            mj.append(gs.sim.blimp.eta.copy())
        np.testing.assert_array_equal(np.array(web), np.array(mj))

    def test_camera_window_trackbars_queue_angles(self):
        from run_mujoco_gui import CameraWindow, GuiSession, run
        import mujoco
        window = CameraWindow(["rover0", "blimp"], blimp_tilt_deg=90.0, rover_pitch_deg=10.0)
        window.on_trackbar("blimp cam tilt (deg)", 90)                 # unchanged: nothing queued
        self.assertEqual(window.take_pending_angles(), {})
        window.on_trackbar("blimp cam tilt (deg)", 45)
        window.on_trackbar("rover cam pitch (deg, -30..60)", 0)        # position 0 -> -30 deg
        self.assertEqual(window.take_pending_angles(), {"blimp_tilt_deg": 45.0, "rover_pitch_deg": -30.0})
        self.assertEqual(window.take_pending_angles(), {})
        P = SimParams(seed=1, rover_backend="mujoco")
        P.task.n_rovers = 1
        P.mujoco.blimp_cam_tilt_deg = 60.0
        session = GuiSession(P, mode="auto", cameras=True, camera_window=False)
        cam = session.world.camera_pose_ned("blimp")
        np.testing.assert_allclose(cam["R_cv"][:, 2], [np.cos(np.radians(60)), 0.0, np.sin(np.radians(60))], atol=1e-9)
        self.assertIn("blimp cam 60 deg", session.hud_lines()[1])

    def test_pip_layout_adapts_to_viewport(self):
        import mujoco
        from run_mujoco_gui import MARGIN, CameraWindow, pip_layout
        names = ["rover0", "rover1", "rover2", "rover3", "blimp"]
        frames = {n: np.full((480, 640, 3), 40 + 10 * i, dtype=np.uint8) for i, n in enumerate(names)}
        wide = pip_layout(mujoco.MjrRect(0, 0, 1600, 900), names, frames, "blimp")
        self.assertEqual(len(wide), 6)
        self.assertEqual(wide[0][2:4], (160, 120))                # largest thumbnails fit across 1600 px
        self.assertEqual(wide[-1][2:4], (320, 240))               # enlarged view
        self.assertEqual(wide[-1][0], 1600 - 320 - MARGIN)
        self.assertTrue(np.all(wide[4][4][0] == (69, 217, 210)))  # selected thumbnail is outlined
        narrow = pip_layout(mujoco.MjrRect(400, 0, 470, 720), names, frames, "rover2")
        self.assertEqual(len(narrow), 6)
        self.assertEqual(narrow[0][2:4], (80, 60))                # 5 x 88 px strip fits in 470 px
        self.assertEqual(narrow[0][0], 400 + MARGIN)
        self.assertEqual(narrow[-1][2:4], (320, 240))
        tiny = pip_layout(mujoco.MjrRect(0, 0, 300, 200), names, frames, None)
        self.assertEqual(len(tiny), 3)                            # only as many 80 px thumbnails as fit, no large view
        self.assertEqual(pip_layout(mujoco.MjrRect(0, 0, 800, 600), names, {}, None), [])
        window = CameraWindow(names)
        grid = window.compose(frames)
        self.assertEqual(grid.shape, (480, 960, 3))               # 2 rows x 3 cols of 240 x 320 tiles
        self.assertEqual(window.compose(frames, "rover1").shape, (480, 640, 3))
        self.assertEqual(window.compose({}).shape, (240, 320, 3))


class ImageEncodingTests(unittest.TestCase):
    def test_png_bytes_and_encode_image(self):
        from sim.png import encode_image, png_bytes
        rng = np.random.default_rng(1)
        img = rng.integers(0, 256, size=(12, 16, 3), dtype=np.uint8)
        png = png_bytes(img)
        self.assertEqual(png[:8], b"\x89PNG\r\n\x1a\n")
        self.assertEqual(struct.unpack(">II", png[16:24]), (16, 12))
        data, media = encode_image(img)
        self.assertIn(media, ("image/jpeg", "image/png"))
        self.assertGreater(len(data), 100)
        if media == "image/jpeg":
            self.assertEqual(data[:2], b"\xff\xd8")


@unittest.skipUnless(HAS_MUJOCO and importlib.util.find_spec("fastapi") is not None, "mujoco and fastapi packages required")
class WebCameraTests(unittest.TestCase):
    def test_camera_endpoint_serves_frames_with_etag(self):
        import time
        from fastapi.testclient import TestClient
        from sim.web_runtime import SimulationRuntime
        from web_server import create_app
        runtime = SimulationRuntime(n=2, mode="auto", rover_backend="mujoco", cameras=True)
        with self.assertRaises(ValueError):
            SimulationRuntime(cameras=True)                     # ideal backend has no cameras
        with TestClient(create_app(runtime), base_url="http://127.0.0.1") as client:
            config = client.get("/api/config").json()
            self.assertEqual(config["cameras"], ["rover0", "rover1", "blimp"])
            self.assertEqual(config["camera_size"], [640, 480])
            deadline = time.time() + 5
            response = client.get("/api/camera/blimp")
            while response.status_code == 503 and time.time() < deadline:   # worker renders its first frame on start
                time.sleep(0.05)
                response = client.get("/api/camera/blimp")
            self.assertEqual(response.status_code, 200, response.text)
            self.assertIn(response.headers["content-type"], ("image/jpeg", "image/png"))
            self.assertEqual(response.headers["x-frame-id"], "0")   # paused at reset
            self.assertEqual(response.headers["x-sim-time"], "0.000")
            etag = response.headers["etag"]
            self.assertGreater(len(response.content), 1000)
            again = client.get("/api/camera/blimp", headers={"If-None-Match": etag})
            self.assertEqual(again.status_code, 304)
            self.assertEqual(client.get("/api/camera/rover1").status_code, 200)
            self.assertEqual(client.get("/api/camera/rover9").status_code, 404)
            with client.websocket_connect("ws://127.0.0.1/ws") as ws:
                ws.receive_json()
                ws.send_json({"type": "command", "id": "run", "action": "pause", "value": False})
                deadline = time.time() + 5
                while runtime.snapshot()["t"] < 0.2 and time.time() < deadline:
                    time.sleep(0.05)
                later = client.get("/api/camera/blimp", headers={"If-None-Match": etag})
                self.assertEqual(later.status_code, 200)
                self.assertGreater(int(later.headers["x-frame-id"]), 0)
                self.assertNotEqual(later.headers["etag"], etag)
                ws.send_json({"type": "command", "id": "stop", "action": "pause", "value": True})

    def test_camera_tilt_parameter_in_browser_runtime(self):
        from sim.params import SLIDERS
        from sim.web_runtime import SimulationRuntime, validate_command
        ideal = SimulationRuntime(n=1)
        self.assertEqual(len(ideal.config()["parameters"]), len(SLIDERS))      # camera controls hidden without MuJoCo
        with self.assertRaises(ValueError):
            ideal._apply(validate_command({"type": "command", "id": "c", "action": "param", "key": "mujoco.blimp_cam_tilt_deg", "value": 45}), 0.0)
        runtime = SimulationRuntime(n=1, rover_backend="mujoco", cameras=True)
        config = runtime.config()
        keys = [p["key"] for p in config["parameters"]]
        self.assertEqual(len(keys), len(SLIDERS) + 2)
        self.assertIn("mujoco.blimp_cam_tilt_deg", keys)
        tilt = next(p for p in config["parameters"] if p["key"] == "mujoco.blimp_cam_tilt_deg")
        self.assertEqual((tilt["min"], tilt["max"], tilt["value"], tilt["group"]), (0.0, 90.0, 90.0, "mujoco"))
        runtime._apply(validate_command({"type": "command", "id": "c", "action": "param", "key": "mujoco.blimp_cam_tilt_deg", "value": 30}), 0.0)
        cam = runtime.sim.world.camera_pose_ned("blimp")
        np.testing.assert_allclose(cam["R_cv"][:, 2], [np.cos(np.radians(30)), 0.0, np.sin(np.radians(30))], atol=1e-9)
        self.assertEqual(runtime._make_state()["params"]["mujoco.blimp_cam_tilt_deg"], 30.0)
        with self.assertRaises(ValueError):
            validate_command({"type": "command", "id": "c", "action": "param", "key": "mujoco.blimp_cam_tilt_deg", "value": 95})
        runtime._apply(validate_command({"type": "command", "id": "c", "action": "param", "key": "mujoco.rover_cam_pitch_deg", "value": -10}), 0.0)
        self.assertEqual(runtime.P.mujoco.rover_cam_pitch_deg, -10.0)

    def test_camera_endpoint_absent_without_cameras(self):
        from fastapi.testclient import TestClient
        from sim.web_runtime import SimulationRuntime
        from web_server import create_app
        runtime = SimulationRuntime(n=1, rover_backend="mujoco")
        with TestClient(create_app(runtime), base_url="http://127.0.0.1") as client:
            self.assertEqual(client.get("/api/config").json()["cameras"], [])
            self.assertEqual(client.get("/api/camera/blimp").status_code, 404)


if __name__ == "__main__":
    unittest.main()
