"""Gondola-bottom clearance regressions; raw poses remain CV/NED coordinates."""
import io
import itertools
import unittest

import numpy as np

from sim.blimp import Blimp, R_zyx
from sim.controllers import BlimpPD
from sim.keyboard import KeyboardBlimpController
from sim.params import BlimpParams, SimParams, SLIDERS, TaskParams
from sim.sim import TeamSim


class ClearanceGeometryTests(unittest.TestCase):
    def test_ground_default_and_raw_cv_pose(self):
        blimp = Blimp(BlimpParams())
        self.assertAlmostEqual(blimp.gondola_bottom_offset(), 0.325)
        self.assertEqual(blimp.altitude, 0)
        self.assertEqual(blimp.bottom_altitude, 0)
        self.assertAlmostEqual(blimp.cv_altitude, 0.325)
        np.testing.assert_allclose(blimp.eta, [0, 0, -0.325, 0, 0, 0])
        raw_pose = np.array([1, 2, -1.0, 0, 0, 0])
        blimp.reset(eta=raw_pose)
        np.testing.assert_array_equal(blimp.eta, raw_pose)
        self.assertAlmostEqual(blimp.altitude, 0.675)
        self.assertEqual(blimp.cv_altitude, 1.0)
        blimp.reset()
        self.assertEqual(blimp.altitude, 0)

    def test_support_matches_independently_sampled_surface(self):
        blimp = Blimp(BlimpParams())
        box = (np.array(list(itertools.product((-1, 1), repeat=3)))
               * blimp.p.gondola_size / 2 + [0, 0, blimp.p.d_VT])
        points = [box]
        theta = np.linspace(0, 2 * np.pi, 2048, endpoint=False)
        for position, axis in zip(blimp.pos, blimp.axes):
            helper = np.eye(3)[np.argmin(np.abs(axis))]
            radial_x = np.cross(axis, helper)
            radial_x /= np.linalg.norm(radial_x)
            radial_y = np.cross(axis, radial_x)
            ring = blimp.p.thruster_radius * (np.cos(theta)[:, None] * radial_x + np.sin(theta)[:, None] * radial_y)
            for end in (-1, 1):
                points.append(position + end * blimp.p.thruster_length / 2 * axis + ring)
        vertices = np.vstack(points)
        for angles in ((0, 0, 0), (0.3, 0, 0), (0, -0.4, 0.8), (0.45, -0.32, 1.2), (1.7, 0.4, 0.7), (np.pi, 0, 0)):
            with self.subTest(angles=angles):
                sampled_bottom = np.max(np.sum(vertices * R_zyx(*angles)[2], axis=1))
                analytic_bottom = blimp.gondola_bottom_offset(angles)
                self.assertGreaterEqual(analytic_bottom + 1e-14, sampled_bottom)
                self.assertLess(abs(analytic_bottom - sampled_bottom), 5e-8)
                blimp.reset(eta=blimp.pose_at_altitude(0.7, x=1.2, y=-0.4, angles=angles))
                self.assertAlmostEqual(blimp.altitude, 0.7)
                np.testing.assert_array_equal(blimp.eta[:2], [1.2, -0.4])
                np.testing.assert_array_equal(blimp.eta[3:], angles)

    def test_floor_ceiling_clamp_preserves_horizontal_and_angular_state(self):
        blimp = Blimp(BlimpParams())
        angles = [0.25, -0.3, 0.5]
        rotation = R_zyx(*angles)
        for height, vertical_velocity, expected in ((-0.1, 0.4, 0.0), (4.1, -0.4, 4.0)):
            blimp.reset(eta=blimp.pose_at_altitude(height, x=0.6, y=-0.5, angles=angles),
                        nu=np.r_[rotation.T @ [0.1, 0.2, vertical_velocity], [0.3, -0.2, 0.1]])
            blimp.step(0.0)
            self.assertAlmostEqual(blimp.altitude, expected)
            np.testing.assert_array_equal(blimp.eta[:2], [0.6, -0.5])
            np.testing.assert_array_equal(blimp.eta[3:], angles)
            np.testing.assert_array_equal(blimp.nu[3:], [0.3, -0.2, 0.1])
            np.testing.assert_allclose(rotation @ blimp.nu[:3], [0.1, 0.2, 0.0], atol=1e-15)

    def test_landing_contact_and_takeoff(self):
        blimp = Blimp(BlimpParams(net_lift_N=-0.03))
        blimp.reset(eta=blimp.pose_at_altitude(0.2))
        for _ in range(200):
            blimp.step(0.01)
            self.assertGreaterEqual(blimp.altitude, -1e-12)
        self.assertAlmostEqual(blimp.altitude, 0)
        self.assertAlmostEqual((R_zyx(*blimp.eta[3:]) @ blimp.nu[:3])[2], 0)
        blimp.p.net_lift_N = 0
        controller = BlimpPD(TaskParams(blimp_height=0.6))
        for _ in range(60):
            controller.command(blimp)
            for _ in range(5):
                blimp.step(0.01)
        self.assertGreater(blimp.altitude, 0.15)


class ClearanceControllerTests(unittest.TestCase):
    def test_cm_and_cv_target_conversion_at_tilt(self):
        blimp = Blimp(BlimpParams())
        for use_cm in (True, False):
            controller = BlimpPD(TaskParams(blimp_height=0.9), use_cm=use_cm)
            for angles in ([0, 0, 0], [0.3, -0.4, 0.2]):
                blimp.reset(eta=blimp.pose_at_altitude(0.9, angles=angles))
                controlled, _, _ = controller.controlled_point(blimp)
                target, _ = controller.reference(blimp)
                self.assertAlmostEqual(target[2], controlled[2])
                self.assertAlmostEqual(controller.altitude_reference_z(blimp, 0.4) - controlled[2], 0.5)

    def test_captured_clearance_survives_attitude_change(self):
        for use_cm in (True, False):
            blimp = Blimp(BlimpParams())
            blimp.reset(eta=blimp.pose_at_altitude(0.8, x=0.3, y=-0.4, angles=[0.2, -0.15, 0.6]))
            keyboard = KeyboardBlimpController(blimp, hold=BlimpPD(TaskParams(), use_cm=use_cm))
            keyboard.on_pid_toggle(True)
            captured_xy = keyboard.hold.p_ref_override[:2].copy()
            captured_yaw = keyboard.hold.yaw_ref_override
            self.assertAlmostEqual(keyboard.desired_altitude, 0.8)
            blimp.eta[3:] = [-0.3, 0.4, -0.2]
            keyboard.command(blimp, np.zeros(4))
            self.assertAlmostEqual(keyboard.desired_altitude, 0.8)
            expected = keyboard.hold.altitude_reference_z(blimp, 0.8)
            self.assertAlmostEqual(keyboard.hold.p_ref_override[2], expected)
            np.testing.assert_array_equal(keyboard.hold.p_ref_override[:2], captured_xy)
            self.assertEqual(keyboard.hold.yaw_ref_override, captured_yaw)

    def test_explicit_target_toggle_reset_and_manual_heave(self):
        blimp = Blimp(BlimpParams())
        hold = BlimpPD(TaskParams())
        keyboard = KeyboardBlimpController(blimp, hold=hold)
        keyboard.set_altitude_target(1.2)
        self.assertEqual(keyboard.desired_altitude, 1.2)
        hold.task.pid_enabled = False
        keyboard.on_pid_toggle(False)
        keyboard.reset()
        blimp.reset()
        self.assertEqual(keyboard.desired_altitude, 1.2)
        hold.task.pid_enabled = True
        keyboard.on_pid_toggle(True)
        self.assertAlmostEqual(hold.p_ref_override[2], hold.altitude_reference_z(blimp, 1.2))
        blimp.reset(eta=blimp.pose_at_altitude(0.6, angles=[0.3, -0.2, 0.1]))
        keyboard.command(blimp, [0, 0, 0.4, 0])
        self.assertIsNone(keyboard._altitude_target)
        self.assertAlmostEqual(keyboard.desired_altitude, 0.6)
        blimp.eta[3:] = [-0.1, 0.3, 0.5]
        keyboard.command(blimp, [0, 0, 0, 0])
        self.assertAlmostEqual(keyboard.desired_altitude, 0.6)
        self.assertAlmostEqual(hold.p_ref_override[2], hold.altitude_reference_z(blimp, 0.6))
        keyboard.set_altitude_target(0.0)
        self.assertEqual(keyboard.desired_altitude, 0)


class ClearanceLogTests(unittest.TestCase):
    def test_team_ground_reset_and_additive_npz(self):
        params = SimParams()
        params.task.circle_center = (1.0, 2.0)
        sim = TeamSim(params)
        sim.reset()
        np.testing.assert_array_equal(sim.blimp.eta[:2], [0, 0])
        self.assertEqual(sim.blimp.altitude, 0)
        self.assertEqual(SLIDERS[[item[1] for item in SLIDERS].index("blimp_height")][2], 0)
        raw_initial = sim.blimp.eta.copy()
        sim.step()
        sim.step()
        output = io.BytesIO()
        sim.save_npz(output)
        output.seek(0)
        with np.load(output) as data:
            np.testing.assert_array_equal(data["Eta"][0], raw_initial)
            self.assertEqual(data["X"].shape, (1, 6))
            self.assertEqual(data["U"].shape, (1, 6))
            self.assertEqual(data["blimp_bottom_altitude"][0], 0)
            self.assertEqual(data["blimp_bottom_altitude_next"][0], sim.log["blimp_bottom_altitude"][1])
            self.assertEqual(data["eta_reference"].item(), "CV_NED")
            self.assertEqual(data["altitude_reference"].item(), "gondola_bottom")
            np.testing.assert_array_equal(data["gondola_size"], params.blimp.gondola_size)
            self.assertEqual(data["thruster_length"], params.blimp.thruster_length)
            self.assertEqual(data["thruster_radius"], params.blimp.thruster_radius)
        explicit = np.array([0.4, -0.3, -1.0, 0.1, 0.2, 0.3])
        sim.reset(blimp_eta0=explicit)
        np.testing.assert_array_equal(sim.blimp.eta, explicit)
        self.assertTrue(all(not entries for entries in sim.log.values()))


if __name__ == "__main__":
    unittest.main()
