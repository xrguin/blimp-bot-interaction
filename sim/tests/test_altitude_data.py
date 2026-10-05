"""Bounded checks for altitude transition collection and independent replay."""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from collect_altitude_data import (
    EPISODES, CollectionGuardError, EpisodeSpec, build_command_schedule,
    collect_dataset, collect_episode, collect_episodes, collection_params, write_dataset,
)
from sim.blimp import Blimp, R_zyx


class AltitudeDataTests(unittest.TestCase):
    def test_schedule_exact_repeatability_whole_blocks_and_excitation(self):
        first, blocks = build_command_schedule(42)
        second, repeated_blocks = build_command_schedule(42)
        different, _ = build_command_schedule(43)
        np.testing.assert_array_equal(first, second)
        self.assertEqual(blocks, repeated_blocks)
        self.assertFalse(np.array_equal(first, different))
        self.assertEqual(first.shape, (1200,))
        np.testing.assert_array_equal(first[:20], 0)
        np.testing.assert_array_equal(first[-20:], 0)
        self.assertEqual({block["amplitude"] for block in blocks}, {0.2, 0.4, 0.6, 0.8, 1.0})
        self.assertEqual({block["dwell_steps"] for block in blocks}, {5, 10, 15})
        for block in blocks:
            pulse = block["sign"] * block["amplitude"]
            expected = np.r_[np.repeat([pulse, -pulse, -pulse, pulse], block["dwell_steps"]), np.zeros(10)]
            np.testing.assert_array_equal(first[block["start_step"]:block["stop_step"]], expected)

    def test_trajectory_repeatability_terminal_transition_and_vertical_only(self):
        spec = EpisodeSpec(2, "train", 2.25)
        first, manifest = collect_episode(spec, 44, duration_s=6.0)
        second, _ = collect_episode(spec, 44, duration_s=6.0)
        for key in first:
            np.testing.assert_array_equal(first[key], second[key])
        self.assertEqual(len(first["t"]), 120)
        self.assertAlmostEqual(first["t"][-1], 5.95)
        self.assertEqual(first["t_next"][-1], 6.0)
        np.testing.assert_array_equal(first["Eta_next"][:-1], first["Eta"][1:])
        np.testing.assert_array_equal(first["X_next"][:-1], first["X"][1:])
        np.testing.assert_array_equal(first["blimp_thrust_next"][:-1], first["blimp_thrust"][1:])
        np.testing.assert_array_equal(first["U"][:, :4], 0)
        np.testing.assert_array_equal(first["U"][:, 4], first["u_z"])
        np.testing.assert_array_equal(first["U"][:, 5], first["u_z"])
        np.testing.assert_allclose(first["Eta_next"][:, [0, 1, 3, 4, 5]], 0, atol=1e-12)
        np.testing.assert_allclose(first["X_next"][:, [0, 1, 3, 4, 5]], 0, atol=1e-12)
        self.assertTrue(np.all(first["valid_transition"]))
        self.assertFalse(np.any(first["contact"]))
        np.testing.assert_array_equal(first["u_z"], manifest["u_z_schedule"])
        # Realized thrust must be a state, not simply the held command.
        self.assertTrue(np.any(np.abs(first["blimp_thrust_next"] - first["thrust_cmd_N"]) > 1e-5))

    def test_saved_transition_independent_replay(self):
        specs = (EpisodeSpec(0, "train", 1.25), EpisodeSpec(8, "validation", 1.5), EpisodeSpec(10, "test", 1.35))
        splits, episodes = collect_episodes(duration_s=6.0, episode_specs=specs)
        with tempfile.TemporaryDirectory() as temp:
            output = Path(temp) / "dataset"
            write_dataset(output, splits, episodes, duration_s=6.0)
            with np.load(output / "train.npz", allow_pickle=False) as saved:
                rows = np.flatnonzero(saved["u_z"] != 0)
                index = int(rows[len(rows) // 2])
                params = collection_params(int(saved["seed"][index]))
                replay = Blimp(params.blimp)
                replay.floor_alt, replay.ceiling_alt = params.floor_alt, params.ceiling_alt
                replay.reset(eta=saved["Eta"][index], nu=saved["X"][index])
                replay.thrust[:] = saved["blimp_thrust"][index]
                replay.command(saved["U"][index])
                np.testing.assert_array_equal(replay.thrust_cmd, saved["thrust_cmd_N"][index])
                for _ in range(5):
                    replay.step(0.01)
                np.testing.assert_array_equal(replay.eta, saved["Eta_next"][index])
                np.testing.assert_array_equal(replay.nu, saved["X_next"][index])
                np.testing.assert_array_equal(replay.thrust, saved["blimp_thrust_next"][index])
                self.assertEqual(replay.altitude, saved["altitude_next"][index])
                rotation = R_zyx(*replay.eta[3:])
                self.assertEqual(-(rotation @ replay.nu[:3])[2], saved["v_up_next"][index])
                self.assertEqual(-(rotation @ replay.wrench(replay.thrust)[:3])[2], saved["force_up_next"][index])

    def test_split_boundaries_manifest_sources_and_safe_arrays(self):
        splits, episodes = collect_episodes(duration_s=2.0)
        self.assertEqual([spec.initial_altitude for spec in EPISODES[:8]], [1.25, 1.75, 2.25, 2.75] * 2)
        self.assertEqual(set(splits["train"]["episode_id"]), set(range(8)))
        self.assertEqual(set(splits["validation"]["episode_id"]), {8, 9})
        self.assertEqual(set(splits["test"]["episode_id"]), {10, 11})
        self.assertEqual([episode["seed"] for episode in episodes], list(range(42, 54)))
        with tempfile.TemporaryDirectory() as temp:
            output = Path(temp) / "dataset"
            write_dataset(output, splits, episodes, duration_s=2.0)
            manifest = json.loads((output / "manifest.json").read_text())
            self.assertEqual(manifest["simulation_parameters"]["task"]["pid_enabled"], False)
            self.assertEqual(manifest["simulation_parameters"]["blimp"]["net_lift_N"], 0)
            self.assertEqual(manifest["timing"]["physics_steps_per_transition"], 5)
            for split in splits:
                self.assertEqual(hashlib.sha256((output / f"{split}.npz").read_bytes()).hexdigest(), manifest["splits"][split]["sha256"])
                with np.load(output / f"{split}.npz", allow_pickle=False) as data:
                    for key in data.files:
                        self.assertNotEqual(data[key].dtype, np.dtype("O"))
                        np.testing.assert_array_equal(data[key], splits[split][key])
            for source in manifest["sources"].values():
                self.assertEqual(hashlib.sha256((output / source["snapshot"]).read_bytes()).hexdigest(), source["sha256"])
            with self.assertRaises(FileExistsError):
                collect_dataset(output)

    def test_guard_aborts_without_retry_or_partial_episode(self):
        with self.assertRaises(CollectionGuardError):
            collect_episode(EpisodeSpec(0, "train", 0.39), 42, duration_s=6.0)
        with self.assertRaises(CollectionGuardError):
            collect_episode(EpisodeSpec(0, "train", 3.61), 42, duration_s=6.0)
        with tempfile.TemporaryDirectory() as temp:
            output = Path(temp) / "aborted"
            with patch("collect_altitude_data.collect_episodes", side_effect=CollectionGuardError("substep guard")):
                with self.assertRaises(CollectionGuardError):
                    collect_dataset(output)
            self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main()
