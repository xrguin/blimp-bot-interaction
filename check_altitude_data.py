"""Validate an altitude-lesson dataset and plot recorded data (no model fitting).

    python check_altitude_data.py results/altitude_lesson/2026-10-04_neutral_seed42
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path

import numpy as np

from sim.blimp import Blimp
from sim.params import BlimpParams, G, SimParams


SPLITS = {"train": range(8), "validation": range(8, 10), "test": range(10, 12)}
INITIAL_HEIGHTS = [1.25, 1.75, 2.25, 2.75] * 2 + [1.5, 2.5, 1.35, 2.65]
VECTOR_FIELDS = ("Eta", "Eta_next", "X", "X_next", "U", "blimp_thrust",
                 "blimp_thrust_next", "thrust_cmd_N")
SCALAR_FIELDS = ("t", "t_next", "episode_id", "step", "seed", "altitude",
                 "altitude_next", "v_up", "v_up_next", "force_up", "force_up_next",
                 "u_z", "net_lift_N", "net_lift_g", "valid_transition", "contact")


def require(condition, message):
    if not bool(condition):
        raise ValueError(message)


def inspect_manifest(directory):
    manifest = json.loads((directory / "manifest.json").read_text())
    require(manifest["schema_version"] == "altitude-transitions-v1", "Unknown dataset schema")
    require(manifest["episode_count"] == 12 and manifest["duration_per_episode_s"] == 60, "Wrong collection protocol")
    timing = manifest["timing"]
    require((timing["dt_phys_s"], timing["dt_ctrl_s"], timing["physics_steps_per_transition"]) == (0.01, 0.05, 5), "Wrong timestep metadata")
    require(timing["state_timing"] == "current before command; next after five physics steps; terminal transition included", "Unknown transition convention")
    require(manifest["gravity_m_s2"] == G, "Gravity metadata mismatch")
    base_seed = manifest["base_seed"]
    require(type(base_seed) is int and 0 <= base_seed <= 2**32 - 12, "Invalid base seed")
    expected_params = SimParams(seed=base_seed)
    expected_params.task.n_rovers = 0
    expected_params.task.pid_enabled = False
    require(manifest["simulation_parameters"] == json.loads(json.dumps(asdict(expected_params))), "Non-default or incorrectly declared collection parameters")
    require(manifest["guards"]["abort_altitude_m"] == [0.4, 3.6], "Incorrect boundary guard")
    require(manifest["guards"]["valid_endpoint_altitude_m"] == [0.2, 3.8], "Incorrect valid range")
    require(manifest["guards"]["retry_or_feedback"] is False, "Unexpected feedback or retry policy")
    action = manifest["action"]
    require(action["amplitudes"] == [0.2, 0.4, 0.6, 0.8, 1.0], "Wrong command levels")
    require(action["dwell_s"] == [0.25, 0.5, 0.75], "Wrong pulse durations")
    require(action["pulse_pattern"] == ["s*a", "-s*a", "-s*a", "s*a"], "Wrong pulse pattern")
    require((action["initial_zero_s"], action["final_zero_s"], action["zero_after_block_s"]) == (1.0, 1.0, 0.5), "Wrong coast intervals")
    units = manifest["units"]
    require(set(units) == set(VECTOR_FIELDS + SCALAR_FIELDS), "Unit metadata does not match fields")
    for suffix in ("", "_next"):
        for key, unit in (("t", "s"), ("altitude", "m"), ("v_up", "m/s"), ("force_up", "N"), ("blimp_thrust", "N")):
            require(units[key + suffix] == unit, f"Wrong unit for {key + suffix}")
        require(units["Eta" + suffix] == ["m"] * 3 + ["rad"] * 3, "Wrong pose units")
        require(units["X" + suffix] == ["m/s"] * 3 + ["rad/s"] * 3, "Wrong velocity units")
    for key, unit in {"U": "normalized thruster command", "u_z": "normalized thruster command",
                      "net_lift_N": "N", "net_lift_g": "g equivalent", "thrust_cmd_N": "N",
                      "episode_id": "index", "step": "index", "seed": "integer",
                      "contact": "boolean", "valid_transition": "boolean"}.items():
        require(units[key] == unit, f"Wrong unit for {key}")
    require(manifest["frames"]["Eta"].startswith("CV_NED"), "Wrong pose frame")
    require(manifest["frames"]["X"].startswith("body-CV"), "Wrong velocity frame")
    require(manifest["frames"]["altitude"] == "lowest gondola assembly clearance", "Wrong altitude reference")
    np.testing.assert_array_equal(manifest["geometry"]["gondola_size_m"], expected_params.blimp.gondola_size)
    require(manifest["geometry"]["thruster_length_m"] == expected_params.blimp.thruster_length, "Wrong motor length")
    require(manifest["geometry"]["thruster_radius_m"] == expected_params.blimp.thruster_radius, "Wrong motor radius")
    root = Path(__file__).resolve().parent
    for relative in ("collect_altitude_data.py", "sim/blimp.py", "sim/params.py"):
        info = manifest["sources"][relative]
        require(info["snapshot"] == f"sources/{relative}", "Unexpected source snapshot path")
        snapshot = (directory / info["snapshot"]).read_bytes()
        require(hashlib.sha256(snapshot).hexdigest() == info["sha256"], f"Source snapshot mismatch: {relative}")
        if relative.startswith("sim/"):
            require((root / relative).read_bytes() == snapshot, f"Active plant source changed: {relative}; use preserved sources for replay")
    episodes = manifest["episodes"]
    require(len(episodes) == 12, "Missing episode provenance")
    require([ep["episode_id"] for ep in episodes] == list(range(12)), "Episode order/IDs mismatch")
    require(len({ep["seed"] for ep in episodes}) == 12, "Duplicate seeds")
    for ep in episodes:
        episode_id = ep["episode_id"]
        require(ep["seed"] == base_seed + episode_id, "Episode seed mismatch")
        require(ep["parameter_overrides"] == {"seed": ep["seed"]}, "Unexpected episode parameter override")
        require(ep["duration_s"] == 60 and ep["transition_count"] == 1200, "Incomplete episode")
        require(ep["initial_altitude"] == INITIAL_HEIGHTS[episode_id], "Unexpected initial height")
        expected_pose = Blimp(expected_params.blimp).pose_at_altitude(INITIAL_HEIGHTS[episode_id])
        np.testing.assert_array_equal(ep["initial_Eta"], expected_pose)
        for field in ("initial_X", "initial_blimp_thrust", "initial_thrust_cmd_N"):
            np.testing.assert_array_equal(ep[field], np.zeros(6))
        require(ep["initial_lift_drift_N"] == 0, "Nonzero initial drift")
        np.testing.assert_array_equal(ep["initial_draught_N"], np.zeros(2))
    return manifest


def inspect_pulses(commands, episode):
    """Validate the recorded block structure independently of the generator."""
    expected = np.zeros(1200)
    previous_stop = 20
    covered = set()
    for block in episode["pulse_blocks"]:
        start, stop, dwell = block["start_step"], block["stop_step"], block["dwell_steps"]
        amplitude, sign = block["amplitude"], block["sign"]
        require(start == previous_stop and stop <= 1180, "Pulse gap/overlap or missing final coast")
        require(dwell in (5, 10, 15) and amplitude in (0.2, 0.4, 0.6, 0.8, 1.0) and sign in (-1, 1), "Invalid pulse")
        require(block["zero_after_steps"] == 10 and stop == start + 4 * dwell + 10, "Invalid pulse/coast duration")
        expected[start:start + 4 * dwell] = np.repeat([sign * amplitude, -sign * amplitude, -sign * amplitude, sign * amplitude], dwell)
        covered.add((amplitude, dwell))
        previous_stop = stop
    require(covered == {(a, d) for a in (0.2, 0.4, 0.6, 0.8, 1.0) for d in (5, 10, 15)}, "Incomplete amplitude/duration coverage")
    np.testing.assert_array_equal(commands, expected)
    np.testing.assert_array_equal(commands, episode["u_z_schedule"])


def inspect_dataset(directory):
    """Check the fixed neutral-trim protocol against independently read arrays."""
    directory = Path(directory)
    manifest = inspect_manifest(directory)
    datasets, summaries, command_hashes = {}, {}, set()
    dt, substeps = 0.05, 5
    params = BlimpParams()
    support = Blimp(params).gondola_bottom_offset()
    for split, expected_ids in SPLITS.items():
        with np.load(directory / f"{split}.npz", allow_pickle=False) as archive:
            data = {key: archive[key] for key in archive.files}
        required = set(VECTOR_FIELDS + SCALAR_FIELDS)
        require(required <= data.keys(), f"{split}: missing fields {required - data.keys()}")
        count = len(data["t"])
        require(count == len(expected_ids) * 1200, f"{split}: wrong number of transitions")
        split_info = manifest["splits"][split]
        require(split_info["file"] == f"{split}.npz" and split_info["episode_ids"] == list(expected_ids), f"{split}: manifest assignment mismatch")
        require(split_info["transition_count"] == count and split_info["episode_count"] == len(expected_ids), f"{split}: manifest count mismatch")
        require(split_info["sha256"] == hashlib.sha256((directory / f"{split}.npz").read_bytes()).hexdigest(), f"{split}: file hash mismatch")
        for key in VECTOR_FIELDS:
            require(data[key].shape == (count, 6), f"{split}/{key}: wrong shape")
        for key in SCALAR_FIELDS:
            require(data[key].shape == (count,), f"{split}/{key}: wrong shape")
        for key in required:
            require(np.all(np.isfinite(data[key])), f"{split}/{key}: non-finite values")
        require(data["valid_transition"].dtype == np.bool_, f"{split}: validity must be boolean")
        require(data["contact"].dtype == np.bool_, f"{split}: contact must be boolean")
        for key in ("episode_id", "step", "seed"):
            require(data[key].dtype.kind in "iu", f"{split}: {key} must be integer")
        require(np.all(data["valid_transition"]), f"{split}: invalid transitions")
        require(not np.any(data["contact"]), f"{split}: contact transitions")
        require(np.array_equal(np.unique(data["episode_id"]), list(expected_ids)), f"{split}: episode assignment")
        np.testing.assert_allclose(data["t_next"] - data["t"], dt, rtol=0, atol=1e-12)
        np.testing.assert_array_equal(data["U"][:, :4], 0)
        np.testing.assert_array_equal(data["U"][:, 4], data["u_z"])
        np.testing.assert_array_equal(data["U"][:, 5], data["u_z"])
        require(np.max(np.abs(data["U"])) <= 1, f"{split}: command out of range")
        np.testing.assert_allclose(data["thrust_cmd_N"], data["U"] * params.T_max, atol=1e-15)
        decay = (1 - 0.01 / params.tau_p) ** substeps
        expected_thrust_next = data["thrust_cmd_N"] + decay * (data["blimp_thrust"] - data["thrust_cmd_N"])
        np.testing.assert_allclose(data["blimp_thrust_next"], expected_thrust_next, atol=1e-14)
        for suffix in ("", "_next"):
            np.testing.assert_allclose(data[f"Eta{suffix}"][:, [0, 1, 3, 4, 5]], 0, atol=1e-12)
            np.testing.assert_allclose(data[f"X{suffix}"][:, [0, 1, 3, 4, 5]], 0, atol=1e-12)
            np.testing.assert_allclose(data[f"altitude{suffix}"], -data[f"Eta{suffix}"][:, 2] - support, atol=1e-12)
            np.testing.assert_allclose(data[f"v_up{suffix}"], -data[f"X{suffix}"][:, 2], atol=1e-12)
            np.testing.assert_allclose(data[f"force_up{suffix}"], data[f"blimp_thrust{suffix}"][:, 4:].sum(axis=1), atol=1e-12)
            height = data[f"altitude{suffix}"]
            require(np.all((height >= 0.4) & (height <= 3.6)), f"{split}: boundary guard crossed")
        np.testing.assert_array_equal(data["net_lift_N"], 0)
        np.testing.assert_array_equal(data["net_lift_g"], 0)
        keys = np.column_stack((data["episode_id"], data["step"]))
        require(len(np.unique(keys, axis=0)) == count, f"{split}: duplicate episode/step keys")
        for episode_id in expected_ids:
            rows = np.flatnonzero(data["episode_id"] == episode_id)
            episode = manifest["episodes"][episode_id]
            require(episode["split"] == split, "Episode split metadata mismatch")
            np.testing.assert_array_equal(data["seed"][rows], episode["seed"])
            np.testing.assert_array_equal(data["Eta"][rows[0]], episode["initial_Eta"])
            np.testing.assert_array_equal(data["X"][rows[0]], episode["initial_X"])
            np.testing.assert_array_equal(data["blimp_thrust"][rows[0]], episode["initial_blimp_thrust"])
            inspect_pulses(data["u_z"][rows], episode)
            np.testing.assert_array_equal(data["step"][rows], np.arange(1200))
            np.testing.assert_allclose(data["t"][rows], np.arange(1200) * dt, atol=1e-12)
            np.testing.assert_allclose(data["t_next"][rows][-1], 60, atol=1e-12)
            require(len(np.unique(data["seed"][rows])) == 1, f"{split}/{episode_id}: inconsistent seed")
            for current, future in (("Eta", "Eta_next"), ("X", "X_next"),
                                    ("blimp_thrust", "blimp_thrust_next"),
                                    ("altitude", "altitude_next"), ("v_up", "v_up_next")):
                np.testing.assert_array_equal(data[future][rows[:-1]], data[current][rows[1:]])
            digest = hashlib.sha256(data["u_z"][rows].tobytes()).hexdigest()
            require(digest not in command_hashes, "Duplicate command schedule across episodes")
            command_hashes.add(digest)
            # Replay the entire flight without resetting to each recorded state.
            blimp = Blimp(BlimpParams())
            blimp.reset(episode["initial_Eta"], episode["initial_X"])
            blimp.thrust[:] = episode["initial_blimp_thrust"]
            for row in rows:
                blimp.command(data["U"][row])
                for _ in range(substeps):
                    blimp.step(0.01)
                    require(0.4 <= blimp.altitude <= 3.6, "Replay crossed boundary guard")
                    require(blimp.floor_alt < blimp.altitude < blimp.ceiling_alt, "Replay touched a contact limit")
                np.testing.assert_allclose(blimp.eta, data["Eta_next"][row], rtol=0, atol=1e-13)
                np.testing.assert_allclose(blimp.nu, data["X_next"][row], rtol=0, atol=1e-13)
                np.testing.assert_allclose(blimp.thrust, data["blimp_thrust_next"][row], rtol=0, atol=1e-13)
        summaries[split] = {
            "episodes": len(expected_ids), "transitions": count,
            "duration_s": count * dt,
            "altitude_range_m": [float(min(data["altitude"].min(), data["altitude_next"].min())),
                                 float(max(data["altitude"].max(), data["altitude_next"].max()))],
            "vertical_velocity_range_m_s": [float(data["v_up"].min()), float(data["v_up"].max())],
            "command_levels": np.unique(data["u_z"]).tolist(),
            "contacts": int(data["contact"].sum()),
        }
        datasets[split] = data
    return datasets, summaries


def plot_data(datasets, directory):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams.update({"font.size": 11, "axes.spines.top": False, "axes.spines.right": False})
    data = datasets["train"]
    rows = np.flatnonzero(data["episode_id"] == 0)
    t = np.r_[data["t"][rows], data["t_next"][rows[-1]]]
    fig, axes = plt.subplots(4, 1, figsize=(12, 9), sharex=True, constrained_layout=True)
    axes[0].plot(t, np.r_[data["altitude"][rows], data["altitude_next"][rows[-1]]], color="#2463a1", label="Training flight 1")
    axes[0].set_ylabel("Gondola height (m)")
    axes[1].plot(t, np.r_[data["v_up"][rows], data["v_up_next"][rows[-1]]], color="#2463a1")
    axes[1].set_ylabel("Upward velocity (m/s)")
    axes[2].step(t, np.r_[data["u_z"][rows], data["u_z"][rows[-1]]], where="post", color="#c26721")
    axes[2].set_ylabel("Vertical command")
    axes[2].set_ylim(-1.1, 1.1)
    commanded = data["thrust_cmd_N"][rows, 4:].sum(axis=1)
    axes[3].step(t, np.r_[commanded, commanded[-1]], where="post", color="#c26721", ls="--", label="Commanded")
    axes[3].plot(t, np.r_[data["force_up"][rows], data["force_up_next"][rows[-1]]], color="#2463a1", label="Realized")
    axes[3].set_ylabel("Upward thrust (N)")
    axes[3].set_xlabel("Time (s)")
    axes[0].legend(frameon=False)
    axes[3].legend(frameon=False, ncol=2)
    axes[-1].set_xlim(0, 60)
    fig.savefig(directory / "example_flight.png", dpi=170)
    fig.savefig(directory / "example_flight.pdf")
    plt.close(fig)

    colors = {"train": "#2463a1", "validation": "#c26721", "test": "#22805e"}
    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True, constrained_layout=True)
    for split, data in datasets.items():
        for index, episode_id in enumerate(np.unique(data["episode_id"])):
            rows = np.flatnonzero(data["episode_id"] == episode_id)
            t = np.r_[data["t"][rows], data["t_next"][rows[-1]]]
            label = split.capitalize() if index == 0 else None
            for ax, current, future in ((axes[0], "altitude", "altitude_next"), (axes[1], "v_up", "v_up_next")):
                ax.plot(t, np.r_[data[current][rows], data[future][rows[-1]]],
                        color=colors[split], alpha=0.65, lw=1.1, label=label)
    axes[0].set_ylabel("Gondola height (m)")
    axes[1].set_ylabel("Upward velocity (m/s)")
    axes[1].set_xlabel("Time in episode (s)")
    axes[0].legend(frameon=False, ncol=3)
    axes[1].set_xlim(0, 60)
    fig.savefig(directory / "all_flights.png", dpi=170)
    fig.savefig(directory / "all_flights.pdf")
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    args = parser.parse_args()
    datasets, summaries = inspect_dataset(args.directory)
    plot_data(datasets, args.directory)
    source = Path(__file__).read_bytes()
    (args.directory / "sources" / Path(__file__).name).write_bytes(source)
    report = {
        "status": "passed", "collection_only": True,
        "checks": ["finite arrays and declared shapes", "unique episode/step keys", "whole-episode splits",
                   "complete final transitions", "0.05 s alignment and episode continuity",
                   "symmetric vertical-only commands and motion", "neutral trim and unit consistency",
                   "no contact or boundary-guard crossings", "exact actuator-lag update",
                   "all 14,400 transitions replayed sequentially, with every physics substep checked",
                   "source snapshots, file hashes, settings, units, initial states, and unique seeds",
                   "complete signed amplitude/duration coverage and recorded command schedules"],
        "splits": summaries, "total_transitions": sum(x["transitions"] for x in summaries.values()),
        "limits": ["simulated vertical flight only", "one fixed neutral-trim parameter setting",
                   "no model has been fitted or evaluated", "simulator thrust states are available without sensor noise",
                   "flight coverage is not a formal excitation or hardware-fidelity guarantee"],
        "checker_sha256": hashlib.sha256(source).hexdigest(),
        "dataset_sha256": {split: hashlib.sha256((args.directory / f"{split}.npz").read_bytes()).hexdigest() for split in SPLITS},
        "manifest_sha256": hashlib.sha256((args.directory / "manifest.json").read_bytes()).hexdigest(),
    }
    (args.directory / "quality.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"status": report["status"], "total_transitions": report["total_transitions"], "splits": summaries}, indent=2))


if __name__ == "__main__":
    main()
