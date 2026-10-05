"""Collect open-loop vertical blimp transitions without fitting a model.

Run ``python collect_altitude_data.py --output-dir results/altitude_<name>``.
Each held action spans five existing 0.01 s physics steps. The saved transition
contains the state immediately before commanding and immediately after all five
steps, including the terminal transition. Units remain SI except net_lift_g.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil

import numpy as np

from sim.blimp import Blimp, R_zyx
from sim.params import G, SimParams


@dataclass(frozen=True)
class EpisodeSpec:
    episode_id: int
    split: str
    initial_altitude: float


EPISODES = tuple(
    [EpisodeSpec(i, "train", height) for i, height in enumerate([1.25, 1.75, 2.25, 2.75] * 2)]
    + [EpisodeSpec(8, "validation", 1.5), EpisodeSpec(9, "validation", 2.5),
       EpisodeSpec(10, "test", 1.35), EpisodeSpec(11, "test", 2.65)]
)
AMPLITUDES = (0.2, 0.4, 0.6, 0.8, 1.0)
DWELLS_S = (0.25, 0.5, 0.75)
GUARD_ALTITUDE = (0.4, 3.6)
VALID_ALTITUDE = (0.2, 3.8)
SPLITS = ("train", "validation", "test")
SCHEMA_VERSION = "altitude-transitions-v1"

UNITS = {
    "t": "s", "t_next": "s", "episode_id": "index", "step": "index", "seed": "integer",
    "Eta": ["m", "m", "m", "rad", "rad", "rad"],
    "Eta_next": ["m", "m", "m", "rad", "rad", "rad"],
    "X": ["m/s", "m/s", "m/s", "rad/s", "rad/s", "rad/s"],
    "X_next": ["m/s", "m/s", "m/s", "rad/s", "rad/s", "rad/s"],
    "U": "normalized thruster command", "blimp_thrust": "N", "blimp_thrust_next": "N", "thrust_cmd_N": "N",
    "altitude": "m", "altitude_next": "m", "v_up": "m/s", "v_up_next": "m/s",
    "force_up": "N", "force_up_next": "N", "u_z": "normalized thruster command",
    "net_lift_N": "N", "net_lift_g": "g equivalent", "valid_transition": "boolean", "contact": "boolean",
}
VECTOR_FIELDS = {"Eta", "Eta_next", "X", "X_next", "U", "blimp_thrust", "blimp_thrust_next", "thrust_cmd_N"}


class CollectionGuardError(RuntimeError):
    """Collection cannot be accepted; no partial dataset should be written."""


def integer_steps(duration, dt):
    count = int(round(duration / dt))
    if count < 0 or not np.isclose(count * dt, duration, atol=1e-12, rtol=0):
        raise ValueError("Durations must be exact multiples of the control timestep")
    return count


def build_command_schedule(seed, duration_s=60.0, dt_ctrl=0.05):
    """Shuffled, signed pulse blocks with full initial/final zero intervals.

    A block is [s*a, -s*a, -s*a, s*a], equal dwell per pulse, then 0.5 s
    at zero. Each shuffled pool contains every amplitude/dwell combination.
    Near the end, the first remaining block that fits is selected; no block is
    truncated. The unused tail is zero, including the final reserved second.
    """
    n_steps = integer_steps(duration_s, dt_ctrl)
    initial_zero = final_zero = integer_steps(1.0, dt_ctrl)
    zero_after = integer_steps(0.5, dt_ctrl)
    if n_steps < initial_zero + final_zero:
        raise ValueError("An episode must allow the initial and final one-second zero intervals")
    rng = np.random.default_rng(seed)
    combinations = [(amplitude, integer_steps(dwell, dt_ctrl)) for amplitude in AMPLITUDES for dwell in DWELLS_S]
    pending = []
    schedule = np.zeros(n_steps, dtype=np.float64)
    blocks = []
    cursor, stop = initial_zero, n_steps - final_zero
    while cursor < stop:
        if not pending:
            pending = [combinations[index] for index in rng.permutation(len(combinations))]
        fitting = next((i for i, (_, dwell) in enumerate(pending) if 4 * dwell + zero_after <= stop - cursor), None)
        if fitting is None:
            break
        amplitude, dwell = pending.pop(fitting)
        sign = int(rng.choice((-1, 1)))
        block_start = cursor
        for value in (sign * amplitude, -sign * amplitude, -sign * amplitude, sign * amplitude):
            schedule[cursor:cursor + dwell] = value
            cursor += dwell
        cursor += zero_after
        blocks.append({"start_step": block_start, "stop_step": cursor, "amplitude": amplitude,
                       "sign": sign, "dwell_steps": dwell, "zero_after_steps": zero_after})
    return schedule, blocks


def collection_params(seed):
    params = SimParams(seed=seed)
    params.task.n_rovers = 0
    params.task.pid_enabled = False
    params.blimp.net_lift_N = 0.0
    params.blimp.net_lift_drift = 0.0
    params.blimp.draught_N = 0.0
    return params


def _measure(blimp):
    rotation = R_zyx(*blimp.eta[3:6])
    return {
        "Eta": blimp.eta.copy(), "X": blimp.nu.copy(), "blimp_thrust": blimp.thrust.copy(),
        "altitude": float(blimp.altitude), "v_up": float(-(rotation @ blimp.nu[:3])[2]),
        "force_up": float(-(rotation @ blimp.wrench(blimp.thrust)[:3])[2]),
    }


def _check_guard(blimp, episode_id, step, substep):
    height = blimp.altitude
    state = np.r_[blimp.eta, blimp.nu, blimp.thrust, height]
    if not np.all(np.isfinite(state)) or not GUARD_ALTITUDE[0] <= height <= GUARD_ALTITUDE[1]:
        raise CollectionGuardError(
            f"Episode {episode_id}, step {step}, physics substep {substep}: "
            f"altitude {height!r} outside {GUARD_ALTITUDE}, or non-finite state. No dataset accepted."
        )


def collect_episode(spec, seed, duration_s=60.0):
    """Return one complete episode and its replay instructions; never truncate."""
    params = collection_params(seed)
    substeps = integer_steps(params.dt_ctrl, params.dt_phys)
    schedule, blocks = build_command_schedule(seed, duration_s, params.dt_ctrl)
    blimp = Blimp(params.blimp)
    blimp.floor_alt, blimp.ceiling_alt = params.floor_alt, params.ceiling_alt
    blimp.reset(eta=blimp.pose_at_altitude(spec.initial_altitude))
    initial = _measure(blimp)
    _check_guard(blimp, spec.episode_id, 0, 0)
    simulation_rng = np.random.default_rng(seed)
    rows = {name: [] for name in UNITS}
    for step, u_z in enumerate(schedule):
        before = _measure(blimp)
        action = np.array([0.0, 0.0, 0.0, 0.0, u_z, u_z])
        blimp.command(action)
        commanded_thrust = blimp.thrust_cmd.copy()
        contact = False
        for substep in range(1, substeps + 1):
            blimp.step(params.dt_phys, rng=simulation_rng)
            contact |= blimp.altitude <= params.floor_alt + 1e-10 or blimp.altitude >= params.ceiling_alt - 1e-10
            _check_guard(blimp, spec.episode_id, step, substep)
        after = _measure(blimp)
        valid = (VALID_ALTITUDE[0] <= before["altitude"] <= VALID_ALTITUDE[1]
                 and VALID_ALTITUDE[0] <= after["altitude"] <= VALID_ALTITUDE[1] and not contact)
        row = {
            "t": step * params.dt_ctrl, "t_next": (step + 1) * params.dt_ctrl,
            "episode_id": spec.episode_id, "step": step, "seed": seed,
            "U": action, "thrust_cmd_N": commanded_thrust, "u_z": float(u_z),
            "net_lift_N": float(blimp.lift), "net_lift_g": float(1000.0 * blimp.lift / G),
            "valid_transition": valid, "contact": contact,
        }
        for name, value in before.items():
            row[name] = value
            row[name + "_next"] = after[name]
        for name, value in row.items():
            rows[name].append(value)
    arrays = {}
    for name, values in rows.items():
        dtype = np.bool_ if name in ("valid_transition", "contact") else np.int64 if name in ("episode_id", "step", "seed") else np.float64
        arrays[name] = np.asarray(values, dtype=dtype)
        if name in VECTOR_FIELDS:
            arrays[name] = arrays[name].reshape((-1, 6))
    episode_manifest = {
        **asdict(spec), "seed": seed, "parameter_overrides": {"seed": seed},
        "duration_s": duration_s, "transition_count": len(schedule),
        "initial_Eta": initial["Eta"].tolist(), "initial_X": initial["X"].tolist(),
        "initial_blimp_thrust": initial["blimp_thrust"].tolist(),
        "initial_thrust_cmd_N": [0.0] * 6, "initial_lift_drift_N": 0.0, "initial_draught_N": [0.0, 0.0],
        "u_z_schedule": schedule.tolist(), "pulse_blocks": blocks,
    }
    return arrays, episode_manifest


def collect_episodes(seed=42, duration_s=60.0, episode_specs=EPISODES):
    """Collect in memory; the optional duration/specs support bounded regression tests."""
    specs = tuple(episode_specs)
    if not specs or len({spec.episode_id for spec in specs}) != len(specs):
        raise ValueError("Episode IDs must be unique and at least one episode is required")
    if any(spec.split not in SPLITS or spec.episode_id < 0 for spec in specs):
        raise ValueError("Invalid episode ID or split")
    if not isinstance(seed, int) or isinstance(seed, bool) or not 0 <= seed <= 2**32 - 1 - max(spec.episode_id for spec in specs):
        raise ValueError("Base seed plus each episode ID must fit an unsigned 32-bit seed")
    grouped = {split: [] for split in SPLITS}
    episodes = []
    for spec in specs:
        arrays, manifest = collect_episode(spec, seed + spec.episode_id, duration_s)
        grouped[spec.split].append(arrays)
        episodes.append(manifest)
    splits = {}
    for split, episode_arrays in grouped.items():
        if episode_arrays:
            splits[split] = {name: np.concatenate([episode[name] for episode in episode_arrays], axis=0) for name in UNITS}
    return splits, episodes


def _source_snapshots():
    root = Path(__file__).resolve().parent
    relative_paths = ("collect_altitude_data.py", "sim/blimp.py", "sim/params.py")
    return {relative: (root / relative).read_bytes() for relative in relative_paths}


def write_dataset(output_dir, splits, episodes, seed=42, duration_s=60.0):
    """Write completed episodes into a new directory; refuse any existing path."""
    output_dir = Path(output_dir)
    if output_dir.exists():
        raise FileExistsError(f"Refusing to overwrite existing output: {output_dir}")
    if set(splits) != set(SPLITS):
        raise ValueError("A saved dataset must contain train, validation, and test splits")
    for split, arrays in splits.items():
        if not np.all(arrays["valid_transition"]) or np.any(arrays["contact"]):
            raise CollectionGuardError(f"Split {split} contains invalid/contact transitions; no dataset accepted")
    snapshots = _source_snapshots()
    params = collection_params(seed)
    manifest = {
        "schema_version": SCHEMA_VERSION, "created_utc": datetime.now(timezone.utc).isoformat(),
        "base_seed": seed, "simulation_parameters": asdict(params), "gravity_m_s2": G,
        "duration_per_episode_s": duration_s, "episode_count": len(episodes),
        "timing": {"dt_phys_s": params.dt_phys, "dt_ctrl_s": params.dt_ctrl,
                   "physics_steps_per_transition": integer_steps(params.dt_ctrl, params.dt_phys),
                   "state_timing": "current before command; next after five physics steps; terminal transition included"},
        "frames": {"Eta": "CV_NED: x,y,z,roll,pitch,yaw", "X": "body-CV: u,v,w,p,q,r",
                   "altitude": "lowest gondola assembly clearance", "v_up": "negative world CV vertical velocity; equals clearance rate for level motion",
                   "force_up": "negative world vertical realized thruster force"},
        "units": UNITS,
        "geometry": {"gondola_size_m": params.blimp.gondola_size.tolist(),
                     "thruster_length_m": params.blimp.thruster_length, "thruster_radius_m": params.blimp.thruster_radius},
        "action": {"description": "Open-loop symmetric vertical commands U[4]=U[5]=u_z; U[0:4]=0. No PID.",
                   "amplitudes": list(AMPLITUDES), "dwell_s": list(DWELLS_S), "pulse_pattern": ["s*a", "-s*a", "-s*a", "s*a"],
                   "zero_after_block_s": 0.5, "initial_zero_s": 1.0, "final_zero_s": 1.0,
                   "schedule_generation": "Independent NumPy default_rng(episode seed), shuffled amplitude/dwell pools, random block sign; only whole blocks; remaining tail zero",
                   "simulation_rng": "A separate NumPy default_rng with the same episode seed"},
        "guards": {"abort_altitude_m": list(GUARD_ALTITUDE), "valid_endpoint_altitude_m": list(VALID_ALTITUDE),
                   "contact_detection": "Check gondola floor/ceiling clearance after every physics substep", "retry_or_feedback": False},
        "limitations": ["Vertical-only, level initial states; no lateral or attitude excitation", "Neutral trim, no drift or draught", "No payload mass or inertia changes", "No rover dynamics", "No model fitted or performance claim"],
        "splits": {split: {"file": f"{split}.npz", "episode_ids": [episode["episode_id"] for episode in episodes if episode["split"] == split],
                           "episode_count": sum(episode["split"] == split for episode in episodes),
                           "transition_count": len(arrays["t"])} for split, arrays in splits.items()},
        "episodes": episodes,
        "sources": {relative: {"sha256": hashlib.sha256(content).hexdigest(), "snapshot": f"sources/{relative}"}
                    for relative, content in snapshots.items()},
    }
    output_dir.mkdir(parents=True, exist_ok=False)
    try:
        for split, arrays in splits.items():
            target = output_dir / f"{split}.npz"
            np.savez_compressed(target, **arrays)
            manifest["splits"][split]["sha256"] = hashlib.sha256(target.read_bytes()).hexdigest()
        for relative, content in snapshots.items():
            target = output_dir / "sources" / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(content)
        (output_dir / "manifest.json").write_text(json.dumps(manifest, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    except Exception:
        shutil.rmtree(output_dir)
        raise
    return manifest


def collect_dataset(output_dir, seed=42):
    if Path(output_dir).exists():
        raise FileExistsError(f"Refusing to overwrite existing output: {output_dir}")
    splits, episodes = collect_episodes(seed=seed)
    return write_dataset(output_dir, splits, episodes, seed=seed)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=42, help="Base seed; each episode uses base + episode ID")
    args = parser.parse_args()
    try:
        manifest = collect_dataset(args.output_dir, seed=args.seed)
    except (CollectionGuardError, ValueError, FileExistsError) as exc:
        parser.exit(1, f"Collection stopped: {exc}\n")
    count = sum(split["transition_count"] for split in manifest["splits"].values())
    print(f"Saved {manifest['episode_count']} episodes / {count} transitions to {args.output_dir.resolve()}")


if __name__ == "__main__":
    main()
