# blimp-bot-interaction — team simulator v1

Pure-NumPy simulator of a GT-MAB-class blimp and N differential-drive rovers, with a 3D
matplotlib viewer and live parameter sliders. Perfect knowledge, no camera, no noise, no
latency (v1 scope). Design documents: `docs/PLAN.md`, `docs/PROBLEM_FORMULATION.md`.

## Run

    python teleop_blimp.py --rovers circle   # fly the blimp with the BlueROV key map (below)
    python teleop_blimp.py --self-test       # headless check of the key-sign conventions

    pip install numpy matplotlib            # ffmpeg needed only for --mp4
    python -m sim.tests.test_blimp          # verification against the paper (see below)
    python verify_dynamics.py               # 8 behavioural checks + results/verify_dynamics.png
    python run_circle.py --gui              # live 3D view + sliders (defaults preset)
    python run_circle.py --T 60 --plots results/circle.png --npz results/circle.npz
    python run_circle.py --T 40 --mp4 results/circle.mp4

Scenario 1: rovers run a circle (radius 1.5 m, 0.2 m/s, equally spaced); the blimp flies to the
circle centre and holds 1 m altitude.

## Keyboard teleop (same map as EDMDc/teleop_tank.py)

    W / S  surge fwd/back   A / D  sway left/right   Q / E  heave up/down   R / F  yaw left/right
    + / -  gain 0.1..1.0    Space  panic (zero cmd)  P  screenshot         Esc  quit
    H      PID hold assist on/off (blimp-only addition; same as the big PID / HOLD button)

Hold-to-move; command = ±gain on each axis, mapped through the same global-scaling allocation the
ArduSub allocator uses (authority per axis printed by `--self-test`). The piloted meaning is kept
(A = left, Q = up) although the blimp body frame is y-right / z-down. Heave keys: Q up / E down by
default (chosen 2026-10-04); `--heave-keys eq` gives E up / Q down, which is what
`EDMDc/teleop_tank.py` has. Keyboard focus must be on
the matplotlib window; a 0.75 s key timeout zeroes the command if a key-release is lost.

## GUI layout (`--gui` and teleop)

    +----------------------------+---------------+------------------------------+
    |                            | attitude HUD  | [ PID / HOLD : ON ]  (green) |
    |        3D scene            | heading tape  | [ RESET ]  [ PAUSE ]         |
    |  blimp, rovers, forces     | stick boxes   | BLIMP PHYSICS   sliders      |
    |                            | telemetry     | CONTROLLER / TASK sliders    |
    |                            | (+ teleop cmd)| VIEW            slider       |
    +----------------------------+---------------+------------------------------+
    | legend                                                                    |
    +---------------------------------------------------------------------------+

The PID / HOLD button is the first control on the panel: green = ON, grey = OFF; click it or
press H. RESET (or R in `run_circle.py --gui`) restarts the sim at t = 0 (teleop: 1 m over the
centre, held keys and hold reference cleared); PAUSE (or space) freezes the physics while the
sliders stay live. Nothing is drawn over the 3D axes any more: numbers go to the telemetry
block under the HUD, and the teleop command / gain / held keys appear there as extra lines.

## Files

    sim/params.py       all parameters; [paper] vs [slider]; SLIDERS list drives the GUI
    sim/blimp.py        6-DoF Fossen-form blimp, 6-thruster BlueROV-style allocation, RK4
    sim/rover.py        exact unicycle + hand-point feedback linearization
    sim/controllers.py  circle tracker (rovers), position PD on the blimp's centre of mass
    sim/sim.py          TeamSim: reset/step/run, 100 Hz physics / 20 Hz control, npz logging
    sim/viewer.py       3D viewer, HUD, force arrows, sliders, headless video
    sim/keyboard.py     keyboard controller (BlueROV key map, authority, global scaling)
    teleop_blimp.py     keyboard teleop entry point (+ --self-test)
    sim/tests/          verification
    run_circle.py       scenario entry point

## Model

Frames as in Tao et al. (ICARCV 2018): inertial NED (z down, floor z = 0), body frame at the
centre of volume (CV = CB), x forward, y right, z down. Centre of mass d_VM = 0.0971 m below CV;
thrust plane d_VT = 0.26 m below CV. Equations: M ν̇ + C(ν)ν + D(ν)ν + g(η) = τ, η̇ = J(η)ν with
M = M_RB(CM offset) + M_A, Fossen's general C(M, ν), linear + quadratic drag, neutral trim.
Paper values: m = 0.1249 kg, I_CM = 0.005821 kg m², b = 0.00098 N m s/rad. Unmeasured values
(yaw inertia, translational drag, added mass, thrust, lag, quadratic yaw damping) are sliders.
Default `T_max` is 0.03 N per thruster: 0.1 N made the 125 g blimp reach 1.2 m/s and swing 30°,
which is not the paper's hover regime. The blimp is clamped between `floor_alt` and `ceiling_alt`.

Thrusters: four horizontal at ±45° at (±l_h, ±l_h, d_VT) and two vertical at (±l_v, 0, d_VT),
BlueROV2 pattern. Only [F_x, F_y, F_z, τ_z] are allocated (pseudo-inverse of the reduced 4×6
map); roll/pitch torques induced by the thrust plane below CV are left to the physics — this is
the paper's swing mechanism and is reproduced, not hand-coded.

Rovers: exact unicycle integration; (v, ω) achieved instantly; the hand point d_hand ahead of
the axle is holonomic and is what the tracker controls.

## Verification (`python -m sim.tests.test_blimp`)

* Free swing from 15° with added mass and translational drag off matches the paper's pendulum
  I_CM θ̈ = −b θ̇ − m g d_VM sin θ with NRMSE fit 100.00 %; period 1.393 s vs 1.390 s from
  Eq. (21).
* Constant surge thrust from rest: q̇(0) = (d_VT − d_VM) f / I_CM exactly, and the centre of
  mass accelerates at f/m.
* Allocator round-trip on surge/sway/heave/yaw; induced pitch torque for 0.05 N surge = 0.013 N m.
* Unicycle: one full turn closes to 1e-14 m.

## Behavioural checks (`python verify_dynamics.py`, all PASS, figure `results/verify_dynamics.png`)

* Free swing from 15° follows the paper pendulum (fit 99.5 % at the 20 Hz log rate).
* Thrusters off: B − W = +0.02 N rises (2.65 m after 8 s), −0.02 N sinks to the floor, 0 stays.
* Heavy blimp (B − W = −0.03 N): the PID holds 1.00 m with 0.030 N of upward thrust (= |B − W|),
  but the transient is slow: it dips to 0.33 m first and is inside 2 cm only after 33 s, because
  the integral term (ki_pos = 0.004 N/(m s)) has to accumulate the whole 0.03 N. PID OFF: sinks to
  the floor; ON again: back to 1 m. A trim heavier than the heave authority 2·T_max = 0.06 N
  cannot be held at all (thrust saturates at 0.060 N, blimp stays on the floor).
* A 1 s surge burst excites a 7–8° swing that then decays at 0.091 1/s vs the paper's
  b/(2 I_CM) = 0.084 1/s.
* PD on the centre of volume pumps the swing (7.8° → 19.7° in 60 s); PID on the centre of mass
  lets it decay to 0.
* Teleop: W held for 10 s raises the altitude 0.42 m with the hold OFF (the thrust plane tilts
  with the nose-up swing, so surge thrust gets an upward component); with the hold ON the
  deviation is 0.13 m and the release point is recovered.

## Finding from the first runs (worth remembering)

A PD on the centre of volume slowly pumps the swing: the CV moves with the pendulum, the
velocity feedback drives the thrusters, and the thrusters sit below CV, so the loop adds energy
to a mode with damping ratio ≈ 0.02 (pitch grew from 3° to 7° in 40 s). Controlling the centre
of mass instead removes the feedback path; the swing excited by manoeuvres then decays on its
own (4.8° → 0 over ~40 s). This is the simulated version of the oscillation the paper set out to
fix and is the first candidate for a learned/explicit swing module later.

## Trim and the blimp controller

`blimp.net_lift_N` is B − W (positive rises; 0 = exactly neutral, the paper's assumption) and
`blimp.net_lift_drift` makes it a random walk (helium loss, temperature). The blimp controller is
a PID on the centre of mass with anti-windup (`task.kp_pos, kd_pos, ki_pos, i_max_N`); set
`ki_pos = 0` for the original PD. `python exp_net_lift.py` shows why the integral term exists:
with +0.01 N of lift the PD holds 0.29 m too high (= lift / kp_pos), the PID returns to 1.00 m,
and with a drifting trim the PID tracks the drift with a lag set by ki_pos.

Sliders take effect immediately: every parameter is read from `SimParams` at each physics step
(drag, lag, thrust, lift, gains, task), and the ones baked into the mass matrix (added mass, yaw
inertia) trigger `Blimp.rebuild()` from the slider callback. (`net_lift_N` was cached at reset in
the first version and ignored the slider — fixed; it is now a live property plus the drift.)

PID on/off is live: the PID / HOLD button (or H) flips `task.pid_enabled`.
In the scenario, OFF idles the thrusters (a heavy blimp sinks to the floor, a light one rises); ON
re-engages the PID with a cleared integrator. In teleop, ON is a hold assist: the reference is
captured where the blimp is when switched on; any axis with a key held is manual (its reference
follows the vehicle), idle axes are held — so altitude is held while you drive with W/A/S/D, and
the release point is held when you let go. `--hold` starts teleop with it on.

There is no inner (rate) loop: PID -> desired wrench -> allocator -> six thrust commands ->
first-order thruster lag -> physics. Rovers have no low-level loop in v1 (commanded (v, ω) are
achieved instantly).

## Conventions

Logs: `t, X, U, X_next, Eta` for the blimp (ν, u ∈ [−1,1]⁶, η) plus `rover_X, rover_U,
rover_X_next, rover_ref`, as in the underwater repo. Seeds via `SimParams.seed`.
