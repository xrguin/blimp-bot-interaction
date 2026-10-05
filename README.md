# blimp-bot-interaction — team simulator v1

Pure-NumPy simulator of a GT-MAB-class blimp and N differential-drive rovers, with an offline
browser control panel and a Matplotlib viewer. Ground-truth control, simulated onboard vision, no sensor noise, no
latency (v1 scope). Design documents: `docs/PLAN.md`, `docs/PROBLEM_FORMULATION.md`.

## Browser GUI (recommended)

Install the Python dependencies once, from the repository root (tested with Python 3.11):

```sh
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements-web.txt
.venv/bin/python web_server.py
```

The last command opens **http://127.0.0.1:8000**. On macOS, double-click
**Start Web GUI.command** for later launches. Keep the terminal open; Ctrl+C stops the server.
Use `--no-browser` to open the address yourself, or `--port 8001` if port 8000 is occupied.
Optional `--n 6 --seed 1 --mode auto --rovers circle` changes the starting scenario.

**No Wi-Fi or internet is needed after installation.** The Python server, HTML/CSS/JavaScript,
and bundled Three.js assets all run on this computer. There are no CDN, cloud, or font downloads.
The server listens only on the local loopback address. A browser with WebGL support is needed
for the 3D scene. The original Matplotlib viewer is still available below.

The simulation starts **paused**, with four rovers in the circle scenario and the blimp at
**(x, y, height) = (0, 0, 0)**, its lowest gondola surface touching the ground. Keyboard mode
and PID hold are enabled. Click **Run** to begin; enter a desired altitude or use Q to take off.
Launching with `--mode auto` starts from the same position and targets 1 m by default.

Altitude means clearance beneath the lowest point of the gondola assembly, including motor
housings. The readout, desired-altitude field, and ground limit use this same reference,
including when the blimp tilts. A desired altitude of **0 m** means ground contact.

- **Scene:** drag to orbit, scroll to zoom, and use Reset view to restore the camera.
  Rover trails show recent motion; arrows show heading and realized thruster force.
- **Blimp controls:** the guide directly below Live World shows movement keys, PID hold,
  release-input and pause shortcuts. Select Teleop, press Run, then click the 3D view
  before holding movement keys; movement directions follow the blimp's heading.
- **Vehicle information:** large altitude and desired-altitude fields, attitude, position,
  body velocity, manual commands, net lift, and thruster force, with units.
- **Controls:** choose keyboard or automatic flight, circle or idle rovers, PID hold,
  Run/Pause, Reset, and manual gain. Switching flight mode preserves the current vehicle state.
  Reset restarts the selected scenario, clears the in-memory log, and pauses; live parameter
  values and explicitly entered altitude targets are retained.
- **Parameters:** every slider has a numeric field. Enter a decimal or scientific-notation value
  and press Enter (or click outside) to apply it within the displayed limits. Invalid values
  show an error. The desired-altitude box uses the same target as its parameter slider.
  In keyboard mode, it changes the hold target without enabling hold. Manual Q/E movement
  overrides that target and returns to holding the release height.
- **Export:** Download log saves the existing NPZ transition format; Capture scene saves a PNG.
  Logs are held in memory until downloaded and are cleared by Reset or server shutdown.

If a numerical failure occurs, the session resets and pauses with an error message; its
in-memory log is cleared. Press Reset to acknowledge the failure before running again.

Browser flight keys:

| Keys | Action |
| --- | --- |
| W / S | Forward / backward |
| A / D | Left / right |
| Q / E | Up / down |
| F / R | Yaw left / right |
| H | Toggle PID hold |
| + / − | Increase / decrease manual gain |
| Space | Release manual commands; hold remains active if enabled |
| Escape | Release commands and pause |
| P | Capture the scene as a PNG |

Click the scene to use flight keys. Typing in a field does not fly the blimp. Changing focus,
hiding the tab, or losing the connection clears held keys; the server also expires stale
key messages after 0.35 s. Closing/disconnecting the controlling tab pauses the simulation.
For the local launcher, additional tabs are read-only; close the controlling tab and reload
another tab to take control. The public demo gives each page a separate flight.

The Python worker retains the existing 100 Hz physics and 20 Hz control steps independently
of browser drawing. The browser receives state at 20 Hz and draws the scene on its own
animation loop. This removes the Matplotlib widget redraw path from flight control; actual
display smoothness still depends on the computer and browser.

Browser-backend checks (the extra dependency is only needed for testing):

```sh
.venv/bin/python -m pip install -r requirements-web-dev.txt
.venv/bin/python -m unittest sim.tests.test_altitude sim.tests.test_web
```

## Online demo on Render Free

Public hosting is explicitly enabled with `BLIMP_PUBLIC=1`; the local launcher above
continues to work offline. Each public page gets a separate paused flight and camera
recording. The repository remains private, and only `web/` assets and the session APIs
are served. Existing research results are not exposed by the website.

Render service settings:

- Runtime: Python 3; `.python-version` selects the latest Python 3.11 patch.
- Plan: Free; region: Ohio; automatic deployment: off.
- Repository branch: `codex/render-demo`.
- Build: `pip install -r requirements-web.txt`.
- Start: `python -m uvicorn web_server:create_deployment_app --factory --host 0.0.0.0 --port $PORT --workers 1 --ws-max-size 8192`.
- Environment: `BLIMP_PUBLIC=1`, `BLIMP_MAX_SESSIONS=2`, `OPENBLAS_NUM_THREADS=1`,
  `OMP_NUM_THREADS=1`, and `PYTHONUNBUFFERED=1`.
- Render supplies `RENDER_EXTERNAL_HOSTNAME`, from which the allowed HTTPS origin is
  derived. An alternative hostname requires an explicit `PUBLIC_ORIGIN`.
- Health endpoint: `/health`. Run exactly one worker because visitor sessions are in memory.

The first deployment admits two simultaneous flights. This is a capacity limit, not
a real-time performance guarantee. Sessions end after 30 minutes, after 10 minutes
without activity, or after a one-minute disconnection grace period. After expiry,
press **Start new flight**; a page reload also creates a fresh flight. Download useful
data before closing the page. A server restart or redeployment ends all sessions.

Public logs pause at 12,000 control steps (10 simulated minutes) and require Reset
to continue. The existing NPZ format is unchanged: N logged states yield N-1 rows.
Camera recording remains limited to 30 simulated seconds. The server retains only
states/actions; image rendering and ZIP generation happen in the visitor's browser.
Each server export is limited to 32 MiB. Data is not permanently saved on the server.

Render Free can sleep after 15 minutes without incoming traffic and take about a minute
to wake. CPU and monthly usage limits make this a small demonstration service. See
[Free service limits](https://render.com/docs/free) and
[FastAPI deployment](https://render.com/docs/deploy-fastapi).

## Matplotlib and headless runs

    python teleop_blimp.py --rovers circle   # fly the blimp with the adapted BlueROV key map (below)
    python teleop_blimp.py --self-test       # headless check of the key-sign conventions

    pip install numpy matplotlib            # ffmpeg needed only for --mp4
    python -m sim.tests.test_blimp          # verification against the paper (see below)
    python verify_dynamics.py               # 8 behavioural checks + results/verify_dynamics.png
    python run_circle.py --gui              # live 3D view + sliders (defaults preset)
    python run_circle.py --T 60 --plots results/circle.png --npz results/circle.npz
    python run_circle.py --T 40 --mp4 results/circle.mp4

Scenario 1: rovers run a circle (radius 1.5 m, 0.2 m/s, equally spaced); the blimp flies to the
circle centre and holds 1 m altitude.

## Altitude prediction lesson: collect data

The first lesson records **simulated vertical flight at neutral trim**. It does not fit a model.
Run the collector and its independent checker from the repository root:

```sh
.venv/bin/python collect_altitude_data.py --output-dir results/altitude_lesson/my_run --seed 42
MPLCONFIGDIR=/tmp/blimp-mpl .venv/bin/python check_altitude_data.py results/altitude_lesson/my_run
```

Use a fresh output directory; the collector refuses to overwrite an existing one. NumPy is
required for collection; Matplotlib is required for the checker's plots. Collection runs
faster than real time and does not use or reset the browser session.

- Twelve independent 60 s flights give 14,400 transitions at 20 Hz, with 100 Hz physics.
  Eight whole flights are assigned to training, two to validation, and two to testing before
  collection. Keep those episode boundaries when later creating history windows or rollouts.
- Each flight starts level and airborne. A seeded sequence of balanced upward/downward
  thrust pulses drives the two vertical thrusters equally, with short idle intervals.
  PID, horizontal commands, draught, and lift drift are off; net lift is 0 g.
- Every row records the state and realized thrust **before** a command, the command held for
  five physics steps, and the resulting state and thrust. The final transition is included.
  The collector aborts if a flight leaves the 0.4–3.6 m clearance guard region.
- Outputs are `train.npz`, `validation.npz`, `test.npz`, and `manifest.json`. The manifest
  records units, full settings, seeds, command schedules, initial conditions, and source
  hashes. `sources/` preserves the collection code and the plant/parameter source used.
  The checker adds `quality.json`, `example_flight.png/.pdf`, and `all_flights.png/.pdf`.

For the first predictor, a useful state is **[gondola height, upward velocity, realized upward
thrust]**, with the normalized vertical command as its action. The realized thrust matters
because the motors have a 0.1 s lag. These are the `altitude`, `v_up`, `force_up`, and `u_z`
arrays; the three corresponding state `_next` arrays are prediction targets, not input features. Heights
are in m, velocities in m/s, and forces in N. Positive velocity, force, and command mean up.
Raw `Eta` retains CV/NED pose and `X` retains the six body velocities; they are not the
three-component lesson state.

This dataset covers one fixed simulator configuration, pure vertical flight, and noise-free
simulator state. It does not establish performance under changed loads, swinging, sensor
noise, or hardware conditions. No prediction accuracy has been measured yet.

## XIAO ESP32-S3 Sense / OV2640 camera

The browser includes a downward-facing camera beneath the gondola, with its image top
pointing toward the blimp's nose. It follows the blimp's position and full attitude. The
default optical centre is 2 mm below the gondola box, 0.312 m body-down from the centre
of volume; at ground startup it is about 13 mm above the floor. Take off to see the room.
Camera placement is a visual sensor setting; it does not add payload mass or change ground
contact geometry. The textured floor is an illustrative scene, not a model of your room.

**Defaults are estimates, not a hardware calibration:** 640 × 480 pixels, 10 frames per
simulated second, 60° horizontal field of view, centred principal point, and zero lens
distortion. The user confirmed an OV2640; Seeed also ships a newer OV3660 variant. The
[Seeed hardware guide](https://wiki.seeedstudio.com/xiao_esp32s3_getting_started/) lists
1600 × 1200 for OV2640. Its [camera examples](https://wiki.seeedstudio.com/xiao_esp32s3_camera_usage/)
use selectable image sizes and JPEG settings; the simulator's frame rates and optics
are configurable choices, not measurements of the board's throughput or lens.

The camera panel provides **Save preview**, **Record**, **Stop**, and **Download frames +
data**. Start the simulation with Run if it is paused. Recording retains the initial
frame, frames at the chosen 5/10/20 Hz simulation cadence, and the terminal state. It
stops after 30 simulated seconds, or on Stop, Reset, disconnect, or a numerical error.
Pauses do not produce duplicate frames. Completed recordings remain available until
Discard or server shutdown; a new recording cannot overwrite one. A numerical failure
retains only the valid intervals preceding the failure.

Download renders every saved pose in an independent hidden scene and produces a ZIP:

- `frames/000000.png`, …: lossless camera images, without telemetry, trails, force arrows,
  the reference circle, or the debug grid. Physical geometry and shadows remain visible.
- `recording.json`: the camera profile, geometry, camera/blimp/rover poses, simulation
  clocks, and every 20 Hz control interval, including pre/post states, actual applied
  actions, requested rover actions, actuator targets, and parameter snapshots.
- `manifest.json`: image timestamps, absolute/relative control steps, indices of the
  control intervals leading to the next image, camera intrinsics/pose, valid-pixel
  fraction, and SHA-256 hashes of images, profile, and state/action data.

Live preview uses the latest state, is capped at 640 × 480 and 10 Hz to limit the effect
on flight controls, and may skip frames if the browser is busy. **Save preview** saves
that displayed resolution with its actual capture time and control step in the filename.
Recordings retain the selected full resolution and cadence (up to 1600 × 1200 at 20 Hz). The
recording dataset is rendered from the exact retained simulation states, independently
of display refresh. A final image can close a shorter-than-normal camera interval;
use its timestamp rather than assuming every interval has the same length. Export
shows progress, can be cancelled, and keeps the recording for retry. Large resolutions
increase export time and browser memory use. This records motion accurately in time;
it is not a full RNG/controller checkpoint for replaying arbitrary disturbed flights.

### Applying your calibration

Open **Camera settings & calibration**. Resolution and frame rate can be changed there,
along with focal lengths, principal point, distortion, and lens position. **Export
settings** saves the complete profile. [config/camera_ov2640.example.json](config/camera_ov2640.example.json)
is the same uncalibrated starting profile. **Import calibration / settings** accepts
that full format, or this compact JSON format from an OpenCV-style calibration:

```json
{
  "image_width": 640,
  "image_height": 480,
  "camera_matrix": [[600.0, 0.0, 319.5], [0.0, 600.0, 239.5], [0.0, 0.0, 1.0]],
  "distortion_coefficients": [-0.1, 0.01, 0.0, 0.0, 0.0]
}
```

The numbers above only illustrate the file format. Replace them with your measured
values. Compact calibration import marks the optics as user-calibrated and retains
the current mounting pose. The **User optics** badge and `calibration_status` describe
only intrinsics/distortion; the default mounting pose remains an estimate until measured.
Full profile import preserves its `calibration_status`.
Changing intrinsics/distortion manually marks the optics as estimated again.

Conventions follow the [OpenCV pinhole model](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html):
`K` is a 3 × 3, zero-skew intrinsic matrix at `reference_width/reference_height`; `D`
is `[k1, k2, p1, p2, k3]`. The renderer applies Brown–Conrady distortion through inverse
sampling, with an expanded ideal image to avoid clipping normal barrel-distorted
corners. Unsupported/invertibility-limited pixels are black and their fraction is
reported. Fisheye and rational 8/12/14-coefficient calibration models are not accepted.
Output modes are 320 × 240, 640 × 480, 800 × 600, and 1600 × 1200.

Pixel centres use integer coordinates starting at zero. Same-aspect output resizing
uses `fx' = s fx`, `fy' = s fy`, `cx' = s(cx + 0.5) - 0.5`, and the corresponding `cy`
formula. A different sensor crop, mirror/flip setting, or firmware readout mode needs
its own calibration; changing resolution here assumes resizing the same field of view.

World and body coordinates are NED, with body x forward, y right, z down, referenced
to the centre of volume. Optical axes are x right, y down, z forward. Full profile
`translation_body_m` locates the lens; `rotation_optical_to_body` maps optical vectors
into body coordinates. The default rotation is `[[0,-1,0],[1,0,0],[0,0,1]]`. Exact camera
poses use `p_NC = p_NB + R_NB r_BC` and `R_NC = R_NB R_BC`.

The current model reproduces geometric projection, mounting motion, configurable lens
distortion, resolution and simulation cadence. Exposure/auto white balance, sensor
noise, rolling shutter, motion blur, focus, OV2640 JPEG processing, microphone audio,
and Wi-Fi latency are not emulated. Hardware visual fidelity remains unverified.

Camera checks:

```sh
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m unittest sim.tests.test_camera sim.tests.test_web
node --test web/tests/*.test.mjs
```

## Matplotlib keyboard teleop (adapted from EDMDc/teleop_tank.py)

    W / S  surge fwd/back   A / D  sway left/right   Q / E  heave up/down   F / R  yaw left/right
    + / -  gain 0.1..1.0    Space  panic (zero cmd)  P  screenshot         Esc  quit
    H      PID hold assist on/off (blimp-only addition; same as the big PID / HOLD button)

Hold-to-move; command = ±gain on each axis, mapped through the same global-scaling allocation the
ArduSub allocator uses (authority per axis printed by `--self-test`). The piloted meaning is kept
(A = left, Q = up) although the blimp body frame is y-right / z-down. Heave keys: Q up / E down by
default (chosen 2026-10-04); `--heave-keys eq` gives E up / Q down, which is what
`EDMDc/teleop_tank.py` has. Keyboard focus must be on
the matplotlib window; a 0.75 s key timeout zeroes the command if a key-release is lost.

## Matplotlib GUI layout (`--gui` and teleop)

    +----------------------------+---------------+------------------------------+
    |                            | attitude HUD  | [ PID / HOLD : ON ]  (green) |
    |        3D scene            | heading tape  | [ RESET ]  [ PAUSE ]         |
    |  blimp, rovers, forces     | stick boxes   | BLIMP PHYSICS   sliders      |
    |                            |               | CONTROLLER / TASK sliders    |
    |                            |               | VIEW            slider       |
    +----------------------------+---------------+------------------------------+
    | VEHICLE INFORMATION: altitude + target, motion, forces |                    |
    | teleop commands and keyboard hints                    |                    |
    | legend                                                                    |
    +---------------------------------------------------------------------------+

The PID / HOLD button is the first control on the panel: green = ON, grey = OFF; click it or
press H. RESET (or R in `run_circle.py --gui`) restarts the sim at t = 0 with x = y = 0 and the
gondola touching the ground; held keys and the captured hold reference are cleared. PAUSE (or space) freezes the physics while the
sliders stay live. Vehicle information appears in a wide panel below the scene and HUD, with
large current-altitude digits and separate motion and force groups. Teleop commands, gain,
held keys, and keyboard hints appear below that panel.

Every slider also has an editable number field on its right. Click the field, edit the
number, and press Enter (or click outside) to apply it; the slider and field stay synchronized.
Decimal and scientific notation are accepted within the slider's existing limits. Invalid or
out-of-range entries revert to the current value. While a field is focused, typing does not
trigger flight controls or GUI shortcuts; click outside the field to resume keyboard control.

The **Desired altitude (m)** box beside the current altitude stays synchronized with the
`blimp_height` slider and its number field. Enter a gondola-bottom height from 0 to 2.5 m and press Enter.
In teleop, it updates the hold controller's altitude target while preserving the horizontal
position and yaw target. It does not turn hold on: a value entered with hold OFF is used when
hold is enabled. Explicit altitude targets survive Reset. Manual up/down commands override
the typed target and return to the existing release-point hold behavior; the displayed target
then follows that captured height. A manually captured height outside the input range remains
visible, with the slider handle at the nearest limit.

## Files

    sim/params.py       all parameters; [paper] vs [slider]; SLIDERS list drives the GUI
    sim/blimp.py        6-DoF Fossen-form blimp, 6-thruster BlueROV-style allocation, RK4
    sim/rover.py        exact unicycle + hand-point feedback linearization
    sim/controllers.py  circle tracker (rovers), position PD on the blimp's centre of mass
    sim/sim.py          TeamSim: reset/step/run, 100 Hz physics / 20 Hz control, npz logging
    sim/viewer.py       3D viewer, HUD, force arrows, sliders, headless video
    sim/keyboard.py     keyboard controller (adapted BlueROV key map, authority, global scaling)
    sim/web_runtime.py  independently timed simulation worker, commands and NPZ export
    web_server.py       loopback server, browser state stream, control ownership
    web/               browser controls, Three.js scene and locally bundled vendor assets
    requirements-web.txt  tested Python dependencies for the browser GUI
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
which is not the paper's hover regime.

Raw `eta` remains the CV pose in NED. Displayed altitude is `-eta[2]` minus the current
world-down distance from CV to the lowest gondola box/motor housing surface. At level attitude
that distance is 0.325 m, so the default raw CV pose is `[0, 0, -0.325, 0, 0, 0]` while the
displayed position is `(0, 0, 0)`. `Blimp.pose_at_altitude()` converts a requested clearance
and attitude into a raw pose; explicit `reset(blimp_eta0=...)` poses retain their raw NED meaning.
The shared box and motor dimensions reproduce the rendered geometry; they are not hardware
measurements. `floor_alt = 0` and `ceiling_alt = 4` bound gondola clearance. Contact uses a
vertical position/velocity clamp, not a force, friction, or landing-impact model.

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

## Behavioural checks (`python verify_dynamics.py`)

The numbers below and the saved `results/verify_dynamics.png` describe the original CV-height
version. They are historical results, not rerun measurements for the gondola-bottom reference.
The script now measures gondola clearance; use `--out /tmp/verify_dynamics.png` to run it without
overwriting the saved figure.

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

Both GUIs show **Net lift in equivalent grams**, with a slider and typed input from **−10 to
+10 g**. Positive values mean surplus upward lift; negative values mean downward imbalance
(for example, −5 g represents the extra downward force of a 5 g load). The simulator uses
`net_lift_N = net_lift_g × 9.81 / 1000`, so ±10 g is ±0.0981 N. The vehicle readout includes
any lift drift; the slider sets the baseline, so drift can move the actual value outside ±10 g.
Lift drift itself remains in N/√s; other force controls remain in newtons.

This is useful for testing trim/load imbalance, controller recovery, and thrust saturation.
It does not attach a physical payload or change mass, inertia, centre of mass, or restoring
torque. At the default two vertical thrusters × 0.03 N, maximum level heave authority is
0.06 N, equivalent to about 6.12 g. A ±10 g imbalance therefore exceeds default altitude-hold
authority; start around ±1–5 g for loads within that limit.

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
rover_X_next, rover_ref`, as in the underwater repo. `blimp_lift` is total net lift in **N**, including
drift, even though GUI controls display grams. The `run_circle.py --net-lift` argument also
remains in N. `Eta`/`Eta_next` still contain raw CV/NED
poses. New exports additionally include `blimp_bottom_altitude`, `blimp_bottom_altitude_next`,
the reference strings `eta_reference='CV_NED'` and `altitude_reference='gondola_bottom'`, and
geometry metadata (`gondola_size`, `thruster_length`, `thruster_radius`, `d_VT`). Existing saved
NPZ files predate these additive fields. Seeds via `SimParams.seed`.
