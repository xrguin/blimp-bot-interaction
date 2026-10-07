# Project instructions and progress

## Working instructions

- Ask the user about relevant details before generating code, creating files, or executing commands; prefer multiple-choice questions. Use answers already given in the current task rather than repeating questions.
- Plan first when the exact procedure is uncertain, and state assumptions before the next action.
- Keep this file updated with verified project progress and the limits of each review.
- When analyzing data, use readable plots containing data, legends, and axes, without additional explanatory text inside the plots.
- The user requests GPT-5.6 Sol for planning and final checks when delegating work. They suggested GPT-5.3 Code Spark or GPT-5.6 Lunar for execution/code tasks when available; do not silently substitute unavailable model names.

## Project purpose

This repository provides a Python simulator for one GT-MAB-class blimp and multiple differential-drive ground rovers. The longer-term research goal is a modular world model whose separately identified modules can be composed for different team sizes without retraining. That goal is a proposed research claim, not a demonstrated result in this checkout.

Primary sources: `README.md` describes the implemented simulator; `docs/PLAN.md` and `docs/PROBLEM_FORMULATION.md` describe the broader research direction. Prefer inspected code when these sources disagree.

## Current implementation

- Blimp: 6-DoF rigid-body dynamics, added mass, drag, buoyancy/weight restoring effects, thruster lag, and RK4 integration. The current geometry uses six thrusters in a BlueROV2-style layout, allocating surge, sway, heave, and yaw.
- Rovers: ideal unicycle motion with instantaneous commanded forward/yaw velocity and hand-point tracking by default. An opt-in MuJoCo backend (`SimParams.rover_backend = "mujoco"`, `--rovers-backend mujoco`) replaces them with contact-based TurtleBot3-Burger-like differential drives (wheel PI loops, slip, collisions) and adds rover/blimp cameras; the blimp plant is unchanged in both cases. Learned context is not implemented.
- Baseline scenario: four rovers by default follow a circle of radius 1.5 m at 0.2 m/s; automatic blimp flight targets its centre at 1 m gondola-bottom clearance. Startup and Reset place the blimp at x = y = 0 with the gondola touching the ground. Rover count is configurable.
- Blimp control: position PID on the centre of mass, yaw PD, and a live PID enable switch. Keyboard control supports optional hold assistance on idle axes.
- Height convention: all altitude readouts and targets use clearance beneath the lowest gondola box/motor housing surface, adjusted for attitude. Raw physics poses and `Eta` logs remain CV/NED coordinates; the level CV is 0.325 m above the lowest surface.
- Net lift: both interfaces expose signed equivalent grams from −10 to +10 g (+ up / − down). This changes trim force only; physical payload mass/inertia are not modeled by that control. Physics and saved net-lift logs use newtons.
- Timing: 100 Hz physics and 20 Hz control by default; per-simulation random seed.
- Interface: localhost browser control panel with a bundled Three.js scene, readable telemetry, numeric fields paired with every slider, an altitude target, NPZ/PNG export, and (with the MuJoCo backend and `--cameras`) a live vehicle-camera carousel. `run_mujoco_gui.py` opens MuJoCo's native viewer for the same scenario. The Matplotlib viewer remains available for existing GUI and video workflows.
- Logging: NumPy NPZ export for blimp and rover transitions. A saved example is present in `results/circle.npz`; suitability for model training has not been audited in this review.
- Altitude lesson: a separate automatic collector records neutral-trim vertical flights with complete state/action/next-state timing and source provenance. The first checked dataset contains 14,400 transitions in `results/altitude_lesson/2026-10-04_neutral_seed42`; no learned model has been fitted yet.

## Key files

| Path | Role |
| --- | --- |
| `sim/params.py` | Physical parameters, controller settings, timing, and GUI slider definitions |
| `sim/blimp.py` | Blimp dynamics, thruster geometry/allocation, and integration |
| `sim/rover.py` | Ideal unicycle rover and hand-point control |
| `sim/mujoco_world.py` | Optional MuJoCo world: contact rovers with wheel PI loops, blimp mocap mirror, vehicle cameras, PNG frame recorder |
| `sim/png.py` | Dependency-free PNG encoder and JPEG/PNG frame encoding used by the recorder and web camera feed |
| `run_mujoco_gui.py` | MuJoCo's interactive viewer: latched keyboard teleop, camera overlays and HUD, optional OpenCV camera/WASD window |
| `sim/tests/test_mujoco_world.py` | Nineteen MuJoCo backend, camera, GUI-session, overlay-layout and web-camera checks (skipped without `mujoco`) |
| `requirements-mujoco.txt` | Optional pinned MuJoCo and OpenCV dependencies |
| `INSTALL.md` | From-scratch Ubuntu install and verification guide (tested on Python 3.10 and 3.12) |
| `sim/controllers.py` | Rover circle tracking and blimp position/yaw control |
| `sim/sim.py` | Team orchestration, stepping, and log export |
| `sim/keyboard.py` | Keyboard mapping and hold assistance |
| `sim/viewer.py` | GUI and video rendering |
| `sim/web_runtime.py` | Single-owner simulation worker, command validation, snapshots, and NPZ export |
| `web_server.py` | Loopback HTTP/WebSocket server; per-visitor sessions, public hosts, access token, browser launch |
| `sim/web_sessions.py` | Session manager: cookie identity, concurrent cap, idle reaper, shared mode |
| `DEPLOY.md`, `deploy/` | Website publishing guide (Cloudflare Tunnel + Access), tunnel config example, systemd user units |
| `web/` | HTML controls, telemetry, 3D scene, and offline vendor assets |
| `requirements-web.txt` | Tested browser-server dependencies |
| `Start Web GUI.command` | macOS launcher using the project virtual environment |
| `run_circle.py` | Autonomous circle scenario entry point |
| `teleop_blimp.py` | Keyboard control entry point and headless self-test |
| `sim/tests/test_blimp.py` | Five dynamics/geometry checks |
| `sim/tests/test_altitude.py` | Gondola clearance, ground contact, target conversion, hold, and log checks |
| `verify_dynamics.py` | Eight behavioral checks; use `--out` to preserve its existing result figure |
| `exp_net_lift.py` | Trim/position-controller comparison experiment |
| `collect_altitude_data.py` | Reproducible altitude-lesson collection, episode splits, and source snapshots |
| `check_altitude_data.py` | Independent manifest/timing/physics replay checks and data-only plots |
| `sim/tests/test_altitude_data.py` | Five focused collection, reproducibility, replay, export, and guard checks |
| `results/` | Existing scenario videos, figures, interface screenshots, and one NPZ log |

## Setup and routine commands

The browser GUI uses NumPy, FastAPI, Uvicorn, and WebSockets, pinned in
`requirements-web.txt`. Three.js is bundled under `web/vendor/three/` with its license and
provenance. Install dependencies once; runtime requires no Wi-Fi or internet. The local
`.venv` is ignored by Git. Matplotlib is needed only for the original viewer/plots, and
FFmpeg additionally for MP4 export.

Run from the repository root:

```sh
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements-web.txt
.venv/bin/python web_server.py
# Original viewer and headless checks:
python3 -m pip install numpy matplotlib
python3 -m sim.tests.test_blimp
python3 teleop_blimp.py --self-test
python3 run_circle.py --gui
python3 teleop_blimp.py --rovers circle --hold
```

Install dependencies only when required for an authorized task. Keyboard defaults are W/S surge, A/D sway, Q/E up/down, F/R yaw left/right, and H hold toggle. See `README.md` for the complete controls and output options.

## Remaining research work and documentation gaps

- Learned rover/blimp modules, context inference, camera perception, estimation/data association, MPC, collision avoidance, and modular-versus-monolithic scaling experiments remain planned.
- A circle-tracking demonstration and a checked neutral-trim vertical dataset exist; broader formation-control, changed-load/attitude collection, and team-scaling milestones remain incomplete.
- `docs/PLAN.md` still says "planning only (no code)". Its five-thruster/four-input design and lagged rover dynamics differ from the implemented six-thruster simulator and ideal rovers. `docs/PROBLEM_FORMULATION.md` also retains the planned state/input definitions.
- Translational drag, added mass, yaw parameters, thrust, and actuator lag are marked as unmeasured/tunable in `sim/params.py`; passing simulator checks does not establish hardware fidelity.
- The documents refer to `ICARV_08581376.pdf` as local, but it is absent from this checkout and explicitly ignored by Git. Source-paper parameter verification remains outstanding for this review.
- Hardware geometry, thrust calibration, rover platform, camera orientation, and motion-capture/communication details remain design questions in the plan.

Suggested next work, subject to the user's chosen task: reconcile the design documents with v1; confirm hardware and parameter sources; audit transition timing and log metadata before dataset collection; then implement and evaluate the first learned module.

## Review record — 2026-10-04

- User-selected scope: project overview, setup, and progress. Assumption: inspect the existing project and record findings without changing simulator code or regenerating results.
- Initial Git state: clean `main` tracking `origin/main`; one local commit, `632575c` (2026-10-04). Remote state was not refreshed.
- Local runtime found: Python 3.11.4, NumPy 2.0.0, Matplotlib 3.11.0, and FFmpeg on PATH. These are observed versions, not pinned requirements.
- Executed `python3 -m sim.tests.test_blimp`: all five checks passed.
- Executed `python3 teleop_blimp.py --self-test`: all nine keyboard-direction/swing checks passed.
- Existing GUI screenshot inspected. Live GUI interaction, video regeneration, the full behavioral suite, saved-log analysis, hardware validation, and literature verification were not performed.
- This review created `AGENTS.md`; simulator code, existing design documents, and saved results were left unchanged.

## GUI numeric entry — 2026-10-04

- User selected a numeric field for every slider, with Enter applying values within the existing slider limits.
- Added all 22 fields in the shared `sim/viewer.py` panel, covering both `run_circle.py --gui` and `teleop_blimp.py`.
- Sliders and fields synchronize through the existing parameter callbacks. Decimal and scientific notation work; malformed, non-finite, and out-of-range entries restore the current value. Matplotlib also submits a field when it loses focus.
- While editing, scenario shortcuts and teleop keypress actions are suppressed. Teleop clears held movement keys before stepping; click outside the field to resume keyboard control.
- Updated the README with input behavior. Simulator dynamics and saved results were not changed.
- Verification: headless checks passed for all 22 control pairs, valid values, endpoints, invalid entries, parameter updates, reset behavior, and the non-GUI viewer. Actual Matplotlib mouse/key events verified scientific notation and focus protection in both entry-point flows, including teleop thrust-authority updates and previously held keys with timeout disabled.
- Rendered GUI screenshot visually inspected: numeric fields, labels, sliders, and footer fit the panel. Native interactive-window behavior was not manually exercised.
- Existing five dynamics tests and nine teleop self-tests passed again. GPT-5.6 Sol provided planning and the final source review.

## Vehicle-information panel and altitude target — 2026-10-04

- Interpreted the requested "desired latitude" as desired altitude above ground, as stated to the user. Added a larger grouped vehicle-information panel beneath the scene/HUD: a prominent current-altitude readout, motion/attitude values, and clearly labeled thruster/lift values with units.
- Added a dedicated desired-altitude field beside current altitude, synchronized with the existing `blimp_height` slider and its field. There are now 23 numeric fields total. New typed targets retain the 0.3–2.5 m limits and Enter/click-outside submission behavior.
- Added `KeyboardBlimpController.set_altitude_target()` so teleop input updates the actual hold reference while preserving horizontal/yaw targets. Entering a target does not enable hold; explicit targets persist while hold is off and through Reset.
- Manual heave cancels an explicit altitude request and preserves the existing hold-at-release behavior. The displayed target then follows the captured reference. Captured heights outside the input limits remain visible; the slider handle stays on its track.
- GUI input synchronization handles live captured references without replacing a value while the user is editing. All numeric fields retain keyboard-focus protection. Teleop command/help text uses two readable lines below the information panel.
- Verification passed: altitude-input mouse/key events, three-way field/slider synchronization, bounds and invalid input, direction of vertical control, active/disabled/reset hold behavior, horizontal/yaw preservation, manual override, captured targets outside the input range, all previous 22-slider checks, and the non-GUI/video telemetry path. The controller implementation also passed 16 focused CM/CV hold checks.
- Existing five dynamics tests and nine teleop self-tests passed. Both GUI layouts were rendered for visual inspection and text-bound checks; native interactive-window behavior was not manually exercised.
- GPT-5.6 Sol planned and reviewed the changes. README updated; existing saved experiment results remain unchanged.

## GUI responsiveness review — 2026-10-04

- User reports lag in both keyboard flight and sliders/number fields and asks whether to switch to a localhost browser GUI.
- Confirmed a redraw regression: with the current 23 TextBoxes, dispatching one blank-area mouse press invokes `canvas.draw()` 23 times. This was checked with instrumented headless callbacks; it measures call count, not native GUI latency. Matplotlib TextBox focus-loss handling requests a full draw even for inactive fields.
- Further source findings: the animation callback couples simulation stepping to whole-figure rendering; moving 3D/HUD artists are recreated each frame; every slider callback rebuilds blimp matrices, including task/view settings.
- Recommendation: first remove redundant widget redraws and unnecessary rebuilds; separate simulation/control timing from display updates. A future localhost interface can retain the Python models/controllers/logging and add HTML controls, a WebSocket connection, and a WebGL scene. Migration remains a proposal, not an adopted or implemented change.
- GPT-5.6 Sol independently reviewed the source. No simulator/GUI implementation changed during this advisory review; no wall-clock responsiveness benchmark was performed.

## Local browser GUI — 2026-10-04 (completed)

- User approved the proposed offline localhost design after discussing numeric entry,
  telemetry readability, keyboard/slider responsiveness, and operation without Wi-Fi.
- Assumptions: retain the existing Python models/controllers and NPZ schema; add an
  alternative interface without replacing the Matplotlib/video entry points; bundle all
  browser dependencies locally; bind only to 127.0.0.1.
- Added an independent Python simulation worker and HTTP/WebSocket server. The worker
  preserves fixed 100 Hz physics and 20 Hz control timing; the browser receives latest
  snapshots at 20 Hz and draws on its own animation loop.
- Commands use a queue, finite/range validation, and a parameter whitelist. Only added
  mass/yaw inertia changes rebuild the mass matrix; maximum thrust updates keyboard
  authority. Altitude input routes through the existing hold-target method.
- The first connected tab controls the simulation; subsequent tabs are read-only.
  Stale keys expire after 0.35 s. Pause/reset/disconnection clear held keys; a controlling
  tab's disconnection pauses the simulator. Startup is paused with teleop hold enabled.
- Added pinned Python dependency lists, a project virtual environment, and a macOS launcher.
  Three.js 0.186.1 is bundled from its official npm archive with verified SHA-512 integrity,
  its MIT license, and the local OrbitControls import change recorded in VERSION.txt.
- Browser controls pair all 22 sliders with numeric fields, plus the dedicated desired
  altitude box and manual-gain entry. Decimal/scientific input is validated; invalid entries
  remain visible. Slider sends are limited to 20 Hz with a final update. Focus changes,
  pause/reset/mode changes, and reconnection clear local held keys; reconnection also clears
  pending slider sends.
- The 3D scene uses persistent geometry, bounded rover trails, and the original NED pose
  transformed into browser coordinates. Vehicle information names tuple axes and units;
  thruster force is labeled separately from net lift. PNG capture and NPZ download are available.
- A numerical failure now resets the full session (including RNG, controller state, and
  logs), publishes a finite paused error state, and requires explicit Reset before resuming.
- Verification: all 15 new backend tests, five existing dynamics checks, and nine teleop
  self-tests pass. Injected partial-step and non-finite failures recover consistently.
  JavaScript syntax, Python compilation, and whitespace checks passed. GPT-5.6 Sol provided
  planning and the final source review, including the final manual-gain entry correction.
- Live browser checks verified rendering, all 22 fields, numeric/scientific input and slider
  synchronization, persistent validation errors, typing/shortcut isolation, manual-key response,
  rapid gain changes, typed fractional gain, PID hold toggling, altitude tracking, Run/Pause,
  auto/teleop switching, Reset, and a disabled spectator tab. Reload/server-restart reconnection
  recovered control while paused. The downloaded NPZ contained 1180 finite transitions with
  the expected blimp and four-rover array shapes.
- Browser asset inventory showed only 127.0.0.1 URLs and no downloaded fonts. Desktop and
  narrow-window layouts were visually inspected. Wi-Fi was not physically disabled; offline
  support is verified by bundled assets, loopback requests, and source inspection. No formal
  input-latency/FPS benchmark, long-duration stress test, or cross-browser matrix was run.
  PNG capture was source-reviewed but its download dialog was not manually exercised.
- README now documents installation, launch, offline operation, controls, ownership, and log
  behavior. A new screenshot is saved as `results/web_gui_preview.jpg`. Existing saved
  experiments and the underlying dynamics remain unchanged.

## Gondola-bottom altitude and ground startup — 2026-10-04

- User requested zero height at the lowest gondola surface and startup at (0, 0, 0), then
  confirmed that desired altitude should use the same reference everywhere.
- Shared box/motor dimensions now define the lowest surface for both renderers and the
  orientation-aware clearance calculation. At level attitude the lowest motor housing is
  0.325 m below CV. Startup and Reset use x = y = 0 and zero clearance in both flight modes
  and both interfaces. Position telemetry now shows (x, y, height).
- Readouts, typed targets, captured keyboard hold heights, floor/ceiling limits, and scenario
  altitude plots use gondola clearance. The target input range is now **0–2.5 m**, superseding
  the historical 0.3–2.5 m limit above. Explicitly entered targets still survive Reset;
  manual heave still cancels them. Launching directly in automatic mode retains the default
  1 m flight target while starting on the ground.
- Physics state `eta` and exported `Eta`/`Eta_next` retain their original CV/NED meaning.
  Added bottom-height transition arrays and geometry/reference metadata to NPZ exports.
  Existing saved experiments were preserved; README labels old numeric results as historical.
- Passed 8 new altitude tests, 16 browser-backend tests, 5 existing dynamics checks, 9 updated
  keyboard checks, and all 8 behavioral checks. Behavioral plots were written to
  `/tmp/verify_dynamics-clearance.png` without overwriting existing results. JavaScript syntax
  and whitespace checks passed; GPT-5.6 Sol completed planning and final source review.
- Live browser QA verified zero startup, takeoff to a typed 0.6 m target, landing at a typed
  0 m target, and Reset to paused (0, 0, 0). No browser warnings/errors were reported. The
  new preview is `results/web_gui_ground_start.jpg`. The Matplotlib ground frame was rendered
  and visually inspected; live Matplotlib input was not repeated in this change.
- Scope limit: dimensions match the rendered model and are not new hardware measurements.
  Ground contact remains a vertical position/velocity clamp; there is no impact, friction,
  contact torque, or balloon-envelope collision model.

## Net lift in equivalent grams — 2026-10-04

- User requested a grams control limited to ±10 g for examining loads. Kept the existing
  net-lift sign convention, as stated to the user: positive is surplus upward lift; negative
  is downward load-equivalent force. This is a trim-imbalance control rather than a payload
  attachment model; it does not change mass, inertia, centre of mass, or restoring torque.
- Added the derived `BlimpParams.net_lift_g` property using the existing `G = 9.81`:
  force in N = equivalent grams × 9.81 / 1000. Shared sliders now expose `net_lift_g` over
  −10 to +10. The browser wire parameter key is `blimp.net_lift_g`; the old newton UI key
  is rejected so it cannot silently reinterpret units.
- Browser and Matplotlib telemetry (including video text) show actual net lift in equivalent
  grams, including drift. The control sets baseline lift. Drift input remains N/√s and can
  move actual lift beyond the baseline range. Browser `net_lift` and NPZ `blimp_lift` remain
  N; the additive browser `net_lift_g` field supplies the new readout. CLI/experiment force
  inputs retain their explicitly documented N units.
- The default level heave authority is 2 × 0.03 N = 0.06 N, about 6.12 g equivalent. Thus
  ±10 g permits intentional overload/saturation tests; ±1–5 g stays inside that static limit.
- Passed all 26 altitude/browser-backend tests, including two new end-to-end checks for
  conversion, metadata, range validation, force sign, unchanged mass/inertia, drift,
  reset retention, and N log units. Matplotlib callback checks passed signed/scientific
  input, ±10 endpoints, invalid rejection, and actual-drift telemetry/video units; its
  rendered panel was visually inspected. JavaScript syntax and whitespace checks passed.
- Live browser verified −10 and +10 g readings, rejection of 10.01 g, scientific entry,
  and Reset retention. Restored neutral 0 g and left the simulator paused. No browser
  warnings/errors were reported. Preview: `results/web_gui_net_lift_grams.jpg`.
- GPT-5.6 Sol provided planning and final unit/semantics review. No saved experiments were
  regenerated, and no new payload-dynamics or hardware-fidelity claim was made.

## Altitude lesson: automatic data collection — 2026-10-04

- User accepted the altitude prediction lesson and selected automatic collection now, before
  adding a manual recording button. Scope was collection only: fixed neutral trim, vertical
  excitation, reproducible separate episodes, and quality checks; no fitting or prediction
  performance comparison was performed.
- Created `collect_altitude_data.py` using the existing full `Blimp` plant directly. Each
  episode starts level and airborne with zero velocity and thrust; no PID, rover dynamics,
  lateral commands, draught, or lift drift are active. The GUI's ground-start behavior and
  live session were not changed by collection.
- Recorded 12 × 60 s at 20 Hz control / 100 Hz physics. Symmetric vertical-thruster commands
  use signed four-pulse blocks with magnitudes 0.2/0.4/0.6/0.8/1.0 and dwell times
  0.25/0.5/0.75 s, separated by zero-command intervals. Seeds 42–53 and complete command
  schedules are saved. Each episode covers every amplitude/dwell combination.
- Saved dataset: `results/altitude_lesson/2026-10-04_neutral_seed42/`. Files are `train.npz`
  (8 episodes, 9,600 transitions), `validation.npz` (2, 2,400), and `test.npz` (2, 2,400).
  Splits are assigned by whole flight before collection; history windows must stay within
  episode boundaries. Total: 14,400 complete transitions / 720 s of simulated flight.
- Rows retain raw CV/NED `Eta`, body velocity `X`, six normalized commands `U`, realized
  thrust before/after, commanded thrust, gondola clearance, upward velocity and force,
  net lift in N and equivalent grams, clocks, episode/step/seed IDs, and validity/contact
  flags. Commands are applied after current-state capture and held for five physics steps;
  the terminal transition is included. State/action timing is explicit in `manifest.json`.
- The manifest stores all settings, frame/unit definitions, initial states, pulse schedules,
  split counts, and dataset/source SHA256 hashes. `sources/` preserves the collector and
  plant/parameter source; the independent checker is also copied there after validation.
- Five focused collector tests passed, covering repeatability, final transitions, split
  boundaries, saved-state replay, source/file hashes, overwrite refusal, and guard failure
  without partial data. `check_altitude_data.py` passed manifest/units/shape/finiteness,
  unique IDs/seeds, exact sequence alignment, complete pulse coverage, actuator-lag update,
  and a sequential replay of all 14,400 transitions, inspecting all 72,000 physics substeps.
  Zero contact transitions; all valid. Observed clearance range: 1.154–2.897 m.
- `quality.json` records the verification evidence and limits. `example_flight.png/.pdf`
  and `all_flights.png/.pdf` contain only data, axes, and legends and were visually checked.
  Existing experiment files were preserved. GPT-5.6 Sol provided planning and final review.
- For the first predictor, use [height, upward velocity, realized upward thrust] as state,
  normalized vertical command as action, and the corresponding state `_next` arrays as
  targets. Current realized thrust represents the actuator memory (0.1 s lag); future thrust
  must not leak into input features. This is noise-free simulated vertical data under one
  fixed neutral-trim configuration, not evidence of hardware accuracy, load generalization,
  attitude dynamics, or modular team-scaling performance.

## Yaw key direction swap — 2026-10-04

- User requested F to rotate left and R to rotate right. Updated the shared keyboard
  mapping to F = negative yaw and R = positive yaw, used by both browser and Matplotlib
  control. Command gain and yaw authority are unchanged.
- Updated both interfaces' key guides, startup text, README, and existing self-test
  expectations. The keyboard map is now described as adapted from BlueROV teleop.
- Passed all nine existing teleop direction/swing checks, including F producing negative
  yaw and R producing positive yaw. Whitespace checks passed. No interactive browser
  session was restarted or visually tested; running simulators must restart to load the
  Python mapping, and browser pages should reload to show the updated key guide.

## Modular world model and MZ research review — 2026-10-05

- User requested a folder review and research ideas combining modular world models,
  Mori–Zwanzig (MZ) formalism, and collaborative agents; clarified that project idea and
  problem formulation come first. GPT-5.6 Sol assisted with planning, literature review,
  and the final formulation check.
- Inspected the research documents, plant/controller/orchestration code, altitude collector,
  saved quality record, and existing flight plot. The three dataset NPZ SHA256 hashes still
  match `quality.json`; the full replay and simulator test suites were not rerun.
- Current vehicle plants step independently. Rovers have exact instantaneous unicycle
  motion, and the blimp targets the fixed circle centre. Reusing this rover model at larger
  N is therefore not by itself evidence of learned compositional generalization.
- Proposed research direction, not an adopted implementation: reusable vehicle dynamics
  plus memory at observation/estimation interfaces for active aerial sensing of rover
  formations. Camera delay, dropout, shared sensing uncertainty, and planning need explicit
  models; they are not present in the current simulator.
- A sharper candidate is memory closure for a joint belief compressed to local means and
  covariances, retaining a shared blimp/calibration node. A common camera calibration bias
  would provide a controlled shared latent; exact pose and independent observation noise
  can otherwise leave the rover filters factorized. Fitting a closure after freezing the
  physical modules is staged training, not the documents' stricter block-separable training.
- Distinguish local hidden actuator memory from joint-belief compression and sensing
  memory. Exact global MZ reduction does not guarantee that separately learned local or
  pairwise memory kernels compose unchanged. Controlled predictions must condition on
  actions; a generic recurrent model alone does not establish an MZ-derived closure.
- Existing primary literature already covers MZ graph dynamics (arXiv:2405.09324 and
  arXiv:2606.14918), MZ belief abstraction for robotics (DOI:10.1007/s10514-024-10185-1),
  and compositional dynamical-model learning (PMLR 211, Neary and Topcu, 2023).
  Small-team-to-larger-team transfer of separately trained memory interfaces remains a
  proposed question here, not a verified novelty or performance claim.
- No predictor was fitted, no new experimental figure or dataset was generated, and no
  simulator implementation or existing research formulation was changed in this review.

## Simulation platform review — 2026-10-06

- User asked whether to keep developing this simulator or move to Gazebo, Webots, Isaac Sim,
  or similar for "more serious" robotics simulation. Advisory only: inspected `docs/`,
  `sim/blimp.py`, `sim/rover.py`, and the repository layout; no simulator code changed.
- Measured throughput: one `Blimp.step` (RK4, 100 Hz) runs at about 5,200 steps/s on one
  core here (~52× real time), so a 60 s episode takes ~1.2 s. Single run, no averaging.
- Recommendation given: keep this simulator for the identification/world-model phase. The
  blimp model (added mass, CM-offset pendulum, thruster lag, lift drift, draught) is the part
  general engines do not provide natively and would have to be re-implemented as a plugin or
  per-step force callback. Current gaps (ideal rovers, no camera, no collisions, placeholder
  aero parameters) are modelling omissions already planned in `docs/PLAN.md`, not engine limits.
- Migration triggers named: rendered-image perception, contact tasks (pushing/herding or
  collision response), ROS 2 sim-in-the-loop with lab code, or large-scale parallel RL.
  If one fires, MuJoCo was suggested as the default; Gazebo only for a ROS 2 driver; Isaac for
  GPU RL/photoreal data; Webots not recommended for a buoyant vehicle.
- Hedges suggested: treat `sim/blimp.py` as the reference plant that any port must reproduce
  (`test_blimp.py`, `verify_dynamics.py`, altitude-lesson replay); add an engine-independent
  env API; implement the geometric camera stub and rover lag/gain context in NumPy.
- Limits: platform feature statements come from general knowledge, not from installing or
  testing those engines; test suites were not rerun; no delegation to GPT-5.6 Sol occurred.
- Follow-up (same day): MuJoCo 3.6.0 is installed only in the `dmcontrol` conda env
  (Python 3.12, dm_control 1.0.38, gymnasium 1.2.3, torch 2.10.0+cu126); PyPI latest was
  3.15.0. Offscreen rendering works with `MUJOCO_GL=glfw`; EGL and osmesa failed.
- Probed MuJoCo's built-in ellipsoid fluid model (`fluidshape="ellipsoid"`, `density`,
  `viscosity`) on a sphere in air: quadratic translational drag and angular drag are present;
  **buoyancy is absent** (a neutral-mass sphere falls at 9.81 m/s²) and the
  **acceleration-proportional added mass is absent** (effective mass equalled body mass both
  instantaneously and after 0.1 s of stepping). `gravcomp` on a jointless child body at the
  CV reproduced neutral buoyancy and the CM-offset restoring torque. Probe script kept in
  the session scratchpad only; no project files added.
- Consequence recorded: the fruit-fly use case (dense body, thin wings) does not exercise
  buoyancy or added mass, which are the dominant terms for this blimp (M_A is 20 %/80 % of m
  horizontally/vertically in `sim/params.py`). A MuJoCo blimp would need buoyancy via
  `gravcomp`, thruster lag via filter actuators, and a user-supplied added-mass force, then
  validation against `test_blimp.py`; alternatively keep `sim/blimp.py` as the plant and
  drive a MuJoCo mocap body for rendering/contact (co-simulation). No port was started.

## Rover RGB camera feasibility review — 2026-10-06

- User requested a current-folder assessment and clarified that each rover should supply
  rendered RGB images for vision algorithms. This was a feasibility review, not an
  implementation request; no camera pipeline was added.
- Inspected rover dynamics, team observations/controllers, browser scene, simulation worker,
  WebSocket transport, and the planned camera section. Each rover already has a rendered
  group updated from its x/y/heading, but the scene renders only the external viewer camera.
  Python observations contain ground-truth poses, not images. The planned camera stub is
  for the blimp and is not implemented.
- A rover-mounted camera can reuse the existing Three.js scene. Official Three.js camera
  and WebGLRenderer documentation confirms multiple camera views, render targets, and
  asynchronous pixel readback. Recommendation: retain the Python plant for a first RGB
  prototype; engine migration is not required merely to obtain synthetic images.
- Main work beyond preview views: calibrated mounts/intrinsics, removal of diagnostic
  scene overlays from sensor views, image transport/storage, simulation-time frame IDs,
  and synchronized capture for reproducible datasets or vision-driven control. The current
  browser receives latest snapshots at nominal 20 Hz and renders independently; it does
  not guarantee one frame per simulation step. Existing rover control uses true state.
- Calculated raw RGB payload for four 640x480 cameras at 20 Hz: 73.728 MB/s or 4.424 GB/min,
  excluding protocol overhead and compression. Four 320x240 cameras at 10 Hz: 9.216 MB/s.
  These are arithmetic estimates, not throughput measurements. Scene assets are currently
  simple rover bodies and an arena; realistic vision transfer needs further scene work.
- Limits: source/documentation review only; no live rendering experiment, camera benchmark,
  plots, simulator tests, dependency installation, or agent delegation. Existing simulator
  code/results and earlier uncommitted AGENTS.md entries were preserved.

## Local `mujoco` conda environment — 2026-10-06

- At the user's request the 8.5 GB `dmcontrol` conda env (created 2026-03-16) was deleted
  after saving its package list to `~/dmcontrol-packages-2026-10-06.txt`, and replaced by
  `/home/xzha/miniconda3/envs/mujoco` (7.9 GB). User choices: Python 3.12, OpenCV 5.0,
  RL stack, MJX + JAX, perception extras, and re-adding the Deep-Koopman editable install.
  Activate with `conda activate mujoco`; this is a machine-local env, not a project pin.
- Installed and verified on the RTX 3060 Ti (driver 590.48.01, CUDA 13.1): Python 3.12.15;
  torch 2.14.1+cu130 and torchvision 0.29.1 (GPU matmul OK); mujoco 3.15.0 with offscreen
  rendering under `MUJOCO_GL=glfw`; mujoco-mjx 3.15.0 with jax 0.11.2 on `cuda:0`
  (4096 parallel free-fall bodies matched analytic drop); opencv-python 5.0.0 with
  `cv2.aruco` AprilTag dictionaries; pupil-apriltags 1.0.4.post11; kornia 0.8.3;
  ultralytics 8.4.174; gymnasium 1.4.0; stable-baselines3 2.9.0; dm_control 1.0.48
  (cartpole load/reset OK); numpy 2.5.2, scipy 1.18.1, matplotlib 3.11.2, pandas 3.0.6,
  scikit-learn 1.9.1; imageio/imageio-ffmpeg, mediapy, pytest, ipython; fastapi 0.142.2,
  uvicorn 0.54.0, websockets 16.1.1 (this repo's pins, with numpy left unpinned);
  `deep-koopman-robotics` 0.2.0 editable from `~/Documents/Deep-Koopman-Multiagent`.
  `pip check` reported no broken requirements.
- MJX's optional Warp backend (`warp-lang`) is not installed; MJX prints a non-fatal
  import notice. EGL/osmesa offscreen rendering was not retested in the new env.
- This repository's checks pass under the new env: five dynamics tests (free-swing
  period 1.393 s vs paper 1.390 s), nine teleop self-tests, and `pytest sim/tests`
  (36 passed, 28 subtests). The `requirements-web.txt` numpy==2.0.0 pin remains the
  documented `.venv` recipe; no project files other than this record were changed.

## MuJoCo rover backend and vehicle cameras — 2026-10-06

- User confirmed the co-simulation architecture and chose: MuJoCo rovers plus cameras in one
  step, TurtleBot3-Burger-like geometry, rover-forward and blimp-downward cameras at 640×480,
  frames rendered at 20 Hz inside the control step with simulation-time stamps and frame IDs.
  Assumptions stated beforehand: opt-in backend (default unchanged), NED↔MuJoCo flip
  C = diag(1, −1, −1), project speed limits retained, non-colliding blimp mocap body,
  2 ms MuJoCo substeps inside the 10 ms physics step, cameras for headless scripts first.
- Added `sim/mujoco_world.py` (MJCF builder, `MujocoRover` subclass of `Rover`, `MujocoWorld`,
  camera intrinsics/poses, dependency-free PNG writer, `FrameRecorder`), `MujocoParams` and
  `SimParams.rover_backend`/`cameras`, backend-aware `TeamSim` (world stepping, blimp mirror,
  frames, extra logs, shared `npz_arrays()`), `--rovers-backend` on `run_circle.py`,
  `web_server.py`, `teleop_blimp.py`, plus `--cameras/--frames/--frame-every` on `run_circle.py`.
  `web_runtime` accepts the backend and exports the extra arrays; the browser view is unchanged
  and shows no camera images yet.
- Two defects found and fixed during verification: cylinder wheels touched the floor at two rim
  points and scrubbed when yawing (open-loop yaw rate halved) → ellipsoid collision geoms with a
  visual cylinder; the P-only velocity actuator left a steady-state error under the centripetal
  load (yaw 0.561 vs 0.5 rad/s, matching the analytic estimate) → integral action as a setpoint
  bias at 100 Hz with anti-windup. Chosen gains: kv 0.004, ki 0.004 (0.12 s rise, ≈11 %
  overshoot, <0.5 % steady error). Contact-stiffness/friction-cone settings had no effect.
- Camera pose bug fixed: camera axes transform with C R, not C R C. The first rover camera
  position sat inside the nose marker; moved to the chassis front edge (0.065, 0, 0.13 m).
- Verified in the `mujoco` env: 49 tests pass (13 new: frame conventions, mocap mirror,
  straight/turn kinematics against the ideal unicycle, limits, head-on collision, reset
  determinism, circle tracking <3 cm with NPZ shapes, ideal backend unchanged, camera metadata,
  recorder/PNG round trip, web runtime). The camera tests also pass headless via EGL. Base
  Python without `mujoco` skips the 13 tests; its pre-existing `test_web` import error is a
  missing `fastapi` there, unchanged from the clean tree. Legacy runners still pass.
- Measured: four rovers with five 640×480 cameras run ≈12× real time (0.3 ms per frame);
  the threaded web runtime holds real-time factor 1.00 and resets cleanly. A 40 s circle run
  converged to millimetre hand-point error; rendered frames were visually inspected
  (blimp view shows rovers and the blimp's own shadow; rover view shows floor, wall, sky and
  the rover ahead). Outputs stayed in the session scratchpad; no results files were added.
- Limits: rover dimensions, masses, gains and friction are placeholders, not lab measurements;
  the blimp body does not collide; shadows come from a single top light; no depth/segmentation
  output, no browser camera display, no ROS bridge; GL contexts are per thread (renderer is
  recreated if first used from another thread). README documents the feature; GPT-5.6 Sol was
  not available for planning or review in this session.

## MuJoCo viewer and browser camera carousel — 2026-10-06

- User asked for a MuJoCo GUI "with all the sliding bars" and a web layer for the cameras,
  then clarified: MuJoCo's native viewer panels (not the project's parameter sliders, which
  MuJoCo cannot host) and a browser panel of vehicle camera feeds; mid-task they asked for the
  camera view to slide/switch between cameras rather than a dropdown.
- Added `run_mujoco_gui.py`: `mujoco.viewer.launch_passive` with the circle scenario stepped
  under the viewer lock at real-time pace (`--speed`), camera frustums on, reference markers and
  a status label in `user_scn`, keys Space/Backspace/M/H, `--dump-mjcf`. `MujocoWorld.manual_ctrl`
  lets the viewer's Control sliders own the wheel setpoints. The viewer's own Run/speed controls
  are inactive by design (physics belongs to TeamSim).
- Added the web camera feed: `SimulationRuntime(cameras=True)` (requires the MuJoCo backend;
  rendering is enabled from the worker thread because GL contexts are thread-bound),
  `GET /api/camera/<name>` with JPEG (OpenCV/Pillow) or PNG fallback via new `sim/png.py`,
  ETag/304, 503 before the first frame, 404 for unknown cameras; `web_server.py --cameras`;
  config lists `cameras` and `camera_size`. Browser: carousel with ‹ › arrows, range slider,
  pointer drag, horizontal wheel, live round-robin thumbnails, Live switch, frame id/time caption.
- Verified: 54 tests pass in the `mujoco` env (five new: manual wheels, GUI session with a fake
  viewer, image encoding, camera endpoint/ETag/304/404 through the FastAPI test client, no
  cameras → 404). The MuJoCo window was opened for 12 s on display :1 and screenshotted
  (`results/mujoco_gui.jpg`). Headless Chrome driven over the DevTools protocol loaded the page,
  pressed Run, switched cameras with the Next arrow and the slider, and reported frames
  advancing with simulation time; console showed only headless WebGL fallback warnings and the
  pre-existing favicon 404 (`results/web_gui_mujoco_cameras.jpg`). The Claude-in-Chrome
  extension was not connected, so no interactive mouse/touch test of drag or wheel switching
  was performed; those paths are source-reviewed only.
- Limits: thumbnails fetch full-size frames (fine on loopback); no depth/segmentation feeds;
  camera images are not saved from the browser; the native viewer cannot edit blimp parameters.
  GPT-5.6 Sol was not available for planning or review in this session.

## MuJoCo viewer: keyboard teleop and camera streams — 2026-10-06

- User noted the MuJoCo GUI had no WASD blimp control and showed no camera streams. Both
  were true: the first version flew the autopilot only and relied on the viewer's camera menu.
- Measured on the real window with synthetic X key events: `launch_passive` forwards only the
  initial key press (one callback for a 1.5 s hold, no release), and it forwards Space, Enter,
  Tab, Backspace, digits and +/−. `mujoco.mjVISSTRING`/`mjRNDSTRING` show every letter bound to
  a viewer toggle, and pressing W/Q in the viewer did flip wireframe/camera frustums while also
  reaching the callback; the render flags are not reachable from the passive handle.
- Design chosen from those facts: flight keys latch (tap on, tap again or partner key off, Space
  releases) with hold assist via `KeyboardBlimpController(key_timeout=0)`; in the viewer the
  collision-free layout is arrows/PgUp/PgDn/Home/End plus Tab (hold) and Del (manual wheels);
  WASD/QE/FR, H, M, P are read only from the OpenCV camera window, which has no reserved keys.
  `--mode auto` keeps the autopilot. Default mode is teleop.
- Camera streams inside the viewer via `viewer.set_images`: a bottom thumbnail strip whose
  subsampling (160/128/80 px) adapts to `viewer.viewport`, the selected camera enlarged at the
  bottom right (320 or 160 px wide) with a cyan outline on its thumbnail, and a compact HUD via
  `viewer.set_texts` (the overlay font lacks "·", so separators use "|"). The optional OpenCV
  window shows the same frames as a grid. The offscreen renderer uses EGL in this script so the
  viewer thread's GLFW state is never shared.
- Verified on display :1 with injected keys: PgUp/Up in the viewer flew the blimp to
  (+3.67, 0) m at 2.31 m clearance; typing d/0 in the camera window latched sway without
  flipping any viewer flag; screenshots show five thumbnails, the enlarged camera and the HUD
  at the default 1280×720 window (`results/mujoco_gui.jpg`, `results/mujoco_camera_window.jpg`).
  55 tests pass in the `mujoco` env (GUI key sources/latching/gain/pause/camera selection with a
  fake viewer, adaptive overlay layout, OpenCV grid composition). Base Python skips the 18
  MuJoCo tests; its `test_web` import error (missing fastapi) is pre-existing.
- Limits: HUD/overlay placement is checked at one window size; key presses were synthetic
  (xdotool), not a human at the keyboard; the OpenCV window must have focus for letter keys;
  no camera recording from this GUI; GPT-5.6 Sol was not available for planning or review.

## MuJoCo GUI input model: pulses and thumbsticks — 2026-10-06

- User reported the blimp dynamics in the MuJoCo GUI as worse than the browser's. Checked
  numerically: the same key timeline through `SimulationRuntime` and `GuiSession` gives
  identical `eta` (max difference 0.0 over 8 s); the plant and `KeyboardBlimpController` are
  shared. The difference was the latched tap model (one tap = full command until the next tap:
  4.2 m and 0.51 m/s after 10 s), which the user experienced as bad dynamics.
- User chose pulse taps and a mouse joystick. Implemented: each flight-key tap applies the
  command for `--pulse` simulated seconds (default 0.5 s; extends on repeat, partner key
  replaces, Space clears, paused time does not drain); `StickKeyboard` adds an analog vector to
  the keys; the camera window hosts two left-button thumbstick pads (left half surge/sway,
  right half up-down/yaw, `--stick-radius` px = full command, release zeroes) with an overlay.
- Found while testing: the right mouse button in OpenCV's Qt window blocks the event loop
  (a minimal script stayed frozen >10 s after the button was released); the left and middle
  buttons deliver events normally with ≤18 ms frame times. The thumbsticks therefore use the
  left button only, and the README warns against right-clicking there.
- Verified: 56 tests pass, including `test_pulse_tap_matches_browser_key_hold` (a 0.5 s tap in
  the GUI equals holding W for 0.5 s in the browser runtime bit-for-bit). Live on display :1:
  an ↑ tap plus a right-pad drag lifted the blimp to 0.62 m clearance, which the hold assist
  then kept, with simulation time matching wall time (15.0 s); the stick overlay is in
  `results/mujoco_camera_window.jpg`. Mouse and key events were synthetic (xdotool).
- Interface inventory for clarity: one Three.js page (`web_server.py` + `web/`, with the
  optional camera carousel fed by MuJoCo-rendered JPEGs over HTTP), one MuJoCo-native viewer
  (`run_mujoco_gui.py`, overlays via `set_images`, optional OpenCV camera/thumbstick window),
  and the original Matplotlib viewer. All run the same `TeamSim` with either rover backend.

## Camera streams on by default with the MuJoCo backend — 2026-10-06

- User ran `web_server.py` and saw no video streams: the panel was hidden because cameras were
  opt-in (`--cameras`) and need the MuJoCo backend. Changes: `web_server.py` now enables the
  streams whenever `--rovers-backend mujoco` is given (`--no-cameras` disables); the browser
  shows an explanatory note in the camera panel when a session has no streams; `TeamSim` raises
  a clear ImportError (with the install hint) instead of a traceback when the `mujoco` package
  is missing, and `web_server.py` reports it as an argument error.
- Verified: default-on streams and the JPEG endpoint on a MuJoCo-backend server, an empty
  camera list on the default server, the note rendered in headless Chrome, the ImportError text
  under the base Python, and the test suite (57 tests in the `mujoco` env).

## Held keys in the MuJoCo GUI: auto-repeat accumulation fixed — 2026-10-06

- User felt WASD made the blimp "drift far more crazy" than the browser and suspected the
  dynamics. Re-confirmed that the blimp plant/controller are shared and bit-identical; the
  cause was input handling: holding a key in the OpenCV camera window delivers X auto-repeat
  events (measured: 35 events for a 1.5 s hold at 33 Hz), and each event added a full 0.5 s
  pulse, queueing many seconds of thrust after release.
- Fix in `run_mujoco_gui.py`: a tap now renews (never accumulates) its pulse; events closer
  than 0.08 s to the previous one are treated as auto-repeat and renew only a 0.1 s tail, so a
  held key follows the physical hold and stops ~0.1 s after release; the camera window drains
  all queued key events each loop iteration. `GuiSession.clock` is injectable for tests.
- Verified: 57 tests pass (repeat-burst test included). Live probe with a real 1.5 s hold in
  the focused camera window: 22 events spanning exactly the hold, command ending just after
  release, blimp settling at +0.43 m. Offline, identical hold timelines through the browser
  runtime and the GUI session agree up to the release tail (e.g. 0.354 vs 0.410 m with the
  former 0.15 s tail). One earlier live run showed no motion because the synthetic key went to
  the MuJoCo viewer (focus race), which ignores letters by design.
- Note for future work: a 1.5 s W hold from the ground moves the blimp 0.18 m when started at
  t = 0.3 s but 0.35 m when started at t = 4 s in both interfaces; this start-time dependence
  lives in the shared plant/hold controller and was not investigated here.

## Browser: world view beside the camera view — 2026-10-07

- User asked to put the world view alongside the camera view. Browser only: the MuJoCo viewer
  already overlays its cameras inside the world view. Added a `view-row` wrapper around the
  Three.js scene panel and the camera panel with a `side-by-side` grid (equal-height halves,
  scene canvas refits through its existing ResizeObserver), a **Stack below / Side by side**
  button on the camera panel whose choice is kept in `localStorage` (wrapped in try/catch), and
  a media query that forces stacking below 1100 px. Default is side by side when cameras exist;
  sessions without cameras keep the single-column note.
- Verified in headless Chrome over the DevTools protocol with the simulation running: at
  1600 px the scene and camera panels are 574 px wide each and 567 px tall side by side; the
  toggle restores the stacked layout (scene 1159 px wide) and back; at 1000 px the layout
  stacks automatically; the preference persisted; no page exceptions. Preview replaced at
  `results/web_gui_mujoco_cameras.jpg`. JavaScript parsed; no Python changed, tests not rerun.

## Live camera tilt controls — 2026-10-07

- User is exploring camera angle allocations (90° = straight down, 0° = straight ahead) and
  asked for a slider + number box and whether MuJoCo can adjust it in real time. It can:
  `MujocoWorld.set_camera_angles()` rewrites `model.cam_quat` for the blimp and rover cameras
  and MuJoCo recomputes the world camera poses at the next forward pass, so the offscreen
  renderer and the viewer window follow immediately without a model rebuild.
- Added `MujocoParams.blimp_cam_tilt_deg` (default 90) alongside the existing
  `rover_cam_pitch_deg`, a shared `camera_axes()` so MJCF and live updates use one convention
  (image x = vehicle's right; 0° ahead, 90° down), and a browser-only `MUJOCO_SLIDERS` group
  ("Cameras (MuJoCo)": tilt 0–90°, rover pitch −30…60°) that appears only with the MuJoCo
  backend; `SLIDERS` and the Matplotlib panel are unchanged (still 22 controls). The web
  runtime applies the group live and rejects it without the MuJoCo backend. The MuJoCo GUI's
  camera window gained two OpenCV trackbars (applied under the viewer lock), `--blimp-cam-tilt`
  / `--rover-cam-pitch` flags, and the angles in the HUD. The world now applies exact camera
  quaternions at construction (the MJCF text is rounded to 6 decimals).
- Verified: 60 tests pass (tilt geometry at 0/20/45/90°, rover pitch −20/0/30°, rendered
  frames differ between 90° and 0°, browser parameter listing/bounds/live effect, trackbar
  queueing, CLI/HUD). Headless Chrome: the group renders, the number box set 20° and the live
  blimp view changed from floor-down to looking across the arena. The MuJoCo GUI with
  `--blimp-cam-tilt 30` showed the matching view and both trackbars (dragging the trackbar
  itself was not exercised synthetically; its callback path is unit-tested).

## Centralized coordination and scaling research review — 2026-10-07

- User selected coordination and scale (task assignment, congestion, and planning as the
  team grows), allowed substantial extensions, and requested both a broad challenge map
  and ranked research questions with concrete experiments. This was a research-design
  review, not authorization to implement a selected benchmark.
- Rechecked the current controller/orchestration, rover, blimp, parameter, and MuJoCo source.
  Circle references remain independent; `observe()` exposes true rover poses. Cameras and
  contact rovers now exist, superseding the camera-absence statement in the 2026-10-05
  review. Camera frames do not feed a perception/planning pipeline. The MuJoCo layout has
  perimeter walls, and the mirrored blimp has collision disabled. No allocator or
  collision-aware team route planner was found in the inspected implementation.
- Recommended candidate, not an adopted formulation: online ground service tasks across
  bottleneck-connected zones, with one mobile blimp providing an explicit inspection
  prerequisite. Study joint aerial-support scheduling, rover assignment, and routing;
  use ground-only congestion-aware assignment/routing as the initial controlled layer.
  Other shortlisted dimensions are execution-delay propagation and bounded planning time.
- Reviewed primary literature including Jiang et al. (arXiv:2404.16162), traffic-flow
  guidance (2308.11234), combined online assignment/routing (2502.07332), GRAND
  (2512.03194v3), REMAP (2511.21886), uncertain support allocation (2509.22469), and
  aerial-perception-based risk-aware assignment (2003.11675). These establish substantial
  prior work; no novelty or performance claim for this project was established.
- Experimental controls should separate robot count, physical density, task demand,
  aerial-support capacity, and compute budget. Compare capable classical planners and
  fixed-camera/patrol/task-aware support policies; an impossible no-blimp condition is
  not a valid baseline for tasks defined to require aerial inspection. Saturation of a
  fixed physical resource alone does not demonstrate an algorithmic deficiency.
- GPT-5.6 Sol independently critiqued the candidates and evaluation design. Replotted
  published GRAND Table II aggregates (200 agents, reassignment enabled) into a standalone
  figure in the chat's visualization directory, with a source-data JSON and plotting script;
  inspected the PNG. This is published benchmark evidence, not a local simulator result.
  No new simulator experiment, dataset, predictor, or controller was produced; no simulator
  tests or hardware validation were run. Existing working-tree changes were preserved.

## Install guide and `mujoco` branch — 2026-10-07

- User asked to push the work to a separate branch named `mujoco` and for install
  instructions that let an agent on a fresh Ubuntu machine without MuJoCo set up and run the
  code and web UI. Camera frames in `out/` are not to be pushed; `out/` is now git-ignored.
- `requirements-mujoco.txt` now pins `mujoco==3.15.0` and `opencv-python==5.0.0.93`. The
  recipe in `INSTALL.md` (apt packages, `python3 -m venv .venv`, the three requirement files)
  was executed here in a clean Python 3.10.12 venv: 55 unittest checks OK, legacy checks pass,
  the headless MuJoCo scenario rendered frames through EGL without a display, and the web
  server streamed JPEG with the MuJoCo backend. `TeamSim.close()` now releases the renderer at
  the end of `run_circle.py` and of the web worker so processes exit without GL teardown noise.
- Limits: the apt package list was assembled from the wheels' known runtime libraries, not from
  a bare container; Mesa-only EGL and the osmesa fallback were not exercised on this NVIDIA
  machine; `run_mujoco_gui.py` needs a display and was not re-tested from the venv.

## Website access with per-visitor sessions — 2026-10-07

- User asked how others could use the simulation through a website without installing MuJoCo
  while the code keeps running here. Chosen: Cloudflare Tunnel with a Cloudflare Access login,
  a private simulation per visitor, hosted on this desktop. Visitors' browsers already needed
  nothing installed; the work was network exposure and multi-user structure.
- Added `sim/web_sessions.py` (`SessionManager`: HttpOnly cookie identity, `max_sessions` cap
  with 409 + Retry-After, idle reaper stopping runtimes with no open WebSocket for
  `idle_timeout` s, shared mode for the original single world) and rewrote `web_server.py`
  around it: `/api/config` creates or resumes the visitor's session (options `?n=&seed=&mode=&rovers=`
  validated against `--max-rovers`), camera/log/WebSocket routes resolve the session from the
  cookie (404 / close code 4404 without one), `--public-host` adds tunnel hostnames to the host
  middleware and https WebSocket origins and marks cookies Secure, `--access-token` is a
  second lock stored via `/?token=` (401 / close code 4401 otherwise), `--shared` keeps one
  world. Control ownership moved from the app to each runtime. The page shows a session badge,
  passes its query string on first load, retries when the server is full and starts a fresh
  session when one expires. `DEPLOY.md` and `deploy/` document the tunnel, Access policy,
  quick-tunnel fallback with a token, Tailscale alternative, systemd user units and limits.
- Verified: 65 tests pass (session manager with a fake clock, origin rules, two visitors with
  independent simulations and controllers through the FastAPI test client, cap 409, token 401
  and redirect-cookie flow, public host accepted / unknown host 400). Live with two headless
  Chrome profiles against a public-mode server: separate sessions (2 rovers seed 11 vs 4 rovers
  seed 0), both controllers, one running while the other stayed at t = 0, `/health` counting 2,
  foreign Host rejected, clean shutdown. The Cloudflare side (tunnel creation, Access policy)
  needs the user's account and domain and was documented, not executed.
- Limits: cost figures per session are estimates from this machine; no per-visitor rate limiting
  beyond the session cap; the Tailscale route requires a reverse proxy because the server stays
  on loopback; GPT-5.6 Sol was not available for planning or review.

## Concurrent-session capacity and the GLFW thread-safety fix — 2026-10-07

- User asked how many people can use the website at once (expecting ≤3). Measured with a load
  script in the `mujoco` env: N private sessions (4 MuJoCo rovers, 5 cameras at 640×480) with
  a simulated viewer each (state 20 Hz, main camera 10 Hz, thumbnails 10 Hz). With EGL, N = 1–6
  all held real-time factor 1.00 at ~0.4–1.2 CPU cores total, ~20 MB RSS per session (plus a
  one-off ~150 MB for GL libraries) and 30–48 % GPU; N = 8 collapsed to 0.38× real time. The
  limit is the single Python process (GIL), not the 24 threads, 62 GB RAM or the GPU.
  Conservative recommendation: `--max-sessions 4` (unit file updated); upload bandwidth of
  ~0.5 MB/s per viewer is the other practical constraint.
- The first run aborted at N = 3 with `_glfwGrabErrorHandlerX11: Assertion ... failed` when
  worker threads created GLFW contexts concurrently. Fixes: `web_server.py` sets
  `MUJOCO_GL=egl` by default (overridable), and `MujocoWorld.renderer()` serialises renderer
  creation with a module lock. The load test then ran to N = 8 without errors.
- Limits: viewers were simulated in-process (no real HTTP/WebSocket encoding cost, which adds
  some asyncio work per viewer); N = 7 was not measured; results are for this machine only.
