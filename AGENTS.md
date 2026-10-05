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
- Rovers: ideal unicycle motion with instantaneous commanded forward/yaw velocity and hand-point tracking. Velocity-loop lag, slip, and learned context are not implemented.
- Baseline scenario: four rovers by default follow a circle of radius 1.5 m at 0.2 m/s; automatic blimp flight targets its centre at 1 m gondola-bottom clearance. Startup and Reset place the blimp at x = y = 0 with the gondola touching the ground. Rover count is configurable.
- Blimp control: position PID on the centre of mass, yaw PD, and a live PID enable switch. Keyboard control supports optional hold assistance on idle axes.
- Height convention: all altitude readouts and targets use clearance beneath the lowest gondola box/motor housing surface, adjusted for attitude. Raw physics poses and `Eta` logs remain CV/NED coordinates; the level CV is 0.325 m above the lowest surface.
- Net lift: both interfaces expose signed equivalent grams from −10 to +10 g (+ up / − down). This changes trim force only; physical payload mass/inertia are not modeled by that control. Physics and saved net-lift logs use newtons.
- Timing: 100 Hz physics and 20 Hz control by default; per-simulation random seed.
- Interface: localhost browser control panel with a bundled Three.js scene, readable telemetry, numeric fields paired with every slider, an altitude target, and NPZ/PNG export. The Matplotlib viewer remains available for existing GUI and video workflows.
- Logging: NumPy NPZ export for blimp and rover transitions. A saved example is present in `results/circle.npz`; suitability for model training has not been audited in this review.
- Altitude lesson: a separate automatic collector records neutral-trim vertical flights with complete state/action/next-state timing and source provenance. The first checked dataset contains 14,400 transitions in `results/altitude_lesson/2026-10-04_neutral_seed42`; no learned model has been fitted yet.
- Onboard vision: configurable OV2640-style pinhole camera under the gondola, looking down, with live preview and synchronized PNG/state/action recording. Optics and mounting position are estimates until measured. No learned perception or hardware image-fidelity result is implied.

## Key files

| Path | Role |
| --- | --- |
| `sim/params.py` | Physical parameters, controller settings, timing, and GUI slider definitions |
| `sim/blimp.py` | Blimp dynamics, thruster geometry/allocation, and integration |
| `sim/rover.py` | Ideal unicycle rover and hand-point control |
| `sim/controllers.py` | Rover circle tracking and blimp position/yaw control |
| `sim/sim.py` | Team orchestration, stepping, and log export |
| `sim/keyboard.py` | Keyboard mapping and hold assistance |
| `sim/viewer.py` | GUI and video rendering |
| `sim/web_runtime.py` | Single-owner simulation worker, command validation, snapshots, and NPZ export |
| `sim/camera.py` | Camera calibration validation, NED/optical pose, and exact simulation-clock recording |
| `web_server.py` | Loopback HTTP/WebSocket server and browser launch |
| `web/` | HTML controls, telemetry, 3D scene, and offline vendor assets |
| `web/camera-math.js`, `web/sensor-camera.js` | Calibrated projection, inverse lens distortion, and sensor-only rendering |
| `web/camera-panel.js`, `web/zip.js` | Camera settings, live preview, and offline PNG/JSON archive export |
| `config/camera_ov2640.example.json` | Uncalibrated default profile; replace optics and mounting pose with measurements |
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
- Hardware geometry, thrust calibration, rover platform, and motion-capture/communication details remain design questions in the plan. The camera is now specified as OV2640 beneath the gondola, looking down; intrinsics, distortion, mounting pose and actual sensor performance still require measurement.

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

## Simulated OV2640 camera and synchronized recording — 2026-10-05

- User selected the OV2640 variant of Seeed XIAO ESP32-S3 Sense, mounted under the
  gondola looking straight down, with live preview and synchronized recording. Kept
  the existing Python/Three.js simulator and offline runtime; added no dependencies.
- Added full-attitude optical pose, pinhole projection and OpenCV five-coefficient
  Brown–Conrady distortion. Image top is body forward, right is body right. Ground
  texture provides motion cues; sensor images exclude debug overlays but retain
  physical geometry, self-occlusion and shadows.
- Defaults are explicit assumptions: 640 × 480 at 10 simulated fps, 60-degree horizontal
  field of view, centred principal point, zero distortion, lens 2 mm below the gondola
  box (0.312 m body-down from CV, about 13 mm above ground at startup). These are not
  measured lens or throughput specifications. Sensor placement does not add payload
  mass or change the existing ground-contact geometry.
- Import/export full camera profiles or compact OpenCV-style JSON. Intrinsics scale
  with pixel-centre convention and fixed aspect ratio. The User optics badge covers
  only intrinsics/distortion; the default mounting pose remains estimated. Invalid
  backend settings restore accepted form values. Renderer failures clear stale images,
  disable recording and wait for explicit retry or a changed profile.
- Live preview is capped at 640 × 480 and 10 Hz; saved previews identify the actual
  captured generation/control-step/time. Recorded output supports 320 × 240 through
  1600 × 1200 and 5/10/20 simulated fps, independently of browser refresh. No measured
  latency or real-time performance guarantee is made.
- Recording retains the initial frame, scheduled exact control-tick states, the final
  state, and every 20 Hz pre/post transition with actual blimp and clipped rover actions,
  actuator targets and parameter metadata. Stops at 30 simulated seconds or on Stop,
  Reset, disconnect or failure; pauses create no duplicates. A completed recording must
  be explicitly discarded before another starts. It survives Reset but not server exit.
- Export renders each retained pose in an independent scene into PNG files, then builds
  a ZIP with recording.json and manifest.json, timestamps, action-interval indices,
  intrinsics/poses, invalid-pixel fractions and SHA-256 hashes. Progress, cancel, retry
  and a persistent Save ZIP link are provided. Large exports use more memory/time.
- Passed 36 Python camera/web/altitude tests, five dynamics checks, nine keyboard checks,
  five camera-math tests and nine camera-panel tests (64 total); whitespace checks pass.
  GPT-5.6 Sol provided planning and final review; reported frontend issues were repaired.
- Browser GPU preview was visually inspected at ground level and in flight with four
  rovers. Tested invalid focal-length rejection/restoration and compact calibration import
  with nonzero radial/tangential distortion, then restored estimated default optics.
  No browser warnings/errors were observed. A 30-second test generated 301 images;
  independently fetched JSON confirmed 301 unique scheduled states, 600 control intervals,
  and exact image-state/pre-post alignment. The ZIP writer separately passed Python
  archive/CRC inspection. The in-app browser download helper timed out for the actual
  generated blob ZIP, so its OS-level save and complete image archive were not inspected.
- Preview: results/web_gui_ov2640_camera.jpg. The isolated test server uses port 8001;
  the older port-8000 session was not restarted. Existing sessions need a Python server
  restart and page reload to load the camera backend. Existing experiments were preserved.
- Limits: stylized scene; uncalibrated hardware fidelity; no sensor noise, exposure/white
  balance, rolling shutter, motion blur, focus, OV2640 JPEG processing, microphone audio
  or Wi-Fi delay model. Recording is not a full controller/RNG checkpoint for arbitrary
  disturbed-flight replay. No visual world model was trained in this task.

## Controls guide beneath Live World — 2026-10-05

- User deferred online publication and requested control instructions beneath Live World;
  confirmed movement keys, PID hold, release controls and pause. No deployment work was
  performed. The public hosting/session choices remain unanswered.
- Moved the keyboard guide from the sidebar to a dedicated card directly below the 3D
  scene. It stays grouped with the scene when the camera panel stacks on smaller screens.
  Added Teleop/Run/focus instructions and plain-language direction labels: W/S forward/back,
  A/D slide left/right, Q/E up/down, F/R rotate left/right, H PID hold, Space release manual
  input, Esc pause. Clarified heading-relative horizontal movement and continued momentum.
- Only static HTML/CSS and documentation changed; physics, key mappings and input-focus
  protection remain unchanged. GPT-5.6 Sol reviewed the layout plan and final source.
- Visually verified the 1280-pixel browser layout, correct DOM ordering, one guide only,
  no horizontal overflow and no browser warnings/errors. Existing flight remained paused
  and was inspected as a spectator. Narrow-screen wrapping was reviewed in CSS, not tested
  interactively. Whitespace checks passed; no new tests were needed for this static guide.
- Preview: results/web_gui_controls_guide.jpg. Reload the browser to display the guide;
  no Python restart is needed for this change.

## Online deployment cancelled — 2026-10-05

- User initially selected Render Free with independent visitor flights, then explicitly
  cancelled publication and requested reverting to local operation.
- No Render service was created, no public simulator URL was published, and no commit
  or branch was pushed. Repository visibility remained private.
- Removed the online deployment configuration and isolated visitor-session changes,
  preserving the previously authorized local simulator, OV2640 camera/recording and
  controls guide. Restored the original main branch and deleted the unused deployment
  branch. All 36 original Python camera/web/altitude tests and 14 JavaScript tests pass;
  whitespace checks and a search for remaining public-session code are clean.
  GPT-5.6 Sol completed the final rollback review with no actionable findings.
- Render was installed/connected during preparation; the user did not request removal
  of that account connection, so it was left unchanged. No credentials were saved in
  the project. Existing research results and the port-8001 user flight were preserved.
- Stopped the disposable public-mode test server and started the normal loopback-only
  local launcher on http://127.0.0.1:8002. Opened the browser as controller at t=0,
  ground clearance 0 m, paused, with camera and controls guide present and no browser
  warnings or errors. The original local startup commands remain unchanged.

## Online deployment resumed — 2026-10-05

- User reported granting Render access to this GitHub repository only, then explicitly
  requested resuming full online deployment. The named Render connection now successfully
  accesses My Workspace. Repository visibility must remain private.
- Restoring the previously tested public-session layer with GPT-5.6 Sol planning/review.
  The target is one native Python Render Free web service in Ohio, two visitor slots,
  one worker and manual deployments. No database, paid service, or persistent disk is
  needed. The normal localhost launcher and existing research files remain available.
- Confirmed expiry will require an explicit Start new flight action, so idle browser
  pages do not automatically reclaim capacity. Transient connections reuse their current
  flight token. Tokens stay in page memory and never appear in request URLs.
- Deployment configuration is documented in README; direct Render service creation is
  used instead of a Blueprint. The active branch is codex/render-demo. Publication and
  end-to-end verification are in progress; no successful deployment is claimed yet.
- Restored-code checks: 49 backend tests and 25 JavaScript tests passed. A temporary
  loopback browser preview with a shortened 25-second lifetime confirmed startup,
  explicit expiry notice, no automatic reallocation, and Start new flight returning
  to a fresh paused state. The stale expiry notice after restart was repaired.
  The native browser timer wrapper regression is retained. Public host/origin rules
  are tested separately from the loopback-only browser adapter.
