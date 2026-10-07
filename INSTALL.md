# Installation and first run (Ubuntu, from scratch)

This guide is for setting up the blimp + rover simulator, its browser GUI and the optional
MuJoCo backend on a machine that has **nothing installed yet** (no MuJoCo, no project virtual
environment). Every command was run as written on Ubuntu 22.04 with Python 3.10.12; the same
package set is also verified on Python 3.12. Follow the steps in order; each step ends with a
check and the output you should see.

Conventions: `$` lines are shell commands run from the repository root. `.venv/bin/python` is
always the project interpreter — do not use a system `python3` or a conda environment unless
you install the same requirement files into it.

## 1. System packages

```sh
$ sudo apt-get update
$ sudo apt-get install -y git python3 python3-venv python3-pip \
      libgl1 libegl1 libglib2.0-0 libxkbcommon0 libxrender1 libxext6 libsm6 \
      libfontconfig1 libdbus-1-3 libxcb-xinerama0 libxcb-cursor0
```

What they are for: `python3-venv` creates the virtual environment; `libgl1`/`libegl1` are the
OpenGL/EGL loaders MuJoCo renders through; the rest are runtime libraries of the OpenCV wheel
(its Qt window). On a **headless server** (no monitor, no X display) also install the Mesa EGL
driver so offscreen rendering works without a GPU driver:

```sh
$ sudo apt-get install -y libegl-mesa0 libgl1-mesa-dri
```

Check: `python3 --version` prints 3.10 or newer (3.10–3.13 are known to work).

## 2. Get the code

```sh
$ git clone git@github.com:xrguin/blimp-bot-interaction.git   # or the https URL
$ cd blimp-bot-interaction
$ git checkout mujoco                  # the branch with the MuJoCo backend, cameras and GUIs
```

If you received the folder another way, just `cd` into it. Check: `ls sim/mujoco_world.py
run_mujoco_gui.py web_server.py` lists all three files.

## 3. Python environment

```sh
$ python3 -m venv .venv
$ .venv/bin/python -m pip install --upgrade pip
$ .venv/bin/python -m pip install -r requirements-web.txt -r requirements-mujoco.txt -r requirements-web-dev.txt
```

This installs the pinned, tested set: NumPy 2.0.0, FastAPI, Uvicorn, websockets (browser GUI),
MuJoCo 3.15.0 and OpenCV 5.0 (MuJoCo backend, camera streams, MuJoCo GUI) and httpx (tests).
Nothing is compiled; all are binary wheels (about 150 MB). Matplotlib and FFmpeg are only
needed for the legacy Matplotlib viewer and MP4 export (`pip install matplotlib`, `apt install ffmpeg`).

Check:

```sh
$ .venv/bin/python -c "import numpy, mujoco, cv2, fastapi; print(numpy.__version__, mujoco.__version__, cv2.__version__, fastapi.__version__)"
2.0.0 3.15.0 5.0.0 0.142.2
```

## 4. Verify the installation

Run the project's own checks (no display needed):

```sh
$ .venv/bin/python -m unittest discover -s sim/tests -t .
```

Expected: `Ran 55 tests ... OK` in under a minute. If it says `skipped=…` for the MuJoCo tests,
the `mujoco` package is not in this interpreter — repeat step 3 with `.venv/bin/python`.

```sh
$ .venv/bin/python -m sim.tests.test_blimp        # ends with "all tests passed"
$ .venv/bin/python teleop_blimp.py --self-test     # ends with "self-test PASSED"
$ .venv/bin/python run_circle.py --rovers-backend mujoco --T 3 --frames out/frames --frame-every 20
saved 20 camera frames to out/frames (index.csv, cameras.json)
final blimp pos [-0. -0.] gondola clearance 0.387 m, max |pitch| 0.0 deg
```

The last command exercises MuJoCo physics and offscreen camera rendering. It works with or
without a display: with `DISPLAY` set it renders through GLFW, otherwise through EGL (see §7).
`out/` is ignored by git. If this step prints an `EGLError` or `GLFW` error, go to §7.

## 5. Run the browser GUI with camera streams

```sh
$ .venv/bin/python web_server.py --rovers-backend mujoco --no-browser
Blimp control panel: http://127.0.0.1:8000
Starts paused. Press Run in the browser. Ctrl+C stops the server.
```

Open <http://127.0.0.1:8000> in a browser **on the same machine** (`--no-browser` only stops
the automatic launch; drop it to open the default browser). The server binds to the loopback
interface only. To use it from another computer, forward the port over SSH instead of exposing
it: `ssh -L 8000:127.0.0.1:8000 user@that-machine`, then open <http://127.0.0.1:8000> locally.

What you should see: the 3D world view with the camera carousel beside it (five live cameras:
four rover forward cameras and the blimp's downward camera), a "Cameras (MuJoCo)" group of
sliders in the tuning panel, and "24 controls". Press **Run**: the rovers drive the circle and,
in Auto mode, the blimp climbs to 1 m. Checks from a shell while it runs:

```sh
$ curl -s http://127.0.0.1:8000/api/config | .venv/bin/python -c "import sys,json; c=json.load(sys.stdin); print(c['rover_backend'], c['cameras'])"
mujoco ['rover0', 'rover1', 'rover2', 'rover3', 'blimp']
$ curl -s -o /dev/null -w "%{http_code} %{content_type}\n" http://127.0.0.1:8000/api/camera/blimp
200 image/jpeg
```

Options: `--n 6` (rovers), `--mode auto`, `--port 8010`, `--no-cameras`, `--rovers-backend
ideal` (original exact-unicycle rovers, no cameras: the camera panel then shows a note saying
so). Without `--rovers-backend mujoco` there are **no** camera streams by design.

## 6. Run the MuJoCo viewer (needs a display)

```sh
$ .venv/bin/python run_mujoco_gui.py
```

Opens MuJoCo's own window (physics flags, joint/actuator sliders, camera menu) with the camera
streams overlaid, plus an OpenCV "Vehicle cameras" window with two trackbars (camera tilt) and
mouse thumbsticks. Keys are printed at start and documented in `README.md` (the viewer only
reads arrow keys etc.; WASD is typed in the camera window). `--T 10` quits after 10 s, handy for
a smoke test over a remote desktop. This script cannot run on a headless machine; everything in
§4–§5 can.

## 7. Rendering backends and troubleshooting

* **Which GL backend is used.** `sim/mujoco_world.py` sets `MUJOCO_GL=glfw` when `DISPLAY` is
  set and `MUJOCO_GL=egl` otherwise, unless `MUJOCO_GL` is already set. `run_mujoco_gui.py`
  uses EGL for its camera renderer regardless. Override explicitly if needed, e.g.
  `MUJOCO_GL=egl .venv/bin/python run_circle.py ...`.
* **NVIDIA machines and EGL.** With an NVIDIA driver, EGL may fail with `EGLError` because
  glvnd tries the Mesa vendor first. The code selects
  `/usr/share/glvnd/egl_vendor.d/10_nvidia.json` automatically when that file exists and
  `__EGL_VENDOR_LIBRARY_FILENAMES` is unset; set the variable yourself if your file has another
  name (`ls /usr/share/glvnd/egl_vendor.d/`).
* **No GPU, headless.** Install `libegl-mesa0 libgl1-mesa-dri` (step 1) and use `MUJOCO_GL=egl`.
  Last resort: `sudo apt-get install -y libosmesa6` and `MUJOCO_GL=osmesa` (slow software
  rendering).
* **`ImportError: The 'mujoco' rover backend needs the mujoco package`.** You ran a different
  interpreter. Use `.venv/bin/python` or repeat step 3.
* **OpenCV window errors (`could not load the Qt platform plugin "xcb"`).** Only affects
  `run_mujoco_gui.py`; install the `libxcb-*` packages from step 1 or run it with
  `--no-camera-window`. The browser GUI never opens an OpenCV window.
* **Port already in use.** `--port 8010` (any free port 1–65535).
* **`Exception ignored in Renderer.__del__ ... EGL_NOT_INITIALIZED` at exit.** Harmless GL
  teardown noise from an interrupted process; the scripts close their renderer on normal exit.
* **Browser shows "No camera streams in this session".** The server was started without
  `--rovers-backend mujoco`, or with `--no-cameras`. Restart it and reload the page.

## 8. Daily commands

```sh
.venv/bin/python web_server.py --rovers-backend mujoco                 # browser GUI + cameras
.venv/bin/python run_mujoco_gui.py                                      # MuJoCo viewer (display needed)
.venv/bin/python run_circle.py --rovers-backend mujoco --T 40 --plots out/circle.png --npz out/circle.npz
.venv/bin/python run_circle.py --rovers-backend mujoco --T 20 --frames out/frames --frame-every 4
.venv/bin/python -m unittest discover -s sim/tests -t .                 # before and after changes
```

To publish the running server as a website for people who install nothing (Cloudflare Tunnel
with a login, private simulation per visitor), continue with [DEPLOY.md](DEPLOY.md).

`README.md` documents the simulator, controls and file layout; `AGENTS.md` records project
decisions, verified results and their limits — read it before changing the simulator, and
append to it after verified work.
