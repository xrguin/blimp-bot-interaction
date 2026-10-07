"""MuJoCo-backed rover team with vehicle cameras; the blimp dynamics stay in sim/blimp.py.

Opt-in through SimParams.rover_backend = "mujoco". Rovers become contact-based differential
drives (TurtleBot3 Burger-like: two driven wheels, rear caster, wheel velocity loops) stepped by
MuJoCo, so slip, actuator lag, wheel/floor contact and rover-rover collisions come from the
engine. The blimp's NumPy pose is mirrored into a MuJoCo mocap body each physics step so that
its downward camera and the rover forward cameras can be rendered. MuJoCo has no buoyancy or
added-mass model, which is why the blimp is not simulated here.

Frames: the project is NED (x north, y east, z down); MuJoCo is z-up. The map is the fixed
flip C = diag(1, -1, -1):  p_mj = C p_ned,  R_mj = C R_ned C,  yaw_mj = -yaw_ned.
Rover body frame in MuJoCo: x forward, y left, z up (NED body: x forward, y right, z down).
"""
from __future__ import annotations

import os
import threading

import numpy as np

from .blimp import Blimp, R_zyx
from .params import SimParams
from .png import write_png  # noqa: F401  (re-exported for callers of the recorder)
from .rover import Rover


def _configure_gl():
    """Pick an offscreen GL backend before mujoco is imported (it reads MUJOCO_GL at import)."""
    if "MUJOCO_GL" not in os.environ:
        os.environ["MUJOCO_GL"] = "glfw" if os.environ.get("DISPLAY") else "egl"
    if os.environ["MUJOCO_GL"] == "egl" and "__EGL_VENDOR_LIBRARY_FILENAMES" not in os.environ:
        nvidia = "/usr/share/glvnd/egl_vendor.d/10_nvidia.json"
        if os.path.exists(nvidia):           # glvnd otherwise tries the Mesa vendor first and fails
            os.environ["__EGL_VENDOR_LIBRARY_FILENAMES"] = nvidia


_configure_gl()
import mujoco  # noqa: E402  (after the GL configuration on purpose)


C_FLIP = np.diag([1.0, -1.0, -1.0])
_RENDERER_LOCK = threading.Lock()      # GL context creation is not thread-safe (GLFW/X11 aborts on concurrent init)
ROVER_COLORS = ["0.20 0.45 0.85 1", "0.95 0.55 0.15 1", "0.25 0.65 0.30 1", "0.85 0.25 0.25 1",
                "0.55 0.35 0.75 1", "0.55 0.40 0.25 1", "0.90 0.50 0.70 1", "0.50 0.50 0.50 1"]


# ---------------------------------------------------------------- frames
def ned_to_mj_pos(p):
    return C_FLIP @ np.asarray(p, float)


def mj_to_ned_pos(p):
    return C_FLIP @ np.asarray(p, float)


def ned_to_mj_rot(R):
    return C_FLIP @ np.asarray(R, float) @ C_FLIP


def mj_to_ned_rot(R):
    return C_FLIP @ np.asarray(R, float) @ C_FLIP


def mj_quat_from_rot(R):
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.ascontiguousarray(R, dtype=float).reshape(9))
    return q


# ---------------------------------------------------------------- cameras
def camera_axes(tilt_deg: float) -> np.ndarray:
    """Camera rotation (columns = camera x, y, z axes in a z-up body frame) for a camera looking
    along body +x tilted `tilt_deg` below the horizon: 0 = straight ahead, 90 = straight down.
    MuJoCo cameras look along their -z axis with +y up in the image, so x points to the body's right."""
    t = np.radians(tilt_deg)
    look = np.array([np.cos(t), 0.0, -np.sin(t)])
    up = np.array([np.sin(t), 0.0, np.cos(t)])
    z = -look
    x = np.cross(up, z)
    return np.column_stack([x, up, z])


def camera_xyaxes(tilt_deg: float) -> str:
    R = camera_axes(tilt_deg)
    return " ".join(f"{v:.6f}" for v in np.concatenate([R[:, 0], R[:, 1]]))


# ---------------------------------------------------------------- model
def _wheel_geoms(M):
    """Collision as an ellipsoid (one contact point, no rim scrubbing when yawing); cylinder for looks only."""
    r, hw = M.wheel_radius, M.wheel_width / 2
    return (f'<geom type="ellipsoid" size="{r} {hw} {r}" mass="{M.wheel_mass}" rgba="0 0 0 0"/>\n        '
            f'<geom type="cylinder" size="{r} {hw}" zaxis="0 1 0" mass="0" rgba="0.08 0.08 0.08 1" contype="0" conaffinity="0"/>')


def _rover_xml(i, P: SimParams, cam_xyaxes):
    M = P.mujoco
    r, L = M.wheel_radius, M.wheel_separation
    cs, co = M.chassis_size, M.chassis_offset
    rgba = ROVER_COLORS[i % len(ROVER_COLORS)]
    cx, cy, cz = M.rover_cam_pos
    return f"""    <body name="rover{i}" pos="0 0 {r}">
      <freejoint name="rover{i}_free"/>
      <site name="rover{i}_site" pos="0 0 0" size="0.004" rgba="0 0 0 0"/>
      <geom name="rover{i}_chassis" type="box" size="{cs[0]} {cs[1]} {cs[2]}" pos="{co[0]} {co[1]} {co[2]}" mass="{M.chassis_mass}" rgba="{rgba}"/>
      <geom name="rover{i}_nose" type="box" size="0.012 0.03 0.008" pos="{co[0] + cs[0] + 0.012} 0 {co[2] + cs[2] - 0.008}" mass="0.005" rgba="0.96 0.96 0.96 1" contype="0" conaffinity="0"/>
      <geom name="rover{i}_caster" type="sphere" size="{M.caster_radius}" pos="{M.caster_offset_x} 0 {-(r - M.caster_radius)}" mass="0.02" friction="0.001 0.0 0.0" priority="1"/>
      <body name="rover{i}_wheel_l" pos="0 {L / 2} 0">
        <joint name="rover{i}_wl" type="hinge" axis="0 1 0"/>
        {_wheel_geoms(M)}
      </body>
      <body name="rover{i}_wheel_r" pos="0 {-L / 2} 0">
        <joint name="rover{i}_wr" type="hinge" axis="0 1 0"/>
        {_wheel_geoms(M)}
      </body>
      <camera name="rover{i}_cam" pos="{cx} {cy} {cz}" xyaxes="{cam_xyaxes}" fovy="{M.cam_fovy}"/>
    </body>
"""


def _blimp_xml(P: SimParams, bottom_offset: float):
    """Mocap body: ellipsoid envelope, gondola box and six thruster housings from BlimpParams."""
    B, M = P.blimp, P.mujoco
    gs = B.gondola_size
    pos, axes = B.thruster_geometry()               # NED body frame
    parts = [f"""    <body name="blimp" mocap="true" pos="0 0 1">
      <geom name="blimp_envelope" type="ellipsoid" size="{B.r_env} {B.r_env} {B.h_env / 2}" rgba="0.92 0.92 0.96 0.6" contype="0" conaffinity="0"/>
      <geom name="blimp_gondola" type="box" size="{gs[0] / 2} {gs[1] / 2} {gs[2] / 2}" pos="0 0 {-B.d_VT}" rgba="0.15 0.15 0.15 1" contype="0" conaffinity="0"/>
"""]
    for k in range(6):
        p_mj, a_mj = ned_to_mj_pos(pos[k]), ned_to_mj_pos(axes[k])
        parts.append(f'      <geom name="blimp_thruster{k}" type="cylinder" size="{B.thruster_radius} {B.thruster_length / 2}" '
                     f'pos="{p_mj[0]:.4f} {p_mj[1]:.4f} {p_mj[2]:.4f}" zaxis="{a_mj[0]:.4f} {a_mj[1]:.4f} {a_mj[2]:.4f}" '
                     f'rgba="0.25 0.25 0.28 1" contype="0" conaffinity="0"/>\n')
    cam_depth = bottom_offset + M.blimp_cam_offset
    parts.append(f'      <camera name="blimp_cam" pos="0 0 {-cam_depth:.4f}" xyaxes="{camera_xyaxes(M.blimp_cam_tilt_deg)}" fovy="{M.cam_fovy}"/>\n    </body>\n')
    return "".join(parts)


def build_mjcf(P: SimParams, n_rovers: int, bottom_offset: float) -> str:
    M = P.mujoco
    half = P.arena / 2
    rover_cam_xyaxes = camera_xyaxes(M.rover_cam_pitch_deg)
    parts = [f"""<mujoco model="blimp_team">
  <option timestep="{M.timestep}" integrator="implicitfast" gravity="0 0 -9.81"/>
  <visual>
    <global offwidth="{max(M.cam_width, 640)}" offheight="{max(M.cam_height, 480)}"/>
    <quality shadowsize="2048"/>
  </visual>
  <asset>
    <texture name="sky" type="skybox" builtin="gradient" rgb1="0.55 0.70 0.90" rgb2="0.92 0.95 1.00" width="256" height="256"/>
    <texture name="floor_tex" type="2d" builtin="checker" rgb1="0.80 0.80 0.78" rgb2="0.62 0.64 0.62" width="512" height="512" mark="edge" markrgb="0.35 0.35 0.35"/>
    <material name="floor_mat" texture="floor_tex" texrepeat="{P.arena * 2:.0f} {P.arena * 2:.0f}" reflectance="0.05"/>
  </asset>
  <default>
    <geom friction="{M.friction} 0.005 0.0001" condim="3"/>
    <velocity kv="{M.wheel_kv}" forcerange="-{M.wheel_torque_max} {M.wheel_torque_max}"/>
  </default>
  <worldbody>
    <light pos="0 0 4" dir="0 0 -1" diffuse="0.8 0.8 0.8" specular="0.2 0.2 0.2" castshadow="true"/>
    <light pos="3 -3 4" dir="-0.5 0.5 -1" diffuse="0.4 0.4 0.4" castshadow="false"/>
    <geom name="floor" type="plane" size="{half} {half} 0.05" material="floor_mat"/>
"""]
    if M.arena_walls:
        h, th = 0.08, 0.02
        walls = (((half, 0, h), (th, half, h)), ((-half, 0, h), (th, half, h)),
                 ((0, half, h), (half, th, h)), ((0, -half, h), (half, th, h)))
        for k, (pos, size) in enumerate(walls):
            parts.append(f'    <geom name="wall{k}" type="box" pos="{pos[0]} {pos[1]} {pos[2]}" size="{size[0]} {size[1]} {size[2]}" rgba="0.45 0.45 0.50 1"/>\n')
    for i in range(n_rovers):
        parts.append(_rover_xml(i, P, rover_cam_xyaxes))
    parts.append(_blimp_xml(P, bottom_offset))
    parts.append("  </worldbody>\n  <actuator>\n")
    for i in range(n_rovers):
        parts.append(f'    <velocity name="rover{i}_vl" joint="rover{i}_wl"/>\n    <velocity name="rover{i}_vr" joint="rover{i}_wr"/>\n')
    parts.append("  </actuator>\n  <sensor>\n")
    for i in range(n_rovers):
        parts.append(f'    <velocimeter name="rover{i}_vel" site="rover{i}_site"/>\n    <gyro name="rover{i}_gyro" site="rover{i}_site"/>\n'
                     f'    <jointvel name="rover{i}_wl_vel" joint="rover{i}_wl"/>\n    <jointvel name="rover{i}_wr_vel" joint="rover{i}_wr"/>\n')
    parts.append("  </sensor>\n</mujoco>\n")
    return "".join(parts)


# ---------------------------------------------------------------- rover proxy
class MujocoRover(Rover):
    """Drop-in for sim.rover.Rover whose motion comes from the shared MuJoCo world.

    `q` = [x, y, theta] in NED like the ideal rover; `v` = measured [forward speed, NED yaw
    rate] from the body velocimeter/gyro; `wheel_w` = measured wheel speeds [left, right].
    `step()` is a no-op: TeamSim steps the whole world once per physics step.

    Wheel velocity loop: MuJoCo's velocity actuator supplies the proportional term at the
    MuJoCo rate; the integral term (zero steady-state speed error under load, as on a PI motor
    driver) is applied at the physics rate as a setpoint bias with anti-windup.
    """

    def __init__(self, world: "MujocoWorld", i: int):
        super().__init__(world.P.rover)
        self.world, self.i = world, i
        m = world.model
        name = lambda kind, s: mujoco.mj_name2id(m, kind, s)  # noqa: E731
        self.body = name(mujoco.mjtObj.mjOBJ_BODY, f"rover{i}")
        jf = name(mujoco.mjtObj.mjOBJ_JOINT, f"rover{i}_free")
        self.qadr, self.vadr = m.jnt_qposadr[jf], m.jnt_dofadr[jf]
        wl, wr = name(mujoco.mjtObj.mjOBJ_JOINT, f"rover{i}_wl"), name(mujoco.mjtObj.mjOBJ_JOINT, f"rover{i}_wr")
        self.wheel_qadr = (m.jnt_qposadr[wl], m.jnt_qposadr[wr])
        self.wheel_vadr = (m.jnt_dofadr[wl], m.jnt_dofadr[wr])
        self.act = (name(mujoco.mjtObj.mjOBJ_ACTUATOR, f"rover{i}_vl"), name(mujoco.mjtObj.mjOBJ_ACTUATOR, f"rover{i}_vr"))
        sadr = lambda s: m.sensor_adr[name(mujoco.mjtObj.mjOBJ_SENSOR, s)]  # noqa: E731
        self.s_vel, self.s_gyro = sadr(f"rover{i}_vel"), sadr(f"rover{i}_gyro")
        self.s_wl, self.s_wr = sadr(f"rover{i}_wl_vel"), sadr(f"rover{i}_wr_vel")
        self.v = np.zeros(2)
        self.wheel_w = np.zeros(2)
        self.wheel_cmd = np.zeros(2)                # commanded wheel speeds [left, right] (rad/s)
        self._int = np.zeros(2)                     # integral of wheel speed error (rad)

    def reset(self, q0):
        x, y, th = np.asarray(q0, float)
        d, M = self.world.data, self.world.P.mujoco
        yaw_mj = -th
        d.qpos[self.qadr:self.qadr + 7] = [x, -y, M.wheel_radius, np.cos(yaw_mj / 2), 0.0, 0.0, np.sin(yaw_mj / 2)]
        d.qvel[self.vadr:self.vadr + 6] = 0.0
        for qa, va in zip(self.wheel_qadr, self.wheel_vadr):
            d.qpos[qa] = 0.0
            d.qvel[va] = 0.0
        d.ctrl[list(self.act)] = 0.0
        self.u[:] = 0.0
        self.wheel_cmd[:] = 0.0
        self._int[:] = 0.0
        self.world.forward()

    def command(self, v, w):
        super().command(v, w)                       # clips to v_max / w_max and stores u
        v, w = self.u
        M = self.world.P.mujoco
        # NED yaw rate is positive toward +y_ned (the rover's right): the left wheel speeds up.
        half = M.wheel_separation / 2
        self.wheel_cmd[:] = ((v + w * half) / M.wheel_radius, (v - w * half) / M.wheel_radius)
        self._apply_ctrl()

    def _apply_ctrl(self):
        if self.world.manual_ctrl:                  # an external GUI owns data.ctrl (wheel speed setpoints)
            return
        M = self.world.P.mujoco
        bias = (M.wheel_ki / M.wheel_kv) * self._int if M.wheel_kv > 0 else 0.0
        self.world.data.ctrl[list(self.act)] = self.wheel_cmd + bias

    def _update_velocity_loop(self, dt):
        """Integral action on the measured wheel speed error (called once per physics step)."""
        M = self.world.P.mujoco
        if M.wheel_ki <= 0 or self.world.manual_ctrl:
            self._int[:] = 0.0
            return
        self._int += (self.wheel_cmd - self.wheel_w) * dt
        lim = M.wheel_torque_max / M.wheel_ki          # anti-windup: integral torque stays within the motor limit
        np.clip(self._int, -lim, lim, out=self._int)
        self._apply_ctrl()

    def step(self, dt):
        return self.q

    def _refresh(self):
        d = self.world.data
        pos, R = d.xpos[self.body], d.xmat[self.body].reshape(3, 3)
        self.q[:] = (pos[0], -pos[1], -np.arctan2(R[1, 0], R[0, 0]))
        self.v[:] = (d.sensordata[self.s_vel], -d.sensordata[self.s_gyro + 2])
        self.wheel_w[:] = (d.sensordata[self.s_wl], d.sensordata[self.s_wr])


# ---------------------------------------------------------------- world
class MujocoWorld:
    def __init__(self, P: SimParams, n_rovers: int | None = None, blimp: Blimp | None = None):
        self.P = P
        self.n = P.task.n_rovers if n_rovers is None else n_rovers
        blimp = blimp or Blimp(P.blimp)
        self.bottom_offset = float(blimp.gondola_bottom_offset(np.zeros(3)))
        self.xml = build_mjcf(P, self.n, self.bottom_offset)
        self.model = mujoco.MjModel.from_xml_string(self.xml)
        self.data = mujoco.MjData(self.model)
        self.substeps = max(1, int(round(P.dt_phys / P.mujoco.timestep)))
        if abs(self.substeps * P.mujoco.timestep - P.dt_phys) > 1e-9:
            raise ValueError("dt_phys must be an integer multiple of mujoco.timestep")
        self.model.opt.timestep = P.dt_phys / self.substeps
        self.blimp_mocap = self.model.body_mocapid[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "blimp")]
        self.rovers = [MujocoRover(self, i) for i in range(self.n)]
        self.frame_names = {f"rover{i}": f"rover{i}_cam" for i in range(self.n)}
        self.frame_names["blimp"] = "blimp_cam"
        self.cam_ids = {key: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, cam) for key, cam in self.frame_names.items()}
        self._renderer = None
        self._renderer_thread = None
        self._warnings = 0
        # True: leave data.ctrl to an external GUI (MuJoCo viewer control sliders); the rover
        # controller's commands are still recorded in `u` but not applied.
        self.manual_ctrl = False
        self.reset()
        self.set_camera_angles()                # exact camera quaternions (the MJCF text is rounded)

    # ------------------------------------------------------------ stepping
    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        self._warnings = 0
        self.forward()

    def forward(self):
        mujoco.mj_forward(self.model, self.data)
        for r in self.rovers:
            r._refresh()

    def sync_blimp(self, eta):
        """Mirror the NumPy blimp pose (CV, NED) into the mocap body."""
        eta = np.asarray(eta, float)
        self.data.mocap_pos[self.blimp_mocap] = ned_to_mj_pos(eta[:3])
        self.data.mocap_quat[self.blimp_mocap] = mj_quat_from_rot(ned_to_mj_rot(R_zyx(*eta[3:6])))

    def step(self, dt):
        n = int(round(dt / self.model.opt.timestep))
        if abs(n * self.model.opt.timestep - dt) > 1e-9 or n < 1:
            raise ValueError("step dt must be an integer multiple of the MuJoCo timestep")
        for r in self.rovers:
            r._update_velocity_loop(dt)
        for _ in range(n):
            mujoco.mj_step(self.model, self.data)
        warnings = int(sum(w.number for w in self.data.warning))
        if warnings != self._warnings:
            self._warnings = warnings
            raise FloatingPointError("MuJoCo reported a simulation warning (instability or bad state)")
        for r in self.rovers:
            r._refresh()
        if not all(np.all(np.isfinite(r.q)) for r in self.rovers):
            raise FloatingPointError("Non-finite rover state from MuJoCo")

    # ------------------------------------------------------------ cameras
    def set_camera_angles(self, blimp_tilt_deg=None, rover_pitch_deg=None):
        """Re-aim the vehicle cameras on the live model (no rebuild): MuJoCo recomputes the world
        camera poses from model.cam_quat at the next forward pass."""
        M = self.P.mujoco
        if blimp_tilt_deg is not None:
            M.blimp_cam_tilt_deg = float(blimp_tilt_deg)
        if rover_pitch_deg is not None:
            M.rover_cam_pitch_deg = float(rover_pitch_deg)
        blimp_q = mj_quat_from_rot(camera_axes(M.blimp_cam_tilt_deg))
        rover_q = mj_quat_from_rot(camera_axes(M.rover_cam_pitch_deg))
        for key, cid in self.cam_ids.items():
            self.model.cam_quat[cid] = blimp_q if key == "blimp" else rover_q
        self.forward()

    def intrinsics(self):
        M = self.P.mujoco
        f = 0.5 * M.cam_height / np.tan(np.radians(M.cam_fovy) / 2)
        return {"width": M.cam_width, "height": M.cam_height, "fovy_deg": M.cam_fovy,
                "fx": f, "fy": f, "cx": M.cam_width / 2, "cy": M.cam_height / 2}

    def camera_pose_ned(self, key):
        """Camera position (NED) and rotation matrices for the OpenGL (x right, y up, -z forward)
        and OpenCV (x right, y down, z forward) conventions, columns = camera axes in NED."""
        cid = self.cam_ids[key]
        p = mj_to_ned_pos(self.data.cam_xpos[cid])
        # Columns are camera axes expressed in the world: only the world frame flips here
        # (unlike body attitudes, where the body frame flips too and R -> C R C).
        R_gl = C_FLIP @ self.data.cam_xmat[cid].reshape(3, 3)
        return {"position": p, "R_gl": R_gl, "R_cv": R_gl @ np.diag([1.0, -1.0, -1.0])}

    def renderer(self):
        tid = threading.get_ident()
        if self._renderer is None or self._renderer_thread != tid:   # GL contexts are thread-bound
            M = self.P.mujoco
            with _RENDERER_LOCK:
                self._renderer = mujoco.Renderer(self.model, height=M.cam_height, width=M.cam_width)
            self._renderer_thread = tid
        return self._renderer

    def render(self, k: int, t: float):
        """Render every vehicle camera at the current state; returns (frames, meta)."""
        r = self.renderer()
        frames = {}
        for key, cam in self.frame_names.items():
            r.update_scene(self.data, camera=cam)
            frames[key] = r.render().copy()
        meta = {"k": int(k), "t": float(t), "intrinsics": self.intrinsics(),
                "cameras": {key: self.camera_pose_ned(key) for key in self.frame_names}}
        return frames, meta

    def close(self):
        if self._renderer is not None and self._renderer_thread == threading.get_ident():
            self._renderer.close()
        self._renderer = None


# ---------------------------------------------------------------- recording
class FrameRecorder:
    """TeamSim.run callback: saves camera frames as PNG with an index of simulation-time stamps.

    index.csv columns: k, t, camera, file, then the camera position (NED, m) and OpenCV-convention
    orientation quaternion (w, x, y, z) in NED. cameras.json holds the shared intrinsics.
    """

    def __init__(self, directory, every: int = 1):
        os.makedirs(directory, exist_ok=True)
        self.dir, self.every = directory, max(1, int(every))
        self.rows, self.intrinsics = [], None

    def __call__(self, sim):
        meta = sim.frame_meta
        if not sim.frames or meta["k"] % self.every:
            return
        self.intrinsics = meta["intrinsics"]
        for key, img in sim.frames.items():
            fn = f"{key}_{meta['k']:06d}.png"
            write_png(os.path.join(self.dir, fn), img)
            cam = meta["cameras"][key]
            quat = mj_quat_from_rot(cam["R_cv"])
            self.rows.append((meta["k"], meta["t"], key, fn, *cam["position"], *quat))

    def close(self):
        import json
        with open(os.path.join(self.dir, "index.csv"), "w") as f:
            f.write("k,t,camera,file,px,py,pz,qw,qx,qy,qz\n")
            for row in self.rows:
                f.write(",".join(str(v) if isinstance(v, (int, str)) else f"{v:.6f}" for v in row) + "\n")
        with open(os.path.join(self.dir, "cameras.json"), "w") as f:
            json.dump({"intrinsics": self.intrinsics, "position_frame": "NED (x north, y east, z down), metres",
                       "orientation": "quaternion (w, x, y, z) of camera axes in NED, OpenCV convention (x right, y down, z forward)",
                       "timing": "frame k rendered after the physics of control step k-1, i.e. at the state seen by controller call k"},
                      f, indent=2)
        return len(self.rows)
