"""All simulator parameters in one place.

Values marked [paper] come from Tao, Cha, Hou & Zhang, "Parameter Identification of Blimp
Dynamics through Swinging Motion", ICARCV 2018 (GT-MAB). Values marked [slider] are not yet
measured; they have physically plausible defaults and are exposed in the live GUI.

Frames (as in the paper): inertial NED with Z DOWN and floor at z = 0.
The raw z coordinate locates CV; altitude is clearance beneath the gondola assembly.
Body frame at the centre of volume (CV = CB): x forward, y right, z down.
"""
from __future__ import annotations

from dataclasses import dataclass, field, fields
import numpy as np

G = 9.81
RHO_AIR = 1.161      # kg/m^3 at ~300 K [paper]
RHO_HE = 0.164       # kg/m^3           [paper]


@dataclass
class BlimpParams:
    # --- geometry / inertia -------------------------------------------------
    m: float = 0.1249            # total mass incl. helium (neutral)        [paper]
    d_VM: float = 0.0971         # CV -> CM, CM below CV (m)                 [paper]
    d_VT: float = 0.26           # CV -> thrust plane (m)                    [paper]
    I_CM_xy: float = 0.005821    # pitch = roll inertia about CM (kg m^2)    [paper]
    I_z: float = 0.0090          # yaw inertia about CM (kg m^2)             [slider]
    r_env: float = 0.349         # inflated envelope radius (m)              [paper, 0.7627*0.457]
    h_env: float = 0.44          # envelope height (m)                        [paper]
    # --- damping --------------------------------------------------------------
    b_xy: float = 0.000980       # linear pitch/roll damping (N m s/rad)      [paper]
    b_z: float = 0.0020          # linear yaw damping (N m s/rad)             [slider]
    d_quad_yaw: float = 0.005    # quadratic YAW damping (N m s^2/rad^2); pitch/roll keep the paper's linear b [slider]
    d_lin: float = 0.03          # linear translational drag (N s/m), all axes [slider]
    d_quad_xy: float = 0.07      # quadratic drag horizontal (N s^2/m^2)      [slider]
    d_quad_z: float = 0.11       # quadratic drag vertical (N s^2/m^2)        [slider]
    # --- added mass (fraction of displaced-air mass m_f = m) -------------------
    k_add_xy: float = 0.20       # horizontal added-mass coefficient          [slider]
    k_add_z: float = 0.80        # vertical added-mass coefficient            [slider]
    k_add_rot: float = 0.10      # rotational added inertia (fraction of I_CM) [slider]
    # --- thrusters: BlueROV2-style 6-thruster layout on the gondola -----------
    l_h: float = 0.10            # horizontal thrusters at (+-l_h, +-l_h, d_VT) [slider]
    l_v: float = 0.08            # vertical thrusters at (+-l_v, 0, d_VT)       [slider]
    T_max: float = 0.03          # max thrust per thruster (N); 0.1 N gave 1.2 m/s and 30 deg swings [slider]
    tau_p: float = 0.10          # thruster first-order lag (s)                 [slider]
    batt_scale: float = 1.0      # battery voltage scale on thrust              [slider]
    # --- trim: buoyancy minus weight ---------------------------------------------
    net_lift_N: float = 0.0      # B - W (N); GUI exposes equivalent grams via net_lift_g
    net_lift_drift: float = 0.0  # random-walk std of net lift (N / sqrt(s)), 0 = off [slider]
    # --- disturbance -----------------------------------------------------------
    draught_N: float = 0.0       # OU draught force std (N), 0 = off           [slider]
    draught_tau: float = 2.0     # OU correlation time (s)

    @property
    def m_f(self) -> float:        # displaced air mass (= m when neutral)
        return self.m

    @property
    def V(self) -> float:
        return self.m / RHO_AIR

    @property
    def net_lift_g(self) -> float:
        """Signed equivalent weight in grams; changes trim force, not vehicle mass."""
        return 1000.0 * self.net_lift_N / G

    @net_lift_g.setter
    def net_lift_g(self, value: float):
        self.net_lift_N = float(value) * G / 1000.0

    @property
    def gondola_size(self) -> np.ndarray:
        """Shared rendered/contact box dimensions in body xyz (metres)."""
        return np.array([self.r_env * 0.82, self.r_env * 0.46, 0.10])

    @property
    def thruster_length(self) -> float:
        """Motor housing length along its thrust axis (metres)."""
        return 0.13

    @property
    def thruster_radius(self) -> float:
        """Motor housing radius (metres)."""
        return 0.025

    def M_RB(self) -> np.ndarray:
        """Rigid-body mass matrix about CV with CM offset r_g = (0, 0, +d_VM) (Fossen 3.3)."""
        m, zg = self.m, self.d_VM
        I_cm = np.diag([self.I_CM_xy, self.I_CM_xy, self.I_z])
        S = skew(np.array([0.0, 0.0, zg]))
        I_o = I_cm - m * S @ S                      # parallel axis to CV
        M = np.zeros((6, 6))
        M[:3, :3] = m * np.eye(3)
        M[:3, 3:] = -m * S
        M[3:, :3] = m * S
        M[3:, 3:] = I_o
        return M

    def M_A(self) -> np.ndarray:
        mf = self.m_f
        Ia = self.k_add_rot * self.I_CM_xy
        return np.diag([self.k_add_xy * mf, self.k_add_xy * mf, self.k_add_z * mf, Ia, Ia, Ia])

    def thruster_geometry(self):
        """Positions (6,3) and unit thrust axes (6,3) in body frame (z down).
        0..3 horizontal at 45 deg (BlueROV2 pattern), 4..5 vertical (thrust up = -z)."""
        l, d, lv = self.l_h, self.d_VT, self.l_v
        c = np.sqrt(0.5)
        pos = np.array([[+l, +l, d], [+l, -l, d], [-l, +l, d], [-l, -l, d], [+lv, 0, d], [-lv, 0, d]])
        axes = np.array([[c, -c, 0], [c, c, 0], [c, c, 0], [c, -c, 0], [0, 0, -1], [0, 0, -1]])
        return pos, axes


@dataclass
class RoverParams:
    v_max: float = 0.5           # m/s
    w_max: float = 3.0           # rad/s
    d_hand: float = 0.08         # hand-point offset ahead of the axle (m)
    body_len: float = 0.16       # drawing only


@dataclass
class MujocoParams:
    """Contact-based rover team in MuJoCo; used only when SimParams.rover_backend == "mujoco".

    Rover geometry is TurtleBot3 Burger-like (wheel radius 0.033 m, wheel separation 0.16 m,
    about 1 kg). The MuJoCo world is z-up; the rest of the simulator is NED. Nothing here is a
    hardware measurement of the lab platform.
    """
    timestep: float = 0.002            # MuJoCo integration step (s); dt_phys must be a multiple
    wheel_radius: float = 0.033        # m
    wheel_separation: float = 0.160    # m, between wheel centres
    wheel_width: float = 0.018         # m
    wheel_mass: float = 0.03           # kg each
    chassis_size: tuple = (0.07, 0.07, 0.06)    # half-sizes (m), body frame x forward / y left / z up
    chassis_offset: tuple = (-0.03, 0.0, 0.05)  # chassis centre relative to the axle midpoint (m)
    chassis_mass: float = 0.85         # kg
    caster_radius: float = 0.012       # m, frictionless rear ball
    caster_offset_x: float = -0.09     # m, behind the axle
    wheel_kv: float = 0.004            # wheel velocity-loop P gain (N m s/rad); ~0.12 s speed time constant
    wheel_ki: float = 0.004            # wheel velocity-loop I gain (N m/rad) at the physics rate; ~11 % speed overshoot, <0.5 % steady error; 0 = P only
    wheel_torque_max: float = 0.15     # N m per wheel
    friction: float = 1.0              # sliding friction, floor/wheels
    arena_walls: bool = True           # low walls at the arena boundary (collide with rovers)
    # --- vehicle cameras (rendered only when SimParams.cameras is True) ---
    cam_width: int = 640
    cam_height: int = 480
    cam_fovy: float = 90.0             # vertical field of view (deg)
    rover_cam_pos: tuple = (0.065, 0.0, 0.13)  # relative to the axle midpoint (m); just ahead of and above the chassis front edge
    rover_cam_pitch_deg: float = 10.0  # rover camera tilt below the horizon (deg, + down); live-adjustable
    blimp_cam_offset: float = 0.01     # camera this far below the lowest gondola surface (m)
    blimp_cam_tilt_deg: float = 90.0   # blimp camera tilt: 90 = straight down, 0 = straight ahead (body x); live-adjustable


@dataclass
class TaskParams:
    n_rovers: int = 4
    circle_center: tuple = (0.0, 0.0)
    circle_radius: float = 1.5   # m
    rover_speed: float = 0.20    # m/s along the circle
    blimp_height: float = 1.0    # m clearance under the lowest gondola assembly point
    blimp_yaw_ref: float = 0.0   # rad
    pid_enabled: bool = True     # live toggle (GUI PID button / H key): PID off -> thrusters idle (scenario) or pure manual (teleop)
    kp_pos: float = 0.035        # N/m   blimp position PID
    kd_pos: float = 0.11         # N s/m
    ki_pos: float = 0.004        # N/(m s); 0 -> plain PD
    i_max_N: float = 0.20        # anti-windup clamp on the integral force (N); must exceed |net lift| to hold height
    kp_yaw: float = 0.004        # N m/rad
    kd_yaw: float = 0.006        # N m s/rad
    k_hand: float = 1.5          # 1/s   rover hand-point tracking gain


@dataclass
class ViewParams:
    force_scale: float = 40.0    # m of arrow per N of force (forces are ~0.01-0.1 N)
    show_components: bool = True


@dataclass
class SimParams:
    dt_phys: float = 0.01
    dt_ctrl: float = 0.05
    T_end: float = 40.0
    seed: int = 0
    arena: float = 6.0           # m, square arena side (drawing/limits)
    floor_alt: float = 0.0       # m, minimum gondola-bottom clearance
    ceiling_alt: float = 4.0     # m, maximum gondola-bottom clearance
    rover_backend: str = "ideal" # "ideal" (exact unicycle) or "mujoco" (contact-based, needs the mujoco package)
    cameras: bool = False        # render vehicle cameras each control step (requires rover_backend == "mujoco")
    blimp: BlimpParams = field(default_factory=BlimpParams)
    rover: RoverParams = field(default_factory=RoverParams)
    mujoco: MujocoParams = field(default_factory=MujocoParams)
    task: TaskParams = field(default_factory=TaskParams)
    view: ViewParams = field(default_factory=ViewParams)


# Sliders shown in the GUI: (object, field, min, max)
SLIDERS = [
    ("blimp", "T_max", 0.005, 0.30), ("blimp", "tau_p", 0.02, 0.50), ("blimp", "d_quad_yaw", 0.0, 0.03),
    ("blimp", "d_lin", 0.0, 0.20), ("blimp", "d_quad_xy", 0.0, 0.30), ("blimp", "d_quad_z", 0.0, 0.40),
    ("blimp", "k_add_xy", 0.0, 1.0), ("blimp", "k_add_z", 0.0, 1.5), ("blimp", "I_z", 0.002, 0.03),
    ("blimp", "b_z", 0.0, 0.01), ("blimp", "b_xy", 0.0, 0.005), ("blimp", "draught_N", 0.0, 0.05),
    ("blimp", "net_lift_g", -10.0, 10.0), ("blimp", "net_lift_drift", 0.0, 0.003),
    ("task", "kp_pos", 0.0, 0.15), ("task", "kd_pos", 0.0, 0.40), ("task", "ki_pos", 0.0, 0.02), ("task", "i_max_N", 0.0, 0.5),
    ("task", "circle_radius", 0.5, 2.5), ("task", "rover_speed", 0.0, 0.5), ("task", "blimp_height", 0.0, 2.5),
    ("view", "force_scale", 1.0, 200.0),
]


# Browser-only controls for the MuJoCo backend (the Matplotlib panel is laid out for SLIDERS alone):
# camera tilt angles in degrees, applied to the live MuJoCo model without a rebuild.
MUJOCO_SLIDERS = [
    ("mujoco", "blimp_cam_tilt_deg", 0.0, 90.0), ("mujoco", "rover_cam_pitch_deg", -30.0, 60.0),
]


def skew(a: np.ndarray) -> np.ndarray:
    return np.array([[0.0, -a[2], a[1]], [a[2], 0.0, -a[0]], [-a[1], a[0], 0.0]])


def as_dict(p) -> dict:
    return {f.name: getattr(p, f.name) for f in fields(p)}
