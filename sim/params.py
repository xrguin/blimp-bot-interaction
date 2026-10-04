"""All simulator parameters in one place.

Values marked [paper] come from Tao, Cha, Hou & Zhang, "Parameter Identification of Blimp
Dynamics through Swinging Motion", ICARCV 2018 (GT-MAB). Values marked [slider] are not yet
measured; they have physically plausible defaults and are exposed in the live GUI.

Frames (as in the paper): inertial NED with Z DOWN, floor at z = 0, altitude = -z.
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
    net_lift_N: float = 0.0      # B - W (N); > 0 rises; 0 = exactly neutral      [slider]
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
class TaskParams:
    n_rovers: int = 4
    circle_center: tuple = (0.0, 0.0)
    circle_radius: float = 1.5   # m
    rover_speed: float = 0.20    # m/s along the circle
    blimp_height: float = 1.0    # m above the floor (z = -height)
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
    floor_alt: float = 0.05      # m, blimp cannot go below (gondola on the floor)
    ceiling_alt: float = 4.0     # m
    blimp: BlimpParams = field(default_factory=BlimpParams)
    rover: RoverParams = field(default_factory=RoverParams)
    task: TaskParams = field(default_factory=TaskParams)
    view: ViewParams = field(default_factory=ViewParams)


# Sliders shown in the GUI: (object, field, min, max)
SLIDERS = [
    ("blimp", "T_max", 0.005, 0.30), ("blimp", "tau_p", 0.02, 0.50), ("blimp", "d_quad_yaw", 0.0, 0.03),
    ("blimp", "d_lin", 0.0, 0.20), ("blimp", "d_quad_xy", 0.0, 0.30), ("blimp", "d_quad_z", 0.0, 0.40),
    ("blimp", "k_add_xy", 0.0, 1.0), ("blimp", "k_add_z", 0.0, 1.5), ("blimp", "I_z", 0.002, 0.03),
    ("blimp", "b_z", 0.0, 0.01), ("blimp", "b_xy", 0.0, 0.005), ("blimp", "draught_N", 0.0, 0.05),
    ("blimp", "net_lift_N", -0.15, 0.15), ("blimp", "net_lift_drift", 0.0, 0.003),
    ("task", "kp_pos", 0.0, 0.15), ("task", "kd_pos", 0.0, 0.40), ("task", "ki_pos", 0.0, 0.02), ("task", "i_max_N", 0.0, 0.5),
    ("task", "circle_radius", 0.5, 2.5), ("task", "rover_speed", 0.0, 0.5), ("task", "blimp_height", 0.3, 2.5),
    ("view", "force_scale", 1.0, 200.0),
]


def skew(a: np.ndarray) -> np.ndarray:
    return np.array([[0.0, -a[2], a[1]], [a[2], 0.0, -a[0]], [-a[1], a[0], 0.0]])


def as_dict(p) -> dict:
    return {f.name: getattr(p, f.name) for f in fields(p)}
