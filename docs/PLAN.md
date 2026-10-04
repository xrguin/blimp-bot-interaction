# Blimp + ground-robot team — simulation and modular world model: design plan

Status: planning only (no code). Version 2026-10-04, revised after reading
Tao, Cha, Hou & Zhang, "Parameter Identification of Blimp Dynamics through Swinging Motion",
ICARCV 2018 (`ICARV_08581376.pdf` in this folder). Companion document:
`PROBLEM_FORMULATION.md` (symbols used here are defined there).

Decisions so far: Python simulator; GT-MAB-class blimp (saucer envelope, gondola below,
thrusters below the centre of volume) modelled in 6 DoF; differential-drive rovers commanded by
body velocity (v, ω) through an onboard velocity loop; first task = formation keeping; blimp is
sensor and active agent; motion capture (OptiTrack-class) available and used for the blimp's own
pose at test time; 3–4 rovers on hardware; headline result = scale to more robots without
retraining.

---

## 1. Headline claim and secondary claims

> One rover module identified on a single robot and one blimp model assembled from
> separately identified sub-modules compose into a team model for N robots with no retraining,
> matching or beating a monolithic team model trained per N, at a fraction of the data.

Secondary: (i) the simulator's ground truth (mocap on hardware) labels every module interface,
so each module is verified alone; (ii) the blimp itself is a modular world model in miniature —
two grey-box modules the GT-MAB papers already identified plus a learned coupling residual
they left as future work; (iii) the planner factorizes the same way as the model.

---

## 2. What the paper fixes about the blimp (GT-MAB)

Frames (adopted as-is so parameters drop in): inertial frame with Z down (NED-style); body
frame at the centre of volume (CV), which coincides with the centre of buoyancy (CB) for the
symmetric saucer envelope. Centre of mass (CM) lies on the body Z axis below CV.

Identified or measured values (Tables I–III, Eq. 21):

| symbol | value | meaning |
|---|---|---|
| m | 0.1249 kg | total mass incl. helium (= ρ_air V, neutral) |
| d_VM | 0.0971 m | CV → CM distance (CM below) |
| d_VT | 0.26 m | CV → thrust line distance (= H_ENV/2 + H_GON) |
| H_ENV, H_GON | 0.44 m, 0.04 m | envelope height, gondola height |
| r_deflated | 0.457 m | deflated envelope radius; inflated radius ≈ 0.7627 r = 0.349 m |
| I_CM (pitch = roll) | 0.005821 kg·m² | identified from swing tests |
| b | 0.000980 N·m·s/rad | linear rotational damping about CM |
| ρ_air, ρ_He | 1.161, 0.164 kg/m³ | at ~300 K |
| thrusters | five: f_x, f_y, f_z, τ_z | mounted on the gondola, below CV |
| f_x(u) | measured curve vs. V_motor = V_batt·u (Fig. 4) | numbers not in text — needed |

Identified pitch (and, by symmetry, roll) dynamics about CM, with θ the pitch angle and
f(u) the thrust along body x:

    θ̈ = −20.4284 sin θ − 0.1684 θ̇ + 27.9933 f(u)        (paper Eq. 21)
      = [ −m g d_VM sin θ − b θ̇ + (d_VT − d_VM) f(u) ] / I_CM

Not in this paper (sources to pull next): translational and yaw dynamics — mass plus added
mass, drag, thrust map — from Cho et al., "Autopilot design for a class of miniature autonomous
blimps", IEEE CCTA 2017; the coupled translation–rotation model and swing-reducing control from
the group's later GT-MAB papers (Tao et al., ~2020–2021; NSF PAR records 10212076 / 10212088
appear to be these — verify and fetch). Until then, translational drag and added mass are
placeholders, flagged in the parameter file.

---

## 3. Architecture

| Module | Type | Interface out | Trained from | Reused |
|---|---|---|---|---|
| M_R rover dynamics (one copy per robot, shared parameters) | known unicycle + learned (v, ω) tracking residual | (x, y, θ, v, ω) | one rover's mocap log | N times |
| M_C rover context | inferred from a ~1 s history window | θ_i (gain, lag, slip) | 1–2 rovers with varied parameters | per robot, no retraining |
| M_B^trans blimp translation + yaw | grey-box (Cho 2017 parameters) + small residual | (x, y, z, ψ, u, v, w, r) | blimp APRBS log | once |
| M_B^swing blimp pitch/roll pendulum | grey-box, paper Eq. 21 | (φ, θ, p, q) | free-swing logs (as in the paper) | once |
| M_B^couple translation ↔ swing coupling | learned residual (the paper's "future work") | wrench corrections | blimp flight logs with thrust on | once |
| M_P perception | camera stub now, learned later | rover poses with covariance, IDs | mocap-labelled frames | once |
| M_I interaction (formation) | none learned; coupling lives in the planner | — | — | — |
| Planner | centralized MPC over N rovers + blimp; distributed later | u_i = (v, ω)_i, u_B | — | — |

Design rule: every boundary carries a quantity mocap can measure; module-preferred coordinates
stay inside modules.

The blimp split deserves emphasis: M_B^swing is an identified pendulum, M_B^trans an
identified translational model, and the thrusters-below-CV torque τ_y = (d_VT − d_VM) f_x is
*known* coupling. What is unknown is the aerodynamic and added-mass coupling the papers did not
identify — that is the learned residual, and its data budget is the blimp-side analogue of the
underwater payload experiment.

---

## 4. Simulator specification

Physics step 0.01 s; control 20 Hz; camera 30 Hz; all randomness via per-episode seeds.
Logging in (X, U, X_next, Eta) npz per agent, same field names as the underwater repo.

### 4.1 Blimp (6 DoF, Fossen form, parameters from §2)

With η = [x, y, z, φ, θ, ψ] the pose in the inertial frame (Z down), ν = [u, v, w, p, q, r] the
body velocity at CV, τ the thruster wrench, and τ_d a disturbance wrench:

    M ν̇ + C(ν) ν + D(ν) ν + g(η) = τ + τ_d,      η̇ = J(η) ν

* Rigid body about CV with CM offset r_g = (0, 0, +d_VM) (down in body frame): M_RB includes
  the m·S(r_g) cross terms; I about CM from I_CM = 0.005821 (pitch, roll) and I_z (unknown —
  placeholder from the ellipsoid CAD, to be fitted from yaw steps).
* Added mass M_A for an *oblate* spheroid (saucer: semi-axes a = b = 0.349 m, c = 0.22 m),
  Lamb coefficients for the oblate case (not the prolate formulas used in the earlier plan),
  as placeholders until the 2020–21 papers' values are in hand.
* Damping: rotational linear damping b on p and q (identified); translational drag and yaw
  damping from Cho 2017 or fitted; quadratic terms optional.
* Restoring g(η): buoyancy ρ_air g V at CV, weight m g at CM; neutral trim; pendulum restoring
  m g d_VM sin θ reproduces Eq. 21 exactly when translation is frozen — this is the first unit
  test of the simulator.
* Thrusters: five units on the gondola at depth d_VT below CV: a left/right pair along body x
  (f_x and τ_z), one along body y (f_y), two along body z (f_z); thrust map f(u) from Fig. 4
  (to be digitised or supplied), first-order lag τ_p; command u_B ∈ [−1, 1]⁴ = (x-pair sum,
  x-pair difference, y, z). Because all thrust lines are d_VT below CV, every horizontal thrust
  induces the swing torque (d_VT − d_VM) f about CM — exactly the paper's mechanism.
* Disturbances: draught as an Ornstein–Uhlenbeck horizontal force; slow buoyancy drift as a
  random walk on the trim (the "slow context").
* Validation before use: (a) free-swing response from θ₀ matches Eq. 21 (NRMSE comparable to
  the paper's ~84 %); (b) step responses in surge/yaw match Cho 2017 once its parameters are
  in.

### 4.2 Rover (differential drive)

State (x, y, θ, v, ω); command (v_c, ω_c):

    ẋ = v cos θ,  ẏ = v sin θ,  θ̇ = ω
    v̇ = (κ_v,i sat(v_c) − v)/τ_v,i + w_v,    ω̇ = (κ_ω,i sat(ω_c) − ω)/τ_ω,i + w_ω

per-robot gains κ ∈ [0.9, 1.0], lags τ_v ≈ 0.15 s, τ_ω ≈ 0.10 s (±20 %), v_max 0.5 m/s,
ω_max 3 rad/s, wheelbase 0.16 m, wheel radius 0.033 m (placeholders — rover model to be
confirmed). The per-robot parameters are the hidden context θ_i.

### 4.3 Camera stub

Pinhole camera on the gondola (orientation to be confirmed — the GT-MAB camera in the human-
following work faces forward; a downward view is assumed for team tracking). 640 × 480, 90°
FOV, 30 Hz; two markers per rover; pixel noise σ = 1 px; dropout outside the image; back-
projection to the floor plane with the blimp's mocap pose gives (x, y, θ) and a covariance per
rover; nearest-neighbour identity tracking. Note the swing: a pitching blimp moves the camera
footprint by ≈ h·tan θ (0.5 m at h = 3 m and θ = 10°), so M_B^swing matters for perception
too — a second, non-obvious coupling between modules.

### 4.4 Controllers for data generation

APRBS excitation per axis (0.4–1.0 s holds, deployment box); formation keeping via a virtual
leader with offsets and point-ahead unicycle tracking, pairwise repulsion; blimp station-
keeping over the team centroid with altitude hold and a swing-aware filter on the camera
pose. Episodes 60 s; 20 sysID episodes per robot type; 20 formation episodes per N ∈ {2, 3, 4, 8}.

---

## 5. Training protocol (staged, each module on its own data — see formulation §4)

1. M_R on rover 1 (ridge/ARX residual on (v, ω) tracking; MLP variant); verify K-step RMSE.
2. M_C on rovers 1–2 with randomized parameters; verify on unseen robots.
3. M_B^swing: confirm the simulator reproduces Eq. 21 from free-swing data (closes the loop with
   the paper); M_B^trans from APRBS; M_B^couple as a residual on the composed grey-box model,
   with a data-vs-error curve.
4. Compose and verify on formation episodes for each N.
5. M_P later; rerun step 4 with M_P in the loop.
6. MPC on the composed model; collision avoidance as hard pairwise constraints or CBFs.

---

## 6. Experiments

E1 rover data efficiency; E2 scaling to N (headline, modular vs monolithic-per-N);
E3 blimp: swing-module validation, coupling-residual data budget, buoyancy-drift context;
E4 closed loop with learned model in MPC, mocap vs camera stub; E5 (optional) jointly trained
multi-agent world model baseline. Details in the previous plan version carry over.

---

## 7. Repository layout and milestones

    blimp_team/   README.md  PLAN.md  PROBLEM_FORMULATION.md
                  blimp.py rover.py camera.py sim.py controllers.py collect.py
                  models/ eval/ exp/ data/ results/
    M1 dynamics + swing unit test · M2 collectors + formation controller · M3 modules + E1/E3
    M4 composition + E2 figure · M5 MPC (E4) · M6 learned perception

---

## 8. Open questions (answers change the design)

1. Is the lab blimp the GT-MAB itself (same envelope, five thrusters) or a variant? Exact
   thruster positions and the f(u) curve of Fig. 4 (numbers), battery voltage range.
2. Do you have Cho et al. CCTA 2017 and the later coupled-model / swing-control papers? They
   supply the translational parameters and may already contain the coupling we plan to learn.
3. Camera: forward or downward facing, resolution, frame rate; is it on the gondola (so it
   swings with the pendulum)?
4. Mocap: OptiTrack, update rate, whether rovers are also tracked (they must be, for labels).
5. Rovers: which platform (wheelbase, max speeds, onboard velocity loop bandwidth), comms
   latency from the planner.
6. Should the blimp carry a certified core (EDMDc on the grey-box model) so a guarantee story
   exists, or is nominal-plus-residual enough on this platform?
7. Formation first is agreed; is pushing or herding the intended second study?

---

## 9. References

Tao, Cha, Hou, Zhang, "Parameter Identification of Blimp Dynamics through Swinging Motion",
ICARCV 2018 (in folder). Cho et al., "Autopilot design for a class of miniature autonomous
blimps", IEEE CCTA 2017 (cited by the above; to fetch). Li, Nahon, Sharf, "Airship dynamics
modeling: A literature review", Progress in Aerospace Sciences 47(3), 2011 (confirmed via the
paper's reference list). Fossen, Handbook of Marine Craft Hydrodynamics and Motion Control,
Wiley 2011. Battaglia et al., Interaction Networks, NeurIPS 2016. Kumar et al., RMA, RSS 2021.
Lee et al., CaDM, ICML 2020. Egorov & Shpilman, MAMBA, AAMAS 2022; Zhang et al., MARIE, TMLR
2025. Bauersfeld et al., NeuroBEM, RSS 2021; Chee et al., KNODE-MPC, RA-L 2022.
