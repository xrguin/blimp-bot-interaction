# Problem formulation — modular world model for a blimp + ground-robot team

Version 2026-10-04. Companion to `PLAN.md`. Every symbol is defined where it first appears.
Formation keeping in simulation is the first instance; the formulation is written so that
pushing/herding and hardware are extensions, not rewrites.

---

## 1. System

Agents: one blimp B and N rovers indexed by i ∈ {1, …, N}. Discrete time k at the control
period Δt = 0.05 s. The inertial frame has Z down (paper convention); the floor is z = h_floor.

**Blimp state** x_B = (η_B, ν_B) ∈ ℝ¹², where η_B = (x, y, z, φ, θ, ψ) is the pose of the
centre of volume (position; roll, pitch, yaw) and ν_B = (u, v, w, p, q, r) the body-frame
linear and angular velocity. Hidden slow parameter β_k ∈ ℝ: buoyancy trim offset (drifts).
Input u_B ∈ [−1, 1]⁴: x-thruster-pair sum (surge), x-pair difference (yaw), y thruster (sway),
z thrusters (heave).

**Rover state** x_i = (p_i, θ_i, v_i, ω_i) ∈ ℝ⁵: planar position p_i ∈ ℝ², heading θ_i,
forward speed v_i, yaw rate ω_i. Hidden constant parameters ϑ_i = (κ_v, κ_ω, τ_v, τ_ω)_i:
velocity-loop gains and time constants. Input u_i = (v_c, ω_c)_i.

**True dynamics** (the simulator; on hardware, the physical system):

    x_B,k+1 = f_B(x_B,k, u_B,k, β_k, d_k)          d_k: draught disturbance
    x_i,k+1 = f_R(x_i,k, u_i,k, ϑ_i, w_i,k)        w_i,k: process noise
    β_k+1   = β_k + ε_k                            slow random walk

**Observations.** Training: mocap gives x_B,k and all x_i,k. Test: mocap gives x_B,k; rover
states only through the blimp camera, z_i,k = h(x_i,k, x_B,k) + n_k, delivered with delay d_c
steps, with dropouts; data association (which detection is which rover) is not given.

**Task.** Formation keeping: a virtual leader pose (p_L,k, θ_L,k) follows a reference path;
rover i should sit at p_L,k + R(θ_L,k) o_i, with R(·) the planar rotation and o_i a fixed
offset. The blimp keeps the team centroid inside its camera footprint at altitude h*. Stage
cost at step k over a horizon H:

    ℓ_k = Σ_i ‖p_i,k − p_L,k − R(θ_L,k) o_i‖² + λ_B ‖c_B(x_B,k) − p̄_k‖² + λ_u (Σ_i ‖u_i,k‖² + ‖u_B,k‖²)

where c_B(·) is the camera footprint centre on the floor and p̄_k the rover centroid, subject to
‖p_i,k − p_j,k‖ ≥ r_min for all i ≠ j and input bounds. Receding-horizon: minimise
Σ_{k'=k}^{k+H−1} ℓ_k' under the *model* (below), apply the first input, repeat.

---

## 2. The modular world model

A world model here is a predictor of the composite state ŝ_k = (x̂_B, x̂_1, …, x̂_N, β̂, ϑ̂_1, …,
ϑ̂_N, a_k), with a_k the identity assignment, built from the following parameterised maps.
Each row is a module with a typed interface (quantities in physical units that mocap can label).

| module | map | parameters | data it is trained on |
|---|---|---|---|
| M_R rover dynamics | x̂_i,k+1 = f̂_R(x_i,k, u_i,k; ϑ̂_i, φ_R) | φ_R shared by all i | D_R: one rover, mocap |
| M_C rover context | ϑ̂_i = g_C(H_i,k; φ_C), H_i,k = last W pairs (x_i, u_i) | φ_C | D_C: 1–2 rovers, varied ϑ |
| M_B^trans | translation + yaw part of f̂_B, grey-box | φ_B1 | D_B1: APRBS flights |
| M_B^swing | pitch/roll pendulum (paper Eq. 21) | φ_B2 (I_CM, b) | D_B2: free swings |
| M_B^couple | residual wrench r(x_B, u_B; φ_B3) added to the composed grey-box | φ_B3 | D_B3: flights with thrust on |
| M_P perception | ẑ_i,k, Σ_i,k = ĥ(image_k, x_B,k; φ_P) | φ_P | D_P: mocap-labelled frames |
| Estimator | x̂_i,k = Filter(x̂_i,k−1, ẑ_i,k−d_c, u_i,k−1), a_k = Assoc(history) | none learned | — |
| Composition | ŝ_k+1 = F(ŝ_k, u_k) = ( f̂_B, f̂_R(·; ϑ̂_1), …, f̂_R(·; ϑ̂_N) ) | — | — |

For formation there is no learned interaction module: inter-rover coupling enters only through
the planner's constraints. For pushing, an object module and per-contact modules are added to
F; for herding, a behaviour module for uncontrolled rovers is added.

---

## 3. What "modular" means, precisely

Let φ = (φ_R, φ_C, φ_B1, φ_B2, φ_B3, φ_P) and let L be the total training objective.

* **Modular (this work):** L(φ) = Σ_m L_m(φ_m; D_m) — block-separable; every module is fit on
  its own dataset with labels at its own interface; no term couples two blocks.
* **Staged (V-JEPA 2-AC, DINO-WM, RMA):** triangular dependence, L_2(φ_2; D_2, φ_1) with φ_1
  frozen — the later module depends on the earlier one, not vice versa.
* **Joint (Dreamer, TD-MPC2, C-SWM, CEE-US):** one non-separable L(φ).

Three consequences of block separability: each φ_m is verifiable alone against its interface
labels; a module can be replaced by any other map with the same interface without retraining
others; adding a rover changes N, not φ.

---

## 4. Evaluation

* **Per module:** sliding-window K-step open-loop RMSE (K = 20 steps = 1 s) per axis on held-
  out episodes; perception by pose error and calibration of Σ.
* **Composed, prediction:** K-step RMSE of F on formation episodes with N ∈ {2, 3, 4, 8},
  modules trained only at N ≤ 2 — the headline. Baseline: a monolithic team model trained per N.
* **Composed, control:** closed-loop cost Σ ℓ_k and constraint violations with F inside MPC,
  with (a) mocap rover states, (b) the camera path through M_P and the estimator.
* **Data accounting:** total labelled samples per module and per N.

---

## 5. Where history enters — and where it is not obvious

Inside modules (standard, not a contribution): short memory terms in f̂_R and f̂_B for actuator
and aerodynamic lag; the window H_i,k in g_C; the slow estimate β̂_k.

In the composition (design and theory content):

1. **Time alignment.** The camera output reaches the estimator d_c steps late and the command
   applied to rover i is the one sent earlier; the composed system needs delay states even
   though every module is Markov. Ignoring this blames M_R for perception latency.
2. **Estimator and identity.** The composed system is partially observed; the glue between
   M_P and M_R is a filter whose state is a function of the whole interface history, and the
   identity assignment a_k exists only through continuity of past poses. Modules are Markov;
   the composition is a POMDP whose memory lives in the glue.
3. **Context ownership.** ϑ̂_i and β̂_k are sufficient statistics of history computed in one
   module and consumed by others (M_R, the planner's constraint tightening, the camera
   footprint model). Which module owns each statistic, how often it is refreshed, and how its
   uncertainty is passed on are integration decisions, not module decisions.
4. **Mode and event memory** (pushing/herding only): dwell times, hysteresis, stick/slip —
   the composition rule becomes a hybrid system with discrete memory.
5. **Error propagation over the horizon.** The composed K-step error is the propagation of
   module errors through each other along the interaction graph. Define for each module m its
   gain γ_m from interface-error-in to state-error-out over one step; the composed bound is a
   small-gain / ISS-cascade statement over K steps. This is where a certified composition is a
   genuine result, and where the blimp's known swing–translation coupling gives a concrete
   non-trivial cycle in the graph.
6. **Training-time provenance.** Each module's data were generated while the other modules
   were at some earlier version; M_P's test-time error distribution depends on how the
   planner behaved during collection. Keeping provenance is the lifelong-learning form of
   history.

Working position: items 1–2 are engineering that must be done right; 3 and 5 are the
research content on the modelling side; 6 becomes relevant once modules are updated online.

---

## 6. Blimp-specific structure worth stating

With translation frozen, the paper's identified pendulum is

    I_CM θ̈ = −b θ̇ − m g d_VM sin θ + (d_VT − d_VM) f_x(u)

(I_CM = 0.005821 kg·m², b = 0.00098 N·m·s/rad, m = 0.1249 kg, d_VM = 0.0971 m, d_VT = 0.26 m).
Any horizontal thrust therefore excites the swing; the swing in turn tilts the camera and moves
its footprint by ≈ h tan θ. So M_B^swing couples to M_P through geometry and to M_B^trans
through aerodynamics. The known part of the coupling (thrust lever arm) is written into the
grey-box; the unknown part is M_B^couple. This is the cleanest small example of the whole
thesis: two identified modules, one declared coupling, one learned residual, and a measurable
consequence (camera footprint error) of getting the composition wrong.

---

## 7. Assumptions (flagged)

Rovers are identical up to ϑ_i; the floor is flat; the blimp's pose is known from mocap at all
times; formation needs no learned interaction; delays are constant and known; the simulator's
translational blimp parameters are placeholders until the CCTA 2017 / later GT-MAB papers are
incorporated; the camera is treated as rigid with the gondola.
