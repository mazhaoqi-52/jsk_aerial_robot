# Beetle Unified Mode — Improvement Roadmap

> Based on comparative analysis with `develop/assemble_quadrotors` branch.
> Created: 2026-03-09

---

## Philosophy

Current Beetle unified mode is **architecturally more powerful** than Assemble Quadrotors
(N-module, arbitrary topology, dynamic allocation), but can benefit from the latter's
**cleaner control-theoretic structure** in three areas:

1. **Local stability preservation** (not fully stripping follower to bare executor)
2. **Mode-switch as state mapping** (not heuristic patching)
3. **Whole-body multi-variable control** (not 6 independent PID loops)

---

## Priority 1 — Local Damping Retention (借鉴点 A)

### Problem
Currently `sendZeroAttitudeGains()` zeros ALL spinal attitude PID gains, making each
module a pure PWM executor. This maximizes centralized authority but:
- Removes all local disturbance rejection
- Makes system sensitive to communication delay / jitter
- No passive damping to absorb allocation noise

### What Assemble Quadrotors does
Keeps spinal attitude PID with full nonzero gains (`setAttitudeGains()`).
Works because each quadrotor is a standard fully-actuated platform.

### Why we can't directly copy
Beetle's unified allocation directly computes `thrust + gimbal_angle` per module.
If spinal also runs full attitude PID, the two loops conflict — spinal would try to
modify thrust based on attitude error, but the centralized allocator already accounts
for attitude in the wrench distribution.

### Proposed approach: Body-rate damping only

**Instead of full attitude PID, retain only D-term (angular velocity damping).**

```
sendDampingOnlyGains():
    rpy_gain_msg.motors[0].roll_p  = 0    // no position tracking
    rpy_gain_msg.motors[0].roll_i  = 0    // no integral
    rpy_gain_msg.motors[0].roll_d  = D_damping  // ← keep small damping
    rpy_gain_msg.motors[0].pitch_p = 0
    rpy_gain_msg.motors[0].pitch_i = 0
    rpy_gain_msg.motors[0].pitch_d = D_damping  // ← keep small damping
    rpy_gain_msg.motors[0].yaw_d   = D_yaw_damping  // optional
```

This gives spinal a passive damping role:
- `roll_pitch_term_[i] = -angular_vel * D_gain` (pure viscous damping)
- Does NOT conflict with centralized allocation (no position/integral terms)
- Suppresses high-frequency oscillations that 40Hz PC loop can't catch
- Fails gracefully: if centralized command is delayed, local damping prevents divergence

### Implementation plan

1. **Add parameter**: `unified_spinal_damping_d` (YAML, default=0 for backward compat)
2. **New function**: `sendDampingOnlyGains()` alongside existing `sendZeroAttitudeGains()`
3. **In `initUnifiedLeaderMode()`**: call `sendDampingOnlyGains()` instead of `sendZeroAttitudeGains()`
4. **In FOLLOWER enter**: same — `sendDampingOnlyGains()` instead of `sendZeroAttitudeGains()`
5. **Leader allocation adjustment**: account for the fact that spinal adds `-ω*D` to thrust.
   Since D is small and purely reactive, the steady-state allocation is unchanged; only
   transient dynamics change slightly. Can be treated as unmodeled but beneficial.
6. **Tuning**: start with D ≈ 10-20% of independent mode D-gain, increase until oscillation improves.

### Files to modify
- `beetle_controller.h` — add `sendDampingOnlyGains()` declaration, add YAML param
- `beetle_controller.cpp` — implement `sendDampingOnlyGains()`, replace calls
- `BeetleControl.yaml` / `BeetleControl_sim.yaml` — add `unified_spinal_damping_d` param

### Risk assessment: LOW
- Backward compatible (default D=0 = current behavior)
- Pure damping cannot cause instability (only adds energy dissipation)
- No architectural change needed

---

## Priority 2 — Mode Switch as State Mapping (借鉴点 B)

### Problem
Current mode switch uses a collection of heuristic mechanisms:
- `Z_INTEGRAL_FREEZE_FRAMES` (5 frames freeze)
- `Z_KI_BOOST_FRAMES/FACTOR` (60 frames × 2.0)
- `Z_SEED_GAIN` (0.8 × adaptive seed)
- `RP_INTEGRAL_FREEZE_FRAMES/BOOST_FRAMES/BOOST_FACTOR`
- `rp_i_keep_ratio_`
- `pitch_i_seed_default_`

These work well but read as engineering patches, not as a principled method.

### What Assemble Quadrotors does
- Mass transition (gradual model parameter change)
- Thrust ratio blending (`transThrustSumCalc`)
- Z integral state transfer (`setCurrentZErrI`)
- Clean 3-step structure with physical meaning

### Proposed refactoring: Equilibrium-Consistent State Mapping

Reframe the existing mechanisms into a 3-layer structure:

#### Layer 1: Equilibrium Output Remapping
**Physical meaning**: independent hover effort → equivalent unified equilibrium

```
Independent steady-state:
  thrust_z_ss = m*g  (per-module gravity compensation)
  
Unified steady-state:
  thrust_z_ss = M_total*g / N_modules + formation_geometry_offset
  
Mapping:
  ΔI_z = (unified_gravity_share - independent_gravity) / Ki
```

This is essentially what `z_i_seed_default_` already does, but should be computed from
the formation model (mass, CoG offset) rather than hardcoded.

**Concrete change**: compute seed from `unified_controller_->getFormationMass()`,
`unified_controller_->getFormationCogOffset()`, and module count.

#### Layer 2: Integral State Initialization
**Physical meaning**: set integrator initial conditions from Layer 1 mapping

Currently done by `setErrI()` in `initUnifiedLeaderMode()`. No code change needed,
just reframe the narrative:
- Z seed = gravity equilibrium mapping
- Pitch seed = formation CoG offset equilibrium mapping
- RP keep_ratio = partial state inheritance (formation yaw alignment continuity)

#### Layer 3: Transient Conditioning
**Physical meaning**: short-term filter to handle mapping imprecision

Freeze and boost are the transient layer. They compensate for:
- Model error in Layer 1 (seed not exactly right)
- Sensor transients during physical reconfiguration
- Communication latency during mode propagation

**Concrete change**: reduce boost parameters as seed accuracy improves from Layer 1.

### Implementation plan

1. **Compute Z seed from formation model** (replace hardcoded default):
   ```cpp
   double formation_mass = unified_controller_->getFormationMass();
   int n_modules = unified_controller_->getModuleCount();
   double my_gravity_share = (formation_mass * 9.81) / n_modules;
   double independent_gravity = beetle_robot_model_->getMass() * 9.81;
   double z_seed = (my_gravity_share - independent_gravity) / Ki_z;
   ```

2. **Compute pitch seed from CoG offset** (replace hardcoded default):
   ```cpp
   Eigen::Vector3d cog_offset = unified_controller_->getFormationCogOffset();
   // Pitch bias ≈ f(cog_offset.x(), formation geometry)
   double pitch_seed = computePitchEquilibriumBias(cog_offset);
   ```

3. **Keep adaptive seed tracking** as online correction (already implemented).

4. **Document the 3-layer structure** in code comments and paper.

### Files to modify
- `beetle_controller.cpp` — `initUnifiedLeaderMode()` seed computation
- `beetle_unified_controller.cpp/h` — add `getFormationMass()`, `getModuleCount()` if not present
- Code comments throughout mode-switch path

### Risk assessment: MEDIUM
- Seed computation depends on formation model accuracy
- Must verify that model-based seed is close enough to empirical values
- Fallback: keep adaptive seed as safety net

---

## Priority 3 — Whole-Body Formation Model (借鉴点 C partial)

### Problem
Current unified mode dynamically builds a formation allocation matrix but doesn't
expose a unified "formation rigid body model" with:
- Total formation mass
- Formation inertia tensor
- Formation CoG in world frame
- Formation actuation capability map

### What Assemble Quadrotors does
Literally replaces the URDF — `AssembleTiltedRobotModel::assemble()` reloads an
8-rotor model. Clean but inflexible (only 2 hardcoded platforms).

### Proposed approach: Virtual Formation Model

`BeetleUnifiedController` already computes formation geometry. Expose it as a
coherent "virtual robot model" interface:

```cpp
class BeetleUnifiedController {
  // Already have:
  Eigen::MatrixXd formation_allocation_matrix_;
  Eigen::Vector3d formation_cog_offset_;
  
  // Add unified model interface:
  double getFormationMass() const;
  Eigen::Matrix3d getFormationInertia() const;
  Eigen::Vector3d getFormationCoG() const;
  int getModuleCount() const;
  Eigen::MatrixXd getFormationWrenchMatrix() const;  // 6×N
};
```

### Benefits
1. Seed computation (Priority 2) can use real model data
2. Paper presentation: "runtime whole-body model synthesis" as contribution
3. Future LQI/LQR upgrade can use this model directly
4. Cleaner separation between model and controller

### Files to modify
- `beetle_unified_controller.h/cpp` — add model accessors
- Code already computes most of this; just needs clean public interface

### Risk assessment: LOW (mostly API exposure, no algorithm change)

---

## Priority 4 — High-Level Controller Upgrade: PID → LQI/LQR (借鉴点 C full)

### Problem
Current unified 6-DOF control is 6 independent PID loops + pseudo-inverse allocation.
This works but:
- Treats MIMO system as 6 SISO loops
- No cross-coupling compensation
- Less "method-like" for paper

### What Assemble Quadrotors does
Dessemble mode: `HydrusTiltedLQIController` (full-state LQI feedback)
Assemble mode: `FullyActuatedController` (PID + wrench allocation, similar to current Beetle)

### Proposed approach (medium-term)

Replace the 6 PID controllers in unified LEADER mode with a single LQR/LQI:

```
State: x = [pos_err(3), vel(3), rpy_err(3), omega(3), integral(3~6)]
       → 12~18 dimensional state

Input: u = wrench(6) = [fx, fy, fz, τx, τy, τz]

Model: Formation rigid body dynamics (from Priority 3 model)
       ẋ = A*x + B*u

Gain: K = lqr(A, B, Q, R)
      u = -K*x
```

Then allocation: `thrust_per_module = Q_inv * u`

### Implementation sketch

1. Use formation model (Priority 3) to build linearized dynamics A, B
2. Solve Riccati equation offline or use `optimalGain()` pattern from Hydrus
3. Replace `PoseLinearController::controlCore()` path in unified LEADER with LQR feedback
4. Keep PID as fallback / independent mode controller

### Dependencies
- Priority 3 (formation model) should be done first
- Requires understanding of `HydrusLQIController::optimalGain()` pattern

### Risk assessment: HIGH
- Significant algorithmic change
- Need to handle gimbal + thrust decomposition after wrench computation
- LQR linearization assumes small angles (may not hold during assembly)
- Tuning Q, R matrices requires systematic procedure
- **Recommend**: do this only after Priorities 1-3 are validated

---

## Priority 5 — Allocation Blending During Transition

### Problem
Current mode switch is instantaneous: one frame independent, next frame unified.
This can cause thrust discontinuity despite I-term seeding.

### Proposed approach: Ramp-in of centralized authority

```
During transition (e.g., 20 frames = 0.5s):
  α = transition_count / TRANSITION_FRAMES   // 0→1 ramp

  effective_command = (1-α) * independent_hover_cmd + α * unified_cmd
  effective_spinal_gain = (1-α) * independent_gains + α * damping_only_gains
```

This makes the transition physically continuous rather than relying on I-term tricks.

### Interaction with other priorities
- Works naturally with Priority 1 (damping retention): endpoint is damping-only, not zero
- Reduces need for Priority 2 mechanisms (seed/freeze/boost become less critical)
- Could simplify the overall transition logic significantly

### Risk assessment: MEDIUM
- Blending independent + unified commands requires both to be valid simultaneously
- Independent path needs to keep running during ramp (but not publishing to spinal)
- Care needed with gimbal angle blending

---

## Implementation Order

```
Phase I  (immediate, low risk):
  ☑ P1: sendDampingOnlyGains() — body-rate damping retention  ✅ DONE
  ☑ P3: Expose formation model interface in BeetleUnifiedController  ✅ DONE

Phase I-B (damping bug fix + differential damping):
  ☑ B1: Fix sendDampingOnlyGains() per-motor path (bypass zero torque_alloc_inv)  ✅ DONE
  ☑ B2: Fix sendZeroAttitudeGains() same per-motor path  ✅ DONE
  ☑ B3: Discover uniform D gains are physically ineffective (only changes total thrust,
         no restoring torque — because spinal sees gimbal_dof_=0, all motors get
         same correction)  ✅ ROOT CAUSE FOUND
  ☑ B4: Implement DIFFERENTIAL per-motor D gains based on X-quad geometry:  ✅ DONE
         pitch_sign[4] = {-1, +1, +1, -1}  (=-sign(x_i))
         roll_sign[4]  = {+1, +1, -1, -1}  (+sign(y_i))
         Now front/rear motors get opposite signs → real pitch torque damping
  ☐ B5: Simulation test — pitch-only (roll_d=0, pitch_d=3.0, yaw_d=0)
  ☐ B6: If pitch improves, add roll differential damping
  ☐ B7: Tune D magnitude via AB test (1.0 / 2.0 / 3.0 / 5.0)

Phase II (short-term, medium risk):
  ☐ P2: Model-based seed computation (replace hardcoded defaults)
  ☐ P2: Refactor initUnifiedLeaderMode() with 3-layer narrative

Phase III (medium-term, high risk — after Phase I+II validated):
  ☐ P5: Allocation blending during transition
  ☐ P4: LQI/LQR for unified LEADER (research prototype)
```

---

## Metrics for Validation

For each improvement, measure in simulation:

1. **Transition smoothness**: max altitude drop/spike during mode switch [m]
2. **Steady-state tracking**: RMS position error in unified hover [m]
3. **Disturbance rejection**: response to step wind (settling time, overshoot)
4. **Communication robustness**: behavior with artificial 50ms, 100ms command delay
5. **Fallback safety**: time from leader failure to stable independent hover [s]

---

## References

- Assemble Quadrotors branch: `develop/assemble_quadrotors`
  - `robots/assemble_quadrotors/src/control/assemble_controller.cpp`
  - `aerial_robot_control/src/control/fully_actuated_controller.cpp`
  - `robots/assemble_quadrotors/src/model/assemble_robot_model.cpp`
- Hydrus LQI: `robots/hydrus/src/hydrus_tilted_lqi_controller.cpp`
- Spinal attitude control: `aerial_robot_nerve/spinal/.../attitude_control.cpp`
