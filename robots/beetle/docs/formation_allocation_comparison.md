# Formation QP / pseudoinverse comparison

## Scope and manual workflow

The existing workflow remains manual: start `n_module_test.launch`, start
`assembly_motion.launch` (`real_machine:=false` for Gazebo), then invoke the
existing switch script. Pushing, towing and valve retain their existing launch
files and state machines. This change does not launch, assemble, contact,
reset, or run any experiment.

```bash
# Original invocation: restore the allocation settings read at bringup.
rosrun beetle set_unified_mode.py true

# Select the allocator on every discovered module, or supply ALL formation IDs.
rosrun beetle set_unified_mode.py true --allocation qp
rosrun beetle set_unified_mode.py true --ids 2,3,1 --allocation pinv

# Original exit operation.
rosrun beetle set_unified_mode.py false
```

The optional argument selects only the allocator. Both choices keep
`unified_control_mode=true`. No yaw flag, PID gain, observer, task weight,
physical model, or final actuator handling is changed. In particular, if
`yaw_in_allocation=false`, flight yaw continues through the existing spinal
bypass; it must not be described as part of the allocation target.

The script stages `/beetleN/controller/unified_allocation_method` on all selected
modules before calling their existing `std_srvs/SetBool` services. The service
reads and applies that parameter to the in-memory allocator, validates the
value, and acknowledges it. `rosparam set` alone does not change a running
allocator. The internal `configured` value restores the original bringup
selection, including its regularized non-QP behavior when configured that way.
No-argument script calls always stage `configured`, so a previous explicit pinv
selection does not silently change the original invocation's meaning.

All selected services are checked before staging. Explicit `--ids` selections
are also checked against discovered peers with the same navigator-published
`assembly_leader_id`. This cannot detect a module whose service is absent or
whose topology metadata is unavailable; the operator must include every member.
An explicit selection requires an acknowledgement from the updated controller.
Failures return nonzero and are reported per module, with no automatic rollback,
landing, restart, or false/true switching. Calls across processes are not atomic.

## Shared allocator and output contract

Every assembled module still runs the existing full formation controller and
takes its own actuator block. Both methods use the same model, state inputs,
coordinate snapshot, flight target, gravity feedforward, full formation task
wrench and residual feedback. Neither consumes the LF `est_wrench_task` shares
or inserts task feedforward through local PID persistent terms.

Let `A = integrated_map_`, `b = b_control + b_task + b_feedback`, and
`H = diag(M I3, I_C)`, where `I_C = R_CB I_body R_CB^T` uses the allocation's
frozen virtual-CoG rotation. Then:

```
requested = H * b
G_F       = H * A
nominal   = G_F * z
pinv: z   = A^+ * b
```

Explicit pinv directly reuses the existing SVD inverse (absolute singular-value
cutoff `1e-4`), bypassing damping and secondary references even if `alloc_lambda`
is positive. It does not execute QP constraints, priorities or secondary costs.
This is the pseudoinverse of the acceleration-space matrix A. For rank-deficient
problems, `A^+ b` and `G_F^+ requested` have different residual metrics.

QP keeps its existing objective, weights, constraints and retry scales
`0.7, 0.4, 0.0`. Only task and residual-feedback demands/weights are scaled by
that retry; flight control stays unchanged. The raw request is captured before
any retry. The adopted target describes the successful solve, not achieved
wrench. The primary comparison is ALWAYS requested versus nominal. Adopted is
an explanatory curve, not a substitute for evaluating the original request.

The existing nominal calculation function is shared by both solvers and the
observer. Its result is frozen into the same diagnostic snapshot as the targets.
All three wrench publications below originate at the existing leader assembly
diagnostic entry point; never mix a follower target with this leader output.

### Required bag data

`L` is the actual C++ formation leader (not necessarily the end-effector host).
Rates below are nominal; actual rates depend on valid inputs and successful
control cycles. No new custom ROS message is introduced.

| Data / topic | Type | Units, axes, reference point | Publication |
|---|---|---|---|
| **Original complete allocation target** `/assemble/debug/formation_wrench_requested` (new) | `geometry_msgs/WrenchStamped` | N, N m; frozen virtual CoG axes; moment about mass-weighted formation CoG; `frame_id=assembly_cog` | Leader, each valid composed request, nominal 40 Hz; also on subsequent solve failure |
| **Nominal allocation result** `/assemble/debug/formation_wrench` (reused) | `geometry_msgs/WrenchStamped` | Same units, axes and moment origin as requested; `G_F z` | Leader, successful QP AND pinv cycles, nominal 40 Hz |
| **Predicted physical-interface loads** `/beetleL/unified_control/interface_load` (reused; also available per module) | `std_msgs/Float32MultiArray` | N, N m; physical formation-body axes; adjacent module-CoG midpoint; sign and layout below | Valid model and successful allocation with subscribers, nominal 40 Hz for BOTH methods |
| Adopted target `/assemble/debug/formation_wrench_adopted` (new) | `geometry_msgs/WrenchStamped` | Same contract as requested | Leader, successful cycles, nominal 40 Hz; equals requested in pinv |
| Method / validity / scaling `/assemble/debug/allocation_status` (new) | `diagnostic_msgs/DiagnosticArray` | State fields; component ratings N / N m; same allocation stamp and source module ID | Leader, every allocation attempt reaching assembly diagnostics, nominal 40 Hz |

The targets and nominal output share the SAME `header.stamp`, module and matrix
snapshot. There are no nominal/adopted samples on failure. Status identifies
`command_updated=false` rather than relabeling an old command as a new result.
Invalid input before target construction has `requested_valid=false` and no
fabricated zero target. Requested includes everything entering the current
allocator, not spinal-side attitude increments or bypass yaw.

Nominal means allocator output, after QP's internal constraints when selected,
BEFORE spinal P/D, bypass yaw, saturation scaling, gimbal filtering and tracking
error. It is neither measured wrench nor a reconstruction of the heterogeneous
local blocks ultimately dispatched by all modules. The existing topic is kept;
status names its `nominal_stage` explicitly. No measured overall propulsion or
docking-interface wrench is introduced by this change.

Status includes source `module_id`, `allocation_sequence`, resolved
`allocation_method` (`qp`, `pinv`, or legacy `regularized`), `requested_valid`,
`command_updated`, `adopted_valid`, successful `task_feedback_scale`, unchanged
`yaw_in_allocation`, interface-model validity, constraint enforcement, whether
an interface array was published, its source topic, and the retained force and
torque component ratings. Existing heartbeat diagnostics remain available for
peer/control liveness; they are too slow and lack per-solve scaling to replace
this small status output. With one module there is no interface to predict.

### Interface prediction and alignment

The existing common predictor is unchanged:

```
w_interface,k = d_k - G_k * z
```

It is evaluated using the current solver's selected `z`, not a parallel QP or
counterfactual pinv prediction. Pinv skips enforcement but preserves and records
the same `alloc_interface_force_limit` / `alloc_interface_torque_limit`.
Positive limits are symmetric per-component bounds; nonpositive values disable
that half of the bound and are not zero physical ratings. Force/torque norms
must not be compared as if these component ratings were norm bounds.

The flat `.data` layout remains wire compatible, with ten float32 values per
interface:

```
[left_id, right_id, Fx, Fy, Fz, Tx, Ty, Tz, norm(F), norm(T)]
```

Interfaces are ordered by physical module X position, negative to positive,
irrespective of numeric IDs or allocation-column order. The wrench is exerted by
the positive-X side on the negative-X subchain. Its moment origin is the adjacent
module-CoG midpoint, the existing docking-plane proxy. `layout.dim` now labels
the values as `predicted`, names the physical axes/sign/reference and fields,
and describes an interfaces-by-ten array. Existing `.data` consumers still work.

The predictor assumes task contact on the positive-X terminal, outside the
negative-X free body. `d_k` uses the existing commanded rigid-body specific
acceleration, including gravity; angular-rate terms and other contacts on that
subchain are omitted. The prediction is not a load measurement. A failed model
is reported as invalid and publishes no zero-filled array. A QP requiring that
model retains its failure behavior; pinv can still allocate with invalid-load
status. This change does not replace the geometric proxy or physical ratings.

The existing array has NO Header. For 40 Hz plotting, keep it and use its bag
record timestamp, the status record timestamp, `interface_load_published`, and
the exact `interface_load_topic` named by leader status. It is published during
that allocation, before the leader's status/curves. Associate the same source's
nearby array in a single control-cycle window, preserve message order, and reject
ambiguous matches, gaps, invalid models and method-transition intervals. Do not
zip independent topic arrays by sample index or combine different modules.
The stamped curves/status align exactly; interface alignment remains approximate
because transport/solve latency and queue loss cannot be reconstructed from this
headerless type. Strict loss-tolerant per-cycle matching would require a future
stamped interface message; no duplicate load topic is added here.

### Reused motion and task data

| Topic | Type / quantities | Axes and point | Nominal rate |
|---|---|---|---|
| `/assemble/debug/pose/pid` | `aerial_robot_msgs/PoseControlPid`; position/velocity references and errors (m, m/s), control-attitude errors (rad) | Formation CoG position in world; virtual control attitude | 40 Hz |
| `/beetleL/unified_control/reference` | Existing `beetle/UnifiedControlReference`; references, mass, inertia, frozen formation offset | World position/velocity; separate virtual attitude and `leader_final_target_baselink_rpy`; offset in physical body axes | Successful cycles, 40 Hz |
| `/assemble/cog/odom` | `nav_msgs/Odometry`; position and virtual-CoG quaternion | World; mass-weighted module CoGs when masses ready, otherwise equal-weight fallback | 40 Hz; pose only, unused twist is not zero-velocity truth |
| `/beetleN/uav/baselink/odom`, `/beetleN/uav/cog/odom` | `nav_msgs/Odometry`; pose, m/s, rad/s | World pose/linear velocity; angular velocity in the corresponding body/CoG axes | Default 100 Hz |
| `/beetleN/ground_truth` | `nav_msgs/Odometry`; existing noisy Gazebo-derived state | Baselink origin; world pose/linear velocity, fc angular velocity; header frame IDs currently empty | About 100 Hz |
| `/gazebo/link_states` | `gazebo_msgs/LinkStates`; noiseless link pose and twist | World axes; named link origin; bag time (no Header) | Gazebo-dependent |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage`; m and quaternion | Recorded parent/child frames, including root-to-CoG geometry | Dynamic / static |
| `/wall/odom`, `/load/odom` | `nav_msgs/Odometry`; object pose (m, quaternion) | World; wall_panel / load_box origin | 50 Hz |
| `/valve/odom` | `nav_msgs/Odometry`; handle pose | World; handle origin | 10 Hz |
| `/beetleN/unified_control/module_model` | Existing `beetle/ModuleModel`; kg, kg m², m | Physical body model; use with TF for weighted CoG | Latched/model events |

Use `link_states` world twist for object velocity if the installed p3d plugin's
`localTwist` convention is not independently confirmed. Project along the task
direction; unwrap successive valve-axis angles across revolutions and project
angular velocity onto that same axis. `valve.urdf` has a continuous joint;
`valve_downward.urdf` currently has a fixed joint for the static torque task.
Do not change the model to manufacture rotation results.

Reconstruct the controller's formation origin as leader CoG position plus
physical body rotation times the reference's locked `formation_cog_offset`.
Use the same masses/CoG definition for Gazebo-derived evaluation. Physical body
attitude and its reference are distinct from the near-level virtual CoG frame.
No truth signal is added to control feedback. Existing task velocity feedback
and adaptive logic remain as they were.

Existing pushing/towing diagnostic strings, static-valve `~phase`, flight state
and `/rosout` remain the task/contact markers. Existing wall/load/valve joint
wrench plugins describe object/support reactions, not overall propulsion or
docking-interface truth. Historical bag names such as observer
`realized_wrench_input` still mean nominal model input; current code names it
`allocated_wrench_input`.

For a new bag, append the five required comparison entries above (using the
actual leader interface topic), `/beetleL/unified_control/reference`, model/TF
data, and the selected task's existing motion/marker topics to the existing
manual recording command. `rosbag_control_data.sh` already forwards extra topic
arguments; no recording/experiment orchestrator is needed. Old bags cannot
retroactively contain the newly published full targets or per-cycle status.

## Actuator and task acceptance

Both methods retain the same `buildModuleThrustCommand` → spinal → Gazebo path,
including local attitude increments, vectoring saturation, PWM handling, servo
filtering and physical limits. QP additionally enforces its existing solve-time
actuator/interface/priority bounds. Pinv may exceed those bounds; raw nominal
wrench does not include the resulting downstream losses. PC has no new common
clipping step; QP's 75-degree bound is not the URDF's approximately 90-degree
physical joint limit. No weights or lower-level limits are changed to hide this.

Static and mocked-interface acceptance covers:

- Both selections stay unified and share full task wrench/weight routes.
- No allocator switch uses a false/true cycle, changes yaw, or reruns PID mode
  initialization while unified was already true.
- Pushing, towing and valve files/launches stay unchanged, including contact
  decisions, speed feedback, adaptation, compensation, unload and exit paths.
- Both solvers reach the same nominal-output calculation and current-result
  interface predictor. Invalid predictions cannot masquerade as zero load.
- CLI staging/acknowledgement, pure pinv without damping, interface field/sign
  behavior, retained ratings and mass/inertia conversion have offline checks.

These checks do not establish full runtime task compatibility. Operator-run
validation after building, not performed automatically by this change:

1. Use the original bringup/assembly steps, then manually select qp or pinv on
   all modules. Confirm service acknowledgements and leader status agree.
2. Independently run each original pushing, towing and dynamic-valve task launch
   when desired. Verify original contact, velocity/adaptation, compensation,
   unload and exit behavior, with no unintended LF routing or mode exit.
3. Inspect the manually recorded bag: requested/nominal/adopted share source,
   stamps and units; both methods provide valid interface arrays when available;
   limits remain identical; overload stays visible in pinv; invalid intervals
   and QP retries are distinguishable. Compare original requested to nominal.
4. Verify actual rates, queue loss and the headerless interface alignment window
   before interpreting time-local load peaks. No comparative outcomes are
   assumed or generated by the implementation.

Offline checks can be built/run separately from the robot. The existing C++
test target requires a test ROS master; use an isolated port, not a live robot's
master. `python3 robots/beetle/test/test_set_unified_mode.py -v` uses mocks and
does not connect to ROS or instantiate task state machines.

### Implementation verification (2026-10-28)

- Both modified C++ translation units compile, and the controller shared library
  and allocation-test binary link against the existing workspace dependencies.
  Verification artifacts were written under `/tmp`, without installing into the
  workspace's build/devel products or replacing a running controller.
- The eight Python CLI/shared-interface checks pass without ROS connections.
- Seventeen C++ tests ran against a temporary isolated test master: sixteen pass,
  including all five added tests. The existing
  `SecondaryReferencePreservesDesiredWrenchWhenCogIsTilted` test fails with
  residual `9.80112` in both assertions. Recompiling the saved pre-change source
  and its original header/tests reproduces the identical failure (eleven of
  twelve baseline tests pass). The existing QP secondary-reference behavior was
  preserved; fixing it would be a separate change to the reviewed QP baseline.
- Python syntax and `git diff --check` pass. No Gazebo, assembly, task launch,
  real control-mode service call, or experiment comparison was performed.
- Actual bag publication rates, interface time alignment and all three complete
  task state machines remain subject to the manual validation above.
