# z: unload Follower contact force, then hover

This command is intended for the **Follower-only hand-contact experiment**.
It is not a coordinated payload landing/release procedure. With a payload
between two robots, removing Follower compression can drop the payload.

## Use

1. Build the updated package on the Follower PC and ground station. Restart
   bringup and the keyboard while landed. This requires both controller and
   keyboard updates; copying `hih_key.py` alone is insufficient.
2. During healthy independent HOVER/position control, press **z**.
3. The keyboard cancels approach/trajectory and zeros both keyboard motions.
   Only Follower receives `/hih2/hih/admittance/release` (`std_msgs/Empty`).
4. Maintain contact while allowing the pressure to decrease. The controller
   unloads at the configured rate; pressing z is a request, not completion.
5. Wait for `/hih2/hih/admittance/status` to report **released_hover**, and verify
   low measured contact force and stable vehicle motion before withdrawing.

After completion, ordinary keyboard hover movement works. Admittance and CoP
reference shaping remain off for the rest of that flight. Pressing z again does
not restart force control. Landing/disarming/reset restores normal startup.

Space still zeros keyboard velocity. It does not cancel the onboard unloading
procedure. Land/force landing/halt retain priority. Keyboard exit or disconnect
does not cancel an already accepted onboard release.

## What changes onboard

- Freeze the navigator task pose and zero its commanded motion during unloading.
- Clear old inward Admittance filter input/velocity without erasing displacement.
- Reduce the normal-force reference toward zero. It never rises above its
  previous value or current measured compression during release; a retreating
  hand is not chased with a larger force target.
- Continue Admittance unloading while freezing CoP adaptation and retaining trim.
- Expected contact loss during this intentional release does not trigger the
  normal contact-loss fault. Sensor validity, timing, overload, accumulated travel
  and additional release-motion limits remain enforced.
- Finish only after measured full wrench, actual motion and shaped command
  motion all meet their tolerances continuously for the dwell time.
- Adopt the final full commanded pose (including CoP trim) into the navigator
  atomically, synchronizing its target bookkeeping. Then clear the shaping state.
  This avoids jumping back to the old nominal position.
- Preserve existing PID integral states and gravity compensation. Blindly
  clearing them at handoff would change the actuator command even with an
  unchanged reference. The completion conditions first verify low measured load
  and small motion; the position/yaw reference is continuous across handoff.

`released_hover` means the software completion checks passed and force control
is latched off. It is not a guarantee of zero real force or flight safety;
MOB bias, hardware calibration and physical stability still need validation.

## Parameters

In `config/hih_control.yaml`, under `follower/controller/admittance/release`:

| Parameter | Default | Meaning |
|---|---:|---|
| `force_rate` | 1.0 | N/s, commanded unloading rate |
| `normal_tolerance` | 0.3 | N, absolute normal force completion threshold |
| `force_tolerance` | 0.5 | N, full contact force norm threshold |
| `torque_tolerance` | 0.1 | Nm, full contact torque norm threshold |
| `speed_tolerance` | 0.05 | m/s, estimated vehicle speed threshold |
| `omega_tolerance` | 0.05 | rad/s, estimated vehicle angular speed threshold |
| `command_speed_tolerance` | 0.005 | m/s, shaped reference speed threshold |
| `command_omega_tolerance` | 0.01 | rad/s, shaped reference angular speed threshold |
| `settle_time` | 1.0 | s, continuous completion-condition dwell |
| `timeout` | 30.0 | s, overall unloading timeout |
| `data_timeout` | 0.5 | s, continuous invalid data/control-gap allowance |
| `max_distance` | 0.15 | m, command displacement from z request |
| `max_yaw` | 0.15 | rad, command yaw change from z request |

These are starting software limits, not validated hardware tolerances. Settings
load at bringup. Existing PID/MOB, Admittance gains and operator contact thresholds
are preserved. The key mapping is `keys/common/release_force: z` in `hih_key.yaml`.

## Failure states

`releasing` is progress. `release_handoff_pending` is internal transition progress.
Only `released_hover` announces completion. The keyboard requires an unloading
acknowledgement before accepting completion, blocks new motion while waiting,
and warns if no updated Follower release subscriber exists.

`release_*_hold` indicates a failure, including invalid data, overload, timeout,
motion bounds, changed flight mode or an existing fault. The last valid reference
is held when independent position control remains available. Failure is never
reported as successful unloading and repeated z does not clear it. Resolve the
cause and use the established landing/reset procedure; do not withdraw solely
because z was pressed. Other flight modes and landing retain their own control.

## Build and inspect

```bash
hw
cd ~/ros/jsk_aerial_robot_ws
catkin build hih -j2 -p1
source devel/setup.bash
# Restart Follower bringup and keyboard while landed, then monitor:
rostopic echo /hih2/hih/admittance/status
```

Record `normal_force_reference`, `normal_force_measured`, `command_position`,
`nominal_position`, `yaw_reference`, `fault`, existing CoP diagnostics and motion
estimates together. `rosbag record -a` includes the release request and statuses.
Do not replay command bags into a ROS master connected to a flying robot.

## Verification

41 keyboard tests, 34 Admittance/contact/guard/release numerical tests, 33 CoP
numerical tests, 12 actual Follower reference-hook ROS tests and 5 CoP ROS tests
passed: **125 tests**. They cover ramp-down, no automatic reactivation, stale data,
overload, timeout, bounds, completion dwell and exact full-pose handoff with PID
integral preservation. A separate real-keyboard test used an isolated localhost
master and test namespaces to verify request/acknowledgement/completion handling.
No flight, motor actuation or physical force release was performed.

## Integrated flight keys

Friend branch flight keys are preserved: r=arm, t=takeoff, l=land, f=force landing, h=halt.
**h stops motors; z unloads Follower contact force.** Follower Z uses [ / ], and Leader -Y uses comma.
