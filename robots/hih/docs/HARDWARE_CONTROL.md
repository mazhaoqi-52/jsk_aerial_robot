# HiH hardware control update

This update addresses the code issues in `hih_hardware_review.md`. It does not
validate flight gains, payload friction, sensor calibration or hardware capacity.
The original Beetle code, existing PID/MOB gains and user README are preserved.

## Follower reference and transport

Follower still responds to its own contact wrench. No Leader trajectory or pose
is copied to Follower, and Follower can run alone. The contact plate remains at
base_link (-0.26,0,0) m with outward normal [-1,0,0]. Positive compression is +Fx
of the environmental reaction. The normal-force target remains **+10 N**.

`moving_reference: true` splits the Admittance displacement into an accumulated
internal reference and a small local correction. When the local correction
reaches half of `max_offset`, it transfers to the reference without changing the
sum, velocity, force reference or final command. This keeps correction capacity
available for continued translation and yaw. No navigator target is overwritten.
The original contact-center compensation uses the **total** yaw displacement,
so transferring a correction does not move the commanded contact center.

The virtual spring must be zero in this mode; configuration rejects nonzero
stiffness. For a fixed-reference spring experiment, disable `moving_reference`.
Existing mass/damping, PID/MOB and weak CoP gains are unchanged.

`max_travel: [1.0,1.0,0.5,2.0]` bounds the total accumulated contact-reference
motion, in world X/Y/Z metres and yaw radians. Reaching a bound rejects that
step and latches a position HOLD instead of silently continuing to saturate.
These are per-axis bounds relative to the navigator task reference, not a global
workspace fence. Manual nominal motion and contact-center compensation are not
counted as Admittance travel. Repeated trajectory 2 executions can reach a bound.

## Contact protection

| Condition | Action |
|---|---|
| Before first compression >= 0.8 N | Wait; no force ramp or autonomous contact search |
| Normal contact established | Admittance and the existing 1 N/s force ramp run |
| Control interval invalid or >0.1 s | Preserve displacement, internal reference and force target; zero correction velocity and hold last command for that cycle |
| Contact force drops below 0.4 N | Immediately pause motion and force ramp; retain last commanded position/yaw |
| Contact loss lasts >=0.2 s | Latch `contact_lost_hold`; no automatic reapproach |
| MOB sample stale/invalid | Immediately hold; latch `mob_timeout_hold` after 0.5 s of stale input |
| Fresh measured absolute normal force >=20 N | Immediately latch `normal_force_limit_hold`, even before initial contact |
| Accumulated travel limit / invalid Admittance input | Latch `admittance_travel_or_input_limit_hold` |

A brief contact/data interruption can resume if contact is restored before its
fault timeout. A latched fault never clears merely because force returns.
Landing/disarming or leaving independent position control resets contact state,
reference and corrections. Resolve the cause before a new takeoff/experiment.

During HOLD, the position/yaw command is the last Admittance-shaped reference;
velocity, acceleration and angular feedforward are zero. CoP integration is
paused, retaining its existing trim. This is position hold, not a promise of
continued normal-force tracking or payload support after contact is lost. An
overload fault detects the estimated force and stops progression; it cannot
physically cap force imposed by another drone or a person.

The ground-station keyboard subscribes to Follower's latched fault flag. On
receipt it zeros both keyboard motions, cancels approach/trajectory, and blocks
new movement/trajectory triggers while the fault is present. Flight commands
including land/force landing/halt remain available. There is no automatic motor
stop or peer landing. Local Follower protection operates without the keyboard;
ground-station propagation requires the ROS connection. The existing navigation
0.5 s command timeout remains the backstop for lost keyboard commands.

## Keyboard and trajectory settings

`g` retains its simple toggle: both robots' local -X at **0.02 m/s**, then zero
keyboard velocity on the next press. Space, manual input and flight commands
cancel approach as before. Stopping approach does not disable normal-force
control. There is no automatic both-contact confirmation or lifting sequence.

| Key | Leader-only WORLD path | Default total period | Peak speed |
|---|---|---:|---:|
| 1 | -X, +Z, +X, -Z | 40 s | 0.0375 m/s |
| 2 | +yaw, +Y, -yaw, +X | 120 s | 0.0125 m/s; 0.0982 rad/s yaw |

Per-leg translation remains 0.20 m and each yaw leg remains 90 degrees. Each
trigger executes once. Trajectory 2 ends at +0.20 m world X and Y, not its start.
`trajectory_max_speed: 0.05` and `trajectory_max_yaw_rate: 0.12` validate the
quintic peak speeds before starting the node. A shorter period exceeding either
limit is rejected with an instruction to increase the period. Keep these limits
consistent with the configured Follower velocity/force-response capacity.

## Experiments

### Follower alone

Run Follower bringup and keyboard as before. Admittance is enabled in the provided
YAML. Establish compression at the -X plate first: the environmental reaction is
local +X, at least 0.8 N. Then test small XYZ/yaw disturbances while retaining
normal contact. Arbitrary tangential pushes before normal contact do not start
constant-normal-force control. The target is the normal component, not the
magnitude of an arbitrary 3D force; sustained normal force needs a physical
opposing contact. Moving the contact by hand is now possible beyond the old
fixed 0.15/0.10 m correction range, within `max_travel`.

### Two modules

Use the same updated package/configuration on both robot PCs and ground station.
Confirm both are in HOVER / independent position control. Face the two local -X
plates toward the cargo, use g to approach, and g/Space to stop keyboard motion.
Confirm force/reference/status before triggering trajectory 1 or 2. Follower
retains force-based autonomy; g does not check Leader contact or readiness.
Assembly/unified control remains a different mode and disables HiH Admittance.

### CoP yaw, then yaw/Z

CoP master remains OFF by default. Load/gain configuration is still in
`hih_control.yaml`. Runtime flags act on the active controller paths:

```bash
rosparam set /hih2/controller/cop_adjustment/enabled false
rosparam set /hih2/controller/cop_adjustment/yaw/enabled true
rosparam set /hih2/controller/cop_adjustment/z/enabled false
rosparam set /hih2/controller/cop_adjustment/enabled true
# After the yaw-only comparison, add Z:
rosparam set /hih2/controller/cop_adjustment/z/enabled true
```

Gains and the +/-0.05 m safe CoP region are unchanged. CoP integrates only during
healthy active Admittance; fault HOLD and stale data cannot wind it up.

## Configuration and logs

Controller settings load at bringup; restart bringup after YAML changes. Only
the three CoP enabled flags have the existing runtime update behavior.

`/hih2/hih/admittance/` topics include:

- `fault` (Bool): latched local fault, also consumed by the keyboard.
- `status`: waiting/active/held/fault reason.
- `position_offset`: small local XYZ correction.
- `reference_offset`: accumulated internal world XYZ reference displacement.
- `nominal_position`: original navigator task position, unchanged by this update.
- `command_position`: completed Admittance position before CoP trim.
- `yaw_reference`: x=task yaw plus internal reference; y=Admittance command yaw;
  z=total Admittance yaw displacement.
- Existing force measured/reference, correction velocity and CoP topics remain.

Record all published topics with `rosbag record -a -O hih_experiment`. Check
`fault=false`, `status=active`, valid force/CoP and reference travel while testing.
`applying=false` during a HOLD does not mean the stored position offset was erased.

## Build and verification

```bash
hw
cd ~/ros/jsk_aerial_robot_ws
catkin build hih -j2 -p1
hw
```

Verification includes 35 keyboard tests, 26 Admittance/contact/guard numerical
tests, 33 CoP numerical tests, 5 CoP ROS tests and 6 tests invoking the actual
Follower reference modifier: **105 tests**. A separate real-keyboard test on an
isolated localhost ROS master confirmed fault reception, both zero commands,
blocked retries and normal Ctrl+C termination. It used test namespaces only.
No flight, motor actuation or physical contact experiment was performed.
