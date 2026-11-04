# Integrated HiH update — 2026-11-05

Source: https://github.com/mazhaoqi-52/jsk_aerial_robot/tree/develop/HiH_init
Base commit: `4752c2cead83cc0a6891b817bb844f6a37a6be8a`. All five new commits were fast-forwarded into the existing
local `develop/hih_init` branch (lowercase h); the source branch uses uppercase H.
`origin` still points to the user's fork. `mazhaoqi` fetches only develop/HiH_init.
New force-release changes remain uncommitted and unpushed.

Preserved friend changes: original Beetle flight keys, base bringup admittance
option, and guard against reading an unavailable IMU timestamp at startup.
Preserved local additions: force ramp-down, completion checks, continuous hover
handoff, no automatic force reactivation, moving reference and contact protection.
User PID/MOB settings, contact thresholds, CoP code and README are preserved.

## Final keys

| Key | Action |
|---|---|
| r | Arm/start |
| t | Takeoff |
| l | Land |
| f | Force landing |
| h | HALT / stop motors |
| z | Unload Follower contact force, then hover |
| g | Toggle both robots' local -X approach |
| Space | Zero keyboard velocity |
| [ / ] | Follower +Z / -Z |
| comma | Leader -Y |

**Do not use the earlier h-release key instructions with this version. h is halt;
z is force release.** See `FORCE_RELEASE.md` for release conditions and parameters.
A z press is a request, not completion. Maintain contact while unloading and verify
`released_hover` and stable actual motion before withdrawing. Payload release
between two drones is not coordinated by this command.

## Deploy while landed

Update the Follower PC and ground station together, rebuild, re-source and restart
bringup and keyboard. The whole updated hih package is included in the ZIP. It
expects the existing HiH reference-modifier hook in the surrounding repository.

```bash
cd ~/ros/jsk_aerial_robot_ws
catkin build hih -j2 -p1
source devel/setup.bash
# Single Follower using the friend's bringup option:
roslaunch hih bringup.launch simulation:=false real_machine:=true robot_id:=2 battery:=true admittance:=true
# Separate ground-station terminal:
rosrun hih hih_key.py
```

The default keyboard Follower namespace is /hih2. Use a matching robot_id or adjust
its configured namespace. The dedicated hih_follower_bringup.launch also selects
the Follower role. The admittance CLI option belongs to the base bringup.launch.

## Verification and backups

125 tests passed: 41 keyboard, 34 Admittance/contact/guard/release numerical,
33 CoP numerical, 12 actual Follower reference-hook ROS, 5 CoP ROS. The startup
regression invokes the observer before an IMU timestamp exists. Four launch
parameter configurations passed without launching hardware nodes. A real
keyboard test on an isolated ROS master confirmed z release and h halt are
separate; only test namespaces were used. No flight or physical unloading tested.

Backup branch: `backup/hih-before-mazhaoqi-20261105` at `54f375de7f09273d3148c3e8ef197d4a01a51883`.
Local changes backup stash: `734baa5c3fd16032d7b543c44e18b7afd49409fa` (retained).
Additional worktree ZIP and binary patch are in work/friend_integration.
