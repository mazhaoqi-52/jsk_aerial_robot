# Hand in Hand — HiH

ROS 1 package for transporting a payload through contact between two Beetle-based aerial robots. Developed on Ubuntu 22.04 with ROS-O (`ROS_DISTRO=one`).

- **Leader:** position control using the inherited PID controller and momentum observer (MOB).
- **Follower:** the same PID/MOB with optional **X, Y, Z, and yaw admittance** corrections for contact-force regulation.
- **Ground station:** shared ROS master, keyboard commands, and rosbag recording.

The original `robots/beetle` package is preserved. HiH uses separate package, message, library, and plugin names. The initial PID/MOB gains match the existing Beetle gains; role-specific settings are in `config/hih_control.yaml`.

## Build

Use the same repository revision on both robot PCs and the ground station.

```bash
source /opt/ros/one/setup.bash
cd ~/ros/jsk_aerial_robot_ws
catkin build hih -j2 -p2
source devel/setup.bash
```

With the configured shell helpers, `hm hih` builds the package and `hw` loads the workspace. Use `hsh` first if the current terminal contains a different ROS environment.

## Run on separate computers

Each robot has its own PC and serial connection. Both may use `/dev/flight_controller` locally. The default namespaces are `/hih1` for Leader and `/hih2` for Follower.

In **every terminal**, load the workspace and configure the network. Replace these example IP addresses with your actual addresses:

```bash
source /opt/ros/one/setup.bash
source ~/ros/jsk_aerial_robot_ws/devel/setup.bash
unset ROS_HOSTNAME
export ROS_MASTER_URI=http://192.168.10.10:11311
export ROS_IP=<THIS_COMPUTER_IP>
```

| Computer | Example `ROS_IP` |
|---|---|
| Ground station | `192.168.10.10` |
| Leader PC | `192.168.10.11` |
| Follower PC | `192.168.10.12` |

All computers must be able to reach one another's ROS node addresses and ports. SSH connectivity alone is insufficient; this also applies when the ground station runs inside WSL. Set these variables **after** opening a clean shell such as `hsh`.

**Terminal 1 — ground station:**

```bash
roscore
```

**Terminal 2 — Leader PC, via SSH:**

```bash
roslaunch hih hih_leader_bringup.launch \
  simulation:=false real_machine:=true robot_id:=1 battery:=true
```

**Terminal 3 — Follower PC, via SSH:**

```bash
roslaunch hih hih_follower_bringup.launch \
  simulation:=false real_machine:=true robot_id:=2 battery:=true
```

Select the model that matches the actual hardware using `hih_model:=...` (for example, `hih_model:=hih_fang` when appropriate). Check the existing serial, sensor, and mocap settings for each robot.

**Terminal 4 — ground station keyboard:**

```bash
rosrun hih hih_key.py
```

The keyboard works with **either robot alone or both robots connected**. It does not wait for the absent robot. Common flight commands go to both configured namespaces; movement commands go only to the selected role. Commands sent before a subscriber connects are not queued for later execution.

## Keyboard mapping

```text
---------------------------

v:  arming motor (please do before takeoff)
b:  takeoff
g:  approach ON/OFF
n:  land
m:  force landing
/:  halt (force stop motor)
Space:  hold in current position

1:  Leader trajectory: +X -> +Z -> -X -> -Z
2:  Leader trajectory: +yaw -> -Y -> -yaw -> -X

======= Follower =======|======== Leader ========
  q     w     e     r   |  u     i     o     p   
(+yaw) (+X) (-yaw) (+Z) |(+yaw) (+X) (-yaw) (+Z) 
                        |
  a     s     d     f   |  j     k     l     ;   
 (+Y)  (-X)  (-Y)  (-Z) | (+Y)  (-X)  (-Y)  (-Z) 

Please don't have caps lock on.
CTRL+c to send zero keyboard velocities and exit.

---------------------------
```

`config/hih_key.yaml` loads automatically. 

To check keys **without creating a ROS node or sending commands**, run:

```bash
rosrun hih hih_key.py --dry-run
```

## Recording

**Terminal 5 — ground station:**

```bash
rosbag record -a -O hih_experiment
```

Records all discovered ROS topics and their received messages: available state/sensor data, keyboard commands, PID/MOB diagnostics, and Follower admittance diagnostics. Robot topics are separated under `/hih1/...` and `/hih2/...`; admittance diagnostics under `/hih2/hih/admittance/...` include force reference/estimate, position/yaw corrections, and status. Only data actually published is recorded; source code and YAML files are not included. Stop with Ctrl+C and open the bag in PlotJuggler.

## Files to edit during experiments

Paths below are relative to `robots/hih/`.

| File | Purpose |
|---|---|
| `config/hih_control.yaml` | Role-specific PID/MOB gains; Follower admittance enable flag, desired contact force, virtual dynamics, and limits. Restart bringup after changes. |
| `launch/hih_leader_bringup.launch`, `launch/hih_follower_bringup.launch` | Role selection, robot ID, hardware model, and configuration loading. |
| `config/HihControl.yaml`, `config/NavigationConfig.yaml`, `config/StateEstimation.yaml`, `config/sensors/` | Inherited controller defaults, navigation, estimation, and sensor configuration. |
| `config/hih_key.yaml` | Keyboard mapping, namespaces, speeds, and input timeout. Restart the keyboard after changes. Use the same movement names as Python: `+X`, `-X`, `+Y`, `-Y`, `+Z`, `-Z`, `+yaw`, `-yaw`. |
| `src/control/hih_leader_controller.cpp` | Leader-specific control extensions; inherited PID/MOB remain in use. |
| `src/control/hih_follower_controller.cpp` | Contact-force processing and application of admittance corrections. |
| `include/hih/control/hih_admittance.h` | Four-DOF admittance dynamics, filtering, and limits. |
| `script/hih_key.py` | Key handling and command publishing. |
