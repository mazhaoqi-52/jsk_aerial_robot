std_msgs/Header header
geometry_msgs/Wrench wrench_acc
geometry_msgs/Wrench desired_wrench
float64 formation_mass
geometry_msgs/Vector3 formation_cog_offset
float64[9] formation_inertia
float64 yaw_pid_raw

# Phase B: leader's navigator setpoints broadcast so followers can derive their
# per-module reference via rigid-formation kinematics (instead of reading their
# own stale navigator). Frame conventions match navigator getters:
#   leader_target_pos / _vel / _acc : world frame
#   leader_target_rpy               : roll/pitch/yaw of formation body
#   leader_target_omega / _ang_acc  : leader's target body frame
geometry_msgs/Vector3 leader_target_pos
geometry_msgs/Vector3 leader_target_vel
geometry_msgs/Vector3 leader_target_acc
geometry_msgs/Vector3 leader_target_rpy
geometry_msgs/Vector3 leader_target_omega
geometry_msgs/Vector3 leader_target_ang_acc