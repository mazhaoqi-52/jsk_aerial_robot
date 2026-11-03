std_msgs/Header header
geometry_msgs/Wrench wrench_acc
geometry_msgs/Wrench desired_wrench
float64[6] desired_wrench_weights
geometry_msgs/Wrench observer_feedback_wrench
float64[6] observer_feedback_wrench_weights
float64 formation_mass
geometry_msgs/Vector3 formation_cog_offset
float64[9] formation_inertia
float64 yaw_pid_raw

# Phase B: leader's navigator setpoints broadcast so followers can derive their
# per-module reference via rigid-formation kinematics (instead of reading their
# own stale navigator). Frame conventions match navigator getters:
#   leader_target_pos / _vel / _acc : world frame
#   leader_target_rpy               : PID target attitude; in unified tilt
#                                     commands, roll/pitch stay decoupled from
#                                     physical baselink tilt to avoid I windup
#   leader_final_target_baselink_rpy: final physical baselink attitude used for
#                                    rigid CoG-offset geometry
#   leader_target_omega / _ang_acc  : leader's target body frame
geometry_msgs/Vector3 leader_target_pos
geometry_msgs/Vector3 leader_target_vel
geometry_msgs/Vector3 leader_target_acc
geometry_msgs/Vector3 leader_target_rpy
geometry_msgs/Vector3 leader_final_target_baselink_rpy
geometry_msgs/Vector3 leader_target_omega
geometry_msgs/Vector3 leader_target_ang_acc
