#pragma once

#include <aerial_robot_control/flight_navigation.h>
#include <ros/ros.h>
#include <tf/LinearMath/Vector3.h>
#include <memory>
#include <string>

namespace hih_control
{
// All calculations and ROS integration are in hih_cop_adjustment.cpp.
// Gains produce an OFFSET RATE: yaw [rad/(m*s)], Z [1/s].
struct CopAxisParameters
{
  bool enabled = true;
  double gain = 0.2;
  double direction = 1.0;
  double max_offset = 0.02;
  double max_rate = 0.005;
};

struct CopAdjustmentParameters
{
  bool enabled = false;
  CopAxisParameters yaw{true, 0.17453292519943295, 1.0, 0.05235987755982989, 0.008726646259971648};
  CopAxisParameters z;
  double safe_y = 0.05, safe_z = 0.05;
  double hysteresis_y = 0.005, hysteresis_z = 0.005;
  double min_normal_force = 1.0;
  double max_abs_cop_y = 0.25, max_abs_cop_z = 0.25;
  double filter_cutoff_hz = 5.0;
  double sample_timeout = 0.15, max_dt = 0.1, debug_period = 0.05;
  tf::Vector3 force_bias_contact{0, 0, 0};
  tf::Vector3 torque_bias_contact{0, 0, 0};
  void validate() const;
};

struct CopAdjustmentState
{
  tf::Vector3 force_contact{0, 0, 0}, torque_contact{0, 0, 0};
  tf::Vector3 raw{0, 0, 0}, filtered{0, 0, 0}, error{0, 0, 0};
  double normal_force = 0;
  double yaw_offset = 0, yaw_rate = 0, z_offset = 0, z_velocity = 0;
  bool valid = false, applying = false;
  bool yaw_boundary_active = false, z_boundary_active = false;
  bool yaw_saturated = false, z_saturated = false;
  std::string status = "uninitialized";
};

class HihCopAdjustment
{
public:
  HihCopAdjustment();
  void configure(const CopAdjustmentParameters& parameters,
                 const tf::Vector3& normal_contact = tf::Vector3(1, 0, 0));
  void initialize(ros::NodeHandle nh, const tf::Vector3& normal_contact,
                  const std::string& world_frame);
  void reset();
  void setEnabled(bool master, bool yaw, bool z);
  const CopAdjustmentState& state() const;

  // Numerical path: no ROS node, timers, subscribers, or parameter access.
  // Wrench is about the model contact center, in contact axes; stamp/now [s].
  const CopAdjustmentState& step(const tf::Vector3& force_contact,
    const tf::Vector3& torque_contact, double sample_stamp, bool sample_valid,
    double now, double dt, bool flight_allowed, bool admittance_applying);
  void applyReference(aerial_robot_navigation::BaseNavigator::TargetSnapshot& target) const;

  // Called once AFTER existing Admittance, BEFORE the inherited PID.
  void update(aerial_robot_navigation::BaseNavigator::TargetSnapshot& target,
    const tf::Vector3& force_contact, const tf::Vector3& torque_contact,
    const ros::Time& sample_stamp, bool sample_valid, bool flight_allowed,
    bool admittance_applying, const ros::Time& now);

private:
  void resetInput();
  void publishDebug(const aerial_robot_navigation::BaseNavigator::TargetSnapshot& before,
                    const aerial_robot_navigation::BaseNavigator::TargetSnapshot& after,
                    const ros::Time& sample_stamp, const ros::Time& now);
  CopAdjustmentParameters p_;
  CopAdjustmentState state_;
  bool supported_normal_ = true, filter_initialized_ = false;
  double normal_sign_ = 1;  // contact outward normal: +X or -X
  double filter_stamp_ = 0, newest_sample_stamp_ = 0, last_control_time_ = 0;
  std::unique_ptr<ros::NodeHandle> settings_;
  std::string world_frame_ = "world", contact_frame_;
  ros::Time last_debug_;
  ros::Publisher wrench_pub_, raw_pub_, filtered_pub_, error_pub_, normal_force_pub_;
  ros::Publisher valid_pub_, applying_pub_, status_pub_, yaw_saturated_pub_, z_saturated_pub_;
  ros::Publisher yaw_offset_pub_, yaw_rate_pub_, z_offset_pub_, z_velocity_pub_;
  ros::Publisher admittance_position_pub_, command_position_pub_, yaw_reference_pub_;
};
}  // namespace hih_control
