#include <hih/control/hih_leader_controller.h>
#include <hih/control/hih_role_parameters.h>
#include <pluginlib/class_list_macros.h>

namespace aerial_robot_control
{
HihLeaderController::HihLeaderController() { wrench_estimate_flag_ = false; }

HihLeaderController::~HihLeaderController()
{
  // Stop the inherited observer before the HihController members are destroyed.
  if (wrench_estimate_thread_.joinable())
  { wrench_estimate_thread_.interrupt(); wrench_estimate_thread_.join(); }
  wrench_estimate_flag_ = false;
}

void HihLeaderController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
  boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator, double ctrl_loop_rate)
{
  hih_control::applyRoleParameters(nh, "leader");
  HihController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);
  ROS_INFO("[HiH Leader] Existing PID, allocation and momentum observer retained");
}
}

PLUGINLIB_EXPORT_CLASS(aerial_robot_control::HihLeaderController, aerial_robot_control::ControlBase);
