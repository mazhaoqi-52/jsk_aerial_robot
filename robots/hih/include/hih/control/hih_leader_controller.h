#pragma once
#include <hih/control/hih_controller.h>

namespace aerial_robot_control
{
class HihLeaderController : public HihController
{
public:
  HihLeaderController();
  ~HihLeaderController() override;
  void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
    double ctrl_loop_rate) override;
};
}
