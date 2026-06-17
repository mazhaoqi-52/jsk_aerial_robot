// -*- mode: c++ -*-

#pragma once

#include <aerial_robot_control/flight_navigation.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <spinal/DesireCoord.h>
#include <mutex>

namespace aerial_robot_navigation
{
  class GimbalrotorNavigator : public BaseNavigator
  {
  public:
    GimbalrotorNavigator();
    ~GimbalrotorNavigator(){}

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                    double loop_du) override;

    void update() override;

    tf::Quaternion getCurrTargetBaselinkRot()
    {
      std::lock_guard<std::recursive_mutex> lock(baselink_target_mutex_);
      return curr_target_baselink_rot_;
    }
    tf::Quaternion getFinalTargetBaselinkRot()
    {
      std::lock_guard<std::recursive_mutex> lock(baselink_target_mutex_);
      return final_target_baselink_rot_;
    }
    tf::Vector3 getCurrTargetBaselinkRPY();
    tf::Vector3 getFinalTargetBaselinkRPY();
 
    void setFinalTargetBaselinkRPY(tf::Vector3 final_target_baselink_rpy);
    void forceSetTargetBaselinkRPY(tf::Vector3 target_baselink_rpy);
  protected:
    void rosParamInit() override;
    virtual void setFinalTargetBaselinkRotCallback(const spinal::DesireCoordConstPtr & msg);
    virtual void naviCallback(const aerial_robot_msgs::FlightNavConstPtr & msg) override;
    
  private:
    ros::Publisher target_baselink_rpy_pub_;
    ros::Subscriber final_target_baselink_rot_sub_, final_target_baselink_rpy_sub_;

    void baselinkRotationProcess();
    void targetBaselinkRotCallback(const geometry_msgs::QuaternionStampedConstPtr & msg);
    void targetBaselinkRPYCallback(const geometry_msgs::Vector3StampedConstPtr & msg);

    void reset() override;

    /* target baselink rotation */
    mutable std::recursive_mutex baselink_target_mutex_;
    double prev_rotation_stamp_;
    tf::Quaternion curr_target_baselink_rot_, final_target_baselink_rot_;
    bool eq_cog_world_;

    /* rosparam */
    double baselink_rot_change_thresh_;
    double baselink_rot_pub_interval_;
  };
};
