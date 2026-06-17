// -*- mode: c++ -*-

#include <gimbalrotor/gimbalrotor_navigation.h>

using namespace aerial_robot_model;
using namespace aerial_robot_navigation;

GimbalrotorNavigator::GimbalrotorNavigator():
  BaseNavigator(),
  eq_cog_world_(false)
{
  curr_target_baselink_rot_.setRPY(0, 0, 0);
  final_target_baselink_rot_.setRPY(0, 0, 0);
}

void GimbalrotorNavigator::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                   boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                      boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                      double loop_du)
{
  /* initialize the flight control */
  BaseNavigator::initialize(nh, nhp, robot_model, estimator, loop_du);

  target_baselink_rpy_pub_ = nh_.advertise<spinal::DesireCoord>("desire_coordinate", 1); // to spinal
  final_target_baselink_rot_sub_ = nh_.subscribe("final_target_baselink_rot", 1, &GimbalrotorNavigator::targetBaselinkRotCallback, this);
  final_target_baselink_rpy_sub_ = nh_.subscribe("final_target_baselink_rpy", 1, &GimbalrotorNavigator::targetBaselinkRPYCallback, this);
  prev_rotation_stamp_ = ros::Time::now().toSec();

}

void GimbalrotorNavigator::update()
{
  BaseNavigator::update();
  baselinkRotationProcess();
}

void GimbalrotorNavigator::setFinalTargetBaselinkRotCallback(const spinal::DesireCoordConstPtr & msg)
{
  std::lock_guard<std::recursive_mutex> lock(baselink_target_mutex_);
  final_target_baselink_rot_.setValue(msg->roll, msg->pitch, msg->yaw);  
}

void GimbalrotorNavigator::reset()
{
  BaseNavigator::reset();

  // reset SO3
  tf::Quaternion curr_target_baselink_rot;
  {
    std::lock_guard<std::recursive_mutex> lock(baselink_target_mutex_);
    eq_cog_world_ = false;
    curr_target_baselink_rot_.setRPY(0, 0, 0);
    final_target_baselink_rot_.setRPY(0, 0, 0);
    curr_target_baselink_rot = curr_target_baselink_rot_;
  }
  KDL::Rotation rot;
  tf::quaternionTFToKDL(curr_target_baselink_rot, rot);
  robot_model_->setCogDesireOrientation(rot);
}

void GimbalrotorNavigator::targetBaselinkRotCallback(const geometry_msgs::QuaternionStampedConstPtr & msg)
{
  const tf::Vector3 target_rpy = getTargetRPY();
  {
    std::lock_guard<std::recursive_mutex> lock(baselink_target_mutex_);
    tf::quaternionMsgToTF(msg->quaternion, final_target_baselink_rot_);

    // special process
    if(target_rpy.z() != 0)
      {
        curr_target_baselink_rot_.setRPY(0, 0, target_rpy.z());
        eq_cog_world_ = true;
      }
  }
  setTargetZeroOmega(); // for sure to reset the target angular velocity
}

void GimbalrotorNavigator::targetBaselinkRPYCallback(const geometry_msgs::Vector3StampedConstPtr & msg)
{
  {
    std::lock_guard<std::recursive_mutex> lock(baselink_target_mutex_);
    final_target_baselink_rot_.setRPY(msg->vector.x, msg->vector.y, msg->vector.z);
  }
  setTargetZeroOmega(); // for sure to reset the target angular velocity
}



void GimbalrotorNavigator::naviCallback(const aerial_robot_msgs::FlightNavConstPtr & msg)
{
  std::lock_guard<std::recursive_mutex> target_lock(target_mutex_);
  BaseNavigator::naviCallback(msg);
  if(msg->roll_nav_mode == 2) setTargetRoll(msg->target_roll);
  if(msg->pitch_nav_mode == 2) setTargetPitch(msg->target_pitch);
}

void GimbalrotorNavigator::baselinkRotationProcess()
{
  const double now = ros::Time::now().toSec();
  tf::Quaternion curr_target_baselink_rot;
  {
    std::lock_guard<std::recursive_mutex> lock(baselink_target_mutex_);
    if(curr_target_baselink_rot_ == final_target_baselink_rot_) return;
    if(now - prev_rotation_stamp_ <= baselink_rot_pub_interval_) return;

    tf::Quaternion delta_q = curr_target_baselink_rot_.inverse() * final_target_baselink_rot_;
    double angle = delta_q.getAngle();
    if (angle > M_PI) angle -= 2 * M_PI;

    if(fabs(angle) > baselink_rot_change_thresh_)
      {
        curr_target_baselink_rot_ *= tf::Quaternion(delta_q.getAxis(), fabs(angle) / angle * baselink_rot_change_thresh_);
      }
    else
      curr_target_baselink_rot_ = final_target_baselink_rot_;

    curr_target_baselink_rot = curr_target_baselink_rot_;
    prev_rotation_stamp_ = now;
  }

  KDL::Rotation rot;
  tf::quaternionTFToKDL(curr_target_baselink_rot, rot);
  robot_model_->setCogDesireOrientation(rot);

  // send to spinal
  spinal::DesireCoord msg;
  double r,p,y;
  tf::Matrix3x3(curr_target_baselink_rot).getRPY(r, p, y);
  msg.roll = r;
  msg.pitch = p;
  msg.yaw = y;
  target_baselink_rpy_pub_.publish(msg);
}

void GimbalrotorNavigator::setFinalTargetBaselinkRPY(tf::Vector3 final_target_baselink_rpy)
{
  std::lock_guard<std::recursive_mutex> lock(baselink_target_mutex_);
  final_target_baselink_rot_.setRPY(final_target_baselink_rpy.x(), final_target_baselink_rpy.y(), final_target_baselink_rpy.z());
}

void GimbalrotorNavigator::forceSetTargetBaselinkRPY(tf::Vector3 target_baselink_rpy)
{
  tf::Quaternion curr_target_baselink_rot;
  {
    std::lock_guard<std::recursive_mutex> lock(baselink_target_mutex_);
    final_target_baselink_rot_.setRPY(target_baselink_rpy.x(), target_baselink_rpy.y(), target_baselink_rpy.z());
    curr_target_baselink_rot_.setRPY(target_baselink_rpy.x(), target_baselink_rpy.y(), target_baselink_rpy.z());
    curr_target_baselink_rot = curr_target_baselink_rot_;
  }

  KDL::Rotation rot;
  tf::quaternionTFToKDL(curr_target_baselink_rot, rot);
  robot_model_->setCogDesireOrientation(rot);

  //send to spinal
  spinal::DesireCoord msg;
  double r,p,y;
  tf::Matrix3x3(curr_target_baselink_rot).getRPY(r, p, y);
  msg.roll = r;
  msg.pitch = p;
  msg.yaw = y;
  target_baselink_rpy_pub_.publish(msg);
}

tf::Vector3 GimbalrotorNavigator::getCurrTargetBaselinkRPY()
{
  std::lock_guard<std::recursive_mutex> lock(baselink_target_mutex_);
  double r,p,y;
  tf::Matrix3x3(curr_target_baselink_rot_).getRPY(r, p, y);
  tf::Vector3 curr_target_baselink_rpy(r,p,y);
  return curr_target_baselink_rpy;
}

tf::Vector3 GimbalrotorNavigator::getFinalTargetBaselinkRPY()
{
  std::lock_guard<std::recursive_mutex> lock(baselink_target_mutex_);
  double r,p,y;
  tf::Matrix3x3(final_target_baselink_rot_).getRPY(r, p, y);
  tf::Vector3 final_target_baselink_rpy(r,p,y);
  return final_target_baselink_rpy;
}

void GimbalrotorNavigator::rosParamInit()
{
  BaseNavigator::rosParamInit();

  ros::NodeHandle navi_nh(nh_, "navigation");

  getParam<double>(navi_nh, "baselink_rot_change_thresh", baselink_rot_change_thresh_, 0.02);  // the threshold to change the baselink rotation
  getParam<double>(navi_nh, "baselink_rot_pub_interval", baselink_rot_pub_interval_, 0.1); // the rate to pub baselink rotation command
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::GimbalrotorNavigator, aerial_robot_navigation::BaseNavigator);
