// -*- mode: c++ -*-

#include <beetle/beetle_navigation.h>
#include <aerial_robot_control/util/joy_parser.h>

using namespace aerial_robot_model;
using namespace aerial_robot_navigation;

BeetleNavigator::BeetleNavigator():
  GimbalrotorNavigator(),
  roll_pitch_control_flag_(false),
  pre_assembled_(false),
  current_assembled_(false),
  module_state_(SEPARATED),
  leader_fix_flag_(false),
  tfBuffer_(),
  tfListener_(tfBuffer_),
  pre_assembled_modules_(0),
  my_index_(0),
  joy_roll_positive_flag_(false),
  joy_roll_negative_flag_(false),
  joy_pitch_positive_flag_(false),
  joy_pitch_negative_flag_(false),
  pseudo_assembly_mode_(false)
{}

void BeetleNavigator::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                   boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                 boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                 double loop_du)
{
  GimbalrotorNavigator::initialize(nh, nhp, robot_model, estimator, loop_du);
  nh_ = nh;
  nhp_ = nhp;
  cog_com_dist_pub_ = nh_.advertise<geometry_msgs::Point>("cog_com_dist", 1);
  assembly_nav_sub_ = nh_.subscribe("/assembly/uav/nav", 1, &BeetleNavigator::assemblyNavCallback, this);
  assembly_target_rot_sub_ = nh_.subscribe("/assembly/final_target_baselink_rot", 1, &BeetleNavigator::setAssemblyFinalTargetBaselinkRotCallback, this);

  // Initialize Assembly CoG odom publisher
  nhp_.param("publish_assembly_odom", publish_assembly_odom_, true);
  if(publish_assembly_odom_) {
    assembly_cog_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/assemble/cog/odom", 10);
    ROS_INFO("[BeetleNavigator] Assembly CoG odom publisher initialized: /assemble/cog/odom");
  }

  beetle_robot_model_ = boost::dynamic_pointer_cast<BeetleRobotModel>(robot_model);

  for(int i = 0; i < max_modules_num_; i++){
    std::string module_name  = string("/") + getMyName() + std::to_string(i+1);
    assembly_flag_subs_.insert(make_pair(module_name, nh_.subscribe( module_name + string("/assembly_flag"), 1, &BeetleNavigator::assemblyFlagCallback, this)));
    // Mirror the unified controller's ModuleModel pub/sub so the nav can do
    // mass-weighted calcCenterOfMoving — same `latched` publisher upstream.
    module_model_subs_.insert(make_pair(module_name, nh_.subscribe( module_name + string("/unified_control/module_model"), 1, &BeetleNavigator::moduleModelCallback, this)));
    assembly_flags_.insert(std::make_pair(i+1,false));
  }

}

void BeetleNavigator::moduleModelCallback(const beetle::ModuleModel& msg)
{
  if (msg.id == 0 || !std::isfinite(msg.mass) || msg.mass <= 0.0) {
    ROS_WARN_THROTTLE(1.0,
                      "[UnifiedNav id=%d] Reject ModuleModel msg id=%d mass=%.6f",
                      my_id_, msg.id, msg.mass);
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_module_masses_);
  const auto it = module_masses_.find(msg.id);
  const bool changed = (it == module_masses_.end() || it->second != msg.mass);
  module_masses_[msg.id] = msg.mass;
  if (changed) {
    ROS_INFO("[UnifiedNav id=%d] Received ModuleModel mass id=%d mass=%.3f",
             my_id_, msg.id, msg.mass);
  }
}

void BeetleNavigator::joyStickControl(const sensor_msgs::JoyConstPtr & joy_msg)
{
  auto copied_joy_msg = boost::make_shared<sensor_msgs::Joy>(*joy_msg);  

  if(joy_duplicated_flag_)
    {
      BaseNavigator::joyStickControl(copied_joy_msg);
      return;
    }

  sensor_msgs::Joy joy_cmd = joyParse(*joy_msg);
  if (joy_cmd.axes.size() == 0 || joy_cmd.buttons.size() == 0)
    {
      ROS_WARN("the joystick type is not supported (buttons: %d, axes: %d)", (int)joy_msg->buttons.size(), (int)joy_msg->axes.size());
      return;
    }  

  /* Motion: Roll and Pitch*/
  /* this is the roll and pitch_angle control */
  
  if(joy_cmd.buttons[JOY_BUTTON_REAR_LEFT_1] == 1 && joy_cmd.buttons[JOY_BUTTON_REAR_RIGHT_1] == 1)
    {
      tf::Vector3 target_roll_pitch;
      target_roll_pitch.setX(0.0);
      target_roll_pitch.setY(0.0);
      setFinalTargetBaselinkRPY(target_roll_pitch);
      ROS_INFO_STREAM("Set target base link rot to horizontal pose.");
      return;
    }
  else if(joy_cmd.buttons[JOY_BUTTON_REAR_LEFT_1] == 1)
    {
      tf::Vector3 target_roll_pitch = getFinalTargetBaselinkRPY();
      //pitch positive
      if(joy_cmd.buttons[JOY_BUTTON_CROSS_UP] == 1 && !joy_pitch_positive_flag_)
        {
          target_roll_pitch.setY(getFinalTargetBaselinkRPY().y()
                                 + max_target_roll_pitch_rate_);
          joy_pitch_positive_flag_ = true;
          ROS_INFO_STREAM("Set target base link rot to [" << target_roll_pitch.x() << ", "<< target_roll_pitch.y() << ", "<< target_roll_pitch.z() << "]");
        }
      else if(joy_cmd.buttons[JOY_BUTTON_CROSS_UP] != 1 && joy_pitch_positive_flag_)
        {
          joy_pitch_positive_flag_ = false;
        }
      //pitch negative
      if(joy_cmd.buttons[JOY_BUTTON_CROSS_DOWN] == 1 && !joy_pitch_negative_flag_)
        {
          target_roll_pitch.setY(getFinalTargetBaselinkRPY().y()
                                 - max_target_roll_pitch_rate_);
          joy_pitch_negative_flag_ = true;
          ROS_INFO_STREAM("Set target base link rot to [" << target_roll_pitch.x() << ", "<< target_roll_pitch.y() << ", "<< target_roll_pitch.z() << "]");
          
        }
      else if(joy_cmd.buttons[JOY_BUTTON_CROSS_DOWN] != 1 && joy_pitch_negative_flag_)
        {
          joy_pitch_negative_flag_ = false;
          
        }

      //roll positive
      if(joy_cmd.buttons[JOY_BUTTON_CROSS_RIGHT] == 1 && !joy_roll_positive_flag_)
        {
          target_roll_pitch.setX(getFinalTargetBaselinkRPY().x()
                                 + max_target_roll_pitch_rate_);
          joy_roll_positive_flag_ = true;
          ROS_INFO_STREAM("Set target base link rot to [" << target_roll_pitch.x() << ", "<< target_roll_pitch.y() << ", "<< target_roll_pitch.z() << "]");
        }
      else if(joy_cmd.buttons[JOY_BUTTON_CROSS_RIGHT] != 1 && joy_roll_positive_flag_)
        {
          joy_roll_positive_flag_ = false;
          
        }
      //roll negative
      if(joy_cmd.buttons[JOY_BUTTON_CROSS_LEFT] == 1 && !joy_roll_negative_flag_)
        {
          target_roll_pitch.setX(getFinalTargetBaselinkRPY().x()
                                 - max_target_roll_pitch_rate_);
          joy_roll_negative_flag_ = true;
          ROS_INFO_STREAM("Set target base link rot to [" << target_roll_pitch.x() << ", "<< target_roll_pitch.y() << ", "<< target_roll_pitch.z() << "]");
        }
      else if(joy_cmd.buttons[JOY_BUTTON_CROSS_LEFT] != 1 && joy_roll_negative_flag_)
        {
          joy_roll_negative_flag_ = false;
          
        }
      setFinalTargetBaselinkRPY(target_roll_pitch);
      return;
    }
  else
    {
      joy_roll_positive_flag_ = false;
      joy_roll_negative_flag_ = false;
      joy_pitch_positive_flag_ = false;
      joy_pitch_negative_flag_ = false;
    }

  
  BaseNavigator::joyStickControl(copied_joy_msg);
}

void BeetleNavigator::naviCallback(const aerial_robot_msgs::FlightNavConstPtr & msg)
{
  if(getNaviState() == TAKEOFF_STATE || BaseNavigator::getNaviState() == LAND_STATE || getModuleState() != SEPARATED) return;

  gps_waypoint_ = false;

  if(force_att_control_flag_) return;

  std::lock_guard<std::recursive_mutex> target_lock(target_mutex_);

  /* yaw */
  if(msg->yaw_nav_mode == aerial_robot_msgs::FlightNav::POS_MODE)
    {
      setTargetYaw(angles::normalize_angle(msg->target_yaw));
      setTargetOmegaZ(0);
    }
  if(msg->yaw_nav_mode == aerial_robot_msgs::FlightNav::VEL_MODE)
    {
      setTargetOmegaZ(msg->target_omega_z);

      teleop_reset_time_ = teleop_reset_duration_ + ros::Time::now().toSec();
    }
  if(msg->yaw_nav_mode == aerial_robot_msgs::FlightNav::POS_VEL_MODE)
    {
      setTargetYaw(angles::normalize_angle(msg->target_yaw));
      setTargetOmegaZ(msg->target_omega_z);

      trajectory_mode_ = true;
      trajectory_reset_time_ = trajectory_reset_duration_ + ros::Time::now().toSec();
    }

  /* xy control */
  switch(msg->pos_xy_nav_mode)
    {
    case aerial_robot_msgs::FlightNav::POS_MODE:
      {
        tf::Vector3 target_cog_pos(msg->target_pos_x, msg->target_pos_y, 0);
        if(msg->target == aerial_robot_msgs::FlightNav::BASELINK)
          {
            /* check the transformation */
            tf::Transform cog2baselink_tf;
            tf::transformKDLToTF(robot_model_->getCog2Baselink<KDL::Frame>(), cog2baselink_tf);
            target_cog_pos -= tf::Matrix3x3(tf::createQuaternionFromYaw(getTargetRPY().z()))
              * cog2baselink_tf.getOrigin();
          }
        else if(msg->target == CONTACT_POINT)
          {
            /* check the transformation */
            tf::Transform cog2cp_tf;
            tf::transformKDLToTF(beetle_robot_model_->getCog2Cp<KDL::Frame>(), cog2cp_tf);
            target_cog_pos -= cog2cp_tf.getOrigin();
          }

        tf::Vector3 target_delta = getTargetPos() - target_cog_pos;
        target_delta.setZ(0);

        if(target_delta.length() > vel_nav_threshold_)
          {
            ROS_WARN_THROTTLE(1.0, "start vel nav control for waypoint");
            vel_based_waypoint_ = true;
            xy_control_mode_ = VEL_CONTROL_MODE;
          }

        if(!vel_based_waypoint_)
          xy_control_mode_ = POS_CONTROL_MODE;

        setTargetPosX(target_cog_pos.x());
        setTargetPosY(target_cog_pos.y());

        setTargetVelX(0);
        setTargetVelY(0);

        break;
      }
    case aerial_robot_msgs::FlightNav::VEL_MODE:
      {
        /* do not switch to pure vel mode */
        xy_control_mode_ = POS_CONTROL_MODE;

        teleop_reset_time_ = teleop_reset_duration_ + ros::Time::now().toSec();

        switch(msg->control_frame)
          {
          case WORLD_FRAME:
            {
              setTargetVelX(msg->target_vel_x);
              setTargetVelY(msg->target_vel_y);
              break;
            }
          case LOCAL_FRAME:
            {
              double yaw_angle = estimator_->getState(Frame::COG, estimate_mode_).z();
              tf::Vector3 target_vel = frameConversion(tf::Vector3(msg->target_vel_x, msg->target_vel_y, 0), yaw_angle);
              setTargetVelX(target_vel.x());
              setTargetVelY(target_vel.y());
              break;
            }
          default:
            {
              break;
            }
          }
        break;
      }
    case aerial_robot_msgs::FlightNav::POS_VEL_MODE:
      {        
        xy_control_mode_ = POS_CONTROL_MODE;

        tf::Vector3 target_cog_pos(msg->target_pos_x, msg->target_pos_y, 0);
        if(msg->target == aerial_robot_msgs::FlightNav::BASELINK)
          {
            /* check the transformation */
            tf::Transform cog2baselink_tf;
            tf::transformKDLToTF(robot_model_->getCog2Baselink<KDL::Frame>(), cog2baselink_tf);
            target_cog_pos -= tf::Matrix3x3(tf::createQuaternionFromYaw(getTargetRPY().z()))
              * cog2baselink_tf.getOrigin();
          }
        else if(msg->target == CONTACT_POINT)
          {
            /* check the transformation */
            tf::Transform cog2cp_tf;
            tf::transformKDLToTF(beetle_robot_model_->getCog2Cp<KDL::Frame>(), cog2cp_tf);
            target_cog_pos -= cog2cp_tf.getOrigin();
          }        

        setTargetPosX(target_cog_pos.x());
        setTargetPosY(target_cog_pos.y());            

        setTargetVelX(msg->target_vel_x);
        setTargetVelY(msg->target_vel_y);

        trajectory_mode_ = true;
        trajectory_reset_time_ = trajectory_reset_duration_ + ros::Time::now().toSec();

        break;
      }
    case aerial_robot_msgs::FlightNav::ACC_MODE:
      {
        /* should be in COG frame */
        xy_control_mode_ = ACC_CONTROL_MODE;
        prev_xy_control_mode_ = ACC_CONTROL_MODE;

        switch(msg->control_frame)
          {
          case WORLD_FRAME:
            {
              setTargetAccX(msg->target_acc_x);
              setTargetAccY(msg->target_acc_y);
              break;
            }
          case LOCAL_FRAME:
            {
              double yaw_angle = estimator_->getState(Frame::COG, estimate_mode_).z();
              tf::Vector3 target_acc = frameConversion(tf::Vector3(msg->target_acc_x, msg->target_acc_y, 0), yaw_angle);
              setTargetAccX(target_acc.x());
              setTargetAccY(target_acc.y());
              break;
            }
          default:
            {
              break;
            }
          }
        break;
      }
    case aerial_robot_msgs::FlightNav::STAY_HERE_MODE:
      {
        xy_control_mode_ = POS_CONTROL_MODE;
        setTargetVelX(0);
        setTargetVelY(0);
        setTargetXyFromCurrentState();
        break;
      }      
    case aerial_robot_msgs::FlightNav::GPS_WAYPOINT_MODE:
      {
        target_wp_ = geodesy::toMsg(msg->target_pos_x, msg->target_pos_y);
        gps_waypoint_ = true;

        break;
      }
    }
  if(msg->pos_xy_nav_mode != aerial_robot_msgs::FlightNav::ACC_MODE) setTargetZeroAcc();

  /* z */
  if(msg->pos_z_nav_mode == aerial_robot_msgs::FlightNav::VEL_MODE)
    {
      setTargetVelZ(msg->target_vel_z);
      teleop_reset_time_ = teleop_reset_duration_ + ros::Time::now().toSec();
    }
  else if(msg->pos_z_nav_mode == aerial_robot_msgs::FlightNav::POS_MODE)
    {
      tf::Vector3 target_cog_pos(0, 0, msg->target_pos_z);
      if(msg->target == aerial_robot_msgs::FlightNav::BASELINK)
        {

          /* check the transformation */
          tf::Transform cog2baselink_tf;
          tf::transformKDLToTF(robot_model_->getCog2Baselink<KDL::Frame>(), cog2baselink_tf);
          target_cog_pos -= cog2baselink_tf.getOrigin();
        }
      else if(msg->target == CONTACT_POINT)
        {
          /* check the transformation */
          tf::Transform cog2cp_tf;
          tf::transformKDLToTF(beetle_robot_model_->getCog2Cp<KDL::Frame>(), cog2cp_tf);
          target_cog_pos -= cog2cp_tf.getOrigin();
        }

      setTargetPosZ(target_cog_pos.z());

      setTargetVelZ(0);
    }
  else if(msg->pos_z_nav_mode == aerial_robot_msgs::FlightNav::POS_VEL_MODE)
    {
      tf::Vector3 target_cog_pos(0, 0, msg->target_pos_z);
      if(msg->target == aerial_robot_msgs::FlightNav::BASELINK)
        {

          /* check the transformation */
          tf::Transform cog2baselink_tf;
          tf::transformKDLToTF(robot_model_->getCog2Baselink<KDL::Frame>(), cog2baselink_tf);
          target_cog_pos -= cog2baselink_tf.getOrigin();
        }
      else if(msg->target == CONTACT_POINT)
        {
          /* check the transformation */
          tf::Transform cog2cp_tf;
          tf::transformKDLToTF(beetle_robot_model_->getCog2Cp<KDL::Frame>(), cog2cp_tf);
          target_cog_pos -= cog2cp_tf.getOrigin();
        }

      trajectory_mode_ = true;
      trajectory_reset_time_ = trajectory_reset_duration_ + ros::Time::now().toSec();

      setTargetPosZ(target_cog_pos.z());
      setTargetVelZ(msg->target_vel_z);
    }

  /* pitch control */
  if(msg->pitch_nav_mode == aerial_robot_msgs::FlightNav::POS_MODE)
    {
      tf::Vector3 target_roll_pitch = getFinalTargetBaselinkRPY();
      target_roll_pitch.setY(msg->target_pitch);
      setFinalTargetBaselinkRPY(target_roll_pitch);
    }
  if(msg->pitch_nav_mode == aerial_robot_msgs::FlightNav::POS_VEL_MODE)
    {
      tf::Vector3 target_roll_pitch = getFinalTargetBaselinkRPY();
      target_roll_pitch.setY(msg->target_pitch);
      setFinalTargetBaselinkRPY(target_roll_pitch);
      
      trajectory_mode_ = true;
      trajectory_reset_time_ = trajectory_reset_duration_ + ros::Time::now().toSec();
    }

  /* roll control */
  if(msg->roll_nav_mode == aerial_robot_msgs::FlightNav::POS_MODE)
    {
      tf::Vector3 target_roll_pitch = getFinalTargetBaselinkRPY();
      target_roll_pitch.setX(msg->target_roll);
      setFinalTargetBaselinkRPY(target_roll_pitch);
    }
  if(msg->roll_nav_mode == aerial_robot_msgs::FlightNav::POS_VEL_MODE)
    {
      tf::Vector3 target_roll_pitch = getFinalTargetBaselinkRPY();
      target_roll_pitch.setX(msg->target_roll);
      setFinalTargetBaselinkRPY(target_roll_pitch);
      
      trajectory_mode_ = true;
      trajectory_reset_time_ = trajectory_reset_duration_ + ros::Time::now().toSec();
    }
}


void BeetleNavigator::assemblyNavCallback(const aerial_robot_msgs::FlightNavConstPtr & msg)
{
  if(getNaviState() == TAKEOFF_STATE || BaseNavigator::getNaviState() == LAND_STATE || getModuleState() == SEPARATED) return;

  gps_waypoint_ = false;

  if(force_att_control_flag_) return;

  // ======== Unified Control Mode Navigation ========
  // In unified mode, user-facing positions are assembly-CoG targets. Store the
  // equivalent module-CoG target internally so the unified controller can
  // reconstruct the same target formation CoG before solving one formation
  // allocation and slicing the result for this module.
  if (getUnifiedControlMode() &&
      (getModuleState() == LEADER || getModuleState() == FOLLOWER)) {

    std::lock_guard<std::recursive_mutex> target_lock(target_mutex_);
    const tf::Vector3 prev_final_target_baselink_rpy = getFinalTargetBaselinkRPY();
    tf::Vector3 final_target_baselink_rpy = prev_final_target_baselink_rpy;
    const double prev_target_yaw = getTargetRPY().z();
    double target_yaw = prev_target_yaw;
    if (msg->pitch_nav_mode == aerial_robot_msgs::FlightNav::POS_MODE) {
      final_target_baselink_rpy.setY(msg->target_pitch);
    }
    if (msg->roll_nav_mode == aerial_robot_msgs::FlightNav::POS_MODE) {
      final_target_baselink_rpy.setX(msg->target_roll);
    }
    if (msg->yaw_nav_mode == aerial_robot_msgs::FlightNav::POS_MODE ||
        msg->yaw_nav_mode == aerial_robot_msgs::FlightNav::POS_VEL_MODE) {
      target_yaw = angles::normalize_angle(msg->target_yaw);
    }

    // Compute assembly CoG -> this module's CoG offset under the commanded
    // baselink attitude. Yaw-only rotation keeps com_offset.z at zero during
    // formation pitch/roll, so modules never receive the height split needed to
    // keep the assembly CoG at the requested altitude.
    tf::Transform cog2com_tf;
    tf::transformKDLToTF(getCog2CoM<KDL::Frame>(), cog2com_tf);
    tf::Matrix3x3 prev_baselink_rot;
    prev_baselink_rot.setRPY(prev_final_target_baselink_rpy.x(),
                             prev_final_target_baselink_rpy.y(),
                             prev_target_yaw);
    const tf::Vector3 prev_com_offset = prev_baselink_rot * cog2com_tf.getOrigin();
    tf::Matrix3x3 target_baselink_rot;
    target_baselink_rot.setRPY(final_target_baselink_rpy.x(),
                               final_target_baselink_rpy.y(),
                               target_yaw);
    const tf::Vector3 com_offset = target_baselink_rot * cog2com_tf.getOrigin();
    const tf::Vector3 preserved_assembly_target = getTargetPos() + prev_com_offset;
    const bool pose_attitude_cmd =
        (msg->pitch_nav_mode == aerial_robot_msgs::FlightNav::POS_MODE ||
         msg->roll_nav_mode == aerial_robot_msgs::FlightNav::POS_MODE ||
         msg->yaw_nav_mode == aerial_robot_msgs::FlightNav::POS_MODE ||
         msg->yaw_nav_mode == aerial_robot_msgs::FlightNav::POS_VEL_MODE);

    /* yaw */
    if(msg->yaw_nav_mode == aerial_robot_msgs::FlightNav::POS_MODE) {
      setTargetYaw(target_yaw);
      setTargetOmegaZ(0);
    }
    if(msg->yaw_nav_mode == aerial_robot_msgs::FlightNav::VEL_MODE) {
      setTargetOmegaZ(msg->target_omega_z);
      teleop_reset_time_ = teleop_reset_duration_ + ros::Time::now().toSec();
    }
    if(msg->yaw_nav_mode == aerial_robot_msgs::FlightNav::POS_VEL_MODE) {
      setTargetYaw(target_yaw);
      setTargetOmegaZ(msg->target_omega_z);
      trajectory_mode_ = true;
      trajectory_reset_time_ = trajectory_reset_duration_ + ros::Time::now().toSec();
    }

    /* xy — convert assembly CoG target to this module's CoG target */
    switch(msg->pos_xy_nav_mode) {
    case aerial_robot_msgs::FlightNav::POS_MODE:
      {
        xy_control_mode_ = POS_CONTROL_MODE;
        vel_based_waypoint_ = false;
        setTargetPosX(msg->target_pos_x - com_offset.x());
        setTargetPosY(msg->target_pos_y - com_offset.y());
        setTargetVelX(0);
        setTargetVelY(0);
        break;
      }
    case aerial_robot_msgs::FlightNav::VEL_MODE:
      {
        xy_control_mode_ = POS_CONTROL_MODE;
        teleop_reset_time_ = teleop_reset_duration_ + ros::Time::now().toSec();
        switch(msg->control_frame) {
        case WORLD_FRAME:
          setTargetVelX(msg->target_vel_x);
          setTargetVelY(msg->target_vel_y);
          break;
        case LOCAL_FRAME:
          {
            double yaw_angle = estimator_->getState(Frame::COG, estimate_mode_).z();
            tf::Vector3 target_vel = frameConversion(
                tf::Vector3(msg->target_vel_x, msg->target_vel_y, 0), yaw_angle);
            setTargetVelX(target_vel.x());
            setTargetVelY(target_vel.y());
            break;
          }
        default: break;
        }
        break;
      }
    case aerial_robot_msgs::FlightNav::POS_VEL_MODE:
      {
        xy_control_mode_ = POS_CONTROL_MODE;
        setTargetPosX(msg->target_pos_x - com_offset.x());
        setTargetPosY(msg->target_pos_y - com_offset.y());
        setTargetVelX(msg->target_vel_x);
        setTargetVelY(msg->target_vel_y);
        trajectory_mode_ = true;
        trajectory_reset_time_ = trajectory_reset_duration_ + ros::Time::now().toSec();
        break;
      }
    case aerial_robot_msgs::FlightNav::STAY_HERE_MODE:
      {
        xy_control_mode_ = POS_CONTROL_MODE;
        setTargetVelX(0);
        setTargetVelY(0);
        setTargetXyFromCurrentState();
        break;
      }
    default: break;
    }
    if (pose_attitude_cmd &&
        msg->pos_xy_nav_mode != aerial_robot_msgs::FlightNav::POS_MODE &&
        msg->pos_xy_nav_mode != aerial_robot_msgs::FlightNav::POS_VEL_MODE &&
        msg->pos_xy_nav_mode != aerial_robot_msgs::FlightNav::VEL_MODE &&
        msg->pos_xy_nav_mode != aerial_robot_msgs::FlightNav::ACC_MODE &&
        msg->pos_xy_nav_mode != aerial_robot_msgs::FlightNav::STAY_HERE_MODE) {
      setTargetPosX(preserved_assembly_target.x() - com_offset.x());
      setTargetPosY(preserved_assembly_target.y() - com_offset.y());
    }
    if(msg->pos_xy_nav_mode != aerial_robot_msgs::FlightNav::ACC_MODE) setTargetZeroAcc();

    /* z — convert assembly CoG target to this module's CoG target */
    if(msg->pos_z_nav_mode == aerial_robot_msgs::FlightNav::VEL_MODE) {
      setTargetVelZ(msg->target_vel_z);
      teleop_reset_time_ = teleop_reset_duration_ + ros::Time::now().toSec();
    }
    else if(msg->pos_z_nav_mode == aerial_robot_msgs::FlightNav::POS_MODE) {
      setTargetPosZ(msg->target_pos_z - com_offset.z());
      setTargetVelZ(0);
    }
    else if(msg->pos_z_nav_mode == aerial_robot_msgs::FlightNav::POS_VEL_MODE) {
      setTargetPosZ(msg->target_pos_z - com_offset.z());
      setTargetVelZ(msg->target_vel_z);
      trajectory_mode_ = true;
      trajectory_reset_time_ = trajectory_reset_duration_ + ros::Time::now().toSec();
    }
    else if (pose_attitude_cmd &&
             msg->pos_z_nav_mode != aerial_robot_msgs::FlightNav::VEL_MODE &&
             msg->pos_z_nav_mode != aerial_robot_msgs::FlightNav::ACC_MODE) {
      setTargetPosZ(preserved_assembly_target.z() - com_offset.z());
      setTargetVelZ(0);
    }

    /* Physical baselink roll/pitch target for the assembled formation. Keep
     * this separate from target_rpy_: the latter stabilizes the virtual CoG
     * frame while desire_coordinate moves the physical baselink equilibrium.
     * Writing both applies the same attitude twice and winds up the R/P I-term. */
    if(msg->pitch_nav_mode == aerial_robot_msgs::FlightNav::POS_MODE ||
       msg->roll_nav_mode == aerial_robot_msgs::FlightNav::POS_MODE) {
      setFinalTargetBaselinkRPY(final_target_baselink_rpy);
    }

    {
      ROS_DEBUG_THROTTLE(1.0, "[UnifiedNav id=%d state=%d modes(xy=%u,z=%u,r=%u,p=%u)] "
                         "msg_target=(%.3f,%.3f,%.3f) preserved_assembly=(%.3f,%.3f,%.3f) "
                         "my_target=(%.3f,%.3f,%.3f) com_offset=(%.3f,%.3f,%.3f) yaw=%.3f",
                         my_id_, static_cast<int>(getModuleState()),
                         msg->pos_xy_nav_mode, msg->pos_z_nav_mode,
                         msg->roll_nav_mode, msg->pitch_nav_mode,
                         msg->target_pos_x, msg->target_pos_y, msg->target_pos_z,
                         preserved_assembly_target.x(), preserved_assembly_target.y(), preserved_assembly_target.z(),
                         getTargetPos().x(), getTargetPos().y(), getTargetPos().z(),
                         com_offset.x(), com_offset.y(), com_offset.z(),
                         getTargetRPY().z());
    }
    return;  // Done — skip leader-follower path below
  }

  // ======== Leader-Follower Mode Navigation (original path) ========
  std::lock_guard<std::recursive_mutex> target_lock(target_mutex_);

  /* yaw */
  if(msg->yaw_nav_mode == aerial_robot_msgs::FlightNav::POS_MODE)
    {
      setTargetYaw(angles::normalize_angle(msg->target_yaw));
      setTargetOmegaZ(0);
    }
  if(msg->yaw_nav_mode == aerial_robot_msgs::FlightNav::VEL_MODE)
    {
      setTargetOmegaZ(msg->target_omega_z);

      teleop_reset_time_ = teleop_reset_duration_ + ros::Time::now().toSec();
    }
  if(msg->yaw_nav_mode == aerial_robot_msgs::FlightNav::POS_VEL_MODE)
    {
      setTargetYaw(angles::normalize_angle(msg->target_yaw));
      setTargetOmegaZ(msg->target_omega_z);
      trajectory_mode_ = true;
      trajectory_reset_time_ = trajectory_reset_duration_ + ros::Time::now().toSec();
    }

  /* xy control */
  switch(msg->pos_xy_nav_mode)
    {
    case aerial_robot_msgs::FlightNav::POS_MODE:
      {
        tf::Vector3 target_cog_pos(msg->target_pos_x, msg->target_pos_y, 0);
        if(msg->target == aerial_robot_msgs::FlightNav::BASELINK)
          {
            ROS_ERROR("[Nav] Only cog frame is available during assembled state!");
            return;

            /* check the transformation */
            tf::Transform cog2baselink_tf;
            tf::transformKDLToTF(robot_model_->getCog2Baselink<KDL::Frame>(), cog2baselink_tf);
            target_cog_pos -= tf::Matrix3x3(tf::createQuaternionFromYaw(getTargetRPY().z()))
              * cog2baselink_tf.getOrigin();
          }
        else if(msg->target == CONTACT_POINT)
          {
            /* check the transformation */
            tf::Transform cog2cp_tf;
            tf::transformKDLToTF(beetle_robot_model_->getCog2Cp<KDL::Frame>(), cog2cp_tf);
            target_cog_pos -= cog2cp_tf.getOrigin();
          }

        tf::Vector3 target_delta = getTargetPos() - target_cog_pos;
        target_delta.setZ(0);

        if(target_delta.length() > vel_nav_threshold_)
          {
            ROS_WARN_THROTTLE(1.0, "start vel nav control for waypoint");
            vel_based_waypoint_ = true;
            xy_control_mode_ = VEL_CONTROL_MODE;
          }

        if(!vel_based_waypoint_)
          xy_control_mode_ = POS_CONTROL_MODE;

        setTargetPosCandX(target_cog_pos.x());
        setTargetPosCandY(target_cog_pos.y());

        setTargetVelX(0);
        setTargetVelY(0);

        break;
      }
    case aerial_robot_msgs::FlightNav::VEL_MODE:
      {
        /* do not switch to pure vel mode */
        xy_control_mode_ = POS_CONTROL_MODE;

        teleop_reset_time_ = teleop_reset_duration_ + ros::Time::now().toSec();

        switch(msg->control_frame)
          {
          case WORLD_FRAME:
            {
              setTargetVelX(msg->target_vel_x);
              setTargetVelY(msg->target_vel_y);
              break;
            }
          case LOCAL_FRAME:
            {
              double yaw_angle = estimator_->getState(Frame::COG, estimate_mode_).z();
              tf::Vector3 target_vel = frameConversion(tf::Vector3(msg->target_vel_x, msg->target_vel_y, 0), yaw_angle);
              setTargetVelX(target_vel.x());
              setTargetVelY(target_vel.y());
              break;
            }
          default:
            {
              break;
            }
          }
        break;
      }
    case aerial_robot_msgs::FlightNav::POS_VEL_MODE:
      {
        xy_control_mode_ = POS_CONTROL_MODE;

        setTargetPosCandX(msg->target_pos_x);
        setTargetPosCandY(msg->target_pos_y);

        setTargetVelX(msg->target_vel_x);
        setTargetVelY(msg->target_vel_y);

        trajectory_mode_ = true;
        trajectory_reset_time_ = trajectory_reset_duration_ + ros::Time::now().toSec();

        break;
      }
    case aerial_robot_msgs::FlightNav::ACC_MODE:
      {
        /* should be in COG frame */
        xy_control_mode_ = ACC_CONTROL_MODE;
        prev_xy_control_mode_ = ACC_CONTROL_MODE;

        switch(msg->control_frame)
          {
          case WORLD_FRAME:
            {
              setTargetAcc(msg->target_acc_x, msg->target_acc_y, 0);
              break;
            }
          case LOCAL_FRAME:
            {
              tf::Vector3 target_acc = frameConversion(tf::Vector3(msg->target_acc_x, msg->target_acc_y, 0), estimator_->getState(Frame::COG, estimate_mode_)[0]);
              setTargetAccX(target_acc.x());
              setTargetAccY(target_acc.y());
              break;
            }
          default:
            {
              break;
            }
          }
        break;
      }
    case aerial_robot_msgs::FlightNav::STAY_HERE_MODE:
      {
        xy_control_mode_ = POS_CONTROL_MODE;
        setTargetVelX(0);
        setTargetVelY(0);
        setTargetXyFromCurrentState();
        break;
      }      
    case aerial_robot_msgs::FlightNav::GPS_WAYPOINT_MODE:
      {
        target_wp_ = geodesy::toMsg(msg->target_pos_x, msg->target_pos_y);
        gps_waypoint_ = true;

        break;
      }
    }
  if(msg->pos_xy_nav_mode != aerial_robot_msgs::FlightNav::ACC_MODE) setTargetZeroAcc();

  /* z */
  if(msg->pos_z_nav_mode == aerial_robot_msgs::FlightNav::VEL_MODE)
    {
      /* special */
      // addTargetPosZ(msg->target_pos_diff_z);
      // setTargetVelZ(0);
      // Support VEL control for Z axis in  assembly mode
      setTargetVelZ(msg->target_vel_z);
      teleop_reset_time_ = teleop_reset_duration_ + ros::Time::now().toSec();
    }
  else if(msg->pos_z_nav_mode == aerial_robot_msgs::FlightNav::POS_MODE)
    {
      tf::Vector3 target_cog_pos(0, 0, msg->target_pos_z);
      if(msg->target == aerial_robot_msgs::FlightNav::BASELINK)
        {
          ROS_ERROR("[Nav] Only CoG can be set as a target frame during assembled state!");
          return;

          /* check the transformation */
          tf::Transform cog2baselink_tf;
          tf::transformKDLToTF(robot_model_->getCog2Baselink<KDL::Frame>(), cog2baselink_tf);
          target_cog_pos -= cog2baselink_tf.getOrigin();
        }
      else if(msg->target == CONTACT_POINT)
        {
          /* check the transformation */
          tf::Transform cog2cp_tf;
          tf::transformKDLToTF(beetle_robot_model_->getCog2Cp<KDL::Frame>(), cog2cp_tf);
          target_cog_pos -= cog2cp_tf.getOrigin();
        }


      setTargetPosCandZ(target_cog_pos.z());

      setTargetVelZ(0);
    }
  else if(msg->pos_z_nav_mode == aerial_robot_msgs::FlightNav::POS_VEL_MODE)
    {

      setTargetPosCandZ(msg->target_pos_z);
      setTargetVelZ(msg->target_vel_z);
    }

  /* Physical baselink attitude. Match the unified-mode interpretation of
   * assembly FlightNav commands; target_rpy_ remains the virtual CoG attitude
   * target used by the independent module controllers. */
  tf::Vector3 final_target_baselink_rpy = getFinalTargetBaselinkRPY();
  bool update_final_target_baselink_rpy = false;
  if(msg->pitch_nav_mode == aerial_robot_msgs::FlightNav::POS_MODE)
    {
      final_target_baselink_rpy.setY(msg->target_pitch);
      update_final_target_baselink_rpy = true;
    }
  if(msg->roll_nav_mode == aerial_robot_msgs::FlightNav::POS_MODE)
    {
      final_target_baselink_rpy.setX(msg->target_roll);
      update_final_target_baselink_rpy = true;
    }
  if(update_final_target_baselink_rpy)
    setFinalTargetBaselinkRPY(final_target_baselink_rpy);
}


void BeetleNavigator::setAssemblyFinalTargetBaselinkRotCallback(const spinal::DesireCoordConstPtr & msg)
{
  if(getModuleState() != SEPARATED) GimbalrotorNavigator::setFinalTargetBaselinkRotCallback(msg);
}


void BeetleNavigator::assemblyFlagCallback(const diagnostic_msgs::KeyValue & msg)
{
  int module_id = std::stoi(msg.key);
  int assembly_flag = std::stoi(msg.value);
  setAssemblyFlag(module_id,assembly_flag);
}

void BeetleNavigator::update()
{
  rotateContactPointFrame();
  calcCenterOfMoving();
  
  // Publish assembly CoG odom
  calculateAndPublishAssemblyCoGOdom();
  
  GimbalrotorNavigator::update();
  setControlFlag((getNaviState() == HOVER_STATE || getNaviState() == TAKEOFF_STATE || getNaviState() == LAND_STATE) ? true : false);
  convertTargetPosFromCoG2CoM();
  land_height_ = getInitHeight();
}

void BeetleNavigator::rotateContactPointFrame()
{
  geometry_msgs::TransformStamped tf = beetle_robot_model_-> getContactFrame<geometry_msgs::TransformStamped>();
  tf.header.stamp = ros::Time::now();
  tf.header.frame_id = tf::resolve(std::string(nh_.getNamespace()), beetle_robot_model_->getRootFrameName());
  tf.child_frame_id = tf::resolve(std::string(nh_.getNamespace()), std::string("contact_point"));
  br_.sendTransform(tf); 
}

void BeetleNavigator::calcCenterOfMoving()
{
  std::string cog_name = my_name_ + std::to_string(my_id_) + "/cog";
  const bool use_module_model_masses = getUnifiedControlMode();
  std::map<int, bool> assembly_flags_snapshot;
  bool control_flag_snapshot = false;
  bool leader_fix_flag_snapshot = false;
  int leader_id_snapshot = leader_id_;
  int module_state_snapshot = module_state_;
  int pre_assembled_modules_snapshot = pre_assembled_modules_;
  {
    std::lock_guard<std::mutex> lock(mutex_assembly_state_);
    assembly_flags_snapshot = assembly_flags_;
    control_flag_snapshot = control_flag_;
    leader_fix_flag_snapshot = leader_fix_flag_;
    leader_id_snapshot = leader_id_;
    module_state_snapshot = module_state_;
    pre_assembled_modules_snapshot = pre_assembled_modules_;
  }
  // Mass-weighted center: matches BeetleUnifiedController::updateFormationGeometry().
  // Without weighting the navigator's Cog2CoM_ drifts ~5 mm from the controller's
  // formation_cog_offset_ under asymmetric per-module masses, opening a residual
  // bias in assemblyNavCallback's CoG↔CoM target conversion.
  Eigen::Vector3f center_of_moving = Eigen::Vector3f::Zero();
  int assembled_module = 0;
  bool all_module_masses_ready = true;
  std::string missing_module_model_ids;
  std::vector<std::pair<int, Eigen::Vector3f>> module_offsets;
  std::vector<int> assembled_modules_ids;
  geometry_msgs::Point cog_com_dist_msg;
  for(const auto & item : assembly_flags_snapshot){
    geometry_msgs::TransformStamped transformStamped;
    int id = item.first;
    bool value = item.second;
    if(!value) continue;
    try
      {
        transformStamped = tfBuffer_.lookupTransform(cog_name, my_name_ + std::to_string(id) + std::string("/cog") , ros::Time(0));
        auto& trans = transformStamped.transform.translation;
        Eigen::Vector3f module_root(trans.x,trans.y,trans.z);
        if (use_module_model_masses) {
          std::lock_guard<std::mutex> lock(mutex_module_masses_);
          auto it = module_masses_.find(id);
          if (it == module_masses_.end()) {
            all_module_masses_ready = false;
            if (!missing_module_model_ids.empty()) missing_module_model_ids += ",";
            missing_module_model_ids += std::to_string(id);
          }
        }
        module_offsets.push_back(std::make_pair(id, module_root));
        assembled_module ++;
        assembled_modules_ids.push_back(id);
      }
    catch (tf2::TransformException& ex)
      {
        ROS_ERROR_STREAM("not exist module is mentioned. ID is "<<id );
        return;
      }
  }
  double total_mass = 0.0;
  if (use_module_model_masses && !all_module_masses_ready) {
    ROS_WARN_THROTTLE(1.0,
                      "[UnifiedNav id=%d] ModuleModel masses incomplete missing=[%s]; using equal weights for Cog2CoM",
                      my_id_, missing_module_model_ids.c_str());
  }
  for (const auto& module : module_offsets) {
    double m_i = 1.0;
    if (use_module_model_masses && all_module_masses_ready) {
      std::lock_guard<std::mutex> lock(mutex_module_masses_);
      m_i = module_masses_[module.first];
    }
    center_of_moving += static_cast<float>(m_i) * module.second;
    total_mass += m_i;
  }
  const auto my_flag = assembly_flags_snapshot.find(my_id_);
  const bool my_assembled =
      my_flag != assembly_flags_snapshot.end() && my_flag->second;
  if(!assembled_module || assembled_module == 1 || !my_assembled){
    KDL::Frame com_frame;
    setCog2CoM(com_frame);
    {
      std::lock_guard<std::mutex> lock(mutex_assembly_state_);
      assembled_modules_ids_ = assembled_modules_ids;
      module_num_ = assembled_module;
      pre_assembled_modules_ = assembled_module;
      current_assembled_ = false;
      module_state_ = SEPARATED;
    }
    const KDL::Frame cog2com = getCog2CoM<KDL::Frame>();
    cog_com_dist_msg.x = cog2com.p.x();
    cog_com_dist_msg.y = cog2com.p.y();
    cog_com_dist_msg.z = cog2com.p.z();
    cog_com_dist_pub_.publish(cog_com_dist_msg);
    return;
  }

  //define a module closest to the center as leader
  std::sort(assembled_modules_ids.begin(), assembled_modules_ids.end());
  int leader_index = std::round((assembled_modules_ids.size())/2.0) -1;
  int leader_id = leader_id_snapshot;
  if(!leader_fix_flag_snapshot) leader_id = assembled_modules_ids[leader_index];
  // Expose leader_id to rosparam so Python scripts can route wrench to the correct module
  nh_.setParam("assembly_leader_id", leader_id);
  int module_state = module_state_snapshot;
  if(my_id_ == leader_id && control_flag_snapshot){
    module_state = LEADER;
  }else if(control_flag_snapshot){
    module_state = FOLLOWER;
  }

  //define a module on the right edge as leader
  // std::sort(assembled_modules_ids_.begin(), assembled_modules_ids_.end());
  // int leader_index = std::round((assembled_modules_ids_.size())) -1;
  // if(!leader_fix_flag_) leader_id_ = assembled_modules_ids_[leader_index];
  // if(my_id_ == leader_id_ && control_flag_){
  //   module_state_ = LEADER;
  // }else if(control_flag_){
  //   module_state_ = FOLLOWER;
  // }

  center_of_moving = (total_mass > 0.0) ? center_of_moving / static_cast<float>(total_mass)
                                        : center_of_moving / assembled_module;

  geometry_msgs::TransformStamped tf;
  tf.header.stamp = ros::Time::now();
  tf.header.frame_id = cog_name;
  tf.child_frame_id = my_name_ + std::to_string(my_id_)+"/center_of_moving";
  tf.transform.translation.x = center_of_moving.x();
  tf.transform.translation.y = center_of_moving.y();
  tf.transform.translation.z = center_of_moving.z();
  tf.transform.rotation.x = 0;
  tf.transform.rotation.y = 0;
  tf.transform.rotation.z = 0;
  tf.transform.rotation.w = 1;
  br_.sendTransform(tf);

  //update com-cog distance only during hovering
  bool reconfig_flag = false;
  if(control_flag_snapshot){
    Eigen::Vector3f cog_com_dist(center_of_moving.norm() * center_of_moving.x()/fabs(center_of_moving.x()),0,0);
    // ROS_INFO_STREAM("cog_com_dist is " << cog_com_dist.transpose());
    tf.transform.translation.x = cog_com_dist.x();
    tf.transform.translation.y = cog_com_dist.y();
    tf.transform.translation.z = cog_com_dist.z();
    setCog2CoM(tf2::transformToKDL(tf));
    reconfig_flag =  (pre_assembled_modules_snapshot != assembled_module) ? true : false;
    if(reconfig_flag){
      Eigen::VectorXi id_vector = Eigen::Map<Eigen::VectorXi>(assembled_modules_ids.data(), assembled_modules_ids.size());
      for(const auto & item : assembly_flags_snapshot){
        if(item.second)
          {
            std::cout << "id: " << item.first << " -> assembled"<< std::endl;
          } else {
          std::cout << "id: " << item.first << " -> separated"<< std::endl;
        }
      }
      ROS_INFO_STREAM(id_vector);
      ROS_INFO_STREAM("Leader's ID is " <<leader_id);
    }
  }
  const KDL::Frame cog2com = getCog2CoM<KDL::Frame>();
  cog_com_dist_msg.x = cog2com.p.x();
  cog_com_dist_msg.y = cog2com.p.y();
  cog_com_dist_msg.z = cog2com.p.z();
  cog_com_dist_pub_.publish(cog_com_dist_msg);
  {
    std::lock_guard<std::mutex> lock(mutex_assembly_state_);
    assembled_modules_ids_ = assembled_modules_ids;
    module_num_ = assembled_module;
    leader_id_ = leader_id;
    module_state_ = module_state;
    reconfig_flag_ = reconfig_flag;
    if(reconfig_flag) pre_assembled_modules_ = assembled_module;
    if(control_flag_snapshot) current_assembled_ = true;
  }
}


void BeetleNavigator::convertTargetPosFromCoG2CoM()
{
  // In unified control mode, the controller manages target positions directly
  // in formation CoG frame. The old CoG→CoM conversion is specific to the
  // leader-follower architecture and must be bypassed to avoid corrupting
  // the formation-level position reference.
  const bool unified_control_mode = getUnifiedControlMode();
  if (unified_control_mode) return;

  //TODO: considering correct rotaion axis

  tf::Transform cog2com_tf;
  tf::transformKDLToTF(getCog2CoM<KDL::Frame>(), cog2com_tf);
  tf::Matrix3x3 cog_orientation_tf;
  tf::matrixEigenToTF(beetle_robot_model_->getCogDesireOrientation<Eigen::Matrix3d>(),cog_orientation_tf);
  tf::Vector3 com_conversion = cog_orientation_tf *  tf::Matrix3x3(tf::createQuaternionFromYaw(getTargetRPY().z())) * cog2com_tf.getOrigin();

  // [EXIT_DIAG] Log CoG→CoM conversion details after exiting unified mode
  // This helps trace whether com_conversion is causing target position drift
  if (was_unified_for_diag_ && !unified_control_mode) {
    cog2com_diag_count_ = 0;  // start logging on transition
  }
  was_unified_for_diag_ = unified_control_mode;
  if (cog2com_diag_count_ >= 0 && cog2com_diag_count_ < 80) {
    if (cog2com_diag_count_ < 10 || cog2com_diag_count_ % 10 == 0) {
      double cdo_r, cdo_p, cdo_y;
      tf::Matrix3x3 cdo_mat;
      tf::matrixEigenToTF(beetle_robot_model_->getCogDesireOrientation<Eigen::Matrix3d>(), cdo_mat);
      cdo_mat.getRPY(cdo_r, cdo_p, cdo_y);
      ROS_WARN("[EXIT_DIAG_COG2COM] id=%d f=%d com_conv=(%.4f,%.4f,%.4f) "
               "cog2com_origin=(%.4f,%.4f,%.4f) CogDesOrient=(%.4f,%.4f,%.4f) "
               "tgt_pos=(%.4f,%.4f,%.4f) tgt_cand=(%.4f,%.4f,%.4f) pre_tgt=(%.4f,%.4f,%.4f)",
               my_id_, cog2com_diag_count_,
               com_conversion.x(), com_conversion.y(), com_conversion.z(),
               cog2com_tf.getOrigin().x(), cog2com_tf.getOrigin().y(), cog2com_tf.getOrigin().z(),
               cdo_r, cdo_p, cdo_y,
               getTargetPos().x(), getTargetPos().y(), getTargetPos().z(),
               getTargetPosCand().x(), getTargetPosCand().y(), getTargetPosCand().z(),
               pre_target_pos_.x(), pre_target_pos_.y(), pre_target_pos_.z());
    }
    cog2com_diag_count_++;
  }

  bool current_assembled = getCurrentAssembled();
  bool reconfig_flag = getReconfigFlag();

  KDL::Frame empty_frame;

  if(pre_assembled_  && !current_assembled){ //disassembly process
    setTargetPosCandX(getTargetPos().x());
    setTargetPosCandY(getTargetPos().y());
    setTargetPosCandZ(getTargetPos().z());
    ROS_INFO("switched");
    pre_assembled_ = current_assembled;
  } else if((!pre_assembled_  && current_assembled) || (current_assembled && reconfig_flag)){ //assembly or reconfig process
    int my_id = getMyID();
    tf::Vector3 pos_cog = estimator_->getPos(Frame::COG, estimate_mode_);
    tf::Vector3 orientation_err = getTargetRPY() - estimator_ ->getEuler(Frame::COG, estimate_mode_);
    ROS_INFO_STREAM("ID: " << my_id << "'s orientation_err is "<< "(" << orientation_err.x() << ", " << orientation_err.y() << ", " << orientation_err.z() << ")");
    tf::Matrix3x3 att_err_mat = tf::Matrix3x3(tf::createQuaternionFromRPY(orientation_err.x(), orientation_err.y(),orientation_err.z()));
    tf::Vector3 corrected_target_pos =  tf::Matrix3x3(tf::createQuaternionFromRPY(orientation_err.x(), orientation_err.y(),orientation_err.z())) * pos_cog;
    if(getNaviState() == HOVER_STATE){
      setTargetPosCandX(pos_cog.x() + (att_err_mat.inverse() * com_conversion).x());
      setTargetPosCandY(pos_cog.y() + (att_err_mat.inverse() * com_conversion).y());
      setTargetPosCandZ(pos_cog.z() + (att_err_mat.inverse() * com_conversion).z());
    }
    ROS_INFO("switched");
    pre_assembled_ = current_assembled;
  }else if(getCog2CoM<KDL::Frame>() == empty_frame && getNaviState() != HOVER_STATE){
    return;
  }


  /* Check whether the target value was changed by someway other than uav nav */
  /* Target pos candidate represents a target pos in a assembly frame */
  if( int(pre_target_pos_.x() * 1000) != int(getTargetPos().x() * 1000)){
    float target_x_com = getTargetPos().x() + com_conversion.x();
    setTargetPosCandX(target_x_com);
  }

  if( int(pre_target_pos_.y() * 1000) != int(getTargetPos().y() * 1000)){
    float target_y_com = getTargetPos().y() + com_conversion.y();
    setTargetPosCandY(target_y_com);
  }

  if( int(pre_target_pos_.z() * 1000) != int(getTargetPos().z() * 1000)){
    float target_z_com = getTargetPos().z() + com_conversion.z();
    setTargetPosCandZ(target_z_com);
  }
  
  tf::Vector3 target_cog_pos = getTargetPosCand();
  target_cog_pos -=  com_conversion;

  if( getNaviState() == HOVER_STATE ||
      getNaviState() == TAKEOFF_STATE){
    setTargetPosX(target_cog_pos.x());
    setTargetPosY(target_cog_pos.y());
    setTargetPosZ(target_cog_pos.z());
  }

  pre_target_pos_.setX(target_cog_pos.x());
  pre_target_pos_.setY(target_cog_pos.y());
  pre_target_pos_.setZ(target_cog_pos.z());
}

void BeetleNavigator::rosParamInit()
{
  ros::NodeHandle nh(nh_, "navigation");
  getParam<double>(nh, "max_target_roll_pitch_rate", max_target_roll_pitch_rate_, 0.0);
  getParam<int>(nh, "max_modules_num", max_modules_num_, 8);
  GimbalrotorNavigator::rosParamInit();

  nh_.getParam("robot_id", my_id_);
  nh_.getParam("aerial_robot_base_node/tf_prefix", my_name_);
  my_name_.pop_back(); // extract common robot name
}

// Assembly CoG odom calculation and publishing
void BeetleNavigator::calculateAndPublishAssemblyCoGOdom()
{
  if(!publish_assembly_odom_) return;
  if(getModuleState() == SEPARATED) return;  // Not assembled
  
  std::vector<int> assembled_ids = getAssemblyIds();
  if(assembled_ids.empty()) return;
  
  // Calculate CoG position (weighted average)
  Eigen::Vector3d cog_pos = calculateAssemblyCoGPosition();
  
  // Calculate CoG orientation (quaternion average)
  Eigen::Quaterniond cog_quat = calculateAssemblyCoGOrientation();
  
  // Fill Odometry message
  assembly_cog_odom_.header.stamp = ros::Time::now();
  assembly_cog_odom_.header.frame_id = "world";
  assembly_cog_odom_.child_frame_id = "assembly_cog";
  
  assembly_cog_odom_.pose.pose.position.x = cog_pos.x();
  assembly_cog_odom_.pose.pose.position.y = cog_pos.y();
  assembly_cog_odom_.pose.pose.position.z = cog_pos.z();
  
  assembly_cog_odom_.pose.pose.orientation.x = cog_quat.x();
  assembly_cog_odom_.pose.pose.orientation.y = cog_quat.y();
  assembly_cog_odom_.pose.pose.orientation.z = cog_quat.z();
  assembly_cog_odom_.pose.pose.orientation.w = cog_quat.w();
  
  // Publish
  assembly_cog_odom_pub_.publish(assembly_cog_odom_);
}

Eigen::Vector3d BeetleNavigator::calculateAssemblyCoGPosition()
{
  std::vector<int> assembled_ids = getAssemblyIds();
  Eigen::Vector3d cog_pos = Eigen::Vector3d::Zero();
  double total_mass = 0.0;
  double module_mass = beetle_robot_model_->getMass();
  
  for(int id : assembled_ids) {
    geometry_msgs::TransformStamped transform;
    try {
      std::string frame_name = my_name_ + std::to_string(id) + "/cog";
      transform = tfBuffer_.lookupTransform("world", frame_name, ros::Time(0));
      
      Eigen::Vector3d module_pos(
        transform.transform.translation.x,
        transform.transform.translation.y,
        transform.transform.translation.z
      );
      
      cog_pos += module_pos * module_mass;
      total_mass += module_mass;
      
    } catch(tf2::TransformException& ex) {
      ROS_WARN_THROTTLE(5.0, "[BeetleNavigator] Failed to get transform for %s%d: %s", 
                        my_name_.c_str(), id, ex.what());
    }
  }
  
  if(total_mass > 0) {
    cog_pos /= total_mass;
  }
  
  return cog_pos;
}

Eigen::Quaterniond BeetleNavigator::calculateAssemblyCoGOrientation()
{
  std::vector<int> assembled_ids = getAssemblyIds();
  
  // Simple quaternion averaging (suitable for small attitude differences <30°)
  Eigen::Vector4d quat_sum = Eigen::Vector4d::Zero();
  int count = 0;
  
  for(int id : assembled_ids) {
    try {
      std::string frame_name = my_name_ + std::to_string(id) + "/cog";
      geometry_msgs::TransformStamped transform = 
        tfBuffer_.lookupTransform("world", frame_name, ros::Time(0));
      
      quat_sum.x() += transform.transform.rotation.x;
      quat_sum.y() += transform.transform.rotation.y;
      quat_sum.z() += transform.transform.rotation.z;
      quat_sum.w() += transform.transform.rotation.w;
      count++;
      
    } catch(tf2::TransformException& ex) {
      ROS_WARN_THROTTLE(5.0, "[BeetleNavigator] Failed to get rotation for %s%d: %s", 
                        my_name_.c_str(), id, ex.what());
    }
  }
  
  if(count > 0) {
    quat_sum /= count;
    quat_sum.normalize();
  } else {
    quat_sum = Eigen::Vector4d(0, 0, 0, 1);  // Default no rotation
  }
  
  return Eigen::Quaterniond(quat_sum.w(), quat_sum.x(), quat_sum.y(), quat_sum.z());
}


/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::BeetleNavigator, aerial_robot_navigation::BaseNavigator);
