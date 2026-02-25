#include <beetle/control/beetle_controller.h>
#include <sstream>

using namespace std;

namespace aerial_robot_control
{
  BeetleController::BeetleController():
    GimbalrotorController(),
    pd_wrench_comp_mode_(false),
    pre_module_state_(SEPARATED),
    des_wrench_pub_flag_(false),
    desired_external_wrench_(Eigen::VectorXd::Zero(6)),
    unified_control_mode_(false),
    prev_unified_control_mode_(false),
    unified_cmd_received_(false),
    follower_unified_active_(false),
    spinal_gains_zeroed_(false)
  {
  }

  void BeetleController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                    double ctrl_loop_rate
                                    )
  {
    GimbalrotorController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);
    wrench_pid_msg_.x.total.resize(1);
    wrench_pid_msg_.x.p_term.resize(1);
    wrench_pid_msg_.x.i_term.resize(1);
    wrench_pid_msg_.x.d_term.resize(1);
    wrench_pid_msg_.y.total.resize(1);
    wrench_pid_msg_.y.p_term.resize(1);
    wrench_pid_msg_.y.i_term.resize(1);
    wrench_pid_msg_.y.d_term.resize(1);
    wrench_pid_msg_.z.total.resize(1);
    wrench_pid_msg_.z.p_term.resize(1);
    wrench_pid_msg_.z.i_term.resize(1);
    wrench_pid_msg_.z.d_term.resize(1);
    wrench_pid_msg_.roll.total.resize(1);
    wrench_pid_msg_.roll.p_term.resize(1);
    wrench_pid_msg_.roll.i_term.resize(1);
    wrench_pid_msg_.roll.d_term.resize(1);
    wrench_pid_msg_.pitch.total.resize(1);
    wrench_pid_msg_.pitch.p_term.resize(1);
    wrench_pid_msg_.pitch.i_term.resize(1);
    wrench_pid_msg_.pitch.d_term.resize(1);
    wrench_pid_msg_.yaw.total.resize(1);
    wrench_pid_msg_.yaw.p_term.resize(1);
    wrench_pid_msg_.yaw.i_term.resize(1);
    wrench_pid_msg_.yaw.d_term.resize(1);

    beetle_robot_model_ = boost::dynamic_pointer_cast<BeetleRobotModel>(robot_model);
    beetle_navigator_ = boost::dynamic_pointer_cast<aerial_robot_navigation::BeetleNavigator>(navigator);
    external_wrench_lower_limit_ = Eigen::VectorXd::Zero(6);
    external_wrench_upper_limit_ = Eigen::VectorXd::Zero(6);
    rosParamInit();
    if(pd_wrench_comp_mode_) ROS_ERROR("PD & Wrench comp mode");
    external_wrench_compensation_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("external_wrench_compensation", 1);
    tagged_external_wrench_pub_ = nh_.advertise<beetle::TaggedWrench>("tagged_wrench", 1);
    whole_external_wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("whole_wrench", 1);
    internal_wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("internal_wrench", 1);
    wrench_comp_pid_pub_ = nh_.advertise<aerial_robot_msgs::PoseControlPid>("debug/wrench_comp/pid", 1);
    des_inter_wrench_pub_ = nh_.advertise<beetle::TaggedWrenches>("des_inter_wnrech", 1);
    desired_ext_wrench_sub_ = nh_.subscribe("desired_external_wrench", 1, &BeetleController::desiredExternalWrenchCallback, this);
    int max_modules_num = beetle_navigator_->getMaxModuleNum();
    for(int i = 0; i < max_modules_num; i++){
      std::string module_name  = string("/") + beetle_navigator_->getMyName() + std::to_string(i+1);
      est_wrench_subs_.insert(make_pair(module_name, nh_.subscribe( module_name + string("/tagged_wrench"), 1, &BeetleController::estExternalWrenchCallback, this)));
      Eigen::VectorXd wrench = Eigen::VectorXd::Zero(6);
      est_wrench_list_.insert(make_pair(i+1, wrench));
      inter_wrench_list_.insert(make_pair(i+1, wrench));
      wrench_comp_list_.insert(make_pair(i+1, wrench));
      ff_inter_wrench_list_.insert(make_pair(i+1, wrench));
      ff_inter_wrench_subs_.insert(make_pair(module_name, nh_.subscribe( module_name + string("/ff_inter_wrench"), 1, &BeetleController::ffInterWrenchCallback, this)));
      ff_inter_wrench_pubs_[i+1] = nh_.advertise<beetle::TaggedWrench>(module_name + string("/ff_inter_wrench"), 1);
      desired_ext_wrench_pubs_[i+1] = nh_.advertise<geometry_msgs::WrenchStamped>(module_name + string("/desired_external_wrench"), 1);
    }
    pid_controllers_.push_back(PID("f_x", wrench_comp_p_gain_, wrench_comp_i_gain_, wrench_comp_d_gain_));
    pid_controllers_.push_back(PID("f_y", wrench_comp_p_gain_, wrench_comp_i_gain_, wrench_comp_d_gain_));
    pid_controllers_.push_back(PID("f_z", wrench_comp_p_gain_, wrench_comp_i_gain_, wrench_comp_d_gain_));
    pid_controllers_.push_back(PID("t_x", wrench_comp_p_gain_, wrench_comp_i_gain_, wrench_comp_d_gain_));
    pid_controllers_.push_back(PID("t_y", wrench_comp_p_gain_, wrench_comp_i_gain_, wrench_comp_d_gain_));
    pid_controllers_.push_back(PID("t_z", wrench_comp_p_gain_, wrench_comp_i_gain_, wrench_comp_d_gain_));

    ros::NodeHandle control_nh(nh_, "controller");
    ros::NodeHandle wrench_nh(control_nh, "wrench_comp");
    std::vector<int> indices = {FX, FY, FZ, TX, TY, TZ};
    pid_reconf_servers_.push_back(boost::make_shared<PidControlDynamicConfig>(wrench_nh));
    pid_reconf_servers_.back()->setCallback(boost::bind(&BeetleController::cfgPidCallback, this, _1, _2, indices));

    prev_comp_update_time_ = -1;

    // Initialize unified controller (unified_control_mode_ is read by rosParamInit)
    unified_controller_ = std::make_shared<BeetleUnifiedController>();
    unified_controller_->initialize(nh_, beetle_robot_model_, beetle_navigator_, estimator_);

    // FOLLOWER: subscribe to unified commands from LEADER
    // Topic names match what BeetleUnifiedController::publishCommands() publishes
    std::string my_ns = std::string("/") + beetle_navigator_->getMyName()
                        + std::to_string(beetle_navigator_->getMyID());
    unified_thrust_sub_ = nh_.subscribe(my_ns + "/unified_thrust_cmd", 1,
                                        &BeetleController::unifiedThrustCallback, this);
    unified_gimbal_sub_ = nh_.subscribe(my_ns + "/unified_gimbal_cmd", 1,
                                        &BeetleController::unifiedGimbalCallback, this);
    // Publishers to this module's own spinal (same topic names as GimbalrotorController)
    follower_thrust_pub_ = nh_.advertise<spinal::FourAxisCommand>("four_axes/command", 1);
    follower_gimbal_pub_ = nh_.advertise<sensor_msgs::JointState>("gimbals_ctrl", 1);
  }

  void BeetleController::controlCore()
  {
    std::map<int, bool> assembly_flag = beetle_navigator_->getAssemblyFlags();
    int max_modules_num = beetle_navigator_->getMaxModuleNum();
    int module_state = beetle_navigator_-> getModuleState();
    bool comp_update_flag = false;
    double comp_update_interval = 1  / comp_term_update_freq_;
    
    // Check unified control mode from rosparam (allows runtime toggle)
    ros::NodeHandle control_nh(nh_, "controller");
    control_nh.getParam("unified_control_mode", unified_control_mode_);
    
    // ======== Unified Control Mode ========
    // Only LEADER runs the unified allocation for the entire formation.
    // In this mode, LEADER does NOT call GimbalrotorController::controlCore().
    // Instead it uses BeetleUnifiedController to allocate across all N*4 rotors.
    if (unified_control_mode_ && module_state == LEADER && module_state != SEPARATED) {
      // Detect mode switch: reset pitch/roll I terms and disable spinal's internal PID
      if (pre_module_state_ != LEADER || !prev_unified_control_mode_) {
        pid_controllers_.at(ROLL).setErrI(0);
        pid_controllers_.at(PITCH).setErrI(0);
        // H2: Disable spinal's internal attitude PID by sending zero gains
        sendZeroAttitudeGains();
        spinal_gains_zeroed_ = true;
        ROS_INFO("[UnifiedCtrl] LEADER mode switch: reset pitch/roll I, zeroed spinal rpy/gain");
      }
      prev_unified_control_mode_ = true;

      // --- 方案γ: Run PID using formation CoG position instead of LEADER CoG ---
      // PoseLinearController::controlCore() measures LEADER CoG, but allocation
      // controls formation CoG. We manually run the PID with corrected position.

      // 1. Get LEADER state (same as PoseLinearController::controlCore())
      pos_ = estimator_->getPos(Frame::COG, estimate_mode_);
      vel_ = estimator_->getVel(Frame::COG, estimate_mode_);
      target_pos_ = navigator_->getTargetPos();
      target_vel_ = navigator_->getTargetVel();
      target_acc_ = navigator_->getTargetAcc();

      tf::Quaternion cog2baselink_rot;
      tf::quaternionKDLToTF(robot_model_->getCogDesireOrientation<KDL::Rotation>(), cog2baselink_rot);
      tf::Matrix3x3 cog_rot = estimator_->getOrientation(Frame::BASELINK, estimate_mode_) * tf::Matrix3x3(cog2baselink_rot).inverse();
      double r, p, y_angle; cog_rot.getRPY(r, p, y_angle);
      rpy_.setValue(r, p, y_angle);

      omega_ = estimator_->getAngularVel(Frame::COG, estimate_mode_);
      target_rpy_ = navigator_->getTargetRPY();
      tf::Matrix3x3 target_rot; target_rot.setRPY(target_rpy_.x(), target_rpy_.y(), target_rpy_.z());
      tf::Vector3 target_omega = navigator_->getTargetOmega();
      target_omega_ = cog_rot.inverse() * target_rot * target_omega;
      target_ang_acc_ = navigator_->getTargetAngAcc();

      // 2. Compute formation CoG position in world frame
      //    formation_cog_offset is in LEADER body frame (from TF lookups).
      //    Transform to world frame: formation_cog_world = leader_pos + R * offset
      const Eigen::Vector3d& cog_offset = unified_controller_->getFormationCogOffset();
      tf::Vector3 offset_body(cog_offset.x(), cog_offset.y(), cog_offset.z());
      tf::Vector3 offset_world = cog_rot * offset_body;
      tf::Vector3 formation_pos = pos_ + offset_world;
      tf::Vector3 formation_vel = vel_;  // ω×r term is small at near-hover, use leader vel as approx

      // target for formation CoG: leader's target + R_target * offset (at target orientation)
      tf::Vector3 target_formation_pos = target_pos_ + target_rot * offset_body;

      // 3. Run X/Y/Z PID with formation CoG position
      double du = ros::Time::now().toSec() - control_timestamp_;

      switch(navigator_->getXyControlMode())
        {
        case aerial_robot_navigation::POS_CONTROL_MODE:
          pid_controllers_.at(X).update(target_formation_pos.x() - formation_pos.x(), du,
                                        target_vel_.x() - formation_vel.x(), target_acc_.x());
          pid_controllers_.at(Y).update(target_formation_pos.y() - formation_pos.y(), du,
                                        target_vel_.y() - formation_vel.y(), target_acc_.y());
          break;
        case aerial_robot_navigation::VEL_CONTROL_MODE:
          pid_controllers_.at(X).update(0, du, target_vel_.x() - formation_vel.x(), target_acc_.x());
          pid_controllers_.at(Y).update(0, du, target_vel_.y() - formation_vel.y(), target_acc_.y());
          break;
        case aerial_robot_navigation::ACC_CONTROL_MODE:
          pid_controllers_.at(X).update(0, du, 0, target_acc_.x());
          pid_controllers_.at(Y).update(0, du, 0, target_acc_.y());
          break;
        default:
          break;
        }

      if(navigator_->getForceLandingFlag()) {
        pid_controllers_.at(X).reset();
        pid_controllers_.at(Y).reset();
      }

      // Z PID: also use formation CoG z
      double err_z = target_formation_pos.z() - formation_pos.z();
      double err_v_z = target_vel_.z() - formation_vel.z();
      double z_p_limit = pid_controllers_.at(Z).getLimitP();  // save before force landing zeroes it
      if(navigator_->getForceLandingFlag()) {
        pid_controllers_.at(Z).setLimitP(0);
        err_z = force_landing_descending_rate_;
        err_v_z = 0;
        target_acc_.setZ(0);
      }
      pid_controllers_.at(Z).update(err_z, du, err_v_z, target_acc_.z());
      if(pid_controllers_.at(Z).getErrI() < 0) pid_controllers_.at(Z).setErrI(0);
      if(navigator_->getForceLandingFlag()) {
        pid_controllers_.at(Z).setLimitP(z_p_limit);  // revert z p limit
        pid_controllers_.at(Z).setErrP(0);
      }

      // Roll/Pitch/Yaw PID (orientation is same for rigid assembly)
      double du_rp = du;
      if(!start_rp_integration_) du_rp = 0;
      pid_controllers_.at(ROLL).update(target_rpy_.x() - rpy_.x(), du_rp,
                                       target_omega_.x() - omega_.x(), target_ang_acc_.x());
      pid_controllers_.at(PITCH).update(target_rpy_.y() - rpy_.y(), du_rp,
                                        target_omega_.y() - omega_.y(), target_ang_acc_.y());
      double err_yaw = angles::shortest_angular_distance(rpy_.z(), target_rpy_.z());
      double err_omega_z = target_omega_.z() - omega_.z();
      if(!need_yaw_d_control_) err_omega_z = target_omega_.z();
      pid_controllers_.at(YAW).update(err_yaw, du, err_omega_z, target_ang_acc_.z());

      control_timestamp_ = ros::Time::now().toSec();

      // Build target wrench in acc space from PID outputs
      tf::Matrix3x3 uav_rot = estimator_->getOrientation(Frame::COG, estimate_mode_);
      tf::Vector3 target_acc_w(pid_controllers_.at(X).result(),
                               pid_controllers_.at(Y).result(),
                               pid_controllers_.at(Z).result());
      tf::Vector3 target_acc_cog = uav_rot.inverse() * target_acc_w;

      Eigen::VectorXd target_wrench_acc = Eigen::VectorXd::Zero(6);
      target_wrench_acc.head(3) = Eigen::Vector3d(target_acc_cog.x(), target_acc_cog.y(), target_acc_cog.z());
      target_wrench_acc(3) = pid_controllers_.at(ROLL).result();
      target_wrench_acc(4) = pid_controllers_.at(PITCH).result();
      target_wrench_acc(5) = pid_controllers_.at(YAW).result();

      // --- 方案δ: Add gyro compensation (ω × I·ω) ---
      {
        const Eigen::Matrix3d& I_form = unified_controller_->getFormationInertia();
        Eigen::Vector3d omega_eigen;
        tf::vectorTFToEigen(omega_, omega_eigen);
        Eigen::Vector3d gyro = omega_eigen.cross(I_form * omega_eigen);
        target_wrench_acc.tail(3) += gyro;
      }

      // Store for external wrench estimator (externalWrenchEstimate uses this)
      setTargetWrenchAccCog(target_wrench_acc);

      // Run unified allocation
      bool ok = unified_controller_->computeUnifiedAllocation(target_wrench_acc, desired_external_wrench_);
      if (ok) {
        // Publish commands to all FOLLOWERs via /beetle{id}/unified_thrust_cmd topics
        unified_controller_->publishCommands();

        // LEADER must also send its own share to its own spinal.
        // publishCommands() only writes to /beetle{id}/unified_thrust_cmd + gimbal_cmd,
        // which FOLLOWERs forward to their four_axes/command + gimbals_ctrl.
        // But LEADER's controlCore doesn't enter the FOLLOWER branch,
        // so we forward LEADER's own command here directly.
        int my_id = beetle_navigator_->getMyID();
        const auto& cmds = unified_controller_->getModuleCommands();
        auto it = cmds.find(my_id);
        if (it != cmds.end()) {
          // Send scalar thrusts (size 4) to own spinal
          spinal::FourAxisCommand my_thrust_msg;
          my_thrust_msg.base_thrust = it->second.full_thrusts;
          my_thrust_msg.angles[0] = 0;
          my_thrust_msg.angles[1] = 0;
          my_thrust_msg.angles[2] = 0;
          follower_thrust_pub_.publish(my_thrust_msg);

          // Send gimbal angles to own spinal
          sensor_msgs::JointState my_gimbal_msg;
          my_gimbal_msg.header.stamp = ros::Time::now();
          my_gimbal_msg.position = it->second.gimbal_angles;
          follower_gimbal_pub_.publish(my_gimbal_msg);
        }
      }

      ROS_INFO_THROTTLE(1.0, "[UnifiedCtrl LEADER] wrench_acc=(%.3f,%.3f,%.3f,%.3f,%.3f,%.3f), ext=(%.2f,%.2f,%.2f)",
                        target_wrench_acc(0), target_wrench_acc(1), target_wrench_acc(2),
                        target_wrench_acc(3), target_wrench_acc(4), target_wrench_acc(5),
                        desired_external_wrench_(0), desired_external_wrench_(1), desired_external_wrench_(2));

      // ===== DIAGNOSTIC: PID breakdown =====
      ROS_INFO_THROTTLE(1.0, "[UnifiedCtrl LEADER DIAG] PID_Z: P=%.4f I=%.4f D=%.4f total=%.4f",
                        pid_controllers_.at(Z).getPTerm(), pid_controllers_.at(Z).getITerm(),
                        pid_controllers_.at(Z).getDTerm(), pid_controllers_.at(Z).result());
      ROS_INFO_THROTTLE(1.0, "[UnifiedCtrl LEADER DIAG] PID_ROLL: P=%.4f I=%.4f D=%.4f total=%.4f",
                        pid_controllers_.at(ROLL).getPTerm(), pid_controllers_.at(ROLL).getITerm(),
                        pid_controllers_.at(ROLL).getDTerm(), pid_controllers_.at(ROLL).result());
      ROS_INFO_THROTTLE(1.0, "[UnifiedCtrl LEADER DIAG] PID_PITCH: P=%.4f I=%.4f D=%.4f total=%.4f",
                        pid_controllers_.at(PITCH).getPTerm(), pid_controllers_.at(PITCH).getITerm(),
                        pid_controllers_.at(PITCH).getDTerm(), pid_controllers_.at(PITCH).result());
      ROS_INFO_THROTTLE(1.0, "[UnifiedCtrl LEADER DIAG] PID_YAW: P=%.4f I=%.4f D=%.4f total=%.4f",
                        pid_controllers_.at(YAW).getPTerm(), pid_controllers_.at(YAW).getITerm(),
                        pid_controllers_.at(YAW).getDTerm(), pid_controllers_.at(YAW).result());
      {
        // Log current RPY and target RPY for reference
        tf::Vector3 rpy_now = estimator_->getEuler(Frame::COG, estimate_mode_);
        tf::Vector3 target_rpy = navigator_->getTargetRPY();
        ROS_INFO_THROTTLE(1.0, "[UnifiedCtrl LEADER DIAG] rpy=(%.4f,%.4f,%.4f) target_rpy=(%.4f,%.4f,%.4f)",
                          rpy_now.x(), rpy_now.y(), rpy_now.z(),
                          target_rpy.x(), target_rpy.y(), target_rpy.z());
        ROS_INFO_THROTTLE(1.0, "[UnifiedCtrl LEADER DIAG] formation_pos=(%.4f,%.4f,%.4f) target_formation_pos=(%.4f,%.4f,%.4f) leader_pos=(%.4f,%.4f,%.4f)",
                          formation_pos.x(), formation_pos.y(), formation_pos.z(),
                          target_formation_pos.x(), target_formation_pos.y(), target_formation_pos.z(),
                          pos_.x(), pos_.y(), pos_.z());
      }
      // ===== DIAGNOSTIC: per-module commands sent =====
      if (ok) {
        const auto& all_cmds = unified_controller_->getModuleCommands();
        std::stringstream cmd_ss;
        for (const auto& kv : all_cmds) {
          std::string thrust_str, gimbal_str;
          for (size_t r = 0; r < kv.second.full_thrusts.size(); r++) {
            thrust_str += std::to_string(kv.second.full_thrusts[r]) + " ";
          }
          for (size_t r = 0; r < kv.second.gimbal_angles.size(); r++) {
            gimbal_str += std::to_string(kv.second.gimbal_angles[r]) + " ";
          }
          char cb[256];
          snprintf(cb, sizeof(cb), " m%d:T=[%s](sz=%zu) G=[%s](sz=%zu)",
                   kv.first, thrust_str.c_str(), kv.second.full_thrusts.size(),
                   gimbal_str.c_str(), kv.second.gimbal_angles.size());
          cmd_ss << cb;
        }
        ROS_INFO_THROTTLE(1.0, "[UnifiedCtrl LEADER DIAG] module cmds:%s", cmd_ss.str().c_str());
      }

      pre_module_state_ = module_state;
      return;  // Skip individual control path entirely
    }

    // ======== Unified Control Mode: FOLLOWER ========
    // FOLLOWER receives thrust + gimbal commands from LEADER via ROS topics,
    // then forwards them to its own spinal. No local PID or wrench comp.
    if (unified_control_mode_ && module_state == FOLLOWER && module_state != SEPARATED) {
      // H2: Disable spinal's internal attitude PID on first entry
      if (!spinal_gains_zeroed_) {
        sendZeroAttitudeGains();
        spinal_gains_zeroed_ = true;
        ROS_INFO("[UnifiedCtrl] FOLLOWER id=%d: zeroed spinal rpy/gain", beetle_navigator_->getMyID());
      }

      // Check if we have a fresh unified command from LEADER
      bool have_valid_cmd = false;
      if (unified_cmd_received_) {
        double age = (ros::Time::now() - unified_cmd_stamp_).toSec();
        if (age < 0.5) {
          have_valid_cmd = true;
        }
      }

      if (have_valid_cmd) {
        // Forward scalar thrusts (size 4) to own spinal's four_axes/command.
        follower_thrust_pub_.publish(unified_thrust_cmd_);
        // Forward gimbal angles to own spinal's gimbals_ctrl.
        follower_gimbal_pub_.publish(unified_gimbal_cmd_);

        // Mark that we have successfully transitioned to unified forwarding
        follower_unified_active_ = true;

        // ===== DIAGNOSTIC: FOLLOWER forwarding details =====
        {
          double age = (ros::Time::now() - unified_cmd_stamp_).toSec();
          std::string thrust_str;
          for (size_t r = 0; r < unified_thrust_cmd_.base_thrust.size(); r++) {
            thrust_str += std::to_string(unified_thrust_cmd_.base_thrust[r]) + " ";
          }
          std::string gimbal_str;
          for (size_t r = 0; r < unified_gimbal_cmd_.position.size(); r++) {
            gimbal_str += std::to_string(unified_gimbal_cmd_.position[r]) + " ";
          }
          ROS_INFO_THROTTLE(1.0, "[UnifiedCtrl FOLLOWER DIAG] id=%d age=%.3fs thrust=[%s](sz=%zu) gimbal=[%s](sz=%zu)",
                            beetle_navigator_->getMyID(), age,
                            thrust_str.c_str(), unified_thrust_cmd_.base_thrust.size(),
                            gimbal_str.c_str(), unified_gimbal_cmd_.position.size());
        }

        pre_module_state_ = module_state;
        return;  // Done — unified command forwarded
      }

      // ---- No valid unified command yet: continue independent hover ----
      // This avoids a thrust gap during the transition ticks before
      // LEADER's first command arrives via cross-process ROS topics.
      ROS_WARN_THROTTLE(1.0, "[UnifiedCtrl FOLLOWER] id=%d, no valid unified cmd yet — maintaining independent hover",
                        beetle_navigator_->getMyID());
      // Fall through to the normal independent control path below.
      // GimbalrotorController::controlCore() will run and produce
      // thrust + gimbal output via the normal sendCmd() chain.
    }

    prev_unified_control_mode_ = false;
    follower_unified_active_ = false;  // Reset so next unified entry starts fresh

    // H2: Restore spinal's attitude gains when exiting unified mode
    if (spinal_gains_zeroed_) {
      setAttitudeGains();
      spinal_gains_zeroed_ = false;
      ROS_INFO("[UnifiedCtrl] Exiting unified mode, restored spinal rpy/gain (id=%d)",
               beetle_navigator_->getMyID());
    }
    
    if(beetle_navigator_->getControlFlag() &&
       module_state != SEPARATED){
      calcInteractionWrench();
      comp_update_flag = true;
    }else{
      for(int i = 0; i < max_modules_num; i++){
        est_wrench_list_[i+1] = Eigen::VectorXd::Zero(6);
        inter_wrench_list_[i+1] = Eigen::VectorXd::Zero(6);
        wrench_comp_list_[i+1] = Eigen::VectorXd::Zero(6);
      }
    }

    double mass_inv = 1 / beetle_robot_model_->getMass();
    Eigen::Matrix3d inertia_inv = (beetle_robot_model_->getInertia<Eigen::Matrix3d>()).inverse();
    int my_id = beetle_navigator_->getMyID();

    if(module_state == FOLLOWER &&
       pd_wrench_comp_mode_ &&
       beetle_navigator_->getControlFlag()&&
       !beetle_navigator_->pseudo_assembly_mode_){

      /* set proper gains for wrench comp */
      int module_num = 0;
      for(const auto & item : est_wrench_list_){
        if(assembly_flag[item.first]){
          module_num ++;
        }
      }
      std::vector<int> wrench_indices = {FX, FY, FZ, TX, TY, TZ};
      for(const auto& index: wrench_indices)
        {
          pid_controllers_.at(index).setPGain(wrench_comp_p_gain_ / std::pow(2, module_num -2) );
          pid_controllers_.at(index).setDGain(wrench_comp_d_gain_ / std::pow(2, module_num -2) );
          pid_controllers_.at(index).setIGain(wrench_comp_i_gain_ / std::pow(2, module_num -2) );
        }
      Eigen::VectorXd wrench_comp_term_cog = wrench_comp_list_[my_id]; // regarding cog
      Eigen::Matrix3d cog_rot;
      tf::matrixTFToEigen(estimator_->getOrientation(Frame::COG, estimate_mode_), cog_rot);
      Eigen::VectorXd wrench_comp_term = wrench_comp_term_cog; 
      wrench_comp_term.head(3) = cog_rot * wrench_comp_term.head(3); // regarding world

      /* current version: I term reconfig mehod */
      /* wrench_comp already includes ff_inter (feedforward) + inter (observer estimate),
         so no separate desired_external_wrench_ injection needed here — that would double-count. */
      Eigen::VectorXd I_reconfig_acc_cog_term = Eigen::VectorXd::Zero(6);
      I_reconfig_acc_cog_term.head(3) = mass_inv * wrench_comp_term.head(3);
      I_reconfig_acc_cog_term.tail(3) = inertia_inv * wrench_comp_term.tail(3); //inavailable

      // Debug: log wrench_comp breakdown (throttled to 2Hz)
      if(desired_external_wrench_.norm() > 1e-6) {
        ROS_INFO_THROTTLE(0.5, "[FF Debug] id=%d, des_ext_wrench=(%.2f,%.2f,%.2f), "
          "wrench_comp_cog=(%.2f,%.2f,%.2f), wrench_comp_world=(%.2f,%.2f,%.2f), "
          "I_reconfig=(%.3f,%.3f,%.3f)",
          my_id,
          desired_external_wrench_(0), desired_external_wrench_(1), desired_external_wrench_(2),
          wrench_comp_term_cog(0), wrench_comp_term_cog(1), wrench_comp_term_cog(2),
          wrench_comp_term(0), wrench_comp_term(1), wrench_comp_term(2),
          I_reconfig_acc_cog_term(0), I_reconfig_acc_cog_term(1), I_reconfig_acc_cog_term(2));
      }

      double IGain_Fx = pid_controllers_.at(X).getIGain();
      double IGain_Fy = pid_controllers_.at(Y).getIGain();
      double IGain_Fz = pid_controllers_.at(Z).getIGain();
      double IGain_Tx = pid_controllers_.at(ROLL).getIGain();
      double IGain_Ty = pid_controllers_.at(PITCH).getIGain();
      double IGain_Tz = pid_controllers_.at(YAW).getIGain();

      double du;
      if(prev_comp_update_time_ < 0){
        prev_comp_update_time_ = ros::Time::now().toSec();
        return;
      }else{
        du = ros::Time::now().toSec() - prev_comp_update_time_;
        prev_comp_update_time_ = ros::Time::now().toSec();
      }

      pid_controllers_.at(FX).updateWoVel(I_reconfig_acc_cog_term(0) / IGain_Fx, du);
      pid_controllers_.at(FY).updateWoVel(I_reconfig_acc_cog_term(1) / IGain_Fy, du);
      pid_controllers_.at(FZ).updateWoVel(I_reconfig_acc_cog_term(2) / IGain_Fz, du);
      pid_controllers_.at(TX).updateWoVel(I_reconfig_acc_cog_term(3) / IGain_Tx, du);
      pid_controllers_.at(TY).updateWoVel(I_reconfig_acc_cog_term(4) / IGain_Ty, du);
      pid_controllers_.at(TZ).updateWoVel(I_reconfig_acc_cog_term(5) / IGain_Tz, du);

      I_comp_Fx_ = pid_controllers_.at(FX).result();
      I_comp_Fy_ = pid_controllers_.at(FY).result();
      I_comp_Fz_ = pid_controllers_.at(FZ).result();
      I_comp_Tx_ = pid_controllers_.at(TX).result();
      I_comp_Ty_ = pid_controllers_.at(TY).result();
      I_comp_Tz_ = pid_controllers_.at(TZ).result();

      // X/Y: inject wrench_comp acceleration as persistent feedforward on position PID
      // Uses setPersistentFF to avoid race condition with nav callback clearing target_acc_
      pid_controllers_.at(X).setPersistentFF(I_reconfig_acc_cog_term(0));
      pid_controllers_.at(Y).setPersistentFF(I_reconfig_acc_cog_term(1));
      pid_controllers_.at(X).setICompTerm(0.0);
      pid_controllers_.at(Y).setICompTerm(0.0);
      // Z and torque: keep original ICompTerm path
      pid_controllers_.at(Z).setICompTerm(I_comp_Fz_);
      pid_controllers_.at(ROLL).setICompTerm(I_comp_Tx_);
      pid_controllers_.at(PITCH).setICompTerm(I_comp_Ty_);
      pid_controllers_.at(YAW).setICompTerm(I_comp_Tz_);

      // Debug: log feedforward injection values (throttled to 2Hz)
      if(desired_external_wrench_.norm() > 1e-6) {
        ROS_INFO_THROTTLE(0.5, "[FF PID] id=%d, ff_acc=(%.3f,%.3f), I_comp_z=%.3f, "
          "FX_pid: p=%.3f i=%.3f d=%.3f, FY_pid: p=%.3f i=%.3f d=%.3f",
          my_id, I_reconfig_acc_cog_term(0), I_reconfig_acc_cog_term(1), I_comp_Fz_,
          pid_controllers_.at(FX).getPTerm(), pid_controllers_.at(FX).getITerm(), pid_controllers_.at(FX).getDTerm(),
          pid_controllers_.at(FY).getPTerm(), pid_controllers_.at(FY).getITerm(), pid_controllers_.at(FY).getDTerm());
      }
      
      geometry_msgs::WrenchStamped wrench_msg;
      wrench_msg.header.stamp.fromSec(estimator_->getImuLatestTimeStamp());
      wrench_msg.wrench.force.x = I_reconfig_acc_cog_term(0);
      wrench_msg.wrench.force.y = I_reconfig_acc_cog_term(1);
      wrench_msg.wrench.force.z = I_reconfig_acc_cog_term(2);
      wrench_msg.wrench.torque.x = I_reconfig_acc_cog_term(3);
      wrench_msg.wrench.torque.y = I_reconfig_acc_cog_term(4);
      wrench_msg.wrench.torque.z = I_reconfig_acc_cog_term(5);
      external_wrench_compensation_pub_.publish(wrench_msg);

      /* publish wrench comp pid value*/
      wrench_pid_msg_.header.stamp.fromSec(estimator_->getImuLatestTimeStamp());
      wrench_pid_msg_.x.total.at(0) = pid_controllers_.at(FX).result();
      wrench_pid_msg_.x.p_term.at(0) = pid_controllers_.at(FX).getPTerm();
      wrench_pid_msg_.x.i_term.at(0) = pid_controllers_.at(FX).getITerm();
      wrench_pid_msg_.x.d_term.at(0) = pid_controllers_.at(FX).getDTerm();

      wrench_pid_msg_.y.total.at(0) = pid_controllers_.at(FY).result();
      wrench_pid_msg_.y.p_term.at(0) = pid_controllers_.at(FY).getPTerm();
      wrench_pid_msg_.y.i_term.at(0) = pid_controllers_.at(FY).getITerm();
      wrench_pid_msg_.y.d_term.at(0) = pid_controllers_.at(FY).getDTerm();

      wrench_pid_msg_.z.total.at(0) = pid_controllers_.at(FZ).result();
      wrench_pid_msg_.z.p_term.at(0) = pid_controllers_.at(FZ).getPTerm();
      wrench_pid_msg_.z.i_term.at(0) = pid_controllers_.at(FZ).getITerm();
      wrench_pid_msg_.z.d_term.at(0) = pid_controllers_.at(FZ).getDTerm();
      
      wrench_pid_msg_.roll.total.at(0) = pid_controllers_.at(TX).result();
      wrench_pid_msg_.roll.p_term.at(0) = pid_controllers_.at(TX).getPTerm();
      wrench_pid_msg_.roll.i_term.at(0) = pid_controllers_.at(TX).getITerm();
      wrench_pid_msg_.roll.d_term.at(0) = pid_controllers_.at(TX).getDTerm();

      wrench_pid_msg_.pitch.total.at(0) = pid_controllers_.at(TY).result();
      wrench_pid_msg_.pitch.p_term.at(0) = pid_controllers_.at(TY).getPTerm();
      wrench_pid_msg_.pitch.i_term.at(0) = pid_controllers_.at(TY).getITerm();
      wrench_pid_msg_.pitch.d_term.at(0) = pid_controllers_.at(TY).getDTerm();

      wrench_pid_msg_.yaw.total.at(0) = pid_controllers_.at(TZ).result();
      wrench_pid_msg_.yaw.p_term.at(0) = pid_controllers_.at(TZ).getPTerm();
      wrench_pid_msg_.yaw.i_term.at(0) = pid_controllers_.at(TZ).getITerm();
      wrench_pid_msg_.yaw.d_term.at(0) = pid_controllers_.at(TZ).getDTerm();

      wrench_comp_pid_pub_.publish(wrench_pid_msg_);

      /*publish desire internal wrench*/
      if(des_wrench_pub_flag_)
        {
          beetle::TaggedWrenches all_tagged_des_wrenche_msg;
          std::vector<int> assembled_ids = beetle_navigator_->getAssemblyIds();
          all_tagged_des_wrenche_msg.tagged_wrenches.resize(assembled_ids.size());
          int cnt =0;
          for(const auto id: assembled_ids)
            {
              beetle::TaggedWrench tagged_des_wrench_msg;
              geometry_msgs::WrenchStamped des_wrench_msg;
              Eigen::VectorXd des_wrench = ff_inter_wrench_list_[id];
              des_wrench_msg.header.stamp.fromSec(estimator_->getImuLatestTimeStamp());
              des_wrench_msg.wrench.force.x = des_wrench(0);
              des_wrench_msg.wrench.force.y = des_wrench(1);
              des_wrench_msg.wrench.force.z = des_wrench(2);
              des_wrench_msg.wrench.torque.x = des_wrench(3);
              des_wrench_msg.wrench.torque.y = des_wrench(4);
              des_wrench_msg.wrench.torque.z = des_wrench(5);

              tagged_des_wrench_msg.index = id;
              tagged_des_wrench_msg.wrench = des_wrench_msg;
              all_tagged_des_wrenche_msg.tagged_wrenches[cnt] = tagged_des_wrench_msg;
              cnt ++;
            }
          des_inter_wrench_pub_.publish(all_tagged_des_wrenche_msg);
        }
    }else{
      pid_controllers_.at(FX).reset();
      pid_controllers_.at(FY).reset();
      pid_controllers_.at(FZ).reset();
      pid_controllers_.at(TX).reset();
      pid_controllers_.at(TY).reset();
      pid_controllers_.at(TZ).reset();

      // LEADER feedforward: inject desired_external_wrench_ as persistent FF on position PID
      // Uses setPersistentFF to avoid race condition with nav callback clearing target_acc_
      if(module_state == LEADER && desired_external_wrench_.norm() > 1e-6) {
        Eigen::Matrix3d cog_rot;
        tf::matrixTFToEigen(estimator_->getOrientation(Frame::COG, estimate_mode_), cog_rot);
        // desired_external_wrench_ is in body frame, rotate to world frame
        Eigen::Vector3d ff_world = cog_rot * desired_external_wrench_.head(3);
        // convert force to acceleration
        Eigen::Vector3d ff_acc = mass_inv * ff_world;
        // X/Y: persistent feedforward (avoids race condition with nav callback)
        pid_controllers_.at(X).setPersistentFF(ff_acc(0));
        pid_controllers_.at(Y).setPersistentFF(ff_acc(1));
        // Z: inject body_z directly as persistent FF, skip cog_rot to avoid
        // pitch-coupling instability. Uses setPersistentFF (not setICompTerm)
        // because ICompTerm accumulates in the I-term integrator every tick.
        double ff_z_direct = mass_inv * desired_external_wrench_(2);
        pid_controllers_.at(Z).setPersistentFF(ff_z_direct);
      } else {
        pid_controllers_.at(X).setPersistentFF(0.0);
        pid_controllers_.at(Y).setPersistentFF(0.0);
        pid_controllers_.at(Z).setPersistentFF(0.0);
      }
      pid_controllers_.at(X).setICompTerm(0.0);
      pid_controllers_.at(Y).setICompTerm(0.0);
      pid_controllers_.at(Z).setICompTerm(0.0);
      pid_controllers_.at(ROLL).setICompTerm(0.0);
      pid_controllers_.at(PITCH).setICompTerm(0.0);
      pid_controllers_.at(YAW).setICompTerm(0.0);
    }
      
    GimbalrotorController::controlCore();

    // Debug: log LEADER position PID output when towing is active
    if(module_state == LEADER && desired_external_wrench_.norm() > 1e-6) {
      ROS_INFO_THROTTLE(0.5, "[LEADER PID] id=%d, X: p=%.3f i=%.3f d=%.3f ff=%.3f sum=%.3f, "
        "Y: p=%.3f i=%.3f d=%.3f ff=%.3f sum=%.3f, Z: p=%.3f i=%.3f d=%.3f ff=%.3f sum=%.3f err_i=%.3f",
        my_id,
        pid_controllers_.at(X).getPTerm(), pid_controllers_.at(X).getITerm(),
        pid_controllers_.at(X).getDTerm(), pid_controllers_.at(X).getPersistentFF(),
        pid_controllers_.at(X).result(),
        pid_controllers_.at(Y).getPTerm(), pid_controllers_.at(Y).getITerm(),
        pid_controllers_.at(Y).getDTerm(), pid_controllers_.at(Y).getPersistentFF(),
        pid_controllers_.at(Y).result(),
        pid_controllers_.at(Z).getPTerm(), pid_controllers_.at(Z).getITerm(),
        pid_controllers_.at(Z).getDTerm(), pid_controllers_.at(Z).getPersistentFF(),
        pid_controllers_.at(Z).result(),
        pid_controllers_.at(Z).getErrI());
    }
    // Debug: log FOLLOWER position PID output when wrench_comp feedforward is active
    if(module_state != LEADER && module_state != SEPARATED && desired_external_wrench_.norm() > 1e-6) {
      ROS_INFO_THROTTLE(0.5, "[FOLLOWER PID] id=%d, X: p=%.3f i=%.3f d=%.3f ff=%.3f sum=%.3f, "
        "Y: p=%.3f i=%.3f d=%.3f ff=%.3f sum=%.3f, Z: p=%.3f i=%.3f sum=%.3f",
        my_id,
        pid_controllers_.at(X).getPTerm(), pid_controllers_.at(X).getITerm(),
        pid_controllers_.at(X).getDTerm(), pid_controllers_.at(X).getPersistentFF(),
        pid_controllers_.at(X).result(),
        pid_controllers_.at(Y).getPTerm(), pid_controllers_.at(Y).getITerm(),
        pid_controllers_.at(Y).getDTerm(), pid_controllers_.at(Y).getPersistentFF(),
        pid_controllers_.at(Y).result(),
        pid_controllers_.at(Z).getPTerm(), pid_controllers_.at(Z).getITerm(),
        pid_controllers_.at(Z).result());
    }

    pre_module_state_ = module_state;
    
  }

  bool BeetleController::update()
  {
    if (unified_control_mode_) {
      int module_state = beetle_navigator_->getModuleState();
      bool is_follower_without_cmd = false;

      // Check if this FOLLOWER lacks a valid unified command and needs
      // to fall back to independent hover (方案 A).
      if (module_state == FOLLOWER && module_state != SEPARATED) {
        bool have_valid_cmd = false;
        if (unified_cmd_received_) {
          double age = (ros::Time::now() - unified_cmd_stamp_).toSec();
          if (age < 0.5) have_valid_cmd = true;
        }
        // Also check if we've ever successfully forwarded a unified cmd.
        // Once active, stay in unified forwarding mode even if a single
        // command is momentarily late.
        if (!have_valid_cmd && !follower_unified_active_) {
          is_follower_without_cmd = true;
        }
      }

      if (is_follower_without_cmd) {
        /* FOLLOWER has not yet received its first unified command from LEADER.
           Use the full GimbalrotorController update chain to maintain
           independent hover, avoiding a thrust gap during the transition. */
        return GimbalrotorController::update();
      }

      /* In unified control mode (LEADER, or FOLLOWER with valid cmd),
         thrust and gimbal commands are published in controlCore().
         Bypass GimbalrotorController::update() which would call
         sendCmd()->sendFourAxisCommand() and overwrite those commands.

         We still need:
         1. ControlBase::update() for activation/timing checks
         2. controlCore() for the unified allocation / forwarding
         3. PoseLinearController::sendCmd() for PID debug publishing */
      if (!ControlBase::update()) return false;
      controlCore();
      PoseLinearController::sendCmd();
      return true;
    }

    /* Non-unified mode: use the full GimbalrotorController update chain
       (sendGimbalCommand + PoseLinearController::update -> controlCore + sendCmd) */
    return GimbalrotorController::update();
  }

  void BeetleController::reset()
  {
    GimbalrotorController::reset();
    pid_controllers_.at(FX).reset();
    pid_controllers_.at(FY).reset();
    pid_controllers_.at(FZ).reset();
    pid_controllers_.at(TX).reset();
    pid_controllers_.at(TY).reset();
    pid_controllers_.at(TZ).reset();
    pid_controllers_.at(X).setICompTerm(0.0);
    pid_controllers_.at(Y).setICompTerm(0.0);
    pid_controllers_.at(Z).setICompTerm(0.0);
    pid_controllers_.at(ROLL).setICompTerm(0.0);
    pid_controllers_.at(PITCH).setICompTerm(0.0);
    pid_controllers_.at(YAW).setICompTerm(0.0);
    pid_controllers_.at(X).setPersistentFF(0.0);
    pid_controllers_.at(Y).setPersistentFF(0.0);
    pid_controllers_.at(Z).setPersistentFF(0.0);

    // Clear unified mode FOLLOWER state so that stale commands are not
    // forwarded when switching back to unified mode.
    unified_cmd_received_ = false;
    follower_unified_active_ = false;
  }

  void BeetleController::unifiedThrustCallback(const spinal::FourAxisCommand& msg)
  {
    unified_thrust_cmd_ = msg;
    unified_cmd_received_ = true;
    unified_cmd_stamp_ = ros::Time::now();
  }

  void BeetleController::unifiedGimbalCallback(const sensor_msgs::JointState& msg)
  {
    unified_gimbal_cmd_ = msg;
    unified_cmd_received_ = true;
    unified_cmd_stamp_ = ros::Time::now();
  }

  void BeetleController::sendZeroAttitudeGains()
  {
    // H2: Send all-zero rpy/gain to this module's spinal.
    // This zeroes out thrust_p/i/d_gain_ inside spinal's AttitudeController,
    // so roll_pitch_term_ becomes 0 and spinal acts as a pure PWM executor.
    // rpy_gain_pub_ is inherited (protected) from GimbalrotorController.
    spinal::RollPitchYawTerms rpy_gain_msg;
    rpy_gain_msg.motors.resize(1);
    rpy_gain_msg.motors.at(0).roll_p = 0;
    rpy_gain_msg.motors.at(0).roll_i = 0;
    rpy_gain_msg.motors.at(0).roll_d = 0;
    rpy_gain_msg.motors.at(0).pitch_p = 0;
    rpy_gain_msg.motors.at(0).pitch_i = 0;
    rpy_gain_msg.motors.at(0).pitch_d = 0;
    rpy_gain_msg.motors.at(0).yaw_d = 0;
    rpy_gain_pub_.publish(rpy_gain_msg);
  }

  void BeetleController::calcInteractionWrench()
  {
    /* 1. calculate external wrench W_w for whole system*/
    Eigen::VectorXd W_w = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd W_sum = Eigen::VectorXd::Zero(6);
    int module_num = 0;
    std::map<int, bool> assembly_flag = beetle_navigator_->getAssemblyFlags();

    for(const auto & item : est_wrench_list_){
      if(assembly_flag[item.first]){
      W_sum += item.second;
      module_num ++;
      }
    }

    if(!module_num) return;
    W_w = W_sum / module_num;
    geometry_msgs::WrenchStamped wrench_msg;
    wrench_msg.header.stamp.fromSec(estimator_->getImuLatestTimeStamp());
    wrench_msg.wrench.force.x = W_w(0);
    wrench_msg.wrench.force.y = W_w(1);
    wrench_msg.wrench.force.z = W_w(2);
    wrench_msg.wrench.torque.x = W_w(3);
    wrench_msg.wrench.torque.y = W_w(4);
    wrench_msg.wrench.torque.z = W_w(5);
    whole_external_wrench_pub_.publish(wrench_msg);

    /* 2. calculate interactional wrench for each module*/
    Eigen::VectorXd left_inter_wrench = Eigen::VectorXd::Zero(6); //'left_inter_wrench' represents the wrench applied from right-side module to left-side module
    for(const auto & item : est_wrench_list_){
      if(assembly_flag[item.first]){
        Eigen::VectorXd right_inter_wrench = item.second - W_w + left_inter_wrench;
        inter_wrench_list_[item.first] = right_inter_wrench;
        left_inter_wrench = right_inter_wrench;
      }else{
        inter_wrench_list_[item.first] = Eigen::VectorXd::Zero(6);
      }
    }
    int my_id = beetle_navigator_->getMyID();
    wrench_msg.wrench.force.x = inter_wrench_list_[my_id](0);
    wrench_msg.wrench.force.y = inter_wrench_list_[my_id](1);
    wrench_msg.wrench.force.z = inter_wrench_list_[my_id](2);
    wrench_msg.wrench.torque.x = inter_wrench_list_[my_id](3);
    wrench_msg.wrench.torque.y = inter_wrench_list_[my_id](4);
    wrench_msg.wrench.torque.z = inter_wrench_list_[my_id](5);
    internal_wrench_pub_.publish(wrench_msg);
    /* 3. calculate wrench compensation term for each module*/
    int leader_id = beetle_navigator_->getLeaderID();
    /* 3.1. process from leader to left*/
    int right_module_id = leader_id;
    Eigen::VectorXd wrench_comp_sum_left = Eigen::VectorXd::Zero(6);
    for(int i = leader_id-1; i > 0; i--){
      if(assembly_flag[i]){
        wrench_comp_sum_left += -ff_inter_wrench_list_[i] + inter_wrench_list_[i];
        // wrench_comp_list_[i] += wrench_comp_gain_ *  wrench_comp_sum_left;
        wrench_comp_list_[i] = wrench_comp_sum_left;
        right_module_id = i;
      }else{
        wrench_comp_list_[i] = Eigen::VectorXd::Zero(6);
      }
    }
    /* 3.2. process from leader to right*/
    int max_modules_num = beetle_navigator_->getMaxModuleNum();
    int left_module_id = leader_id;
    Eigen::VectorXd wrench_comp_sum_right = Eigen::VectorXd::Zero(6);
    for(int i = leader_id+1; i <= max_modules_num; i++){
      if(assembly_flag[i]){
        wrench_comp_sum_right += ff_inter_wrench_list_[left_module_id] - inter_wrench_list_[left_module_id];
        // wrench_comp_list_[i] += wrench_comp_gain_ * wrench_comp_sum_right;
        wrench_comp_list_[i] = wrench_comp_sum_right;
        left_module_id = i;
      }else{
        wrench_comp_list_[i] = Eigen::VectorXd::Zero(6);
      }
    }

    // Debug: log ff_inter and wrench_comp for this module (throttled to 2Hz)
    if(desired_external_wrench_.norm() > 1e-6) {
      ROS_INFO_THROTTLE(0.5, "[WrenchComp Debug] id=%d, ff_inter[%d]=(%.3f,%.3f,%.3f), "
        "inter[%d]=(%.3f,%.3f,%.3f), wrench_comp[%d]=(%.3f,%.3f,%.3f)",
        my_id,
        my_id, ff_inter_wrench_list_[my_id](0), ff_inter_wrench_list_[my_id](1), ff_inter_wrench_list_[my_id](2),
        my_id, inter_wrench_list_[my_id](0), inter_wrench_list_[my_id](1), inter_wrench_list_[my_id](2),
        my_id, wrench_comp_list_[my_id](0), wrench_comp_list_[my_id](1), wrench_comp_list_[my_id](2));
    }
  }
  void BeetleController::rosParamInit()
  {
    GimbalrotorController::rosParamInit();
    ros::NodeHandle control_nh(nh_, "controller");
    getParam<bool>(control_nh, "pd_wrench_comp_mode", pd_wrench_comp_mode_, false);

    double external_force_upper_limit, external_force_lower_limit, external_torque_upper_limit, external_torque_lower_limit;
    getParam<double>(control_nh, "external_force_upper_limit", external_force_upper_limit, 0.5);
    getParam<double>(control_nh, "external_force_lower_limit", external_force_lower_limit, -0.5);
    getParam<double>(control_nh, "external_torque_upper_limit", external_torque_upper_limit, 0.01);
    getParam<double>(control_nh, "external_torque_lower_limit", external_torque_lower_limit, -0.01);
    external_wrench_upper_limit_.head(3) = Eigen::Vector3d::Constant(external_force_upper_limit);
    external_wrench_upper_limit_.tail(3) = Eigen::Vector3d::Constant(external_torque_upper_limit);
    external_wrench_lower_limit_.head(3) = Eigen::Vector3d::Constant(external_force_lower_limit);
    external_wrench_lower_limit_.tail(3) = Eigen::Vector3d::Constant(external_torque_lower_limit);
    ROS_INFO_STREAM("upper limit of external wrench : "<<external_wrench_upper_limit_.transpose());
    ROS_INFO_STREAM("lower limit of external wrench : "<<external_wrench_lower_limit_.transpose());

    getParam<double>(control_nh, "comp_term_update_freq", comp_term_update_freq_, 10);

    ros::NodeHandle wrench_nh(control_nh, "wrench_comp");
    getParam<double>(wrench_nh, "p_gain", wrench_comp_p_gain_, 0.1);
    getParam<double>(wrench_nh, "i_gain", wrench_comp_i_gain_, 0.005);
    getParam<double>(wrench_nh, "d_gain", wrench_comp_d_gain_, 0.07);

    getParam<bool>(control_nh, "unified_control_mode", unified_control_mode_, false);
  }

  void BeetleController::externalWrenchEstimate()
  {
    const Eigen::VectorXd target_wrench_acc_cog = getTargetWrenchAccCog();

    if(navigator_->getNaviState() != aerial_robot_navigation::HOVER_STATE &&
       navigator_->getNaviState() != aerial_robot_navigation::TAKEOFF_STATE &&
       navigator_->getNaviState() != aerial_robot_navigation:: LAND_STATE)
      {
        prev_est_wrench_timestamp_ = 0;
        integrate_term_ = Eigen::VectorXd::Zero(6);
        return;
      }else if(target_wrench_acc_cog.size() == 0){
        ROS_WARN("Target wrench value for wrench estimation is not setted.");
        prev_est_wrench_timestamp_ = 0;
        integrate_term_ = Eigen::VectorXd::Zero(6);
        return;
      }

    Eigen::Vector3d vel_w, omega_cog; // workaround: use the filtered value
    auto imu_handler = boost::dynamic_pointer_cast<sensor_plugin::Imu>(estimator_->getImuHandler(0));
    tf::vectorTFToEigen(imu_handler->getFilteredVelCog(), vel_w);
    tf::vectorTFToEigen(imu_handler->getFilteredOmegaCog(), omega_cog);
    Eigen::Matrix3d cog_rot;
    tf::matrixTFToEigen(estimator_->getOrientation(Frame::COG, estimate_mode_), cog_rot);

    Eigen::Matrix3d inertia = robot_model_->getInertia<Eigen::Matrix3d>();
    double mass = robot_model_->getMass();

    Eigen::VectorXd sum_momentum = Eigen::VectorXd::Zero(6);
    sum_momentum.head(3) = mass * vel_w;
    sum_momentum.tail(3) = inertia * omega_cog;

    Eigen::VectorXd target_wrench_cog = Eigen::VectorXd::Zero(6);
    target_wrench_cog.head(3) = mass * target_wrench_acc_cog.head(3);
    target_wrench_cog.tail(3) = inertia * target_wrench_acc_cog.tail(3);

    Eigen::MatrixXd J_t = Eigen::MatrixXd::Identity(6,6);
    J_t.topLeftCorner(3,3) = cog_rot;

    Eigen::VectorXd N = mass * robot_model_->getGravity();
    N.tail(3) = aerial_robot_model::skew(omega_cog) * (inertia * omega_cog);

    if(prev_est_wrench_timestamp_ == 0)
      {
        prev_est_wrench_timestamp_ = ros::Time::now().toSec();
        init_sum_momentum_ = sum_momentum; // not good
      }

    double dt = ros::Time::now().toSec() - prev_est_wrench_timestamp_;

    integrate_term_ += (J_t * target_wrench_cog - N + est_external_wrench_) * dt;

    est_external_wrench_ = momentum_observer_matrix_ * (sum_momentum - init_sum_momentum_ - integrate_term_);

    Eigen::VectorXd est_external_wrench_cog = est_external_wrench_;
    est_external_wrench_cog.head(3) = cog_rot.inverse() * est_external_wrench_.head(3);

    geometry_msgs::WrenchStamped wrench_msg;
    wrench_msg.header.stamp.fromSec(estimator_->getImuLatestTimeStamp());
    wrench_msg.wrench.force.x = est_external_wrench_cog(0);
    wrench_msg.wrench.force.y = est_external_wrench_cog(1);
    wrench_msg.wrench.force.z = est_external_wrench_cog(2);
    wrench_msg.wrench.torque.x = est_external_wrench_cog(3);
    wrench_msg.wrench.torque.y = est_external_wrench_cog(4);
    wrench_msg.wrench.torque.z = est_external_wrench_cog(5);
    estimate_external_wrench_pub_.publish(wrench_msg);

    beetle::TaggedWrench tagged_wrench;
    tagged_wrench.index = beetle_navigator_->getMyID();
    tagged_wrench.wrench = wrench_msg;
    tagged_external_wrench_pub_.publish(tagged_wrench);

    prev_est_wrench_timestamp_ = ros::Time::now().toSec();
  }

  void BeetleController::estExternalWrenchCallback(const beetle::TaggedWrench & msg)
  {
    int id = msg.index;
    geometry_msgs::Wrench wrench_msg = msg.wrench.wrench;
    double time_stamp = msg.wrench.header.stamp.toSec();
    Eigen::VectorXd wrench = Eigen::VectorXd::Zero(6);
    wrench(0) =  wrench_msg.force.x;
    wrench(1) =  wrench_msg.force.y;
    wrench(2) =  wrench_msg.force.z;
    wrench(3) =  wrench_msg.torque.x;
    wrench(4) =  wrench_msg.torque.y;
    wrench(5) =  wrench_msg.torque.z;
    est_wrench_list_[id] = wrench;
  }

  void BeetleController::ffInterWrenchCallback(const beetle::TaggedWrench & msg)
  {
    int id = msg.index;
    geometry_msgs::Wrench wrench_msg = msg.wrench.wrench;
    double time_stamp = msg.wrench.header.stamp.toSec();
    Eigen::VectorXd wrench = Eigen::VectorXd::Zero(6);
    wrench(0) =  wrench_msg.force.x;
    wrench(1) =  wrench_msg.force.y;
    wrench(2) =  wrench_msg.force.z;
    wrench(3) =  wrench_msg.torque.x;
    wrench(4) =  wrench_msg.torque.y;
    wrench(5) =  wrench_msg.torque.z;
    ff_inter_wrench_list_[id] = wrench;
  }

  void BeetleController::desiredExternalWrenchCallback(const geometry_msgs::WrenchStamped & msg)
  {
    // Receive desired total external wrench for the whole assembly (body frame),
    // then distribute to per-module ff_inter_wrench via ROS topics so that
    // every module's ffInterWrenchCallback updates its local ff_inter_wrench_list_.
    //
    // Force distribution strategy:
    //   F_total is split equally among ALL assembled modules (N_total).
    //   - LEADER: gets share via desired_external_wrench_ -> setPersistentFF in else branch
    //   - FOLLOWERs: get share via ff_inter_wrench_list_ -> wrench_comp -> setPersistentFF
    //
    // ff_inter mapping (after calcInteractionWrench sign fix):
    //   3.1 (i < leader): wrench_comp[i] = -ff_inter[i] + inter[i]
    //        To drive FOLLOWER i with +F_share: need -ff_inter[i] = F_share => ff_inter[i] = -F_share
    //   3.2 (i > leader): wrench_comp[i] = ff_inter[left] - inter[left]
    //        To drive FOLLOWER i with +F_share: need ff_inter[left] = F_share

    Eigen::VectorXd desired = Eigen::VectorXd::Zero(6);
    desired(0) = msg.wrench.force.x;
    desired(1) = msg.wrench.force.y;
    desired(2) = msg.wrench.force.z;
    desired(3) = msg.wrench.torque.x;
    desired(4) = msg.wrench.torque.y;
    desired(5) = msg.wrench.torque.z;

    // Only LEADER distributes; FOLLOWERs just store their share and return
    if(beetle_navigator_->getModuleState() != LEADER) {
      // FOLLOWERs receive their share via desired_ext_wrench_pubs_ (set below)
      desired_external_wrench_ = desired;
      return;
    }

    std::map<int, bool> assembly_flag = beetle_navigator_->getAssemblyFlags();
    int leader_id = beetle_navigator_->getLeaderID();

    // Count ALL assembled modules (including leader)
    int total_count = 0;
    for(const auto & item : assembly_flag) {
      if(item.second) total_count++;
    }
    if(total_count == 0) return;

    Eigen::VectorXd share = desired / total_count;

    // LEADER stores its own share (not full desired!)
    desired_external_wrench_ = share;

    // Build per-module ff_inter values based on the derivation:
    //   For module i < leader: ff_inter[i] = -share  (so wrench_comp[i] = share when inter=0)
    //   For module i > leader: ff_inter[left_of_i] = share  (so wrench_comp[i] = share when inter=0)
    std::map<int, Eigen::VectorXd> ff_values;
    for(const auto & item : assembly_flag) {
      if(!item.second) continue;
      int id = item.first;
      if(id < leader_id) {
        // Section 3.1: wrench_comp[i] = -ff_inter[i] + inter[i]
        // Want wrench_comp[i] = share => ff_inter[i] = -share
        ff_values[id] = -share;
      } else if(id > leader_id) {
        // Section 3.2: wrench_comp[i] = ff_inter[left] - inter[left]
        // Want wrench_comp[i] = share => ff_inter[left] = share
        int left_id = leader_id;
        for(int j = id - 1; j >= 1; j--) {
          if(assembly_flag.count(j) && assembly_flag.at(j)) {
            left_id = j;
            break;
          }
        }
        ff_values[left_id] = share;
      }
    }

    // Publish ff_inter via ROS topics so all modules receive the update
    ros::Time stamp = msg.header.stamp;
    for(const auto & kv : ff_values) {
      beetle::TaggedWrench tw;
      tw.index = kv.first;
      tw.wrench.header.stamp = stamp;
      tw.wrench.wrench.force.x = kv.second(0);
      tw.wrench.wrench.force.y = kv.second(1);
      tw.wrench.wrench.force.z = kv.second(2);
      tw.wrench.wrench.torque.x = kv.second(3);
      tw.wrench.wrench.torque.y = kv.second(4);
      tw.wrench.wrench.torque.z = kv.second(5);
      if(ff_inter_wrench_pubs_.count(kv.first))
        ff_inter_wrench_pubs_[kv.first].publish(tw);
    }

    // Publish zeros for assembled modules not in ff_values (e.g., leader itself, or modules
    // whose ff_inter was not explicitly set)
    for(const auto & item : assembly_flag) {
      if(!item.second) continue;
      if(ff_values.count(item.first)) continue;
      beetle::TaggedWrench tw;
      tw.index = item.first;
      tw.wrench.header.stamp = stamp;
      // wrench fields default to 0
      if(ff_inter_wrench_pubs_.count(item.first))
        ff_inter_wrench_pubs_[item.first].publish(tw);
    }

    // Distribute per-follower share via desired_external_wrench topics
    // so each FOLLOWER's desired_external_wrench_ gets its share value
    for(const auto & item : assembly_flag) {
      if(!item.second) continue;
      if(item.first == leader_id) continue;  // skip LEADER to avoid cascade
      if(desired_ext_wrench_pubs_.count(item.first) == 0) continue;
      geometry_msgs::WrenchStamped fw;
      fw.header.stamp = stamp;
      fw.wrench.force.x = share(0);
      fw.wrench.force.y = share(1);
      fw.wrench.force.z = share(2);
      fw.wrench.torque.x = share(3);
      fw.wrench.torque.y = share(4);
      fw.wrench.torque.z = share(5);
      desired_ext_wrench_pubs_[item.first].publish(fw);
    }
  }

} //namespace aerial_robot_controller

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::BeetleController, aerial_robot_control::ControlBase);
