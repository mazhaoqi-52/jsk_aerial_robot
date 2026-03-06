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
    unified_transition_count_(-1),
    z_integral_freeze_count_(0),
    z_ki_boost_count_(0),
    spinal_gains_zeroed_(false),
    last_unified_z_i_ss_(0.8),
    has_unified_z_i_ss_(false),
    z_i_seed_default_(0.8),
    has_cached_independent_cmd_(false),
    gains_switched_(false)
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
    
    // Note: unified_control_mode_ is now read in update() before routing,
    // so controlCore() always sees the up-to-date value.

    // ======== Unified Control Mode ========
    // LEADER does full 6-DOF PID (position + attitude) → formation allocation.
    // Spinal acts as pure PWM executor (zero attitude gains).
    if (unified_control_mode_ && module_state == LEADER && module_state != SEPARATED) {
      // Detect mode switch: reset targets and zero spinal gains
      if (pre_module_state_ != LEADER || !prev_unified_control_mode_) {
        // Reset target position/yaw to current state at mode switch
        {
          tf::Vector3 cur_pos = estimator_->getPos(Frame::COG, estimate_mode_);
          navigator_->setXyControlMode(aerial_robot_navigation::POS_CONTROL_MODE);
          navigator_->setTargetPosX(cur_pos.x());
          navigator_->setTargetPosY(cur_pos.y());
          navigator_->setTargetVelX(0);
          navigator_->setTargetVelY(0);
          navigator_->setTargetAccX(0);
          navigator_->setTargetAccY(0);

          double cur_yaw = estimator_->getEuler(Frame::COG, estimate_mode_).z();
          navigator_->setTargetYaw(cur_yaw);
          navigator_->setTargetOmegaZ(0);
        }
        pid_controllers_.at(ROLL).setErrI(0);
        pid_controllers_.at(PITCH).setErrI(0);
        pid_controllers_.at(X).setErrI(0);
        pid_controllers_.at(Y).setErrI(0);

        // De-gravity Z I-term migration + seed injection (Plan E'):
        // Step 1: Remove gravity from I-term (de-gravity, same as before)
        // Step 2: Add unified-mode steady-state bias seed (NEW)
        //
        // Independent mode Z I-term ≈ G (compensates gravity implicitly).
        // Unified mode has explicit gravity FF, so de-gravity subtracts it.
        // But unified mode also needs an additional Z_i bias (≈0.86–0.94) due to
        // formation geometry (CoG offset, gimbal deflection for pitch moment, etc.).
        // This bias doesn't exist in independent mode, so we preload it as a seed.
        {
          // 1. Get current I output in acceleration domain (same units as gravity_ff)
          double i_output_old = pid_controllers_.at(Z).getITerm();  // clamp(err_i * Ki, -limit_i, limit_i)

          // 2. Compute gravity_ff_z using the SAME path as unified mode (reuse R^{-1} * (0,0,G))
          tf::Matrix3x3 uav_rot_mig = estimator_->getOrientation(Frame::COG, estimate_mode_);
          tf::Vector3 gravity_w_mig(0, 0, aerial_robot_estimation::G);
          tf::Vector3 gravity_cog_mig = uav_rot_mig.inverse() * gravity_w_mig;
          double gravity_ff_z = gravity_cog_mig.z();  // body-z component of gravity in acc domain

          // 3. De-gravity: new I output = old I output - gravity_ff (keep only bias)
          double i_output_degrav = i_output_old - gravity_ff_z;

          // 4. Seed injection: add unified-mode steady-state bias
          //    seed source: last recorded SS value, or configurable default
          double seed_value = has_unified_z_i_ss_ ? last_unified_z_i_ss_ : z_i_seed_default_;
          double i_output_seeded = i_output_degrav + Z_SEED_GAIN * seed_value;

          // 5. Convert back to err_i domain: err_i = i_output / Ki
          double Ki = std::max(pid_controllers_.at(Z).getIGain(), 1e-6);
          double iz_new = i_output_seeded / Ki;

          // 6. Symmetric clamp (allow negative bias) + finite check
          double iz_limit = pid_controllers_.at(Z).getLimitI() / Ki;
          if (!std::isfinite(iz_new)) iz_new = 0.0;
          iz_new = boost::algorithm::clamp(iz_new, -iz_limit, iz_limit);

          pid_controllers_.at(Z).setErrI(iz_new);
          // Freeze Z integration for a few frames to avoid transient pollution
          z_integral_freeze_count_ = Z_INTEGRAL_FREEZE_FRAMES;
          // Start Ki-boost phase right after freeze ends (gentler now with seed)
          z_ki_boost_count_ = Z_KI_BOOST_FRAMES;
          ROS_WARN("[UnifiedCtrl] Z de-gravity + seed: i_old=%.4f, gravity_ff=%.4f, "
                   "i_degrav=%.4f, seed=%.4f(×%.1f=%s), i_seeded=%.4f, err_i=%.4f "
                   "(limit=±%.1f), freeze=%d, boost=%d(×%.1f)",
                   i_output_old, gravity_ff_z, i_output_degrav,
                   seed_value, Z_SEED_GAIN,
                   has_unified_z_i_ss_ ? "adaptive" : "default",
                   i_output_seeded, iz_new, iz_limit,
                   z_integral_freeze_count_, z_ki_boost_count_, Z_KI_BOOST_FACTOR);
        }

        // [SINK_DIAG] Snapshot state at mode switch to identify sinking root cause
        {
          tf::Vector3 cur_pos = estimator_->getPos(Frame::COG, estimate_mode_);
          tf::Vector3 cur_vel = estimator_->getVel(Frame::COG, estimate_mode_);
          tf::Vector3 cur_rpy = estimator_->getEuler(Frame::COG, estimate_mode_);
          ROS_WARN("[SINK_DIAG_SWITCH] LEADER snapshot: pos=(%.4f,%.4f,%.4f) vel=(%.3f,%.3f,%.3f) "
                   "rpy=(%.4f,%.4f,%.4f) Z_I_kept=%.4f",
                   cur_pos.x(), cur_pos.y(), cur_pos.z(),
                   cur_vel.x(), cur_vel.y(), cur_vel.z(),
                   cur_rpy.x(), cur_rpy.y(), cur_rpy.z(),
                   pid_controllers_.at(Z).getErrI());
        }

        sendZeroAttitudeGains();
        applyUnifiedGains();
        spinal_gains_zeroed_ = true;
        unified_transition_count_ = 0;
        ROS_WARN("[UnifiedCtrl] LEADER mode switch: reset targets, zeroed spinal gains, applied unified PID gains, t=%.4f",
                 ros::Time::now().toSec());
      }
      prev_unified_control_mode_ = true;

      // --- Get LEADER state ---
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

      // --- Compute formation CoG position in world frame ---
      unified_controller_->updateFormationGeometry();
      const Eigen::Vector3d& cog_offset = unified_controller_->getFormationCogOffset();
      tf::Vector3 offset_body(cog_offset.x(), cog_offset.y(), cog_offset.z());
      tf::Vector3 offset_world = cog_rot * offset_body;
      tf::Vector3 formation_pos = pos_ + offset_world;
      tf::Vector3 omega_world = cog_rot * omega_;
      tf::Vector3 formation_vel = vel_ + omega_world.cross(offset_world);
      tf::Vector3 target_formation_pos = target_pos_ + target_rot * offset_body;

      // --- Position PID (X/Y/Z) with formation CoG ---
      double du = ros::Time::now().toSec() - control_timestamp_;
      switch(navigator_->getXyControlMode()) {
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
        default: break;
      }
      if(navigator_->getForceLandingFlag()) {
        pid_controllers_.at(X).reset();
        pid_controllers_.at(Y).reset();
      }

      double err_z = target_formation_pos.z() - formation_pos.z();
      double err_v_z = target_vel_.z() - formation_vel.z();
      double z_p_limit = pid_controllers_.at(Z).getLimitP();
      if(navigator_->getForceLandingFlag()) {
        pid_controllers_.at(Z).setLimitP(0);
        err_z = force_landing_descending_rate_;
        err_v_z = 0;
        target_acc_.setZ(0);
      }
      pid_controllers_.at(Z).update(err_z, du, err_v_z, target_acc_.z());
      // In unified mode, Z I-term is allowed to be negative (explicit gravity FF handles g,
      // so I-term only compensates model bias which can be positive or negative).
      // The floor>=0 constraint is only applied in the independent mode path (base class).

      // Integral freeze: during the first few frames after mode switch,
      // revert Z I-term to its pre-update value to avoid eating transient errors
      // (CoG jump, pitch moment onset, etc.)
      if (z_integral_freeze_count_ > 0) {
        pid_controllers_.at(Z).setErrI(pid_controllers_.at(Z).getPrevErrI());
        z_integral_freeze_count_--;
      }
      // Ki-boost (D' scheme): after freeze ends, accelerate Z I-term convergence
      // by adding extra integral increment: err_i += err_p * dt * (boost_factor - 1).
      // The normal PID::update() already did err_i += err_p * dt * 1, so total
      // effective rate = boost_factor * Ki.
      // AW1: if any motor thrust (from PREVIOUS frame's allocation) is near its
      // min/max limit, suppress the boost to prevent windup when allocation is saturated.
      else if (z_ki_boost_count_ > 0) {
        bool allocation_saturated = false;
        // Check motor thrust saturation using previous frame's allocation result
        const auto& cmds_aw = unified_controller_->getModuleCommands();
        if (!cmds_aw.empty()) {
          const double t_max = beetle_robot_model_->getThrustUpperLimit();
          const double t_min = beetle_robot_model_->getThrustLowerLimit();
          const double sat_margin = 0.05;  // 5% margin to detect near-saturation
          const double t_upper = t_max * (1.0 - sat_margin);
          const double t_lower = t_min + t_max * sat_margin;
          for (const auto& kv : cmds_aw) {
            for (float t : kv.second.full_thrusts) {
              if (t >= t_upper || t <= t_lower) {
                allocation_saturated = true;
                break;
              }
            }
            if (allocation_saturated) break;
          }
        }

        if (!allocation_saturated) {
          // Apply boost: add extra (boost_factor - 1) × err_p × dt to err_i
          double clamped_err_p = pid_controllers_.at(Z).getErrP();  // already clamped by PID::update
          double extra_increment = clamped_err_p * du * (Z_KI_BOOST_FACTOR - 1.0);
          double new_err_i = pid_controllers_.at(Z).getErrI() + extra_increment;

          // Respect err_i limits
          double Ki = std::max(pid_controllers_.at(Z).getIGain(), 1e-6);
          double iz_limit = pid_controllers_.at(Z).getLimitI() / Ki;
          new_err_i = boost::algorithm::clamp(new_err_i, -iz_limit, iz_limit);
          pid_controllers_.at(Z).setErrI(new_err_i);
        }

        z_ki_boost_count_--;
        if (z_ki_boost_count_ % 20 == 0 || z_ki_boost_count_ == 0) {
          ROS_WARN("[Z_BOOST] remaining=%d sat=%d err_p=%.4f err_i=%.4f i_term=%.4f",
                   z_ki_boost_count_, allocation_saturated ? 1 : 0,
                   pid_controllers_.at(Z).getErrP(),
                   pid_controllers_.at(Z).getErrI(),
                   pid_controllers_.at(Z).getITerm());
        }
      }

      if(navigator_->getForceLandingFlag()) {
        pid_controllers_.at(Z).setLimitP(z_p_limit);
        pid_controllers_.at(Z).setErrP(0);
      }

      // --- Attitude PID (Roll/Pitch/Yaw) ---
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

      // --- Build 6-DOF target wrench in acceleration space ---
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

      // Gravity feedforward: in unified mode spinal's attitude PID is zeroed,
      // so PC must explicitly provide gravity compensation in the body Z axis.
      {
        tf::Matrix3x3 uav_rot_ff = estimator_->getOrientation(Frame::COG, estimate_mode_);
        tf::Vector3 gravity_w(0, 0, aerial_robot_estimation::G);
        tf::Vector3 gravity_cog = uav_rot_ff.inverse() * gravity_w;
        target_wrench_acc.head(3) += Eigen::Vector3d(gravity_cog.x(), gravity_cog.y(), gravity_cog.z());
      }

      // --- Gyro compensation: ω × I·ω (same as GimbalrotorController: add torque directly) ---
      {
        const Eigen::Matrix3d& I_form = unified_controller_->getFormationInertia();
        Eigen::Vector3d omega_eigen;
        tf::vectorTFToEigen(omega_, omega_eigen);
        Eigen::Vector3d gyro = omega_eigen.cross(I_form * omega_eigen);
        target_wrench_acc.tail(3) += gyro;
      }

      // Store for external wrench estimator
      setTargetWrenchAccCog(target_wrench_acc);

      // --- Run unified 6-DOF allocation ---
      bool ok = unified_controller_->computeUnifiedAllocation(target_wrench_acc, desired_external_wrench_);

      if (ok) {
        // Publish commands to all FOLLOWERs
        unified_controller_->publishCommands();

        // LEADER also sends its own command to its own spinal
        int my_id = beetle_navigator_->getMyID();
        const auto& cmds = unified_controller_->getModuleCommands();
        auto it = cmds.find(my_id);
        if (it != cmds.end()) {
          spinal::FourAxisCommand my_thrust_msg;
          // Send scalar thrusts (same format as GimbalrotorController::sendFourAxisCommand
          // with gimbal_calc_in_fc=false): spinal has motor_number_=motor_num=4.
          my_thrust_msg.base_thrust = it->second.full_thrusts;
          my_thrust_msg.angles[0] = 0;
          my_thrust_msg.angles[1] = 0;
          my_thrust_msg.angles[2] = 0;
          follower_thrust_pub_.publish(my_thrust_msg);

          sensor_msgs::JointState my_gimbal_msg;
          my_gimbal_msg.header.stamp = ros::Time::now();
          my_gimbal_msg.position = it->second.gimbal_angles;
          follower_gimbal_pub_.publish(my_gimbal_msg);
        }
      }

      // --- Diagnostics ---
      if (unified_transition_count_ >= 0 && unified_transition_count_ < 20) {
        ROS_WARN("[TRANS_DIAG frame=%d] rpy=(%.5f,%.5f,%.5f) wrench_acc=(%.4f,%.4f,%.4f,%.4f,%.4f,%.4f) ok=%d",
                 unified_transition_count_,
                 rpy_.x(), rpy_.y(), rpy_.z(),
                 target_wrench_acc(0), target_wrench_acc(1), target_wrench_acc(2),
                 target_wrench_acc(3), target_wrench_acc(4), target_wrench_acc(5), ok);

        // [SINK_DIAG] Extended transition diagnostics
        {
          // Target vs actual formation position
          const Eigen::Vector3d& cog_off = unified_controller_->getFormationCogOffset();
          tf::Matrix3x3 cur_rot = estimator_->getOrientation(Frame::COG, estimate_mode_);
          tf::Vector3 off_body(cog_off.x(), cog_off.y(), cog_off.z());
          tf::Vector3 off_world = cur_rot * off_body;
          tf::Vector3 fpos = pos_ + off_world;
          tf::Vector3 tpos = target_pos_;
          tf::Matrix3x3 tgt_rot; tgt_rot.setRPY(target_rpy_.x(), target_rpy_.y(), target_rpy_.z());
          tf::Vector3 tfpos = tpos + tgt_rot * off_body;

          ROS_WARN("[SINK_DIAG_POS frame=%d] form_pos=(%.4f,%.4f,%.4f) tgt_form_pos=(%.4f,%.4f,%.4f) "
                   "err_z=%.5f Z_pid: p=%.4f i=%.4f d=%.4f sum=%.4f",
                   unified_transition_count_,
                   fpos.x(), fpos.y(), fpos.z(),
                   tfpos.x(), tfpos.y(), tfpos.z(),
                   tfpos.z() - fpos.z(),
                   pid_controllers_.at(Z).getPTerm(),
                   pid_controllers_.at(Z).getITerm(),
                   pid_controllers_.at(Z).getDTerm(),
                   pid_controllers_.at(Z).result());

          // Per-module thrust summary from allocation
          if (ok) {
            const auto& cmds2 = unified_controller_->getModuleCommands();
            for (const auto& kv : cmds2) {
              float tsum = 0;
              for (float t : kv.second.full_thrusts) tsum += t;
              ROS_WARN("[SINK_DIAG_THRUST frame=%d] module=%d thrust_sum=%.3f thrusts=[%.3f,%.3f,%.3f,%.3f]",
                       unified_transition_count_, kv.first, tsum,
                       kv.second.full_thrusts.size() > 0 ? kv.second.full_thrusts[0] : 0.0f,
                       kv.second.full_thrusts.size() > 1 ? kv.second.full_thrusts[1] : 0.0f,
                       kv.second.full_thrusts.size() > 2 ? kv.second.full_thrusts[2] : 0.0f,
                       kv.second.full_thrusts.size() > 3 ? kv.second.full_thrusts[3] : 0.0f);
            }
          }
        }

        unified_transition_count_++;
      }

      ROS_INFO_THROTTLE(1.0, "[UnifiedCtrl LEADER] wrench_acc=(%.3f,%.3f,%.3f,%.4f,%.4f,%.4f) ok=%d",
                        target_wrench_acc(0), target_wrench_acc(1), target_wrench_acc(2),
                        target_wrench_acc(3), target_wrench_acc(4), target_wrench_acc(5), ok);

      // [SINK_DIAG] Steady-state Z tracking (1Hz) — shows if Z PID I term is building up,
      // and if formation altitude is converging to target
      ROS_WARN_THROTTLE(1.0, "[SINK_DIAG_Z_SS] pos_z=%.4f vel_z=%.3f form_pos_z=%.4f tgt_z=%.4f "
                        "Z_pid: p=%.4f i=%.4f d=%.4f sum=%.4f gravity_ff_z=%.4f boost=%d seed=%.4f(%s)",
                        pos_.z(), vel_.z(),
                        pos_.z() + (estimator_->getOrientation(Frame::COG, estimate_mode_) *
                                    tf::Vector3(unified_controller_->getFormationCogOffset().x(),
                                                unified_controller_->getFormationCogOffset().y(),
                                                unified_controller_->getFormationCogOffset().z())).z(),
                        target_pos_.z(),
                        pid_controllers_.at(Z).getPTerm(),
                        pid_controllers_.at(Z).getITerm(),
                        pid_controllers_.at(Z).getDTerm(),
                        pid_controllers_.at(Z).result(),
                        target_wrench_acc(2) - pid_controllers_.at(Z).result(),
                        z_ki_boost_count_,
                        last_unified_z_i_ss_,
                        has_unified_z_i_ss_ ? "adaptive" : "default");

      // Adaptive seed tracking: when in quasi-steady-state, low-pass update
      // last_unified_z_i_ss_ so the NEXT mode switch gets a better seed.
      // Conditions: boost finished, position error small, velocity small.
      if (z_ki_boost_count_ == 0 && z_integral_freeze_count_ == 0) {
        double form_pos_z = pos_.z() + (estimator_->getOrientation(Frame::COG, estimate_mode_) *
                            tf::Vector3(unified_controller_->getFormationCogOffset().x(),
                                        unified_controller_->getFormationCogOffset().y(),
                                        unified_controller_->getFormationCogOffset().z())).z();
        double err_z_abs = std::abs(target_pos_.z() - form_pos_z);
        double vel_z_abs = std::abs(vel_.z());
        if (err_z_abs < 0.05 && vel_z_abs < 0.05) {
          double current_z_i = pid_controllers_.at(Z).getITerm();
          last_unified_z_i_ss_ = (1.0 - Z_SEED_LPF_ALPHA) * last_unified_z_i_ss_
                                + Z_SEED_LPF_ALPHA * current_z_i;
          has_unified_z_i_ss_ = true;
        }
      }

      pre_module_state_ = module_state;
      return;  // Skip individual control path
    }

    // ======== Unified Control Mode: FOLLOWER ========
    // FOLLOWER receives thrust + gimbal commands from LEADER via ROS topics,
    // then forwards them to its own spinal. No local PID.
    if (unified_control_mode_ && module_state == FOLLOWER && module_state != SEPARATED) {
      if (!prev_unified_control_mode_) {
        // Zero spinal's attitude PID on FOLLOWER too
        sendZeroAttitudeGains();
        spinal_gains_zeroed_ = true;
        unified_transition_count_ = 0;
        ROS_WARN("[UnifiedCtrl] FOLLOWER id=%d entering unified mode, zeroed spinal gains, t=%.4f",
                 beetle_navigator_->getMyID(), ros::Time::now().toSec());
      }

      bool have_valid_cmd = false;
      if (unified_cmd_received_) {
        double age = (ros::Time::now() - unified_cmd_stamp_).toSec();
        if (age < 0.5) have_valid_cmd = true;
      }

      if (unified_transition_count_ >= 0 && unified_transition_count_ < 20) {
        tf::Vector3 fol_rpy = estimator_->getEuler(Frame::COG, estimate_mode_);
        ROS_WARN("[FOLLOWER_TRANS_DIAG id=%d frame=%d] t=%.4f have_cmd=%d rpy=(%.5f,%.5f,%.5f)",
                 beetle_navigator_->getMyID(), unified_transition_count_,
                 ros::Time::now().toSec(), have_valid_cmd,
                 fol_rpy.x(), fol_rpy.y(), fol_rpy.z());

        // [SINK_DIAG] Show what commands the FOLLOWER has/hasn't received
        if (have_valid_cmd && unified_thrust_cmd_.base_thrust.size() >= 4) {
          float tsum = 0;
          for (float t : unified_thrust_cmd_.base_thrust) tsum += t;
          ROS_WARN("[SINK_DIAG_FOL_CMD id=%d frame=%d] n_elements=%zu thrust_sum=%.3f first4=[%.3f,%.3f,%.3f,%.3f] "
                   "angles=[%.3f,%.3f,%.3f] age=%.4f",
                   beetle_navigator_->getMyID(), unified_transition_count_,
                   unified_thrust_cmd_.base_thrust.size(),
                   tsum,
                   unified_thrust_cmd_.base_thrust[0], unified_thrust_cmd_.base_thrust[1],
                   unified_thrust_cmd_.base_thrust[2], unified_thrust_cmd_.base_thrust[3],
                   unified_thrust_cmd_.angles[0], unified_thrust_cmd_.angles[1], unified_thrust_cmd_.angles[2],
                   (ros::Time::now() - unified_cmd_stamp_).toSec());
        }

        unified_transition_count_++;
      }

      if (have_valid_cmd) {
        // Forward vectoring force commands to own spinal
        follower_thrust_pub_.publish(unified_thrust_cmd_);
        // Forward gimbal angles
        follower_gimbal_pub_.publish(unified_gimbal_cmd_);
        follower_unified_active_ = true;

        ROS_INFO_THROTTLE(1.0, "[UnifiedCtrl FOLLOWER] id=%d forwarding: thrust_sz=%zu",
                          beetle_navigator_->getMyID(),
                          unified_thrust_cmd_.base_thrust.size());

        pre_module_state_ = module_state;
        prev_unified_control_mode_ = true;
        return;
      }

      // No valid command yet — fall through to independent control
      ROS_WARN("[UnifiedCtrl FOLLOWER] id=%d, no valid unified cmd yet — fallback to independent hover",
               beetle_navigator_->getMyID());
    }

    prev_unified_control_mode_ = false;
    follower_unified_active_ = false;
    spinal_gains_zeroed_ = false;
    restoreIndependentGains();  // restore per-module roll/pitch PID gains
    
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
    // Read unified_control_mode from rosparam at the START of update(),
    // so routing decisions below use the latest value (not stale from last cycle).
    {
      ros::NodeHandle control_nh(nh_, "controller");
      control_nh.getParam("unified_control_mode", unified_control_mode_);
    }

    if (unified_control_mode_) {
      int module_state = beetle_navigator_->getModuleState();

      // --- FOLLOWER without valid unified command: FREEZE (defensive) ---
      // Note: In practice this rarely triggers because LEADER publishes unified
      // commands before FOLLOWERs detect the mode switch. Kept as a safety net.
      if (module_state == FOLLOWER && module_state != SEPARATED) {
        bool have_valid_cmd = false;
        if (unified_cmd_received_) {
          double age = (ros::Time::now() - unified_cmd_stamp_).toSec();
          if (age < 0.5) have_valid_cmd = true;
        }

        if (!have_valid_cmd && !follower_unified_active_) {
          // Transition: zero spinal PID immediately, then freeze on last hover output.
          // Do NOT call GimbalrotorController::update() — that would run its own
          // attitude PID and publish competing commands on four_axes/command.
          if (!prev_unified_control_mode_) {
            sendZeroAttitudeGains();
            spinal_gains_zeroed_ = true;
            unified_transition_count_ = 0;
            ROS_WARN("[UnifiedCtrl] FOLLOWER id=%d freeze: zeroed spinal gains, awaiting unified cmd, t=%.4f",
                     beetle_navigator_->getMyID(), ros::Time::now().toSec());
          }
          prev_unified_control_mode_ = true;

          // Re-send cached independent hover commands to keep motors running
          if (has_cached_independent_cmd_) {
            // Override angles to zero (spinal PID is zeroed, these are ignored anyway)
            last_independent_thrust_cmd_.angles[0] = 0;
            last_independent_thrust_cmd_.angles[1] = 0;
            last_independent_thrust_cmd_.angles[2] = 0;
            follower_thrust_pub_.publish(last_independent_thrust_cmd_);
            follower_gimbal_pub_.publish(last_independent_gimbal_cmd_);
            ROS_WARN_THROTTLE(0.5, "[UnifiedCtrl] FOLLOWER id=%d freeze: re-sending cached hover cmd (thrust_sz=%zu)",
                              beetle_navigator_->getMyID(), last_independent_thrust_cmd_.base_thrust.size());
          } else {
            ROS_WARN_THROTTLE(0.5, "[UnifiedCtrl] FOLLOWER id=%d freeze: no cached cmd yet, waiting",
                              beetle_navigator_->getMyID());
          }

          pre_module_state_ = module_state;
          return true;  // Skip everything — no competing publish
        }
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
    bool result = GimbalrotorController::update();

    // Cache the commands that GimbalrotorController just published,
    // so we can freeze on them during unified mode transition.
    // Cache scalar thrusts (same format as sendFourAxisCommand with gimbal_calc_in_fc=false)
    // and gimbal angles separately.
    if (result) {
      last_independent_thrust_cmd_.base_thrust = target_full_thrust_;
      last_independent_thrust_cmd_.angles[0] = 0;
      last_independent_thrust_cmd_.angles[1] = 0;
      last_independent_thrust_cmd_.angles[2] = 0;

      last_independent_gimbal_cmd_.header.stamp = ros::Time::now();
      last_independent_gimbal_cmd_.position.clear();
      for (int i = 0; i < motor_num_; i++) {
        last_independent_gimbal_cmd_.position.push_back(target_gimbal_angles_.at(i));
      }
      has_cached_independent_cmd_ = true;
    }

    return result;
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
    // Send all-zero rpy/gain to this module's spinal.
    // This zeroes out thrust_p/i/d_gain_ inside spinal's AttitudeController,
    // so roll_pitch_term_ becomes 0 and spinal acts as a pure PWM executor.
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

  void BeetleController::applyUnifiedGains()
  {
    if (gains_switched_) return;  // already applied

    // Save current (independent) gains
    auto& roll_pid = pid_controllers_.at(ROLL);
    auto& pitch_pid = pid_controllers_.at(PITCH);
    saved_roll_gains_ = {roll_pid.getPGain(), roll_pid.getIGain(), roll_pid.getDGain(),
                         roll_pid.getLimitSum(), roll_pid.getLimitP(), roll_pid.getLimitI(), roll_pid.getLimitD()};
    saved_pitch_gains_ = {pitch_pid.getPGain(), pitch_pid.getIGain(), pitch_pid.getDGain(),
                          pitch_pid.getLimitSum(), pitch_pid.getLimitP(), pitch_pid.getLimitI(), pitch_pid.getLimitD()};

    // Apply unified (formation) gains
    roll_pid.setGains(unified_roll_gains_.p, unified_roll_gains_.i, unified_roll_gains_.d);
    roll_pid.setLimitSum(unified_roll_gains_.limit_sum);
    roll_pid.setLimitP(unified_roll_gains_.limit_p);
    roll_pid.setLimitI(unified_roll_gains_.limit_i);
    roll_pid.setLimitD(unified_roll_gains_.limit_d);

    pitch_pid.setGains(unified_pitch_gains_.p, unified_pitch_gains_.i, unified_pitch_gains_.d);
    pitch_pid.setLimitSum(unified_pitch_gains_.limit_sum);
    pitch_pid.setLimitP(unified_pitch_gains_.limit_p);
    pitch_pid.setLimitI(unified_pitch_gains_.limit_i);
    pitch_pid.setLimitD(unified_pitch_gains_.limit_d);

    gains_switched_ = true;
    ROS_WARN("[UnifiedCtrl] Applied unified roll/pitch gains: P=%.1f D=%.1f (was P=%.1f D=%.1f)",
             unified_pitch_gains_.p, unified_pitch_gains_.d,
             saved_pitch_gains_.p, saved_pitch_gains_.d);
  }

  void BeetleController::restoreIndependentGains()
  {
    if (!gains_switched_) return;  // nothing to restore

    auto& roll_pid = pid_controllers_.at(ROLL);
    auto& pitch_pid = pid_controllers_.at(PITCH);

    roll_pid.setGains(saved_roll_gains_.p, saved_roll_gains_.i, saved_roll_gains_.d);
    roll_pid.setLimitSum(saved_roll_gains_.limit_sum);
    roll_pid.setLimitP(saved_roll_gains_.limit_p);
    roll_pid.setLimitI(saved_roll_gains_.limit_i);
    roll_pid.setLimitD(saved_roll_gains_.limit_d);

    pitch_pid.setGains(saved_pitch_gains_.p, saved_pitch_gains_.i, saved_pitch_gains_.d);
    pitch_pid.setLimitSum(saved_pitch_gains_.limit_sum);
    pitch_pid.setLimitP(saved_pitch_gains_.limit_p);
    pitch_pid.setLimitI(saved_pitch_gains_.limit_i);
    pitch_pid.setLimitD(saved_pitch_gains_.limit_d);

    gains_switched_ = false;
    ROS_WARN("[UnifiedCtrl] Restored independent roll/pitch gains: P=%.1f D=%.1f",
             saved_pitch_gains_.p, saved_pitch_gains_.d);
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

    // Z I-term seed default for unified mode switch (Plan E')
    getParam<double>(control_nh, "z_i_seed_default", z_i_seed_default_, 0.8);
    last_unified_z_i_ss_ = z_i_seed_default_;

    // Load unified-mode PID gains for roll/pitch (formation pendulum compensation)
    ros::NodeHandle u_roll_nh(control_nh, "unified_roll");
    getParam<double>(u_roll_nh, "p_gain", unified_roll_gains_.p, 36.0);
    getParam<double>(u_roll_nh, "i_gain", unified_roll_gains_.i, 1.0);
    getParam<double>(u_roll_nh, "d_gain", unified_roll_gains_.d, 12.0);
    getParam<double>(u_roll_nh, "limit_sum", unified_roll_gains_.limit_sum, 50.0);
    getParam<double>(u_roll_nh, "limit_p", unified_roll_gains_.limit_p, 50.0);
    getParam<double>(u_roll_nh, "limit_i", unified_roll_gains_.limit_i, 10.0);
    getParam<double>(u_roll_nh, "limit_d", unified_roll_gains_.limit_d, 50.0);

    ros::NodeHandle u_pitch_nh(control_nh, "unified_pitch");
    getParam<double>(u_pitch_nh, "p_gain", unified_pitch_gains_.p, 36.0);
    getParam<double>(u_pitch_nh, "i_gain", unified_pitch_gains_.i, 1.0);
    getParam<double>(u_pitch_nh, "d_gain", unified_pitch_gains_.d, 12.0);
    getParam<double>(u_pitch_nh, "limit_sum", unified_pitch_gains_.limit_sum, 50.0);
    getParam<double>(u_pitch_nh, "limit_p", unified_pitch_gains_.limit_p, 50.0);
    getParam<double>(u_pitch_nh, "limit_i", unified_pitch_gains_.limit_i, 10.0);
    getParam<double>(u_pitch_nh, "limit_d", unified_pitch_gains_.limit_d, 50.0);
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
