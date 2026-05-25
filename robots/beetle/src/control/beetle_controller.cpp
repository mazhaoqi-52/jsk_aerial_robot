#include <beetle/control/beetle_controller.h>

#include <iomanip>
#include <limits>
#include <sstream>

using namespace std;

namespace aerial_robot_control
{
  BeetleController::BeetleController():
    GimbalrotorController(),
    pd_wrench_comp_mode_(false),
    pre_module_state_(SEPARATED),
    formation_desired_wrench_(Eigen::VectorXd::Zero(6)),
    unified_control_mode_(false),
    prev_unified_control_mode_(false),
    unified_cmd_received_(false),
    prev_navi_state_for_diag_(-1),
    unified_reference_wrench_acc_(Eigen::VectorXd::Zero(6)),
    unified_reference_desired_wrench_(Eigen::VectorXd::Zero(6)),
    unified_reference_yaw_pid_raw_(0.0),
    unified_reference_leader_id_(-1),
    unified_reference_warmup_count_(0),
    unified_reference_warmup_frames_(20),
    local_unified_cascade_setup_sent_(false),
    unified_torque_alloc_inv_pub_interval_(0.05),
    last_unified_torque_alloc_inv_pub_time_(-1.0),
    unified_internal_wrench_diag_(true),
    unified_internal_wrench_log_(true),
    unified_internal_wrench_log_period_(1.0),
    unified_internal_wrench_secondary_gain_(0.0),
    leader_target_pos_(0, 0, 0),
    leader_target_vel_(0, 0, 0),
    leader_target_acc_(0, 0, 0),
    leader_target_rpy_(0, 0, 0),
    leader_final_target_baselink_rpy_(0, 0, 0),
    leader_target_omega_(0, 0, 0),
    leader_target_ang_acc_(0, 0, 0),
    unified_transition_count_(-1),
    yaw_in_allocation_(false),
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
    // [Step D'] Pairwise observer disagreement diagnostic (leader-only publish).
    inter_disagreement_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("inter_disagreement", 1);
    desired_ext_wrench_sub_ = nh_.subscribe("desired_external_wrench", 1, &BeetleController::desiredExternalWrenchCallback, this);
    formation_desired_wrench_sub_ = nh_.subscribe("formation_desired_wrench", 1, &BeetleController::formationDesiredWrenchCallback, this);
    int max_modules_num = beetle_navigator_->getMaxModuleNum();
    for(int i = 0; i < max_modules_num; i++){
      std::string module_name  = string("/") + beetle_navigator_->getMyName() + std::to_string(i+1);
      est_wrench_subs_.insert(make_pair(module_name, nh_.subscribe( module_name + string("/tagged_wrench"), 1, &BeetleController::estExternalWrenchCallback, this)));
      Eigen::VectorXd wrench = Eigen::VectorXd::Zero(6);
      est_wrench_list_.insert(make_pair(i+1, wrench));
      inter_wrench_list_.insert(make_pair(i+1, wrench));
      wrench_comp_list_.insert(make_pair(i+1, wrench));
      est_wrench_task_list_.insert(make_pair(i+1, wrench));
      est_residual_list_.insert(make_pair(i+1, wrench));
      est_wrench_task_subs_.insert(make_pair(module_name, nh_.subscribe( module_name + string("/est_wrench_task"), 1, &BeetleController::estWrenchTaskCallback, this)));
      est_wrench_task_pubs_[i+1] = nh_.advertise<beetle::TaggedWrench>(module_name + string("/est_wrench_task"), 1);
      desired_ext_wrench_pubs_[i+1] = nh_.advertise<geometry_msgs::WrenchStamped>(module_name + string("/desired_external_wrench"), 1);
      module_model_subs_.insert(make_pair(module_name, nh_.subscribe(module_name + string("/unified_control/module_model"), 1,
                                                                     &BeetleController::moduleModelCallback, this)));
    }
    pid_controllers_.push_back(PID("f_x", wrench_comp_p_gain_, wrench_comp_i_gain_, wrench_comp_d_gain_));
    pid_controllers_.push_back(PID("f_y", wrench_comp_p_gain_, wrench_comp_i_gain_, wrench_comp_d_gain_));
    pid_controllers_.push_back(PID("f_z", wrench_comp_p_gain_, wrench_comp_i_gain_, wrench_comp_d_gain_));
    pid_controllers_.push_back(PID("t_x", wrench_comp_p_gain_, wrench_comp_i_gain_, wrench_comp_d_gain_));
    pid_controllers_.push_back(PID("t_y", wrench_comp_p_gain_, wrench_comp_i_gain_, wrench_comp_d_gain_));
    pid_controllers_.push_back(PID("t_z", wrench_comp_p_gain_, wrench_comp_i_gain_, wrench_comp_d_gain_));

    // Assemble debug publishers (global namespace)
    assemble_pid_pub_ = nh_.advertise<aerial_robot_msgs::PoseControlPid>("/assemble/debug/pose/pid", 10);
    assemble_vectoring_f_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/assemble/debug/vectoring_force", 1);
    assemble_formation_wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("/assemble/debug/formation_wrench", 1);
    // Initialize assemble_pid_msg_ arrays
    auto initPidField = [](aerial_robot_msgs::Pid& f) {
      f.total.resize(1, 0); f.p_term.resize(1, 0); f.i_term.resize(1, 0); f.d_term.resize(1, 0);
    };
    initPidField(assemble_pid_msg_.x);
    initPidField(assemble_pid_msg_.y);
    initPidField(assemble_pid_msg_.z);
    initPidField(assemble_pid_msg_.roll);
    initPidField(assemble_pid_msg_.pitch);
    initPidField(assemble_pid_msg_.yaw);

    ros::NodeHandle control_nh(nh_, "controller");
    ros::NodeHandle wrench_nh(control_nh, "wrench_comp");
    std::vector<int> indices = {FX, FY, FZ, TX, TY, TZ};
    pid_reconf_servers_.push_back(boost::make_shared<PidControlDynamicConfig>(wrench_nh));
    pid_reconf_servers_.back()->setCallback(boost::bind(&BeetleController::cfgPidCallback, this, _1, _2, indices));

    // Unified-mode XY/Z dynamic reconfigure servers
    ros::NodeHandle unified_xy_nh(control_nh, "unified_xy");
    unified_xy_reconf_server_ = boost::make_shared<PidControlDynamicConfig>(unified_xy_nh);
    unified_xy_reconf_server_->setCallback(boost::bind(&BeetleController::cfgUnifiedPidCallback, this, _1, _2, std::vector<int>{X, Y}, boost::ref(unified_xy_gains_)));

    ros::NodeHandle unified_z_nh(control_nh, "unified_z");
    unified_z_reconf_server_ = boost::make_shared<PidControlDynamicConfig>(unified_z_nh);
    unified_z_reconf_server_->setCallback(boost::bind(&BeetleController::cfgUnifiedPidCallback, this, _1, _2, std::vector<int>{Z}, boost::ref(unified_z_gains_)));

    ros::NodeHandle unified_yaw_nh(control_nh, "unified_yaw");
    unified_yaw_reconf_server_ = boost::make_shared<PidControlDynamicConfig>(unified_yaw_nh);
    unified_yaw_reconf_server_->setCallback(boost::bind(&BeetleController::cfgUnifiedPidCallback, this, _1, _2, std::vector<int>{YAW}, boost::ref(unified_yaw_gains_)));

    prev_comp_update_time_ = -1;

    // Initialize unified controller (unified_control_mode_ is read by rosParamInit)
    unified_controller_ = std::make_shared<BeetleUnifiedController>();
    unified_controller_->initialize(nh_, beetle_robot_model_, beetle_navigator_, estimator_);

    // Initialize formation-level momentum observer (Phase U2)
    formation_observer_ = std::make_shared<FormationMomentumObserver>();
    formation_observer_->initialize(nh_);

    unified_reference_pub_ = nh_.advertise<beetle::UnifiedControlReference>("unified_control/reference", 1);
    module_model_pub_ = nh_.advertise<beetle::ModuleModel>("unified_control/module_model", 1, true);
    // Publishers to this module's own spinal (same topic names as GimbalrotorController)
    follower_thrust_pub_ = nh_.advertise<spinal::FourAxisCommand>("four_axes/command", 1);
    follower_gimbal_pub_ = nh_.advertise<sensor_msgs::JointState>("gimbals_ctrl", 1);

    // Service for toggling unified control mode
    ros::NodeHandle srv_nh(nh_, "controller");
    set_unified_mode_srv_ = srv_nh.advertiseService("set_unified_mode",
                                                     &BeetleController::setUnifiedModeCb, this);
  }

  void BeetleController::resetToIndependentHover()
  {
    // Restore PID gains to pid_controllers_ (local state only, not yet sent)
    restoreIndependentGains();

    // ORDER MATTERS: send matrix FIRST, then gains.
    // spinal's rpyGainCallback() calls thrustGainMapping() which uses the
    // current torque_allocation_matrix_inv. If we send gains before the matrix,
    // thrustGainMapping() computes per-motor gains using the stale formation-level
    // matrix × independent-mode torque gains → wrong values for up to one frame.
    // By sending the matrix first, torqueAllocationMatrixInvCallback() stores it
    // AND calls thrustGainMapping() with the old torque_p/d_gain (cascade values).
    // Then rpyGainCallback() overwrites torque_p/d_gain and calls thrustGainMapping()
    // again with the now-correct matrix → correct per-motor gains immediately.
    sendTorqueAllocationMatrixInv();  // single-module alloc matrix (from GimbalrotorController)
    setAttitudeGains();               // independent-mode P/I/D gains

    // Reset target to current state (zero initial error)
    tf::Vector3 cur_pos = estimator_->getPos(Frame::COG, estimate_mode_);
    navigator_->setXyControlMode(aerial_robot_navigation::POS_CONTROL_MODE);
    navigator_->setTargetPosX(cur_pos.x());
    navigator_->setTargetPosY(cur_pos.y());
    navigator_->setTargetPosZ(cur_pos.z());
    navigator_->setTargetVelX(0);
    navigator_->setTargetVelY(0);

    double cur_yaw = estimator_->getEuler(Frame::COG, estimate_mode_).z();
    navigator_->setTargetYaw(cur_yaw);
    navigator_->setTargetOmegaZ(0);

    // Sync target_pos_candidate_ (CoM frame) = cur_pos(CoG) + com_conversion
    // to ensure convertTargetPosFromCoG2CoM() recovers the correct CoG target.
    tf::Transform cog2com_tf;
    tf::transformKDLToTF(beetle_navigator_->getCog2CoM<KDL::Frame>(), cog2com_tf);
    tf::Matrix3x3 cog_orient;
    tf::matrixEigenToTF(beetle_robot_model_->getCogDesireOrientation<Eigen::Matrix3d>(), cog_orient);
    tf::Vector3 com_conv = cog_orient * tf::Matrix3x3(tf::createQuaternionFromYaw(cur_yaw)) * cog2com_tf.getOrigin();
    beetle_navigator_->setTargetPosCandX(cur_pos.x() + com_conv.x());
    beetle_navigator_->setTargetPosCandY(cur_pos.y() + com_conv.y());
    beetle_navigator_->setTargetPosCandZ(cur_pos.z() + com_conv.z());
    beetle_navigator_->syncPreTargetPos();

    // Seed Z I-term with gravity compensation.
    // Skip if force landing / halt — robot is on ground.
    if (navigator_->getForceLandingFlag() ||
        navigator_->getNaviState() == aerial_robot_navigation::STOP_STATE) {
      pid_controllers_.at(Z).setErrI(0);
    } else {
      double gravity_acc = aerial_robot_estimation::G;
      double Ki_z = std::max(pid_controllers_.at(Z).getIGain(), 1e-6);
      double z_i_limit = pid_controllers_.at(Z).getLimitI() / Ki_z;
      double seeded_z_i = boost::algorithm::clamp(gravity_acc / Ki_z, -z_i_limit, z_i_limit);
      pid_controllers_.at(Z).setErrI(seeded_z_i);
    }

    // Clear RP/XY I-terms
    pid_controllers_.at(ROLL).setErrI(0);
    pid_controllers_.at(PITCH).setErrI(0);
    pid_controllers_.at(X).setErrI(0);
    pid_controllers_.at(Y).setErrI(0);

    // Reset wrench comp timer to avoid abnormal du on first post-exit frame
    prev_comp_update_time_ = -1;

    // Phase U2: deactivate formation observer on exiting unified mode
    if (formation_observer_) {
      formation_observer_->setActive(false);
      formation_observer_->reset();
      ROS_INFO("[UnifiedCtrl] Formation observer deactivated (reset + inactive)");
    }

    // Re-init single-module momentum observer on unified exit. The observer is
    // also used for short-term unified residual diagnostics, so do not carry
    // momentum/integral state across controller modes.
    prev_est_wrench_timestamp_ = 0;
    integrate_term_ = Eigen::VectorXd::Zero(6);
    est_external_wrench_ = Eigen::VectorXd::Zero(6);
    init_sum_momentum_ = Eigen::VectorXd::Zero(6);
    ROS_INFO("[UnifiedCtrl] Single-module observer re-initialized (timestamp/integrate/est zeroed)");

    // Clear formation-observer FF on mode exit to avoid stale values
    pid_controllers_.at(X).setPersistentFF(0.0);
    pid_controllers_.at(Y).setPersistentFF(0.0);
    pid_controllers_.at(Z).setPersistentFF(0.0);
    pid_controllers_.at(YAW).setPersistentFF(0.0);
    formation_desired_wrench_.setZero();
    clearInternalWrenchState();
  }

  void BeetleController::clearInternalWrenchState()
  {
    for (auto& kv : est_wrench_list_) kv.second = Eigen::VectorXd::Zero(6);
    for (auto& kv : est_residual_list_) kv.second = Eigen::VectorXd::Zero(6);
    for (auto& kv : inter_wrench_list_) kv.second = Eigen::VectorXd::Zero(6);
    for (auto& kv : wrench_comp_list_) kv.second = Eigen::VectorXd::Zero(6);
    if (unified_controller_) {
      unified_controller_->clearInternalWrenchSecondaryReference();
    }
  }

  void BeetleController::initUnifiedMode(bool is_leader)
  {
    tf::Vector3 cur_pos = estimator_->getPos(Frame::COG, estimate_mode_);
    unified_controller_->updateFormationGeometry();

    // Determine whether we are switching from stable hover (I-terms accumulated)
    // or starting from the ground (I-terms ≈ 0, no migration needed).
    bool from_hover = (navigator_->getNaviState() == aerial_robot_navigation::HOVER_STATE);

    navigator_->setXyControlMode(aerial_robot_navigation::POS_CONTROL_MODE);
    navigator_->setTargetVelX(0);
    navigator_->setTargetVelY(0);
    navigator_->setTargetAccX(0);
    navigator_->setTargetAccY(0);

    if (from_hover) {
      // Hover → Unified: reset target position to current state so PID starts with zero error.
      navigator_->setTargetPosX(cur_pos.x());
      navigator_->setTargetPosY(cur_pos.y());
      navigator_->setTargetPosZ(cur_pos.z());
    } else {
      // Ground start: preserve the navigator's existing target_pos_z (set by motorArming
      // to takeoff_height_). Only reset XY to current position. The takeoff ramp
      // in BaseNavigator needs target_z = takeoff_height to know where to climb to.
      navigator_->setTargetPosX(cur_pos.x());
      navigator_->setTargetPosY(cur_pos.y());
      // Do NOT override target_pos_z — keep the takeoff_height set by motorArming().
      ROS_INFO("[UnifiedCtrl] Ground start: preserving target_z=%.3f (takeoff_height), cur_z=%.3f",
               navigator_->getTargetPos().z(), cur_pos.z());
    }

    double cur_yaw = estimator_->getEuler(Frame::COG, estimate_mode_).z();
    navigator_->setTargetYaw(cur_yaw);
    navigator_->setTargetOmegaZ(0);
    beetle_navigator_->setUnifiedControlMode(true);

    // v4 architecture — outer ROLL/PITCH PID is inert in unified mode
    // (target_wrench_acc(3,4) ≡ 0; spinal owns full P+I+D), so just zero them.
    // XY I-terms are reset (target_pos = cur_pos above → fresh PID starts at 0 err).
    pid_controllers_.at(ROLL).setErrI(0);
    pid_controllers_.at(PITCH).setErrI(0);
    pid_controllers_.at(X).setErrI(0);
    pid_controllers_.at(Y).setErrI(0);

    if (from_hover) {
      // ===== Hover → Unified: Z I-term bumpless transfer (de-gravity) =====
      // Independent mode I-term implicitly carries gravity (≈ G). Unified mode has
      // explicit gravity FF in target_wrench_acc(2), so subtract gravity from the
      // I-term to keep the total Z command continuous across the switch.
      // This is the entire LF↔unified bumpless-transfer mechanism — no seed
      // bucket, no integral freeze, no Ki boost.
      double i_output_old = pid_controllers_.at(Z).getITerm();
      tf::Matrix3x3 uav_rot_mig = estimator_->getOrientation(Frame::COG, estimate_mode_);
      tf::Vector3 gravity_w_mig(0, 0, aerial_robot_estimation::G);
      tf::Vector3 gravity_cog_mig = uav_rot_mig.inverse() * gravity_w_mig;
      double gravity_ff_z = gravity_cog_mig.z();
      double i_output_degrav = i_output_old - gravity_ff_z;

      double Ki = std::max(pid_controllers_.at(Z).getIGain(), 1e-6);
      double iz_new = i_output_degrav / Ki;
      double iz_limit = pid_controllers_.at(Z).getLimitI() / Ki;
      if (!std::isfinite(iz_new)) iz_new = 0.0;
      iz_new = boost::algorithm::clamp(iz_new, -iz_limit, iz_limit);
      pid_controllers_.at(Z).setErrI(iz_new);

      ROS_WARN("[UnifiedCtrl] Z bumpless transfer (hover→unified): "
               "i_old=%.4f, gravity_ff=%.4f, i_degrav=%.4f, err_i=%.4f (limit=±%.1f)",
               i_output_old, gravity_ff_z, i_output_degrav, iz_new, iz_limit);
    } else {
      pid_controllers_.at(Z).setErrI(0);
      ROS_WARN("[UnifiedCtrl] Ground start: Z/RP/XY I-terms zeroed (naviState=%d)",
               navigator_->getNaviState());
    }

    if (is_leader) {
      sendCascadeSetup();     // reads from unified_*_gains_ directly, order-independent
      local_unified_cascade_setup_sent_ = true;  // leader one-shot is owned by BeetleUnifiedController
    } else {
      local_unified_cascade_setup_sent_ = false;
      sendFollowerCascadeSetup();  // follower: gimbal_dof now, alloc_inv/gains after matrix is ready
      ensureUnifiedReferenceSubscription();  // still subscribe for debug/monitoring
    }
    applyUnifiedGains();      // set unified PID gains into pid_controllers_ for PC loop
    unified_transition_count_ = 0;
    unified_reference_warmup_count_ = 0;
    last_unified_torque_alloc_inv_pub_time_ = ros::Time::now().toSec();

    // Phase U2: activate formation observer on entering unified LEADER mode.
    // Follower does NOT run observer (leader is the observation point).
    if (is_leader && formation_observer_) {
      formation_observer_->reset();
      formation_observer_->setActive(true);
      ROS_INFO("[UnifiedCtrl] Formation observer activated (reset + active)");
    }

    // Start the per-module momentum observer from the unified-mode state.
    // Its output feeds only the LF-style residual diagnostic by default; the
    // secondary allocation path remains disabled unless the gain is set > 0.
    prev_est_wrench_timestamp_ = 0;
    integrate_term_ = Eigen::VectorXd::Zero(6);
    est_external_wrench_ = Eigen::VectorXd::Zero(6);
    init_sum_momentum_ = Eigen::VectorXd::Zero(6);
    ROS_INFO("[UnifiedCtrl] Single-module observer reset for unified residual diagnostics");

    ROS_WARN("[UnifiedCtrl] %s id=%d mode switch: reset targets, %s, "
             "applied unified PID gains, starting local warmup window (%d frames), t=%.4f",
             is_leader ? "LEADER" : "FOLLOWER",
             beetle_navigator_->getMyID(),
             is_leader ? "sent/deferred cascade setup to all spinals"
                       : "deferred local alloc_inv/gains until matrix ready",
             unified_reference_warmup_frames_, ros::Time::now().toSec());
    formation_desired_wrench_.setZero();
    clearInternalWrenchState();
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

    // ======== Unified Control Mode (symmetric local control) ========
    // Both LEADER and FOLLOWER run the full 6-DOF outer PID + formation allocation
    // locally using their own estimator data. This eliminates the network/CPU delay
    // from leader→follower reference propagation and allows each module to react
    // to its own state (IMU/odom) without contamination by a stale peer reference.
    // LEADER additionally broadcasts a debug reference (for monitoring) and runs
    // the formation-level momentum observer.
    if (unified_control_mode_ &&
        (module_state == LEADER || module_state == FOLLOWER) &&
        module_state != SEPARATED) {
      bool is_leader = (module_state == LEADER);
      // Two independent edges trigger (re)initialization:
      //   - role_changed: module just became LEADER/FOLLOWER (assembly/reconfig)
      //   - mode_just_entered: unified mode just turned on this cycle
      bool role_changed = (pre_module_state_ != module_state);
      bool mode_just_entered = !prev_unified_control_mode_;
      if (role_changed || mode_just_entered) {
        initUnifiedMode(is_leader);
      }
      prev_unified_control_mode_ = true;
      runUnifiedControlCommon(is_leader);
      pre_module_state_ = module_state;
      return;
    }

    // (Legacy per-branch bodies removed — both leader and follower now share runUnifiedControlCommon.)

    // ======== Unified → Leader-Follower Transition (T4.4) ========
    // Runs ONLY when we were previously in unified mode and now exited.
    // If we never entered unified (prev_unified_control_mode_ == false), there
    // is nothing to restore — and crucially, we must NOT touch the rosparam,
    // otherwise a freshly-set service request can be silently overwritten.
    //
    // Restored items:
    //   1. spinal attitude PID gains (via restoreIndependentGains in resetToIndependentHover)
    //   2. target position → current position (avoid P-term spike)
    //   3. Z I-term seeded with gravity (avoid altitude drop)
    //   4. RP/XY I-terms cleared (unified I-term values meaningless for independent mode)
    //   5. target_pos_candidate_ synced (used by CoG→CoM conversion)
    if (prev_unified_control_mode_) {
      ROS_WARN("[UnifiedCtrl] id=%d exiting unified mode → restoring independent hover state",
               beetle_navigator_->getMyID());
      resetToIndependentHover();  // calls restoreIndependentGains() → clears gains_switched_

      prev_unified_control_mode_ = false;
      unified_controller_->resetCascadeAllocSent();
      unified_controller_->resetQPState();
      unified_reference_warmup_count_ = 0;
      local_unified_cascade_setup_sent_ = false;
      last_unified_torque_alloc_inv_pub_time_ = -1.0;
      beetle_navigator_->setUnifiedControlMode(false);

      ros::NodeHandle control_nh(nh_, "controller");
      control_nh.setParam("unified_control_mode", false);
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
      /* wrench_comp accumulates the parasitic residual (task prediction
         already subtracted upstream in calcInteractionWrench), so no
         separate formation_desired_wrench_ injection here — that would
         double-count the task component. */
      Eigen::VectorXd I_reconfig_acc_cog_term = Eigen::VectorXd::Zero(6);
      I_reconfig_acc_cog_term.head(3) = mass_inv * wrench_comp_term.head(3);
      I_reconfig_acc_cog_term.tail(3) = inertia_inv * wrench_comp_term.tail(3); //inavailable

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
        if (du < 0.0) du = 0.0;
        if (du > 0.1) du = 0.1;  // clamp callback-stall gaps
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
    }else{
      pid_controllers_.at(FX).reset();
      pid_controllers_.at(FY).reset();
      pid_controllers_.at(FZ).reset();
      pid_controllers_.at(TX).reset();
      pid_controllers_.at(TY).reset();
      pid_controllers_.at(TZ).reset();

      // Clear persistent FF (legacy LEADER task-FF injection removed; unified
      // mode handles task FF inside runUnifiedControlCommon).
      pid_controllers_.at(X).setPersistentFF(0.0);
      pid_controllers_.at(Y).setPersistentFF(0.0);
      pid_controllers_.at(Z).setPersistentFF(0.0);
      pid_controllers_.at(X).setICompTerm(0.0);
      pid_controllers_.at(Y).setICompTerm(0.0);
      pid_controllers_.at(Z).setICompTerm(0.0);
      pid_controllers_.at(ROLL).setICompTerm(0.0);
      pid_controllers_.at(PITCH).setICompTerm(0.0);
      pid_controllers_.at(YAW).setICompTerm(0.0);
    }
      
    GimbalrotorController::controlCore();

    pre_module_state_ = module_state;
    
  }

  bool BeetleController::update()
  {
    // unified_control_mode_ is the single source of truth. It is mutated only by:
    //   1. setUnifiedModeCb (explicit user/script intent)
    //   2. T4.3 below (force_landing/halt auto-exit)
    //   3. FOLLOWER auto-latch (callback + safety net below)
    // We deliberately do NOT re-read it from rosparam each cycle — doing so
    // would let any external setParam (config reload / other tools / our own
    // T4.4 cleanup writing back) silently flip the controller mode mid-flight.
    // The setParam writes elsewhere are now broadcast-only (for rqt/Python).

    // ======== One-shot takeoff diagnostic ========
    // On the rising edge into TAKEOFF_STATE, snapshot the unified-mode wiring
    // so silent mode mismatches (e.g. leader still in legacy while follower in
    // unified) are immediately visible in the log.
    {
      int navi_state = navigator_->getNaviState();
      if (navi_state == aerial_robot_navigation::TAKEOFF_STATE &&
          prev_navi_state_for_diag_ != aerial_robot_navigation::TAKEOFF_STATE) {
        ROS_WARN("[UnifiedCtrl] takeoff snapshot: id=%d module_state=%d unified_mode=%s prev_unified=%s",
                 beetle_navigator_->getMyID(),
                 beetle_navigator_->getModuleState(),
                 unified_control_mode_ ? "ON" : "OFF",
                 prev_unified_control_mode_ ? "ON" : "OFF");
      }
      prev_navi_state_for_diag_ = navi_state;
    }

    // ======== Auto-latch: FOLLOWER safety net ========
    // If this module is a FOLLOWER and has recently received unified references
    // from the LEADER, but unified_control_mode_ is false (e.g. rosparam was
    // overwritten by config reload, node restart, or race condition), force it on.
    // This is a secondary check complementing the callback-level auto-latch.
    if (!unified_control_mode_ && unified_cmd_received_ &&
        beetle_navigator_->getModuleState() == FOLLOWER)
    {
      double age = (ros::Time::now() - unified_cmd_stamp_).toSec();
      if (age < 0.5) {
        ROS_WARN_THROTTLE(1.0, "[UnifiedCtrl] FOLLOWER id=%d auto-latch in update(): "
                 "unified reference fresh (age=%.3fs) but mode is off — forcing ON",
                 beetle_navigator_->getMyID(), age);
        unified_control_mode_ = true;
        ros::NodeHandle ctrl_nh(nh_, "controller");
        ctrl_nh.setParam("unified_control_mode", true);
      }
    }

    // ======== T4.3: Auto-exit unified mode on force landing / halt ========
    // When LEADER detects force_landing or halt (STOP_STATE) while in unified mode,
    // immediately clear unified_control_mode_ so BOTH LEADER and all FOLLOWERs
    // fall through to the T4.4 exit path in controlCore(). This prevents the
    // dangerous scenario where LEADER stops publishing commands while FOLLOWERs
    // are still in hold-last phase with non-zero thrust on the ground.
    if (unified_control_mode_ && prev_unified_control_mode_) {
      int navi_state = navigator_->getNaviState();
      bool force_landing = navigator_->getForceLandingFlag();
      bool halt_state = (navi_state == aerial_robot_navigation::STOP_STATE);

      if (force_landing || halt_state) {
        ROS_ERROR("[T4.3] Auto-exit unified mode: %s detected — clearing unified_control_mode for all modules",
                  force_landing ? "FORCE_LANDING" : "HALT");
        unified_control_mode_ = false;
        // Write to rosparam so FOLLOWERs also pick up the change on next cycle
        ros::NodeHandle control_nh(nh_, "controller");
        control_nh.setParam("unified_control_mode", false);
      }
    }

    if (unified_control_mode_) {
      int module_state = beetle_navigator_->getModuleState();

      /* In unified control mode, every assembled module (LEADER and FOLLOWER)
         now runs the full controller locally in controlCore(). There is no
         wait-for-reference freeze anymore — each module produces its own
         thrust/gimbal commands from its own state.

         We still need:
           1. ControlBase::update() for activation/timing checks
           2. controlCore() for the local PID + unified allocation + pickup
           3. PoseLinearController::sendCmd() for PID debug publishing

         Bypass GimbalrotorController::update() which would publish competing
         four_axes/command from the single-module control path. */
      bool base_ok = ControlBase::update();
      if (!base_ok) {
        if (module_state == LEADER) {
          ROS_WARN_THROTTLE(0.5,
                            "[UnifiedCtrl LEADER] unified loop gated before controlCore: navi_state=%d control_timestamp=%.4f unified=%s module_state=%d",
                            navigator_->getNaviState(), control_timestamp_,
                            unified_control_mode_ ? "ON" : "OFF", module_state);
        }
        return false;
      }

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
    unified_reference_sub_.shutdown();
    unified_reference_leader_id_ = -1;
    unified_reference_warmup_count_ = 0;
    unified_controller_->clearFormationModelOverride();
    unified_controller_->clearInternalWrenchSecondaryReference();
  }

  void BeetleController::ensureUnifiedReferenceSubscription()
  {
    if (beetle_navigator_->getModuleState() != FOLLOWER) return;

    int leader_id = beetle_navigator_->getLeaderID();
    if (leader_id <= 0 || leader_id == beetle_navigator_->getMyID()) return;
    if (leader_id == unified_reference_leader_id_) return;

    std::string leader_ns = std::string("/") + beetle_navigator_->getMyName()
                            + std::to_string(leader_id);
    unified_reference_sub_.shutdown();
    unified_reference_sub_ = nh_.subscribe(leader_ns + "/unified_control/reference", 1,
                                           &BeetleController::unifiedReferenceCallback, this);
    unified_reference_leader_id_ = leader_id;
    unified_cmd_received_ = false;

    ROS_INFO("[UnifiedCtrl] FOLLOWER id=%d subscribed to unified reference: %s",
             beetle_navigator_->getMyID(),
             (leader_ns + "/unified_control/reference").c_str());
  }

  bool BeetleController::publishLocalUnifiedCommand()
  {
    spinal::FourAxisCommand local_cmd;
    if (!unified_controller_->buildModuleThrustCommand(beetle_navigator_->getMyID(), local_cmd)) {
      ROS_WARN_THROTTLE(0.5, "[UnifiedCtrl] id=%d failed to pick local module block from unified allocation",
                        beetle_navigator_->getMyID());
      return false;
    }

    unified_thrust_cmd_ = local_cmd;
    follower_thrust_pub_.publish(unified_thrust_cmd_);
    return true;
  }

  bool BeetleController::publishLocalUnifiedTorqueAllocationMatrixInv()
  {
    spinal::TorqueAllocationMatrixInv msg;
    if (!unified_controller_->buildModuleTorqueAllocationMatrixInv(beetle_navigator_->getMyID(), msg)) {
      ROS_WARN_THROTTLE(0.5, "[UnifiedCtrl] id=%d failed to build local torque_allocation_matrix_inv",
                        beetle_navigator_->getMyID());
      return false;
    }

    torque_allocation_matrix_inv_pub_.publish(msg);
    return true;
  }

  bool BeetleController::sendLocalUnifiedCascadeSetupOnce()
  {
    if (local_unified_cascade_setup_sent_) return true;

    if (!publishLocalUnifiedTorqueAllocationMatrixInv()) {
      return false;
    }

    sendFollowerCascadeGains();
    local_unified_cascade_setup_sent_ = true;
    ROS_WARN("[UnifiedCtrl] FOLLOWER id=%d one-shot local cascade setup: "
             "alloc_inv + gains sent to own spinal",
             beetle_navigator_->getMyID());
    return true;
  }

  void BeetleController::publishUnifiedReference(const Eigen::VectorXd& target_wrench_acc,
                                                 const Eigen::VectorXd& desired_wrench,
                                                 double yaw_pid_raw)
  {
    beetle::UnifiedControlReference msg;
    msg.header.stamp = ros::Time::now();
    msg.wrench_acc.force.x = target_wrench_acc(0);
    msg.wrench_acc.force.y = target_wrench_acc(1);
    msg.wrench_acc.force.z = target_wrench_acc(2);
    msg.wrench_acc.torque.x = target_wrench_acc(3);
    msg.wrench_acc.torque.y = target_wrench_acc(4);
    msg.wrench_acc.torque.z = target_wrench_acc(5);
    msg.desired_wrench.force.x = desired_wrench(0);
    msg.desired_wrench.force.y = desired_wrench(1);
    msg.desired_wrench.force.z = desired_wrench(2);
    msg.desired_wrench.torque.x = desired_wrench(3);
    msg.desired_wrench.torque.y = desired_wrench(4);
    msg.desired_wrench.torque.z = desired_wrench(5);
    msg.formation_mass = unified_controller_->getFormationMass();
    const Eigen::Vector3d& formation_cog_offset = unified_controller_->getFormationCogOffset();
    msg.formation_cog_offset.x = formation_cog_offset.x();
    msg.formation_cog_offset.y = formation_cog_offset.y();
    msg.formation_cog_offset.z = formation_cog_offset.z();
    const Eigen::Matrix3d& formation_inertia = unified_controller_->getFormationInertia();
    for (int r = 0; r < 3; r++) {
      for (int c = 0; c < 3; c++) {
        msg.formation_inertia[r * 3 + c] = formation_inertia(r, c);
      }
    }
    msg.yaw_pid_raw = yaw_pid_raw;

    // Phase B: broadcast leader's navigator setpoints. Followers reconstruct
    // their per-module reference from these via rigid-formation kinematics.
    // Keep PID target attitude and physical baselink attitude separate: unified
    // roll/pitch tilt lives in final_target_baselink_rpy, not target_rpy_.
    {
      tf::Vector3 lt_pos     = navigator_->getTargetPos();
      tf::Vector3 lt_vel     = navigator_->getTargetVel();
      tf::Vector3 lt_acc     = navigator_->getTargetAcc();
      tf::Vector3 lt_rpy     = navigator_->getTargetRPY();
      tf::Vector3 lt_final_baselink_rpy = beetle_navigator_->getFinalTargetBaselinkRPY();
      tf::Vector3 lt_omega   = navigator_->getTargetOmega();
      tf::Vector3 lt_ang_acc = navigator_->getTargetAngAcc();
      lt_final_baselink_rpy.setZ(lt_rpy.z());
      msg.leader_target_pos.x     = lt_pos.x();     msg.leader_target_pos.y     = lt_pos.y();     msg.leader_target_pos.z     = lt_pos.z();
      msg.leader_target_vel.x     = lt_vel.x();     msg.leader_target_vel.y     = lt_vel.y();     msg.leader_target_vel.z     = lt_vel.z();
      msg.leader_target_acc.x     = lt_acc.x();     msg.leader_target_acc.y     = lt_acc.y();     msg.leader_target_acc.z     = lt_acc.z();
      msg.leader_target_rpy.x     = lt_rpy.x();     msg.leader_target_rpy.y     = lt_rpy.y();     msg.leader_target_rpy.z     = lt_rpy.z();
      msg.leader_final_target_baselink_rpy.x = lt_final_baselink_rpy.x();
      msg.leader_final_target_baselink_rpy.y = lt_final_baselink_rpy.y();
      msg.leader_final_target_baselink_rpy.z = lt_final_baselink_rpy.z();
      msg.leader_target_omega.x   = lt_omega.x();   msg.leader_target_omega.y   = lt_omega.y();   msg.leader_target_omega.z   = lt_omega.z();
      msg.leader_target_ang_acc.x = lt_ang_acc.x(); msg.leader_target_ang_acc.y = lt_ang_acc.y(); msg.leader_target_ang_acc.z = lt_ang_acc.z();
    }

    unified_reference_pub_.publish(msg);

    ROS_DEBUG_THROTTLE(1.0,
                       "[UnifiedCtrl REF_PUB] leader_id=%d stamp=%.4f mass=%.3f wrench_z=%.3f pitch_i=%.3f yaw_raw=%.3f",
                       beetle_navigator_->getMyID(),
                       msg.header.stamp.toSec(),
                       msg.formation_mass,
                       target_wrench_acc(2),
                       target_wrench_acc(4),
                       yaw_pid_raw);
  }

  void BeetleController::publishModuleModel()
  {
    if (!beetle_robot_model_ || !unified_controller_) return;

    BeetleUnifiedController::ModuleModelDescriptor model;
    model.mass = beetle_robot_model_->getMass();
    model.inertia = beetle_robot_model_->getInertia<Eigen::Matrix3d>();
    model.rotor_origins_from_cog =
        beetle_robot_model_->getRotorsOriginFromCog<Eigen::Vector3d>();
    model.rotor_direction = beetle_robot_model_->getRotorDirection();
    model.mf_rate = beetle_robot_model_->getMFRate();

    const int my_id = beetle_navigator_->getMyID();
    const int rotor_num = beetle_robot_model_->getRotorNum();
    if (!model.valid(rotor_num)) {
      ROS_WARN_THROTTLE(1.0,
                        "[UnifiedCtrl] ModuleModel publish blocked id=%d mass=%.6f inertia_finite=%d rotor_origins=%zu rotor_dirs=%zu rotor_num=%d mf_rate=%.6f",
                        my_id, model.mass, model.inertia.allFinite(),
                        model.rotor_origins_from_cog.size(), model.rotor_direction.size(),
                        rotor_num, model.mf_rate);
      return;
    }

    unified_controller_->setModuleModelDescriptor(my_id, model);

    beetle::ModuleModel msg;
    msg.header.stamp = ros::Time::now();
    msg.id = my_id;
    msg.mass = model.mass;
    for (int r = 0; r < 3; r++) {
      for (int c = 0; c < 3; c++) {
        msg.inertia[r * 3 + c] = model.inertia(r, c);
      }
    }
    msg.rotor_origin_from_cog.resize(model.rotor_origins_from_cog.size());
    for (size_t i = 0; i < model.rotor_origins_from_cog.size(); i++) {
      msg.rotor_origin_from_cog[i].x = model.rotor_origins_from_cog[i].x();
      msg.rotor_origin_from_cog[i].y = model.rotor_origins_from_cog[i].y();
      msg.rotor_origin_from_cog[i].z = model.rotor_origins_from_cog[i].z();
    }
    msg.rotor_direction.resize(rotor_num);
    for (int r = 0; r < rotor_num; r++) {
      msg.rotor_direction[r] = static_cast<int8_t>(model.rotor_direction.at(r + 1));
    }
    msg.mf_rate = model.mf_rate;
    module_model_pub_.publish(msg);
    ROS_INFO_THROTTLE(5.0,
                      "[UnifiedCtrl] Published ModuleModel snapshot id=%d mass=%.3f rotors=%zu dirs=%zu mf_rate=%.6f",
                      my_id, msg.mass, msg.rotor_origin_from_cog.size(),
                      msg.rotor_direction.size(), msg.mf_rate);
  }

  void BeetleController::moduleModelCallback(const beetle::ModuleModel& msg)
  {
    if (!unified_controller_ || msg.id == 0) {
      ROS_WARN_THROTTLE(1.0,
                        "[UnifiedCtrl] Reject ModuleModel msg: controller_ready=%d id=%d mass=%.6f",
                        static_cast<int>(static_cast<bool>(unified_controller_)),
                        msg.id, msg.mass);
      return;
    }

    BeetleUnifiedController::ModuleModelDescriptor model;
    model.mass = msg.mass;
    for (int r = 0; r < 3; r++) {
      for (int c = 0; c < 3; c++) {
        model.inertia(r, c) = msg.inertia[r * 3 + c];
      }
    }
    model.rotor_origins_from_cog.resize(msg.rotor_origin_from_cog.size());
    for (size_t i = 0; i < msg.rotor_origin_from_cog.size(); i++) {
      model.rotor_origins_from_cog[i] =
          Eigen::Vector3d(msg.rotor_origin_from_cog[i].x,
                          msg.rotor_origin_from_cog[i].y,
                          msg.rotor_origin_from_cog[i].z);
    }
    for (size_t i = 0; i < msg.rotor_direction.size(); i++) {
      model.rotor_direction[static_cast<int>(i) + 1] = msg.rotor_direction[i];
    }
    model.mf_rate = msg.mf_rate;

    if (!model.valid(static_cast<int>(msg.rotor_direction.size()))) {
      ROS_WARN_THROTTLE(1.0,
                        "[UnifiedCtrl] Reject invalid ModuleModel msg id=%d mass=%.6f inertia_finite=%d rotors=%zu dirs=%zu mf_rate=%.6f",
                        msg.id, model.mass, model.inertia.allFinite(),
                        model.rotor_origins_from_cog.size(), model.rotor_direction.size(),
                        model.mf_rate);
      return;
    }

    unified_controller_->setModuleModelDescriptor(msg.id, model);
    ROS_INFO_THROTTLE(5.0,
                      "[UnifiedCtrl] Accepted ModuleModel msg id=%d mass=%.3f rotors=%zu dirs=%zu",
                      msg.id, msg.mass, msg.rotor_origin_from_cog.size(),
                      msg.rotor_direction.size());
  }

  void BeetleController::unifiedReferenceCallback(const beetle::UnifiedControlReference& msg)
  {
    unified_reference_wrench_acc_(0) = msg.wrench_acc.force.x;
    unified_reference_wrench_acc_(1) = msg.wrench_acc.force.y;
    unified_reference_wrench_acc_(2) = msg.wrench_acc.force.z;
    unified_reference_wrench_acc_(3) = msg.wrench_acc.torque.x;
    unified_reference_wrench_acc_(4) = msg.wrench_acc.torque.y;
    unified_reference_wrench_acc_(5) = msg.wrench_acc.torque.z;

    unified_reference_desired_wrench_(0) = msg.desired_wrench.force.x;
    unified_reference_desired_wrench_(1) = msg.desired_wrench.force.y;
    unified_reference_desired_wrench_(2) = msg.desired_wrench.force.z;
    unified_reference_desired_wrench_(3) = msg.desired_wrench.torque.x;
    unified_reference_desired_wrench_(4) = msg.desired_wrench.torque.y;
    unified_reference_desired_wrench_(5) = msg.desired_wrench.torque.z;
    unified_reference_yaw_pid_raw_ = msg.yaw_pid_raw;

    // Phase B: cache leader's navigator setpoints for follower target derivation.
    leader_target_pos_.setValue(msg.leader_target_pos.x,
                                msg.leader_target_pos.y,
                                msg.leader_target_pos.z);
    leader_target_vel_.setValue(msg.leader_target_vel.x,
                                msg.leader_target_vel.y,
                                msg.leader_target_vel.z);
    leader_target_acc_.setValue(msg.leader_target_acc.x,
                                msg.leader_target_acc.y,
                                msg.leader_target_acc.z);
    leader_target_rpy_.setValue(msg.leader_target_rpy.x,
                                msg.leader_target_rpy.y,
                                msg.leader_target_rpy.z);
    leader_final_target_baselink_rpy_.setValue(msg.leader_final_target_baselink_rpy.x,
                                               msg.leader_final_target_baselink_rpy.y,
                                               msg.leader_final_target_baselink_rpy.z);
    leader_target_omega_.setValue(msg.leader_target_omega.x,
                                  msg.leader_target_omega.y,
                                  msg.leader_target_omega.z);
    leader_target_ang_acc_.setValue(msg.leader_target_ang_acc.x,
                                    msg.leader_target_ang_acc.y,
                                    msg.leader_target_ang_acc.z);

    unified_cmd_received_ = true;
    unified_cmd_stamp_ = msg.header.stamp.isZero() ? ros::Time::now() : msg.header.stamp;

    ROS_DEBUG_THROTTLE(1.0,
              "[UnifiedCtrl REF_RX] follower_id=%d leader_id=%d age=%.4f wrench_z=%.3f pitch_i=%.3f yaw_raw=%.3f",
              beetle_navigator_->getMyID(),
              beetle_navigator_->getLeaderID(),
              (ros::Time::now() - unified_cmd_stamp_).toSec(),
              unified_reference_wrench_acc_(2),
              unified_reference_wrench_acc_(4),
              unified_reference_yaw_pid_raw_);

    // Auto-latch: if this FOLLOWER receives a unified reference from the LEADER
    // but unified_control_mode_ is false (e.g. rosparam was overwritten by a
    // config reload or node restart), force-enable unified mode.
    // This prevents the dangerous scenario where the FOLLOWER silently runs
    // its independent controller while the LEADER expects unified coordination.
    if (!unified_control_mode_ &&
        beetle_navigator_->getModuleState() == FOLLOWER)
    {
      ROS_WARN("[UnifiedCtrl] FOLLOWER id=%d AUTO-LATCH: received unified reference "
               "while unified_control_mode is false — forcing unified mode ON",
               beetle_navigator_->getMyID());
      unified_control_mode_ = true;
      ros::NodeHandle control_nh(nh_, "controller");
      control_nh.setParam("unified_control_mode", true);
    }
  }

  void BeetleController::sendCascadeSetup()
  {
    // LEADER-only: send torque allocation matrix inverse and P/I/D gains
    // to ALL assembled modules' spinals. This configures each spinal for 1000Hz
    // P+I+D attitude tracking using thrustGainMapping().
    //
    // v5 architecture: spinal owns the high-bandwidth P+D inner loop only.
    // I-term is integrated by PC's outer R/P PID and fed via target_wrench_acc(3,4).
    // Therefore roll_i / pitch_i sent to spinal are forced to 0 here, regardless
    // of the unified-mode YAML I gain (which IS used for the PC outer integrator).
    //
    // PC's outer roll/pitch PIDs are now ACTIVE in unified mode (mirrors beetle
    // independent mode with i_term_rp_calc_in_pc=true).
    //
    // Gains are read directly from unified_*_gains_ (not pid_controllers_).
    // This eliminates call-order dependency: sendCascadeSetup() always reads
    // the correct unified gains regardless of whether applyUnifiedGains() has
    // been called yet.
    //
    // NOTE: At mode-switch time, integrated_map_inv_rot_ may not be computed yet.
    // The one-shot logic inside computeUnifiedAllocation() will resend on first
    // successful computation. See cascade_alloc_sent_ flag.

    double roll_p  = unified_roll_gains_.p;
    double roll_i  = 0.0;  // v5: PC owns I; spinal P+D only
    double roll_d  = unified_roll_gains_.d;
    double pitch_p = unified_pitch_gains_.p;
    double pitch_i = 0.0;  // v5: PC owns I; spinal P+D only
    double pitch_d = unified_pitch_gains_.d;
    double yaw_d   = unified_yaw_gains_.d;

    // Cache gains for deferred one-shot resend (must be done BEFORE the attempt)
    unified_controller_->cacheCascadeGains(roll_p, roll_i, roll_d,
                                           pitch_p, pitch_i, pitch_d, yaw_d);

    // Attempt to send now. If matrix is not yet computed, skip gains too —
    // sending cascade gains with the old independent-mode allocation matrix
    // causes thrustGainMapping() to produce wrong per-motor gains (P5 fix).
    unified_controller_->updateFormationGeometry();
    bool matrix_sent = unified_controller_->sendTorqueAllocationMatrixInv();
    if (matrix_sent) {
      unified_controller_->sendCascadeGains(roll_p, roll_i, roll_d,
                                            pitch_p, pitch_i, pitch_d, yaw_d);
    } else {
      ROS_WARN("[UnifiedCtrl] Cascade setup: matrix not ready, deferring gains to one-shot");
    }

    // Publish gimbal_dof=1 to own spinal (LEADER).
    {
      std_msgs::UInt8 gimbal_dof_msg;
      gimbal_dof_msg.data = 1;
      gimbal_dof_pub_.publish(gimbal_dof_msg);
    }

    ROS_INFO("[UnifiedCtrl] Cascade setup (LEADER): alloc_inv %s, gains %s, "
             "(P_r=%.1f I_r=%.2f D_r=%.1f P_p=%.1f I_p=%.2f D_p=%.1f D_y=%.1f) %zu modules + gimbal_dof=1",
             matrix_sent ? "SENT" : "DEFERRED", matrix_sent ? "SENT" : "DEFERRED",
             roll_p, roll_i, roll_d, pitch_p, pitch_i, pitch_d, yaw_d,
             beetle_navigator_->getAssemblyIds().size());
  }

  void BeetleController::sendFollowerCascadeSetup()
  {
    // Set gimbal_dof=1 on own spinal
    {
      std_msgs::UInt8 gimbal_dof_msg;
      gimbal_dof_msg.data = 1;
      gimbal_dof_pub_.publish(gimbal_dof_msg);
    }

    ROS_INFO("[UnifiedCtrl] Cascade setup (FOLLOWER id=%d): gimbal_dof=1 sent, "
             "alloc_inv/gains deferred until local allocation matrix is ready",
             beetle_navigator_->getMyID());
  }

  void BeetleController::sendFollowerCascadeGains()
  {
    double roll_p  = unified_roll_gains_.p;
    double roll_i  = 0.0;  // v5: PC owns I; spinal P+D only
    double roll_d  = unified_roll_gains_.d;
    double pitch_p = unified_pitch_gains_.p;
    double pitch_i = 0.0;  // v5: PC owns I; spinal P+D only
    double pitch_d = unified_pitch_gains_.d;
    double yaw_d   = unified_yaw_gains_.d;

    spinal::RollPitchYawTerms rpy_gain_msg;
    rpy_gain_msg.motors.resize(1);  // torque-level path
    rpy_gain_msg.motors[0].roll_p  = static_cast<int16_t>(roll_p  * 1000);
    rpy_gain_msg.motors[0].roll_i  = static_cast<int16_t>(roll_i  * 1000);
    rpy_gain_msg.motors[0].roll_d  = static_cast<int16_t>(roll_d  * 1000);
    rpy_gain_msg.motors[0].pitch_p = static_cast<int16_t>(pitch_p * 1000);
    rpy_gain_msg.motors[0].pitch_i = static_cast<int16_t>(pitch_i * 1000);
    rpy_gain_msg.motors[0].pitch_d = static_cast<int16_t>(pitch_d * 1000);
    rpy_gain_msg.motors[0].yaw_d   = static_cast<int16_t>(yaw_d   * 1000);
    rpy_gain_pub_.publish(rpy_gain_msg);

    ROS_INFO("[UnifiedCtrl] FOLLOWER id=%d sent cascade gains "
             "(P_r=%.1f I_r=%.2f D_r=%.1f P_p=%.1f I_p=%.2f D_p=%.1f D_y=%.1f) to own spinal",
             beetle_navigator_->getMyID(),
             roll_p, roll_i, roll_d, pitch_p, pitch_i, pitch_d, yaw_d);
  }

  void BeetleController::applyUnifiedGains()
  {
    if (gains_switched_) return;  // already applied

    // Save current (independent) gains
    auto& roll_pid = pid_controllers_.at(ROLL);
    auto& pitch_pid = pid_controllers_.at(PITCH);
    auto& x_pid = pid_controllers_.at(X);
    auto& y_pid = pid_controllers_.at(Y);
    auto& z_pid = pid_controllers_.at(Z);
    auto& yaw_pid = pid_controllers_.at(YAW);
    saved_roll_gains_ = {roll_pid.getPGain(), roll_pid.getIGain(), roll_pid.getDGain(),
                         roll_pid.getLimitSum(), roll_pid.getLimitP(), roll_pid.getLimitI(), roll_pid.getLimitD(),
                         roll_pid.getErrDLpfCutoffFreq()};
    saved_pitch_gains_ = {pitch_pid.getPGain(), pitch_pid.getIGain(), pitch_pid.getDGain(),
                          pitch_pid.getLimitSum(), pitch_pid.getLimitP(), pitch_pid.getLimitI(), pitch_pid.getLimitD(),
                          pitch_pid.getErrDLpfCutoffFreq()};
    saved_xy_gains_ = {x_pid.getPGain(), x_pid.getIGain(), x_pid.getDGain(),
                       x_pid.getLimitSum(), x_pid.getLimitP(), x_pid.getLimitI(), x_pid.getLimitD(),
                       x_pid.getErrDLpfCutoffFreq()};
    saved_z_gains_ = {z_pid.getPGain(), z_pid.getIGain(), z_pid.getDGain(),
                      z_pid.getLimitSum(), z_pid.getLimitP(), z_pid.getLimitI(), z_pid.getLimitD(),
                      z_pid.getErrDLpfCutoffFreq()};
    saved_yaw_gains_ = {yaw_pid.getPGain(), yaw_pid.getIGain(), yaw_pid.getDGain(),
                        yaw_pid.getLimitSum(), yaw_pid.getLimitP(), yaw_pid.getLimitI(), yaw_pid.getLimitD(),
                        yaw_pid.getErrDLpfCutoffFreq()};

    // Apply unified (formation) gains.
    // Full P/I/D for roll/pitch: P+D are sent to each spinal for 1000Hz inner-loop
    // tracking via sendCascadeSetup(). The PC wrench only uses getITerm() for
    // roll/pitch, so having P/D here doesn't affect the PC wrench output — they
    // exist solely so sendCascadeSetup() can read from pid_controllers_ directly.
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

    // Apply unified XY gains
    x_pid.setGains(unified_xy_gains_.p, unified_xy_gains_.i, unified_xy_gains_.d);
    x_pid.setLimitSum(unified_xy_gains_.limit_sum);
    x_pid.setLimitP(unified_xy_gains_.limit_p);
    x_pid.setLimitI(unified_xy_gains_.limit_i);
    x_pid.setLimitD(unified_xy_gains_.limit_d);
    y_pid.setGains(unified_xy_gains_.p, unified_xy_gains_.i, unified_xy_gains_.d);
    y_pid.setLimitSum(unified_xy_gains_.limit_sum);
    y_pid.setLimitP(unified_xy_gains_.limit_p);
    y_pid.setLimitI(unified_xy_gains_.limit_i);
    y_pid.setLimitD(unified_xy_gains_.limit_d);

    // Apply unified D-term LPF cutoff if configured (0 = keep existing)
    if (unified_xy_gains_.err_d_lpf_cutoff_freq > 0.0) {
      x_pid.setErrDLpfCutoffFreq(unified_xy_gains_.err_d_lpf_cutoff_freq);
      y_pid.setErrDLpfCutoffFreq(unified_xy_gains_.err_d_lpf_cutoff_freq);
    }

    // Apply unified Z gains
    z_pid.setGains(unified_z_gains_.p, unified_z_gains_.i, unified_z_gains_.d);
    z_pid.setLimitSum(unified_z_gains_.limit_sum);
    z_pid.setLimitP(unified_z_gains_.limit_p);
    z_pid.setLimitI(unified_z_gains_.limit_i);
    z_pid.setLimitD(unified_z_gains_.limit_d);

    // Apply unified Yaw gains (full P+I+D on PC)
    yaw_pid.setGains(unified_yaw_gains_.p, unified_yaw_gains_.i, unified_yaw_gains_.d);
    yaw_pid.setLimitSum(unified_yaw_gains_.limit_sum);
    yaw_pid.setLimitP(unified_yaw_gains_.limit_p);
    yaw_pid.setLimitI(unified_yaw_gains_.limit_i);
    yaw_pid.setLimitD(unified_yaw_gains_.limit_d);

    gains_switched_ = true;
    ROS_WARN("[UnifiedCtrl] Applied unified gains: roll P=%.1f I=%.1f D=%.1f, pitch P=%.1f I=%.1f D=%.1f, "
             "xy P=%.1f I=%.1f D=%.1f, z P=%.1f I=%.1f D=%.1f, yaw P=%.1f I=%.1f D=%.1f",
             unified_roll_gains_.p, unified_roll_gains_.i, unified_roll_gains_.d,
             unified_pitch_gains_.p, unified_pitch_gains_.i, unified_pitch_gains_.d,
             unified_xy_gains_.p, unified_xy_gains_.i, unified_xy_gains_.d,
             unified_z_gains_.p, unified_z_gains_.i, unified_z_gains_.d,
             unified_yaw_gains_.p, unified_yaw_gains_.i, unified_yaw_gains_.d);
  }

  void BeetleController::restoreIndependentGains()
  {
    if (!gains_switched_) return;  // nothing to restore

    auto& roll_pid = pid_controllers_.at(ROLL);
    auto& pitch_pid = pid_controllers_.at(PITCH);
    auto& x_pid = pid_controllers_.at(X);
    auto& y_pid = pid_controllers_.at(Y);
    auto& z_pid = pid_controllers_.at(Z);
    auto& yaw_pid = pid_controllers_.at(YAW);

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

    // Restore independent XY gains
    x_pid.setGains(saved_xy_gains_.p, saved_xy_gains_.i, saved_xy_gains_.d);
    x_pid.setLimitSum(saved_xy_gains_.limit_sum);
    x_pid.setLimitP(saved_xy_gains_.limit_p);
    x_pid.setLimitI(saved_xy_gains_.limit_i);
    x_pid.setLimitD(saved_xy_gains_.limit_d);
    y_pid.setGains(saved_xy_gains_.p, saved_xy_gains_.i, saved_xy_gains_.d);
    y_pid.setLimitSum(saved_xy_gains_.limit_sum);
    y_pid.setLimitP(saved_xy_gains_.limit_p);
    y_pid.setLimitI(saved_xy_gains_.limit_i);
    y_pid.setLimitD(saved_xy_gains_.limit_d);
    x_pid.setErrDLpfCutoffFreq(saved_xy_gains_.err_d_lpf_cutoff_freq);
    y_pid.setErrDLpfCutoffFreq(saved_xy_gains_.err_d_lpf_cutoff_freq);

    // Restore independent Z gains
    z_pid.setGains(saved_z_gains_.p, saved_z_gains_.i, saved_z_gains_.d);
    z_pid.setLimitSum(saved_z_gains_.limit_sum);
    z_pid.setLimitP(saved_z_gains_.limit_p);
    z_pid.setLimitI(saved_z_gains_.limit_i);
    z_pid.setLimitD(saved_z_gains_.limit_d);

    // Restore independent Yaw gains
    yaw_pid.setGains(saved_yaw_gains_.p, saved_yaw_gains_.i, saved_yaw_gains_.d);
    yaw_pid.setLimitSum(saved_yaw_gains_.limit_sum);
    yaw_pid.setLimitP(saved_yaw_gains_.limit_p);
    yaw_pid.setLimitI(saved_yaw_gains_.limit_i);
    yaw_pid.setLimitD(saved_yaw_gains_.limit_d);

    gains_switched_ = false;
    ROS_WARN("[UnifiedCtrl] Restored independent gains: pitch P=%.1f D=%.1f, xy P=%.1f D=%.1f, z P=%.1f D=%.1f, yaw P=%.1f D=%.1f",
             saved_pitch_gains_.p, saved_pitch_gains_.d,
             saved_xy_gains_.p, saved_xy_gains_.d,
             saved_z_gains_.p, saved_z_gains_.d,
             saved_yaw_gains_.p, saved_yaw_gains_.d);
  }

  void BeetleController::cfgUnifiedPidCallback(aerial_robot_control::PIDConfig &config, uint32_t level, std::vector<int> controller_indices, AxisGainSet& gain_set)
  {
    using Levels = aerial_robot_msgs::DynamicReconfigureLevels;
    if(!config.pid_control_flag) return;

    switch(level)
      {
      case Levels::RECONFIGURE_P_GAIN:
        gain_set.p = config.p_gain;
        break;
      case Levels::RECONFIGURE_I_GAIN:
        gain_set.i = config.i_gain;
        break;
      case Levels::RECONFIGURE_D_GAIN:
        gain_set.d = config.d_gain;
        break;
      default:
        return;
      }

    // If unified mode is active, push the new gain to live pid_controllers_ immediately
    if (gains_switched_)
      {
        for (const auto& index : controller_indices)
          {
            switch(level)
              {
              case Levels::RECONFIGURE_P_GAIN:
                pid_controllers_.at(index).setPGain(config.p_gain);
                break;
              case Levels::RECONFIGURE_I_GAIN:
                pid_controllers_.at(index).setIGain(config.i_gain);
                break;
              case Levels::RECONFIGURE_D_GAIN:
                pid_controllers_.at(index).setDGain(config.d_gain);
                break;
              }
            ROS_INFO_STREAM("[UnifiedCtrl] change unified gain for controller '" << pid_controllers_.at(index).getName() << "'");
          }

        // Resend P/D gains when unified roll/pitch/yaw gains change. Followers
        // keep the same matrix-first ordering via their local one-shot path.
        if (level == Levels::RECONFIGURE_P_GAIN || level == Levels::RECONFIGURE_D_GAIN) {
          if (unified_controller_) {
            if (beetle_navigator_->getModuleState() == FOLLOWER) {
              local_unified_cascade_setup_sent_ = false;
              ROS_INFO("[UnifiedCtrl] Marked follower local cascade setup dirty after dynreconf P/D change");
            } else {
              sendCascadeSetup();
              ROS_INFO("[UnifiedCtrl] Resent cascade gains to Spinals after dynreconf P/D change");
            }
          }
        }
      }
  }

  void BeetleController::calcInteractionWrench()
  {
    /* 1. Subtract per-module observer task prediction up-front.
     *
     *    est_residual_list_[i] = est_wrench_list_[i] - est_wrench_task_list_[i]
     *
     *    Theory: observer output ŷ_i = c_i + d_i + b_i (joint force + direct
     *    external + parasitic). Task prediction ŷ_i^task models the c_i+d_i
     *    induced by the active task. Their difference is the parasitic-only
     *    component b_i (plus modelling error). All downstream products
     *    (W_w, inter_wrench_list_, wrench_comp_list_) are computed from
     *    est_residual_list_ so they are naturally parasitic by construction.
     *
     *    Sum invariant: Σ est_wrench_task_list_[i] = W_ext (Newton 2nd on
     *    whole formation). With matching Σ est_wrench_list_[i] this gives
     *    Σ est_residual_list_[i] ≈ 0, so the resulting damping correction
     *    is naturally zero-sum and does not contaminate formation-level
     *    wrench tracking.
     *
     *    When no task is active (demo publishes zeros), est_residual = est_wrench
     *    and the rest of the function reduces to the pre-existing behaviour.
     */
    std::map<int, bool> assembly_flag = beetle_navigator_->getAssemblyFlags();
    int module_num = 0;
    Eigen::VectorXd W_sum = Eigen::VectorXd::Zero(6);
    for(const auto & item : est_wrench_list_){
      if(assembly_flag[item.first]){
        Eigen::VectorXd y_task = est_wrench_task_list_.count(item.first)
                                  ? est_wrench_task_list_[item.first]
                                  : Eigen::VectorXd::Zero(6);
        if(y_task.size() != 6) y_task = Eigen::VectorXd::Zero(6);
        Eigen::VectorXd residual = item.second - y_task;
        est_residual_list_[item.first] = residual;
        W_sum += residual;
        module_num ++;
      }else{
        est_residual_list_[item.first] = Eigen::VectorXd::Zero(6);
      }
    }

    if(!module_num) return;
    Eigen::VectorXd W_w = W_sum / module_num;
    geometry_msgs::WrenchStamped wrench_msg;
    wrench_msg.header.stamp.fromSec(estimator_->getImuLatestTimeStamp());
    wrench_msg.wrench.force.x = W_w(0);
    wrench_msg.wrench.force.y = W_w(1);
    wrench_msg.wrench.force.z = W_w(2);
    wrench_msg.wrench.torque.x = W_w(3);
    wrench_msg.wrench.torque.y = W_w(4);
    wrench_msg.wrench.torque.z = W_w(5);
    whole_external_wrench_pub_.publish(wrench_msg);

    /* 2. Recursion on the residual: inter_wrench_list_[i] is now the
     *    PARASITIC joint-cut wrench across the boundary between module i
     *    and module i+1 (along the leader→i traversal). */
    Eigen::VectorXd left_inter_wrench = Eigen::VectorXd::Zero(6);
    for(const auto & item : est_residual_list_){
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

    /* 2b. [Step D'] Leader-only diagnostic: pairwise disagreement of
     *     per-module inter wrenches. PURE OBSERVATION — does not affect
     *     control. Useful to detect divergent observer states between
     *     modules (model error, drift, comm dropout). */
    int leader_id_diag = beetle_navigator_->getLeaderID();
    bool is_diag_leader = (my_id == leader_id_diag);
    double max_f = 0.0, max_t = 0.0;
    double rms_f = 0.0, rms_t = 0.0;
    int n_pairs = 0;
    if (is_diag_leader) {
      std::vector<int> active_ids;
      for (const auto& kv : inter_wrench_list_) {
        if (assembly_flag[kv.first] && kv.second.size() == 6) {
          active_ids.push_back(kv.first);
        }
      }
      double sum_f2 = 0.0, sum_t2 = 0.0;
      for (size_t a = 0; a < active_ids.size(); ++a) {
        for (size_t b = a + 1; b < active_ids.size(); ++b) {
          Eigen::VectorXd diff =
              inter_wrench_list_[active_ids[a]] - inter_wrench_list_[active_ids[b]];
          double nf = diff.head(3).norm();
          double nt = diff.tail(3).norm();
          if (nf > max_f) max_f = nf;
          if (nt > max_t) max_t = nt;
          sum_f2 += nf * nf;
          sum_t2 += nt * nt;
          n_pairs++;
        }
      }
      rms_f = (n_pairs > 0) ? std::sqrt(sum_f2 / n_pairs) : 0.0;
      rms_t = (n_pairs > 0) ? std::sqrt(sum_t2 / n_pairs) : 0.0;
      std_msgs::Float32MultiArray diag_msg;
      diag_msg.data.resize(4);
      diag_msg.data[0] = static_cast<float>(max_f);
      diag_msg.data[1] = static_cast<float>(max_t);
      diag_msg.data[2] = static_cast<float>(rms_f);
      diag_msg.data[3] = static_cast<float>(rms_t);
      inter_disagreement_pub_.publish(diag_msg);
    }
    /* 3. Compute wrench_comp_list_[i] for the LF cascade. Since inter is now
     *    the PARASITIC joint-cut wrench (task already subtracted at step 1),
     *    wrench_comp_list_[i] is a simple cumulative sum of parasitic joint
     *    wrenches between the leader and module i. */
    int leader_id = beetle_navigator_->getLeaderID();
    /* 3.1. process from leader to left*/
    Eigen::VectorXd wrench_comp_sum_left = Eigen::VectorXd::Zero(6);
    for(int i = leader_id-1; i > 0; i--){
      if(assembly_flag[i]){
        wrench_comp_sum_left += inter_wrench_list_[i];
        wrench_comp_list_[i] = wrench_comp_sum_left;
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
        wrench_comp_sum_right += -inter_wrench_list_[left_module_id];
        wrench_comp_list_[i] = wrench_comp_sum_right;
        left_module_id = i;
      }else{
        wrench_comp_list_[i] = Eigen::VectorXd::Zero(6);
      }
    }

    if (unified_control_mode_ && unified_internal_wrench_log_) {
      auto fmtWrench = [](const Eigen::VectorXd& v) {
        std::ostringstream os;
        os << std::fixed << std::setprecision(3);
        if (v.size() < 6) {
          os << "[invalid:" << v.size() << "]";
        } else {
          os << "[" << v(0) << "," << v(1) << "," << v(2)
             << ";" << v(3) << "," << v(4) << "," << v(5) << "]";
        }
        return os.str();
      };

      double max_res_f = 0.0, max_res_t = 0.0;
      double max_comp_f = 0.0, max_comp_t = 0.0;
      for (int i = 1; i <= max_modules_num; ++i) {
        if (!assembly_flag[i]) continue;
        if (est_residual_list_[i].size() >= 6) {
          max_res_f = std::max(max_res_f, est_residual_list_[i].head(3).norm());
          max_res_t = std::max(max_res_t, est_residual_list_[i].tail(3).norm());
        }
        if (wrench_comp_list_[i].size() >= 6) {
          max_comp_f = std::max(max_comp_f, wrench_comp_list_[i].head(3).norm());
          max_comp_t = std::max(max_comp_t, wrench_comp_list_[i].tail(3).norm());
        }
      }

      std::ostringstream ss;
      ss << std::fixed << std::setprecision(3);
      ss << "[UnifiedInternalWrench id=" << my_id
         << " leader=" << leader_id
         << " gain=" << unified_internal_wrench_secondary_gain_
         << " src=single_module_observer"
         << "] W_res_avg=" << fmtWrench(W_w)
         << " maxRes=[F=" << max_res_f << ",T=" << max_res_t << "]"
         << " maxComp=[F=" << max_comp_f << ",T=" << max_comp_t << "]";
      if (is_diag_leader) {
        ss << " disagree=[maxF=" << max_f
           << ",maxT=" << max_t
           << ",rmsF=" << rms_f
           << ",rmsT=" << rms_t
           << ",pairs=" << n_pairs << "]";
      }
      ROS_INFO_STREAM_THROTTLE(unified_internal_wrench_log_period_, ss.str());

      std::ostringstream detail_ss;
      detail_ss << ss.str() << " modules:";
      for (int i = 1; i <= max_modules_num; ++i) {
        if (!assembly_flag[i]) continue;
        detail_ss << " m" << i
                  << "{est=" << fmtWrench(est_wrench_list_[i])
                  << ",task=" << fmtWrench(est_wrench_task_list_[i])
                  << ",res=" << fmtWrench(est_residual_list_[i])
                  << ",inter=" << fmtWrench(inter_wrench_list_[i])
                  << ",comp=" << fmtWrench(wrench_comp_list_[i]) << "}";
      }
      ROS_INFO_STREAM_THROTTLE(unified_internal_wrench_log_period_, detail_ss.str());
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

    getParam<bool>(control_nh, "yaw_in_allocation", yaw_in_allocation_, false);

    getParam<int>(control_nh, "unified_reference_warmup_frames", unified_reference_warmup_frames_, 20);
    unified_reference_warmup_frames_ = std::max(0, unified_reference_warmup_frames_);
    getParam<double>(control_nh, "torque_allocation_matrix_inv_pub_interval",
                     unified_torque_alloc_inv_pub_interval_, 0.05);
    unified_torque_alloc_inv_pub_interval_ =
        std::max(0.0, unified_torque_alloc_inv_pub_interval_);
    getParam<bool>(control_nh, "unified_internal_wrench_diag", unified_internal_wrench_diag_, true);
    getParam<bool>(control_nh, "unified_internal_wrench_log", unified_internal_wrench_log_, true);
    getParam<double>(control_nh, "unified_internal_wrench_log_period",
                     unified_internal_wrench_log_period_, 1.0);
    unified_internal_wrench_log_period_ =
        std::max(0.1, unified_internal_wrench_log_period_);
    getParam<double>(control_nh, "unified_internal_wrench_secondary_gain",
                     unified_internal_wrench_secondary_gain_, 0.0);
    unified_internal_wrench_secondary_gain_ =
        std::min(0.2, std::max(0.0, unified_internal_wrench_secondary_gain_));

    // Roll/Pitch I-term keep ratio removed: outer R/P I-channel disabled in unified mode.

    // Load unified-mode PID gains for roll/pitch.
    // Full P/I/D: P+D are sent to each module's spinal for 1000Hz inner-loop tracking.
    // I-term is retained on PC for slow formation-level bias correction.
    ros::NodeHandle u_roll_nh(control_nh, "unified_roll");
    getParam<double>(u_roll_nh, "p_gain", unified_roll_gains_.p, 8.0);
    getParam<double>(u_roll_nh, "i_gain", unified_roll_gains_.i, 1.0);
    getParam<double>(u_roll_nh, "d_gain", unified_roll_gains_.d, 5.0);
    getParam<double>(u_roll_nh, "limit_sum", unified_roll_gains_.limit_sum, 50.0);
    getParam<double>(u_roll_nh, "limit_p", unified_roll_gains_.limit_p, 20.0);
    getParam<double>(u_roll_nh, "limit_i", unified_roll_gains_.limit_i, 10.0);
    getParam<double>(u_roll_nh, "limit_d", unified_roll_gains_.limit_d, 20.0);

    ros::NodeHandle u_pitch_nh(control_nh, "unified_pitch");
    getParam<double>(u_pitch_nh, "p_gain", unified_pitch_gains_.p, 8.0);
    getParam<double>(u_pitch_nh, "i_gain", unified_pitch_gains_.i, 1.0);
    getParam<double>(u_pitch_nh, "d_gain", unified_pitch_gains_.d, 5.0);
    getParam<double>(u_pitch_nh, "limit_sum", unified_pitch_gains_.limit_sum, 50.0);
    getParam<double>(u_pitch_nh, "limit_p", unified_pitch_gains_.limit_p, 20.0);
    getParam<double>(u_pitch_nh, "limit_i", unified_pitch_gains_.limit_i, 10.0);
    getParam<double>(u_pitch_nh, "limit_d", unified_pitch_gains_.limit_d, 20.0);

    // Load unified-mode PID gains for XY and Z (used when formation is assembled)
    ros::NodeHandle u_xy_nh(control_nh, "unified_xy");
    getParam<double>(u_xy_nh, "p_gain", unified_xy_gains_.p, 2.0);
    getParam<double>(u_xy_nh, "i_gain", unified_xy_gains_.i, 0.2);
    getParam<double>(u_xy_nh, "d_gain", unified_xy_gains_.d, 2.5);
    getParam<double>(u_xy_nh, "limit_sum", unified_xy_gains_.limit_sum, 8.0);
    getParam<double>(u_xy_nh, "limit_p", unified_xy_gains_.limit_p, 12.0);
    getParam<double>(u_xy_nh, "limit_i", unified_xy_gains_.limit_i, 8.0);
    getParam<double>(u_xy_nh, "limit_d", unified_xy_gains_.limit_d, 12.0);
    getParam<double>(u_xy_nh, "err_d_lpf_cutoff_freq", unified_xy_gains_.err_d_lpf_cutoff_freq, 0.0);

    ros::NodeHandle u_z_nh(control_nh, "unified_z");
    getParam<double>(u_z_nh, "p_gain", unified_z_gains_.p, 5.0);
    getParam<double>(u_z_nh, "i_gain", unified_z_gains_.i, 1.0);
    getParam<double>(u_z_nh, "d_gain", unified_z_gains_.d, 2.0);
    getParam<double>(u_z_nh, "limit_sum", unified_z_gains_.limit_sum, 25.0);
    getParam<double>(u_z_nh, "limit_p", unified_z_gains_.limit_p, 25.0);
    getParam<double>(u_z_nh, "limit_i", unified_z_gains_.limit_i, 20.0);
    getParam<double>(u_z_nh, "limit_d", unified_z_gains_.limit_d, 25.0);

    // Load unified-mode yaw PID gains (full P+I+D on PC side)
    ros::NodeHandle u_yaw_nh(control_nh, "unified_yaw");
    getParam<double>(u_yaw_nh, "p_gain", unified_yaw_gains_.p, 15.0);
    getParam<double>(u_yaw_nh, "i_gain", unified_yaw_gains_.i, 1.0);
    getParam<double>(u_yaw_nh, "d_gain", unified_yaw_gains_.d, 10.0);
    getParam<double>(u_yaw_nh, "limit_sum", unified_yaw_gains_.limit_sum, 20.0);
    getParam<double>(u_yaw_nh, "limit_p", unified_yaw_gains_.limit_p, 20.0);
    getParam<double>(u_yaw_nh, "limit_i", unified_yaw_gains_.limit_i, 5.0);
    getParam<double>(u_yaw_nh, "limit_d", unified_yaw_gains_.limit_d, 20.0);

  }

  void BeetleController::externalWrenchEstimate()
  {
    // In unified mode this single-module observer is kept alive for short-term
    // residual diagnostics. Its estimate is not injected into pose/attitude PID;
    // calcInteractionWrench() only feeds the allocation secondary when the
    // explicit unified_internal_wrench_secondary_gain is positive and HOVER-gated.
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

    // Synchronous self-update: write our own observer result directly into
    // est_wrench_list_ so calcInteractionWrench() never reads a stale (or
    // zero) entry for this module within the same control tick. The cross-
    // module entries still arrive via estExternalWrenchCallback. Frame is
    // CoG (matches what the publish carries and what callbacks store).
    est_wrench_list_[beetle_navigator_->getMyID()] = est_external_wrench_cog;

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

  void BeetleController::estWrenchTaskCallback(const beetle::TaggedWrench & msg)
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
    est_wrench_task_list_[id] = wrench;
  }

  void BeetleController::desiredExternalWrenchCallback(const geometry_msgs::WrenchStamped & msg)
  {
    // Legacy topic alias for formation-level desired wrench.
    //
    // Storage semantic:
    //   formation_desired_wrench_ = FULL formation-level wrench on EVERY module.
    //   Unified mode consumes it only in computeUnifiedAllocation(); it is not
    //   injected as PID persistent FF.
    //
    // Per-module observer task prediction (est_wrench_task_list_) is published
    // by the demo layer (BeetleInterface) on /<robot>{i}/est_wrench_task and
    // arrives via estWrenchTaskCallback. The leader no longer derives or
    // re-publishes those values here.

    Eigen::VectorXd desired = Eigen::VectorXd::Zero(6);
    desired(0) = msg.wrench.force.x;
    desired(1) = msg.wrench.force.y;
    desired(2) = msg.wrench.force.z;
    desired(3) = msg.wrench.torque.x;
    desired(4) = msg.wrench.torque.y;
    desired(5) = msg.wrench.torque.z;

    // Every module stores the FULL desired wrench (semantic unified across modules).
    formation_desired_wrench_ = desired;

    // Only LEADER rebroadcasts to followers so every module sees the same value.
    if(beetle_navigator_->getModuleState() != LEADER) {
      return;
    }

    std::map<int, bool> assembly_flag = beetle_navigator_->getAssemblyFlags();
    int leader_id = beetle_navigator_->getLeaderID();
    ros::Time stamp = msg.header.stamp;

    // Rebroadcast FULL desired wrench to all followers so every module stores
    // the same formation_desired_wrench_ (FULL semantic).
    for(const auto & item : assembly_flag) {
      if(!item.second) continue;
      if(item.first == leader_id) continue;  // skip LEADER to avoid cascade
      if(desired_ext_wrench_pubs_.count(item.first) == 0) continue;
      geometry_msgs::WrenchStamped fw;
      fw.header.stamp = stamp;
      fw.wrench.force.x = desired(0);
      fw.wrench.force.y = desired(1);
      fw.wrench.force.z = desired(2);
      fw.wrench.torque.x = desired(3);
      fw.wrench.torque.y = desired(4);
      fw.wrench.torque.z = desired(5);
      desired_ext_wrench_pubs_[item.first].publish(fw);
    }
  }

  bool BeetleController::setUnifiedModeCb(std_srvs::SetBool::Request &req,
                                          std_srvs::SetBool::Response &res)
  {
    unified_control_mode_ = req.data;
    if (req.data) {
      publishModuleModel();
    }
    // Write to rosparam for consistency (backward compat with rosparam-based tools)
    ros::NodeHandle control_nh(nh_, "controller");
    control_nh.setParam("unified_control_mode", req.data);
    res.success = true;
    res.message = req.data ? "unified mode enabled" : "unified mode disabled";
    ROS_INFO("[BeetleController] set_unified_mode service: %s", res.message.c_str());
    return true;
  }

  void BeetleController::formationDesiredWrenchCallback(const geometry_msgs::WrenchStamped& msg)
  {
    // Receive desired formation-level wrench (formation body frame, full 6D).
    // Reuse the legacy callback so leader rebroadcast and storage semantics
    // stay identical across both input topics.
    desiredExternalWrenchCallback(msg);
  }

  void BeetleController::publishAssembleDebug(
      const tf::Vector3& formation_pos, const tf::Vector3& formation_vel,
      const tf::Vector3& target_formation_pos, bool alloc_ok)
  {
    // 1. PID debug: formation-level pose control PID
    assemble_pid_msg_.header.stamp = ros::Time::now();

    // X
    assemble_pid_msg_.x.total.at(0) = pid_controllers_.at(X).result();
    assemble_pid_msg_.x.p_term.at(0) = pid_controllers_.at(X).getPTerm();
    assemble_pid_msg_.x.i_term.at(0) = pid_controllers_.at(X).getITerm();
    assemble_pid_msg_.x.d_term.at(0) = pid_controllers_.at(X).getDTerm();
    assemble_pid_msg_.x.target_p = target_formation_pos.x();
    assemble_pid_msg_.x.err_p = target_formation_pos.x() - formation_pos.x();
    assemble_pid_msg_.x.target_d = target_vel_.x();
    assemble_pid_msg_.x.err_d = target_vel_.x() - formation_vel.x();

    // Y
    assemble_pid_msg_.y.total.at(0) = pid_controllers_.at(Y).result();
    assemble_pid_msg_.y.p_term.at(0) = pid_controllers_.at(Y).getPTerm();
    assemble_pid_msg_.y.i_term.at(0) = pid_controllers_.at(Y).getITerm();
    assemble_pid_msg_.y.d_term.at(0) = pid_controllers_.at(Y).getDTerm();
    assemble_pid_msg_.y.target_p = target_formation_pos.y();
    assemble_pid_msg_.y.err_p = target_formation_pos.y() - formation_pos.y();
    assemble_pid_msg_.y.target_d = target_vel_.y();
    assemble_pid_msg_.y.err_d = target_vel_.y() - formation_vel.y();

    // Z
    assemble_pid_msg_.z.total.at(0) = pid_controllers_.at(Z).result();
    assemble_pid_msg_.z.p_term.at(0) = pid_controllers_.at(Z).getPTerm();
    assemble_pid_msg_.z.i_term.at(0) = pid_controllers_.at(Z).getITerm();
    assemble_pid_msg_.z.d_term.at(0) = pid_controllers_.at(Z).getDTerm();
    assemble_pid_msg_.z.target_p = target_formation_pos.z();
    assemble_pid_msg_.z.err_p = target_formation_pos.z() - formation_pos.z();
    assemble_pid_msg_.z.target_d = target_vel_.z();
    assemble_pid_msg_.z.err_d = target_vel_.z() - formation_vel.z();

    // Roll
    assemble_pid_msg_.roll.total.at(0) = pid_controllers_.at(ROLL).result();
    assemble_pid_msg_.roll.p_term.at(0) = pid_controllers_.at(ROLL).getPTerm();
    assemble_pid_msg_.roll.i_term.at(0) = pid_controllers_.at(ROLL).getITerm();
    assemble_pid_msg_.roll.d_term.at(0) = pid_controllers_.at(ROLL).getDTerm();
    assemble_pid_msg_.roll.target_p = target_rpy_.x();
    assemble_pid_msg_.roll.err_p = target_rpy_.x() - rpy_.x();
    assemble_pid_msg_.roll.target_d = target_omega_.x();
    assemble_pid_msg_.roll.err_d = target_omega_.x() - omega_.x();

    // Pitch
    assemble_pid_msg_.pitch.total.at(0) = pid_controllers_.at(PITCH).result();
    assemble_pid_msg_.pitch.p_term.at(0) = pid_controllers_.at(PITCH).getPTerm();
    assemble_pid_msg_.pitch.i_term.at(0) = pid_controllers_.at(PITCH).getITerm();
    assemble_pid_msg_.pitch.d_term.at(0) = pid_controllers_.at(PITCH).getDTerm();
    assemble_pid_msg_.pitch.target_p = target_rpy_.y();
    assemble_pid_msg_.pitch.err_p = target_rpy_.y() - rpy_.y();
    assemble_pid_msg_.pitch.target_d = target_omega_.y();
    assemble_pid_msg_.pitch.err_d = target_omega_.y() - omega_.y();

    // Yaw
    assemble_pid_msg_.yaw.total.at(0) = pid_controllers_.at(YAW).result();
    assemble_pid_msg_.yaw.p_term.at(0) = pid_controllers_.at(YAW).getPTerm();
    assemble_pid_msg_.yaw.i_term.at(0) = pid_controllers_.at(YAW).getITerm();
    assemble_pid_msg_.yaw.d_term.at(0) = pid_controllers_.at(YAW).getDTerm();
    assemble_pid_msg_.yaw.target_p = target_rpy_.z();
    assemble_pid_msg_.yaw.err_p = angles::shortest_angular_distance(rpy_.z(), target_rpy_.z());
    assemble_pid_msg_.yaw.target_d = target_omega_.z();
    assemble_pid_msg_.yaw.err_d = target_omega_.z() - omega_.z();

    assemble_pid_pub_.publish(assemble_pid_msg_);

    // 2. Vectoring force
    if (alloc_ok) {
      const Eigen::VectorXd& vf = unified_controller_->getTargetVectoringForce();
      std_msgs::Float32MultiArray vf_msg;
      vf_msg.data.resize(vf.size());
      for (int i = 0; i < vf.size(); i++)
        vf_msg.data[i] = static_cast<float>(vf(i));
      assemble_vectoring_f_pub_.publish(vf_msg);

      // 3. Formation realized wrench
      Eigen::VectorXd rw = unified_controller_->getRealizedWrenchBody();
      geometry_msgs::WrenchStamped wrench_msg;
      wrench_msg.header.stamp = ros::Time::now();
      wrench_msg.header.frame_id = "assembly_cog";
      wrench_msg.wrench.force.x = rw(0);
      wrench_msg.wrench.force.y = rw(1);
      wrench_msg.wrench.force.z = rw(2);
      wrench_msg.wrench.torque.x = rw(3);
      wrench_msg.wrench.torque.y = rw(4);
      wrench_msg.wrench.torque.z = rw(5);
      assemble_formation_wrench_pub_.publish(wrench_msg);
    }
  }

  // =====================================================================
  // runUnifiedControlCommon: symmetric unified-mode control body.
  // Executed on BOTH leader and follower. Each module uses its own
  // estimator / navigator / TF state. The leader additionally broadcasts
  // a debug reference and feeds the formation-level momentum observer.
  // =====================================================================
  void BeetleController::runUnifiedControlCommon(bool is_leader)
  {
    int my_id = beetle_navigator_->getMyID();
    int leader_id = beetle_navigator_->getLeaderID();
    // --- Gather local state ---
    pos_ = estimator_->getPos(Frame::COG, estimate_mode_);
    vel_ = estimator_->getVel(Frame::COG, estimate_mode_);
    target_pos_ = navigator_->getTargetPos();
    target_vel_ = navigator_->getTargetVel();
    target_acc_ = navigator_->getTargetAcc();

    tf::Quaternion cog2baselink_rot;
    tf::quaternionKDLToTF(robot_model_->getCogDesireOrientation<KDL::Rotation>(), cog2baselink_rot);
    tf::Matrix3x3 cog_rot = estimator_->getOrientation(Frame::BASELINK, estimate_mode_)
                          * tf::Matrix3x3(cog2baselink_rot).inverse();
    double r, p, y_angle; cog_rot.getRPY(r, p, y_angle);
    rpy_.setValue(r, p, y_angle);
    omega_ = estimator_->getAngularVel(Frame::COG, estimate_mode_);

    // [Fix A] Restore the canonical cog-frame tracker pattern: rely solely on
    // navigator_->getTargetRPY(), which is the spinal-side rate-limited target.
    // Previously this routine overwrote x/y with beetle_navigator_->getFinalTargetBaselinkRPY(),
    // i.e. the *unramped* final baselink target. That mismatch injected a step (up to
    // ~0.2 rad on pitch=0.2 hover transitions) directly into the outer PID error,
    // causing pitch_i wind-up and a cascading Z drop. Removing the override restores
    // the same single-source-of-truth used by PoseLinearController (see
    // pose_linear_controller.cpp:244).
    target_rpy_ = navigator_->getTargetRPY();
    tf::Matrix3x3 target_rot; target_rot.setRPY(target_rpy_.x(), target_rpy_.y(), target_rpy_.z());
    tf::Vector3 target_baselink_rpy = beetle_navigator_->getFinalTargetBaselinkRPY();
    target_baselink_rpy.setZ(target_rpy_.z());
    tf::Matrix3x3 target_baselink_rot;
    target_baselink_rot.setRPY(target_baselink_rpy.x(),
                               target_baselink_rpy.y(),
                               target_baselink_rpy.z());
    tf::Vector3 target_omega = navigator_->getTargetOmega();
    target_omega_ = cog_rot.inverse() * target_rot * target_omega;
    target_ang_acc_ = navigator_->getTargetAngAcc();

    // --- Formation geometry: each module computes locally ---
    unified_controller_->updateFormationGeometry();
    // Detect geometry changes (e.g., peer ModuleModel late-arrival flipping the
    // formation from leader-only fallback to true heterogeneous mass-weighted).
    // The leader's cascade_alloc_sent_ is re-armed internally by
    // updateFormationGeometry; the follower's local one-shot lives here and
    // needs to be re-armed too so the spinal does not keep a stale
    // torque_alloc_inv built from the transient (pre-convergence) geometry.
    uint64_t cur_formation_rev = unified_controller_->getFormationRevision();
    if (cur_formation_rev != prev_formation_revision_) {
      if (!is_leader) local_unified_cascade_setup_sent_ = false;
      ROS_INFO("[UnifiedCtrl] id=%d formation_revision %lu → %lu, re-arm cascade one-shot",
               my_id, (unsigned long)prev_formation_revision_, (unsigned long)cur_formation_rev);
      prev_formation_revision_ = cur_formation_rev;
    }
    const Eigen::Vector3d& cog_offset_leader_frame = unified_controller_->getFormationCogOffset();
    Eigen::Vector3d cog_offset_self = cog_offset_leader_frame;
    if (!is_leader) {
      // Convert leader-frame offset to this module's baselink frame.
      // Rigid assembly ⇒ all modules share the same body orientation, so the
      // formation CoG offset expressed in this module's frame is simply
      // (leader→formation_CoG) − (leader→my_CoG), all in the common orientation.
      // NOTE: tf2 frame_ids MUST NOT start with '/' (same convention as
      // BeetleUnifiedController::updateFormationGeometry).
      try {
        std::string leader_cog_frame = beetle_navigator_->getMyName()
                                      + std::to_string(leader_id) + "/cog";
        std::string my_cog_frame = beetle_navigator_->getMyName()
                                  + std::to_string(my_id) + "/cog";
        geometry_msgs::TransformStamped tf_stamped =
            beetle_navigator_->getTfBuffer().lookupTransform(leader_cog_frame, my_cog_frame, ros::Time(0));
        cog_offset_self.x() -= tf_stamped.transform.translation.x;
        cog_offset_self.y() -= tf_stamped.transform.translation.y;
        cog_offset_self.z() -= tf_stamped.transform.translation.z;
      } catch (tf2::TransformException& ex) {
        // Soft fallback: keep leader-frame offset. One-frame positional bias is
        // vastly safer than a full thrust drop-out from early-return. The next
        // frame will retry.
        ROS_WARN_THROTTLE(1.0, "[UnifiedCtrl FOLLOWER id=%d] cog_offset_self TF failed (%s), falling back to leader-frame offset",
                          my_id, ex.what());
      }
    }

    // ---- Phase B follower override --------------------------------------
    // In unified mode the follower must derive its module reference from the
    // leader's broadcast so all modules close the same formation target, even
    // if a local navigator command is late, absent, or expressed in a different
    // intermediate frame.
    //
    //   delta_body = (leader_baselink → formation_CoG) − (follower_baselink → formation_CoG)
    //              = follower_baselink → leader_baselink   [shared body frame]
    //   p_follower = p_leader + R_baselink_target * delta_body
    //   v_follower = v_leader + ω × (R_baselink_target * delta_body)
    //   a_follower = a_leader + α × Δworld + ω × (ω × Δworld)
    // For the leader itself this branch is skipped (its own navigator already
    // holds the correct setpoint). If no leader message has arrived yet, we
    // fall back to the navigator value (bounded transient at first frame).
    if (!is_leader && unified_cmd_received_) {
      const tf::Vector3& lt_pos     = leader_target_pos_;
      const tf::Vector3& lt_vel     = leader_target_vel_;
      const tf::Vector3& lt_acc     = leader_target_acc_;
      const tf::Vector3& lt_rpy     = leader_target_rpy_;
      const tf::Vector3& lt_final_baselink_rpy = leader_final_target_baselink_rpy_;
      const tf::Vector3& lt_omega   = leader_target_omega_;
      const tf::Vector3& lt_ang_acc = leader_target_ang_acc_;

      // Recompute target_rot with leader's RPY (rigid assembly ⇒ shared orientation).
      target_rpy_ = lt_rpy;
      target_rot.setRPY(target_rpy_.x(), target_rpy_.y(), target_rpy_.z());
      target_baselink_rpy = lt_final_baselink_rpy;
      target_baselink_rpy.setZ(target_rpy_.z());
      target_baselink_rot.setRPY(target_baselink_rpy.x(),
                                 target_baselink_rpy.y(),
                                 target_baselink_rpy.z());

      tf::Vector3 delta_body(cog_offset_leader_frame.x() - cog_offset_self.x(),
                             cog_offset_leader_frame.y() - cog_offset_self.y(),
                             cog_offset_leader_frame.z() - cog_offset_self.z());
      tf::Vector3 delta_world      = target_baselink_rot * delta_body;
      tf::Vector3 lt_omega_world   = target_rot * lt_omega;
      tf::Vector3 lt_ang_acc_world = target_rot * lt_ang_acc;

      target_pos_ = lt_pos + delta_world;
      target_vel_ = lt_vel + lt_omega_world.cross(delta_world);
      target_acc_ = lt_acc + lt_ang_acc_world.cross(delta_world)
                  + lt_omega_world.cross(lt_omega_world.cross(delta_world));

      target_omega    = lt_omega;
      target_omega_   = cog_rot.inverse() * target_rot * target_omega;
      target_ang_acc_ = lt_ang_acc;
    }
    tf::Vector3 offset_body(cog_offset_self.x(), cog_offset_self.y(), cog_offset_self.z());
    tf::Vector3 offset_world = cog_rot * offset_body;
    tf::Vector3 formation_pos = pos_ + offset_world;
    tf::Vector3 omega_world = cog_rot * omega_;
    tf::Vector3 formation_vel = vel_ + omega_world.cross(offset_world);
    tf::Vector3 target_formation_pos = target_pos_ + target_baselink_rot * offset_body;

    // --- Position PID (X/Y/Z) with formation CoG ---
    double du = ros::Time::now().toSec() - control_timestamp_;
    if (du < 0.0) du = 0.0;
    if (du > 0.1) du = 0.1;  // clamp callback-stall gaps so PID-D and FF integrators stay sane

    // Optional LF-style internal wrench processing in unified mode.
    // With gain=0 this is diagnostic only: it publishes the same residual /
    // inter-wrench signals as the legacy leader-follower path without feeding
    // them back into the allocator. A positive gain biases the QP secondary
    // reference, not the primary formation wrench objective.
    const bool unified_secondary_ready =
        beetle_navigator_->getControlFlag() &&
        navigator_->getNaviState() == aerial_robot_navigation::HOVER_STATE &&
        !navigator_->getForceLandingFlag();
    const bool unified_secondary_active =
        unified_internal_wrench_secondary_gain_ > 0.0 && unified_secondary_ready;

    if (beetle_navigator_->getControlFlag() &&
        (unified_internal_wrench_diag_ || unified_secondary_active)) {
      calcInteractionWrench();
    }
    if (unified_secondary_active) {
      unified_controller_->setInternalWrenchSecondaryReference(
          wrench_comp_list_, unified_internal_wrench_secondary_gain_);
    } else {
      unified_controller_->clearInternalWrenchSecondaryReference();
    }

    // Unified mode has a single task-wrench path: formation_desired_wrench_
    // is added directly inside computeUnifiedAllocation(). Keep PID persistent
    // FF clear so the same external wrench cannot be injected twice.
    pid_controllers_.at(X).setPersistentFF(0.0);
    pid_controllers_.at(Y).setPersistentFF(0.0);
    pid_controllers_.at(Z).setPersistentFF(0.0);
    pid_controllers_.at(YAW).setPersistentFF(0.0);

    Eigen::VectorXd formation_wrench_cmd = formation_desired_wrench_;
    if (navigator_->getForceLandingFlag()) {
      formation_wrench_cmd.setZero();
    }

    switch (navigator_->getXyControlMode()) {
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
    if (navigator_->getForceLandingFlag()) {
      pid_controllers_.at(X).reset();
      pid_controllers_.at(Y).reset();
    }

    double err_z = target_formation_pos.z() - formation_pos.z();
    double err_v_z = target_vel_.z() - formation_vel.z();
    double z_p_limit = pid_controllers_.at(Z).getLimitP();
    if (navigator_->getForceLandingFlag()) {
      pid_controllers_.at(Z).setLimitP(0);
      err_z = force_landing_descending_rate_;
      err_v_z = 0;
      target_acc_.setZ(0);
    }
    // [Fix C] sec(tilt) compensation: with non-zero baselink tilt the world-frame
    // vertical lift component drops by cos(roll)*cos(pitch). Scale the Z position
    // error so the outer PID commands enough total thrust to recover the world-Z
    // setpoint. A floor of 0.5 prevents divergence near 60 deg tilt.
    {
      const double tilt_cos = std::max(std::cos(rpy_.x()) * std::cos(rpy_.y()), 0.5);
      err_z /= tilt_cos;
    }
    pid_controllers_.at(Z).update(err_z, du, err_v_z, target_acc_.z());

    if (navigator_->getForceLandingFlag()) {
      pid_controllers_.at(Z).setLimitP(z_p_limit);
      pid_controllers_.at(Z).setErrP(0);
    }

    // --- Attitude PID (Roll/Pitch/Yaw) ---
    if (!start_rp_integration_) {
      if (pos_.z() - navigator_->getInitHeight() > start_rp_integration_height_) {
        start_rp_integration_ = true;
        spinal::FlightConfigCmd flight_config_cmd;
        flight_config_cmd.cmd = spinal::FlightConfigCmd::INTEGRATION_CONTROL_ON_CMD;
        navigator_->getFlightConfigPublisher().publish(flight_config_cmd);
        ROS_WARN("[UnifiedCtrl] id=%d start roll/pitch I control (height threshold passed)", my_id);
      }
    }
    double du_rp = du;
    if (!start_rp_integration_) du_rp = 0;

    // v5: Restored outer R/P PID. Mirrors beetle independent-mode
    // (gimbal_calc_in_fc=true && i_term_rp_calc_in_pc=true): PC runs full
    // P+I+D update each frame, but only the I-term is fed into
    // target_wrench_acc(3,4); spinal owns the high-bandwidth P+D inner loop.
    // The PC P-/D-term states are still maintained for clean exit to
    // independent hover (no discontinuity).
    pid_controllers_.at(ROLL).update(target_rpy_.x() - rpy_.x(), du_rp,
                                     target_omega_.x() - omega_.x(), target_ang_acc_.x());
    pid_controllers_.at(PITCH).update(target_rpy_.y() - rpy_.y(), du_rp,
                                      target_omega_.y() - omega_.y(), target_ang_acc_.y());
    if (navigator_->getForceLandingFlag()) {
      pid_controllers_.at(ROLL).reset();
      pid_controllers_.at(PITCH).reset();
    }

    double err_yaw = angles::shortest_angular_distance(rpy_.z(), target_rpy_.z());
    double err_omega_z = target_omega_.z() - omega_.z();
    if (!need_yaw_d_control_) err_omega_z = target_omega_.z();
    pid_controllers_.at(YAW).update(err_yaw, du, err_omega_z, target_ang_acc_.z());

    control_timestamp_ = ros::Time::now().toSec();

    // Local warmup (same on leader and follower)
    if (unified_reference_warmup_count_ < unified_reference_warmup_frames_) {
      if (navigator_->getNaviState() != aerial_robot_navigation::TAKEOFF_STATE) {
        pid_controllers_.at(Z).setErrI(pid_controllers_.at(Z).getPrevErrI());
        pid_controllers_.at(X).setErrI(pid_controllers_.at(X).getPrevErrI());
        pid_controllers_.at(Y).setErrI(pid_controllers_.at(Y).getPrevErrI());
      }
      unified_reference_warmup_count_++;
    }

    // --- Build 6-DOF target wrench in acceleration space ---
    tf::Matrix3x3 uav_rot = estimator_->getOrientation(Frame::COG, estimate_mode_);
    tf::Vector3 target_acc_w(pid_controllers_.at(X).result(),
                             pid_controllers_.at(Y).result(),
                             pid_controllers_.at(Z).result());
    tf::Vector3 target_acc_cog = uav_rot.inverse() * target_acc_w;

    Eigen::VectorXd target_wrench_acc = Eigen::VectorXd::Zero(6);
    target_wrench_acc.head(3) = Eigen::Vector3d(target_acc_cog.x(), target_acc_cog.y(), target_acc_cog.z());
    // v5 architecture (mirrors beetle independent mode with i_term_rp_calc_in_pc=true):
    //   target_wrench_acc(3,4) = (ROLL.getITerm(), PITCH.getITerm())
    //   spinal owns P+D high-bandwidth (roll_i/pitch_i sent as 0 in sendCascadeSetup)
    // The outer I-term integrates slow CoG-offset / model-error torques; spinal's P+D
    // delivers the fast attitude-tracking response. This is the same dual-loop split
    // db6cec4d adopted and that the original beetle / ninja architectures have used
    // for years.
    target_wrench_acc(3) = pid_controllers_.at(ROLL).getITerm();
    target_wrench_acc(4) = pid_controllers_.at(PITCH).getITerm();
    double yaw_pid_raw = pid_controllers_.at(YAW).result();
    target_wrench_acc(5) = yaw_in_allocation_ ? yaw_pid_raw : 0.0;

    // Gravity FF with takeoff ramp
    {
      tf::Vector3 gravity_w(0, 0, aerial_robot_estimation::G);
      tf::Vector3 gravity_cog = uav_rot.inverse() * gravity_w;
      double gravity_ramp = 1.0;
      if (navigator_->getNaviState() == aerial_robot_navigation::TAKEOFF_STATE) {
        constexpr int GRAVITY_RAMP_FRAMES = 20;
        gravity_ramp = std::min(static_cast<double>(unified_transition_count_) / GRAVITY_RAMP_FRAMES, 1.0);
      }
      target_wrench_acc.head(3) += gravity_ramp * Eigen::Vector3d(gravity_cog.x(), gravity_cog.y(), gravity_cog.z());
    }

    // Followers use the leader's formation-level wrench reference for allocation
    // so all assembled modules solve the same QP and only pick their own block.
    // Keep the spinal attitude target local for now; this limits reference-link
    // latency to the 40Hz allocation input, not the lower-level attitude channel.
    const double unified_ref_age =
        (!is_leader && unified_cmd_received_) ?
        (ros::Time::now() - unified_cmd_stamp_).toSec() :
        std::numeric_limits<double>::infinity();
    const bool use_leader_allocation_reference =
        !is_leader && unified_cmd_received_ &&
        unified_ref_age >= -0.05 && unified_ref_age < 0.5 &&
        unified_reference_wrench_acc_.size() == 6 &&
        unified_reference_desired_wrench_.size() == 6;
    if (use_leader_allocation_reference) {
      target_wrench_acc = unified_reference_wrench_acc_;
      formation_wrench_cmd = unified_reference_desired_wrench_;
      yaw_pid_raw = unified_reference_yaw_pid_raw_;
      if (navigator_->getForceLandingFlag()) {
        formation_wrench_cmd.setZero();
      }
    } else if (!is_leader && navigator_->getNaviState() != aerial_robot_navigation::TAKEOFF_STATE) {
      ROS_WARN_THROTTLE(1.0,
                        "[UnifiedCtrl FOLLOWER id=%d] leader allocation reference unavailable/stale (received=%d age=%.3fs); using local PID wrench",
                        my_id, unified_cmd_received_ ? 1 : 0, unified_ref_age);
    }

    setTargetWrenchAccCog(target_wrench_acc);

    // --- Run unified 6-DOF allocation ---
    unified_controller_->clearFormationModelOverride();
    bool ok = unified_controller_->computeUnifiedAllocation(target_wrench_acc, formation_wrench_cmd, yaw_pid_raw);

    if (ok) {
      if (!is_leader) {
        sendLocalUnifiedCascadeSetupOnce();
      }
      publishLocalUnifiedCommand();
      const double now = ros::Time::now().toSec();
      if (last_unified_torque_alloc_inv_pub_time_ < 0.0 ||
          now - last_unified_torque_alloc_inv_pub_time_ >=
              unified_torque_alloc_inv_pub_interval_) {
        if (publishLocalUnifiedTorqueAllocationMatrixInv()) {
          last_unified_torque_alloc_inv_pub_time_ = now;
        }
      }
      if (is_leader) {
        publishUnifiedReference(target_wrench_acc, formation_wrench_cmd, yaw_pid_raw);
        if (formation_observer_ && formation_observer_->isActive()) {
          Eigen::Matrix3d cog_rot_eigen;
          tf::matrixTFToEigen(uav_rot, cog_rot_eigen);
          Eigen::Vector3d offset_w(offset_world.x(), offset_world.y(), offset_world.z());
          auto imu_handler_obs = boost::dynamic_pointer_cast<sensor_plugin::Imu>(
              estimator_->getImuHandler(0));
          Eigen::Vector3d vel_leader_w, omega_body;
          tf::vectorTFToEigen(imu_handler_obs->getFilteredVelCog(), vel_leader_w);
          tf::vectorTFToEigen(imu_handler_obs->getFilteredOmegaCog(), omega_body);
          Eigen::Vector3d omega_w = cog_rot_eigen * omega_body;
          Eigen::Vector3d vel_formation_w = vel_leader_w + omega_w.cross(offset_w);
          Eigen::VectorXd realized_wrench = unified_controller_->getRealizedWrenchBody();

          // Arm the future observer-FF gate only while the formation is in hover.
          const bool in_hover =
              (navigator_->getNaviState() == aerial_robot_navigation::HOVER_STATE);
          formation_observer_->setFfArmed(in_hover);
          formation_observer_->update(
              unified_controller_->getFormationMass(),
              unified_controller_->getFormationInertia(),
              cog_rot_eigen, vel_formation_w, omega_body,
              realized_wrench, du);
        }
      }
    }

    if (unified_transition_count_ >= 0) unified_transition_count_++;

    ROS_DEBUG_THROTTLE(1.0, "[UnifiedCtrl %s id=%d] wrench_acc=(%.3f,%.3f,%.3f,%.4f,%.4f,%.4f) ok=%d yaw_raw=%.4f",
                       is_leader ? "LEADER" : "FOLLOWER", my_id,
                       target_wrench_acc(0), target_wrench_acc(1), target_wrench_acc(2),
                       target_wrench_acc(3), target_wrench_acc(4), target_wrench_acc(5),
                       ok, yaw_pid_raw);

    // Only leader publishes assembly-debug topics (shared global namespace)
    if (is_leader) {
      publishAssembleDebug(formation_pos, formation_vel, target_formation_pos, ok);
    }
  }

} //namespace aerial_robot_controller

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::BeetleController, aerial_robot_control::ControlBase);
