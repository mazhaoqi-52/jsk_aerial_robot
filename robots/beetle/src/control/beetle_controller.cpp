#include <beetle/control/beetle_controller.h>

using namespace std;

namespace aerial_robot_control
{
  BeetleController::BeetleController():
    GimbalrotorController(),
    pd_wrench_comp_mode_(false),
    pre_module_state_(SEPARATED),
    desired_external_wrench_(Eigen::VectorXd::Zero(6)),
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
    unified_diff_damp_gain_(0.0),
    bias_pid_settled_rate_thresh_(0.05),
    bias_pid_settled_frames_(40),
    pid_settled_count_(0),
    last_roll_i_for_settle_(0.0),
    last_pitch_i_for_settle_(0.0),
    last_yaw_i_for_settle_(0.0),
    pid_settle_tracker_init_(false),
    unified_transition_count_(-1),
    z_integral_freeze_count_(0),
    z_ki_boost_count_(0),
    yaw_in_allocation_(false),
    last_unified_z_i_ss_(0.8),
    has_unified_z_i_ss_(false),
    z_i_seed_default_(0.8),
    z_ki_boost_frames_(60),
    z_ki_boost_factor_(2.0),
    z_ki_boost_frames_default_(60),
    z_ki_boost_factor_default_(2.0),
    gains_switched_(false),
    fobs_comp_enable_(false),
    fobs_comp_force_gain_(0.0),
    fobs_comp_torque_gain_(0.0),
    fobs_comp_ff_force_limit_(0.5),
    fobs_comp_ff_torque_limit_(0.3),
    fobs_comp_ff_torque_x_(0.0),
    fobs_comp_ff_torque_y_(0.0)
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
    // D3: subscribe to the global formation observer output (leader publishes;
    // every module — leader and followers — receives it). Used as common-mode
    // reference in unified diff-damping so the differential signal is immune
    // to common-mode model error in the per-module observers.
    formation_observer_wrench_ = Eigen::VectorXd::Zero(6);
    formation_observer_wrench_stamp_ = ros::Time(0);
    formation_observer_wrench_sub_ = nh_.subscribe(
        "/assemble/formation_observer/est_ext_wrench", 1,
        &BeetleController::formationObserverWrenchCallback, this);
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

    // P0-fix: Re-init single-module momentum observer on unified exit.
    // During unified mode, externalWrenchEstimate() returns early, so
    // prev_est_wrench_timestamp_ is frozen at the pre-unified value.
    // Without reset, the first post-exit call sees dt = entire unified duration
    // → integrate_term_ explodes → est_external_wrench_ corrupted
    // → wrench_comp → ICompTerm(PITCH) → pitch crash.
    // Setting timestamp to 0 triggers the re-init path (new init_sum_momentum_).
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
    fobs_comp_ff_torque_x_ = 0.0;
    fobs_comp_ff_torque_y_ = 0.0;
    formation_desired_wrench_.setZero();
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

    // P2: Determine module count for per-N seed/boost bucketing
    int N_modules = static_cast<int>(beetle_navigator_->getAssemblyIds().size());

    if (from_hover) {
      // ===== Hover → Unified: migrate existing I-terms carefully =====

      // Outer R/P I-term is unused in unified mode (target_wrench_acc(3,4)
      // carries only FF torque, see runUnifiedControlCommon). Zero on entry
      // so that a later switch back to hover/split mode starts clean.
      pid_controllers_.at(ROLL).setErrI(0);
      pid_controllers_.at(PITCH).setErrI(0);
      pid_controllers_.at(X).setErrI(0);
      pid_controllers_.at(Y).setErrI(0);

      // --- Z I-term: de-gravity + seed injection (Plan E') ---
      // Independent mode I-term ≈ G (implicit gravity). Unified mode has explicit
      // gravity FF, so de-gravity subtracts it, then seed injects unified-mode bias.
      {
        double i_output_old = pid_controllers_.at(Z).getITerm();

        tf::Matrix3x3 uav_rot_mig = estimator_->getOrientation(Frame::COG, estimate_mode_);
        tf::Vector3 gravity_w_mig(0, 0, aerial_robot_estimation::G);
        tf::Vector3 gravity_cog_mig = uav_rot_mig.inverse() * gravity_w_mig;
        double gravity_ff_z = gravity_cog_mig.z();

        double i_output_degrav = i_output_old - gravity_ff_z;

        // P2: Select Z seed by N, fall back to global default
        double z_seed_default_n = z_i_seed_default_;
        if (z_i_seed_by_n_.count(N_modules))
          z_seed_default_n = z_i_seed_by_n_.at(N_modules);
        double seed_value = has_unified_z_i_ss_ ? last_unified_z_i_ss_ : z_seed_default_n;
        double i_output_seeded = i_output_degrav + Z_SEED_GAIN * seed_value;

        double Ki = std::max(pid_controllers_.at(Z).getIGain(), 1e-6);
        double iz_new = i_output_seeded / Ki;
        double iz_limit = pid_controllers_.at(Z).getLimitI() / Ki;
        if (!std::isfinite(iz_new)) iz_new = 0.0;
        iz_new = boost::algorithm::clamp(iz_new, -iz_limit, iz_limit);

        pid_controllers_.at(Z).setErrI(iz_new);
        z_integral_freeze_count_ = Z_INTEGRAL_FREEZE_FRAMES;

        // P2: Select Z boost params by N
        z_ki_boost_frames_ = z_ki_boost_frames_default_;
        z_ki_boost_factor_ = z_ki_boost_factor_default_;
        if (z_ki_boost_frames_by_n_.count(N_modules))
          z_ki_boost_frames_ = z_ki_boost_frames_by_n_.at(N_modules);
        if (z_ki_boost_factor_by_n_.count(N_modules))
          z_ki_boost_factor_ = z_ki_boost_factor_by_n_.at(N_modules);
        z_ki_boost_count_ = z_ki_boost_frames_;

        ROS_WARN("[UnifiedCtrl] Z de-gravity + seed (hover→unified): i_old=%.4f, gravity_ff=%.4f, "
                 "i_degrav=%.4f, seed=%.4f(×%.1f=%s, N=%d), i_seeded=%.4f, err_i=%.4f "
                 "(limit=±%.1f), freeze=%d, boost=%d(×%.1f)",
                 i_output_old, gravity_ff_z, i_output_degrav,
                 seed_value, Z_SEED_GAIN,
                 has_unified_z_i_ss_ ? "adaptive" : "default",
                 N_modules,
                 i_output_seeded, iz_new, iz_limit,
                 z_integral_freeze_count_, z_ki_boost_count_, z_ki_boost_factor_);
      }
    } else {
      // ===== Ground start → Unified =====
      // Outer R/P I-term is unused in unified mode; XY/Z are not yet accumulated
      // on the ground. Just zero everything.
      pid_controllers_.at(X).setErrI(0);
      pid_controllers_.at(Y).setErrI(0);
      pid_controllers_.at(Z).setErrI(0);
      pid_controllers_.at(ROLL).setErrI(0);
      pid_controllers_.at(PITCH).setErrI(0);

      z_integral_freeze_count_ = 0;
      z_ki_boost_count_ = 0;

      // P2: Still initialize per-N boost params for later use
      z_ki_boost_frames_ = z_ki_boost_frames_default_;
      z_ki_boost_factor_ = z_ki_boost_factor_default_;
      if (z_ki_boost_frames_by_n_.count(N_modules))
        z_ki_boost_frames_ = z_ki_boost_frames_by_n_.at(N_modules);
      if (z_ki_boost_factor_by_n_.count(N_modules))
        z_ki_boost_factor_ = z_ki_boost_factor_by_n_.at(N_modules);

      ROS_WARN("[UnifiedCtrl] Ground start: Z/RP/XY I-terms zeroed (naviState=%d, N=%d)",
               navigator_->getNaviState(), N_modules);
    }

    if (is_leader) {
      sendCascadeSetup();     // reads from unified_*_gains_ directly, order-independent
    } else {
      sendFollowerCascadeSetup();  // follower: own spinal only
      ensureUnifiedReferenceSubscription();  // still subscribe for debug/monitoring
    }
    applyUnifiedGains();      // set unified PID gains into pid_controllers_ for PC loop
    unified_transition_count_ = 0;
    unified_reference_warmup_count_ = 0;

    // Phase U2: activate formation observer on entering unified LEADER mode.
    // Follower does NOT run observer (leader is the observation point).
    if (is_leader && formation_observer_) {
      formation_observer_->reset();
      formation_observer_->setActive(true);
      ROS_INFO("[UnifiedCtrl] Formation observer activated (reset + active)");
    }
    // Reset PID-settled tracker so new takeoff starts with a clean state.
    pid_settle_tracker_init_ = false;
    pid_settled_count_ = 0;

    ROS_WARN("[UnifiedCtrl] %s id=%d mode switch: reset targets, sent cascade gains%s, "
             "applied unified PID gains, starting local warmup window (%d frames), t=%.4f",
             is_leader ? "LEADER" : "FOLLOWER",
             beetle_navigator_->getMyID(),
             is_leader ? " + alloc_inv to all spinals" : " to own spinal",
             unified_reference_warmup_frames_, ros::Time::now().toSec());
    formation_desired_wrench_.setZero();
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
         separate desired_external_wrench_ injection here — that would
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
    unified_reference_pub_.publish(msg);

    ROS_INFO_THROTTLE(1.0,
                      "[UnifiedCtrl REF_PUB] leader_id=%d stamp=%.4f mass=%.3f wrench_z=%.3f pitch_i=%.3f yaw_raw=%.3f",
                      beetle_navigator_->getMyID(),
                      msg.header.stamp.toSec(),
                      msg.formation_mass,
                      target_wrench_acc(2),
                      target_wrench_acc(4),
                      yaw_pid_raw);
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

    unified_cmd_received_ = true;
    unified_cmd_stamp_ = msg.header.stamp.isZero() ? ros::Time::now() : msg.header.stamp;

    ROS_INFO_THROTTLE(1.0,
              "[UnifiedCtrl REF_RX] follower_id=%d leader_id=%d age=%.4f wrench_z=%.3f pitch_i=%.3f yaw_raw=%.3f (debug only - follower uses local control)",
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
    // LEADER-only: send torque allocation matrix inverse and P/D gains
    // to ALL assembled modules' spinals. This configures each spinal for 1000Hz
    // P+D attitude tracking using thrustGainMapping().
    //
    // Gains are read directly from unified_*_gains_ (not pid_controllers_).
    // This eliminates call-order dependency: sendCascadeSetup() always reads
    // the correct unified gains regardless of whether applyUnifiedGains() has
    // been called yet. I-term is set to 0 for Spinal (PC handles I-term).
    //
    // NOTE: At mode-switch time, integrated_map_inv_rot_ may not be computed yet.
    // The one-shot logic inside computeUnifiedAllocation() will resend on first
    // successful computation. See cascade_alloc_sent_ flag.

    double roll_p  = unified_roll_gains_.p;
    double roll_d  = unified_roll_gains_.d;
    double pitch_p = unified_pitch_gains_.p;
    double pitch_d = unified_pitch_gains_.d;
    double yaw_d   = unified_yaw_gains_.d;

    // Cache gains for deferred one-shot resend (must be done BEFORE the attempt)
    unified_controller_->cacheCascadeGains(roll_p, roll_d, pitch_p, pitch_d, yaw_d);

    // Attempt to send now. If matrix is not yet computed, skip gains too —
    // sending cascade gains with the old independent-mode allocation matrix
    // causes thrustGainMapping() to produce wrong per-motor gains (P5 fix).
    unified_controller_->updateFormationGeometry();
    bool matrix_sent = unified_controller_->sendTorqueAllocationMatrixInv();
    if (matrix_sent) {
      unified_controller_->sendCascadeGains(roll_p, roll_d, pitch_p, pitch_d, yaw_d);
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
             "(P_r=%.1f D_r=%.1f P_p=%.1f D_p=%.1f D_y=%.1f) %zu modules + gimbal_dof=1",
             matrix_sent ? "SENT" : "DEFERRED", matrix_sent ? "SENT" : "DEFERRED",
             roll_p, roll_d, pitch_p, pitch_d, yaw_d,
             beetle_navigator_->getAssemblyIds().size());
  }

  void BeetleController::sendFollowerCascadeSetup()
  {
    // FOLLOWER-only: send P/D gains and gimbal_dof to THIS module's
    // own spinal only (via base-class publishers).
    //
    // Read directly from unified_*_gains_ (not pid_controllers_) because
    // FOLLOWER does not call applyUnifiedGains() (it doesn't run PID).
    // pid_controllers_ may still hold independent-mode values at this point.

    double roll_p  = unified_roll_gains_.p;
    double roll_d  = unified_roll_gains_.d;
    double pitch_p = unified_pitch_gains_.p;
    double pitch_d = unified_pitch_gains_.d;
    double yaw_d   = unified_yaw_gains_.d;

    // Send gains to own spinal via base-class publisher (rpy_gain_pub_)
    {
      spinal::RollPitchYawTerms rpy_gain_msg;
      rpy_gain_msg.motors.resize(1);  // torque-level path
      rpy_gain_msg.motors[0].roll_p  = static_cast<int16_t>(roll_p * 1000);
      rpy_gain_msg.motors[0].roll_i  = 0;  // I-term handled by PC
      rpy_gain_msg.motors[0].roll_d  = static_cast<int16_t>(roll_d * 1000);
      rpy_gain_msg.motors[0].pitch_p = static_cast<int16_t>(pitch_p * 1000);
      rpy_gain_msg.motors[0].pitch_i = 0;  // I-term handled by PC
      rpy_gain_msg.motors[0].pitch_d = static_cast<int16_t>(pitch_d * 1000);
      rpy_gain_msg.motors[0].yaw_d   = static_cast<int16_t>(yaw_d * 1000);
      rpy_gain_pub_.publish(rpy_gain_msg);
    }

    // Set gimbal_dof=1 on own spinal
    {
      std_msgs::UInt8 gimbal_dof_msg;
      gimbal_dof_msg.data = 1;
      gimbal_dof_pub_.publish(gimbal_dof_msg);
    }

    ROS_INFO("[UnifiedCtrl] Cascade setup (FOLLOWER id=%d): sent gains"
             "(P_r=%.1f D_r=%.1f P_p=%.1f D_p=%.1f D_y=%.1f) + gimbal_dof=1 to own spinal only",
             beetle_navigator_->getMyID(),
             roll_p, roll_d, pitch_p, pitch_d, yaw_d);
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

    // Debug: check damping ratio for oscillation diagnosis
    // For a PD system: zeta = D / (2*sqrt(P)). zeta < 0.7 => likely oscillatory.
    if (unified_xy_gains_.p > 0.0) {
      double zeta_xy = unified_xy_gains_.d / (2.0 * std::sqrt(unified_xy_gains_.p));
      ROS_INFO("[UnifiedCtrl] XY damping ratio zeta=%.3f (P=%.2f D=%.2f) %s",
               zeta_xy, unified_xy_gains_.p, unified_xy_gains_.d,
               zeta_xy < 0.7 ? "** UNDERDAMPED - may oscillate **" : "OK");
    }
    if (unified_z_gains_.p > 0.0) {
      double zeta_z = unified_z_gains_.d / (2.0 * std::sqrt(unified_z_gains_.p));
      ROS_INFO("[UnifiedCtrl] Z damping ratio zeta=%.3f (P=%.2f D=%.2f) %s",
               zeta_z, unified_z_gains_.p, unified_z_gains_.d,
               zeta_z < 0.7 ? "** UNDERDAMPED - may oscillate **" : "OK");
    }
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

        // Damping ratio check after gain change
        if (gain_set.p > 0.0) {
          double zeta = gain_set.d / (2.0 * std::sqrt(gain_set.p));
          if (zeta < 0.7) {
            ROS_WARN("[UnifiedCtrl] Damping ratio zeta=%.3f after gain change (P=%.2f D=%.2f) - UNDERDAMPED, oscillation likely!",
                     zeta, gain_set.p, gain_set.d);
          }
        }

        // Resend P/D gains to all Spinals when unified roll/pitch/yaw gains change.
        // Since sendCascadeSetup() reads from unified_*_gains_ directly, it will
        // pick up the new value automatically.
        if (level == Levels::RECONFIGURE_P_GAIN || level == Levels::RECONFIGURE_D_GAIN) {
          if (unified_controller_) {
            sendCascadeSetup();
            ROS_INFO("[UnifiedCtrl] Resent cascade gains to Spinals after dynreconf P/D change");
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
    if (my_id == leader_id_diag) {
      std::vector<int> active_ids;
      for (const auto& kv : inter_wrench_list_) {
        if (assembly_flag[kv.first] && kv.second.size() == 6) {
          active_ids.push_back(kv.first);
        }
      }
      double max_f = 0.0, max_t = 0.0;
      double sum_f2 = 0.0, sum_t2 = 0.0;
      int n_pairs = 0;
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
      double rms_f = (n_pairs > 0) ? std::sqrt(sum_f2 / n_pairs) : 0.0;
      double rms_t = (n_pairs > 0) ? std::sqrt(sum_t2 / n_pairs) : 0.0;
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

    // Differential-mode damping gain for symmetric-local unified control.
    // Injects -K * inter_wrench/mass into target_wrench_acc as passive dissipation
    // on the disagreement between per-module external-wrench estimators. Zero disables.
    getParam<double>(control_nh, "unified_diff_damp_gain", unified_diff_damp_gain_, 0.0);

    // Bias-calibration PID-settled gate (real-hardware safety): the FormationObserver
    // bias will only be calibrated once |d(R/P/Y I-term)/dt| summed drops below the
    // threshold for N consecutive frames while HOVER state is active.
    getParam<double>(control_nh, "bias_pid_settled_rate_thresh",
                     bias_pid_settled_rate_thresh_, 0.05);
    getParam<int>(control_nh, "bias_pid_settled_frames",
                  bias_pid_settled_frames_, 40);

    // Z I-term seed default for unified mode switch (Plan E')
    getParam<double>(control_nh, "z_i_seed_default", z_i_seed_default_, 0.8);
    last_unified_z_i_ss_ = z_i_seed_default_;

    // P2: Per-N seed bucketing — load z_i_seed_by_n/n2, z_i_seed_by_n/n3, etc.
    {
      ros::NodeHandle seed_nh(control_nh, "z_i_seed_by_n");
      for (int n = 2; n <= 6; n++) {
        double val;
        if (seed_nh.getParam("n" + std::to_string(n), val)) {
          z_i_seed_by_n_[n] = val;
          ROS_INFO("[SeedBucket] z_i_seed_by_n[%d] = %.4f", n, val);
        }
      }
    }

    // Pitch I-term seed removed: outer R/P I-channel disabled in unified mode.

    // P2: Per-N Z boost parameters
    getParam<int>(control_nh, "z_ki_boost_frames_default", z_ki_boost_frames_default_, 60);
    getParam<double>(control_nh, "z_ki_boost_factor_default", z_ki_boost_factor_default_, 2.0);
    z_ki_boost_frames_ = z_ki_boost_frames_default_;
    z_ki_boost_factor_ = z_ki_boost_factor_default_;
    {
      ros::NodeHandle boost_nh(control_nh, "z_ki_boost_by_n");
      for (int n = 2; n <= 6; n++) {
        int frames;
        double factor;
        if (boost_nh.getParam("n" + std::to_string(n) + "/frames", frames))
          z_ki_boost_frames_by_n_[n] = frames;
        if (boost_nh.getParam("n" + std::to_string(n) + "/factor", factor))
          z_ki_boost_factor_by_n_[n] = factor;
      }
    }

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

    // Formation observer feedforward (redesigned). Default disabled.
    // Force/torque gains scale ramped, bias-subtracted, LPF-filtered observer output;
    // ff_*_limit then hard-clamps the resulting FF acceleration.
    ros::NodeHandle obs_comp_nh(control_nh, "formation_observer_comp");
    getParam<bool>(obs_comp_nh,   "enable",          fobs_comp_enable_,           false);
    getParam<double>(obs_comp_nh, "force_gain",      fobs_comp_force_gain_,       0.0);
    getParam<double>(obs_comp_nh, "torque_gain",     fobs_comp_torque_gain_,      0.0);
    getParam<double>(obs_comp_nh, "ff_force_limit",  fobs_comp_ff_force_limit_,   0.5);
    getParam<double>(obs_comp_nh, "ff_torque_limit", fobs_comp_ff_torque_limit_,  0.3);

  }

  void BeetleController::externalWrenchEstimate()
  {
    // NOTE (unified mode): the single-module momentum observer also runs in
    // unified mode so that inter_wrench_list_[my_id] (computed by
    // calcInteractionWrench from est_wrench_list_) is populated and can be
    // consumed by the differential-mode damping term in runUnifiedControlCommon.
    //
    // Feedforward correction (Stage 1 fix): in unified mode the observer's
    // commanded-wrench input is taken from the formation QP allocation result
    // (getLocalRealizedWrenchBody(my_id)), which is the wrench this module's
    // own rotors are actually producing — NOT (single_module_mass *
    // formation_target_acc), which would create an acceleration-proportional
    // residual that contaminates inter_wrench during tilted or accelerated
    // flight. In LF / independent mode the legacy single-module expression is
    // retained.
    //
    // Only the differential component (est - mean, via calcInteractionWrench)
    // is fed back through unified_diff_damp_gain_, which is small by design.
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
    if (unified_control_mode_ && unified_controller_) {
      // Unified mode: observer feedforward must use the wrench actually produced
      // by THIS module's rotors (from the formation QP allocation), not
      // (single_mass * formation_target_acc) — which would mismatch the
      // physical force/torque this module is generating and inject a spurious
      // signal into est_external_wrench_ that scales with formation acceleration
      // (the "differential-mode contamination" problem during tilted / accelerated
      // flight).
      int my_id = beetle_navigator_->getMyID();
      target_wrench_cog = unified_controller_->getLocalRealizedWrenchBody(my_id);
      // Fall back to the legacy expression only if local realized wrench is not
      // yet available (e.g. first frame before allocation has converged).
      if (target_wrench_cog.size() != 6 || target_wrench_cog.isZero(0.0)) {
        target_wrench_cog = Eigen::VectorXd::Zero(6);
        target_wrench_cog.head(3) = mass * target_wrench_acc_cog.head(3);
        target_wrench_cog.tail(3) = inertia * target_wrench_acc_cog.tail(3);
      }
    } else {
      target_wrench_cog.head(3) = mass * target_wrench_acc_cog.head(3);
      target_wrench_cog.tail(3) = inertia * target_wrench_acc_cog.tail(3);
    }

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
    // Receive desired total external wrench for the whole assembly (body frame).
    //
    // Storage semantic:
    //   desired_external_wrench_ = FULL formation-level wrench on EVERY module.
    //   Used directly by runUnifiedControlCommon as the formation-level task FF.
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

    // Every module stores the FULL desired wrench (semantic unified across modules)
    desired_external_wrench_ = desired;

    // Only LEADER rebroadcasts to followers so every module sees the same value.
    if(beetle_navigator_->getModuleState() != LEADER) {
      return;
    }

    std::map<int, bool> assembly_flag = beetle_navigator_->getAssemblyFlags();
    int leader_id = beetle_navigator_->getLeaderID();
    ros::Time stamp = msg.header.stamp;

    // Rebroadcast FULL desired wrench to all followers so every module stores
    // the same desired_external_wrench_ (FULL semantic).
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
    // Only used in unified LEADER mode via computeUnifiedAllocation().
    // Independent-mode per-module distribution uses desiredExternalWrenchCallback() instead.
    formation_desired_wrench_(0) = msg.wrench.force.x;
    formation_desired_wrench_(1) = msg.wrench.force.y;
    formation_desired_wrench_(2) = msg.wrench.force.z;
    formation_desired_wrench_(3) = msg.wrench.torque.x;
    formation_desired_wrench_(4) = msg.wrench.torque.y;
    formation_desired_wrench_(5) = msg.wrench.torque.z;
  }

  void BeetleController::formationObserverWrenchCallback(const geometry_msgs::WrenchStamped& msg)
  {
    // D3: cache the formation-level external wrench estimate published by the
    // leader's FormationMomentumObserver. Used as common-mode reference when
    // computing the per-module differential signal in unified diff-damping.
    // Frame convention: world-frame force, body-frame torque (matches observer output).
    formation_observer_wrench_(0) = msg.wrench.force.x;
    formation_observer_wrench_(1) = msg.wrench.force.y;
    formation_observer_wrench_(2) = msg.wrench.force.z;
    formation_observer_wrench_(3) = msg.wrench.torque.x;
    formation_observer_wrench_(4) = msg.wrench.torque.y;
    formation_observer_wrench_(5) = msg.wrench.torque.z;
    formation_observer_wrench_stamp_ = msg.header.stamp.isZero() ? ros::Time::now() : msg.header.stamp;
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
    tf::Vector3 target_omega = navigator_->getTargetOmega();
    target_omega_ = cog_rot.inverse() * target_rot * target_omega;
    target_ang_acc_ = navigator_->getTargetAngAcc();

    // --- Formation geometry: each module computes locally ---
    unified_controller_->updateFormationGeometry();
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
    tf::Vector3 offset_body(cog_offset_self.x(), cog_offset_self.y(), cog_offset_self.z());
    tf::Vector3 offset_world = cog_rot * offset_body;
    tf::Vector3 formation_pos = pos_ + offset_world;
    tf::Vector3 omega_world = cog_rot * omega_;
    tf::Vector3 formation_vel = vel_ + omega_world.cross(offset_world);
    tf::Vector3 target_formation_pos = target_pos_ + target_rot * offset_body;

    // [DBG-ATT2POS] Attitude-to-position projection bias: how much pitch/roll
    // error leaks into the Z position target via offset_body rotation. At hover
    // with pitch_err 0.05 rad and offset_body.x=0.265, this is ~1.3 cm — large
    // enough to drive Z PID windup. Throttled 1 Hz.
    {
      tf::Vector3 att_proj_bias = (target_rot * offset_body) - (cog_rot * offset_body);
      ROS_INFO_THROTTLE(1.0,
        "[DBG-ATT2POS id=%d] (tgt_rot-cog_rot)*off_body=(%.4f,%.4f,%.4f) m | "
        "off_body=(%.3f,%.3f,%.3f) tgt_rpy=(%.3f,%.3f,%.3f) cur_rpy=(%.3f,%.3f,%.3f)",
        my_id,
        att_proj_bias.x(), att_proj_bias.y(), att_proj_bias.z(),
        offset_body.x(), offset_body.y(), offset_body.z(),
        target_rpy_.x(), target_rpy_.y(), target_rpy_.z(),
        rpy_.x(), rpy_.y(), rpy_.z());
    }

    // --- Position PID (X/Y/Z) with formation CoG ---
    double du = ros::Time::now().toSec() - control_timestamp_;
    if (du < 0.0) du = 0.0;
    if (du > 0.1) du = 0.1;  // clamp callback-stall gaps so PID-D and FF integrators stay sane

    // --- Task-level external wrench feedforward (towing / valve_rotation) ---
    // desired_external_wrench_ holds the FULL formation-level wrench (body frame).
    // Every module independently uses it to compute the same formation acc:
    //   acc_world = cog_rot * F_full / formation_mass
    //   ang_acc   = inertia_inv * tau_full
    // Sign: positive (drone provides +F to apply +F via interface, opposite of
    // observer FF which compensates a detected residual push).
    Eigen::VectorXd desired_total_ff = Eigen::VectorXd::Zero(6);
    if (desired_external_wrench_.norm() > 1e-6 &&
        !navigator_->getForceLandingFlag()) {
      desired_total_ff = desired_external_wrench_;
    }
    bool task_ff_active = (desired_total_ff.norm() > 1e-6);

    // DEBUG: detect large position jump that may indicate a mocap/estimator discontinuity.
    // A jump > 1.5 cm in one 40 Hz frame (25 ms) is physically implausible at hover.
    {
      static tf::Vector3 s_dbg_prev_pos(0, 0, 0);
      double djump = (formation_pos - s_dbg_prev_pos).length();
      if (djump > 0.015 && s_dbg_prev_pos.length() > 0.01)
        ROS_WARN("[PosJump id=%d] dt=%.4f |dp|=%.4f dx=%.4f dy=%.4f dz=%.4f",
                 my_id, du, djump,
                 formation_pos.x() - s_dbg_prev_pos.x(),
                 formation_pos.y() - s_dbg_prev_pos.y(),
                 formation_pos.z() - s_dbg_prev_pos.z());
      s_dbg_prev_pos = formation_pos;
    }

    // Formation observer feedforward (XY): leader-only, two-stage attenuated.
    //   stage 1 (in observer): LPF @ 0.05 Hz on bias-subtracted estimate
    //   stage 2 (here):        soft ramp * gain, then hard clamp to ff_force_limit
    // Inject via setPersistentFF since X/Y consume PID.result() downstream.
    {
      double mass_inv_f = 1.0 / std::max(unified_controller_->getFormationMass(), 0.01);
      double ff_x = 0.0, ff_y = 0.0;
      // Observer FF (leader-only, residual compensation): negative sign because
      // it counteracts a detected unmodeled external push.
      if (is_leader && fobs_comp_enable_ &&
          formation_observer_ && formation_observer_->isFfReady() &&
          !navigator_->getForceLandingFlag()) {
        double ramp = formation_observer_->getFfRampFactor();
        Eigen::Vector3d f_body = formation_observer_->getEstExternalForceBody();
        tf::Vector3 f_world = cog_rot * tf::Vector3(f_body.x(), f_body.y(), f_body.z());
        double k = ramp * fobs_comp_force_gain_;
        ff_x += boost::algorithm::clamp(-k * f_world.x() * mass_inv_f,
                                        -fobs_comp_ff_force_limit_,  fobs_comp_ff_force_limit_);
        ff_y += boost::algorithm::clamp(-k * f_world.y() * mass_inv_f,
                                        -fobs_comp_ff_force_limit_,  fobs_comp_ff_force_limit_);
      }
      // Task FF (all modules, towing / valve_rotation): positive sign, body→world rotation.
      if (task_ff_active) {
        tf::Vector3 task_world = cog_rot * tf::Vector3(desired_total_ff(0),
                                                       desired_total_ff(1),
                                                       desired_total_ff(2));
        ff_x += task_world.x() * mass_inv_f;
        ff_y += task_world.y() * mass_inv_f;
      }
      pid_controllers_.at(X).setPersistentFF(ff_x);
      pid_controllers_.at(Y).setPersistentFF(ff_y);
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
    {
      double mass_inv_z = 1.0 / std::max(unified_controller_->getFormationMass(), 0.01);
      double ff_z = 0.0;
      if (is_leader && fobs_comp_enable_ &&
          formation_observer_ && formation_observer_->isFfReady() &&
          !navigator_->getForceLandingFlag()) {
        double ramp = formation_observer_->getFfRampFactor();
        double fz_body = formation_observer_->getEstExternalForceBody().z();
        ff_z += boost::algorithm::clamp(-ramp * fobs_comp_force_gain_ * fz_body * mass_inv_z,
                                        -fobs_comp_ff_force_limit_, fobs_comp_ff_force_limit_);
      }
      // Task FF Z: body z direct (skip cog_rot) to avoid pitch-coupling instability.
      if (task_ff_active) {
        ff_z += desired_total_ff(2) * mass_inv_z;
      }
      pid_controllers_.at(Z).setPersistentFF(ff_z);
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

    if (z_integral_freeze_count_ > 0) {
      pid_controllers_.at(Z).setErrI(pid_controllers_.at(Z).getPrevErrI());
      z_integral_freeze_count_--;
    } else if (z_ki_boost_count_ > 0) {
      if (!unified_controller_->isAllocationSaturated()) {
        double clamped_err_p = pid_controllers_.at(Z).getErrP();
        double extra_increment = clamped_err_p * du * (z_ki_boost_factor_ - 1.0);
        double new_err_i = pid_controllers_.at(Z).getErrI() + extra_increment;
        double Ki = std::max(pid_controllers_.at(Z).getIGain(), 1e-6);
        double iz_limit = pid_controllers_.at(Z).getLimitI() / Ki;
        new_err_i = boost::algorithm::clamp(new_err_i, -iz_limit, iz_limit);
        pid_controllers_.at(Z).setErrI(new_err_i);
      }
      z_ki_boost_count_--;
    }
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

    // Formation observer torque feedforward (leader-only, two-stage attenuated).
    //   ROLL / PITCH → cached as fobs_comp_ff_torque_{x,y}_, ADDED below to
    //                  target_wrench_acc(3,4) on top of PID.getITerm().
    //                  This is mathematically equivalent to a one-shot
    //                  setICompTerm(ff/ki) but stateless — PID's err_i_ is
    //                  not contaminated, so it cannot accumulate frame-on-frame.
    //   YAW          → setPersistentFF (PID.result() feeds wrench_acc(5)).
    {
      fobs_comp_ff_torque_x_ = 0.0;
      fobs_comp_ff_torque_y_ = 0.0;
      double ff_yaw = 0.0;
      // Observer torque FF (leader-only, residual compensation): negative sign.
      if (is_leader && fobs_comp_enable_ &&
          formation_observer_ && formation_observer_->isFfReady() &&
          !navigator_->getForceLandingFlag()) {
        double ramp = formation_observer_->getFfRampFactor();
        Eigen::Vector3d tau_ext = formation_observer_->getEstExternalTorqueBody();
        Eigen::Vector3d alpha_ext = unified_controller_->getFormationInertia().inverse() * tau_ext;
        double k = ramp * fobs_comp_torque_gain_;
        fobs_comp_ff_torque_x_ += boost::algorithm::clamp(-k * alpha_ext.x(),
                                                          -fobs_comp_ff_torque_limit_, fobs_comp_ff_torque_limit_);
        fobs_comp_ff_torque_y_ += boost::algorithm::clamp(-k * alpha_ext.y(),
                                                          -fobs_comp_ff_torque_limit_, fobs_comp_ff_torque_limit_);
        ff_yaw += boost::algorithm::clamp(-k * alpha_ext.z(),
                                          -fobs_comp_ff_torque_limit_, fobs_comp_ff_torque_limit_);
      }
      // Task torque FF (all modules, valve_rotation etc.): positive sign, body frame.
      if (task_ff_active) {
        Eigen::Vector3d task_alpha =
            unified_controller_->getFormationInertia().inverse() * desired_total_ff.tail(3);
        fobs_comp_ff_torque_x_ += task_alpha.x();
        fobs_comp_ff_torque_y_ += task_alpha.y();
        ff_yaw += task_alpha.z();
      }
      pid_controllers_.at(YAW).setPersistentFF(ff_yaw);
    }

    pid_controllers_.at(ROLL).update(target_rpy_.x() - rpy_.x(), du_rp,
                                     target_omega_.x() - omega_.x(), target_ang_acc_.x());
    pid_controllers_.at(PITCH).update(target_rpy_.y() - rpy_.y(), du_rp,
                                      target_omega_.y() - omega_.y(), target_ang_acc_.y());
    // gimbalrotor-standard architecture (i_term_rp_calc_in_pc=true):
    //   - PID's P+D terms feed target_roll_/target_pitch_ via atan2 -> spinal
    //     cascade tracks the body attitude.
    //   - PID's I-term is routed to target_wrench_acc(3,4) below, where the
    //     formation allocation matrix turns it into per-rotor thrust trim.
    //   - This is the ONLY path that can compensate a constant external
    //     torque (e.g. CoG modelling error, payload imbalance). Without it,
    //     hover pitch sits at a P+D steady-state error (observed +0.10 rad).
    // The earlier setErrI(0) was a wind-up workaround that simultaneously
    // killed the legitimate steady-state compensation. Removed.

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
        if (z_integral_freeze_count_ == 0 && z_ki_boost_count_ == 0)
          z_integral_freeze_count_ = Z_INTEGRAL_FREEZE_FRAMES;
      }
      unified_reference_warmup_count_++;
    }

    // --- Build 6-DOF target wrench in acceleration space ---
    tf::Matrix3x3 uav_rot = estimator_->getOrientation(Frame::COG, estimate_mode_);
    tf::Vector3 target_acc_w(pid_controllers_.at(X).result(),
                             pid_controllers_.at(Y).result(),
                             pid_controllers_.at(Z).result());
    tf::Vector3 target_acc_cog = uav_rot.inverse() * target_acc_w;

    // [DIAG-A+C] Rolling stddev of XY target_acc over last N=80 samples (~2s @40Hz).
    // Quantifies how much D-term + observer noise gets injected into target attitude.
    // High stddev (> ~0.5 m/s²) at hover indicates D-gain / LPF cutoff noise amplification.
    {
      static constexpr int DIAG_BUF_N = 80;
      static double diag_ax_buf[DIAG_BUF_N] = {0};
      static double diag_ay_buf[DIAG_BUF_N] = {0};
      static int diag_idx = 0;
      static int diag_count = 0;
      diag_ax_buf[diag_idx] = target_acc_cog.x();
      diag_ay_buf[diag_idx] = target_acc_cog.y();
      diag_idx = (diag_idx + 1) % DIAG_BUF_N;
      if (diag_count < DIAG_BUF_N) diag_count++;
      double mx = 0, my = 0;
      for (int k = 0; k < diag_count; k++) { mx += diag_ax_buf[k]; my += diag_ay_buf[k]; }
      mx /= diag_count; my /= diag_count;
      double vx = 0, vy = 0;
      for (int k = 0; k < diag_count; k++) {
        double dx = diag_ax_buf[k] - mx; double dy = diag_ay_buf[k] - my;
        vx += dx*dx; vy += dy*dy;
      }
      double sx = std::sqrt(vx / std::max(diag_count, 1));
      double sy = std::sqrt(vy / std::max(diag_count, 1));
      ROS_INFO_THROTTLE(1.0,
        "[DIAG-A+C id=%d] tgt_acc_cog mean=(%.3f,%.3f) std=(%.3f,%.3f) [m/s^2 over 2s] "
        "X.PID p=%.3f i=%.3f d=%.3f Y.PID p=%.3f i=%.3f d=%.3f",
        my_id, mx, my, sx, sy,
        pid_controllers_.at(X).getPTerm(), pid_controllers_.at(X).getITerm(), pid_controllers_.at(X).getDTerm(),
        pid_controllers_.at(Y).getPTerm(), pid_controllers_.at(Y).getITerm(), pid_controllers_.at(Y).getDTerm());
    }

    Eigen::VectorXd target_wrench_acc = Eigen::VectorXd::Zero(6);
    target_wrench_acc.head(3) = Eigen::Vector3d(target_acc_cog.x(), target_acc_cog.y(), target_acc_cog.z());
    // gimbalrotor-standard architecture (i_term_rp_calc_in_pc=true):
    //   target_wrench_acc(3,4) = ROLL/PITCH PID I-term + observer/task FF.
    // The I-term provides DC compensation for constant external torques
    // (CoG offset, payload imbalance) via the formation allocation matrix.
    // P+D feed the cascade through target_roll_/target_pitch_ atan2 path.
    target_wrench_acc(3) = pid_controllers_.at(ROLL).getITerm()  + fobs_comp_ff_torque_x_;
    target_wrench_acc(4) = pid_controllers_.at(PITCH).getITerm() + fobs_comp_ff_torque_y_;
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

    // --- Differential-mode damping (D3: formation-observer common-mode reference) ---
    //
    // D3 design rationale:
    //   The legacy formulation used inter_wrench_list_[my_id], built by
    //   calcInteractionWrench as a cumulative-sum minus average over the
    //   per-module observer residuals. That removes ONLY the empirical mean
    //   of the per-module observer outputs, which is not the true formation
    //   external wrench when every module shares the same model error
    //   (same CoG/inertia mismatch → same bias). The leftover common-mode
    //   then leaks into every module's "differential" channel and shows up
    //   as a persistent diff-damping signal at hover.
    //
    //   D3 substitutes the GLOBAL formation observer output as the
    //   common-mode truth (W_truth) and computes
    //       diff_i = est_residual_list_[my_id] - W_truth / N
    //   where N is the number of assembled modules. The formation observer
    //   is an independent momentum-based estimator on the whole rigid body,
    //   so its model error structure is uncorrelated with the per-module
    //   observers and the differential is genuinely module-specific.
    //
    //   Frames: both est_residual_list_ entries (per-module observer output
    //   minus task prediction) and formation_observer_wrench_ are published
    //   in formation_body frame, so the subtraction is well-defined.
    //
    //   Gating: a single continuous freshness weight w_fresh in [0,1] derived
    //   from the age of formation_observer_wrench_stamp_ — no boolean if/else.
    //   When the formation observer has never published (stamp == 0) or its
    //   last sample is older than T_decay, w_fresh = 0 and no damping is
    //   injected.
    if (unified_diff_damp_gain_ > 0.0) {
      // Continuous-weight D3 injection.
      //   diff_i = w_fresh * ( residual_i  -  W_obs / max(1, N_assembled) )
      //   w_fresh = clip(1 - dt_obs / T_decay, 0, 1)
      constexpr double kFreshDecay = 0.5;   // [s]
      double dt_obs = formation_observer_wrench_stamp_.isZero()
                      ? std::numeric_limits<double>::infinity()
                      : (ros::Time::now() - formation_observer_wrench_stamp_).toSec();
      double w_fresh = std::max(0.0, std::min(1.0, 1.0 - dt_obs / kFreshDecay));
      std::map<int, bool> aflag = beetle_navigator_->getAssemblyFlags();
      int n_assembled = 0;
      for (const auto& kv : aflag) if (kv.second) n_assembled++;
      double inv_N = 1.0 / static_cast<double>(std::max(1, n_assembled));
      Eigen::VectorXd diff = w_fresh *
        (est_residual_list_[my_id] - formation_observer_wrench_ * inv_N);
      double mass = std::max(unified_controller_->getFormationMass(), 0.01);
      Eigen::Matrix3d inertia_inv = unified_controller_->getFormationInertia().inverse();
      target_wrench_acc.head(3) -= unified_diff_damp_gain_ * diff.head(3) / mass;
      target_wrench_acc.tail(3) -= unified_diff_damp_gain_ * (inertia_inv * diff.tail(3));

      // [DBG-DIFFDAMP-D3] Throttled 1 Hz: differential magnitude vs the
      // per-module residual and the formation-observer common-mode share.
      // A near-zero |diff| at hover with non-zero |residual| confirms that
      // the common-mode is being captured by the formation observer and
      // that the differential channel is correctly model-error-free.
      ROS_INFO_THROTTLE(1.0,
        "[DBG-DIFFDAMP-D3 id=%d] w_fresh=%.2f N=%d |res|F=%.3f T=%.3f "
        "|Wobs|F=%.3f T=%.3f |diff|F=%.3f T=%.3f gain=%.3f mass=%.2f",
        my_id, w_fresh, n_assembled,
        est_residual_list_[my_id].head(3).norm(),
        est_residual_list_[my_id].tail(3).norm(),
        formation_observer_wrench_.head(3).norm(),
        formation_observer_wrench_.tail(3).norm(),
        diff.head(3).norm(), diff.tail(3).norm(),
        unified_diff_damp_gain_, mass);
    }

    // [DBG-NANGUARD] Catch non-finite or absurd target_wrench_acc BEFORE feeding
    // it to allocation pseudoinverse. SIGSEGV in spinal pipeline is typically
    // caused by NaN/Inf propagating through Eigen path. Print FULL context so
    // the offending source is identifiable from a single log line.
    {
      bool any_bad = false;
      double max_abs = 0.0;
      for (int k = 0; k < 6; ++k) {
        double v = target_wrench_acc(k);
        if (!std::isfinite(v)) { any_bad = true; break; }
        max_abs = std::max(max_abs, std::fabs(v));
      }
      // Absurd threshold: 6 g translational or 50 rad/s^2 angular implies
      // upstream divergence. Treat as soft alarm (still proceed, no behavioural
      // change) so we capture the LAST sane frame before SIGSEGV.
      if (any_bad || max_abs > 60.0) {
        ROS_ERROR(
          "[DBG-NANGUARD id=%d] wrench_acc=(%.3f,%.3f,%.3f, %.4f,%.4f,%.4f) "
          "pitch_I=%.4f roll_I=%.4f yaw_I=%.4f "
          "fobs_ff_tx=%.4f fobs_ff_ty=%.4f "
          "pos=(%.3f,%.3f,%.3f) tgt_pos=(%.3f,%.3f,%.3f) rpy=(%.3f,%.3f,%.3f) tgt_rpy=(%.3f,%.3f,%.3f) "
          "off_body=(%.3f,%.3f,%.3f) any_bad=%d max_abs=%.3f",
          my_id,
          target_wrench_acc(0), target_wrench_acc(1), target_wrench_acc(2),
          target_wrench_acc(3), target_wrench_acc(4), target_wrench_acc(5),
          pid_controllers_.at(PITCH).getITerm(), pid_controllers_.at(ROLL).getITerm(),
          pid_controllers_.at(YAW).getITerm(),
          fobs_comp_ff_torque_x_, fobs_comp_ff_torque_y_,
          pos_.x(), pos_.y(), pos_.z(),
          target_pos_.x(), target_pos_.y(), target_pos_.z(),
          rpy_.x(), rpy_.y(), rpy_.z(),
          target_rpy_.x(), target_rpy_.y(), target_rpy_.z(),
          offset_body.x(), offset_body.y(), offset_body.z(),
          any_bad ? 1 : 0, max_abs);
      }
    }

    setTargetWrenchAccCog(target_wrench_acc);

    // --- Run unified 6-DOF allocation ---
    unified_controller_->clearFormationModelOverride();
    bool ok = unified_controller_->computeUnifiedAllocation(target_wrench_acc, formation_desired_wrench_, yaw_pid_raw);

    // [DBG-NANGUARD2] Detect non-finite allocation output: if the pseudoinverse
    // produced NaN, downstream spinal packing will likely SIGSEGV.
    if (ok) {
      const Eigen::VectorXd& fw = formation_desired_wrench_;
      bool fw_bad = false;
      for (int k = 0; k < fw.size(); ++k) {
        if (!std::isfinite(fw(k))) { fw_bad = true; break; }
      }
      if (fw_bad) {
        ROS_ERROR("[DBG-NANGUARD2 id=%d] formation_desired_wrench has NaN/Inf, size=%ld",
                  my_id, (long)fw.size());
      }
    }

    if (ok) {
      publishLocalUnifiedCommand();
      publishLocalUnifiedTorqueAllocationMatrixInv();
      if (is_leader) {
        publishUnifiedReference(target_wrench_acc, formation_desired_wrench_, yaw_pid_raw);
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

          // PID-settled gate: block bias calibration while R/P/Y I-terms are
          // still drifting (otherwise a real cog model error would be absorbed
          // as observer bias and the FF path would stop compensating it).
          const bool in_hover =
              (navigator_->getNaviState() == aerial_robot_navigation::HOVER_STATE);
          bool pid_settled = false;
          double last_drift = 0.0;
          if (!in_hover) {
            pid_settled_count_ = 0;
            pid_settle_tracker_init_ = false;
          } else {
            double cur_roll_i  = pid_controllers_.at(ROLL).getITerm();
            double cur_pitch_i = pid_controllers_.at(PITCH).getITerm();
            double cur_yaw_i   = pid_controllers_.at(YAW).getITerm();
            if (!pid_settle_tracker_init_) {
              pid_settle_tracker_init_ = true;
              pid_settled_count_       = 0;
            } else {
              last_drift = std::fabs(cur_roll_i  - last_roll_i_for_settle_)
                         + std::fabs(cur_pitch_i - last_pitch_i_for_settle_)
                         + std::fabs(cur_yaw_i   - last_yaw_i_for_settle_);
              if (last_drift < bias_pid_settled_rate_thresh_) pid_settled_count_++;
              else                                            pid_settled_count_ = 0;
            }
            last_roll_i_for_settle_  = cur_roll_i;
            last_pitch_i_for_settle_ = cur_pitch_i;
            last_yaw_i_for_settle_   = cur_yaw_i;
            pid_settled = (pid_settled_count_ >= bias_pid_settled_frames_);
            ROS_INFO_THROTTLE(2.0,
                "[UnifiedCtrl] PID-settle gate: count=%d/%d, drift=%.4f (<%.4f?), settled=%d",
                pid_settled_count_, bias_pid_settled_frames_,
                last_drift, bias_pid_settled_rate_thresh_, pid_settled ? 1 : 0);
          }
          formation_observer_->setFfArmed(in_hover && pid_settled);
          formation_observer_->update(
              unified_controller_->getFormationMass(),
              unified_controller_->getFormationInertia(),
              cog_rot_eigen, vel_formation_w, omega_body,
              realized_wrench, du);
        }
      }
    }

    if (unified_transition_count_ >= 0) unified_transition_count_++;

    ROS_INFO_THROTTLE(1.0, "[UnifiedCtrl %s id=%d] wrench_acc=(%.3f,%.3f,%.3f,%.4f,%.4f,%.4f) ok=%d yaw_raw=%.4f",
                      is_leader ? "LEADER" : "FOLLOWER", my_id,
                      target_wrench_acc(0), target_wrench_acc(1), target_wrench_acc(2),
                      target_wrench_acc(3), target_wrench_acc(4), target_wrench_acc(5),
                      ok, yaw_pid_raw);

    // Adaptive SS seed tracking for Z (leader only - followers use YAML default).
    // Pitch SS seeding removed: outer R/P I-term is no longer used in unified mode.
    if (is_leader && z_ki_boost_count_ == 0 && z_integral_freeze_count_ == 0) {
      double err_z_abs = std::abs(target_pos_.z() - formation_pos.z());
      double vel_z_abs = std::abs(vel_.z());
      if (err_z_abs < 0.05 && vel_z_abs < 0.05) {
        last_unified_z_i_ss_ = (1.0 - Z_SEED_LPF_ALPHA) * last_unified_z_i_ss_
                              + Z_SEED_LPF_ALPHA * pid_controllers_.at(Z).getITerm();
        has_unified_z_i_ss_ = true;
      }
    }

    // Only leader publishes assembly-debug topics (shared global namespace)
    if (is_leader) {
      publishAssembleDebug(formation_pos, formation_vel, target_formation_pos, ok);
    }
  }

} //namespace aerial_robot_controller

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::BeetleController, aerial_robot_control::ControlBase);
