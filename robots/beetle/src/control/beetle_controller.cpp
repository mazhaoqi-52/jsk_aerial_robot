#include <beetle/control/beetle_controller.h>

using namespace std;

namespace aerial_robot_control
{
  BeetleController::BeetleController():
    GimbalrotorController(),
    pd_wrench_comp_mode_(false),
    pre_module_state_(SEPARATED),
    des_wrench_pub_flag_(false),
    desired_external_wrench_(Eigen::VectorXd::Zero(6)),
    formation_desired_wrench_(Eigen::VectorXd::Zero(6)),
    unified_control_mode_(false),
    prev_unified_control_mode_(false),
    unified_cmd_received_(false),
    follower_unified_active_(false),
    unified_transition_count_(-1),
    z_integral_freeze_count_(0),
    z_ki_boost_count_(0),
    rp_integral_freeze_count_(0),
    rp_ki_boost_count_(0),
    rp_i_keep_ratio_(0.5),
    spinal_gains_zeroed_(false),
    cascade_roll_p_(8.0),
    cascade_roll_d_(5.0),
    cascade_pitch_p_(8.0),
    cascade_pitch_d_(5.0),
    cascade_yaw_d_(2.5),
    yaw_alloc_weight_(1.0),
    last_unified_z_i_ss_(0.8),
    has_unified_z_i_ss_(false),
    z_i_seed_default_(0.8),
    last_unified_pitch_i_ss_(-1.5),
    has_unified_pitch_i_ss_(false),
    pitch_i_seed_default_(-0.55),
    z_ki_boost_frames_(60),
    z_ki_boost_factor_(2.0),
    z_ki_boost_frames_default_(60),
    z_ki_boost_factor_default_(2.0),
    has_cached_independent_cmd_(false),
    gains_switched_(false),
    formation_obs_comp_enable_(false),
    formation_obs_comp_z_gain_(1.0),
    formation_obs_comp_xy_gain_(1.0),
    formation_obs_comp_torque_gain_(1.0)
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
    formation_desired_wrench_sub_ = nh_.subscribe("formation_desired_wrench", 1, &BeetleController::formationDesiredWrenchCallback, this);
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

    prev_comp_update_time_ = -1;

    // Initialize unified controller (unified_control_mode_ is read by rosParamInit)
    unified_controller_ = std::make_shared<BeetleUnifiedController>();
    unified_controller_->initialize(nh_, beetle_robot_model_, beetle_navigator_, estimator_);

    // Initialize formation-level momentum observer (Phase U2)
    formation_observer_ = std::make_shared<FormationMomentumObserver>();
    formation_observer_->initialize(nh_);

    // FOLLOWER: subscribe to unified commands from LEADER
    // Topic names match what BeetleUnifiedController::publishCommands() publishes
    std::string my_ns = std::string("/") + beetle_navigator_->getMyName()
                        + std::to_string(beetle_navigator_->getMyID());
    unified_thrust_sub_ = nh_.subscribe(my_ns + "/unified_thrust_cmd", 1,
                                        &BeetleController::unifiedThrustCallback, this);
    // Publishers to this module's own spinal (same topic names as GimbalrotorController)
    follower_thrust_pub_ = nh_.advertise<spinal::FourAxisCommand>("four_axes/command", 1);
    follower_gimbal_pub_ = nh_.advertise<sensor_msgs::JointState>("gimbals_ctrl", 1);

    // FOLLOWER Ready Sync (P2.1): publisher created lazily when entering unified mode,
    // because we need to know the leader's namespace (leader_id may change).
    follower_ready_sent_ = false;

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

    // UO-4: clear observer feedforward on mode exit to avoid stale values
    pid_controllers_.at(X).setPersistentFF(0.0);
    pid_controllers_.at(Y).setPersistentFF(0.0);
    pid_controllers_.at(Z).setPersistentFF(0.0);
    pid_controllers_.at(ROLL).setPersistentFF(0.0);
    pid_controllers_.at(PITCH).setPersistentFF(0.0);
    pid_controllers_.at(YAW).setPersistentFF(0.0);
    formation_desired_wrench_.setZero();
  }

  void BeetleController::initUnifiedLeaderMode()
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

      // --- Roll/Pitch I-term: partial retention + seed injection + freeze/boost ---
      // Pitch has significant SS bias in unified mode (formation geometry offset),
      // so we preload a seed. Roll bias ≈ 0 → partial retention only.
      {
        double old_roll_i = pid_controllers_.at(ROLL).getErrI();
        double old_pitch_i = pid_controllers_.at(PITCH).getErrI();

        double new_roll_i = rp_i_keep_ratio_ * old_roll_i;

        // P2: Select pitch seed by N, fall back to global default
        double pitch_seed_default_n = pitch_i_seed_default_;
        if (pitch_i_seed_by_n_.count(N_modules))
          pitch_seed_default_n = pitch_i_seed_by_n_.at(N_modules);
        double pitch_seed = has_unified_pitch_i_ss_ ? last_unified_pitch_i_ss_ : pitch_seed_default_n;
        double new_pitch_i = rp_i_keep_ratio_ * old_pitch_i + PITCH_SEED_GAIN * pitch_seed;

        // Clamp to unified-mode I limits (err_i domain = limit_i / Ki)
        double Ki_roll = std::max(unified_roll_gains_.i, 1e-6);
        double Ki_pitch = std::max(unified_pitch_gains_.i, 1e-6);
        double roll_i_limit = unified_roll_gains_.limit_i / Ki_roll;
        double pitch_i_limit = unified_pitch_gains_.limit_i / Ki_pitch;
        new_roll_i = boost::algorithm::clamp(new_roll_i, -roll_i_limit, roll_i_limit);
        new_pitch_i = boost::algorithm::clamp(new_pitch_i, -pitch_i_limit, pitch_i_limit);

        pid_controllers_.at(ROLL).setErrI(new_roll_i);
        pid_controllers_.at(PITCH).setErrI(new_pitch_i);
        rp_integral_freeze_count_ = RP_INTEGRAL_FREEZE_FRAMES;
        rp_ki_boost_count_ = RP_KI_BOOST_FRAMES;

        ROS_WARN("[UnifiedCtrl] RP I-term transition (hover→unified): "
                 "roll_i: old=%.4f → new=%.4f (keep×%.1f), "
                 "pitch_i: old=%.4f → new=%.4f (keep×%.1f + seed=%.4f×%.1f=%s), "
                 "freeze=%d, boost=%d(×%.1f)",
                 old_roll_i, new_roll_i, rp_i_keep_ratio_,
                 old_pitch_i, new_pitch_i, rp_i_keep_ratio_,
                 pitch_seed, PITCH_SEED_GAIN,
                 has_unified_pitch_i_ss_ ? "adaptive" : "default",
                 rp_integral_freeze_count_, rp_ki_boost_count_, RP_KI_BOOST_FACTOR);
      }
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
      // ===== Ground start → Unified: inject pitch I-seed =====
      // PID hasn't accumulated meaningful I-terms yet (robot is on the ground).
      // Unified controlCore() has explicit gravity FF in wrench_acc for Z,
      // so Z PID can accumulate from zero. But pitch needs an initial seed
      // to compensate formation CoG offset from the first frame — otherwise
      // cascade P+D alone (P=8, D=5) cannot hold pitch against the offset
      // during the gravity ramp, causing pitch divergence on real hardware.
      pid_controllers_.at(X).setErrI(0);
      pid_controllers_.at(Y).setErrI(0);
      pid_controllers_.at(Z).setErrI(0);
      pid_controllers_.at(ROLL).setErrI(0);

      // Pitch I-seed: same logic as hover→unified, compensating CoG offset
      {
        double pitch_seed_default_n = pitch_i_seed_default_;
        if (pitch_i_seed_by_n_.count(N_modules))
          pitch_seed_default_n = pitch_i_seed_by_n_.at(N_modules);
        double pitch_seed = has_unified_pitch_i_ss_ ? last_unified_pitch_i_ss_ : pitch_seed_default_n;
        double new_pitch_i = PITCH_SEED_GAIN * pitch_seed;

        // Clamp to unified-mode pitch I limit
        double Ki_pitch = std::max(unified_pitch_gains_.i, 1e-6);
        double pitch_i_limit = unified_pitch_gains_.limit_i / Ki_pitch;
        new_pitch_i = boost::algorithm::clamp(new_pitch_i, -pitch_i_limit, pitch_i_limit);

        pid_controllers_.at(PITCH).setErrI(new_pitch_i);
        rp_integral_freeze_count_ = RP_INTEGRAL_FREEZE_FRAMES;
        rp_ki_boost_count_ = RP_KI_BOOST_FRAMES;

        ROS_WARN("[UnifiedCtrl] Ground start: pitch I-seed=%.4f (x%.1f=%s, N=%d), "
                 "err_i=%.4f (limit=+/-%.1f), freeze=%d, boost=%d(x%.1f)",
                 pitch_seed, PITCH_SEED_GAIN,
                 has_unified_pitch_i_ss_ ? "adaptive" : "default", N_modules,
                 new_pitch_i, pitch_i_limit,
                 rp_integral_freeze_count_, rp_ki_boost_count_, RP_KI_BOOST_FACTOR);
      }

      z_integral_freeze_count_ = 0;
      z_ki_boost_count_ = 0;

      // P2: Still initialize per-N boost params for later use
      z_ki_boost_frames_ = z_ki_boost_frames_default_;
      z_ki_boost_factor_ = z_ki_boost_factor_default_;
      if (z_ki_boost_frames_by_n_.count(N_modules))
        z_ki_boost_frames_ = z_ki_boost_frames_by_n_.at(N_modules);
      if (z_ki_boost_factor_by_n_.count(N_modules))
        z_ki_boost_factor_ = z_ki_boost_factor_by_n_.at(N_modules);

      ROS_WARN("[UnifiedCtrl] Ground start: Z/Roll/XY I-terms zeroed (naviState=%d, N=%d)",
               navigator_->getNaviState(), N_modules);
    }

    sendCascadeSetup();
    applyUnifiedGains();
    spinal_gains_zeroed_ = true;
    unified_transition_count_ = 0;
    unified_controller_->resetFollowerReady();  // P2.1: start fresh ready tracking

    // Phase U2: activate formation observer on entering unified LEADER mode
    if (formation_observer_) {
      formation_observer_->reset();
      formation_observer_->setActive(true);
      ROS_INFO("[UnifiedCtrl] Formation observer activated (reset + active)");
    }

    ROS_WARN("[UnifiedCtrl] LEADER mode switch: reset targets, sent cascade gains + alloc_inv to all spinals, "
             "applied unified PID gains, waiting for %d FOLLOWERs, t=%.4f",
             unified_controller_->pendingFollowerCount(), ros::Time::now().toSec());
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

    // ======== Unified Control Mode ========
    // LEADER does full 6-DOF PID (position + attitude) → formation allocation.
    // Spinal acts as pure PWM executor (zero attitude gains).
    if (unified_control_mode_ && module_state == LEADER && module_state != SEPARATED) {
      // Detect mode switch: reset targets and zero spinal gains
      if (pre_module_state_ != LEADER || !prev_unified_control_mode_) {
        initUnifiedLeaderMode();
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

      // T3.2: In unified mode, the PC-side attitude PID controls roll/pitch directly
      // (spinal attitude PID is zeroed). Use FinalTargetBaselinkRPY as the attitude
      // setpoint — this is what the user commands via joystick or FlightNav messages.
      // In independent mode, target_rpy_ roll/pitch comes from position PID (acc→angle),
      // but in unified mode we do full 6-DOF PID, so we use the explicit command.
      {
        tf::Vector3 baselink_rpy = beetle_navigator_->getFinalTargetBaselinkRPY();
        target_rpy_.setX(baselink_rpy.x());  // roll target
        target_rpy_.setY(baselink_rpy.y());  // pitch target
        // yaw is already set from navigator_->getTargetRPY().z()
      }

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

      // UO-4: inject formation observer X/Y horizontal force as feedforward.
      // f_ext_body → world frame via cog_rot → divide by M → negate (oppose disturbance).
      // Only active when: comp enabled, bias calibrated, not force-landing.
      if (formation_obs_comp_enable_ &&
          formation_observer_ && formation_observer_->isBiasCalibrated() &&
          !navigator_->getForceLandingFlag())
      {
        Eigen::Vector3d f_body = formation_observer_->getEstExternalForceBody();
        tf::Vector3 f_world = cog_rot * tf::Vector3(f_body.x(), f_body.y(), f_body.z());
        double mass = std::max(unified_controller_->getFormationMass(), 0.01);
        pid_controllers_.at(X).setPersistentFF(-formation_obs_comp_xy_gain_ * f_world.x() / mass);
        pid_controllers_.at(Y).setPersistentFF(-formation_obs_comp_xy_gain_ * f_world.y() / mass);
      }
      else
      {
        pid_controllers_.at(X).setPersistentFF(0.0);
        pid_controllers_.at(Y).setPersistentFF(0.0);
      }

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

      // UO-4: inject formation observer estimated external force as slow Z feedforward.
      // Compensation: if observer detects downward load (f_body_z < 0), add upward FF.
      // Only active when: comp enabled, bias calibrated, not force-landing.
      // Sign: ff = -f_ext_body_z / M  (oppose the estimated load direction).
      // NOTE: verify sign with E2 experiment before trusting for real compensation.
      if (formation_obs_comp_enable_ &&
          formation_observer_ && formation_observer_->isBiasCalibrated() &&
          !navigator_->getForceLandingFlag())
      {
        double fz_body = formation_observer_->getEstExternalForceBody().z();
        double ff_z = -formation_obs_comp_z_gain_ * fz_body /
                      std::max(unified_controller_->getFormationMass(), 0.01);
        pid_controllers_.at(Z).setPersistentFF(ff_z);
        ROS_INFO_THROTTLE(2.0, "[UO4_FF] f_body_z=%.3f N → ff_z=%.4f m/s²",
                          fz_body, ff_z);
      }
      else
      {
        pid_controllers_.at(Z).setPersistentFF(0.0);
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
        if (!unified_controller_->isAllocationSaturated()) {
          // Apply boost: add extra (boost_factor - 1) × err_p × dt to err_i
          double clamped_err_p = pid_controllers_.at(Z).getErrP();  // already clamped by PID::update
          double extra_increment = clamped_err_p * du * (z_ki_boost_factor_ - 1.0);
          double new_err_i = pid_controllers_.at(Z).getErrI() + extra_increment;

          // Respect err_i limits
          double Ki = std::max(pid_controllers_.at(Z).getIGain(), 1e-6);
          double iz_limit = pid_controllers_.at(Z).getLimitI() / Ki;
          new_err_i = boost::algorithm::clamp(new_err_i, -iz_limit, iz_limit);
          pid_controllers_.at(Z).setErrI(new_err_i);
        }

        z_ki_boost_count_--;
      }

      if(navigator_->getForceLandingFlag()) {
        pid_controllers_.at(Z).setLimitP(z_p_limit);
        pid_controllers_.at(Z).setErrP(0);
      }

      // --- Attitude PID (Roll/Pitch/Yaw) ---
      // Unified path bypasses PoseLinearController::controlCore(), so replicate
      // the start_rp_integration_ height check here (same logic as base class).
      if (!start_rp_integration_) {
        if (pos_.z() - navigator_->getInitHeight() > start_rp_integration_height_) {
          start_rp_integration_ = true;
          spinal::FlightConfigCmd flight_config_cmd;
          flight_config_cmd.cmd = spinal::FlightConfigCmd::INTEGRATION_CONTROL_ON_CMD;
          navigator_->getFlightConfigPublisher().publish(flight_config_cmd);
          ROS_WARN("[UnifiedCtrl] start roll/pitch I control (height threshold passed)");
        }
      }
      double du_rp = du;
      if(!start_rp_integration_) du_rp = 0;

      // UO-4: inject formation observer torque estimate as Roll/Pitch/Yaw feedforward.
      // tau_ext (N·m, body) → alpha_ext (rad/s²) via I^{-1}; negate to oppose the disturbance.
      if (formation_obs_comp_enable_ &&
          formation_observer_ && formation_observer_->isBiasCalibrated() &&
          !navigator_->getForceLandingFlag())
      {
        Eigen::Vector3d tau_ext = formation_observer_->getEstExternalTorqueBody();
        Eigen::Vector3d alpha_ext = unified_controller_->getFormationInertia().inverse() * tau_ext;
        double tgain = formation_obs_comp_torque_gain_;
        pid_controllers_.at(ROLL).setPersistentFF(-tgain * alpha_ext.x());
        pid_controllers_.at(PITCH).setPersistentFF(-tgain * alpha_ext.y());
        pid_controllers_.at(YAW).setPersistentFF(-tgain * alpha_ext.z());
      }
      else
      {
        pid_controllers_.at(ROLL).setPersistentFF(0.0);
        pid_controllers_.at(PITCH).setPersistentFF(0.0);
        pid_controllers_.at(YAW).setPersistentFF(0.0);
      }

      pid_controllers_.at(ROLL).update(target_rpy_.x() - rpy_.x(), du_rp,
                                       target_omega_.x() - omega_.x(), target_ang_acc_.x());
      pid_controllers_.at(PITCH).update(target_rpy_.y() - rpy_.y(), du_rp,
                                        target_omega_.y() - omega_.y(), target_ang_acc_.y());

      // --- Roll/Pitch I-term freeze + boost (mirrors Z axis logic) ---
      // Freeze: revert I-term to pre-update value for the first few frames
      if (rp_integral_freeze_count_ > 0) {
        pid_controllers_.at(ROLL).setErrI(pid_controllers_.at(ROLL).getPrevErrI());
        pid_controllers_.at(PITCH).setErrI(pid_controllers_.at(PITCH).getPrevErrI());
        rp_integral_freeze_count_--;
      }
      // Boost: after freeze ends, accelerate I-term convergence
      else if (rp_ki_boost_count_ > 0) {
        if (!unified_controller_->isAllocationSaturated()) {
          // Roll boost
          {
            double err_p_roll = pid_controllers_.at(ROLL).getErrP();
            double extra = err_p_roll * du_rp * (RP_KI_BOOST_FACTOR - 1.0);
            double new_i = pid_controllers_.at(ROLL).getErrI() + extra;
            double Ki_r = std::max(pid_controllers_.at(ROLL).getIGain(), 1e-6);
            double lim_r = pid_controllers_.at(ROLL).getLimitI() / Ki_r;
            new_i = boost::algorithm::clamp(new_i, -lim_r, lim_r);
            pid_controllers_.at(ROLL).setErrI(new_i);
          }
          // Pitch boost
          {
            double err_p_pitch = pid_controllers_.at(PITCH).getErrP();
            double extra = err_p_pitch * du_rp * (RP_KI_BOOST_FACTOR - 1.0);
            double new_i = pid_controllers_.at(PITCH).getErrI() + extra;
            double Ki_p = std::max(pid_controllers_.at(PITCH).getIGain(), 1e-6);
            double lim_p = pid_controllers_.at(PITCH).getLimitI() / Ki_p;
            new_i = boost::algorithm::clamp(new_i, -lim_p, lim_p);
            pid_controllers_.at(PITCH).setErrI(new_i);
          }
        }

        rp_ki_boost_count_--;
      }

      double err_yaw = angles::shortest_angular_distance(rpy_.z(), target_rpy_.z());
      double err_omega_z = target_omega_.z() - omega_.z();
      if(!need_yaw_d_control_) err_omega_z = target_omega_.z();
      pid_controllers_.at(YAW).update(err_yaw, du, err_omega_z, target_ang_acc_.z());

      control_timestamp_ = ros::Time::now().toSec();

      // --- P2.1: FOLLOWER Ready Gate ---
      // If not all FOLLOWERs have reported ready, freeze ALL I-terms to prevent
      // the outer loop from building up corrections while inner loops aren't synced.
      // PID still runs (position tracking), allocation still runs (FOLLOWERs get commands
      // which will trigger their ready ack), but I-terms are held constant.
      if (!unified_controller_->allFollowersReady()) {
        // Hover → Unified: freeze I-terms to prevent buildup while inner loops aren't synced.
        // Ground start (TAKEOFF_STATE): skip freeze — I-terms start from zero, freezing
        // only delays Z integral accumulation and slows the thrust ramp.
        if (navigator_->getNaviState() != aerial_robot_navigation::TAKEOFF_STATE) {
          pid_controllers_.at(Z).setErrI(pid_controllers_.at(Z).getPrevErrI());
          pid_controllers_.at(ROLL).setErrI(pid_controllers_.at(ROLL).getPrevErrI());
          pid_controllers_.at(PITCH).setErrI(pid_controllers_.at(PITCH).getPrevErrI());
          pid_controllers_.at(X).setErrI(pid_controllers_.at(X).getPrevErrI());
          pid_controllers_.at(Y).setErrI(pid_controllers_.at(Y).getPrevErrI());

          if (z_integral_freeze_count_ == 0 && z_ki_boost_count_ == 0) {
            z_integral_freeze_count_ = Z_INTEGRAL_FREEZE_FRAMES;
          }
          if (rp_integral_freeze_count_ == 0 && rp_ki_boost_count_ == 0) {
            rp_integral_freeze_count_ = RP_INTEGRAL_FREEZE_FRAMES;
          }
        }

        unified_controller_->incrementFollowerReadyWait();
        ROS_WARN_THROTTLE(0.5, "[UnifiedCtrl LEADER] Waiting for %d FOLLOWERs to report ready "
                          "(frame %d, I-terms %s)",
                          unified_controller_->pendingFollowerCount(),
                          unified_controller_->getFollowerReadyWaitCount(),
                          (navigator_->getNaviState() == aerial_robot_navigation::TAKEOFF_STATE) ? "accumulating" : "frozen");
      }

      // --- Build 6-DOF target wrench in acceleration space ---
      // CASCADE: Roll/Pitch use I-TERM ONLY in wrench_acc.
      // P+D are handled by spinal at 1000Hz using cascade gains.
      // Yaw uses full PID (no cascade for yaw yet).
      tf::Matrix3x3 uav_rot = estimator_->getOrientation(Frame::COG, estimate_mode_);
      tf::Vector3 target_acc_w(pid_controllers_.at(X).result(),
                               pid_controllers_.at(Y).result(),
                               pid_controllers_.at(Z).result());
      tf::Vector3 target_acc_cog = uav_rot.inverse() * target_acc_w;

      Eigen::VectorXd target_wrench_acc = Eigen::VectorXd::Zero(6);
      target_wrench_acc.head(3) = Eigen::Vector3d(target_acc_cog.x(), target_acc_cog.y(), target_acc_cog.z());
      // Roll/Pitch: I-term only (spinal does P+D at 1000Hz)
      target_wrench_acc(3) = pid_controllers_.at(ROLL).getITerm();
      target_wrench_acc(4) = pid_controllers_.at(PITCH).getITerm();
      // Yaw: full PID result (no cascade)
      target_wrench_acc(5) = pid_controllers_.at(YAW).result();

      // Gravity feedforward: spinal's inner loop sees angles[0/1] as target roll/pitch,
      // but does NOT add gravity. PC must provide gravity compensation in body Z.
      // During TAKEOFF_STATE, ramp gravity FF from 0→1 over ~0.5s (20 frames @40Hz)
      // to avoid instantaneous full-thrust "pop-up". In HOVER/LAND the ramp is 1.0.
      {
        tf::Matrix3x3 uav_rot_ff = estimator_->getOrientation(Frame::COG, estimate_mode_);
        tf::Vector3 gravity_w(0, 0, aerial_robot_estimation::G);
        tf::Vector3 gravity_cog = uav_rot_ff.inverse() * gravity_w;
        double gravity_ramp = 1.0;
        if (navigator_->getNaviState() == aerial_robot_navigation::TAKEOFF_STATE) {
          constexpr int GRAVITY_RAMP_FRAMES = 20;  // ~0.5s @40Hz
          gravity_ramp = std::min(static_cast<double>(unified_transition_count_) / GRAVITY_RAMP_FRAMES, 1.0);
        }
        target_wrench_acc.head(3) += gravity_ramp * Eigen::Vector3d(gravity_cog.x(), gravity_cog.y(), gravity_cog.z());
      }

      // NOTE: Gyro compensation (ω × I·ω) is NOT added here.
      // In cascade mode, spinal handles gyro internally at 1000Hz.

      // Store for external wrench estimator
      setTargetWrenchAccCog(target_wrench_acc);

      // --- Yaw allocation weight: reduce yaw PID noise coupling into base_thrust ---
      // Scale wrench_acc(5) AFTER storing for ext-wrench estimator (which needs true value)
      // but BEFORE allocation. candidate_yaw_term inside computeUnifiedAllocation also
      // uses the scaled value, so both paths (base_thrust and yaw_term) are consistent.
      double yaw_raw = target_wrench_acc(5);
      target_wrench_acc(5) *= yaw_alloc_weight_;

      // --- Run unified 6-DOF allocation ---
      // formation_desired_wrench_ is set via the formation_desired_wrench topic (feedforward).
      // It carries the full 6D contact wrench for manipulation tasks (pulling, valve rotation).
      bool ok = unified_controller_->computeUnifiedAllocation(target_wrench_acc, formation_desired_wrench_);

      if (ok) {
        // Publish commands to all FOLLOWERs
        unified_controller_->publishCommands();

        // LEADER also sends its own command to its own spinal
        // CASCADE format: vectoring base_thrust[motor_num*rotor_coef] + angles[roll, pitch, yaw_term]
        int my_id = beetle_navigator_->getMyID();
        std::vector<int> assembled_ids = beetle_navigator_->getAssemblyIds();
        int my_index = -1;
        for (size_t m = 0; m < assembled_ids.size(); m++) {
          if (assembled_ids[m] == my_id) { my_index = m; break; }
        }
        if (my_index >= 0) {
          const Eigen::VectorXd& vf = unified_controller_->getTargetVectoringForce();
          int elems_per_module = motor_num_ * rotor_coef_;
          int col_start = my_index * elems_per_module;
          spinal::FourAxisCommand my_thrust_msg;
          my_thrust_msg.base_thrust.resize(elems_per_module);
          for (int i = 0; i < elems_per_module; i++)
            my_thrust_msg.base_thrust[i] = static_cast<float>(vf(col_start + i));
          my_thrust_msg.angles[0] = unified_controller_->getTargetRoll();
          my_thrust_msg.angles[1] = unified_controller_->getTargetPitch();
          my_thrust_msg.angles[2] = unified_controller_->getCandidateYawTerm();
          follower_thrust_pub_.publish(my_thrust_msg);
        }

        // ---- Formation Momentum Observer (Phase U2) ----
        // Feed the observer with formation-level data.
        // Input: realized wrench from allocation (A * f), NOT PID command.
        // This is cascade-agnostic and represents the actual control applied.
        if (formation_observer_ && formation_observer_->isActive()) {
          // Formation CoG velocity in world frame:
          //   v_formation = v_leader + omega × r_offset
          // where r_offset = formation_cog_offset in world frame.
          tf::Matrix3x3 uav_rot_obs = estimator_->getOrientation(Frame::COG, estimate_mode_);
          Eigen::Matrix3d cog_rot_eigen;
          tf::matrixTFToEigen(uav_rot_obs, cog_rot_eigen);

          const Eigen::Vector3d& cog_offset = unified_controller_->getFormationCogOffset();
          Eigen::Vector3d offset_w = cog_rot_eigen * cog_offset;

          // Use IMU-filtered signals (same as single-module observer) to avoid
          // formation inertia amplifying raw IMU noise (~50x without filtering).
          auto imu_handler_obs = boost::dynamic_pointer_cast<sensor_plugin::Imu>(
              estimator_->getImuHandler(0));
          Eigen::Vector3d vel_leader_w, omega_body;
          tf::vectorTFToEigen(imu_handler_obs->getFilteredVelCog(), vel_leader_w);
          tf::vectorTFToEigen(imu_handler_obs->getFilteredOmegaCog(), omega_body);

          // Formation CoG velocity: v_f = v_leader + omega_w × r_offset_w
          Eigen::Vector3d omega_w = cog_rot_eigen * omega_body;
          Eigen::Vector3d vel_formation_w = vel_leader_w + omega_w.cross(offset_w);

          // Realized wrench from allocation (body frame)
          Eigen::VectorXd realized_wrench = unified_controller_->getRealizedWrenchBody();

          // Observer dt: use the same du as PID (interval between controlCore calls).
          // NOTE: do NOT use (ros::Time::now() - control_timestamp_) here because
          // control_timestamp_ was already updated earlier in this same frame,
          // giving dt ≈ 0 which triggers the sanity check and skips the update.
          formation_observer_->update(
              unified_controller_->getFormationMass(),
              unified_controller_->getFormationInertia(),
              cog_rot_eigen,
              vel_formation_w,
              omega_body,
              realized_wrench,
              du);
        }
      }

      if (unified_transition_count_ >= 0)
        unified_transition_count_++;

      ROS_INFO_THROTTLE(1.0, "[UnifiedCtrl LEADER] wrench_acc=(%.3f,%.3f,%.3f,%.4f,%.4f,%.4f) ok=%d yaw_w=%.2f(raw=%.4f)",
                        target_wrench_acc(0), target_wrench_acc(1), target_wrench_acc(2),
                        target_wrench_acc(3), target_wrench_acc(4), target_wrench_acc(5), ok,
                        yaw_alloc_weight_, yaw_raw);

      // Formation observer diagnostic (Phase U2, debug-only)
      if (formation_observer_ && formation_observer_->isActive() && formation_observer_->isInitialized()) {
        const Eigen::Vector3d fext_w = formation_observer_->getEstExternalForceWorld();  // bias-subtracted
        Eigen::Vector3d fext_b = formation_observer_->getEstExternalForceBody();         // bias-subtracted
        ROS_INFO_THROTTLE(2.0, "[FormObs] corrected_w=(%.3f,%.3f,%.3f) corrected_b=(%.3f,%.3f,%.3f) |f|=%.3f bias_cal=%s",
                          fext_w.x(), fext_w.y(), fext_w.z(),
                          fext_b.x(), fext_b.y(), fext_b.z(),
                          fext_w.norm(),
                          formation_observer_->isBiasCalibrated() ? "YES" : "NO");
      }

      // Steady-state diagnostics: attitude error, position error, I-terms, saturation
      ROS_INFO_THROTTLE(5.0, "[UnifiedCtrl DIAG] roll_err=%.4f pitch_err=%.4f yaw_err=%.4f "
                        "z_err=%.4f | I: roll=%.4f pitch=%.4f z=%.4f | sat=%s",
                        target_rpy_.x() - rpy_.x(),
                        target_rpy_.y() - rpy_.y(),
                        angles::shortest_angular_distance(rpy_.z(), target_rpy_.z()),
                        target_pos_.z() - pos_.z(),
                        pid_controllers_.at(ROLL).getITerm(),
                        pid_controllers_.at(PITCH).getITerm(),
                        pid_controllers_.at(Z).getITerm(),
                        unified_controller_->isAllocationSaturated() ? "YES" : "no");

      // Adaptive seed tracking: when in quasi-steady-state, low-pass update
      // last_unified_*_i_ss_ so the NEXT mode switch gets a better seed.
      // Conditions: boost/freeze finished, errors small, allocation unsaturated.
      if (z_ki_boost_count_ == 0 && z_integral_freeze_count_ == 0 &&
          rp_ki_boost_count_ == 0 && rp_integral_freeze_count_ == 0) {
        double form_pos_z = pos_.z() + (estimator_->getOrientation(Frame::COG, estimate_mode_) *
                            tf::Vector3(unified_controller_->getFormationCogOffset().x(),
                                        unified_controller_->getFormationCogOffset().y(),
                                        unified_controller_->getFormationCogOffset().z())).z();
        double err_z_abs = std::abs(target_pos_.z() - form_pos_z);
        double vel_z_abs = std::abs(vel_.z());
        double pitch_err_abs = std::abs(target_rpy_.y() - rpy_.y());
        double pitch_rate_abs = std::abs(omega_.y());

        // Z seed: moderate conditions (position + velocity)
        if (err_z_abs < 0.05 && vel_z_abs < 0.05) {
          double current_z_i = pid_controllers_.at(Z).getITerm();
          last_unified_z_i_ss_ = (1.0 - Z_SEED_LPF_ALPHA) * last_unified_z_i_ss_
                                + Z_SEED_LPF_ALPHA * current_z_i;
          has_unified_z_i_ss_ = true;
        }

        // Pitch seed: stricter conditions (attitude + rate + position stability)
        // NOTE: use getErrI() (err_i domain), NOT getITerm() (= err_i × Ki).
        // The seed is injected via setErrI(), so must be in err_i domain.
        if (pitch_err_abs < 0.03 && pitch_rate_abs < 0.05 && vel_z_abs < 0.05) {
          double current_pitch_err_i = pid_controllers_.at(PITCH).getErrI();
          last_unified_pitch_i_ss_ = (1.0 - PITCH_SEED_LPF_ALPHA) * last_unified_pitch_i_ss_
                                    + PITCH_SEED_LPF_ALPHA * current_pitch_err_i;
          has_unified_pitch_i_ss_ = true;
        }
      }

      // --- Publish assemble debug info ---
      publishAssembleDebug(formation_pos, formation_vel, target_formation_pos, ok);

      pre_module_state_ = module_state;
      return;  // Skip individual control path
    }

    // ======== Unified Control Mode: FOLLOWER ========
    // FOLLOWER receives thrust + gimbal commands from LEADER via ROS topics,
    // then forwards them to its own spinal. No local PID.
    if (unified_control_mode_ && module_state == FOLLOWER && module_state != SEPARATED) {
      if (!prev_unified_control_mode_) {
        // Set spinal to cascade mode on this FOLLOWER's own spinal only.
        // LEADER handles sending to all modules; FOLLOWER only needs its own.
        sendFollowerCascadeSetup();
        spinal_gains_zeroed_ = true;
        unified_transition_count_ = 0;
        beetle_navigator_->setUnifiedControlMode(true);  // P3: skip CoG→CoM conversion
        ROS_WARN("[UnifiedCtrl] FOLLOWER id=%d entering unified mode, sent cascade gains to own spinal, t=%.4f",
                 beetle_navigator_->getMyID(), ros::Time::now().toSec());
      }

      bool have_valid_cmd = false;
      if (unified_cmd_received_) {
        double age = (ros::Time::now() - unified_cmd_stamp_).toSec();
        if (age < 0.5) have_valid_cmd = true;
      }

      if (have_valid_cmd) {
        // Forward vectoring force commands to own spinal
        follower_thrust_pub_.publish(unified_thrust_cmd_);
        // Gimbal: when gimbal_calc_in_fc, spinal computes gimbal angles internally
        // from the vectoring forces — no separate gimbal command needed.
        follower_unified_active_ = true;
        follower_cmd_timeout_count_ = 0;  // reset timeout counter

        // FOLLOWER Ready Sync (P2.1): publish "I'm ready" once on first valid forward.
        // This tells LEADER that this FOLLOWER has cascade gains set + is actively forwarding.
        if (!follower_ready_sent_) {
          // Lazy-create publisher to LEADER's namespace
          int leader_id = beetle_navigator_->getLeaderID();
          std::string leader_ns = std::string("/") + beetle_navigator_->getMyName()
                                  + std::to_string(leader_id);
          follower_ready_pub_ = nh_.advertise<std_msgs::Int32>(
              leader_ns + "/unified_control/follower_ready", 1, true);  // latched
          std_msgs::Int32 ready_msg;
          ready_msg.data = beetle_navigator_->getMyID();
          follower_ready_pub_.publish(ready_msg);
          follower_ready_sent_ = true;
          ROS_WARN("[UnifiedCtrl] FOLLOWER id=%d published READY to %s",
                   beetle_navigator_->getMyID(),
                   (leader_ns + "/unified_control/follower_ready").c_str());
        }

        ROS_INFO_THROTTLE(1.0, "[UnifiedCtrl FOLLOWER] id=%d forwarding: thrust_sz=%zu",
                          beetle_navigator_->getMyID(),
                          unified_thrust_cmd_.base_thrust.size());

        pre_module_state_ = module_state;
        prev_unified_control_mode_ = true;
        return;
      }

      // ======== T4.1: FOLLOWER Heartbeat Timeout & Fallback ========
      // Command is stale (age > 0.5s) or never received.
      // Strategy depends on whether we were ever actively forwarding:
      if (follower_unified_active_) {
        // We WERE forwarding — leader command has gone stale.
        // Increment timeout counter for graceful degradation.
        follower_cmd_timeout_count_++;

        if (follower_cmd_timeout_count_ <= FOLLOWER_HOLD_LAST_FRAMES) {
          // Phase 1: Hold last command (hold-last-sample).
          // Better than nothing for short glitches (< 0.5s = 20 frames @40Hz).
          follower_thrust_pub_.publish(unified_thrust_cmd_);
          ROS_WARN_THROTTLE(0.5, "[UnifiedCtrl FOLLOWER] id=%d HOLD-LAST: stale cmd, "
                            "holding for %d/%d frames",
                            beetle_navigator_->getMyID(),
                            follower_cmd_timeout_count_, FOLLOWER_HOLD_LAST_FRAMES);
          pre_module_state_ = module_state;
          prev_unified_control_mode_ = true;
          return;
        }

        // Phase 2: Full fallback — restore independent hover.
        // This is the "circuit breaker": leader is truly gone.
        ROS_ERROR("[UnifiedCtrl FOLLOWER] id=%d FALLBACK: leader cmd timeout (%d frames), "
                  "restoring independent hover!",
                  beetle_navigator_->getMyID(), follower_cmd_timeout_count_);

        resetToIndependentHover();
        spinal_gains_zeroed_ = false;

        // Clear unified state
        follower_unified_active_ = false;
        unified_cmd_received_ = false;
        follower_cmd_timeout_count_ = 0;
        follower_ready_sent_ = false;
        prev_unified_control_mode_ = false;
        unified_controller_->resetCascadeAllocSent();
        unified_controller_->resetTargetAngleLpf();
        beetle_navigator_->setUnifiedControlMode(false);

        // Fall through to independent control below
      } else {
        // Never received any unified command — still waiting for LEADER's first publish.
        // MUST return here to avoid falling through to T4.4 which would clear
        // unified_control_mode rosparam and force this FOLLOWER back to independent.
        ROS_WARN_THROTTLE(0.5, "[UnifiedCtrl FOLLOWER] id=%d, no valid unified cmd yet — waiting (age=%.3f)",
                 beetle_navigator_->getMyID(),
                 unified_cmd_received_ ? (ros::Time::now() - unified_cmd_stamp_).toSec() : -1.0);
        pre_module_state_ = module_state;
        prev_unified_control_mode_ = true;
        return;
      }
    }

    // ======== Unified → Leader-Follower Transition (T4.4) ========
    // This exit path runs for BOTH LEADER and FOLLOWER when unified_control_mode_
    // becomes false. We must restore ALL state to avoid transient jumps that crash.
    //
    // Skip when SEPARATED: unified control was never active, so there is nothing
    // to restore. This allows setting unified_control_mode rosparam on the ground
    // (before takeoff) without T4.4 immediately clearing it.
    //
    // Critical items:
    //   1. Restore spinal attitude PID gains (setAttitudeGains restores independent-mode gains)
    //   2. Reset target position to CURRENT position (avoid P-term spike)
    //   3. Seed Z I-term with gravity (avoid altitude drop)
    //   4. Clear RP/XY I-terms (unified I-term values are meaningless for independent mode)
    //   5. Sync target_pos_candidate_ (used by CoG→CoM conversion in leader-follower mode)
    // Skip T4.4 cleanup when SEPARATED: unified control was never active,
    // nothing to restore. Preserves rosparam for ground-set unified mode.
    // Let control flow continue to GimbalrotorController::controlCore() below.
    if (module_state != SEPARATED) {
      if (prev_unified_control_mode_) {
        ROS_WARN("[UnifiedCtrl] id=%d exiting unified mode → restoring independent hover state",
                 beetle_navigator_->getMyID());
        resetToIndependentHover();
      }

      prev_unified_control_mode_ = false;
      follower_unified_active_ = false;
      follower_ready_sent_ = false;
      spinal_gains_zeroed_ = false;
      gains_switched_ = false;
      unified_controller_->resetCascadeAllocSent();
      unified_controller_->resetTargetAngleLpf();
      unified_controller_->resetFollowerReady();
      beetle_navigator_->setUnifiedControlMode(false);

      {
        ros::NodeHandle control_nh(nh_, "controller");
        control_nh.setParam("unified_control_mode", false);
      }
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
          // Transition: set spinal damping-only immediately, then freeze on last hover output.
          // Do NOT call GimbalrotorController::update() — that would run its own
          // attitude PID and publish competing commands on four_axes/command.
          if (!prev_unified_control_mode_) {
            sendFollowerCascadeSetup();
            spinal_gains_zeroed_ = true;
            unified_transition_count_ = 0;
            beetle_navigator_->setUnifiedControlMode(true);  // P3: skip CoG→CoM conversion
            ROS_WARN("[UnifiedCtrl] FOLLOWER id=%d freeze: sent cascade gains to own spinal, awaiting unified cmd, t=%.4f",
                     beetle_navigator_->getMyID(), ros::Time::now().toSec());
          }
          prev_unified_control_mode_ = true;

          // Re-send cached independent hover commands to keep motors running.
          // Keep the cached angles (roll, pitch, yaw_term) — in cascade mode
          // spinal uses them for P+D attitude stabilisation.
          if (has_cached_independent_cmd_) {
            follower_thrust_pub_.publish(last_independent_thrust_cmd_);
            if (!gimbal_calc_in_fc_)
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
    // Must match the format of sendFourAxisCommand():
    //   gimbal_calc_in_fc=true  → base_thrust = target_base_thrust_ (size=8, vectoring forces)
    //                             angles = [target_roll_, target_pitch_, candidate_yaw_term_]
    //   gimbal_calc_in_fc=false → base_thrust = target_full_thrust_ (size=4, scalar thrusts)
    //                             angles = [target_roll_, target_pitch_, 0]
    if (result) {
      if (gimbal_calc_in_fc_) {
        last_independent_thrust_cmd_.base_thrust = target_base_thrust_;
        last_independent_thrust_cmd_.angles[2] = candidate_yaw_term_;
      } else {
        last_independent_thrust_cmd_.base_thrust = target_full_thrust_;
        last_independent_thrust_cmd_.angles[2] = 0;
      }
      last_independent_thrust_cmd_.angles[0] = target_roll_;
      last_independent_thrust_cmd_.angles[1] = target_pitch_;

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
    follower_ready_sent_ = false;
  }

  void BeetleController::unifiedThrustCallback(const spinal::FourAxisCommand& msg)
  {
    unified_thrust_cmd_ = msg;
    unified_cmd_received_ = true;
    unified_cmd_stamp_ = ros::Time::now();
  }

  void BeetleController::sendCascadeSetup()
  {
    // LEADER-only: send torque allocation matrix inverse and cascade P/D gains
    // to ALL assembled modules' spinals. This configures each spinal for 1000Hz
    // P+D attitude tracking using thrustGainMapping().
    //
    // NOTE: At mode-switch time, integrated_map_inv_rot_ may not be computed yet
    // (computeUnifiedAllocation() hasn't run). In that case, sendTorqueAllocationMatrixInv()
    // will silently return without sending. The one-shot logic inside
    // computeUnifiedAllocation() will resend matrix + gains on the first successful
    // computation. See cascade_alloc_sent_ flag.
    //
    // Also publish gimbal_dof=1 to own spinal (LEADER's).
    // FOLLOWERs' gimbal_dof is set by publishCommands() each frame,
    // but publishCommands() skips LEADER, so we set it here at mode switch.

    // Cache gains for deferred one-shot resend (must be done BEFORE the attempt)
    unified_controller_->cacheCascadeGains(
        cascade_roll_p_, cascade_roll_d_,
        cascade_pitch_p_, cascade_pitch_d_,
        cascade_yaw_d_);

    // Attempt to send now. If matrix is not yet computed, skip gains too —
    // sending cascade gains with the old independent-mode allocation matrix
    // causes thrustGainMapping() to produce wrong per-motor gains (P5 fix).
    // The one-shot in computeUnifiedAllocation() will send matrix + gains
    // together once the allocation is first successfully computed.
    unified_controller_->updateFormationGeometry();
    bool matrix_sent = unified_controller_->sendTorqueAllocationMatrixInv();
    if (matrix_sent) {
      unified_controller_->sendCascadeGains(
          cascade_roll_p_, cascade_roll_d_,
          cascade_pitch_p_, cascade_pitch_d_,
          cascade_yaw_d_);
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
             cascade_roll_p_, cascade_roll_d_, cascade_pitch_p_, cascade_pitch_d_, cascade_yaw_d_,
             beetle_navigator_->getAssemblyIds().size());
  }

  void BeetleController::sendFollowerCascadeSetup()
  {
    // FOLLOWER-only: send cascade P/D gains and gimbal_dof to THIS module's
    // own spinal only (via base-class publishers).
    //
    // FOLLOWER does NOT:
    //   - Send to other modules' spinals (LEADER is responsible for all-module setup)
    //   - Send alloc_inv (LEADER's one-shot handles this after allocation computation)
    //   - Cache gains for one-shot (FOLLOWER doesn't compute allocation)
    //
    // The gains sent here are temporary insurance — LEADER's sendCascadeSetup()
    // will also send to this module. But if LEADER's message arrives late,
    // the FOLLOWER's own spinal at least has cascade-mode gains to prevent
    // it from running with stale independent-mode per-motor PID.

    // Send cascade gains to own spinal via base-class publisher (rpy_gain_pub_)
    {
      spinal::RollPitchYawTerms rpy_gain_msg;
      rpy_gain_msg.motors.resize(1);  // torque-level path
      rpy_gain_msg.motors[0].roll_p  = static_cast<int16_t>(cascade_roll_p_ * 1000);
      rpy_gain_msg.motors[0].roll_i  = 0;  // I-term handled by PC
      rpy_gain_msg.motors[0].roll_d  = static_cast<int16_t>(cascade_roll_d_ * 1000);
      rpy_gain_msg.motors[0].pitch_p = static_cast<int16_t>(cascade_pitch_p_ * 1000);
      rpy_gain_msg.motors[0].pitch_i = 0;  // I-term handled by PC
      rpy_gain_msg.motors[0].pitch_d = static_cast<int16_t>(cascade_pitch_d_ * 1000);
      rpy_gain_msg.motors[0].yaw_d   = static_cast<int16_t>(cascade_yaw_d_ * 1000);
      rpy_gain_pub_.publish(rpy_gain_msg);
    }

    // Set gimbal_dof=1 on own spinal
    {
      std_msgs::UInt8 gimbal_dof_msg;
      gimbal_dof_msg.data = 1;
      gimbal_dof_pub_.publish(gimbal_dof_msg);
    }

    ROS_INFO("[UnifiedCtrl] Cascade setup (FOLLOWER id=%d): sent cascade gains"
             "(P_r=%.1f D_r=%.1f P_p=%.1f D_p=%.1f D_y=%.1f) + gimbal_dof=1 to own spinal only",
             beetle_navigator_->getMyID(),
             cascade_roll_p_, cascade_roll_d_, cascade_pitch_p_, cascade_pitch_d_, cascade_yaw_d_);
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
    saved_roll_gains_ = {roll_pid.getPGain(), roll_pid.getIGain(), roll_pid.getDGain(),
                         roll_pid.getLimitSum(), roll_pid.getLimitP(), roll_pid.getLimitI(), roll_pid.getLimitD()};
    saved_pitch_gains_ = {pitch_pid.getPGain(), pitch_pid.getIGain(), pitch_pid.getDGain(),
                          pitch_pid.getLimitSum(), pitch_pid.getLimitP(), pitch_pid.getLimitI(), pitch_pid.getLimitD()};
    saved_xy_gains_ = {x_pid.getPGain(), x_pid.getIGain(), x_pid.getDGain(),
                       x_pid.getLimitSum(), x_pid.getLimitP(), x_pid.getLimitI(), x_pid.getLimitD()};
    saved_z_gains_ = {z_pid.getPGain(), z_pid.getIGain(), z_pid.getDGain(),
                      z_pid.getLimitSum(), z_pid.getLimitP(), z_pid.getLimitI(), z_pid.getLimitD()};

    // Apply unified (formation) gains.
    // CASCADE MODE: wrench_acc only uses getITerm() for roll/pitch — P and D
    // from the PC-side PID are computed but never included in the wrench.
    // Set P=0, D=0 to avoid wasted computation and make the cascade semantics
    // explicit. Only Ki and limit_i matter at the PC outer loop.
    roll_pid.setGains(0.0, unified_roll_gains_.i, 0.0);
    roll_pid.setLimitSum(unified_roll_gains_.limit_sum);
    roll_pid.setLimitP(0.0);
    roll_pid.setLimitI(unified_roll_gains_.limit_i);
    roll_pid.setLimitD(0.0);

    pitch_pid.setGains(0.0, unified_pitch_gains_.i, 0.0);
    pitch_pid.setLimitSum(unified_pitch_gains_.limit_sum);
    pitch_pid.setLimitP(0.0);
    pitch_pid.setLimitI(unified_pitch_gains_.limit_i);
    pitch_pid.setLimitD(0.0);

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

    // Apply unified Z gains
    z_pid.setGains(unified_z_gains_.p, unified_z_gains_.i, unified_z_gains_.d);
    z_pid.setLimitSum(unified_z_gains_.limit_sum);
    z_pid.setLimitP(unified_z_gains_.limit_p);
    z_pid.setLimitI(unified_z_gains_.limit_i);
    z_pid.setLimitD(unified_z_gains_.limit_d);

    gains_switched_ = true;
    ROS_WARN("[UnifiedCtrl] Applied unified gains: roll/pitch I=%.1f/%.1f, "
             "xy P=%.1f I=%.1f D=%.1f, z P=%.1f I=%.1f D=%.1f",
             unified_roll_gains_.i, unified_pitch_gains_.i,
             unified_xy_gains_.p, unified_xy_gains_.i, unified_xy_gains_.d,
             unified_z_gains_.p, unified_z_gains_.i, unified_z_gains_.d);
  }

  void BeetleController::restoreIndependentGains()
  {
    if (!gains_switched_) return;  // nothing to restore

    auto& roll_pid = pid_controllers_.at(ROLL);
    auto& pitch_pid = pid_controllers_.at(PITCH);
    auto& x_pid = pid_controllers_.at(X);
    auto& y_pid = pid_controllers_.at(Y);
    auto& z_pid = pid_controllers_.at(Z);

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

    // Restore independent Z gains
    z_pid.setGains(saved_z_gains_.p, saved_z_gains_.i, saved_z_gains_.d);
    z_pid.setLimitSum(saved_z_gains_.limit_sum);
    z_pid.setLimitP(saved_z_gains_.limit_p);
    z_pid.setLimitI(saved_z_gains_.limit_i);
    z_pid.setLimitD(saved_z_gains_.limit_d);

    gains_switched_ = false;
    ROS_WARN("[UnifiedCtrl] Restored independent gains: pitch P=%.1f D=%.1f, xy P=%.1f D=%.1f, z P=%.1f D=%.1f",
             saved_pitch_gains_.p, saved_pitch_gains_.d,
             saved_xy_gains_.p, saved_xy_gains_.d,
             saved_z_gains_.p, saved_z_gains_.d);
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
      }
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

    // Cascade control gains: torque-level P/D sent to each spinal for 1000Hz inner loop.
    // Default values are conservative starting points.
    getParam<double>(control_nh, "cascade_roll_p", cascade_roll_p_, 8.0);
    getParam<double>(control_nh, "cascade_roll_d", cascade_roll_d_, 5.0);
    getParam<double>(control_nh, "cascade_pitch_p", cascade_pitch_p_, 8.0);
    getParam<double>(control_nh, "cascade_pitch_d", cascade_pitch_d_, 5.0);
    getParam<double>(control_nh, "cascade_yaw_d", cascade_yaw_d_, 2.5);

    // Yaw allocation weight: scale wrench_acc(5) before pseudoinverse allocation.
    // 1.0 = original behavior, <1.0 = reduce yaw noise coupling into base_thrust.
    getParam<double>(control_nh, "yaw_alloc_weight", yaw_alloc_weight_, 1.0);

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

    // Pitch I-term seed default for unified mode switch (same philosophy as Z seed)
    getParam<double>(control_nh, "pitch_i_seed_default", pitch_i_seed_default_, -0.55);
    last_unified_pitch_i_ss_ = pitch_i_seed_default_;

    // P2: Per-N pitch seed bucketing
    {
      ros::NodeHandle seed_nh(control_nh, "pitch_i_seed_by_n");
      for (int n = 2; n <= 6; n++) {
        double val;
        if (seed_nh.getParam("n" + std::to_string(n), val)) {
          pitch_i_seed_by_n_[n] = val;
          ROS_INFO("[SeedBucket] pitch_i_seed_by_n[%d] = %.4f", n, val);
        }
      }
    }

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

    // Roll/Pitch I-term keep ratio for unified mode switch
    // Fraction of independent-mode I-term to preserve at switch (0=clear, 1=full keep)
    getParam<double>(control_nh, "rp_i_keep_ratio", rp_i_keep_ratio_, 0.5);

    // Load unified-mode PID gains for roll/pitch (formation pendulum compensation)
    // Only i_gain, limit_sum, limit_i are used — PC outer loop runs I-only;
    // P and D are handled by spinal cascade (cascade_roll_p/d, cascade_pitch_p/d).
    ros::NodeHandle u_roll_nh(control_nh, "unified_roll");
    getParam<double>(u_roll_nh, "i_gain", unified_roll_gains_.i, 1.0);
    getParam<double>(u_roll_nh, "limit_sum", unified_roll_gains_.limit_sum, 50.0);
    getParam<double>(u_roll_nh, "limit_i", unified_roll_gains_.limit_i, 10.0);

    ros::NodeHandle u_pitch_nh(control_nh, "unified_pitch");
    getParam<double>(u_pitch_nh, "i_gain", unified_pitch_gains_.i, 1.0);
    getParam<double>(u_pitch_nh, "limit_sum", unified_pitch_gains_.limit_sum, 50.0);
    getParam<double>(u_pitch_nh, "limit_i", unified_pitch_gains_.limit_i, 10.0);

    // Load unified-mode PID gains for XY and Z (used when formation is assembled)
    ros::NodeHandle u_xy_nh(control_nh, "unified_xy");
    getParam<double>(u_xy_nh, "p_gain", unified_xy_gains_.p, 2.0);
    getParam<double>(u_xy_nh, "i_gain", unified_xy_gains_.i, 0.2);
    getParam<double>(u_xy_nh, "d_gain", unified_xy_gains_.d, 2.5);
    getParam<double>(u_xy_nh, "limit_sum", unified_xy_gains_.limit_sum, 8.0);
    getParam<double>(u_xy_nh, "limit_p", unified_xy_gains_.limit_p, 12.0);
    getParam<double>(u_xy_nh, "limit_i", unified_xy_gains_.limit_i, 8.0);
    getParam<double>(u_xy_nh, "limit_d", unified_xy_gains_.limit_d, 12.0);

    ros::NodeHandle u_z_nh(control_nh, "unified_z");
    getParam<double>(u_z_nh, "p_gain", unified_z_gains_.p, 5.0);
    getParam<double>(u_z_nh, "i_gain", unified_z_gains_.i, 1.0);
    getParam<double>(u_z_nh, "d_gain", unified_z_gains_.d, 2.0);
    getParam<double>(u_z_nh, "limit_sum", unified_z_gains_.limit_sum, 25.0);
    getParam<double>(u_z_nh, "limit_p", unified_z_gains_.limit_p, 25.0);
    getParam<double>(u_z_nh, "limit_i", unified_z_gains_.limit_i, 20.0);
    getParam<double>(u_z_nh, "limit_d", unified_z_gains_.limit_d, 25.0);

    // UO-4: formation observer feedforward compensation
    ros::NodeHandle obs_comp_nh(control_nh, "formation_observer_comp");
    getParam<bool>(obs_comp_nh,   "enable",   formation_obs_comp_enable_,  false);
    getParam<double>(obs_comp_nh, "z_gain",   formation_obs_comp_z_gain_,  1.0);
    getParam<double>(obs_comp_nh, "xy_gain",     formation_obs_comp_xy_gain_,    1.0);
    getParam<double>(obs_comp_nh, "torque_gain", formation_obs_comp_torque_gain_, 1.0);

  }

  void BeetleController::externalWrenchEstimate()
  {
    // In unified mode the formation-level observer (FormationMomentumObserver) handles
    // estimation. Single-module observer has no formation-level semantics here.
    if (unified_control_mode_) return;

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

} //namespace aerial_robot_controller

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::BeetleController, aerial_robot_control::ControlBase);
