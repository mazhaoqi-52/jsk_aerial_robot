// -*- mode: c++ -*-
// Formation-level momentum observer — implementation.
//
// Version 1: 3D external force estimation only (debug-only, no control feedback).
//
// Observer equation (3D force channel):
//
//   Linear momentum:  p_lin = M * v_w   (world frame)
//
//   Realized force → world:  f_realized_w = R * f_realized_body_xyz
//
//   Non-linear term (force):  N_f = M * g_w  (gravity in world = [0, 0, +9.8])
//
//   Integration:
//     integrate_term_f += (f_realized_w - N_f + f_ext_hat) * dt
//
//   Observer output (raw):
//     f_ext_hat_raw = K_f * (p_lin - p_lin_0 - integrate_term_f)
//
// Diagnostic equation check:
//   finite_diff_ext = d(M*v_w)/dt - (f_realized_w - M*g_w)
// This should roughly match f_ext_hat when the allocation/thrust model is
// consistent. If it does not, the observer output is a model residual rather
// than a trustworthy physical external force.

#include <beetle/control/formation_momentum_observer.h>
#include <algorithm>

namespace aerial_robot_control
{

FormationMomentumObserver::FormationMomentumObserver()
  : initialized_(false),
    active_(false),
    init_linear_momentum_(Eigen::Vector3d::Zero()),
    prev_linear_momentum_(Eigen::Vector3d::Zero()),
    integrate_term_force_(Eigen::Vector3d::Zero()),
    est_ext_force_w_(Eigen::Vector3d::Zero()),
    prev_linear_momentum_valid_(false),
    init_angular_momentum_(Eigen::Vector3d::Zero()),
    integrate_term_torque_(Eigen::Vector3d::Zero()),
    est_ext_torque_body_(Eigen::Vector3d::Zero()),
    ff_armed_(false),
    ff_armed_time_(-1.0),
    ff_ramp_seconds_(5.0),
    last_cog_rot_(Eigen::Matrix3d::Identity()),
    force_observer_gain_(3.0),
    torque_observer_gain_(2.5),
    est_force_lpf_cutoff_freq_(0.05),
    est_force_lpf_initialized_(false),
    est_ext_force_w_filt_(Eigen::Vector3d::Zero()),
    est_torque_lpf_cutoff_freq_(0.05),
    est_torque_lpf_initialized_(false),
    est_ext_torque_body_filt_(Eigen::Vector3d::Zero()),
    enable_force_observer_(true),
    enable_torque_observer_(false)
{
}

void FormationMomentumObserver::initialize(ros::NodeHandle nh)
{
  nh_ = nh;
  loadParams();

  // Debug-only publishers under global /assemble/formation_observer/ namespace.
  // Formation observer is a system-level concept (not per-module), so we publish
  // to a global namespace without the /beetleX/ prefix.
  ros::NodeHandle obs_nh("/assemble/formation_observer");
  est_ext_torque_body_pub_   = obs_nh.advertise<geometry_msgs::Vector3Stamped>("est_ext_torque_body", 1);
  est_ext_wrench_pub_        = obs_nh.advertise<geometry_msgs::WrenchStamped>("est_ext_wrench", 1);
  observer_residual_pub_     = obs_nh.advertise<geometry_msgs::Vector3Stamped>("residual_force", 1);
  observer_residual_torque_pub_ = obs_nh.advertise<geometry_msgs::Vector3Stamped>("residual_torque", 1);
  realized_wrench_debug_pub_ = obs_nh.advertise<geometry_msgs::WrenchStamped>("realized_wrench_input", 1);

  ROS_INFO("[FormationObserver] Initialized: force_gain=%.2f, torque_gain=%.2f, "
           "force_en=%d, torque_en=%d",
           force_observer_gain_, torque_observer_gain_,
           enable_force_observer_, enable_torque_observer_);
}

void FormationMomentumObserver::loadParams()
{
  ros::NodeHandle obs_nh(nh_, "controller/formation_observer");
  obs_nh.param<double>("force_observer_gain", force_observer_gain_, 3.0);
  obs_nh.param<double>("torque_observer_gain", torque_observer_gain_, 2.5);
  obs_nh.param<bool>("enable_force_observer", enable_force_observer_, true);
  obs_nh.param<bool>("enable_torque_observer", enable_torque_observer_, false);
  obs_nh.param<double>("est_force_lpf_cutoff_freq",  est_force_lpf_cutoff_freq_,  0.05);
  obs_nh.param<double>("est_torque_lpf_cutoff_freq", est_torque_lpf_cutoff_freq_, 0.05);
  obs_nh.param<double>("ff_ramp_seconds",            ff_ramp_seconds_,            5.0);
}

void FormationMomentumObserver::reset()
{
  initialized_ = false;
  init_linear_momentum_ = Eigen::Vector3d::Zero();
  prev_linear_momentum_ = Eigen::Vector3d::Zero();
  integrate_term_force_ = Eigen::Vector3d::Zero();
  est_ext_force_w_ = Eigen::Vector3d::Zero();
  prev_linear_momentum_valid_ = false;
  est_ext_force_w_filt_ = Eigen::Vector3d::Zero();
  est_force_lpf_initialized_ = false;

  init_angular_momentum_ = Eigen::Vector3d::Zero();
  integrate_term_torque_ = Eigen::Vector3d::Zero();
  est_ext_torque_body_ = Eigen::Vector3d::Zero();
  est_ext_torque_body_filt_ = Eigen::Vector3d::Zero();
  est_torque_lpf_initialized_ = false;

  last_cog_rot_ = Eigen::Matrix3d::Identity();

  // No bias state to reset. Just clear the future FF arming gate.
  ff_armed_ = false;
  ff_armed_time_ = -1.0;

  ROS_INFO("[FormationObserver] State reset (no bias subtraction)");
}

void FormationMomentumObserver::setFfArmed(bool armed)
{
  if (ff_armed_ == armed) return;

  ff_armed_ = armed;
  if (armed)
  {
    ff_armed_time_ = ros::Time::now().toSec();
    ROS_INFO("[FormationObserver] FF gate ARMED (ramp %.1fs starts now)", ff_ramp_seconds_);
  }
  else
  {
    ff_armed_time_ = -1.0;
    ROS_INFO("[FormationObserver] FF gate DISARMED");
  }
}

void FormationMomentumObserver::update(
    double formation_mass,
    const Eigen::Matrix3d& formation_inertia,
    const Eigen::Matrix3d& cog_rot,
    const Eigen::Vector3d& vel_w,
    const Eigen::Vector3d& omega_body,
    const Eigen::VectorXd& realized_wrench_body,
    double dt)
{
  if (!active_) return;
  if (dt <= 0) return;             // sanity: skip backward / zero dt
  if (dt > 0.1) dt = 0.1;          // clamp huge gap (callback stall) instead of dropping the frame
  if (formation_mass < 0.01) return;  // sanity: skip zero mass

  last_cog_rot_ = cog_rot;

  // ========== 3D Force Observer (V1) ==========
  Eigen::Vector3d residual = Eigen::Vector3d::Zero();

  if (enable_force_observer_)
  {
    // 1. Current linear momentum: p = M * v_w  (world frame)
    Eigen::Vector3d p_lin = formation_mass * vel_w;

    // 2. First-time initialization: record initial momentum
    if (!initialized_)
    {
      init_linear_momentum_ = p_lin;
      initialized_ = true;
      ROS_INFO("[FormationObserver] First update: p_lin_0 = (%.4f, %.4f, %.4f), "
               "mass = %.3f (no bias subtraction)",
               init_linear_momentum_.x(), init_linear_momentum_.y(),
               init_linear_momentum_.z(), formation_mass);
    }

    // 3. Realized force: rotate body-frame force to world frame
    Eigen::Vector3d realized_force_body = Eigen::Vector3d::Zero();
    if (realized_wrench_body.size() >= 3)
    {
      realized_force_body = realized_wrench_body.head(3);
    }
    Eigen::Vector3d realized_force_w = cog_rot * realized_force_body;

    // 4. Gravity term (world frame)
    constexpr double G = 9.797;  // same as aerial_robot_estimation::G
    Eigen::Vector3d gravity_force_w = formation_mass * Eigen::Vector3d(0, 0, G);

    Eigen::Vector3d model_net_force_w = realized_force_w - gravity_force_w;
    Eigen::Vector3d p_dot_w = Eigen::Vector3d::Zero();
    Eigen::Vector3d finite_diff_ext_w = Eigen::Vector3d::Zero();
    const bool p_dot_ready = prev_linear_momentum_valid_ && dt > 1e-6;
    if (p_dot_ready)
    {
      p_dot_w = (p_lin - prev_linear_momentum_) / dt;
      finite_diff_ext_w = p_dot_w - model_net_force_w;
    }
    prev_linear_momentum_ = p_lin;
    prev_linear_momentum_valid_ = true;

    // 5. Integration step:
    //    integrate_term_f += (f_realized_w - N_f + f_ext_hat) * dt
    integrate_term_force_ += (model_net_force_w + est_ext_force_w_) * dt;

    // 6. Observer output (raw, internal feedback uses this directly):
    residual = p_lin - init_linear_momentum_ - integrate_term_force_;
    est_ext_force_w_ = force_observer_gain_ * residual;

    // 6b. Output LPF: smooth the raw estimate for publishing/accessors.
    //     Internal feedback (step 5) keeps using raw est_ext_force_w_ to
    //     avoid changing observer dynamics.
    if (!est_force_lpf_initialized_)
    {
      est_ext_force_w_filt_ = est_ext_force_w_;
      est_force_lpf_initialized_ = true;
    }
    else
    {
      // First-order LPF: alpha = τ/(τ+dt),  τ = 1/(2π·fc)
      double tau = 1.0 / (2.0 * M_PI * est_force_lpf_cutoff_freq_);
      double alpha = tau / (tau + dt);
      est_ext_force_w_filt_ = alpha * est_ext_force_w_filt_ + (1.0 - alpha) * est_ext_force_w_;
    }

    // 7. Equation diagnostic. In unloaded hover, finite_diff_ext_w should stay
    //    small; a persistent offset means the allocation/thrust model and the
    //    measured momentum are not self-consistent.
    ROS_INFO_THROTTLE(
        2.0,
        "[FormObsEq_F] f_real_w=(%.2f,%.2f,%.2f) Mg=(%.2f,%.2f,%.2f) "
        "model_net=(%.2f,%.2f,%.2f) p_dot=(%.2f,%.2f,%.2f) "
        "fd_ext=(%.2f,%.2f,%.2f) obs_raw=(%.2f,%.2f,%.2f) obs_filt=(%.2f,%.2f,%.2f) "
        "ready=%d dt=%.3f ff=%s",
        realized_force_w.x(), realized_force_w.y(), realized_force_w.z(),
        gravity_force_w.x(), gravity_force_w.y(), gravity_force_w.z(),
        model_net_force_w.x(), model_net_force_w.y(), model_net_force_w.z(),
        p_dot_w.x(), p_dot_w.y(), p_dot_w.z(),
        finite_diff_ext_w.x(), finite_diff_ext_w.y(), finite_diff_ext_w.z(),
        est_ext_force_w_.x(), est_ext_force_w_.y(), est_ext_force_w_.z(),
        est_ext_force_w_filt_.x(), est_ext_force_w_filt_.y(), est_ext_force_w_filt_.z(),
        p_dot_ready ? 1 : 0, dt, ff_armed_ ? "ARMED" : "DISARMED");
  }

  // ========== 3D Torque Observer (V2) ==========
  // Angular momentum observer in body frame:
  //   p_ang = I * omega_body
  //   N_torque = omega × (I * omega)   (gyroscopic coupling)
  //   integrate_torque += (tau_realized_body - N_torque + tau_ext_hat) * dt
  //   tau_ext_hat = K_t * (p_ang - p_ang_0 - integrate_torque)
  //
  // Note: torque channel works entirely in body frame (no rotation needed).
  Eigen::Vector3d residual_torque = Eigen::Vector3d::Zero();

  if (enable_torque_observer_)
  {
    // 1. Current angular momentum: p_ang = I * omega (body frame)
    Eigen::Vector3d p_ang = formation_inertia * omega_body;

    // 2. First-time initialization for angular channel
    //    (shares initialized_ flag with force channel — if force is disabled,
    //     torque still initializes on first call)
    if (!initialized_)
    {
      init_angular_momentum_ = p_ang;
      if (!enable_force_observer_)
      {
        initialized_ = true;
      }
      ROS_INFO("[FormationObserver] Torque init: p_ang_0 = (%.4f, %.4f, %.4f)",
               init_angular_momentum_.x(), init_angular_momentum_.y(),
               init_angular_momentum_.z());
    }

    // 3. Realized torque in body frame (from allocation)
    Eigen::Vector3d realized_torque_body = Eigen::Vector3d::Zero();
    if (realized_wrench_body.size() >= 6)
    {
      realized_torque_body = realized_wrench_body.tail(3);
    }

    // 4. Gyroscopic term: N_torque = omega × (I * omega)
    Eigen::Vector3d gyroscopic = omega_body.cross(formation_inertia * omega_body);

    // 5. Integration step:
    //    integrate_torque += (tau_realized_body - N_torque + tau_ext_hat) * dt
    integrate_term_torque_ += (realized_torque_body - gyroscopic + est_ext_torque_body_) * dt;

    // 6. Observer output (raw):
    //    tau_ext_hat = K_t * (p_ang - p_ang_0 - integrate_torque)
    residual_torque = p_ang - init_angular_momentum_ - integrate_term_torque_;
    est_ext_torque_body_ = torque_observer_gain_ * residual_torque;

    // 6b. Output LPF (mirrors force channel; internal feedback uses raw value).
    if (!est_torque_lpf_initialized_)
    {
      est_ext_torque_body_filt_ = est_ext_torque_body_;
      est_torque_lpf_initialized_ = true;
    }
    else
    {
      double tau_t = 1.0 / (2.0 * M_PI * est_torque_lpf_cutoff_freq_);
      double alpha_t = tau_t / (tau_t + dt);
      est_ext_torque_body_filt_ = alpha_t * est_ext_torque_body_filt_ + (1.0 - alpha_t) * est_ext_torque_body_;
    }

    // 7. Torque output diagnostic. Keep this at DEBUG to avoid log spam while
    //    force-observer validation is the current focus.
    double tau_raw_filt_dev = (est_ext_torque_body_ - est_ext_torque_body_filt_).norm();
    ROS_DEBUG_THROTTLE(2.0, "[FormObs_T] raw=(%.4f,%.4f,%.4f) filt=(%.4f,%.4f,%.4f) "
                       "|raw-filt|=%.4f gyro=(%.4f,%.4f,%.4f)",
                      est_ext_torque_body_.x(), est_ext_torque_body_.y(), est_ext_torque_body_.z(),
                      est_ext_torque_body_filt_.x(), est_ext_torque_body_filt_.y(), est_ext_torque_body_filt_.z(),
                      tau_raw_filt_dev,
                      gyroscopic.x(), gyroscopic.y(), gyroscopic.z());
  }

  // ========== Publish all debug topics ==========
  publishDebug(ros::Time::now(), residual, residual_torque);

  // Publish realized wrench input for verification
  {
    geometry_msgs::WrenchStamped rw_msg;
    rw_msg.header.stamp = ros::Time::now();
    rw_msg.header.frame_id = "formation_body";
    if (realized_wrench_body.size() >= 3) {
      rw_msg.wrench.force.x = realized_wrench_body(0);
      rw_msg.wrench.force.y = realized_wrench_body(1);
      rw_msg.wrench.force.z = realized_wrench_body(2);
    }
    if (realized_wrench_body.size() >= 6) {
      rw_msg.wrench.torque.x = realized_wrench_body(3);
      rw_msg.wrench.torque.y = realized_wrench_body(4);
      rw_msg.wrench.torque.z = realized_wrench_body(5);
    }
    realized_wrench_debug_pub_.publish(rw_msg);
  }
}

void FormationMomentumObserver::publishDebug(
    const ros::Time& stamp,
    const Eigen::Vector3d& residual_force,
    const Eigen::Vector3d& residual_torque)
{
  // Observer output is the LPF-filtered estimate directly. No bias subtraction
  // is applied here.
  Eigen::Vector3d f_ext_corrected = est_ext_force_w_filt_;

  // Estimated external torque — body frame (LPF-filtered, no bias subtraction).
  Eigen::Vector3d t_ext_corrected = est_ext_torque_body_filt_;
  if (enable_torque_observer_)
  {
    geometry_msgs::Vector3Stamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "formation_body";
    msg.vector.x = t_ext_corrected.x();
    msg.vector.y = t_ext_corrected.y();
    msg.vector.z = t_ext_corrected.z();
    est_ext_torque_body_pub_.publish(msg);
  }

  // Full 6D wrench in formation_body frame (no bias subtraction).
  // Both force and torque are in body frame for consistent interpretation.
  {
    Eigen::Vector3d force_body = last_cog_rot_.transpose() * f_ext_corrected;
    geometry_msgs::WrenchStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "formation_body";
    msg.wrench.force.x = force_body.x();
    msg.wrench.force.y = force_body.y();
    msg.wrench.force.z = force_body.z();
    if (enable_torque_observer_) {
      msg.wrench.torque.x = t_ext_corrected.x();
      msg.wrench.torque.y = t_ext_corrected.y();
      msg.wrench.torque.z = t_ext_corrected.z();
    }
    est_ext_wrench_pub_.publish(msg);
  }

  // Observer residual — force (raw, before gain multiplication)
  {
    geometry_msgs::Vector3Stamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "world";
    msg.vector.x = residual_force.x();
    msg.vector.y = residual_force.y();
    msg.vector.z = residual_force.z();
    observer_residual_pub_.publish(msg);
  }

  // Observer residual — torque (only when torque observer is active)
  if (enable_torque_observer_)
  {
    geometry_msgs::Vector3Stamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "formation_body";
    msg.vector.x = residual_torque.x();
    msg.vector.y = residual_torque.y();
    msg.vector.z = residual_torque.z();
    observer_residual_torque_pub_.publish(msg);
  }
}

Eigen::VectorXd FormationMomentumObserver::getEstExternalWrench6D() const
{
  Eigen::VectorXd wrench = Eigen::VectorXd::Zero(6);
  // No bias subtraction. This is a diagnostic model residual until validated.
  wrench.head(3) = last_cog_rot_.transpose() * est_ext_force_w_filt_;
  wrench.tail(3) = est_ext_torque_body_filt_;
  return wrench;
}

double FormationMomentumObserver::getFfRampFactor() const
{
  if (!ff_armed_ || ff_armed_time_ < 0.0) return 0.0;
  if (ff_ramp_seconds_ <= 1e-3) return 1.0;
  double dt_since = ros::Time::now().toSec() - ff_armed_time_;
  if (dt_since <= 0.0) return 0.0;
  if (dt_since >= ff_ramp_seconds_) return 1.0;
  return dt_since / ff_ramp_seconds_;
}

} // namespace aerial_robot_control
