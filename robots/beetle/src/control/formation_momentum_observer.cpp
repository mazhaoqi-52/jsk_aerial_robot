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
//   Bias calibration:
//     After the observer settles (bias_settle_time seconds), average the raw
//     estimate over bias_calib_samples to get bias_force_w_.
//     Output = f_ext_hat_raw - bias_force_w_
//
//     This cancels the inherent offset from PID I-term compensating for model
//     error (mass, thrust coefficient mismatch), which the observer cannot
//     distinguish from true external force.  Analogous to the original per-module
//     observer's differential cancellation (individual - average).

#include <beetle/control/formation_momentum_observer.h>

namespace aerial_robot_control
{

FormationMomentumObserver::FormationMomentumObserver()
  : initialized_(false),
    active_(false),
    init_linear_momentum_(Eigen::Vector3d::Zero()),
    integrate_term_force_(Eigen::Vector3d::Zero()),
    est_ext_force_w_(Eigen::Vector3d::Zero()),
    init_angular_momentum_(Eigen::Vector3d::Zero()),
    integrate_term_torque_(Eigen::Vector3d::Zero()),
    est_ext_torque_body_(Eigen::Vector3d::Zero()),
    last_cog_rot_(Eigen::Matrix3d::Identity()),
    force_observer_gain_(3.0),
    torque_observer_gain_(2.5),
    enable_force_observer_(true),
    enable_torque_observer_(false),
    bias_calibrated_(false),
    bias_calibrating_(false),
    bias_sample_count_(0),
    bias_accumulator_(Eigen::Vector3d::Zero()),
    bias_force_w_(Eigen::Vector3d::Zero()),
    bias_torque_calibrated_(false),
    bias_torque_calibrating_(false),
    bias_torque_sample_count_(0),
    bias_torque_accumulator_(Eigen::Vector3d::Zero()),
    bias_torque_body_(Eigen::Vector3d::Zero()),
    bias_settle_time_(3.0),
    bias_calib_samples_(40),
    update_count_(0)
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
  est_ext_force_world_pub_   = obs_nh.advertise<geometry_msgs::Vector3Stamped>("est_ext_force_world", 1);
  est_ext_force_body_pub_    = obs_nh.advertise<geometry_msgs::Vector3Stamped>("est_ext_force_body", 1);
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
  obs_nh.param<double>("bias_settle_time", bias_settle_time_, 3.0);
  obs_nh.param<int>("bias_calib_samples", bias_calib_samples_, 40);
}

void FormationMomentumObserver::reset()
{
  initialized_ = false;
  init_linear_momentum_ = Eigen::Vector3d::Zero();
  integrate_term_force_ = Eigen::Vector3d::Zero();
  est_ext_force_w_ = Eigen::Vector3d::Zero();

  init_angular_momentum_ = Eigen::Vector3d::Zero();
  integrate_term_torque_ = Eigen::Vector3d::Zero();
  est_ext_torque_body_ = Eigen::Vector3d::Zero();

  last_cog_rot_ = Eigen::Matrix3d::Identity();

  // Reset bias calibration state
  bias_calibrated_ = false;
  bias_calibrating_ = false;
  bias_sample_count_ = 0;
  bias_accumulator_ = Eigen::Vector3d::Zero();
  bias_force_w_ = Eigen::Vector3d::Zero();

  bias_torque_calibrated_ = false;
  bias_torque_calibrating_ = false;
  bias_torque_sample_count_ = 0;
  bias_torque_accumulator_ = Eigen::Vector3d::Zero();
  bias_torque_body_ = Eigen::Vector3d::Zero();

  update_count_ = 0;

  ROS_INFO("[FormationObserver] State reset (including bias calibration)");
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
  if (dt <= 0 || dt > 0.5) return;  // sanity: skip bad dt
  if (formation_mass < 0.01) return;  // sanity: skip zero mass

  last_cog_rot_ = cog_rot;
  update_count_++;

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
               "mass = %.3f, bias_settle=%.1fs, bias_samples=%d",
               init_linear_momentum_.x(), init_linear_momentum_.y(),
               init_linear_momentum_.z(), formation_mass,
               bias_settle_time_, bias_calib_samples_);
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

    // 5. Integration step:
    //    integrate_term_f += (f_realized_w - N_f + f_ext_hat) * dt
    integrate_term_force_ += (realized_force_w - gravity_force_w + est_ext_force_w_) * dt;

    // 6. Observer output (raw, before bias subtraction):
    //    f_ext_hat = K_f * (p_lin - p_lin_0 - integrate_term_f)
    residual = p_lin - init_linear_momentum_ - integrate_term_force_;
    est_ext_force_w_ = force_observer_gain_ * residual;

    // 7. Bias auto-calibration
    if (!bias_calibrated_)
    {
      int settle_frames = static_cast<int>(bias_settle_time_ / dt);
      if (settle_frames < 10) settle_frames = 10;

      if (!bias_calibrating_ && update_count_ >= settle_frames)
      {
        bias_calibrating_ = true;
        bias_sample_count_ = 0;
        bias_accumulator_ = Eigen::Vector3d::Zero();
        ROS_INFO("[FormationObserver] Force bias calibration started after %d frames "
                 "(settle=%.1fs), collecting %d samples...",
                 update_count_, bias_settle_time_, bias_calib_samples_);
      }

      if (bias_calibrating_)
      {
        bias_accumulator_ += est_ext_force_w_;
        bias_sample_count_++;

        if (bias_sample_count_ >= bias_calib_samples_)
        {
          bias_force_w_ = bias_accumulator_ / static_cast<double>(bias_calib_samples_);
          bias_calibrated_ = true;
          bias_calibrating_ = false;
          ROS_INFO("[FormationObserver] Force bias calibrated: (%.3f, %.3f, %.3f) N",
                   bias_force_w_.x(), bias_force_w_.y(), bias_force_w_.z());
        }
      }
    }

    Eigen::Vector3d f_ext_corrected = est_ext_force_w_ - bias_force_w_;
    ROS_INFO_THROTTLE(2.0, "[FormObs_F] raw=(%.3f,%.3f,%.3f) bias=(%.3f,%.3f,%.3f) "
                      "corrected=(%.3f,%.3f,%.3f) |f|=%.3f calib=%s",
                      est_ext_force_w_.x(), est_ext_force_w_.y(), est_ext_force_w_.z(),
                      bias_force_w_.x(), bias_force_w_.y(), bias_force_w_.z(),
                      f_ext_corrected.x(), f_ext_corrected.y(), f_ext_corrected.z(),
                      f_ext_corrected.norm(),
                      bias_calibrated_ ? "YES" : (bias_calibrating_ ? "SAMPLING" : "SETTLING"));
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

    // 7. Torque bias auto-calibration (same settle/sample scheme as force)
    if (!bias_torque_calibrated_)
    {
      int settle_frames = static_cast<int>(bias_settle_time_ / dt);
      if (settle_frames < 10) settle_frames = 10;

      if (!bias_torque_calibrating_ && update_count_ >= settle_frames)
      {
        bias_torque_calibrating_ = true;
        bias_torque_sample_count_ = 0;
        bias_torque_accumulator_ = Eigen::Vector3d::Zero();
        ROS_INFO("[FormationObserver] Torque bias calibration started, collecting %d samples...",
                 bias_calib_samples_);
      }

      if (bias_torque_calibrating_)
      {
        bias_torque_accumulator_ += est_ext_torque_body_;
        bias_torque_sample_count_++;

        if (bias_torque_sample_count_ >= bias_calib_samples_)
        {
          bias_torque_body_ = bias_torque_accumulator_ / static_cast<double>(bias_calib_samples_);
          bias_torque_calibrated_ = true;
          bias_torque_calibrating_ = false;
          ROS_INFO("[FormationObserver] Torque bias calibrated: (%.4f, %.4f, %.4f) Nm",
                   bias_torque_body_.x(), bias_torque_body_.y(), bias_torque_body_.z());
        }
      }
    }

    Eigen::Vector3d tau_ext_corrected = est_ext_torque_body_ - bias_torque_body_;
    ROS_INFO_THROTTLE(2.0, "[FormObs_T] raw=(%.4f,%.4f,%.4f) bias=(%.4f,%.4f,%.4f) "
                      "corrected=(%.4f,%.4f,%.4f) gyro=(%.4f,%.4f,%.4f)",
                      est_ext_torque_body_.x(), est_ext_torque_body_.y(), est_ext_torque_body_.z(),
                      bias_torque_body_.x(), bias_torque_body_.y(), bias_torque_body_.z(),
                      tau_ext_corrected.x(), tau_ext_corrected.y(), tau_ext_corrected.z(),
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
  // Bias-subtracted estimates for all published topics
  Eigen::Vector3d f_ext_corrected = est_ext_force_w_ - bias_force_w_;

  // Estimated external force — world frame (bias-subtracted)
  {
    geometry_msgs::Vector3Stamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "world";
    msg.vector.x = f_ext_corrected.x();
    msg.vector.y = f_ext_corrected.y();
    msg.vector.z = f_ext_corrected.z();
    est_ext_force_world_pub_.publish(msg);
  }

  // Estimated external force — body frame (bias-subtracted)
  {
    Eigen::Vector3d force_body = last_cog_rot_.transpose() * f_ext_corrected;
    geometry_msgs::Vector3Stamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "formation_body";
    msg.vector.x = force_body.x();
    msg.vector.y = force_body.y();
    msg.vector.z = force_body.z();
    est_ext_force_body_pub_.publish(msg);
  }

  // Estimated external torque — body frame (bias-subtracted)
  Eigen::Vector3d t_ext_corrected = est_ext_torque_body_ - bias_torque_body_;
  {
    geometry_msgs::Vector3Stamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "formation_body";
    msg.vector.x = t_ext_corrected.x();
    msg.vector.y = t_ext_corrected.y();
    msg.vector.z = t_ext_corrected.z();
    est_ext_torque_body_pub_.publish(msg);
  }

  // Full 6D wrench (force_world + torque_body) — bias-subtracted
  {
    geometry_msgs::WrenchStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "world";
    msg.wrench.force.x = f_ext_corrected.x();
    msg.wrench.force.y = f_ext_corrected.y();
    msg.wrench.force.z = f_ext_corrected.z();
    msg.wrench.torque.x = t_ext_corrected.x();
    msg.wrench.torque.y = t_ext_corrected.y();
    msg.wrench.torque.z = t_ext_corrected.z();
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

  // Observer residual — torque (raw, before gain multiplication)
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
  wrench.head(3) = est_ext_force_w_ - bias_force_w_;  // bias-subtracted
  wrench.tail(3) = est_ext_torque_body_ - bias_torque_body_;
  return wrench;
}

} // namespace aerial_robot_control
