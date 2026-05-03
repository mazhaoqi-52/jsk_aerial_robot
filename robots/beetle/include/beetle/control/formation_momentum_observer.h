// -*- mode: c++ -*-
// Formation-level momentum observer for assembled beetle formation.
//
// Design rationale:
//   - Separate class from single-module observer (different semantics)
//   - Uses formation mass/inertia, not single-module parameters
//   - Observer input = "realized wrench" from allocation (A * f), not PID commands
//   - Cascade-agnostic: no dependency on which PID terms are in PC vs spinal
//   - Auto bias calibration: after observer converges during unloaded hover,
//     records steady-state estimate as baseline and subtracts it from output.
//     This cancels the inherent offset from PID I-term compensating model error,
//     analogous to original per-module observer's differential cancellation.
//
// Version 1 (Phase U2):
//   - 3D external force estimation only (no torque)
//   - Debug-only: publishes topic, no feedback to control
//
// Version 2 (Phase U3):
//   - Full 6D wrench estimation (force + torque)
//   - Still debug-only
//
// Version 3 (future):
//   - Low-frequency feedforward compensation (Z, then pitch, then full 6D)

#pragma once

#include <ros/ros.h>
#include <Eigen/Dense>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <aerial_robot_model/utils/math_utils.h>

namespace aerial_robot_control
{

class FormationMomentumObserver
{
public:
  FormationMomentumObserver();
  ~FormationMomentumObserver() = default;

  /**
   * @brief Initialize the observer with ROS handles and parameters.
   * @param nh  Node handle for publishing debug topics.
   */
  void initialize(ros::NodeHandle nh);

  /**
   * @brief Reset observer state (call on mode switch or when entering unified mode).
   */
  void reset();

  /**
   * @brief Main update: estimate external wrench on the formation.
   *
   * Implements a generalized-momentum observer:
   *   p(t) = [M * v_w;  I * omega_body]
   *   integrate_term += (J * tau_realized - N + f_ext_hat) * dt
   *   f_ext_hat = K_obs * (p(t) - p(0) - integrate_term)
   *
   * where:
   *   tau_realized = realized wrench from allocation (A * f), NOT PID command
   *   N = gravity + gyroscopic terms
   *   K_obs = diagonal observer gain matrix
   *
   * @param formation_mass         Total mass of the assembled formation [kg].
   * @param formation_inertia      3x3 inertia matrix of the formation about formation CoG [kg·m²].
   * @param cog_rot                3x3 rotation matrix: formation body → world.
   * @param vel_w                  Formation CoG linear velocity in world frame [m/s].
   * @param omega_body             Formation angular velocity in body frame [rad/s].
   * @param realized_wrench_body   6D realized wrench in body frame [Fx,Fy,Fz,Tx,Ty,Tz] (N, N·m).
   *                               Computed as: integrated_map * target_vectoring_f * [M; I]
   *                               (i.e., allocation result converted back to force/torque space).
   * @param dt                     Time step [s].
   */
  void update(double formation_mass,
              const Eigen::Matrix3d& formation_inertia,
              const Eigen::Matrix3d& cog_rot,
              const Eigen::Vector3d& vel_w,
              const Eigen::Vector3d& omega_body,
              const Eigen::VectorXd& realized_wrench_body,
              double dt);

  // ---- Accessors (debug / future compensation) ----

  /** @brief Get estimated external force in world frame [N] (LPF-filtered, bias-subtracted). */
  Eigen::Vector3d getEstExternalForceWorld() const { return est_ext_force_w_filt_ - bias_force_w_; }

  /** @brief Get raw (pre-LPF, pre-bias) estimated external force in world frame [N]. */
  const Eigen::Vector3d& getRawEstExternalForceWorld() const { return est_ext_force_w_; }

  /** @brief Get estimated external force in body frame [N] (LPF-filtered, bias-subtracted). */
  Eigen::Vector3d getEstExternalForceBody() const { return last_cog_rot_.transpose() * (est_ext_force_w_filt_ - bias_force_w_); }

  /** @brief Get estimated external torque in body frame [N·m] (LPF-filtered, bias-subtracted, V2). */
  Eigen::Vector3d getEstExternalTorqueBody() const { return est_ext_torque_body_filt_ - bias_torque_body_; }

  /** @brief Get raw (pre-LPF, pre-bias) estimated external torque in body frame [N·m]. */
  const Eigen::Vector3d& getRawEstExternalTorqueBody() const { return est_ext_torque_body_; }

  /** @brief Get full 6D estimated external wrench (bias-subtracted) in formation_body frame.
   *  [force_body(3); torque_body(3)]. torque is zero when torque observer is disabled. */
  Eigen::VectorXd getEstExternalWrench6D() const;

  /** @brief Is the observer initialized (has received at least one update)? */
  bool isInitialized() const { return initialized_; }

  /** @brief Is the observer active (receiving updates and producing estimates)? */
  bool isActive() const { return active_; }

  /** @brief Set the observer to active/inactive. When inactive, update() is a no-op. */
  void setActive(bool active) { active_ = active; }

  /** @brief Allow bias calibration only when the controller judges the formation to be hovering stably. */
  void setBiasCalibrationAllowed(bool allowed);

  /** @brief Is the bias calibrated? */
  bool isBiasCalibrated() const { return bias_calibrated_; }

  /** @brief Soft-ramp factor used by downstream FF compensation. Returns 0 until
   *  bias is calibrated, then linearly ramps 0→1 over ff_ramp_seconds_, then 1.0.
   *  Combined with the very-low-cutoff LPF this provides the second stage of
   *  attenuation for the formation-observer feedforward path. */
  double getFfRampFactor() const;

  /** @brief Get the current bias value [N] (world frame). */
  const Eigen::Vector3d& getBias() const { return bias_force_w_; }

private:
  ros::NodeHandle nh_;

  // ---- Observer state ----
  bool initialized_;        // true after first update() call
  bool active_;             // external enable/disable

  // Momentum observer internal state
  Eigen::Vector3d init_linear_momentum_;     // p_lin(t=0)
  Eigen::Vector3d integrate_term_force_;     // accumulated integral for force channel
  Eigen::Vector3d est_ext_force_w_;          // estimated external force in world frame

  // V2: angular momentum observer (placeholder, zeroed in V1)
  Eigen::Vector3d init_angular_momentum_;    // p_ang(t=0)
  Eigen::Vector3d integrate_term_torque_;    // accumulated integral for torque channel
  Eigen::Vector3d est_ext_torque_body_;      // estimated external torque in body frame

  // ---- Bias calibration ----
  // Two-stage Dragon-style bias tracking:
  //   1) Wait bias_settle_time during stable hover, then snap bias_*_ from filt.
  //   2) After snap, continuously update bias_*_ via a slow LPF
  //      (bias_lpf_cutoff_freq, e.g. 0.02 Hz) so bias tracks system drift
  //      (battery sag, thermal, aerodynamic ground effect changes).
  // Output = filt - bias captures only mid-frequency real disturbances.
  bool bias_calibrated_;                     // true once force bias is snapped
  Eigen::Vector3d bias_force_w_;             // tracked force bias (subtracted from output)

  bool bias_torque_calibrated_;              // true once torque bias is snapped
  Eigen::Vector3d bias_torque_body_;         // tracked torque bias (subtracted from output)

  double bias_settle_time_;                  // seconds to wait before snap
  double bias_lpf_cutoff_freq_;              // Hz; LPF cutoff for continuous bias update
  int    update_count_;                      // total update() calls since initialization
  bool   bias_calibration_allowed_;          // true only while unified hover is active
  int    bias_ready_count_;                  // hover-allowed frame counter for settle timing

  // Ramp from 0→1 starting at the moment force bias finishes calibration.
  // bias_calibrated_time_ < 0 means "not yet calibrated".
  double bias_calibrated_time_;              // ros::Time::now().toSec() at completion
  double ff_ramp_seconds_;                   // duration of the 0→1 soft ramp [s]

  // Last rotation matrix (cached for body↔world conversion)
  Eigen::Matrix3d last_cog_rot_;

  // ---- Observer gains ----
  // Diagonal gain matrix K_obs: higher = faster response but more noise.
  // Recommended starting point: 3–5 for force, 2–3 for torque.
  double force_observer_gain_;
  double torque_observer_gain_;  // V2: not used in V1

  // ---- Output LPF on force estimate ----
  // Applied AFTER computing raw estimate; internal feedback still uses raw value.
  // lower freq = less noise but slower response to real external forces.
  double est_force_lpf_cutoff_freq_;     // Hz
  bool   est_force_lpf_initialized_;     // false until first update
  Eigen::Vector3d est_ext_force_w_filt_; // LPF-smoothed estimate (published / returned by accessors)

  // ---- Output LPF on torque estimate (mirrors force channel) ----
  double est_torque_lpf_cutoff_freq_;         // Hz
  bool   est_torque_lpf_initialized_;         // false until first update
  Eigen::Vector3d est_ext_torque_body_filt_;  // LPF-smoothed torque estimate

  // ---- Enable flags ----
  bool enable_force_observer_;   // V1: default true
  bool enable_torque_observer_;  // V2: default false (placeholder)

  // ---- ROS publishers (debug-only) ----
  ros::Publisher est_ext_torque_body_pub_;     // geometry_msgs/Vector3Stamped (V2)
  ros::Publisher est_ext_wrench_pub_;          // geometry_msgs/WrenchStamped (full 6D)
  ros::Publisher observer_residual_pub_;       // geometry_msgs/Vector3Stamped (force residual)
  ros::Publisher observer_residual_torque_pub_; // geometry_msgs/Vector3Stamped (torque residual, V2)
  ros::Publisher realized_wrench_debug_pub_;   // geometry_msgs/WrenchStamped (input for verification)

  // ---- Internal helpers ----
  void publishDebug(const ros::Time& stamp,
                    const Eigen::Vector3d& residual_force,
                    const Eigen::Vector3d& residual_torque);

  void loadParams();
};

} // namespace aerial_robot_control
