// -*- mode: c++ -*-
// Formation-level momentum observer for assembled beetle formation.
//
// Design rationale:
//   - Separate class from single-module observer (different semantics)
//   - Uses formation mass/inertia, not single-module parameters
//   - Observer input = final-FC actuator-command wrench reconstructed locally
//     from Spinal's atomic post-clamp thrust and gimbal-target feedback
//   - The input is about the formation CoG in the virtual CoG control frame
//   - It remains a command-model estimate, not measured rotor thrust
//
// Version 1 (Phase U2):
//   - 3D external force estimation only (no torque)
//   - Publishes estimator diagnostics
//
// Version 2 (Phase U3):
//   - Full 6D wrench estimation (force + torque)
//   - BeetleController may optionally apply low-gain downstream feedback
//
// The observer itself remains estimator-only.

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
   *   p(t) = [M * v_w;  I * omega_cog]
   *   integrate_term += (J * tau_known - N + f_ext_hat) * dt
   *   f_ext_hat = K_obs * (p(t) - p(0) - integrate_term)
   *
   * where:
   *   tau_known = spatial sum of the per-module final-FC command wrenches
   *   N = gravity + gyroscopic terms
   *   K_obs = diagonal observer gain matrix
   *
   * @param formation_mass         Total mass of the assembled formation [kg].
   * @param formation_inertia      3x3 inertia matrix of the formation about formation CoG [kg·m²].
   * @param cog_rot                3x3 rotation matrix: formation body → world.
   * @param vel_w                  Formation CoG linear velocity in world frame [m/s].
   * @param omega_cog              Formation angular velocity in virtual CoG frame [rad/s].
   * @param known_actuator_wrench_cog 6D final-FC actuator-command wrench about
   *                               formation CoG, expressed in virtual CoG axes
   *                               [Fx,Fy,Fz,Tx,Ty,Tz] (N, N·m).
   * @param dt                     Time step [s].
   */
  void update(double formation_mass,
              const Eigen::Matrix3d& formation_inertia,
              const Eigen::Matrix3d& cog_rot,
              const Eigen::Vector3d& vel_w,
              const Eigen::Vector3d& omega_cog,
              const Eigen::VectorXd& known_actuator_wrench_cog,
              double dt);

  // ---- Accessors (debug / optional controller feedback) ----

  /** @brief Get estimated external force in world frame [N] (LPF-filtered).
   *  No bias subtraction is applied. Treat this as a diagnostic model residual
   *  until the observer equation has been validated on hardware. */
  Eigen::Vector3d getEstExternalForceWorld() const { return est_ext_force_w_filt_; }

  /** @brief Get raw (pre-LPF) estimated external force in world frame [N]. */
  const Eigen::Vector3d& getRawEstExternalForceWorld() const { return est_ext_force_w_; }

  /** @brief Get estimated external force in virtual CoG frame [N] (LPF-filtered). */
  Eigen::Vector3d getEstExternalForceCog() const { return last_cog_rot_.transpose() * est_ext_force_w_filt_; }

  /** @brief Get estimated external torque in virtual CoG frame [N·m] (LPF-filtered, V2). */
  Eigen::Vector3d getEstExternalTorqueCog() const { return est_ext_torque_cog_filt_; }

  /** @brief Get raw (pre-LPF) estimated external torque in virtual CoG frame [N·m]. */
  const Eigen::Vector3d& getRawEstExternalTorqueCog() const { return est_ext_torque_cog_; }

  /** @brief Get full 6D estimated external wrench about and expressed in assembly_cog.
   *  [force_cog(3); torque_cog(3)]. torque is zero when torque observer is disabled. */
  Eigen::VectorXd getEstExternalWrench6D() const;

  /** @brief Is the observer initialized (has received at least one update)? */
  bool isInitialized() const { return initialized_; }

  /** @brief Is the observer active (receiving updates and producing estimates)? */
  bool isActive() const { return active_; }

  /** @brief Set the observer to active/inactive. When inactive, update() is a no-op. */
  void setActive(bool active) { active_ = active; }

  /** @brief Gate downstream FF/feedback compensation. Called by the controller when the
   *  formation is judged to be in stable hover. When the gate flips OFF→ON, the
   *  ff_armed_time_ is recorded and getFfRampFactor() ramps 0→1 over
   *  ff_ramp_seconds_. */
  void setFfArmed(bool armed);

  /** @brief Is the downstream FF/feedback gate currently armed? */
  bool isFfReady() const { return ff_armed_ && initialized_; }

  /** @brief Soft-ramp factor [0,1] used by downstream FF/feedback compensation.
   *  0 until ff is armed, then linearly ramps 0→1 over ff_ramp_seconds_, then 1.0. */
  double getFfRampFactor() const;

private:
  ros::NodeHandle nh_;

  // ---- Observer state ----
  bool initialized_;        // true after all enabled channels have initialized
  bool force_initialized_;
  bool torque_initialized_;
  bool active_;             // external enable/disable

  // Momentum observer internal state
  Eigen::Vector3d init_linear_momentum_;     // p_lin(t=0)
  Eigen::Vector3d prev_linear_momentum_;     // p_lin(t-dt), for equation diagnostics
  Eigen::Vector3d integrate_term_force_;     // accumulated integral for force channel
  Eigen::Vector3d est_ext_force_w_;          // estimated external force in world frame
  bool prev_linear_momentum_valid_;

  // V2: angular momentum observer (placeholder, zeroed in V1)
  Eigen::Vector3d init_angular_momentum_;    // p_ang(t=0)
  Eigen::Vector3d integrate_term_torque_;    // accumulated integral for torque channel
  Eigen::Vector3d est_ext_torque_cog_;       // estimated external torque in virtual CoG frame

  // ---- FF arming (no bias subtraction) ----
  // Used only as a downstream gate/ramp; the observer itself stays estimator-only.
  bool   ff_armed_;                          // FF gate state (controlled by setFfArmed)
  double ff_armed_time_;                     // ros::Time::now().toSec() when armed (<0 = unarmed)
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
  Eigen::Vector3d est_ext_torque_cog_filt_;   // LPF-smoothed torque estimate

  // ---- Enable flags ----
  bool enable_force_observer_;   // V1: default true
  bool enable_torque_observer_;  // V2: default false (placeholder)

  // ---- ROS publishers (debug-only) ----
  ros::Publisher est_ext_torque_cog_pub_;      // geometry_msgs/Vector3Stamped (V2)
  ros::Publisher est_ext_wrench_pub_;          // geometry_msgs/WrenchStamped (full 6D)
  ros::Publisher observer_residual_pub_;       // geometry_msgs/Vector3Stamped (force residual)
  ros::Publisher observer_residual_torque_pub_; // geometry_msgs/Vector3Stamped (torque residual, V2)
  ros::Publisher known_wrench_input_pub_;       // geometry_msgs/WrenchStamped (input for verification)

  // ---- Internal helpers ----
  void publishDebug(const ros::Time& stamp,
                    const Eigen::Vector3d& residual_force,
                    const Eigen::Vector3d& residual_torque);

  void loadParams();
};

} // namespace aerial_robot_control
