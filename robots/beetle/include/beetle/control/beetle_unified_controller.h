// -*- mode: c++ -*-
// Unified 4N-rotor controller for assembled beetle formation.
//
// Design: PC outer loop (40Hz) does formation-level 6-DOF allocation.
// Spinal inner loop (1000Hz) does P+D attitude tracking per-motor.
// PC retains I-term only for roll/pitch (formation-level slow bias).
//
// Data flow:
//   PC → spinal per module:
//     - FourAxisCommand: base_thrust[motor_num*rotor_coef] + angles[roll,pitch,yaw_term]
//     - TorqueAllocationMatrixInv: per-module sub-block of formation inv_rot
//     - RollPitchYawTerms: torque-level P/D gains (I=0)
//     - gimbal_dof = 1 (so spinal does vectoring force decomposition)
//   Spinal (1000Hz):
//     - err = target_angle - IMU_angle → P+D → roll_pitch_term[i]
//     - target_thrust[i] = base_thrust[i] + roll_pitch_term[i] + yaw_term[i]
//     - sqrt+atan2 → thrust magnitude + gimbal angle → PWM + servo

#pragma once

#include <ros/ros.h>
#include <Eigen/Dense>
#include <set>
#include <spinal/FourAxisCommand.h>
#include <spinal/TorqueAllocationMatrixInv.h>
#include <spinal/RollPitchYawTerms.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int32.h>

#include <beetle/model/beetle_robot_model.h>
#include <beetle/beetle_navigation.h>
#include <aerial_robot_estimation/state_estimation.h>
#include <aerial_robot_model/utils/math_utils.h>

namespace aerial_robot_control
{

class BeetleUnifiedController
{
public:
  BeetleUnifiedController();
  ~BeetleUnifiedController() = default;

  void initialize(ros::NodeHandle nh,
                  boost::shared_ptr<BeetleRobotModel> robot_model,
                  boost::shared_ptr<aerial_robot_navigation::BeetleNavigator> navigator,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator);

  /**
   * @brief Main entry: compute thrust + gimbal commands for ALL rotors in the formation.
   *
   * @param target_wrench_acc_cog  6D wrench in acceleration space (from LEADER's PID):
   *                               [acc_x, acc_y, acc_z, ang_acc_roll, ang_acc_pitch, ang_acc_yaw]
   *                               In CoG frame, referenced to formation CoG.
   * @param desired_ext_wrench     6D external wrench demand in body frame [Fx,Fy,Fz,Tx,Ty,Tz] (N, Nm).
   *                               Added as feedforward BEFORE allocation.
   * @return true if allocation succeeded, false otherwise.
   */
  bool computeUnifiedAllocation(const Eigen::VectorXd& target_wrench_acc_cog,
                                const Eigen::VectorXd& desired_ext_wrench);

  /** @brief Send computed commands to all modules via ROS topics.
   *  Cascade mode: base_thrust[motor_num*rotor_coef] + angles[roll,pitch,yaw_term].
   *  Also publishes gimbal_dof=1 to each module's spinal. */
  void publishCommands();

  /** @brief Send torque_allocation_matrix_inv sub-blocks to each module's spinal.
   *  Each module receives only its own rows (motor_num_per_module_ * rotor_coef_ rows × 3 cols).
   *  Called once at mode switch and periodically to handle late spinal startup. */
  void sendTorqueAllocationMatrixInv();

  /** @brief Send cascade P/D attitude gains to each module's spinal.
   *  Uses motors.resize(1) path → spinal stores as torque-level gains and
   *  multiplies by torque_allocation_matrix_inv to get per-motor gains.
   *  I=0 because PC handles I-term. */
  void sendCascadeGains(double roll_p, double roll_d,
                        double pitch_p, double pitch_d,
                        double yaw_d);

  /** @brief Cache cascade gains for deferred one-shot resend.
   *  Called by BeetleController::sendCascadeSetup() so that the one-shot
   *  logic inside computeUnifiedAllocation() can resend gains without
   *  needing to know the gain values. */
  void cacheCascadeGains(double roll_p, double roll_d,
                         double pitch_p, double pitch_d,
                         double yaw_d);

  /** @brief Reset the one-shot flag so the next computeUnifiedAllocation()
   *  will re-send allocation matrix + cascade gains to all spinals.
   *  Call this when exiting unified mode. */
  void resetCascadeAllocSent() { cascade_alloc_sent_ = false; }

  /** @brief Update formation CoG offset and inertia from current TF. */
  bool updateFormationGeometry();

  // Per-module output commands
  struct ModuleCommand {
    std::vector<float> full_thrusts;    // scalar thrust per rotor (size = motor_num_per_module_)
    std::vector<double> gimbal_angles;  // gimbal angles (size = motor_num_per_module_ * gimbal_dof_)
  };

  // Accessors — Formation Model Interface
  // Exposes the unified formation as a coherent "virtual robot model" for:
  //   - Model-based seed computation (Phase II)
  //   - Paper presentation ("runtime whole-body model synthesis")
  //   - Future LQI/LQR upgrade (Phase IV)
  const Eigen::Vector3d& getFormationCogOffset() const { return formation_cog_offset_; }
  const Eigen::Matrix3d& getFormationInertia() const { return formation_inertia_; }
  double getFormationMass() const { return formation_mass_; }
  int getModuleCount() const { return static_cast<int>(module_commands_.size()); }
  int getMotorNumPerModule() const { return motor_num_per_module_; }
  double getSingleModuleMass() const { return robot_model_ ? robot_model_->getMass() : 0.0; }
  const Eigen::MatrixXd& getFormationWrenchMatrix() const { return integrated_map_; }
  const Eigen::MatrixXd& getFormationWrenchMatrixInv() const { return integrated_map_inv_; }
  const Eigen::MatrixXd& getFormationWrenchMatrixInvRot() const { return integrated_map_inv_rot_; }
  double getTargetRoll() const { return target_roll_; }
  double getTargetPitch() const { return target_pitch_; }
  double getCandidateYawTerm() const { return candidate_yaw_term_; }
  const Eigen::VectorXd& getTargetVectoringForce() const { return target_vectoring_f_; }
  const std::map<int, ModuleCommand>& getModuleCommands() const { return module_commands_; }

  /** @brief Check if any rotor in the formation allocation is near thrust limits (anti-windup). */
  bool isAllocationSaturated() const;

  // ---- FOLLOWER Ready Sync (P2.1) ----
  // LEADER waits for all FOLLOWERs to report ready before enabling outer-loop PID.
  // FOLLOWERs publish their ID on /<myname><leader_id>/unified_control/follower_ready
  // when they have sent cascade gains + forwarded the first valid unified command.

  /** @brief Check if all expected FOLLOWERs have reported ready.
   *  Returns true when every non-LEADER assembled module has acked,
   *  OR when the timeout has elapsed. Always true if only 1 module (solo). */
  bool allFollowersReady() const;

  /** @brief Reset follower ready state. Call when entering/exiting unified mode. */
  void resetFollowerReady();

  /** @brief Number of followers still not ready. For logging. */
  int pendingFollowerCount() const;

  /** @brief Increment the wait frame counter (called each frame while waiting). */
  void incrementFollowerReadyWait() { follower_ready_wait_count_++; }

  /** @brief Get current wait frame count (for logging/timeout check). */
  int getFollowerReadyWaitCount() const { return follower_ready_wait_count_; }

private:
  ros::NodeHandle nh_;
  boost::shared_ptr<BeetleRobotModel> robot_model_;
  boost::shared_ptr<aerial_robot_navigation::BeetleNavigator> navigator_;
  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator_;

  // Parameters
  int motor_num_per_module_;
  int gimbal_dof_;
  int rotor_coef_;
  bool gimbal_calc_in_fc_;

  // Formation state
  double formation_mass_;
  Eigen::Vector3d formation_cog_offset_;
  Eigen::Matrix3d formation_inertia_;

  // Allocation matrices
  Eigen::MatrixXd integrated_map_;        // 6 x (rotor_coef * total_rotors)
  Eigen::MatrixXd integrated_map_inv_;    // pseudoinverse
  Eigen::MatrixXd integrated_map_inv_rot_; // last 3 cols of pseudoinverse (torque part)
  Eigen::VectorXd target_vectoring_f_;    // allocation result

  // Target angles for spinal cascade inner loop
  double target_roll_;
  double target_pitch_;
  double candidate_yaw_term_;

  // Per-module commands
  std::map<int, ModuleCommand> module_commands_;

  // Cascade one-shot state: deferred resend of allocation matrix + gains
  bool cascade_alloc_sent_;           // true once one-shot has fired
  bool has_cascade_gain_cache_;       // true once cacheCascadeGains() has been called
  double cached_cascade_roll_p_;
  double cached_cascade_roll_d_;
  double cached_cascade_pitch_p_;
  double cached_cascade_pitch_d_;
  double cached_cascade_yaw_d_;

  // ROS publishers per module
  std::map<int, ros::Publisher> module_thrust_pubs_;   // unified_thrust_cmd (FourAxisCommand)
  std::map<int, ros::Publisher> module_torque_alloc_inv_pubs_;  // torque_allocation_matrix_inv
  std::map<int, ros::Publisher> module_rpy_gain_pubs_;          // rpy/gain
  std::map<int, ros::Publisher> module_gimbal_dof_pubs_;        // gimbal_dof

  // Debug publishers
  ros::Publisher formation_wrench_pub_;
  ros::Publisher formation_vectoring_f_pub_;

  // FOLLOWER Ready Sync state
  ros::Subscriber follower_ready_sub_;               // subscribe to follower_ready topic
  std::set<int> follower_ready_set_;                  // IDs of followers that reported ready
  int follower_ready_wait_count_;                     // frames since reset (for timeout)
  static constexpr int FOLLOWER_READY_TIMEOUT_FRAMES = 120;  // 3s @40Hz
  void followerReadyCallback(const std_msgs::Int32& msg);

  // Internal methods
  Eigen::MatrixXd buildFormationAllocationMatrix(
      const std::vector<int>& assembled_ids,
      double formation_mass,
      const Eigen::Matrix3d& formation_inertia,
      const Eigen::Vector3d& formation_cog_offset);

  Eigen::Matrix3d computeFormationInertia(
      const std::vector<int>& assembled_ids,
      const Eigen::Vector3d& formation_cog_offset);

  void extractThrustAndGimbal(const Eigen::VectorXd& vectoring_f,
                              const std::vector<int>& assembled_ids);

  void rosParamInit();
};

} // namespace aerial_robot_control
