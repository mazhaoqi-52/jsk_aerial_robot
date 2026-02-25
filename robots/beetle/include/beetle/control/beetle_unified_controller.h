// -*- mode: c++ -*-
// Unified 4N-rotor controller for assembled beetle formation.
// Treats the entire assembly as a single rigid body with N*4 gimbal rotors.
// Allocation follows the same gimbalrotor approach (full_q_mat * integrated_rot)
// but extended to all rotors across all assembled modules.
//
// Architecture (matching GimbalrotorController gimbal_calc_in_fc=true path):
//   FC (40Hz):  XYZ position PID → base_thrust (2D vectoring, per-rotor)
//   Spinal (1kHz): RPY attitude PID using TorqueAllocationMatrixInv → roll_pitch_term
//   Spinal combines base_thrust + roll_pitch_term → scalar thrust + gimbal angle

#pragma once

#include <ros/ros.h>
#include <Eigen/Dense>
#include <spinal/FourAxisCommand.h>
#include <spinal/TorqueAllocationMatrixInv.h>
#include <spinal/RollPitchYawTerms.h>
#include <spinal/DesireCoord.h>
#include <std_msgs/UInt8.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float32MultiArray.h>

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
   * @brief Main entry: compute allocation for the assembled formation.
   *
   * Splits allocation into:
   *   - base_thrust (2D vectoring, position-only) → sent to spinal as FourAxisCommand.base_thrust
   *   - torque_allocation_matrix_inv (per-module 8x3) → sent to spinal for high-freq RPY PID
   *
   * @param target_wrench_acc_cog  6D wrench in acceleration space (from LEADER's PID):
   *                               [acc_x, acc_y, acc_z, ang_acc_roll, ang_acc_pitch, ang_acc_yaw]
   * @param desired_ext_wrench     6D external wrench demand in body frame [Fx,Fy,Fz,Tx,Ty,Tz] (N, Nm).
   * @return true if allocation succeeded, false otherwise.
   */
  bool computeUnifiedAllocation(const Eigen::VectorXd& target_wrench_acc_cog,
                                const Eigen::VectorXd& desired_ext_wrench);

  /** @brief Send computed commands to all modules via ROS topics.
   *  Publishes: FourAxisCommand (base_thrust 2D vectoring + target RPY angles),
   *             TorqueAllocationMatrixInv, RPY gains, DesireCoord, gimbal_dof.
   *  @param target_roll  Target roll angle for spinal attitude PID
   *  @param target_pitch Target pitch angle for spinal attitude PID
   *  @param candidate_yaw_term  Yaw control term for spinal
   *  @param rpy_p_gains  RPY proportional gains [roll, pitch, yaw]
   *  @param rpy_i_gains  RPY integral gains [roll, pitch, yaw]
   *  @param rpy_d_gains  RPY derivative gains [roll, pitch, yaw]
   */
  void publishCommands(double target_roll, double target_pitch, double candidate_yaw_term,
                       const std::vector<double>& rpy_p_gains,
                       const std::vector<double>& rpy_i_gains,
                       const std::vector<double>& rpy_d_gains);

  // Per-module output commands
  // In unified mode with gimbal_calc_in_fc=true logic:
  //   base_thrust_2d: 2D vectoring force per rotor (size = motor_num * rotor_coef = 8)
  //                   [fx_r1, fz_r1, fx_r2, fz_r2, fx_r3, fz_r3, fx_r4, fz_r4]
  //   torque_alloc_inv: per-module 8x3 sub-matrix for spinal's TorqueAllocationMatrixInv
  struct ModuleCommand {
    std::vector<float> base_thrust_2d;        // size = motor_num_per_module_ * rotor_coef_ (= 8)
    Eigen::MatrixXd torque_alloc_inv;         // motor_num*rotor_coef x 3
  };

  /** @brief Get formation allocation debug info. */
  const Eigen::VectorXd& getTargetVectoringForce() const { return target_vectoring_f_; }
  const Eigen::MatrixXd& getIntegratedMap() const { return integrated_map_; }
  const std::map<int, ModuleCommand>& getModuleCommands() const { return module_commands_; }

  /** @brief Get the candidate yaw term computed from allocation. */
  double getCandidateYawTerm() const { return candidate_yaw_term_; }

  /** @brief Get cached formation geometry (valid after computeUnifiedAllocation or updateFormationGeometry). */
  const Eigen::Vector3d& getFormationCogOffset() const { return formation_cog_offset_; }
  const Eigen::Matrix3d& getFormationInertia() const { return formation_inertia_; }

  /**
   * @brief Update formation CoG offset and inertia from current TF, without running allocation.
   * @return true if TF lookups succeeded.
   */
  bool updateFormationGeometry();

private:
  ros::NodeHandle nh_;
  boost::shared_ptr<BeetleRobotModel> robot_model_;
  boost::shared_ptr<aerial_robot_navigation::BeetleNavigator> navigator_;
  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator_;

  // Parameters (read from robot model, NOT hardcoded)
  int motor_num_per_module_;   // 4 for beetle
  int gimbal_dof_;             // 1 for beetle
  int rotor_coef_;             // gimbal_dof + 1 = 2

  // Formation allocation results
  Eigen::MatrixXd integrated_map_;
  Eigen::MatrixXd integrated_map_inv_;
  Eigen::MatrixXd integrated_map_inv_trans_;  // left 3 cols (XYZ → vectoring force)
  Eigen::MatrixXd integrated_map_inv_rot_;    // right 3 cols (RPY → vectoring force) = TorqueAllocationMatrixInv
  Eigen::VectorXd target_vectoring_f_;        // full vectoring force (trans + rot)
  Eigen::VectorXd target_vectoring_f_trans_;  // position-only vectoring force → base_thrust
  double candidate_yaw_term_;                 // yaw PD term for spinal
  Eigen::Vector3d formation_cog_offset_;
  Eigen::Matrix3d formation_inertia_;

  std::map<int, ModuleCommand> module_commands_;

  // ROS publishers: per-module spinal commands
  std::map<int, ros::Publisher> module_thrust_pubs_;        // unified_thrust_cmd (FourAxisCommand)
  std::map<int, ros::Publisher> module_torque_alloc_pubs_;  // unified_torque_alloc_inv (TorqueAllocationMatrixInv)
  std::map<int, ros::Publisher> module_rpy_gain_pubs_;      // unified_rpy_gain (RollPitchYawTerms)
  std::map<int, ros::Publisher> module_desire_coord_pubs_;  // unified_desire_coord (DesireCoord)
  std::map<int, ros::Publisher> module_gimbal_dof_pubs_;    // unified_gimbal_dof (UInt8)

  // Debug publisher
  ros::Publisher formation_wrench_pub_;
  ros::Publisher formation_vectoring_f_pub_;

  Eigen::MatrixXd buildFormationAllocationMatrix(
      const std::vector<int>& assembled_ids,
      double formation_mass,
      const Eigen::Matrix3d& formation_inertia,
      const Eigen::Vector3d& formation_cog_offset);

  Eigen::Matrix3d computeFormationInertia(const std::vector<int>& assembled_ids,
                                          const Eigen::Vector3d& formation_cog_offset);

  /**
   * @brief Extract per-module base_thrust (2D vectoring, position-only) and
   *        torque_allocation_matrix_inv from the full allocation result.
   */
  void extractModuleCommands(const Eigen::VectorXd& vectoring_f_trans,
                             const Eigen::MatrixXd& map_inv_rot,
                             const std::vector<int>& assembled_ids);

  void rosParamInit();
};

} // namespace aerial_robot_control
