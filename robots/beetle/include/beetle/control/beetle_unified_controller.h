// -*- mode: c++ -*-
// Unified controller for assembled beetle formation (Plan B).
//
// Design (following assemble_quadrotors / FullyActuatedController):
//   PC side:  position PID -> formation-wide allocation -> base_thrust per motor
//   Spinal:   receives base_thrust + target_rpy + RPY gains + torque_allocation_matrix_inv
//             -> does attitude PID internally (roll_pitch_term + yaw_term)
//
// This keeps spinal's attitude inner loop active, avoiding the problems of
// zeroing spinal gains (residual yaw terms, PWM saturation interference, etc.)

#pragma once

#include <ros/ros.h>
#include <Eigen/Dense>
#include <spinal/FourAxisCommand.h>
#include <spinal/RollPitchYawTerms.h>
#include <spinal/TorqueAllocationMatrixInv.h>
#include <spinal/PMatrixPseudoInverseWithInertia.h>
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
   * @brief Compute position-only allocation for the formation.
   *
   * @param target_acc_cog  3D acceleration in CoG body frame [ax, ay, az]
   * @param candidate_yaw_term  Yaw PID term (scaled), forwarded to spinal
   * @param target_rpy  Target roll/pitch/yaw for spinal's attitude PID
   * @return true if allocation succeeded.
   */
  bool computePositionAllocation(const Eigen::Vector3d& target_acc_cog,
                                 double candidate_yaw_term,
                                 const Eigen::Vector3d& target_rpy);

  /** @brief Send torque_allocation_matrix_inv to each module's spinal. */
  void sendFormationTorqueAllocationMatrixInv();

  /** @brief Send RPY gains to each module's spinal. */
  void sendFormationAttitudeGains(double roll_p, double roll_i, double roll_d,
                                  double pitch_p, double pitch_i, double pitch_d,
                                  double yaw_d);

  /** @brief Send p_matrix + inertia to each module's spinal for gyro compensation. */
  void sendFormationPMatrixInertia();

  /** @brief Publish base_thrust + target_rpy to all modules. */
  void publishCommands();

  /** @brief Update formation geometry from TF. */
  bool updateFormationGeometry();

  // Accessors
  const Eigen::Vector3d& getFormationCogOffset() const { return formation_cog_offset_; }
  const Eigen::Matrix3d& getFormationInertia() const { return formation_inertia_; }
  double getFormationMass() const { return formation_mass_; }
  const Eigen::MatrixXd& getIntegratedMap() const { return integrated_map_; }

  struct ModuleCommand {
    std::vector<float> base_thrust;     // vectoring force components for spinal
    std::vector<double> gimbal_angles;  // gimbal angles (for gimbal_calc_in_fc=false, sent separately)
    float target_roll;
    float target_pitch;
    float candidate_yaw_term;
  };

  const std::map<int, ModuleCommand>& getModuleCommands() const { return module_commands_; }

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
  double torque_alloc_pub_interval_;

  // Formation state
  double formation_mass_;
  Eigen::Vector3d formation_cog_offset_;
  Eigen::Matrix3d formation_inertia_;

  // Allocation matrices (formation-wide)
  Eigen::MatrixXd integrated_map_;        // 6 x total_cols
  Eigen::MatrixXd integrated_map_inv_;    // total_cols x 6
  Eigen::MatrixXd q_mat_inv_trans_;       // total_cols x 3 (position part)
  Eigen::MatrixXd q_mat_inv_rot_;         // total_cols x 3 (attitude part)

  // Per-module torque allocation (split from q_mat_inv_rot_)
  std::map<int, Eigen::MatrixXd> per_module_torque_alloc_inv_;

  // Per-module commands
  std::map<int, ModuleCommand> module_commands_;

  // ROS publishers per module
  std::map<int, ros::Publisher> module_thrust_pubs_;   // unified_thrust_cmd
  std::map<int, ros::Publisher> module_gimbal_pubs_;   // unified_gimbal_cmd
  std::map<int, ros::Publisher> module_rpy_gain_pubs_; // rpy/gain
  std::map<int, ros::Publisher> module_torque_alloc_pubs_; // torque_allocation_matrix_inv
  std::map<int, ros::Publisher> module_p_matrix_pubs_; // p_matrix_pseudo_inverse_inertia

  // Debug publishers
  ros::Publisher formation_wrench_pub_;
  ros::Publisher formation_vectoring_f_pub_;

  double torque_alloc_pub_stamp_;

  // Internal methods
  Eigen::MatrixXd buildFormationAllocationMatrix(
      const std::vector<int>& assembled_ids,
      double formation_mass,
      const Eigen::Matrix3d& formation_inertia,
      const Eigen::Vector3d& formation_cog_offset);

  Eigen::Matrix3d computeFormationInertia(
      const std::vector<int>& assembled_ids,
      const Eigen::Vector3d& formation_cog_offset);

  void extractBaseThrust(const Eigen::VectorXd& vectoring_f_trans,
                         const std::vector<int>& assembled_ids,
                         double candidate_yaw_term,
                         const Eigen::Vector3d& target_rpy);

  void splitTorqueAllocationPerModule(const std::vector<int>& assembled_ids);

  void rosParamInit();
};

} // namespace aerial_robot_control
