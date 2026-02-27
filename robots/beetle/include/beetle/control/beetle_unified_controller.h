// -*- mode: c++ -*-
// Unified 4N-rotor controller for assembled beetle formation.
//
// Design: PC does full 6-DOF PID (position + attitude) and formation-wide
// pseudoinverse allocation. Spinal acts as pure PWM executor (zero attitude gains).
// This matches beetle's normal gimbal_calc_in_fc=false architecture where spinal
// has gimbal_dof=0 and torque_alloc_inv=0, so it never does attitude PID.

#pragma once

#include <ros/ros.h>
#include <Eigen/Dense>
#include <spinal/FourAxisCommand.h>
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

  /** @brief Send computed commands to all modules via ROS topics. */
  void publishCommands();

  /** @brief Update formation CoG offset and inertia from current TF. */
  bool updateFormationGeometry();

  // Per-module output commands
  struct ModuleCommand {
    std::vector<float> full_thrusts;    // scalar thrust per rotor (size = motor_num_per_module_)
    std::vector<double> gimbal_angles;  // gimbal angles (size = motor_num_per_module_ * gimbal_dof_)
  };

  // Accessors
  const Eigen::Vector3d& getFormationCogOffset() const { return formation_cog_offset_; }
  const Eigen::Matrix3d& getFormationInertia() const { return formation_inertia_; }
  double getFormationMass() const { return formation_mass_; }
  const Eigen::MatrixXd& getIntegratedMap() const { return integrated_map_; }
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

  // Formation state
  double formation_mass_;
  Eigen::Vector3d formation_cog_offset_;
  Eigen::Matrix3d formation_inertia_;

  // Allocation matrices
  Eigen::MatrixXd integrated_map_;        // 6 x (rotor_coef * total_rotors)
  Eigen::MatrixXd integrated_map_inv_;    // pseudoinverse
  Eigen::VectorXd target_vectoring_f_;    // allocation result

  // Per-module commands
  std::map<int, ModuleCommand> module_commands_;

  // ROS publishers per module
  std::map<int, ros::Publisher> module_thrust_pubs_;   // unified_thrust_cmd
  std::map<int, ros::Publisher> module_gimbal_pubs_;   // unified_gimbal_cmd

  // Debug publishers
  ros::Publisher formation_wrench_pub_;
  ros::Publisher formation_vectoring_f_pub_;

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
