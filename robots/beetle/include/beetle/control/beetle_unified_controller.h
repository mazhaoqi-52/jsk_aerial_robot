// -*- mode: c++ -*-
// Unified 4N-rotor controller for assembled beetle formation.
// Treats the entire assembly as a single rigid body with N*4 gimbal rotors.
// Allocation follows the same gimbalrotor approach (full_q_mat * integrated_rot)
// but extended to all rotors across all assembled modules.

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
   *                               Already in CoG frame of the LEADER (treated as formation CoG).
   * @param desired_ext_wrench     6D external wrench demand in body frame [Fx,Fy,Fz,Tx,Ty,Tz] (N, Nm).
   *                               Added as feedforward BEFORE allocation.
   * @return true if allocation succeeded, false otherwise.
   */
  bool computeUnifiedAllocation(const Eigen::VectorXd& target_wrench_acc_cog,
                                const Eigen::VectorXd& desired_ext_wrench);

  /** @brief Send computed commands to all modules via ROS topics. */
  void publishCommands();

  // Per-module output commands (public for LEADER to forward its own share)
  struct ModuleCommand {
    std::vector<float> full_thrusts;    // size = motor_num_per_module_
    std::vector<double> gimbal_angles;  // size = motor_num_per_module_ * gimbal_dof_
  };

  /** @brief Get formation allocation debug info. */
  const Eigen::VectorXd& getTargetVectoringForce() const { return target_vectoring_f_; }
  const Eigen::MatrixXd& getIntegratedMap() const { return integrated_map_; }
  const std::map<int, ModuleCommand>& getModuleCommands() const { return module_commands_; }

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
  Eigen::VectorXd target_vectoring_f_;

  std::map<int, ModuleCommand> module_commands_;

  // ROS publishers: per-module thrust + gimbal
  std::map<int, ros::Publisher> module_thrust_pubs_;
  std::map<int, ros::Publisher> module_gimbal_pubs_;

  // Debug publisher
  ros::Publisher formation_wrench_pub_;
  ros::Publisher formation_vectoring_f_pub_;

  /**
   * @brief Build the formation-wide allocation matrix.
   *
   * This is the core math: extends gimbalrotor_controller's allocation to N modules.
   * For each module i with motor_num_per_module_ rotors:
   *   1. Get rotor positions relative to formation CoG (via TF lookup)
   *   2. Build wrench_map (6x3) per rotor: [I; skew(r) + m_f_rate*dir*I]
   *   3. Apply gimbal mask via thrust_coord_rot
   *   4. Stack into formation-wide integrated_map (6 x N*motor_num*rotor_coef)
   *
   * @param assembled_ids  Sorted list of assembled module IDs.
   * @param formation_mass Total mass of the formation.
   * @param formation_inertia Total inertia of the formation (with parallel axis).
   * @return The 6 x (N * motor_num * rotor_coef) integrated allocation matrix.
   */
  Eigen::MatrixXd buildFormationAllocationMatrix(
      const std::vector<int>& assembled_ids,
      double formation_mass,
      const Eigen::Matrix3d& formation_inertia,
      const Eigen::Vector3d& formation_cog_offset);

  /**
   * @brief Compute formation inertia using parallel axis theorem.
   *
   * I_formation = sum_i [ I_module_i + m_i * (d_i^T*d_i*I - d_i*d_i^T) ]
   * where d_i is the offset of module_i's CoG from formation CoG.
   */
  Eigen::Matrix3d computeFormationInertia(const std::vector<int>& assembled_ids,
                                          const Eigen::Vector3d& formation_cog_offset);

  /**
   * @brief Extract per-rotor thrust magnitude and gimbal angle from vectoring force.
   *
   * For gimbal_dof_=1: thrust = norm(f_i), angle = atan2(-f_i[0], f_i[1])
   * Same formula as gimbalrotor_controller.cpp lines 271-280.
   */
  void extractThrustAndGimbal(const Eigen::VectorXd& vectoring_f,
                              const std::vector<int>& assembled_ids);

  void rosParamInit();
};

} // namespace aerial_robot_control
