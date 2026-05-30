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
#include <cmath>
#include <memory>
#include <map>
#include <mutex>
#include <utility>
#include <vector>

// Forward-declare OsqpEigen::Solver so downstream packages that include this
// header (e.g. ninja) do not need to link against OsqpEigen.
namespace OsqpEigen { class Solver; }
#include <spinal/FourAxisCommand.h>
#include <spinal/TorqueAllocationMatrixInv.h>
#include <spinal/RollPitchYawTerms.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt8.h>
#include <tf/LinearMath/Vector3.h>

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
  ~BeetleUnifiedController();  // defined in .cpp where OsqpEigen::Solver is complete

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
                                const Eigen::VectorXd& desired_ext_wrench,
                                double yaw_pid_raw);

  /** @brief Optional LF-style internal wrench compensation used only as a
   *  secondary allocation reference. Gain 0 disables the effect. The input map
   *  is keyed by module id and stores 6D body-frame compensation wrench. */
  void setInternalWrenchSecondaryReference(const std::map<int, Eigen::VectorXd>& module_wrench_comp,
                                           double gain);
  void clearInternalWrenchSecondaryReference();

  /** @brief Send this module's torque_allocation_matrix_inv sub-block to its spinal.
   *  Called at mode switch / one-shot resend; not part of the control loop.
   *  @return true if matrix was sent, false if not yet computed. */
  bool sendTorqueAllocationMatrixInv();

  /** @brief Send cascade P/I/D attitude gains to this module's spinal.
   *  Uses motors.resize(1) path → spinal stores as torque-level gains and
   *  multiplies by torque_allocation_matrix_inv to get per-motor gains.
   *  v5: roll/pitch I-term is owned by the PC outer loop, so callers pass
   *  zero roll_i/pitch_i for the spinal P+D cascade. */
  void sendCascadeGains(double roll_p, double roll_i, double roll_d,
                        double pitch_p, double pitch_i, double pitch_d,
                        double yaw_d);

  /** @brief Cache cascade gains for deferred one-shot resend.
   *  Called by BeetleController::sendCascadeSetup() so that the one-shot
   *  logic inside computeUnifiedAllocation() can resend gains without
   *  needing to know the gain values. */
  void cacheCascadeGains(double roll_p, double roll_i, double roll_d,
                         double pitch_p, double pitch_i, double pitch_d,
                         double yaw_d);

  /** @brief Reset the one-shot flag so the next computeUnifiedAllocation()
   *  will re-send allocation matrix + cascade gains to this module's spinal.
   *  Call this when exiting unified mode. */
  void resetCascadeAllocSent() { cascade_alloc_sent_ = false; }

  /** @brief Monotonically increases whenever formation geometry (mass,
   *  cog_offset, inertia, rotor lever arms) is actually re-latched in
   *  updateFormationGeometry(). Callers cache the last seen value and detect
   *  changes to re-arm side-effects such as the per-follower spinal one-shot
   *  cascade matrix. The leader's own one-shot is reset internally by
   *  updateFormationGeometry() itself. */
  uint64_t getFormationRevision() const { return cached_module_model_revision_; }

  /** @brief Reset QP solver dimensions/solver internals when entering unified mode. */
  void resetQPState() {
    qp_n_vars_ = -1;
    qp_n_constraints_ = -1;
    prev_vectoring_f_.resize(0);
  }

  void setFormationModelOverride(double formation_mass,
                                 const Eigen::Vector3d& formation_cog_offset,
                                 const Eigen::Matrix3d& formation_inertia)
  {
    use_external_formation_model_ = true;
    external_formation_mass_ = formation_mass;
    external_formation_cog_offset_ = formation_cog_offset;
    external_formation_inertia_ = formation_inertia;
  }

  void clearFormationModelOverride()
  {
    use_external_formation_model_ = false;
  }

  /** @brief Update formation CoG offset and inertia from current TF. */
  bool updateFormationGeometry();

  // Per-module output commands
  struct ModuleCommand {
    std::vector<float> full_thrusts;    // scalar thrust per rotor (size = motor_num_per_module_)
    std::vector<double> gimbal_angles;  // gimbal angles (size = motor_num_per_module_ * gimbal_dof_)
  };

  struct ModuleModelDescriptor {
    double mass = 0.0;
    Eigen::Matrix3d inertia = Eigen::Matrix3d::Zero();
    std::vector<Eigen::Vector3d> rotor_origins_from_cog;
    std::map<int, int> rotor_direction;
    double mf_rate = 0.0;

    bool valid(int motor_num) const
    {
      if (!std::isfinite(mass) || mass <= 0.0 ||
          !inertia.allFinite() ||
          rotor_origins_from_cog.size() != static_cast<size_t>(motor_num) ||
          static_cast<int>(rotor_direction.size()) < motor_num ||
          !std::isfinite(mf_rate)) {
        return false;
      }
      for (const auto& origin : rotor_origins_from_cog) {
        if (!origin.allFinite()) return false;
      }
      return true;
    }
  };

  void setModuleModelDescriptor(int module_id, const ModuleModelDescriptor& model);

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
  double getCandidateYawTerm() const { return candidate_yaw_term_; }
  const Eigen::VectorXd& getTargetVectoringForce() const { return target_vectoring_f_; }
  const std::map<int, ModuleCommand>& getModuleCommands() const { return module_commands_; }
  void setCommandTargetRPY(const tf::Vector3& rpy) { command_target_rpy_ = rpy; }
  int getModuleIndex(int module_id) const;
  bool buildModuleThrustCommand(int module_id, spinal::FourAxisCommand& thrust_msg) const;
  bool buildModuleTorqueAllocationMatrixInv(int module_id, spinal::TorqueAllocationMatrixInv& msg) const;

  /**
   * @brief Compute the realized 6D wrench in body (CoG) frame from the allocation result.
   *
   * This is the allocation-model wrench commanded by the PC-side allocator:
   *   w_realized_acc = A * f   (integrated_map_ * target_vectoring_f_)
   * then converted from acc-space to force/torque space:
   *   F = M * w_realized_acc.head(3)
   *   T = I * w_realized_acc.tail(3)
   *
   * In cascade mode this does not include spinal-side P/D increments, motor
   * dynamics, or thrust/gimbal tracking errors. Treat it as a diagnostic
   * model input, not a measured actuator wrench.
   *
   * @return 6D wrench [Fx,Fy,Fz,Tx,Ty,Tz] in body frame (N, N·m).
   *         Returns zero vector if allocation has not been computed yet.
   */
  Eigen::VectorXd getRealizedWrenchBody() const;
  bool getRealizedModuleWrenchBody(int module_id, Eigen::VectorXd& realized) const;

  /** @brief Check if any rotor in the formation allocation is near thrust limits (anti-windup). */
  bool isAllocationSaturated() const;

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
  bool yaw_in_allocation_;

  // Formation state
  double formation_mass_;
  Eigen::Vector3d formation_cog_offset_;
  Eigen::Matrix3d formation_inertia_;
  bool use_external_formation_model_;
  double external_formation_mass_;
  Eigen::Vector3d external_formation_cog_offset_;
  Eigen::Matrix3d external_formation_inertia_;

  // ε-fix: cache formation geometry keyed on the assembled-IDs set.
  // Per-cycle tf::lookupTransform under large tilt produced 10 cm Z drift in
  // cog_offset (real-hw pitch=0.4 log). Latch module offsets when the
  // assembled set changes, then reuse them even if module mass/inertia updates.
  std::vector<int> cached_assembled_ids_;
  std::map<int, Eigen::Vector3d> cached_module_offsets_from_leader_;
  uint64_t module_model_revision_;
  uint64_t cached_module_model_revision_;
  std::map<int, ModuleModelDescriptor> module_models_;
  mutable std::mutex module_model_mutex_;
  mutable std::mutex allocation_mutex_;

  // Allocation matrices
  Eigen::MatrixXd integrated_map_;        // 6 x (rotor_coef * total_rotors)
  Eigen::MatrixXd integrated_map_inv_;    // pseudoinverse
  Eigen::MatrixXd integrated_map_inv_rot_; // last 3 cols of pseudoinverse (torque part)
  Eigen::VectorXd target_vectoring_f_;    // allocation result
  std::map<int, Eigen::VectorXd> module_internal_wrench_comp_;
  double internal_wrench_secondary_gain_;

  // Target yaw term for spinal cascade inner loop
  double candidate_yaw_term_;
  tf::Vector3 command_target_rpy_;

  // Per-module commands
  std::map<int, ModuleCommand> module_commands_;

  // Cascade one-shot state: deferred resend of allocation matrix + gains
  bool cascade_alloc_sent_;           // true once one-shot has fired
  bool has_cascade_gain_cache_;       // true once cacheCascadeGains() has been called
  double cached_cascade_roll_p_;
  double cached_cascade_roll_i_;
  double cached_cascade_roll_d_;
  double cached_cascade_pitch_p_;
  double cached_cascade_pitch_i_;
  double cached_cascade_pitch_d_;
  double cached_cascade_yaw_d_;

  // ROS publishers per module
  std::map<int, ros::Publisher> module_torque_alloc_inv_pubs_;  // torque_allocation_matrix_inv
  std::map<int, ros::Publisher> module_rpy_gain_pubs_;          // rpy/gain
  std::map<int, ros::Publisher> module_gimbal_dof_pubs_;        // gimbal_dof

  // Debug publishers
  ros::Publisher formation_wrench_pub_;
  ros::Publisher formation_vectoring_f_pub_;
  ros::Publisher interface_load_pub_;

  // Internal methods
  Eigen::MatrixXd buildFormationAllocationMatrix(
      const std::vector<int>& assembled_ids,
      double formation_mass,
      const Eigen::Matrix3d& formation_inertia,
      const Eigen::Vector3d& formation_cog_offset);

  Eigen::Matrix3d computeFormationInertia(
      const std::vector<int>& assembled_ids,
      const Eigen::Vector3d& formation_cog_offset);

  bool getModuleModelDescriptor(int module_id, ModuleModelDescriptor& model) const;
  Eigen::Vector3d getModuleOffsetFromLeader(int module_id) const;
  bool lookupModuleOffsetFromLeader(int module_id, Eigen::Vector3d& offset) const;
  bool getCachedModuleOffsetFromLeader(int module_id, Eigen::Vector3d& offset) const;
  std::vector<Eigen::MatrixXd> buildRotorMask() const;

  void extractThrustAndGimbal(const Eigen::VectorXd& vectoring_f,
                              const std::vector<int>& assembled_ids);

  // Full-vector QP constrained allocation
  // Decision variables: f ∈ R^{rotor_coef * n_rotors} (all force components).
  // Constraints:
  //   - Gimbal angle:  |f_x| ≤ tan(θ_max) * f_z  (linearized)
  //   - Thrust bound:  each component within [-T_max, T_max], f_z ≥ 0
  bool use_constrained_alloc_;        // if true, use OsqpEigen QP; fallback to pseudoinverse
  double alloc_lambda_;               // secondary objective weight toward balanced hover reference
  double alloc_t_max_;                // per-rotor thrust upper bound [N]
  double alloc_gimbal_limit_rad_;     // gimbal angle hard limit [rad]
  double alloc_rate_weight_;          // optional smoothness weight toward previous allocation
  double alloc_rate_limit_;           // optional per-cycle component delta bound [N], <=0 disables
  std::vector<double> alloc_module_weights_;  // module-id indexed multiplier on alloc_lambda_
  double alloc_interface_force_weight_;   // optional soft cost on interface force proxy [1/N^2]
  double alloc_interface_torque_weight_;  // optional soft cost on interface torque proxy [1/(Nm)^2]
  double alloc_interface_force_limit_;    // optional component-wise interface force proxy limit [N]
  double alloc_interface_torque_limit_;   // optional component-wise interface torque proxy limit [Nm]
  int qp_n_vars_;                     // number of QP variables (= rotor_coef * n_rotors), -1 = uninit
  int qp_n_constraints_;              // number of linear constraints, -1 = uninit
  std::unique_ptr<OsqpEigen::Solver> qp_solver_;
  Eigen::VectorXd prev_vectoring_f_;   // previous successful allocation, used by rate terms

  /**
   * @brief Full-vector constrained QP allocation.
   *
   * Formulation:
   *   min_{f} ||A*f - w||^2 + λ||f - f_ref||^2
   *   s.t.  linear gimbal-angle constraints (per rotor)
   *         component bounds
   *
   * @param alloc_matrix  Formation allocation matrix A (6 x n_cols)
   * @param w_total       Desired 6D wrench-acceleration vector
   * @param secondary_ref Preferred allocation in the nullspace / soft secondary objective
   * @param vectoring_f_out  Output: full vectoring force vector (n_cols)
   * @return true on success, false on failure (caller falls back to pseudoinverse)
   */
  bool solveFullVectorQP(const Eigen::MatrixXd& alloc_matrix,
                         const Eigen::VectorXd& w_total,
                         const Eigen::VectorXd& secondary_ref,
                         const std::vector<int>& assembled_ids,
                         const Eigen::MatrixXd& interface_load_matrix,
                         Eigen::VectorXd& vectoring_f_out);

  /** @brief Build the current secondary allocation reference.
   *  Base term is balanced hover load. Optional internal-wrench compensation
   *  adds a small per-module 6D bias through that module's allocation block
   *  when internal_wrench_secondary_gain_ > 0. */
  Eigen::VectorXd buildSecondaryAllocationReference(const std::vector<int>& assembled_ids) const;

  /** @brief Build actuator-side cut-load proxy rows for each adjacent module
   *  boundary. Rows are ordered [Fx,Fy,Fz,Tx,Ty,Tz] per cut, with the cut placed
   *  halfway between adjacent module CoGs in formation body coordinates. This is
   *  a model-based allocation proxy D*f, not a measured connector load and not
   *  gravity/inertia compensated. */
  bool buildInterfaceLoadMatrix(const std::vector<int>& assembled_ids,
                                Eigen::MatrixXd& interface_load_matrix,
                                std::vector<std::pair<int, int>>& interface_cuts) const;

  void publishInterfaceLoadDiagnostics(const Eigen::MatrixXd& interface_load_matrix,
                                       const std::vector<std::pair<int, int>>& interface_cuts,
                                       const Eigen::VectorXd& vectoring_f);

  double getModuleAllocationWeight(int module_id) const;

  void rosParamInit();
};

} // namespace aerial_robot_control
