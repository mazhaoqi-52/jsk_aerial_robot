// -*- mode: c++ -*-
// Unified 4N-rotor controller for assembled beetle formation.
//
// Design: PC outer loop (40Hz) does formation-level 6-DOF allocation.
// Spinal inner loop (1000Hz) does P+D attitude tracking per-motor.
// PC retains roll/pitch I-term as a soft allocation target.
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
#include <Eigen/Sparse>
#include <cmath>
#include <cstdint>
#include <memory>
#include <map>
#include <mutex>
#include <utility>
#include <vector>

// Forward-declare OsqpEigen::Solver so downstream packages that include this
// header (e.g. ninja) do not need to link against OsqpEigen.
namespace OsqpEigen { class Solver; }
#include <spinal/FourAxisCommand.h>
#include <spinal/ActuatorCommandFeedback.h>
#include <spinal/MotorInfo.h>
#include <spinal/Pwms.h>
#include <spinal/TorqueAllocationMatrixInv.h>
#include <spinal/RollPitchYawTerms.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float32.h>
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
  // Grants the offline allocation unit test access to the private QP solver
  // (solveFullVectorQP) and its tuning members, so the math can be exercised
  // without ROS / hardware. Test-only; no effect on production behavior.
  friend class BeetleUnifiedAllocTest;

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
   * @param target_wrench_acc_cog  6D soft wrench target in acceleration space:
   *                               [acc_x, acc_y, acc_z, ang_acc_roll, ang_acc_pitch, ang_acc_yaw]
   *                               In CoG frame, referenced to formation CoG.
   * @param desired_ext_wrench     6D external wrench demand at formation CoG in body frame
   *                               [Fx,Fy,Fz,Tx,Ty,Tz] (N, Nm). Active task axes can be
   *                               tracked as narrow hard bands, with soft weights used
   *                               for remaining axes and secondary shaping.
   * @param priority_wrench_acc_cog Optional 6D reference for hard priority bands. When empty,
   *                                hard bands use target_wrench_acc_cog. Rows whose priority
   *                                center is approximately zero are kept soft-only.
   * @param task_wrench_weights    Optional 6D task soft tracking weights. When empty, controller
   *                               parameters choose the task weighting.
   * @param observer_feedback_wrench Optional 6D residual-feedback wrench in the virtual
   *                                 CoG control frame (the formation observer's native frame).
   * @param observer_feedback_wrench_weights Optional low weights paired with observer_feedback_wrench.
   * @return true if allocation succeeded, false otherwise.
   */
  bool computeUnifiedAllocation(const Eigen::VectorXd& target_wrench_acc_cog,
                                const Eigen::VectorXd& desired_ext_wrench,
                                double yaw_pid_raw,
                                const Eigen::VectorXd& priority_wrench_acc_cog = Eigen::VectorXd(),
                                const Eigen::VectorXd& task_wrench_weights = Eigen::VectorXd(),
                                const Eigen::VectorXd& observer_feedback_wrench = Eigen::VectorXd(),
                                const Eigen::VectorXd& observer_feedback_wrench_weights = Eigen::VectorXd());

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
    qp_hessian_nnz_ = -1;
    qp_constraint_nnz_ = -1;
    qp_hessian_outer_.clear();
    qp_hessian_inner_.clear();
    qp_constraint_outer_.clear();
    qp_constraint_inner_.clear();
    prev_vectoring_f_.resize(0);
    last_qp_diag_log_time_ = -1.0;
    last_pinv_pwm_pred_pub_time_ = -1.0;
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
    // Body-fixed model. The legacy member/message name is retained for wire
    // compatibility, but inertia and rotor origins are expressed in the
    // physical baselink axes, not the attitude-dependent virtual CoG axes.
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
  int getMotorNumPerModule() const { return motor_num_per_module_; }
  bool getModuleMassInertia(int module_id, double& mass, Eigen::Matrix3d& inertia) const;
  /** @brief Return the latched leader-to-module offset in the common physical
   *  baselink/body frame. Falls back to a frame-corrected TF lookup before the
   *  formation geometry has been latched. */
  bool getModuleBodyOffsetFromLeader(int module_id, Eigen::Vector3d& offset) const;
  double getCandidateYawTerm() const { return candidate_yaw_term_; }
  Eigen::VectorXd getTargetVectoringForce() const;
  std::map<int, ModuleCommand> getModuleCommands() const;
  void setCommandTargetRPY(const tf::Vector3& rpy);
  int getModuleIndex(int module_id) const;
  bool buildModuleThrustCommand(int module_id, spinal::FourAxisCommand& thrust_msg) const;
  bool buildModuleTorqueAllocationMatrixInv(int module_id, spinal::TorqueAllocationMatrixInv& msg) const;

  /**
   * @brief Compute the allocated 6D wrench in the virtual CoG control frame.
   *
   * This is the model wrench commanded by the PC-side allocator:
   *   w_allocated_acc = A * f   (integrated_map_ * target_vectoring_f_)
   * then converted from acc-space to force/torque space:
   *   F = M * w_allocated_acc.head(3)
   *   T = I_cog * w_allocated_acc.tail(3)
   *
   * It deliberately remains a command/allocation diagnostic; it does not
   * include spinal-side P/D increments, saturation shedding, or actuator
   * tracking error and must not be used as the momentum-observer known input.
   *
   * @return 6D wrench [Fx,Fy,Fz,Tx,Ty,Tz] in virtual CoG frame (N, N·m).
   *         Returns zero vector if allocation has not been computed yet.
   */
  Eigen::VectorXd getAllocatedWrenchCog() const;
  bool getAllocatedModuleWrenchCog(int module_id, Eigen::VectorXd& allocated) const;

  /** @brief Convert one module's atomic Spinal command feedback into a wrench
   *  about that module's CoG, expressed in its virtual CoG control axes.
   *  The feedback thrust is a post-PWM-clamp command-model value, not measured
   *  rotor thrust. */
  bool computeModuleActuatorWrenchCog(
      int module_id,
      const spinal::ActuatorCommandFeedback& feedback,
      Eigen::VectorXd& wrench) const;

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
  Eigen::Vector3d formation_cog_offset_;  // physical baselink/body frame
  Eigen::Matrix3d formation_inertia_;     // physical baselink/body frame
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
  // integrated_map_ maps actuator coordinates to acceleration in the virtual
  // CoG control frame. Keep the exact body->CoG rotation paired with the map
  // so allocated-wrench diagnostics use the same frame snapshot.
  Eigen::Matrix3d allocation_cog_from_body_;
  Eigen::MatrixXd integrated_map_;        // 6 x (rotor_coef * total_rotors)
  Eigen::MatrixXd integrated_map_inv_;    // pseudoinverse
  Eigen::MatrixXd integrated_map_inv_rot_; // last 3 cols of pseudoinverse (torque part)
  Eigen::VectorXd target_vectoring_f_;    // allocation result

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
  ros::Publisher qp_pwm_pred_pub_;
  ros::Publisher pinv_pwm_pred_pub_;
  ros::Publisher qp_thrust_margin_pub_;
  ros::Publisher pinv_thrust_margin_pub_;
  ros::Subscriber battery_voltage_sub_;

  // Gimbal tracking diagnostics: QP-intended gimbal angle vs Dynamixel
  // encoder feedback (from each module's /joint_states).
  ros::Publisher gimbal_tracking_pub_;
  std::map<int, ros::Subscriber> module_joint_state_subs_;
  mutable std::mutex gimbal_meas_mutex_;
  std::map<int, std::vector<double>> module_gimbal_meas_;  // module id -> per-rotor measured angle

  // Internal methods
  Eigen::MatrixXd buildFormationAllocationMatrix(
      const std::vector<int>& assembled_ids,
      double formation_mass,
      const Eigen::Matrix3d& formation_inertia_body,
      const Eigen::Vector3d& formation_cog_offset_body,
      const Eigen::Matrix3d& cog_from_body,
      const std::vector<Eigen::MatrixXd>& masked_rot_cog);

  Eigen::Matrix3d computeFormationInertia(
      const std::vector<int>& assembled_ids,
      const Eigen::Vector3d& formation_cog_offset);

  bool getModuleModelDescriptor(int module_id, ModuleModelDescriptor& model) const;
  Eigen::Vector3d getModuleOffsetFromLeader(int module_id) const;
  bool lookupModuleOffsetFromLeader(int module_id, Eigen::Vector3d& offset) const;
  bool getCachedModuleOffsetFromLeader(int module_id, Eigen::Vector3d& offset) const;
  std::vector<Eigen::MatrixXd> buildRotorMask() const;
  bool buildModuleTorqueAllocationMatrixInvLocked(
      int module_id, spinal::TorqueAllocationMatrixInv& msg) const;
  bool sendTorqueAllocationMatrixInvLocked();

  void extractThrustAndGimbal(const Eigen::VectorXd& vectoring_f,
                              const std::vector<int>& assembled_ids);

  // Full-vector QP constrained allocation
  // Decision variables: f ∈ R^{rotor_coef * n_rotors} (all force components).
  // Constraints:
  //   - Gimbal angle:  |f_x| ≤ tan(θ_max) * f_z  (linearized)
  //   - Thrust bound:  each component within the configured/dynamic T_max, f_z ≥ 0
  bool use_constrained_alloc_;        // if true, use OsqpEigen QP; reject command update on failure
  double alloc_lambda_;               // secondary objective weight toward balanced hover reference
  double alloc_t_max_;                // per-rotor thrust upper bound [N]
  double alloc_gimbal_limit_rad_;     // gimbal angle hard limit [rad]
  double alloc_rate_weight_;          // optional smoothness weight toward previous allocation
  double alloc_rate_limit_;           // optional per-cycle component delta bound [N], <=0 disables
  double alloc_direction_rate_limit_rad_;  // optional per-cycle gimbal direction band [rad]
  double alloc_lateral_rate_weight_;  // soft penalty on 1-DOF rotor fx changes
  Eigen::VectorXd alloc_wrench_weights_;  // 6D control residual weights [Fx,Fy,Fz,Tx,Ty,Tz]
  Eigen::VectorXd alloc_task_wrench_weights_;  // 6D task residual weights [Fx,Fy,Fz,Tx,Ty,Tz]
  double alloc_effort_weight_;        // optional total-effort penalty on vectoring force
  std::vector<double> alloc_module_weights_;  // module-id indexed multiplier on alloc_lambda_
  double alloc_interface_force_limit_;   // component-wise model interface-force bound [N]
  double alloc_interface_torque_limit_;  // component-wise model interface-torque bound [Nm]
  double alloc_module_balance_weight_;    // optional soft penalty on per-module vertical-thrust spread
  bool alloc_priority_enabled_;       // hard-prioritize selected 6D wrench tracking rows
  Eigen::VectorXd alloc_priority_tolerances_;  // [Fx,Fy,Fz,Tx,Ty,Tz] acc-space bands; <=0 disables row
  bool alloc_task_priority_enabled_;  // hard-band active task rows before secondary objectives
  double alloc_task_priority_min_weight_;  // task weight threshold for hard task rows
  int qp_n_vars_;                     // number of QP variables (= rotor_coef * n_rotors), -1 = uninit
  int qp_n_constraints_;              // number of linear constraints, -1 = uninit
  int qp_hessian_nnz_;                // sparse pattern guard for safe OsqpEigen updates
  int qp_constraint_nnz_;
  std::vector<int> qp_hessian_outer_;
  std::vector<int> qp_hessian_inner_;
  std::vector<int> qp_constraint_outer_;
  std::vector<int> qp_constraint_inner_;
  std::unique_ptr<OsqpEigen::Solver> qp_solver_;
  Eigen::VectorXd prev_vectoring_f_;   // previous successful allocation, used by rate terms
  double last_qp_diag_log_time_;       // wall time of last QPDiag log emission

  /**
   * @brief Full-vector constrained QP allocation.
   *
   * Formulation:
   *   w_des = w_control + w_task + w_feedback
   *
   *   min_{f} ||W_eff^(1/2)(A*f - w_des)||^2
   *           + ρ||f - f_ref||^2
   *           + λ||f - f_ref||^2
   *           + smoothness terms
   *
   *   W_eff is a single per-axis wrench-tracking weight (alloc_wrench_weights_);
   *   task/feedback weights only select which rows are promoted to hard bands,
   *   they no longer scale the soft tracking.
   *
   *   Active task-priority rows get hard bands around w_control+w_task and
   *   KEEP their W_eff soft tracking toward w_des: the band is a guaranteed
   *   corridor, the soft term places the solution inside it (this keeps
   *   w_feedback effective on banded rows and avoids the band-edge bias).
   *   This follows the same hierarchy as spidar's static balance QPs:
   *   satisfy the task/balance rows first, then shape redundancy.
   *   s.t.  linear gimbal-angle constraints (per rotor)
   *         component bounds
   *         optional model interface-wrench bounds
   *         optional task/6D wrench priority bands
   *
   * @param alloc_matrix  Formation allocation matrix A (6 x n_cols)
   * @param w_control     Desired 6D control/stabilization wrench-acceleration vector
   * @param w_task        6D task feedforward wrench-acceleration vector
   * @param task_weights  6D task soft residual weights
   * @param w_feedback    6D low-priority observer residual-feedback wrench-acceleration vector
   * @param feedback_weights 6D low feedback residual weights
   * @param w_priority    6D wrench-acceleration vector used as the center of hard priority bands
   * @param secondary_ref Balanced/task-consistent soft allocation reference
   * @param interface_actuation_matrix D in w_I = d - D*f for all physical-chain cuts
   * @param interface_required_wrench d in w_I = d - D*f, from commanded rigid-body dynamics
   * @param vectoring_f_out  Output: full vectoring force vector (n_cols)
   * @return true on success, false on failure (caller may retry, then rejects the command update)
   */
  bool solveFullVectorQP(const Eigen::MatrixXd& alloc_matrix,
                         const Eigen::VectorXd& w_control,
                         const Eigen::VectorXd& w_task,
                         const Eigen::VectorXd& task_weights,
                         const Eigen::VectorXd& w_feedback,
                         const Eigen::VectorXd& feedback_weights,
                         const Eigen::VectorXd& w_priority,
                         const Eigen::VectorXd& secondary_ref,
                         const std::vector<int>& assembled_ids,
                         const Eigen::MatrixXd& interface_actuation_matrix,
                         const Eigen::VectorXd& interface_required_wrench,
                         Eigen::VectorXd& vectoring_f_out);

  /** @brief Build the QP linear-constraint triplets and [lb, ub] bounds:
   *  gimbal-angle, thrust polygon, component bounds, optional rate / direction
   *  / interface rows, and task/priority hard bands. Returns false if the
   *  built row count disagrees with n_constraints. Pure assembly of the
   *  pre-counted constraints; no member state is modified. */
  bool buildAllocationConstraints(const Eigen::MatrixXd& alloc_matrix,
                                  const Eigen::MatrixXd& interface_actuation_matrix,
                                  const Eigen::VectorXd& interface_required_wrench,
                                  int n_cols, int n_rotors, int n_constraints,
                                  double thrust_limit,
                                  double cos_limit, double sin_limit, int thrust_poly_edges,
                                  bool use_rate_bound, bool use_direction_rate_bound,
                                  int n_interface_rows,
                                  const std::vector<int>& task_priority_rows,
                                  const Eigen::VectorXd& task_priority_target,
                                  const std::vector<int>& priority_rows,
                                  const Eigen::VectorXd& priority_target,
                                  std::vector<Eigen::Triplet<double>>& C_trips,
                                  Eigen::VectorXd& lb, Eigen::VectorXd& ub) const;

  /** @brief Throttled post-solve QP diagnostic logging (saturation, residuals,
   *  per-rotor thrust/angle). Pure side-effect; no control impact. */
  void logQpDiagnostics(const Eigen::VectorXd& f_sol,
                        const Eigen::MatrixXd& alloc_matrix,
                        const Eigen::VectorXd& desired_tracking_target,
                        const Eigen::VectorXd& task_priority_target,
                        const std::vector<int>& task_priority_rows,
                        const Eigen::VectorXd& priority_target,
                        const std::vector<int>& priority_rows,
                        double thrust_limit,
                        int n_rotors);

  /** @brief Build a wrench-consistent secondary allocation reference.
   *
   *    f_ref = A^+ * desired_wrench_acc
   *          + (I - A^+ A) * balanced_hover_load
   *
   *  The balanced per-module hover load is expressed in actuator coordinates,
   *  so using it directly would tilt the QP reference with the physical body
   *  when the virtual CoG frame changes. Projecting it into null(A) preserves
   *  its load-sharing preference without changing a reachable requested 6D
   *  wrench. The final component guard may relax that equality only if this
   *  soft reference itself lies outside the configured actuator bounds. */
  Eigen::VectorXd buildSecondaryAllocationReference(
      const std::vector<int>& assembled_ids,
      const Eigen::VectorXd& desired_wrench_acc) const;

  /** @brief Sort allocation IDs into the physical Beetle chain (increasing
   *  leader-frame CoG x), while preserving the original ID-to-QP-column map. */
  bool getPhysicalChainOrder(const std::vector<int>& allocation_ids,
                             std::vector<int>& chain_ids,
                             std::map<int, int>& allocation_indices) const;

  /** @brief Build the affine model interface wrench for every physical cut:
   *
   *    w_I,k(f) = d_k(a*, alpha*) - D_k f.
   *
   *  D_k maps negative-x-subchain rotor forces to actuator wrench about the
   *  midpoint of the adjacent module CoGs. This subchain excludes Beetle's
   *  standard positive-x-end task contact. d_k is the wrench required by the
   *  commanded rigid-body specific acceleration, including gravity feedforward.
   *  Rows are [Fx,Fy,Fz,Tx,Ty,Tz] per cut. This is a quasi-static/commanded-
   *  dynamics model quantity, not a force-sensor measurement; unmodelled
   *  contacts on the selected subchain and angular-rate terms are excluded. */
  bool buildInterfaceLoadModel(const std::vector<int>& allocation_ids,
                               const Eigen::VectorXd& control_wrench_acc_cog,
                               const Eigen::Matrix3d& cog_from_body,
                               const std::vector<Eigen::MatrixXd>& masked_rot_cog,
                               Eigen::MatrixXd& interface_actuation_matrix,
                               Eigen::VectorXd& interface_required_wrench,
                               std::vector<std::pair<int, int>>& interface_cuts) const;
  // Pure body-frame core used by the runtime wrapper and offline tests.
  bool buildInterfaceLoadModelWithMasks(
      const std::vector<int>& allocation_ids,
      const Eigen::VectorXd& control_wrench_acc_body,
      const std::vector<Eigen::MatrixXd>& masked_rot_body,
      Eigen::MatrixXd& interface_actuation_matrix,
      Eigen::VectorXd& interface_required_wrench,
      std::vector<std::pair<int, int>>& interface_cuts) const;

  void publishInterfaceLoadDiagnostics(const Eigen::MatrixXd& interface_actuation_matrix,
                                       const Eigen::VectorXd& interface_required_wrench,
                                       const std::vector<std::pair<int, int>>& interface_cuts,
                                       const Eigen::VectorXd& vectoring_f);
  void publishAllocationPwmPredictions(const Eigen::VectorXd& qp_vectoring_f,
                                       const Eigen::VectorXd& pinv_vectoring_f,
                                       const std::vector<int>& assembled_ids);
  bool buildPwmPredictionMsg(const Eigen::VectorXd& vectoring_f,
                             const std::vector<int>& assembled_ids,
                             spinal::Pwms& msg) const;
  bool buildThrustMarginMsg(const Eigen::VectorXd& vectoring_f,
                            const std::vector<int>& assembled_ids,
                            std_msgs::Float32MultiArray& msg) const;
  void batteryVoltageCallback(const std_msgs::Float32ConstPtr& msg);
  uint16_t predictPwmFromThrust(double thrust) const;
  double convertThrustToPwmDuty(double thrust) const;
  double predictThrustLimit() const;
  double getAllocationThrustLimit() const;

  // Formation-consistent thrust ceiling. Each module publishes its own
  // voltage-derived per-rotor thrust limit (quantized to 0.25 N); every
  // module's QP then bounds itself with the min over the assembled set, so
  // all redundant allocation copies solve with the SAME actuator bounds.
  // Rationale: in the 2026-07-03 pushing test beetle1's sagging battery shrank
  // its local predictThrustLimit() to 13.5 N while beetle3 still solved with
  // 17.5 N; once the bound became active the two "formation-optimal" solutions
  // diverged and each module executed half of a different solution.
  ros::Publisher shared_thrust_limit_pub_;
  std::map<int, ros::Subscriber> peer_thrust_limit_subs_;
  mutable std::mutex shared_thrust_limit_mutex_;
  std::map<int, std::pair<double, double>> peer_shared_thrust_limits_;  // id -> (limit [N], stamp [s])
  double own_shared_thrust_limit_;     // last published own limit [N]; <=0 = not yet published
  double thrust_limit_share_timeout_;  // peer staleness warning threshold [s]; <=0 disables the warning
  void peerThrustLimitCallback(int module_id, const std_msgs::Float32ConstPtr& msg);
  void publishSharedThrustLimit();

  void moduleJointStateCallback(int module_id, const sensor_msgs::JointStateConstPtr& msg);
  void publishGimbalTracking(const std::vector<int>& assembled_ids);

  double getModuleAllocationWeight(int module_id) const;

  double pinv_pwm_pred_pub_interval_;
  double last_pinv_pwm_pred_pub_time_;
  double pinv_pwm_min_;
  double pinv_pwm_max_;
  double pinv_pwm_min_thrust_;
  int pinv_pwm_conversion_mode_;
  std::vector<spinal::MotorInfo> pinv_motor_info_;
  double battery_voltage_;
  bool battery_voltage_received_;

  void rosParamInit();
};

} // namespace aerial_robot_control
