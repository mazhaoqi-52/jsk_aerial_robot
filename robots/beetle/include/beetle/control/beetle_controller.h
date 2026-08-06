// -*- mode: c++ -*-

#pragma once
#include <beetle/model/beetle_robot_model.h>
#include <beetle/beetle_navigation.h>
#include <beetle/TaggedWrench.h>
#include <beetle/ModuleModel.h>
#include <beetle/UnifiedControlReference.h>
#include <gimbalrotor/control/gimbalrotor_controller.h>
#include <beetle/sensor/imu.h>
#include <beetle/control/beetle_unified_controller.h>
#include <beetle/control/formation_momentum_observer.h>
#include <diagnostic_msgs/KeyValue.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Float32MultiArray.h>
#include <mutex>

namespace aerial_robot_control
{
  enum
    {
     FX = YAW +1,
     FY,
     FZ,
     TX,
     TY,
     TZ,
    };

  class BeetleController: public GimbalrotorController
  {
  public:
    BeetleController();
    ~BeetleController() = default;

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                    double ctrl_loop_rate
                    ) override;
    // Inject the legacy leader-follower per-module task-wrench share. Despite
    // the historical est_wrench_task name, this is a decomposed command-side
    // wrench, not a prediction produced by the momentum observer.
    void setTaskWrench(int id, Eigen::VectorXd y_task)
    {
      std::lock_guard<std::mutex> lock(unified_wrench_state_mutex_);
      est_wrench_task_list_[id] = y_task;
    }
    
  private:
    boost::shared_ptr<BeetleRobotModel> beetle_robot_model_;
    boost::shared_ptr<aerial_robot_navigation::BeetleNavigator> beetle_navigator_;
    
    // Unified 4N-rotor controller for assembled formation
    std::shared_ptr<BeetleUnifiedController> unified_controller_;
    bool unified_control_mode_;
    bool prev_unified_control_mode_;  // for detecting mode switch

    // Formation-level momentum observer. Runs only in unified LEADER mode and
    // uses the spatial sum of per-module final-FC command feedback.
    std::shared_ptr<FormationMomentumObserver> formation_observer_;
    enum ObserverKnownWrenchSource {
      OBSERVER_WRENCH_SOURCE_UNKNOWN = -1,
      OBSERVER_WRENCH_SOURCE_UNAVAILABLE = 0,
      OBSERVER_WRENCH_SOURCE_ACTUATOR_COMMAND = 1,
      OBSERVER_WRENCH_SOURCE_LEGACY_PC_TARGET = 2,
    };
    int formation_observer_known_wrench_source_;
    int module_observer_known_wrench_source_;
    double formation_wrench_feedback_timeout_;
    ros::Publisher formation_observer_input_source_pub_;
    bool unified_external_wrench_feedback_;
    double unified_external_wrench_feedback_gain_;
    double unified_external_wrench_feedback_task_weight_;
    double unified_external_wrench_feedback_settle_pos_;
    double unified_external_wrench_feedback_settle_vel_;
    bool unified_external_wrench_feedback_bias_ready_;
    int unified_external_wrench_feedback_bias_samples_;
    Eigen::VectorXd unified_external_wrench_feedback_bias_;

    // Service for toggling unified control mode (replaces rosparam polling)
    ros::ServiceServer set_unified_mode_srv_;
    bool setUnifiedModeCb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
    int unified_transition_count_;    // frame counter since last mode switch (for high-freq diag)

    // v4 architecture — seed_bucket / z_integral_freeze / z_ki_boost machinery removed.
    // LF↔unified switching uses a pure de-gravity bumpless transfer in initUnifiedMode():
    // i_new = i_old - gravity_ff_cog. No per-N seed lookup, no integral freeze, no Ki boost.
    // Unified roll/pitch keeps the spinal as the high-rate P+D owner, while
    // the formation allocator receives only the slow PC I-term as a soft target.

    bool yaw_in_allocation_;   // true: yaw enters QP/allocation, false: yaw uses spinal-only channel
    bool unified_towing_debug_log_;
    double unified_towing_debug_log_period_;
    bool unified_command_stall_debug_;
    double unified_command_stall_warn_gap_;
    double unified_command_stall_trace_gap_;
    double unified_command_stall_trace_duration_;
    const char* unified_debug_stage_;
    double unified_debug_stage_time_;
    double unified_debug_cycle_start_time_;
    double unified_debug_trace_until_time_;
    double unified_heartbeat_pub_interval_;
    double last_unified_heartbeat_pub_time_;
    /** @brief LEADER-only: send cascade gains + allocation matrix inverse to this
     *  module's spinal. Uses unified_controller_'s publishers. Also sends gimbal_dof=1
     *  to LEADER's own spinal. */
    void sendCascadeSetup();

    /** @brief FOLLOWER-only: set gimbal_dof=1 on THIS module's own spinal.
     *  Called by the local one-shot after alloc_inv is ready. */
    void sendFollowerCascadeSetup();
    void sendFollowerCascadeGains();

    /** @brief Common exit path: restore gains, reset targets, seed Z I-term, clear RP/XY. */
    void resetToIndependentHover();

    /** @brief Unified-mode switch (LEADER or FOLLOWER): reset targets, migrate I-terms,
     *  configure this module's spinal when ready, apply unified gains. */
    void initUnifiedMode(bool is_leader);
    void updateFormationObserverKnownWrenchSource(bool actuator_command_ready);
    void actuatorCommandFeedbackCallback(
      const spinal::ActuatorCommandFeedback::ConstPtr& msg);
    void actuatorCommandWrenchCallback(
      const geometry_msgs::WrenchStamped::ConstPtr& msg, int module_id);
    bool computeFormationSpatialWrenchSum(
      const std::map<int, Eigen::VectorXd>& module_wrenches,
      const std::map<int, double>& receipt_stamps,
      const std::map<int, std::string>& frame_ids,
      Eigen::VectorXd& formation_wrench,
      std::string& status) const;
    void publishSpatialWrench(
      ros::Publisher& publisher, const Eigen::VectorXd& wrench) const;
    void publishSpatialSumStatus(
      ros::Publisher& publisher, std::string& previous_status,
      const std::string& key, const std::string& status);

    /** @brief Symmetric unified-mode control body. Both leader and follower run the full
     *  outer PID + formation allocation locally using their own estimator state. */
    void runUnifiedControlCommon(bool is_leader);
    bool computeFormationRpError(const tf::Vector3& target_rpy,
                                 tf::Vector3& formation_rpy,
                                 tf::Vector3& formation_rp_error,
                                 Eigen::Vector2d& rp_weight_sum);
    void markUnifiedDebugStage(const char* stage);
    void reportUnifiedCommandGap(const char* event, double gap, double now);
    void publishUnifiedHeartbeat(const char* event, double now, bool force = false);

    /** @brief Switch roll/pitch PID gains for unified (formation) mode. */
    void applyUnifiedGains();
    /** @brief Restore original (per-module) roll/pitch PID gains. */
    void restoreIndependentGains();
    void clearInternalWrenchState();

    // Unified-mode PID gain parameters (loaded from YAML)
    struct AxisGainSet {
      double p, i, d;
      double limit_sum, limit_p, limit_i, limit_d;
      double err_d_lpf_cutoff_freq;  // D-term velocity error LPF cutoff [Hz], 0=disabled
    };
    AxisGainSet unified_roll_gains_, unified_pitch_gains_;
    AxisGainSet unified_xy_gains_, unified_z_gains_;  // unified-mode XY/Z gains
    AxisGainSet unified_yaw_gains_;                     // unified-mode yaw gains
    AxisGainSet saved_roll_gains_, saved_pitch_gains_;  // backup of original gains
    AxisGainSet saved_xy_gains_, saved_z_gains_;        // backup of independent XY/Z gains
    AxisGainSet saved_yaw_gains_;                       // backup of independent yaw gains
    bool gains_switched_;  // true when unified gains are active

    // Dynamic reconfigure servers for unified-mode XY/Z/Yaw PID
    boost::shared_ptr<PidControlDynamicConfig> unified_xy_reconf_server_;
    boost::shared_ptr<PidControlDynamicConfig> unified_z_reconf_server_;
    boost::shared_ptr<PidControlDynamicConfig> unified_yaw_reconf_server_;
    void cfgUnifiedPidCallback(aerial_robot_control::PIDConfig &config, uint32_t level, std::vector<int> controller_indices, AxisGainSet& gain_set);

    // Unified reference broadcast: leader publishes a formation-level reference;
    // each module computes allocation locally and picks its own block.
    ros::Publisher unified_reference_pub_;
    ros::Subscriber unified_reference_sub_;
    ros::Publisher follower_thrust_pub_;   // re-publish to own four_axes/command
    ros::Publisher follower_gimbal_pub_;   // re-publish to own gimbals_ctrl (only when !gimbal_calc_in_fc)
    ros::Publisher module_model_pub_;
    ros::Publisher unified_heartbeat_pub_;
    map<string, ros::Subscriber> module_model_subs_;
    map<int, ros::Subscriber> unified_peer_heartbeat_subs_;
    std::map<int, double> unified_peer_heartbeat_stamp_;
    std::mutex unified_peer_heartbeat_mutex_;
    bool unified_peer_stall_guard_;
    double unified_peer_heartbeat_timeout_;
    // Last formation revision seen in runUnifiedControlCommon. When the unified
    // controller's getFormationRevision() bumps (peer ModuleModel late-arrival
    // or assembled-id change), the follower local cascade one-shot is re-armed
    // so the spinal does not keep a stale torque_alloc_inv.
    uint64_t prev_formation_revision_ = 0;
    spinal::FourAxisCommand unified_thrust_cmd_;
    bool unified_cmd_received_;
    ros::Time unified_cmd_stamp_;

    // Edge-detector for one-shot takeoff diagnostic in update().
    int prev_navi_state_for_diag_;
    // Reference msg fields cached on the follower side. Guarded by
    // unified_reference_mutex_ so callbacks and the control loop see a
    // physically consistent reference snapshot.
    // wrench_acc / desired_wrench / observer_feedback_wrench / yaw_pid_raw :
    //   leader low-frequency reference and diagnostics. Followers still solve
    //   QP locally; desired_wrench is the explicit formation-level task and
    //   observer_feedback_wrench is a separate low-weight residual task.
    // desired_wrench_weights : QP task weights paired with desired_wrench.
    // leader_target_* / final_target_baselink  : Phase B — drives the
    //   follower's target_pos/_vel/_acc/_rpy/_omega/_ang_acc via rigid-formation
    //   kinematics inside runUnifiedControlCommon() while keeping PID attitude
    //   and physical baselink tilt separate.
    Eigen::VectorXd unified_reference_wrench_acc_;
    Eigen::VectorXd unified_reference_desired_wrench_;
    Eigen::VectorXd unified_reference_desired_wrench_weights_;
    Eigen::VectorXd unified_reference_observer_feedback_wrench_;
    Eigen::VectorXd unified_reference_observer_feedback_wrench_weights_;
    double unified_reference_yaw_pid_raw_;
    int unified_reference_leader_id_;
    int unified_reference_warmup_count_;
    int unified_reference_warmup_frames_;
    double unified_reference_timeout_;
    bool local_unified_cascade_setup_sent_;
    double unified_torque_alloc_inv_pub_interval_;
    double last_unified_torque_alloc_inv_pub_time_;
    double last_unified_command_pub_time_;
    tf::Vector3 leader_target_pos_;
    tf::Vector3 leader_target_vel_;
    tf::Vector3 leader_target_acc_;
    tf::Vector3 leader_target_rpy_;
    tf::Vector3 leader_final_target_baselink_rpy_;
    tf::Vector3 leader_target_omega_;
    tf::Vector3 leader_target_ang_acc_;
    std::mutex unified_reference_mutex_;

    void unifiedReferenceCallback(const beetle::UnifiedControlReference& msg);
    void unifiedPeerHeartbeatCallback(const diagnostic_msgs::KeyValue::ConstPtr& msg, int module_id);
    void ensureUnifiedReferenceSubscription();
    bool publishLocalUnifiedCommand();
    bool sendLocalUnifiedCascadeSetupOnce();
    bool publishLocalUnifiedTorqueAllocationMatrixInv();
    void publishUnifiedReference(const Eigen::VectorXd& target_wrench_acc,
                   const Eigen::VectorXd& desired_wrench,
                   const Eigen::VectorXd& desired_wrench_weights,
                   const Eigen::VectorXd& observer_feedback_wrench,
                   const Eigen::VectorXd& observer_feedback_wrench_weights,
                   double yaw_pid_raw);
    void publishModuleModel();
    void moduleModelCallback(const beetle::ModuleModel& msg);
    
    map<string, ros::Subscriber> est_wrench_task_subs_;
    map<int, ros::Publisher> est_wrench_task_pubs_;
    map<int, ros::Publisher> desired_ext_wrench_pubs_;
    map<int, ros::Publisher> desired_ext_wrench_weights_pubs_;
    ros::Subscriber desired_ext_wrench_sub_;
    ros::Subscriber desired_ext_wrench_weights_sub_;

    aerial_robot_msgs::PoseControlPid wrench_pid_msg_;

    map<string, ros::Subscriber> est_wrench_subs_;
    map<int, ros::Subscriber> actuator_command_wrench_subs_;
    ros::Subscriber actuator_command_feedback_sub_;
    ros::Publisher actuator_command_wrench_pub_;
    ros::Publisher module_observer_spatial_sum_pub_;
    ros::Publisher actuator_command_spatial_sum_pub_;
    ros::Publisher module_observer_spatial_sum_status_pub_;
    ros::Publisher actuator_command_spatial_sum_status_pub_;
    std::string module_observer_spatial_sum_status_;
    std::string actuator_command_spatial_sum_status_;
    
    void estExternalWrenchCallback(const beetle::TaggedWrench & msg);

  protected:
    std::map<int, Eigen::VectorXd> est_wrench_list_;
    std::map<int, double> est_wrench_receipt_stamps_;
    std::map<int, std::string> est_wrench_frame_ids_;
    std::map<int, Eigen::VectorXd> actuator_command_wrench_list_;
    std::map<int, double> actuator_command_wrench_receipt_stamps_;
    std::map<int, std::string> actuator_command_wrench_frame_ids_;
    // Legacy leader-follower task-wrench shares, set by the demo layer through
    // /<robot>{i}/est_wrench_task. These are command-side mass/inertia shares;
    // they are not momentum-observer outputs and are not consumed by unified
    // control.
    std::map<int, Eigen::VectorXd> est_wrench_task_list_;
    // Legacy leader-follower residual used only by calcInteractionWrench().
    std::map<int, Eigen::VectorXd> est_residual_list_;
    std::map<int, Eigen::VectorXd> inter_wrench_list_;
    std::map<int, Eigen::VectorXd> wrench_comp_list_;
    std::mutex unified_wrench_state_mutex_;

    /* external wrench compensation */
    bool pd_wrench_comp_mode_;
    Eigen::VectorXd external_wrench_upper_limit_;
    Eigen::VectorXd external_wrench_lower_limit_;

    int pre_module_state_;

    // Formation-level desired wrench for unified mode (full 6D, formation body frame),
    // direct input to BeetleUnifiedController::computeUnifiedAllocation().
    // The legacy desired_external_wrench topic is treated as an alias and is
    // stored here too; unified mode does not inject task wrench via PID FF.
    Eigen::VectorXd formation_desired_wrench_;
    Eigen::VectorXd formation_desired_wrench_weights_;
    double formation_desired_wrench_timestamp_;
    double formation_desired_wrench_weights_timestamp_;
    // Dedicated independent-UAV task wrench. It is deliberately isolated from
    // formation_desired_wrench_ so an assembly role transition cannot reinterpret
    // a formation command as a full single-module command.
    Eigen::VectorXd single_desired_wrench_;
    double single_desired_wrench_timestamp_;
    double desired_wrench_timeout_;
    ros::Subscriber formation_desired_wrench_sub_;
    ros::Subscriber formation_desired_wrench_weights_sub_;
    ros::Subscriber single_desired_wrench_sub_;

    double comp_term_update_freq_;
    double prev_comp_update_time_;
    double wrench_comp_p_gain_;
    double wrench_comp_i_gain_;
    double wrench_comp_d_gain_;
    double I_comp_Fx_;
    double I_comp_Fy_;
    double I_comp_Fz_;
    double I_comp_Tx_;
    double I_comp_Ty_;
    double I_comp_Tz_;

    virtual void calcInteractionWrench();
    
    ros::Publisher tagged_external_wrench_pub_;
    ros::Publisher external_wrench_compensation_pub_;
    ros::Publisher whole_external_wrench_pub_;
    ros::Publisher internal_wrench_pub_;
    ros::Publisher wrench_comp_pid_pub_;
    // [Step D'] Leader-only diagnostic: pairwise disagreement of the
    // per-module observer-derived inter-wrenches. Float32MultiArray:
    // [max_force_norm, max_torque_norm, rms_force_norm, rms_torque_norm].
    // Published only when the running controller instance is the leader.
    // PURE DIAGNOSTIC — no control feedback.
    ros::Publisher inter_disagreement_pub_;
    // Leader-only diagnostic: mass/CoG model error derived from the hover
    // external-wrench bias (gravity-only assumption). Float32MultiArray:
    // [delta_mass_kg, cog_offset_x_m, cog_offset_y_m,
    //  resid_fx_N, resid_fy_N, resid_tz_Nm, model_mass_kg].
    // PURE DIAGNOSTIC — no control feedback.
    ros::Publisher model_error_estimate_pub_;

    // Assemble debug publishers (global /assemble/debug/ namespace)
    ros::Publisher assemble_pid_pub_;
    ros::Publisher assemble_vectoring_f_pub_;
    ros::Publisher assemble_formation_wrench_pub_;
    aerial_robot_msgs::PoseControlPid assemble_pid_msg_;
    void publishAssembleDebug(const tf::Vector3& formation_pos, const tf::Vector3& formation_vel,
                              const tf::Vector3& formation_rpy,
                              const tf::Vector3& formation_rp_error,
                              const tf::Vector3& target_formation_pos, bool alloc_ok,
                              const Eigen::VectorXd& target_wrench_acc,
                              const Eigen::VectorXd& formation_wrench_cmd,
                              double yaw_pid_raw);
    void logFollowerAllocationImbalance(const Eigen::VectorXd& target_wrench_acc,
                                        double alloc_ms,
                                        double since_pub,
                                        bool alloc_ok);
    void controlCore() override;
    bool update() override;
    
    virtual void estWrenchTaskCallback(const beetle::TaggedWrench & msg);
    void singleDesiredExternalWrenchCallback(
      const geometry_msgs::WrenchStamped & msg);
    void desiredExternalWrenchCallback(const geometry_msgs::WrenchStamped & msg);
    void desiredExternalWrenchWeightsCallback(const std_msgs::Float32MultiArray & msg);
    void formationDesiredWrenchCallback(const geometry_msgs::WrenchStamped& msg);
    void formationDesiredWrenchWeightsCallback(const std_msgs::Float32MultiArray& msg);
    void rosParamInit() override;
    void externalWrenchEstimate() override;
    void reset() override;
  };
};
