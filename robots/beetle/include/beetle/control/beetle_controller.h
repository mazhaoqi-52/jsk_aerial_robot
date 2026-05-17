// -*- mode: c++ -*-

#pragma once
#include <beetle/model/beetle_robot_model.h>
#include <beetle/beetle_navigation.h>
#include <beetle/TaggedWrench.h>
#include <beetle/UnifiedControlReference.h>
#include <gimbalrotor/control/gimbalrotor_controller.h>
#include <beetle/sensor/imu.h>
#include <beetle/control/beetle_unified_controller.h>
#include <beetle/control/formation_momentum_observer.h>
#include <std_srvs/SetBool.h>

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
    // Inject per-module observer task prediction (ŷ_i^task). New semantic:
    //   ŷ_i^task = predicted observer output for module i under the active
    //   task model (sums to W_ext across modules). Retained as the public
    //   setter for external controllers (e.g. NinjaController joint PID).
    void setTaskWrench(int id, Eigen::VectorXd y_task){est_wrench_task_list_[id] = y_task;}
    
  private:
    boost::shared_ptr<BeetleRobotModel> beetle_robot_model_;
    boost::shared_ptr<aerial_robot_navigation::BeetleNavigator> beetle_navigator_;
    
    // Unified 4N-rotor controller for assembled formation
    std::shared_ptr<BeetleUnifiedController> unified_controller_;
    bool unified_control_mode_;
    bool prev_unified_control_mode_;  // for detecting mode switch

    // Formation-level momentum observer (Phase U2)
    // Runs only in unified LEADER mode; uses realized wrench from allocation.
    // V1: 3D force estimation, debug-only (no control feedback).
    std::shared_ptr<FormationMomentumObserver> formation_observer_;

    // Formation observer feedforward (redesigned, leader-follower-style two-stage attenuation)
    //   Stage 1 = observer-internal LPF (very low cutoff, e.g. 0.05 Hz)
    //   Stage 2 = soft ramp + per-axis hard clamp + injection through PID
    //             integrator (limit_i / limit_sum naturally cap final command).
    // Default disabled; enable only after E2 known-mass validation.
    // Routing per axis:
    //   X, Y, Z, YAW       → setPersistentFF (consumed via PID.result())
    //   ROLL, PITCH        → additive direct FF on target_wrench_acc(3,4)
    //                       (PID.getITerm() + fobs_comp_ff_torque_*_)
    //                       Avoids the setICompTerm() accumulation pathology:
    //                       PID::update() does err_i_ = clamp(err_i_+errp*du)+i_comp,
    //                       so a constant i_comp written each frame grows err_i_
    //                       linearly until limit_err_i clamps. A direct additive
    //                       FF on wrench_acc has the SAME effect as one-shot i_comp
    //                       (since wrench_acc(3,4)=ROLL/PITCH.getITerm()) but is
    //                       stateless and cannot accumulate.
    bool   fobs_comp_enable_;            // master switch (default false)
    double fobs_comp_force_gain_;        // scalar gain on F_x/F_y/F_z FF acc [0,1]
    double fobs_comp_torque_gain_;       // scalar gain on Tau_x/_y/_z FF ang-acc [0,1]
    double fobs_comp_ff_force_limit_;    // hard clamp on |FF acc|     [m/s^2]
    double fobs_comp_ff_torque_limit_;   // hard clamp on |FF ang-acc| [rad/s^2]
    // Cached FF roll/pitch ang-acc per frame, added directly to wrench_acc(3,4)
    double fobs_comp_ff_torque_x_;       // [rad/s^2], leader-only, 0 when disabled
    double fobs_comp_ff_torque_y_;       // [rad/s^2], leader-only, 0 when disabled

    // Service for toggling unified control mode (replaces rosparam polling)
    ros::ServiceServer set_unified_mode_srv_;
    bool setUnifiedModeCb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
    int unified_transition_count_;    // frame counter since last mode switch (for high-freq diag)
    int z_integral_freeze_count_;     // frames remaining to freeze Z I-term after mode switch
    static constexpr int Z_INTEGRAL_FREEZE_FRAMES = 5;  // freeze Z integration for first N frames

    // Ki-boost for Z axis after mode switch (D' scheme):
    // Temporarily multiply Z integral update rate to accelerate bias convergence.
    // With seed preload, boost only needs to cover residual error → gentler settings.
    // P2: boost duration and factor are now per-N (see z_ki_boost_frames_by_n_).
    int z_ki_boost_count_;            // frames remaining in boost phase (0 = normal)

    // Roll/Pitch I-term transition support: removed.
    // Outer R/P I-term is structurally unused in unified mode (the spinal
    // cascade tracks target_roll_/target_pitch_ directly, and the formation
    // allocation absorbs the constant trim torque via cog_offset). All of
    // rp_integral_freeze / rp_ki_boost / rp_i_keep_ratio / pitch_i_seed have
    // been removed; the ROLL/PITCH PID I-accumulator is zeroed every frame
    // in runUnifiedControlCommon.

    // I-term seed for unified mode switch (Plan E'):
    // Unified mode needs steady-state I-term biases that don't exist in independent mode.
    // Instead of waiting for I-term to accumulate (→overshoot), preload estimated biases
    // at mode switch. Seeds are adaptively updated from the most recent unified-mode
    // steady state, or use configurable defaults.
    //
    // P2: Seed defaults are bucketed by module count N (2-module vs 3-module have
    // very different allocation geometry and efficiency). The maps are keyed by N;
    // if the current N is not found, falls back to the global default.
    //
    // Z axis: bias ≈ 0.86–0.94 (formation efficiency offset under explicit gravity FF)
    double last_unified_z_i_ss_;      // most recent unified steady-state Z I-term
    bool has_unified_z_i_ss_;         // true after at least one SS sample recorded
    double z_i_seed_default_;         // global fallback when N not in map
    std::map<int, double> z_i_seed_by_n_;     // per-N seed defaults from YAML
    static constexpr double Z_SEED_GAIN = 0.8;       // inject 80% of seed to be conservative
    static constexpr double Z_SEED_LPF_ALPHA = 0.05; // low-pass filter for SS tracking
    //
    // Per-N Z boost parameters: 2-module needs stronger/longer boost than 3-module.
    int z_ki_boost_frames_;           // current N-specific boost duration
    double z_ki_boost_factor_;        // current N-specific boost factor
    int z_ki_boost_frames_default_;   // global fallback
    double z_ki_boost_factor_default_;
    std::map<int, int> z_ki_boost_frames_by_n_;
    std::map<int, double> z_ki_boost_factor_by_n_;

    bool yaw_in_allocation_;   // true: yaw enters QP/allocation, false: yaw uses spinal-only channel

    /** @brief LEADER-only: send cascade gains + allocation matrix inverse to ALL assembled
     *  modules' spinals. Uses unified_controller_'s publishers. Also sends gimbal_dof=1
     *  to LEADER's own spinal. */
    void sendCascadeSetup();

    /** @brief FOLLOWER-only: send cascade gains + gimbal_dof=1 to THIS module's own spinal
     *  only (via base-class publishers). Does NOT send alloc_inv or affect other modules. */
    void sendFollowerCascadeSetup();

    /** @brief Common exit path: restore gains, reset targets, seed Z I-term, clear RP/XY. */
    void resetToIndependentHover();

    /** @brief Unified-mode switch (LEADER or FOLLOWER): reset targets, migrate I-terms,
     *  configure spinal (all modules if leader, own only if follower), apply unified gains. */
    void initUnifiedMode(bool is_leader);

    /** @brief Symmetric unified-mode control body. Both leader and follower run the full
     *  outer PID + formation allocation locally using their own estimator state. */
    void runUnifiedControlCommon(bool is_leader);

    /** @brief Switch roll/pitch PID gains for unified (formation) mode. */
    void applyUnifiedGains();
    /** @brief Restore original (per-module) roll/pitch PID gains. */
    void restoreIndependentGains();

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
    spinal::FourAxisCommand unified_thrust_cmd_;
    bool unified_cmd_received_;
    ros::Time unified_cmd_stamp_;

    // Edge-detector for one-shot takeoff diagnostic in update().
    int prev_navi_state_for_diag_;
    // Reference msg fields — kept as debug/monitoring only (leader broadcasts,
    // follower stores). NOT used for follower's control output: follower computes
    // its own wrench_acc locally in runUnifiedControlCommon().
    Eigen::VectorXd unified_reference_wrench_acc_;
    Eigen::VectorXd unified_reference_desired_wrench_;
    double unified_reference_yaw_pid_raw_;
    int unified_reference_leader_id_;
    int unified_reference_warmup_count_;
    int unified_reference_warmup_frames_;

    // Differential-mode damping gain: injects a passive dissipation term
    // -K_damp * inter_wrench/mass into target_wrench_acc. D3 architecture:
    //   diff_i = est_residual_list_[my_id] - formation_observer_wrench_/N
    // where the common-mode is taken from FormationMomentumObserver (an
    // INDEPENDENT formation-level external-wrench estimator), not from the
    // simple per-module average. The previous "residual_i - mean(residual)"
    // formula had no immunity against common-mode model error (same CoG /
    // inertia mismatch in every module's observer) and was the source of the
    // persistent diff-damping signal at hover.
    // Zero = disabled.
    double unified_diff_damp_gain_;

    // D3 common-mode source: cached output of the global formation observer
    // (published by the leader to /assemble/formation_observer/est_ext_wrench).
    // 6D in formation_body frame ([force(3); torque(3)]).
    Eigen::VectorXd formation_observer_wrench_;
    ros::Time formation_observer_wrench_stamp_;
    ros::Subscriber formation_observer_wrench_sub_;
    void formationObserverWrenchCallback(const geometry_msgs::WrenchStamped & msg);

    // PID-settled gating for FormationObserver bias calibration (leader only).
    // Bias is only calibrated when |d/dt of roll/pitch/yaw I-terms| stays below
    // bias_pid_settled_rate_thresh_ for bias_pid_settled_frames_ consecutive frames,
    // AND the state is HOVER. This prevents bias absorbing still-growing I-terms
    // (real-hardware cog model error would otherwise be "calibrated away").
    double bias_pid_settled_rate_thresh_;  // [Nm / frame] summed over R/P/Y I-terms
    int    bias_pid_settled_frames_;       // required consecutive quiet frames
    int    pid_settled_count_;             // running counter
    double last_roll_i_for_settle_;
    double last_pitch_i_for_settle_;
    double last_yaw_i_for_settle_;
    bool   pid_settle_tracker_init_;

    void unifiedReferenceCallback(const beetle::UnifiedControlReference& msg);
    void ensureUnifiedReferenceSubscription();
    bool publishLocalUnifiedCommand();
    bool publishLocalUnifiedTorqueAllocationMatrixInv();
    void publishUnifiedReference(const Eigen::VectorXd& target_wrench_acc,
                   const Eigen::VectorXd& desired_wrench,
                   double yaw_pid_raw);
    
    map<string, ros::Subscriber> est_wrench_task_subs_;
    map<int, ros::Publisher> est_wrench_task_pubs_;
    map<int, ros::Publisher> desired_ext_wrench_pubs_;
    ros::Subscriber desired_ext_wrench_sub_;

    aerial_robot_msgs::PoseControlPid wrench_pid_msg_;

    map<string, ros::Subscriber> est_wrench_subs_;
    
    void estExternalWrenchCallback(const beetle::TaggedWrench & msg);

  protected:
    std::map<int, Eigen::VectorXd> est_wrench_list_;
    // Per-module observer task prediction (ŷ_i^task). Set by demo layer via
    // /<robot>{i}/est_wrench_task topic. SEMANTIC: predicted observer output
    // under the active task model (NOT joint-on-module force). Sum invariant:
    //   Σ est_wrench_task_list_[i] = W_ext  (Newton 2nd on whole formation)
    // For uniform allocation + similar modules:
    //   est_wrench_task_list_[i] ≈ (m_i / m_total) * W_ext   for ALL i
    std::map<int, Eigen::VectorXd> est_wrench_task_list_;
    // Per-module observer residual (parasitic) = est_wrench - est_wrench_task.
    // Computed in calcInteractionWrench and consumed by the recursion so that
    // inter_wrench_list_ and wrench_comp_list_ are naturally parasitic-only.
    std::map<int, Eigen::VectorXd> est_residual_list_;
    std::map<int, Eigen::VectorXd> inter_wrench_list_;
    std::map<int, Eigen::VectorXd> wrench_comp_list_;

    /* external wrench compensation */
    bool pd_wrench_comp_mode_;
    Eigen::VectorXd external_wrench_upper_limit_;
    Eigen::VectorXd external_wrench_lower_limit_;

    int pre_module_state_;

    // Desired external wrench for the whole assembly (body frame, FULL value).
    // Set on every module from desiredExternalWrenchCallback (leader receives from
    // user, followers receive rebroadcast from leader). Used directly by
    // runUnifiedControlCommon as the formation-level task FF. The leader
    // does NOT distribute it as per-module ff_inter anymore — demo layer
    // publishes est_wrench_task per module directly.
    Eigen::VectorXd desired_external_wrench_;

    // Formation-level desired wrench for unified mode (full 6D, formation body frame),
    // alternative input path bypassing the per-module share machinery.
    Eigen::VectorXd formation_desired_wrench_;
    ros::Subscriber formation_desired_wrench_sub_;

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

    // Assemble debug publishers (global /assemble/debug/ namespace)
    ros::Publisher assemble_pid_pub_;
    ros::Publisher assemble_vectoring_f_pub_;
    ros::Publisher assemble_formation_wrench_pub_;
    aerial_robot_msgs::PoseControlPid assemble_pid_msg_;
    void publishAssembleDebug(const tf::Vector3& formation_pos, const tf::Vector3& formation_vel,
                              const tf::Vector3& target_formation_pos, bool alloc_ok);
    void controlCore() override;
    bool update() override;
    
    virtual void estWrenchTaskCallback(const beetle::TaggedWrench & msg);
    void desiredExternalWrenchCallback(const geometry_msgs::WrenchStamped & msg);
    void formationDesiredWrenchCallback(const geometry_msgs::WrenchStamped& msg);
    void rosParamInit() override;
    void externalWrenchEstimate() override;
    void reset() override;
  };
};
