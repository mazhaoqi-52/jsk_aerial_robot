// -*- mode: c++ -*-

#pragma once
#include <beetle/model/beetle_robot_model.h>
#include <beetle/beetle_navigation.h>
#include <beetle/TaggedWrench.h>
#include <beetle/TaggedWrenches.h>
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
    void setFfInterWrench(int id, Eigen::VectorXd des_int_wrench){ff_inter_wrench_list_[id] = des_int_wrench;}
    
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

    // UO-4: formation observer feedforward compensation (Phase UO-4)
    // Injects LPF-filtered, bias-subtracted external force estimate as slow FF
    // into the position PID. Enable only after bias is calibrated and E2 validation passes.
    bool   formation_obs_comp_enable_;   // master switch (default false)
    double formation_obs_comp_z_gain_;   // scaling factor for Z compensation [0,1]
    double formation_obs_comp_xy_gain_;      // scaling factor for X/Y compensation [0,1]
    double formation_obs_comp_torque_gain_;   // scaling factor for Roll/Pitch/Yaw torque compensation [0,1]

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

    // Roll/Pitch I-term transition support for unified mode switch:
    // Same philosophy as Z axis — freeze then boost — but with different parameters
    // because roll/pitch bias is typically smaller but more attitude-sensitive.
    int rp_integral_freeze_count_;    // frames remaining to freeze Roll/Pitch I-term
    int rp_ki_boost_count_;           // frames remaining in Roll/Pitch boost phase
    static constexpr int RP_INTEGRAL_FREEZE_FRAMES = 3;   // shorter freeze (3 frames)
    static constexpr int RP_KI_BOOST_FRAMES = 80;         // boost duration: 80 frames = 2.0s @40Hz
    static constexpr double RP_KI_BOOST_FACTOR = 6.0;     // effective Ki = nominal_Ki * 6.0 during boost phase
    double rp_i_keep_ratio_;          // fraction of old I-term to keep at switch (0~1, from YAML)

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
    // Pitch axis: SS err_i ≈ -0.54 (i_term ≈ -2.7 / Ki=5), from formation geometry offset.
    // All pitch seed values are in err_i domain (consistent with setErrI at injection).
    double last_unified_pitch_i_ss_;  // most recent unified steady-state pitch err_i
    bool has_unified_pitch_i_ss_;     // true after at least one SS sample recorded
    double pitch_i_seed_default_;     // global fallback when N not in map
    std::map<int, double> pitch_i_seed_by_n_; // per-N seed defaults from YAML
    static constexpr double PITCH_SEED_GAIN = 0.8;       // inject 80% of seed
    static constexpr double PITCH_SEED_LPF_ALPHA = 0.02; // slower LPF than Z (pitch more sensitive)
    //
    // Per-N Z boost parameters: 2-module needs stronger/longer boost than 3-module.
    int z_ki_boost_frames_;           // current N-specific boost duration
    double z_ki_boost_factor_;        // current N-specific boost factor
    int z_ki_boost_frames_default_;   // global fallback
    double z_ki_boost_factor_default_;
    std::map<int, int> z_ki_boost_frames_by_n_;
    std::map<int, double> z_ki_boost_factor_by_n_;

    bool spinal_gains_zeroed_;        // track whether we sent zero rpy/gain to spinal

    // Cascade control gains for unified mode:
    // These are TORQUE-LEVEL P/D gains sent to each module's spinal via
    // the unified_controller's sendCascadeGains(). Spinal does P+D at 1000Hz
    // using thrustGainMapping() to distribute per-motor.
    // PC retains only I-term for roll/pitch.
    double cascade_roll_p_;    // spinal roll P gain (torque-level)
    double cascade_roll_d_;    // spinal roll D gain (torque-level)
    double cascade_pitch_p_;   // spinal pitch P gain (torque-level)
    double cascade_pitch_d_;   // spinal pitch D gain (torque-level)
    double cascade_yaw_d_;     // spinal yaw D gain (torque-level)
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

    /** @brief LEADER mode switch: reset targets, migrate I-terms, zero spinal gains, apply unified gains. */
    void initUnifiedLeaderMode();

    /** @brief Switch roll/pitch PID gains for unified (formation) mode. */
    void applyUnifiedGains();
    /** @brief Restore original (per-module) roll/pitch PID gains. */
    void restoreIndependentGains();

    // Unified-mode PID gain parameters (loaded from YAML)
    struct AxisGainSet {
      double p, i, d;
      double limit_sum, limit_p, limit_i, limit_d;
    };
    AxisGainSet unified_roll_gains_, unified_pitch_gains_;
    AxisGainSet unified_xy_gains_, unified_z_gains_;  // unified-mode XY/Z gains
    AxisGainSet saved_roll_gains_, saved_pitch_gains_;  // backup of original gains
    AxisGainSet saved_xy_gains_, saved_z_gains_;        // backup of independent XY/Z gains
    bool gains_switched_;  // true when unified gains are active

    // Dynamic reconfigure servers for unified-mode XY/Z PID
    boost::shared_ptr<PidControlDynamicConfig> unified_xy_reconf_server_;
    boost::shared_ptr<PidControlDynamicConfig> unified_z_reconf_server_;
    void cfgUnifiedPidCallback(aerial_robot_control::PIDConfig &config, uint32_t level, std::vector<int> controller_indices, AxisGainSet& gain_set);

    // FOLLOWER unified mode: receive commands from LEADER
    ros::Subscriber unified_thrust_sub_;
    ros::Publisher follower_thrust_pub_;   // re-publish to own four_axes/command
    ros::Publisher follower_gimbal_pub_;   // re-publish to own gimbals_ctrl (only when !gimbal_calc_in_fc)
    spinal::FourAxisCommand unified_thrust_cmd_;
    bool unified_cmd_received_;
    bool follower_unified_active_;  // true once FOLLOWER has successfully forwarded at least one unified cmd
    ros::Time unified_cmd_stamp_;
    int follower_cmd_timeout_count_ = 0;  // consecutive frames without valid unified cmd
    // T4.1: FOLLOWER holds last command for this many frames before full fallback.
    // At 40Hz, 20 frames = 0.5s — covers short ROS communication glitches.
    static constexpr int FOLLOWER_HOLD_LAST_FRAMES = 20;

    // Freeze: cache last independent hover commands to avoid competing publish during transition
    spinal::FourAxisCommand last_independent_thrust_cmd_;
    sensor_msgs::JointState last_independent_gimbal_cmd_;
    bool has_cached_independent_cmd_;  // true once we've cached at least one frame
    void unifiedThrustCallback(const spinal::FourAxisCommand& msg);

    // FOLLOWER Ready Sync (P2.1): publish "I'm ready" once cascade gains are set
    // and the first valid unified command has been forwarded to spinal.
    ros::Publisher follower_ready_pub_;
    bool follower_ready_sent_;  // true once this FOLLOWER has published its ready signal
    
    map<string, ros::Subscriber> ff_inter_wrench_subs_;
    map<int, ros::Publisher> ff_inter_wrench_pubs_;
    map<int, ros::Publisher> desired_ext_wrench_pubs_;
    ros::Subscriber desired_ext_wrench_sub_;

    aerial_robot_msgs::PoseControlPid wrench_pid_msg_;

    map<string, ros::Subscriber> est_wrench_subs_;
    
    void estExternalWrenchCallback(const beetle::TaggedWrench & msg);

  protected:
    std::map<int, Eigen::VectorXd> est_wrench_list_;
    std::map<int, Eigen::VectorXd> inter_wrench_list_;
    std::map<int, Eigen::VectorXd> wrench_comp_list_;
    std::map<int, Eigen::VectorXd> ff_inter_wrench_list_;

    /* external wrench compensation */
    bool pd_wrench_comp_mode_;
    Eigen::VectorXd external_wrench_upper_limit_;
    Eigen::VectorXd external_wrench_lower_limit_;

    int pre_module_state_;

    bool des_wrench_pub_flag_;

    // Desired external wrench for the whole assembly (body frame)
    Eigen::VectorXd desired_external_wrench_;

    // Formation-level desired wrench for unified mode (full 6D, formation body frame).
    // Unlike desired_external_wrench_ which holds a 1/N per-module share (independent mode),
    // this holds the FULL wrench and is passed directly to computeUnifiedAllocation().
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
    ros::Publisher des_inter_wrench_pub_;

    // Assemble debug publishers (global /assemble/debug/ namespace)
    ros::Publisher assemble_pid_pub_;
    ros::Publisher assemble_vectoring_f_pub_;
    ros::Publisher assemble_formation_wrench_pub_;
    aerial_robot_msgs::PoseControlPid assemble_pid_msg_;
    void publishAssembleDebug(const tf::Vector3& formation_pos, const tf::Vector3& formation_vel,
                              const tf::Vector3& target_formation_pos, bool alloc_ok);
    void controlCore() override;
    bool update() override;
    
    virtual void ffInterWrenchCallback(const beetle::TaggedWrench & msg);
    void desiredExternalWrenchCallback(const geometry_msgs::WrenchStamped & msg);
    void formationDesiredWrenchCallback(const geometry_msgs::WrenchStamped& msg);
    void rosParamInit() override;
    void externalWrenchEstimate() override;
    void reset() override;
  };
};
