// -*- mode: c++ -*-

#pragma once
#include <beetle/model/beetle_robot_model.h>
#include <beetle/beetle_navigation.h>
#include <beetle/TaggedWrench.h>
#include <beetle/TaggedWrenches.h>
#include <gimbalrotor/control/gimbalrotor_controller.h>
#include <beetle/sensor/imu.h>
#include <beetle/control/beetle_unified_controller.h>
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

    // Service for toggling unified control mode (replaces rosparam polling)
    ros::ServiceServer set_unified_mode_srv_;
    bool setUnifiedModeCb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
    int unified_transition_count_;    // frame counter since last mode switch (for high-freq diag)
    int z_integral_freeze_count_;     // frames remaining to freeze Z I-term after mode switch
    static constexpr int Z_INTEGRAL_FREEZE_FRAMES = 5;  // freeze Z integration for first N frames

    // Ki-boost for Z axis after mode switch (D' scheme):
    // Temporarily multiply Z integral update rate to accelerate bias convergence.
    // With seed preload, boost only needs to cover residual error → gentler settings.
    int z_ki_boost_count_;            // frames remaining in boost phase (0 = normal)
    static constexpr int Z_KI_BOOST_FRAMES = 60;   // boost duration: 60 frames = 1.5s @40Hz
    static constexpr double Z_KI_BOOST_FACTOR = 2.0;  // effective Ki multiplier during boost

    // Roll/Pitch I-term transition support for unified mode switch:
    // Same philosophy as Z axis — freeze then boost — but with different parameters
    // because roll/pitch bias is typically smaller but more attitude-sensitive.
    int rp_integral_freeze_count_;    // frames remaining to freeze Roll/Pitch I-term
    int rp_ki_boost_count_;           // frames remaining in Roll/Pitch boost phase
    static constexpr int RP_INTEGRAL_FREEZE_FRAMES = 3;   // shorter freeze (3 frames)
    static constexpr int RP_KI_BOOST_FRAMES = 80;         // boost duration: 80 frames = 2.0s @40Hz
    static constexpr double RP_KI_BOOST_FACTOR = 6.0;     // stronger boost to accelerate convergence with Ki=5
    double rp_i_keep_ratio_;          // fraction of old I-term to keep at switch (0~1, from YAML)

    // I-term seed for unified mode switch (Plan E'):
    // Unified mode needs steady-state I-term biases that don't exist in independent mode.
    // Instead of waiting for I-term to accumulate (→overshoot), preload estimated biases
    // at mode switch. Seeds are adaptively updated from the most recent unified-mode
    // steady state, or use configurable defaults.
    //
    // Z axis: bias ≈ 0.86–0.94 (formation efficiency offset under explicit gravity FF)
    double last_unified_z_i_ss_;      // most recent unified steady-state Z I-term
    bool has_unified_z_i_ss_;         // true after at least one SS sample recorded
    double z_i_seed_default_;         // default seed when no history (from YAML, e.g. 0.8)
    static constexpr double Z_SEED_GAIN = 0.8;       // inject 80% of seed to be conservative
    static constexpr double Z_SEED_LPF_ALPHA = 0.05; // low-pass filter for SS tracking
    //
    // Pitch axis: SS err_i ≈ -0.54 (i_term ≈ -2.7 / Ki=5), from formation geometry offset.
    // All pitch seed values are in err_i domain (consistent with setErrI at injection).
    double last_unified_pitch_i_ss_;  // most recent unified steady-state pitch err_i
    bool has_unified_pitch_i_ss_;     // true after at least one SS sample recorded
    double pitch_i_seed_default_;     // default seed when no history (from YAML, e.g. -0.55)
    static constexpr double PITCH_SEED_GAIN = 0.8;       // inject 80% of seed
    static constexpr double PITCH_SEED_LPF_ALPHA = 0.02; // slower LPF than Z (pitch more sensitive)

    bool spinal_gains_zeroed_;        // track whether we sent zero rpy/gain to spinal

    /** @brief Send all-zero rpy/gain to this module's spinal, disabling its internal attitude PID. */
    void sendZeroAttitudeGains();

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
    AxisGainSet saved_roll_gains_, saved_pitch_gains_;  // backup of original gains
    bool gains_switched_;  // true when unified gains are active

    // FOLLOWER unified mode: receive commands from LEADER
    ros::Subscriber unified_thrust_sub_;
    ros::Subscriber unified_gimbal_sub_;
    ros::Publisher follower_thrust_pub_;   // re-publish to own four_axes/command
    ros::Publisher follower_gimbal_pub_;   // re-publish to own gimbals_ctrl
    spinal::FourAxisCommand unified_thrust_cmd_;
    sensor_msgs::JointState unified_gimbal_cmd_;
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
    void unifiedGimbalCallback(const sensor_msgs::JointState& msg);
    
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
    void controlCore() override;
    bool update() override;
    
    virtual void ffInterWrenchCallback(const beetle::TaggedWrench & msg);
    void desiredExternalWrenchCallback(const geometry_msgs::WrenchStamped & msg);
    void rosParamInit() override;
    void externalWrenchEstimate() override;
    void reset() override;
  };
};
