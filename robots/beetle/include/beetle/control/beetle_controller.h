// -*- mode: c++ -*-

#pragma once
#include <beetle/model/beetle_robot_model.h>
#include <beetle/beetle_navigation.h>
#include <beetle/TaggedWrench.h>
#include <beetle/TaggedWrenches.h>
#include <gimbalrotor/control/gimbalrotor_controller.h>
#include <beetle/sensor/imu.h>
#include <beetle/control/beetle_unified_controller.h>
#include <spinal/TorqueAllocationMatrixInv.h>
#include <spinal/RollPitchYawTerms.h>
#include <spinal/DesireCoord.h>
#include <std_msgs/UInt8.h>

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
    bool spinal_gains_zeroed_;        // H2: track whether we sent zero rpy/gain to spinal
    int unified_transition_count_;    // frame counter since last mode switch (for high-freq diag)

    // FOLLOWER unified mode: receive commands from LEADER
    ros::Subscriber unified_thrust_sub_;
    ros::Subscriber unified_torque_alloc_sub_;
    ros::Subscriber unified_rpy_gain_sub_;
    ros::Subscriber unified_desire_coord_sub_;
    ros::Subscriber unified_gimbal_dof_sub_;

    // Publishers to own spinal for forwarding unified commands
    ros::Publisher follower_thrust_pub_;         // → four_axes/command
    ros::Publisher follower_torque_alloc_pub_;    // → torque_allocation_matrix_inv
    ros::Publisher follower_rpy_gain_pub_;        // → rpy/gain
    ros::Publisher follower_desire_coord_pub_;    // → desire_coordinate
    ros::Publisher follower_gimbal_dof_pub_;      // → gimbal_dof

    // Cached unified messages from LEADER (for forwarding to own spinal)
    spinal::FourAxisCommand unified_thrust_cmd_;
    spinal::TorqueAllocationMatrixInv unified_torque_alloc_cmd_;
    spinal::RollPitchYawTerms unified_rpy_gain_cmd_;
    spinal::DesireCoord unified_desire_coord_cmd_;
    std_msgs::UInt8 unified_gimbal_dof_cmd_;
    bool unified_cmd_received_;
    bool unified_torque_alloc_received_;
    bool unified_rpy_gain_received_;
    bool unified_desire_coord_received_;
    bool unified_gimbal_dof_received_;
    bool follower_unified_active_;  // true once FOLLOWER has successfully forwarded at least one unified cmd
    ros::Time unified_cmd_stamp_;

    // Callbacks for unified topics from LEADER
    void unifiedThrustCallback(const spinal::FourAxisCommand& msg);
    void unifiedTorqueAllocCallback(const spinal::TorqueAllocationMatrixInv& msg);
    void unifiedRpyGainCallback(const spinal::RollPitchYawTerms& msg);
    void unifiedDesireCoordCallback(const spinal::DesireCoord& msg);
    void unifiedGimbalDofCallback(const std_msgs::UInt8& msg);
    
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
