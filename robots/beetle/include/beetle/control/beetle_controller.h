// -*- mode: c++ -*-

#pragma once
#include <beetle/model/beetle_robot_model.h>
#include <beetle/beetle_navigation.h>
#include <beetle/TaggedWrench.h>
#include <beetle/TaggedWrenches.h>
#include <gimbalrotor/control/gimbalrotor_controller.h>
#include <beetle/sensor/imu.h>

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
    
    // Feedforward external force compensation for valve rotation
    void setExternalForceFeedforward(const Eigen::VectorXd& ff_wrench);
    void enableValveRotationFeedforward(bool enable) { valve_rotation_ff_enabled_ = enable; }
    
  private:
    boost::shared_ptr<BeetleRobotModel> beetle_robot_model_;
    boost::shared_ptr<aerial_robot_navigation::BeetleNavigator> beetle_navigator_;
    
    map<string, ros::Subscriber> ff_inter_wrench_subs_;

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

    // Feedforward control for external forces during valve rotation
    bool valve_rotation_ff_enabled_;
    Eigen::VectorXd external_force_feedforward_;
    ros::Publisher feedforward_wrench_pub_;

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
    
    // Unified 4n-rotor control for assembled formation
    virtual void calcUnifiedRotorControl();
    
    // Helper methods for unified control
    Eigen::VectorXd distributeWrenchToRotors(const Eigen::VectorXd& target_wrench, 
                                            const std::vector<int>& assembled_ids);
    Eigen::MatrixXd buildFormationGeometryMatrix(const std::vector<int>& assembled_ids);
    void publishUnifiedRotorCommands(const Eigen::VectorXd& rotor_commands, 
                                   const std::vector<int>& assembled_ids);
    void publishFormationControlDebug(const Eigen::VectorXd& total_wrench_demand, 
                                    const std::vector<int>& assembled_ids);
    
    // Geometry helper methods for unified control
    Eigen::Vector3d getModulePosition(int module_id);
    Eigen::Vector3d getRotorPosition(int rotor_index);
    double getRotorTorqueDirection(int rotor_index);
    void publishModuleRotorCommands(int module_id, const Eigen::Vector4d& commands);
    
    ros::Publisher tagged_external_wrench_pub_;
    ros::Publisher external_wrench_compensation_pub_;
    ros::Publisher whole_external_wrench_pub_;
    ros::Publisher internal_wrench_pub_;
    ros::Publisher wrench_comp_pid_pub_;
    ros::Publisher des_inter_wrench_pub_;
    ros::Publisher formation_wrench_pub_;  // Debug publisher for formation control
    void controlCore() override;
    
    virtual void ffInterWrenchCallback(const beetle::TaggedWrench & msg);
    void rosParamInit() override;
    void externalWrenchEstimate() override;
    void reset() override;
  };
};
