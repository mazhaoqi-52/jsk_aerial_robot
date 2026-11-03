// -*- mode: c++ -*-

#pragma once

#include <gimbalrotor/model/gimbalrotor_robot_model.h>

using namespace aerial_robot_model;

enum module_state
  {
   SEPARATED,
   FOLLOWER,
   LEADER
  };

class HihRobotModel : public GimbalrotorRobotModel{
public:
  HihRobotModel(bool init_with_rosparam = true,
                    bool verbose = false,
                    double fc_t_min_thre = 0,
                    double epsilon = 10);
  virtual ~HihRobotModel() = default;

  template<class T> T getContactFrame();
  template<class T> T getCog2Cp();

  void setContactFrame(const KDL::Frame contact_frame){contact_frame_ = contact_frame;}
  void setCog2Cp(const KDL::Frame Cog2Cp){Cog2Cp_ = Cog2Cp;}

protected:
  KDL::Frame contact_frame_;
  std::mutex mutex_contact_frame_;
  KDL::Frame Cog2Cp_;
  std::mutex mutex_cog2cp_;

  void updateRobotModelImpl(const KDL::JntArray& joint_positions) override;
};

template<> inline KDL::Frame HihRobotModel::getContactFrame()
{
  std::lock_guard<std::mutex> lock(mutex_contact_frame_);
  return contact_frame_;
}

template<> inline geometry_msgs::TransformStamped HihRobotModel::getContactFrame()
{
  return aerial_robot_model::kdlToMsg(HihRobotModel::getContactFrame<KDL::Frame>());
}

template<> inline KDL::Frame HihRobotModel::getCog2Cp()
{
  std::lock_guard<std::mutex> lock(mutex_cog2cp_);
  return Cog2Cp_;
}
