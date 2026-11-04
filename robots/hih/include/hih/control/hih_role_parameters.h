#pragma once
#include <ros/ros.h>
#include <stdexcept>

namespace hih_control
{
// Apply role overrides BEFORE the inherited PID/MOB initialization reads them.
// Unspecified sections continue to come from the unchanged HihControl.yaml.
inline void applyRoleParameters(ros::NodeHandle nh, const std::string& role)
{
  XmlRpc::XmlRpcValue controller;
  const std::string key = "hih/" + role + "/controller";
  if (!nh.getParam(key, controller) || controller.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    throw std::invalid_argument("Missing role controller settings: " + nh.resolveName(key));
  for (auto it = controller.begin(); it != controller.end(); ++it)
    nh.setParam("controller/" + it->first, it->second);
  nh.setParam("hih_role", role);
}
}  // namespace hih_control
