#pragma once

#include <aerial_robot_control/flight_navigation.h>

namespace aerial_robot_control
{
// Optional cross-cast interface. Does not change ControlBase/PoseLinearController
// layout or vtables. Existing controllers do not implement it and are unaffected.
class TargetReferenceModifier
{
public:
  virtual ~TargetReferenceModifier() = default;
  virtual void modifyTargetSnapshot(aerial_robot_navigation::BaseNavigator::TargetSnapshot& target) = 0;
};
}  // namespace aerial_robot_control
