#pragma once
#include <hih/control/hih_admittance.h>
#include <tf/LinearMath/Transform.h>

namespace hih_control
{
struct ContactWrench { tf::Vector3 force, torque; };

inline ContactWrench wrenchAtContact(const tf::Vector3& force_world,
  const tf::Vector3& torque_cog, const tf::Matrix3x3& world_from_cog,
  const tf::Transform& cog_from_contact)
{
  const tf::Vector3 force_cog = world_from_cog.inverse() * force_world;
  const tf::Matrix3x3 contact_from_cog = cog_from_contact.getBasis().inverse();
  return {contact_from_cog * force_cog,
    contact_from_cog * (torque_cog - cog_from_contact.getOrigin().cross(force_cog))};
}

inline Vector4 contactWrenchErrorWorld(const ContactWrench& measured,
  const tf::Matrix3x3& world_from_contact, const tf::Vector3& normal_contact, double normal_reference)
{
  const tf::Vector3 error_force_world = world_from_contact *
    (measured.force + normal_reference * normal_contact);
  const tf::Vector3 error_torque_world = world_from_contact * measured.torque;
  return {{error_force_world.x(), error_force_world.y(), error_force_world.z(), error_torque_world.z()}};
}

inline tf::Vector3 contactCenterCompensation(const tf::Matrix3x3& nominal,
  const tf::Matrix3x3& command, const tf::Vector3& contact_offset_cog)
{ return nominal * contact_offset_cog - command * contact_offset_cog; }
}
