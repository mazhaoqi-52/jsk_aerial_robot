#include <hih/control/hih_cop_adjustment.h>
#include <angles/angles.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace
{
bool finite(const tf::Vector3& v)
{ return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z()); }

template<class T> void read(ros::NodeHandle nh, const std::string& key, T& value)
{
  if (nh.hasParam(key) && !nh.getParam(key, value))
    throw std::invalid_argument("Invalid CoP parameter type: " + nh.resolveName(key));
}

void readBias(ros::NodeHandle nh, const std::string& key, tf::Vector3& value)
{
  if (!nh.hasParam(key)) return;
  std::vector<double> v;
  if (!nh.getParam(key, v) || v.size() != 3)
    throw std::invalid_argument("Expected three CoP bias components: " + nh.resolveName(key));
  value.setValue(v[0], v[1], v[2]);
}

void readAxis(ros::NodeHandle nh, hih_control::CopAxisParameters& axis)
{
  read(nh, "enabled", axis.enabled); read(nh, "gain", axis.gain);
  read(nh, "direction", axis.direction); read(nh, "max_offset", axis.max_offset);
  read(nh, "max_rate", axis.max_rate);
}

void validateAxis(const hih_control::CopAxisParameters& axis)
{
  for (double x : {axis.gain, axis.direction, axis.max_offset, axis.max_rate})
    if (!std::isfinite(x)) throw std::invalid_argument("CoP axis parameters must be finite");
  if (axis.gain < 0 || std::abs(std::abs(axis.direction) - 1.0) > 1e-12 ||
      axis.max_offset <= 0 || axis.max_rate <= 0)
    throw std::invalid_argument("CoP needs gain>=0, direction=+/-1 and positive offset/rate limits");
}

double boundaryError(double cop, double safe, double hysteresis, bool& active)
{
  active = active ? std::abs(cop) >= safe - hysteresis : std::abs(cop) > safe;
  // The latch may stay active just inside the boundary, but the rate is ZERO there.
  return active ? cop - std::clamp(cop, -safe, safe) : 0.0;
}

void advanceAxis(double error, const hih_control::CopAxisParameters& p,
                 bool enabled, bool track, double dt, double& offset, double& actual_rate)
{
  actual_rate = 0;
  const double previous = offset;
  if (!enabled)
  {
    const double step = p.max_rate * dt;
    offset = offset > 0 ? std::max(0.0, offset - step) : std::min(0.0, offset + step);
  }
  else if (track)
  {
    const double rate = error == 0 || p.gain == 0 ? 0 :
      std::clamp(p.direction * p.gain * error, -p.max_rate, p.max_rate);
    // Clamp the stored state itself: no hidden windup, immediate release on reversal.
    offset = std::clamp(offset + rate * dt, -p.max_offset, p.max_offset);
  }
  actual_rate = (offset - previous) / dt;
}
}  // namespace

namespace hih_control
{
void CopAdjustmentParameters::validate() const
{
  validateAxis(yaw); validateAxis(z);
  for (double x : {safe_y, safe_z, hysteresis_y, hysteresis_z, min_normal_force,
                  max_abs_cop_y, max_abs_cop_z, filter_cutoff_hz,
                  sample_timeout, max_dt, debug_period})
    if (!std::isfinite(x)) throw std::invalid_argument("CoP parameters must be finite");
  if (safe_y <= 0 || safe_z <= 0 || hysteresis_y < 0 || hysteresis_z < 0 ||
      hysteresis_y >= safe_y || hysteresis_z >= safe_z || min_normal_force <= 0 ||
      max_abs_cop_y < safe_y || max_abs_cop_z < safe_z || filter_cutoff_hz < 0 ||
      sample_timeout <= 0 || max_dt <= 0 || debug_period <= 0 || yaw.max_offset >= 3.14159265358979323846 ||
      !finite(force_bias_contact) || !finite(torque_bias_contact))
    throw std::invalid_argument("Invalid CoP boundary, force threshold, timing, bias or yaw limit");
}

HihCopAdjustment::HihCopAdjustment() { configure(CopAdjustmentParameters()); }

void HihCopAdjustment::configure(const CopAdjustmentParameters& parameters,
                                const tf::Vector3& normal_contact)
{
  parameters.validate();
  const bool supported = finite(normal_contact) &&
    std::abs(std::abs(normal_contact.x()) - 1) < 1e-6 &&
    std::abs(normal_contact.y()) < 1e-6 && std::abs(normal_contact.z()) < 1e-6;
  if (parameters.enabled && !supported)
    throw std::invalid_argument("HiH CoP adjustment requires contact-frame normal_contact=[+/-1,0,0]");
  p_ = parameters;
  supported_normal_ = supported;
  normal_sign_ = supported ? normal_contact.x() : 1;
  reset();
}

void HihCopAdjustment::initialize(ros::NodeHandle nh, const tf::Vector3& normal_contact,
                                 const std::string& world_frame)
{
  settings_ = std::make_unique<ros::NodeHandle>(nh, "controller/cop_adjustment");
  CopAdjustmentParameters p;
  read(*settings_, "enabled", p.enabled);
  readAxis(ros::NodeHandle(*settings_, "yaw"), p.yaw);
  readAxis(ros::NodeHandle(*settings_, "z"), p.z);
  read(*settings_, "safe_y", p.safe_y); read(*settings_, "safe_z", p.safe_z);
  read(*settings_, "hysteresis_y", p.hysteresis_y); read(*settings_, "hysteresis_z", p.hysteresis_z);
  read(*settings_, "min_normal_force", p.min_normal_force);
  read(*settings_, "max_abs_cop_y", p.max_abs_cop_y); read(*settings_, "max_abs_cop_z", p.max_abs_cop_z);
  read(*settings_, "filter_cutoff_hz", p.filter_cutoff_hz);
  read(*settings_, "sample_timeout", p.sample_timeout); read(*settings_, "max_dt", p.max_dt);
  read(*settings_, "debug_period", p.debug_period);
  readBias(*settings_, "force_bias_contact", p.force_bias_contact);
  readBias(*settings_, "torque_bias_contact", p.torque_bias_contact);
  configure(p, normal_contact);
  world_frame_ = world_frame;
  contact_frame_ = nh.getNamespace();
  if (!contact_frame_.empty() && contact_frame_.front() == '/') contact_frame_.erase(0, 1);
  if (!contact_frame_.empty()) contact_frame_ += "/";
  contact_frame_ += "contact_point";
  bool observer = false;
  nh.param("controller/wrench_estimate_flag", observer, false);
  if (p.enabled && !observer)
    throw std::invalid_argument("CoP adjustment requires the existing MOB: wrench_estimate_flag=true");

  ros::NodeHandle debug(nh, "hih/cop");
  wrench_pub_ = debug.advertise<geometry_msgs::WrenchStamped>("wrench_contact", 1);
  raw_pub_ = debug.advertise<geometry_msgs::PointStamped>("raw", 1);
  filtered_pub_ = debug.advertise<geometry_msgs::PointStamped>("filtered", 1);
  error_pub_ = debug.advertise<geometry_msgs::Vector3Stamped>("boundary_error", 1);
  normal_force_pub_ = debug.advertise<std_msgs::Float64>("normal_force", 1);
  valid_pub_ = debug.advertise<std_msgs::Bool>("valid", 1);
  applying_pub_ = debug.advertise<std_msgs::Bool>("applying", 1);
  status_pub_ = debug.advertise<std_msgs::String>("status", 1);
  yaw_saturated_pub_ = debug.advertise<std_msgs::Bool>("yaw_saturated", 1);
  z_saturated_pub_ = debug.advertise<std_msgs::Bool>("z_saturated", 1);
  yaw_offset_pub_ = debug.advertise<std_msgs::Float64>("yaw_offset", 1);
  yaw_rate_pub_ = debug.advertise<std_msgs::Float64>("yaw_rate", 1);
  z_offset_pub_ = debug.advertise<std_msgs::Float64>("z_offset", 1);
  z_velocity_pub_ = debug.advertise<std_msgs::Float64>("z_velocity", 1);
  admittance_position_pub_ = debug.advertise<geometry_msgs::PointStamped>("admittance_position", 1);
  command_position_pub_ = debug.advertise<geometry_msgs::PointStamped>("command_position", 1);
  yaw_reference_pub_ = debug.advertise<geometry_msgs::Vector3Stamped>("yaw_reference", 1);
  ROS_INFO("[HiH CoP] MOB-only; yaw=%s Z=%s, master=%s; safe Y/Z = +/-%.3f/%.3f m",
    p.yaw.enabled ? "ON" : "OFF", p.z.enabled ? "ON" : "OFF", p.enabled ? "ON" : "OFF", p.safe_y, p.safe_z);
}

void HihCopAdjustment::resetInput()
{
  filter_initialized_ = false;
  filter_stamp_ = 0;
  state_.error.setZero();
  state_.yaw_boundary_active = state_.z_boundary_active = false;
}

void HihCopAdjustment::reset()
{
  state_ = CopAdjustmentState();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  state_.raw.setValue(0, nan, nan); state_.filtered = state_.raw;
  resetInput();
  newest_sample_stamp_ = last_control_time_ = 0;
  last_debug_ = ros::Time(0);
}

void HihCopAdjustment::setEnabled(bool master, bool yaw, bool z)
{ p_.enabled = master; p_.yaw.enabled = yaw; p_.z.enabled = z; }

const CopAdjustmentState& HihCopAdjustment::state() const { return state_; }

const CopAdjustmentState& HihCopAdjustment::step(const tf::Vector3& force_contact,
  const tf::Vector3& torque_contact, double sample_stamp, bool sample_valid,
  double now, double dt, bool flight_allowed, bool admittance_applying)
{
  state_.yaw_rate = state_.z_velocity = 0;
  state_.valid = false;
  state_.error.setZero();
  state_.force_contact = force_contact - p_.force_bias_contact;
  state_.torque_contact = torque_contact - p_.torque_bias_contact;
  state_.normal_force = -normal_sign_ * state_.force_contact.x();  // Positive compression for either X-facing plate.
  const double nan = std::numeric_limits<double>::quiet_NaN();
  state_.raw.setValue(0, nan, nan);
  const bool valid_dt = std::isfinite(now) && std::isfinite(dt) && dt > 0 && dt <= p_.max_dt;
  if (!valid_dt) { resetInput(); newest_sample_stamp_ = 0; }

  std::string input_status;
  const double age = now - sample_stamp;
  if (!supported_normal_) input_status = "unsupported_contact_normal";
  else if (!sample_valid || !finite(state_.force_contact) || !finite(state_.torque_contact))
    input_status = "invalid_wrench_held";
  else if (!std::isfinite(now) || !std::isfinite(sample_stamp) || sample_stamp <= 0 ||
           !std::isfinite(age) || age < 0 || age > p_.sample_timeout)
    input_status = "stale_input_held";
  else if (sample_stamp < newest_sample_stamp_) input_status = "out_of_order_input_held";
  else if (state_.normal_force < p_.min_normal_force) input_status = "low_normal_force_held";
  else
  {
    // Plane x_C=0. Full MOB torque about contact center, BEFORE 4-DOF projection.
    // tau = r x F + torsion*n -> y=-tau_z/F_x, z=tau_y/F_x.
    state_.raw.setValue(0, -state_.torque_contact.z() / state_.force_contact.x(),
                            state_.torque_contact.y() / state_.force_contact.x());
    if (!finite(state_.raw) || std::abs(state_.raw.y()) > p_.max_abs_cop_y ||
        std::abs(state_.raw.z()) > p_.max_abs_cop_z)
      input_status = "cop_outlier_held";
    else state_.valid = true;
  }

  if (!state_.valid) resetInput();
  else
  {
    const double sample_dt = sample_stamp - filter_stamp_;
    if (!filter_initialized_ || sample_dt > p_.sample_timeout)
    {
      state_.filtered = state_.raw;
      filter_initialized_ = true;
      filter_stamp_ = sample_stamp;
    }
    else if (sample_dt > 0)
    {
      const double alpha = p_.filter_cutoff_hz == 0 ? 1.0 :
        -std::expm1(-2.0 * 3.14159265358979323846 * p_.filter_cutoff_hz * sample_dt);
      state_.filtered += (state_.raw - state_.filtered) * alpha;
      filter_stamp_ = sample_stamp;
    }
    newest_sample_stamp_ = sample_stamp;
    state_.error.setY(boundaryError(state_.filtered.y(), p_.safe_y, p_.hysteresis_y,
                                   state_.yaw_boundary_active));
    state_.error.setZ(boundaryError(state_.filtered.z(), p_.safe_z, p_.hysteresis_z,
                                   state_.z_boundary_active));
  }

  if (!flight_allowed)
  {
    // Landing, force landing, assembly/unified mode and non-position control win.
    state_.yaw_offset = state_.z_offset = 0;
    resetInput();
    state_.status = "inactive_flight_mode";
  }
  else
  {
    const bool track = state_.valid && admittance_applying;
    if (valid_dt)
    {
      // Moving the plate from +X to -X reverses its lateral sweep under yaw.
      advanceAxis(normal_sign_ * state_.error.y(), p_.yaw, p_.enabled && p_.yaw.enabled, track,
                  dt, state_.yaw_offset, state_.yaw_rate);
      advanceAxis(state_.error.z(), p_.z, p_.enabled && p_.z.enabled, track,
                  dt, state_.z_offset, state_.z_velocity);
    }
    if (!p_.enabled) state_.status = "disabled";
    else if (!p_.yaw.enabled && !p_.z.enabled) state_.status = "axes_disabled";
    else if (!valid_dt) state_.status = "invalid_dt_held";
    else if (!state_.valid) state_.status = input_status;
    else if (!admittance_applying) state_.status = "admittance_inactive_held";
    else if ((p_.yaw.enabled && state_.error.y() != 0) || (p_.z.enabled && state_.error.z() != 0))
      state_.status = "tracking_boundary";
    else state_.status = "inside_boundary_held";
  }
  state_.yaw_saturated = std::abs(state_.yaw_offset) >= p_.yaw.max_offset;
  state_.z_saturated = std::abs(state_.z_offset) >= p_.z.max_offset;
  state_.applying = state_.yaw_offset != 0 || state_.z_offset != 0 ||
                    state_.yaw_rate != 0 || state_.z_velocity != 0;
  return state_;
}

void HihCopAdjustment::applyReference(aerial_robot_navigation::BaseNavigator::TargetSnapshot& target) const
{
  if (!state_.applying) return;  // Disabled/zero correction is an exact no-op.
  if (state_.z_offset != 0) target.pos.setZ(target.pos.z() + state_.z_offset);
  if (state_.z_velocity != 0) target.vel.setZ(target.vel.z() + state_.z_velocity);
  if (state_.yaw_offset != 0)
    target.rpy.setZ(angles::normalize_angle(target.rpy.z() + state_.yaw_offset));
  if (state_.yaw_rate != 0)
  {
    tf::Matrix3x3 final_rotation;
    final_rotation.setRPY(target.rpy.x(), target.rpy.y(), target.rpy.z());
    // Euler yaw offset is a left rotation about WORLD Z. Existing BODY omega
    // components remain unchanged; only the added WORLD yaw rate is converted.
    target.omega += final_rotation.inverse() * tf::Vector3(0, 0, state_.yaw_rate);
  }
  // Deliberately no X/Y translation or roll/pitch trim, no navigator writes,
  // no second admittance, no PID/MOB gain changes and no new acceleration FF.
}

void HihCopAdjustment::update(aerial_robot_navigation::BaseNavigator::TargetSnapshot& target,
  const tf::Vector3& force_contact, const tf::Vector3& torque_contact,
  const ros::Time& sample_stamp, bool sample_valid, bool flight_allowed,
  bool admittance_applying, const ros::Time& now)
{
  // Cached booleans permit deliberate runtime ON/OFF; gains/limits load at startup.
  if (settings_)
  {
    bool master = p_.enabled, yaw = p_.yaw.enabled, z = p_.z.enabled;
    settings_->getParamCached("enabled", master);
    settings_->getParamCached("yaw/enabled", yaw);
    settings_->getParamCached("z/enabled", z);
    setEnabled(master, yaw, z);
  }
  const double time = now.toSec();
  const double dt = last_control_time_ == 0 ? 0 : time - last_control_time_;
  last_control_time_ = time;
  step(force_contact, torque_contact, sample_stamp.toSec(), sample_valid,
       time, dt, flight_allowed, admittance_applying);
  const auto before = target;
  applyReference(target);
  if (settings_) publishDebug(before, target, sample_stamp, now);
}

void HihCopAdjustment::publishDebug(
  const aerial_robot_navigation::BaseNavigator::TargetSnapshot& before,
  const aerial_robot_navigation::BaseNavigator::TargetSnapshot& after,
  const ros::Time& sample_stamp, const ros::Time& now)
{
  if (!last_debug_.isZero() && now >= last_debug_ && (now - last_debug_).toSec() < p_.debug_period) return;
  last_debug_ = now;
  geometry_msgs::WrenchStamped wrench;
  wrench.header.stamp = sample_stamp; wrench.header.frame_id = contact_frame_;
  tf::vector3TFToMsg(state_.force_contact, wrench.wrench.force);
  tf::vector3TFToMsg(state_.torque_contact, wrench.wrench.torque);
  wrench_pub_.publish(wrench);
  geometry_msgs::PointStamped point;
  point.header = wrench.header;
  point.point.x = 0; point.point.y = state_.raw.y(); point.point.z = state_.raw.z(); raw_pub_.publish(point);
  point.point.y = state_.filtered.y(); point.point.z = state_.filtered.z(); filtered_pub_.publish(point);
  geometry_msgs::Vector3Stamped vector;
  vector.header = wrench.header;
  tf::vector3TFToMsg(state_.error, vector.vector); error_pub_.publish(vector);
  const auto scalar = [](ros::Publisher& pub, double value) {
    std_msgs::Float64 msg; msg.data = value; pub.publish(msg);
  };
  scalar(normal_force_pub_, state_.normal_force);
  scalar(yaw_offset_pub_, state_.yaw_offset); scalar(yaw_rate_pub_, state_.yaw_rate);
  scalar(z_offset_pub_, state_.z_offset); scalar(z_velocity_pub_, state_.z_velocity);
  std_msgs::Bool flag;
  flag.data = state_.valid; valid_pub_.publish(flag);
  flag.data = state_.applying; applying_pub_.publish(flag);
  flag.data = state_.yaw_saturated; yaw_saturated_pub_.publish(flag);
  flag.data = state_.z_saturated; z_saturated_pub_.publish(flag);
  std_msgs::String status; status.data = state_.status; status_pub_.publish(status);
  point.header.stamp = now; point.header.frame_id = world_frame_;
  point.point.x = before.pos.x(); point.point.y = before.pos.y(); point.point.z = before.pos.z();
  admittance_position_pub_.publish(point);
  point.point.x = after.pos.x(); point.point.y = after.pos.y(); point.point.z = after.pos.z();
  command_position_pub_.publish(point);
  vector.header = point.header;
  vector.vector.x = before.rpy.z(); vector.vector.y = after.rpy.z(); vector.vector.z = state_.yaw_offset;
  yaw_reference_pub_.publish(vector);
}
}  // namespace hih_control
