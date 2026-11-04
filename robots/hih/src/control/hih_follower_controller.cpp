#include <hih/control/hih_follower_controller.h>
#include <hih/control/hih_role_parameters.h>
#include <hih/control/hih_contact_wrench.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64.h>
#include <stdexcept>

namespace
{
bool finiteVector(const tf::Vector3& v)
{ return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z()); }

void readVector(ros::NodeHandle nh, const std::string& name, hih_control::Vector4& values)
{
  std::vector<double> v;
  if (!nh.getParam(name, v) || v.size() != 4)
    throw std::invalid_argument("Expected [x, y, z, yaw]: " + nh.resolveName(name));
  std::copy(v.begin(), v.end(), values.begin());
}
}

namespace aerial_robot_control
{
HihFollowerController::HihFollowerController()
{ wrench_estimate_flag_ = false; contact_to_world_.setIdentity(); }

HihFollowerController::~HihFollowerController()
{
  ready_.store(false);
  if (wrench_estimate_thread_.joinable())
  { wrench_estimate_thread_.interrupt(); wrench_estimate_thread_.join(); }
  wrench_estimate_flag_ = false;
}

void HihFollowerController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
  boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator, double ctrl_loop_rate)
{
  hih_control::applyRoleParameters(nh, "follower");
  ros::NodeHandle settings(nh, "controller/admittance");
  settings.param("enabled", enabled_, false);
  settings.param("apply_velocity", apply_velocity_, true);
  settings.param("sample_timeout", sample_timeout_, 0.15);
  settings.param("debug_period", debug_period_, 0.05);
  settings.param("normal_force_desired", normal_force_desired_, 10.0);
  settings.param("normal_force_max", normal_force_max_, 20.0);
  settings.param("normal_force_rate", normal_force_rate_, 1.0);
  settings.param("contact_threshold", contact_threshold_, 0.8);
  std::vector<double> normal;
  if (!settings.getParam("normal_contact", normal) || normal.size() != 3)
    throw std::invalid_argument("normal_contact must contain three contact-frame components");
  normal_contact_.setValue(normal[0], normal[1], normal[2]);
  if (!std::isfinite(normal_contact_.length2()) || normal_contact_.length2() < 1e-12)
    throw std::invalid_argument("normal_contact must be finite and nonzero");
  normal_contact_.normalize();
  settings.param<std::string>("world_frame", world_frame_, "world");
  hih_control::AdmittanceParameters p;
  readVector(settings, "mass", p.mass);
  readVector(settings, "damping", p.damping);
  readVector(settings, "stiffness", p.stiffness);
  readVector(settings, "max_offset", p.max_offset);
  readVector(settings, "max_velocity", p.max_velocity);
  readVector(settings, "max_acceleration", p.max_acceleration);
  readVector(settings, "max_wrench", p.max_wrench);
  readVector(settings, "deadband", p.deadband);
  readVector(settings, "bias", p.bias);
  readVector(settings, "gain", p.gain);
  settings.param("moving_reference", p.moving_reference, false);
  if (p.moving_reference) readVector(settings, "max_travel", p.max_travel);
  settings.param("filter_cutoff_hz", p.filter_cutoff_hz, 2.0);
  settings.param("max_dt", p.max_dt, 0.1);
  admittance_.configure(p);
  max_dt_ = p.max_dt;
  if (!std::isfinite(sample_timeout_) || sample_timeout_ <= 0 ||
      !std::isfinite(debug_period_) || debug_period_ <= 0 || world_frame_.empty())
    throw std::invalid_argument("Invalid Admittance timeout/debug period/world frame");
  if (!std::isfinite(normal_force_desired_) || !std::isfinite(normal_force_max_) ||
      !std::isfinite(normal_force_rate_) || !std::isfinite(contact_threshold_) ||
      normal_force_desired_ < 0 || normal_force_desired_ > normal_force_max_ ||
      normal_force_max_ <= 0 || normal_force_rate_ <= 0 || contact_threshold_ <= 0)
    throw std::invalid_argument("Invalid normal-force target/ramp/contact threshold");
  hih_control::ContactGuardParameters guard;
  guard.acquire_force=contact_threshold_; guard.max_force=normal_force_max_;
  settings.param("contact_release", guard.release_force, 0.4);
  settings.param("contact_loss_timeout", guard.loss_timeout, 0.2);
  settings.param("stale_fault_timeout", guard.stale_timeout, 0.5);
  contact_guard_.configure(guard);
  bool observer_enabled = false;
  nh.param("controller/wrench_estimate_flag", observer_enabled, false);
  if (enabled_ && !observer_enabled)
    throw std::invalid_argument("Admittance requires controller/wrench_estimate_flag=true");
  hih_nav_ = boost::dynamic_pointer_cast<aerial_robot_navigation::HihNavigator>(navigator);
  if (!hih_nav_) throw std::invalid_argument("HiH Follower requires HihNavigator");
  hih_model_ = boost::dynamic_pointer_cast<HihRobotModel>(robot_model);
  if (!hih_model_) throw std::invalid_argument("HiH Follower requires HihRobotModel");

  cop_adjustment_.initialize(nh, normal_contact_, world_frame_);
  HihController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);
  ros::NodeHandle debug(nh_, "hih/admittance");
  nominal_pub_ = debug.advertise<geometry_msgs::PointStamped>("nominal_position", 1);
  command_pub_ = debug.advertise<geometry_msgs::PointStamped>("command_position", 1);
  offset_pub_ = debug.advertise<geometry_msgs::Vector3Stamped>("position_offset", 1);
  velocity_pub_ = debug.advertise<geometry_msgs::Vector3Stamped>("velocity_offset", 1);
  wrench_pub_ = debug.advertise<geometry_msgs::WrenchStamped>("input_wrench_world", 1);
  // x = nominal yaw [rad], y = command yaw [rad], z = offset [rad].
  yaw_pub_ = debug.advertise<geometry_msgs::Vector3Stamped>("yaw_reference", 1);
  active_pub_ = debug.advertise<std_msgs::Bool>("applying", 1);
  status_pub_ = debug.advertise<std_msgs::String>("status", 1);
  force_reference_pub_ = debug.advertise<std_msgs::Float64>("normal_force_reference", 1);
  measured_normal_pub_ = debug.advertise<std_msgs::Float64>("normal_force_measured", 1);
  fault_pub_ = debug.advertise<std_msgs::Bool>("fault", 1);
  reference_offset_pub_ = debug.advertise<geometry_msgs::Vector3Stamped>("reference_offset", 1);
  ready_.store(true);
  ROS_INFO("[HiH Follower] PID/MOB retained; world XYZ+yaw Admittance %s", enabled_ ? "ENABLED" : "DISABLED");
}

void HihFollowerController::externalWrenchEstimate()
{
  // The inherited initialize starts this thread before all HiH pointers exist.
  if (!ready_.load()) return;
  HihController::externalWrenchEstimate();
  // Before the observer starts (e.g. on the ground during Gazebo startup),
  // the estimator's IMU timestamp queue may still be empty. Its getter calls
  // back() unconditionally, so only read it after an estimate was produced.
  const double stamp = prev_est_wrench_timestamp_ > 0
    ? estimator_->getImuLatestTimeStamp() : 0.0;
  const int state = navigator_->getNaviState();
  bool valid = prev_est_wrench_timestamp_ > 0 && est_external_wrench_.size() == 6 &&
    est_external_wrench_.allFinite() && std::isfinite(stamp) && stamp > 0 &&
    (state == aerial_robot_navigation::HOVER_STATE || state == aerial_robot_navigation::TAKEOFF_STATE ||
     state == aerial_robot_navigation::LAND_STATE);
  tf::Vector3 force_contact(0, 0, 0), torque_contact(0, 0, 0), contact_offset(0, 0, 0);
  tf::Matrix3x3 contact_to_world;
  contact_to_world.setIdentity();
  if (valid)
  {
    // Same convention as HIH_sim: environment-on-robot wrench, shifted from
    // CoG to the contact center before normal-force/moment regulation.
    const tf::Matrix3x3 world_from_cog = estimator_->getOrientation(Frame::COG, estimate_mode_);
    const tf::Vector3 force_world(est_external_wrench_(0), est_external_wrench_(1), est_external_wrench_(2));
    const tf::Vector3 torque_cog(est_external_wrench_(3), est_external_wrench_(4), est_external_wrench_(5));
    tf::Transform cog_from_contact;
    tf::transformKDLToTF(hih_model_->getCog2Cp<KDL::Frame>(), cog_from_contact);
    contact_offset = cog_from_contact.getOrigin();
    const tf::Matrix3x3 cog_from_contact_rot = cog_from_contact.getBasis();
    const auto measured = hih_control::wrenchAtContact(force_world, torque_cog, world_from_cog, cog_from_contact);
    force_contact = measured.force;
    torque_contact = measured.torque;
    contact_to_world = world_from_cog * cog_from_contact_rot;
    valid = finiteVector(force_contact) && finiteVector(torque_contact) && finiteVector(contact_offset);
    for (int i = 0; i < 3; ++i) valid = valid && finiteVector(contact_to_world[i]);
  }
  std::lock_guard<std::mutex> lock(sample_mutex_);
  if (valid)
  {
    sample_force_contact_ = force_contact;
    sample_torque_contact_ = torque_contact;
    contact_offset_cog_ = contact_offset;
    contact_to_world_ = contact_to_world;
  }
  sample_valid_ = valid;
  sample_stamp_.fromSec(valid ? stamp : 0.0);
}

void HihFollowerController::reset()
{
  HihController::reset();
  admittance_.reset();
  contact_guard_.reset(); have_held_target_=false;
  cop_adjustment_.reset();
  admittance_applying_ = false;
  cop_flight_allowed_last_ = false;
  last_update_ = ros::Time(0);
  contact_acquired_ = false;
  normal_force_reference_ = 0.0;
  std::lock_guard<std::mutex> lock(sample_mutex_);
  sample_valid_ = false;
}

bool HihFollowerController::copFlightAllowed() const
{
  return ready_.load() && navigator_->getNaviState() == aerial_robot_navigation::HOVER_STATE &&
    !navigator_->getForceLandingFlag() &&
    navigator_->getXyControlMode() == aerial_robot_navigation::POS_CONTROL_MODE &&
    !hih_nav_->getCurrentAssembled() && !hih_nav_->getUnifiedControlMode();
}

bool HihFollowerController::update()
{
  // Unified control can bypass the reference modifier entirely. Clear stored
  // trim on mode exit even then, so it cannot reappear on later independent flight.
  const bool allowed = copFlightAllowed();
  if (!allowed && cop_flight_allowed_last_)
  {
    cop_adjustment_.reset(); admittance_.reset(); contact_guard_.reset();
    have_held_target_=false; contact_acquired_=false; normal_force_reference_=0;
  }
  cop_flight_allowed_last_ = allowed;
  return HihController::update();
}

void HihFollowerController::modifyTargetSnapshot(aerial_robot_navigation::BaseNavigator::TargetSnapshot& target)
{
  if (!ready_.load()) return;
  admittance_applying_ = false;
  modifyAdmittanceTarget(target);  // Existing outer-loop calculation is unchanged.
  if (admittance_applying_) { held_target_=target; have_held_target_=true; }

  tf::Vector3 force, torque;
  ros::Time stamp;
  bool valid;
  {
    std::lock_guard<std::mutex> lock(sample_mutex_);
    force = sample_force_contact_; torque = sample_torque_contact_;
    stamp = sample_stamp_; valid = sample_valid_;
  }
  const bool flight_allowed = copFlightAllowed();
  // Add only CoP yaw/world-Z trim to the completed Admittance reference.
  // The inherited PID consumes this local snapshot; the navigator is never overwritten.
  cop_adjustment_.update(target, force, torque, stamp, valid,
                         flight_allowed, admittance_applying_, ros::Time::now());
}

void HihFollowerController::modifyAdmittanceTarget(aerial_robot_navigation::BaseNavigator::TargetSnapshot& target)
{
  if (!ready_.load()) return;
  const auto nominal = target;
  const ros::Time now = ros::Time::now();
  const double dt = last_update_.isZero() ? 0.0 : (now - last_update_).toSec();
  last_update_ = now;
  std::string reason;
  if (!enabled_) reason = "disabled";
  else if (navigator_->getForceLandingFlag()) reason = "force_landing";
  else if (navigator_->getNaviState() != aerial_robot_navigation::HOVER_STATE) reason = "not_hovering";
  else if (hih_nav_->getCurrentAssembled() || hih_nav_->getUnifiedControlMode()) reason = "assembled_mode";
  else if (navigator_->getXyControlMode() != aerial_robot_navigation::POS_CONTROL_MODE) reason = "not_position_mode";
  if (!reason.empty())
  {
    admittance_.reset();
    contact_guard_.reset(); have_held_target_=false;
    contact_acquired_ = false;
    normal_force_reference_ = 0.0;
    publishDebug(nominal, target, reason, false, now);
    return;
  }

  hih_control::Vector4 wrench{{0, 0, 0, 0}};
  tf::Vector3 force_contact, torque_contact, contact_offset;
  tf::Matrix3x3 contact_to_world;
  ros::Time sample_stamp;
  bool fresh;
  {
    std::lock_guard<std::mutex> lock(sample_mutex_);
    force_contact = sample_force_contact_; torque_contact = sample_torque_contact_;
    contact_offset = contact_offset_cog_; contact_to_world = contact_to_world_;
    sample_stamp = sample_stamp_; fresh = sample_valid_;
  }
  const double age = (now - sample_stamp).toSec();
  fresh = fresh && std::isfinite(age) && age >= 0 && age <= sample_timeout_;
  const bool acquired_before=contact_guard_.acquired();
  const auto guard_state=contact_guard_.step(now.toSec(),
    std::isfinite(dt) && dt>0 && dt<=max_dt_, fresh, -normal_contact_.dot(force_contact));
  if (guard_state!=hih_control::ContactGuard::ACTIVE)
  {
    admittance_.hold();
    if (guard_state!=hih_control::ContactGuard::WAITING) holdAdmittanceTarget(target);
    publishDebug(nominal,target,contact_guard_.status(),false,now);
    return;
  }
  if (!acquired_before)
    normal_force_reference_=std::clamp(-normal_contact_.dot(force_contact),0.0,normal_force_desired_);
  contact_acquired_=true;
  normal_force_reference_ += std::clamp(normal_force_desired_ - normal_force_reference_,
                                      -normal_force_rate_ * dt, normal_force_rate_ * dt);
  // Desired environmental reaction is -F_normal*n; tangential targets are zero.
  wrench = hih_control::contactWrenchErrorWorld(
    {force_contact, torque_contact}, contact_to_world, normal_contact_, normal_force_reference_);
  if (!admittance_.step(wrench, dt))
  {
    contact_guard_.fault("admittance_travel_or_input_limit_hold");
    admittance_.hold(); holdAdmittanceTarget(target);
    publishDebug(nominal, target, contact_guard_.status(), false, now);
    return;
  }
  const auto x = admittance_.totalOffset();
  const auto& v = admittance_.velocity();
  target.rpy.setZ(angles::normalize_angle(nominal.rpy.z() + x[3]));
  tf::Matrix3x3 nominal_rot, command_rot;
  nominal_rot.setRPY(nominal.rpy.x(), nominal.rpy.y(), nominal.rpy.z());
  command_rot.setRPY(target.rpy.x(), target.rpy.y(), target.rpy.z());
  // Virtual translation describes the contact center, not the CoG. Compensate
  // the plate-center sweep when yaw changes, as in the simulation.
  target.pos += tf::Vector3(x[0], x[1], x[2]) +
    hih_control::contactCenterCompensation(nominal_rot, command_rot, contact_offset);
  if (apply_velocity_)
  {
    // TargetSnapshot::omega is expressed in target CoG axes, not world axes.
    const tf::Vector3 nominal_omega_world = nominal_rot * nominal.omega;
    const tf::Vector3 command_omega_world = nominal_omega_world + tf::Vector3(0, 0, v[3]);
    target.omega = command_rot.inverse() * command_omega_world;
    target.vel += tf::Vector3(v[0], v[1], v[2]) +
      nominal_omega_world.cross(nominal_rot * contact_offset) -
      command_omega_world.cross(command_rot * contact_offset);
  }
  // Moving reference is internal; keyboard/navigation targets remain independent.
  admittance_applying_ = true;
  publishDebug(nominal, target, fresh ? "active" : "stale_input_frozen", true, now);
}

void HihFollowerController::holdAdmittanceTarget(
  aerial_robot_navigation::BaseNavigator::TargetSnapshot& target)
{
  if (!have_held_target_) { held_target_=target; have_held_target_=true; }
  target=held_target_;
  target.vel.setZero(); target.acc.setZero(); target.omega.setZero(); target.ang_acc.setZero();
}

void HihFollowerController::publishDebug(
  const aerial_robot_navigation::BaseNavigator::TargetSnapshot& nominal,
  const aerial_robot_navigation::BaseNavigator::TargetSnapshot& command,
  const std::string& status, bool applying, const ros::Time& now)
{
  if (!last_debug_.isZero() && now >= last_debug_ && (now - last_debug_).toSec() < debug_period_) return;
  last_debug_ = now;
  geometry_msgs::PointStamped point;
  point.header.stamp = now; point.header.frame_id = world_frame_;
  point.point.x = nominal.pos.x(); point.point.y = nominal.pos.y(); point.point.z = nominal.pos.z();
  nominal_pub_.publish(point);
  point.point.x = command.pos.x(); point.point.y = command.pos.y(); point.point.z = command.pos.z();
  command_pub_.publish(point);
  geometry_msgs::Vector3Stamped vector;
  vector.header = point.header;
  const auto& x = admittance_.offset(); const auto& v = admittance_.velocity();
  vector.vector.x = x[0]; vector.vector.y = x[1]; vector.vector.z = x[2]; offset_pub_.publish(vector);
  vector.vector.x = v[0]; vector.vector.y = v[1]; vector.vector.z = v[2]; velocity_pub_.publish(vector);
  const auto& baseline=admittance_.referenceOffset();
  vector.vector.x=baseline[0]; vector.vector.y=baseline[1]; vector.vector.z=baseline[2]; reference_offset_pub_.publish(vector);
  vector.vector.x = nominal.rpy.z()+baseline[3]; vector.vector.y = command.rpy.z();
  vector.vector.z = admittance_.totalOffset()[3]; yaw_pub_.publish(vector);
  geometry_msgs::WrenchStamped force;
  force.header = point.header;
  const auto& f = admittance_.filteredWrench();
  force.wrench.force.x = f[0]; force.wrench.force.y = f[1]; force.wrench.force.z = f[2];
  force.wrench.torque.z = f[3]; wrench_pub_.publish(force);
  std_msgs::Bool active; active.data = applying; active_pub_.publish(active);
  active.data=contact_guard_.state()==hih_control::ContactGuard::FAULT; fault_pub_.publish(active);
  std_msgs::String text; text.data = status; status_pub_.publish(text);
  std_msgs::Float64 normal;
  normal.data = normal_force_reference_; force_reference_pub_.publish(normal);
  { std::lock_guard<std::mutex> lock(sample_mutex_); normal.data = -normal_contact_.dot(sample_force_contact_); }
  measured_normal_pub_.publish(normal);
}
}

PLUGINLIB_EXPORT_CLASS(aerial_robot_control::HihFollowerController, aerial_robot_control::ControlBase);
