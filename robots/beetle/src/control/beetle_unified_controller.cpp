// Unified controller for assembled beetle formation.
// PC outer loop (40Hz): 6-DOF PID + formation-wide pseudoinverse allocation.
// Spinal inner loop (1000Hz): P+D attitude tracking per-motor.
// PC retains I-term only for roll/pitch.

#include <beetle/control/beetle_unified_controller.h>
#include <OsqpEigen/OsqpEigen.h>
#include <tf_conversions/tf_kdl.h>
#include <tf_conversions/tf_eigen.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/TransformStamped.h>
#include <XmlRpcValue.h>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

namespace aerial_robot_control
{

BeetleUnifiedController::BeetleUnifiedController()
  : motor_num_per_module_(4),
    gimbal_dof_(1),
    rotor_coef_(2),
    gimbal_calc_in_fc_(false),
    yaw_in_allocation_(false),
    formation_mass_(0),
    formation_cog_offset_(Eigen::Vector3d::Zero()),
    formation_inertia_(Eigen::Matrix3d::Zero()),
    use_external_formation_model_(false),
    external_formation_mass_(0),
    external_formation_cog_offset_(Eigen::Vector3d::Zero()),
    external_formation_inertia_(Eigen::Matrix3d::Zero()),
    module_model_revision_(0),
    cached_module_model_revision_(0),
    internal_wrench_secondary_gain_(0.0),
    candidate_yaw_term_(0),
    command_target_rpy_(0, 0, 0),
    cascade_alloc_sent_(false),
    has_cascade_gain_cache_(false),
    cached_cascade_roll_p_(0),
    cached_cascade_roll_i_(0),
    cached_cascade_roll_d_(0),
    cached_cascade_pitch_p_(0),
    cached_cascade_pitch_i_(0),
    cached_cascade_pitch_d_(0),
    cached_cascade_yaw_d_(0),
    use_constrained_alloc_(false),
	    alloc_lambda_(1e-4),
	    alloc_t_max_(20.0),
	    alloc_gimbal_limit_rad_(M_PI / 2.0),
	    alloc_rate_weight_(0.0),
	    alloc_rate_limit_(0.0),
	    alloc_direction_rate_limit_rad_(0.0),
	    alloc_lateral_rate_weight_(0.0),
	    alloc_wrench_weights_(Eigen::VectorXd::Ones(6)),
	    alloc_task_wrench_weights_(Eigen::VectorXd::Ones(6)),
	    alloc_effort_weight_(0.0),
	    alloc_interface_force_weight_(0.0),
	    alloc_interface_torque_weight_(0.0),
	    alloc_interface_force_limit_(0.0),
	    alloc_interface_torque_limit_(0.0),
	    alloc_module_balance_weight_(0.0),
	    alloc_priority_enabled_(false),
	    alloc_priority_tolerances_(Eigen::VectorXd::Zero(6)),
	    alloc_task_priority_enabled_(true),
	    alloc_task_priority_min_weight_(0.5),
	    qp_n_vars_(-1),
	    qp_n_constraints_(-1),
	    qp_hessian_nnz_(-1),
	    qp_constraint_nnz_(-1),
	    last_qp_diag_log_time_(-1.0),
	    pinv_pwm_pred_pub_interval_(0.1),
	    last_pinv_pwm_pred_pub_time_(-1.0),
	    pinv_pwm_min_(0.5),
	    pinv_pwm_max_(0.85),
	    pinv_pwm_min_thrust_(0.0),
	    pinv_pwm_conversion_mode_(-1),
	    battery_voltage_(0.0),
	    battery_voltage_received_(false),
	    qp_solver_(std::make_unique<OsqpEigen::Solver>())
{
}

BeetleUnifiedController::~BeetleUnifiedController() = default;

void BeetleUnifiedController::initialize(
    ros::NodeHandle nh,
    boost::shared_ptr<BeetleRobotModel> robot_model,
    boost::shared_ptr<aerial_robot_navigation::BeetleNavigator> navigator,
    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator)
{
  nh_ = nh;
  robot_model_ = robot_model;
  navigator_ = navigator;
  estimator_ = estimator;

  motor_num_per_module_ = robot_model_->getRotorNum();
  rosParamInit();
  rotor_coef_ = gimbal_dof_ + 1;

  int max_modules = navigator_->getMaxModuleNum();
  std::string my_name = navigator_->getMyName();
  for (int i = 1; i <= max_modules; i++) {
    std::string ns = std::string("/") + my_name + std::to_string(i);
    module_torque_alloc_inv_pubs_[i] = nh_.advertise<spinal::TorqueAllocationMatrixInv>(
        ns + "/torque_allocation_matrix_inv", 1);
    module_rpy_gain_pubs_[i] = nh_.advertise<spinal::RollPitchYawTerms>(ns + "/rpy/gain", 1);
    module_gimbal_dof_pubs_[i] = nh_.advertise<std_msgs::UInt8>(ns + "/gimbal_dof", 1);
  }

  formation_wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("unified_control/formation_wrench", 1);
  formation_vectoring_f_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("unified_control/vectoring_force", 1);
  interface_load_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("unified_control/interface_load", 1);
  qp_pwm_pred_pub_ = nh_.advertise<spinal::Pwms>("unified_control/qp_pwm_pred", 1);
  pinv_pwm_pred_pub_ = nh_.advertise<spinal::Pwms>("unified_control/pinv_pwm_pred", 1);
  qp_thrust_margin_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("unified_control/qp_thrust_margin", 1);
  pinv_thrust_margin_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("unified_control/pinv_thrust_margin", 1);
  battery_voltage_sub_ = nh_.subscribe("battery_voltage_status", 1,
                                       &BeetleUnifiedController::batteryVoltageCallback, this);

  ROS_INFO("[UnifiedCtrl] Initialized: motor_per_module=%d, gimbal_dof=%d, rotor_coef=%d, gimbal_calc_in_fc=%d",
           motor_num_per_module_, gimbal_dof_, rotor_coef_, gimbal_calc_in_fc_);
}

void BeetleUnifiedController::rosParamInit()
{
  ros::NodeHandle control_nh(nh_, "controller");
  control_nh.param<int>("gimbal_dof", gimbal_dof_, 1);
  control_nh.param<bool>("gimbal_calc_in_fc", gimbal_calc_in_fc_, false);
  control_nh.param<bool>("yaw_in_allocation", yaw_in_allocation_, false);
  control_nh.param<bool>("use_constrained_alloc", use_constrained_alloc_, false);
  control_nh.param<double>("alloc_lambda", alloc_lambda_, 1e-4);
  control_nh.param<double>("alloc_t_max", alloc_t_max_, 20.0);
  control_nh.param<double>("alloc_rate_weight", alloc_rate_weight_, 0.0);
  control_nh.param<double>("alloc_rate_limit", alloc_rate_limit_, 0.0);
  double direction_rate_limit_deg = 0.0;
  control_nh.param<double>("alloc_direction_rate_limit_deg", direction_rate_limit_deg, 0.0);
  control_nh.param<double>("alloc_lateral_rate_weight", alloc_lateral_rate_weight_, 0.0);
  control_nh.param<double>("alloc_effort_weight", alloc_effort_weight_, 0.0);
  control_nh.param<double>("alloc_interface_force_weight", alloc_interface_force_weight_, 0.0);
  control_nh.param<double>("alloc_interface_torque_weight", alloc_interface_torque_weight_, 0.0);
  control_nh.param<double>("alloc_interface_force_limit", alloc_interface_force_limit_, 0.0);
  control_nh.param<double>("alloc_interface_torque_limit", alloc_interface_torque_limit_, 0.0);
  control_nh.param<double>("alloc_module_balance_weight", alloc_module_balance_weight_, 0.0);
  alloc_module_balance_weight_ = std::max(0.0, alloc_module_balance_weight_);
  control_nh.param<bool>("alloc_priority_enabled", alloc_priority_enabled_, false);
  control_nh.param<bool>("alloc_task_priority_enabled", alloc_task_priority_enabled_, true);
  control_nh.param<double>("alloc_task_priority_min_weight", alloc_task_priority_min_weight_, 0.5);
  control_nh.param<double>("pinv_pwm_pred_pub_interval", pinv_pwm_pred_pub_interval_, 0.1);
  double gimbal_limit_deg;
  control_nh.param<double>("alloc_gimbal_limit_deg", gimbal_limit_deg, 90.0);
  alloc_gimbal_limit_rad_ = gimbal_limit_deg * M_PI / 180.0;

  alloc_rate_weight_ = std::max(0.0, alloc_rate_weight_);
  alloc_rate_limit_ = std::max(0.0, alloc_rate_limit_);
  alloc_direction_rate_limit_rad_ =
      std::max(0.0, direction_rate_limit_deg) * M_PI / 180.0;
  alloc_lateral_rate_weight_ = std::max(0.0, alloc_lateral_rate_weight_);
  alloc_effort_weight_ = std::max(0.0, alloc_effort_weight_);
  alloc_interface_force_weight_ = std::max(0.0, alloc_interface_force_weight_);
  alloc_interface_torque_weight_ = std::max(0.0, alloc_interface_torque_weight_);
  alloc_interface_force_limit_ = std::max(0.0, alloc_interface_force_limit_);
  alloc_interface_torque_limit_ = std::max(0.0, alloc_interface_torque_limit_);
  alloc_task_priority_min_weight_ = std::max(0.0, alloc_task_priority_min_weight_);
  pinv_pwm_pred_pub_interval_ = std::max(0.0, pinv_pwm_pred_pub_interval_);

  ros::NodeHandle motor_nh(nh_, "motor_info");
  motor_nh.param<double>("min_pwm", pinv_pwm_min_, 0.5);
  motor_nh.param<double>("max_pwm", pinv_pwm_max_, 0.85);
  motor_nh.param<double>("min_thrust", pinv_pwm_min_thrust_, 0.0);
  motor_nh.param<int>("pwm_conversion_mode", pinv_pwm_conversion_mode_, -1);

  int vel_ref_num = 0;
  motor_nh.param<int>("vel_ref_num", vel_ref_num, 0);
  pinv_motor_info_.clear();
  pinv_motor_info_.reserve(std::max(0, vel_ref_num));
  for (int i = 0; i < vel_ref_num; i++) {
    std::stringstream ss;
    ss << i + 1;
    ros::NodeHandle ref_nh(motor_nh, "ref" + ss.str());

    spinal::MotorInfo info;
    double val = 0.0;
    ref_nh.param<double>("voltage", val, 0.0);
    info.voltage = val;
    ref_nh.param<double>("max_thrust", val, 0.0);
    info.max_thrust = val;
    for (int j = 0; j < 5; j++) {
      std::stringstream ss2;
      ss2 << j;
      ref_nh.param<double>("polynominal" + ss2.str(), val, 0.0);
      info.polynominal[j] = val;
    }
    pinv_motor_info_.push_back(info);
  }

  alloc_priority_tolerances_ = Eigen::VectorXd::Zero(6);
  XmlRpc::XmlRpcValue priority_tolerances;
  if (control_nh.getParam("alloc_priority_wrench_tolerances", priority_tolerances)) {
    if (priority_tolerances.getType() == XmlRpc::XmlRpcValue::TypeArray &&
        priority_tolerances.size() == 6) {
      for (int i = 0; i < 6; i++) {
        double tolerance = 0.0;
        if (priority_tolerances[i].getType() == XmlRpc::XmlRpcValue::TypeInt) {
          tolerance = static_cast<int>(priority_tolerances[i]);
        } else if (priority_tolerances[i].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
          tolerance = static_cast<double>(priority_tolerances[i]);
        } else {
          ROS_WARN("[UnifiedCtrl] alloc_priority_wrench_tolerances[%d] is not numeric; disabling row", i);
        }
        alloc_priority_tolerances_(i) = std::max(0.0, tolerance);
      }
    } else {
      ROS_WARN("[UnifiedCtrl] alloc_priority_wrench_tolerances must be a YAML list of 6 numbers; disabling priority rows");
    }
  }

  alloc_wrench_weights_ = Eigen::VectorXd::Ones(6);
  XmlRpc::XmlRpcValue wrench_weights;
  if (control_nh.getParam("alloc_wrench_weights", wrench_weights)) {
    if (wrench_weights.getType() == XmlRpc::XmlRpcValue::TypeArray &&
        wrench_weights.size() == 6) {
      for (int i = 0; i < 6; i++) {
        double weight = 1.0;
        if (wrench_weights[i].getType() == XmlRpc::XmlRpcValue::TypeInt) {
          weight = static_cast<int>(wrench_weights[i]);
        } else if (wrench_weights[i].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
          weight = static_cast<double>(wrench_weights[i]);
        } else {
          ROS_WARN("[UnifiedCtrl] alloc_wrench_weights[%d] is not numeric; using 1.0", i);
        }
        alloc_wrench_weights_(i) = std::max(0.0, weight);
      }
    } else {
      ROS_WARN("[UnifiedCtrl] alloc_wrench_weights must be a YAML list of 6 numbers; using all ones");
    }
  }

  alloc_task_wrench_weights_ = Eigen::VectorXd::Ones(6);
  XmlRpc::XmlRpcValue task_wrench_weights;
  if (control_nh.getParam("alloc_task_wrench_weights", task_wrench_weights)) {
    if (task_wrench_weights.getType() == XmlRpc::XmlRpcValue::TypeArray &&
        task_wrench_weights.size() == 6) {
      for (int i = 0; i < 6; i++) {
        double weight = 1.0;
        if (task_wrench_weights[i].getType() == XmlRpc::XmlRpcValue::TypeInt) {
          weight = static_cast<int>(task_wrench_weights[i]);
        } else if (task_wrench_weights[i].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
          weight = static_cast<double>(task_wrench_weights[i]);
        } else {
          ROS_WARN("[UnifiedCtrl] alloc_task_wrench_weights[%d] is not numeric; using 1.0", i);
        }
        alloc_task_wrench_weights_(i) = std::max(0.0, weight);
      }
    } else {
      ROS_WARN("[UnifiedCtrl] alloc_task_wrench_weights must be a YAML list of 6 numbers; using all ones");
    }
  }

  alloc_module_weights_.clear();
  XmlRpc::XmlRpcValue module_weights;
  if (control_nh.getParam("alloc_module_weights", module_weights)) {
    if (module_weights.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      for (int i = 0; i < module_weights.size(); i++) {
        if (module_weights[i].getType() == XmlRpc::XmlRpcValue::TypeInt) {
          alloc_module_weights_.push_back(static_cast<int>(module_weights[i]));
        } else if (module_weights[i].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
          alloc_module_weights_.push_back(static_cast<double>(module_weights[i]));
        } else {
          ROS_WARN("[UnifiedCtrl] alloc_module_weights[%d] is not numeric; using default weight 1.0", i);
          alloc_module_weights_.push_back(1.0);
        }
      }
    } else {
      ROS_WARN("[UnifiedCtrl] alloc_module_weights must be a YAML list; ignoring it");
    }
  }
}

void BeetleUnifiedController::setInternalWrenchSecondaryReference(
    const std::map<int, Eigen::VectorXd>& module_wrench_comp,
    double gain)
{
  module_internal_wrench_comp_ = module_wrench_comp;
  internal_wrench_secondary_gain_ = std::max(0.0, gain);
}

void BeetleUnifiedController::clearInternalWrenchSecondaryReference()
{
  module_internal_wrench_comp_.clear();
  internal_wrench_secondary_gain_ = 0.0;
}

void BeetleUnifiedController::setModuleModelDescriptor(
    int module_id, const ModuleModelDescriptor& model)
{
  if (module_id <= 0 || !model.valid(motor_num_per_module_)) {
    ROS_WARN_THROTTLE(1.0, "[UnifiedCtrl] Reject invalid module model id=%d mass=%.3f rotors=%zu dirs=%zu mf_rate=%.6f",
                      module_id, model.mass,
                      model.rotor_origins_from_cog.size(),
                      model.rotor_direction.size(),
                      model.mf_rate);
    return;
  }

  std::lock_guard<std::mutex> lock(module_model_mutex_);
  auto prev = module_models_.find(module_id);
  if (prev != module_models_.end()) {
    bool same = std::abs(prev->second.mass - model.mass) < 1e-9 &&
                (prev->second.inertia - model.inertia).norm() < 1e-9 &&
                std::abs(prev->second.mf_rate - model.mf_rate) < 1e-12 &&
                prev->second.rotor_direction == model.rotor_direction &&
                prev->second.rotor_origins_from_cog.size() == model.rotor_origins_from_cog.size();
    if (same) {
      for (size_t i = 0; i < model.rotor_origins_from_cog.size(); i++) {
        if ((prev->second.rotor_origins_from_cog[i] - model.rotor_origins_from_cog[i]).norm() >= 1e-9) {
          same = false;
          break;
        }
      }
    }
    if (same) return;
  }
  module_models_[module_id] = model;
  module_model_revision_++;
}

bool BeetleUnifiedController::getModuleModelDescriptor(int module_id,
                                                       ModuleModelDescriptor& model) const
{
  std::lock_guard<std::mutex> lock(module_model_mutex_);
  auto it = module_models_.find(module_id);
  if (it == module_models_.end() || !it->second.valid(motor_num_per_module_)) {
    return false;
  }
  model = it->second;
  return true;
}

bool BeetleUnifiedController::getModuleMassInertia(
    int module_id, double& mass, Eigen::Matrix3d& inertia) const
{
  ModuleModelDescriptor model;
  if (!getModuleModelDescriptor(module_id, model)) return false;
  mass = model.mass;
  inertia = model.inertia;
  return true;
}

bool BeetleUnifiedController::lookupModuleOffsetFromLeader(
    int module_id, Eigen::Vector3d& offset) const
{
  offset.setZero();
  int leader_id = navigator_->getLeaderID();
  if (module_id == leader_id) return true;

  try {
    std::string leader_cog_frame = navigator_->getMyName() + std::to_string(leader_id) + "/cog";
    std::string module_cog_frame = navigator_->getMyName() + std::to_string(module_id) + "/cog";
    geometry_msgs::TransformStamped tf_stamped =
        navigator_->getTfBuffer().lookupTransform(leader_cog_frame, module_cog_frame, ros::Time(0));
    offset << tf_stamped.transform.translation.x,
              tf_stamped.transform.translation.y,
              tf_stamped.transform.translation.z;
    return true;
  } catch (tf2::TransformException& ex) {
    ROS_WARN_THROTTLE(1.0, "[UnifiedCtrl] TF lookup for module offset failed: %s", ex.what());
    return false;
  }
}

Eigen::Vector3d BeetleUnifiedController::getModuleOffsetFromLeader(int module_id) const
{
  Eigen::Vector3d offset = Eigen::Vector3d::Zero();
  lookupModuleOffsetFromLeader(module_id, offset);
  return offset;
}

bool BeetleUnifiedController::getCachedModuleOffsetFromLeader(
    int module_id, Eigen::Vector3d& offset) const
{
  auto it = cached_module_offsets_from_leader_.find(module_id);
  if (it != cached_module_offsets_from_leader_.end()) {
    offset = it->second;
    return true;
  }
  return lookupModuleOffsetFromLeader(module_id, offset);
}

std::vector<Eigen::MatrixXd> BeetleUnifiedController::buildRotorMask() const
{
  std::vector<KDL::Rotation> thrust_coords_rot =
      robot_model_->getThrustCoordRot<KDL::Rotation>();

  std::vector<Eigen::MatrixXd> masked_rot_single;
  masked_rot_single.reserve(motor_num_per_module_);
  for (int r = 0; r < motor_num_per_module_; r++) {
    tf::Quaternion quat;
    tf::quaternionKDLToTF(thrust_coords_rot.at(r), quat);
    Eigen::Matrix3d conv_cog_from_thrust;
    tf::matrixTFToEigen(tf::Matrix3x3(quat), conv_cog_from_thrust);

    if (gimbal_dof_ == 1) {
      Eigen::MatrixXd mask(3, 2);
      mask << 0, 0, 1, 0, 0, 1;
      masked_rot_single.push_back(conv_cog_from_thrust * mask);
    } else if (gimbal_dof_ == 2) {
      masked_rot_single.push_back(conv_cog_from_thrust);
    }
  }
  return masked_rot_single;
}

bool BeetleUnifiedController::updateFormationGeometry()
{
  if (use_external_formation_model_) {
    if (external_formation_mass_ <= 0.0) {
      ROS_WARN_THROTTLE(1.0, "[UnifiedCtrl] Invalid external formation mass %.4f", external_formation_mass_);
      return false;
    }
    formation_mass_ = external_formation_mass_;
    formation_cog_offset_ = external_formation_cog_offset_;
    formation_inertia_ = external_formation_inertia_;
    cached_assembled_ids_.clear();  // invalidate cache so next non-external call refreshes
    cached_module_offsets_from_leader_.clear();
    return true;
  }

  std::vector<int> assembled_ids = navigator_->getAssemblyIds();
  if (assembled_ids.empty()) return false;

  // ε-fix: reuse the latched module offsets while the assembled-IDs set is unchanged.
  // Re-running lookupTransform per cycle introduced ~10 cm Z drift in cog_offset
  // under sustained tilt (real-hw pitch=0.4). Geometry is structurally constant
  // for a given assembled set; mass/inertia may update without relatching TF.
  uint64_t model_revision = 0;
  {
    std::lock_guard<std::mutex> lock(module_model_mutex_);
    model_revision = module_model_revision_;
  }
  bool ids_changed = assembled_ids != cached_assembled_ids_;
  if (!ids_changed && formation_mass_ > 0.0 &&
      model_revision == cached_module_model_revision_) return true;

  int N = assembled_ids.size();
  if (ids_changed) cached_module_offsets_from_leader_.clear();
  for (int module_id : assembled_ids) {
    if (cached_module_offsets_from_leader_.count(module_id) > 0) continue;
    Eigen::Vector3d module_offset;
    if (!lookupModuleOffsetFromLeader(module_id, module_offset)) return false;
    cached_module_offsets_from_leader_[module_id] = module_offset;
  }

  formation_mass_ = 0.0;
  Eigen::Vector3d weighted_cog_offset = Eigen::Vector3d::Zero();
  std::string missing_models;
  for (int module_id : assembled_ids) {
    Eigen::Vector3d module_offset;
    if (!getCachedModuleOffsetFromLeader(module_id, module_offset)) return false;
    ModuleModelDescriptor model;
    if (!getModuleModelDescriptor(module_id, model)) {
      missing_models += std::to_string(module_id) + ",";
      continue;
    }
    formation_mass_ += model.mass;
    weighted_cog_offset += model.mass * module_offset;
  }
  if (!missing_models.empty()) {
    formation_mass_ = 0.0;
    ROS_WARN_THROTTLE(1.0,
                      "[UnifiedCtrl] Formation geometry blocked: missing ModuleModel for assembled ids=[%s]",
                      missing_models.c_str());
    return false;
  }
  if (formation_mass_ <= 0.0) {
    ROS_WARN_THROTTLE(1.0, "[UnifiedCtrl] Invalid heterogeneous formation mass %.4f",
                      formation_mass_);
    return false;
  }
  formation_cog_offset_ = weighted_cog_offset / formation_mass_;
  formation_inertia_ = computeFormationInertia(assembled_ids, formation_cog_offset_);
  cached_assembled_ids_ = assembled_ids;  // ε-fix: latch
  cached_module_model_revision_ = model_revision;
  // Geometry actually changed → spinals' one-shot torque_alloc_inv + cascade
  // gains are stale. Re-arm so computeUnifiedAllocation() will re-send this
  // cycle (leader). Followers detect via getFormationRevision() bump from the
  // BeetleController side.
  cascade_alloc_sent_ = false;
  ROS_INFO("[UnifiedCtrl] Formation geometry latched for assembled_ids=[%s] "
           "N=%d cog_offset=(%.4f,%.4f,%.4f) mass=%.3f",
           [&]{ std::string s; for(int id : assembled_ids){ s += std::to_string(id) + ","; } return s; }().c_str(),
           N, formation_cog_offset_.x(), formation_cog_offset_.y(), formation_cog_offset_.z(),
           formation_mass_);
  return true;
}

bool BeetleUnifiedController::computeUnifiedAllocation(
  const Eigen::VectorXd& target_wrench_acc_cog,
  const Eigen::VectorXd& desired_ext_wrench,
  double yaw_pid_raw,
  const Eigen::VectorXd& priority_wrench_acc_cog,
  const Eigen::VectorXd& task_wrench_weights,
  const Eigen::VectorXd& observer_feedback_wrench,
  const Eigen::VectorXd& observer_feedback_wrench_weights)
{
  std::vector<int> assembled_ids = navigator_->getAssemblyIds();
  if (assembled_ids.empty()) {
    ROS_WARN_THROTTLE(1.0,
                      "[UnifiedCtrl QP] stage=alloc_reject reason=no_assembled_ids");
    return false;
  }

  int N = assembled_ids.size();

  // Update formation geometry
  if (!updateFormationGeometry()) {
    ROS_WARN_THROTTLE(1.0,
                      "[UnifiedCtrl QP] stage=alloc_reject reason=geometry_update_failed N=%d",
                      N);
    return false;
  }

  std::lock_guard<std::mutex> alloc_lock(allocation_mutex_);

  // Build formation-wide allocation matrix (6 x rotor_coef*total_rotors)
  integrated_map_ = buildFormationAllocationMatrix(assembled_ids, formation_mass_,
                                                    formation_inertia_, formation_cog_offset_);

  // Pseudoinverse
  integrated_map_inv_ = aerial_robot_model::pseudoinverse(integrated_map_);

  // Split: rotational part (last 3 cols) for torque_allocation_matrix_inv
  integrated_map_inv_rot_ = integrated_map_inv_.rightCols(3);

  // Convert external task wrench from force/torque to acceleration space. High
  // task-weight axes are tracked as narrow bands inside the QP; lower-weight
  // axes remain soft so stabilization keeps residual authority.
  Eigen::VectorXd control_wrench_acc = target_wrench_acc_cog;
  Eigen::VectorXd task_wrench_acc = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd feedback_wrench_acc = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd priority_wrench_acc =
      (priority_wrench_acc_cog.size() == target_wrench_acc_cog.size())
          ? priority_wrench_acc_cog : target_wrench_acc_cog;
  Eigen::VectorXd effective_task_weights =
      (task_wrench_weights.size() == 6) ? task_wrench_weights : alloc_task_wrench_weights_;
  Eigen::VectorXd effective_feedback_weights =
      (observer_feedback_wrench_weights.size() == 6)
          ? observer_feedback_wrench_weights : Eigen::VectorXd::Zero(6);
  if (effective_task_weights.size() != 6) effective_task_weights = Eigen::VectorXd::Ones(6);
  if (effective_feedback_weights.size() != 6) effective_feedback_weights = Eigen::VectorXd::Zero(6);
  for (int i = 0; i < effective_task_weights.size(); i++) {
    effective_task_weights(i) = std::max(0.0, effective_task_weights(i));
    effective_feedback_weights(i) = std::max(0.0, effective_feedback_weights(i));
  }
  const bool has_desired_ext_wrench =
      (desired_ext_wrench.size() == 6 && desired_ext_wrench.norm() > 1e-6);
  const bool has_feedback_ext_wrench =
      (observer_feedback_wrench.size() == 6 &&
       observer_feedback_wrench.norm() > 1e-6 &&
       effective_feedback_weights.maxCoeff() > 0.0);
  double mass_inv = 1.0 / formation_mass_;
  Eigen::Matrix3d inertia_inv = formation_inertia_.inverse();
  if (has_desired_ext_wrench) {
    task_wrench_acc.head(3) = mass_inv * desired_ext_wrench.head(3);
    task_wrench_acc.tail(3) = inertia_inv * desired_ext_wrench.tail(3);
  }
  if (has_feedback_ext_wrench) {
    feedback_wrench_acc.head(3) = mass_inv * observer_feedback_wrench.head(3);
    feedback_wrench_acc.tail(3) = inertia_inv * observer_feedback_wrench.tail(3);
  }
  if (has_desired_ext_wrench || has_feedback_ext_wrench) {
    ROS_INFO_THROTTLE(
        1.0,
        "[UnifiedCtrl QPRef] id=%d control=(%.2f,%.2f,%.2f;%.2f,%.2f,%.2f) "
        "task_acc=(%.2f,%.2f,%.2f;%.2f,%.2f,%.2f) "
        "fb_acc=(%.2f,%.2f,%.2f;%.2f,%.2f,%.2f) "
        "task_w=(%.2f,%.2f,%.2f;%.2f,%.2f,%.2f) "
        "fb_w=(%.2f,%.2f,%.2f;%.2f,%.2f,%.2f) "
        "task_prio=(enabled=%d,min_w=%.2f,tol_xy=%.2f/%.2f) "
        "rate=(w=%.1e,fx=%.1e,lim=%.2f,dir=%.1fdeg)",
        navigator_ ? navigator_->getMyID() : 0,
        control_wrench_acc(0), control_wrench_acc(1), control_wrench_acc(2),
        control_wrench_acc(3), control_wrench_acc(4), control_wrench_acc(5),
        task_wrench_acc(0), task_wrench_acc(1), task_wrench_acc(2),
        task_wrench_acc(3), task_wrench_acc(4), task_wrench_acc(5),
        feedback_wrench_acc(0), feedback_wrench_acc(1), feedback_wrench_acc(2),
        feedback_wrench_acc(3), feedback_wrench_acc(4), feedback_wrench_acc(5),
        effective_task_weights(0), effective_task_weights(1), effective_task_weights(2),
        effective_task_weights(3), effective_task_weights(4), effective_task_weights(5),
        effective_feedback_weights(0), effective_feedback_weights(1), effective_feedback_weights(2),
        effective_feedback_weights(3), effective_feedback_weights(4), effective_feedback_weights(5),
        alloc_task_priority_enabled_ ? 1 : 0, alloc_task_priority_min_weight_,
        alloc_priority_tolerances_.size() > 0 ? alloc_priority_tolerances_(0) : 0.0,
        alloc_priority_tolerances_.size() > 1 ? alloc_priority_tolerances_(1) : 0.0,
        alloc_rate_weight_, alloc_lateral_rate_weight_, alloc_rate_limit_,
        alloc_direction_rate_limit_rad_ * 180.0 / M_PI);
  }
  Eigen::VectorXd pinv_vectoring_f =
      integrated_map_inv_ * (control_wrench_acc + task_wrench_acc + feedback_wrench_acc);
  Eigen::VectorXd desired_wrench_acc =
      control_wrench_acc + task_wrench_acc + feedback_wrench_acc;
  Eigen::VectorXd secondary_ref = buildSecondaryAllocationReference(assembled_ids);
  Eigen::MatrixXd interface_load_matrix;
  std::vector<std::pair<int, int>> interface_cuts;
  buildInterfaceLoadMatrix(assembled_ids, interface_load_matrix, interface_cuts);
  Eigen::VectorXd interface_load_reference = Eigen::VectorXd::Zero(interface_load_matrix.rows());
  if (interface_load_matrix.cols() == secondary_ref.size()) {
    interface_load_reference = interface_load_matrix * secondary_ref;
  }

  // Allocate: active task axes are narrow hard bands; effort/rate/balancing
  // terms then choose the actuator-side solution inside that feasible set.
  if (use_constrained_alloc_) {
    bool qp_ok = solveFullVectorQP(integrated_map_, control_wrench_acc,
                                   task_wrench_acc, effective_task_weights,
                                   feedback_wrench_acc, effective_feedback_weights,
                                   priority_wrench_acc,
                                   secondary_ref, assembled_ids,
                                   interface_load_matrix, interface_load_reference,
                                   target_vectoring_f_);
    if (!qp_ok && (has_desired_ext_wrench || has_feedback_ext_wrench)) {
      const double retry_task_scales[] = {0.7, 0.4, 0.0};
      for (double retry_scale : retry_task_scales) {
        Eigen::VectorXd retry_task_wrench_acc = retry_scale * task_wrench_acc;
        Eigen::VectorXd retry_task_weights = retry_scale * effective_task_weights;
        Eigen::VectorXd retry_feedback_wrench_acc = retry_scale * feedback_wrench_acc;
        Eigen::VectorXd retry_feedback_weights = retry_scale * effective_feedback_weights;
        Eigen::VectorXd retry_priority_acc = priority_wrench_acc_cog;
        if (retry_priority_acc.size() != target_wrench_acc_cog.size()) {
          retry_priority_acc = target_wrench_acc_cog;
        }

        qp_ok = solveFullVectorQP(integrated_map_, control_wrench_acc,
                                  retry_task_wrench_acc, retry_task_weights,
                                  retry_feedback_wrench_acc, retry_feedback_weights,
                                  retry_priority_acc,
                                  secondary_ref, assembled_ids,
                                  interface_load_matrix, interface_load_reference,
                                  target_vectoring_f_);
        if (!qp_ok) continue;

        task_wrench_acc = retry_task_wrench_acc;
        effective_task_weights = retry_task_weights;
        feedback_wrench_acc = retry_feedback_wrench_acc;
        effective_feedback_weights = retry_feedback_weights;
        if (retry_scale > 1e-6) {
          ROS_WARN_THROTTLE(
              1.0,
              "[UnifiedCtrl QP] external wrench objectives scaled to %.0f%% after constrained solve failed",
              retry_scale * 100.0);
        } else {
          ROS_WARN_THROTTLE(
              1.0,
              "[UnifiedCtrl QP] external wrench dropped after constrained solve failed");
        }
        break;
      }
    }
    if (!qp_ok) {
      ROS_ERROR_THROTTLE(1.0,
                         "[UnifiedCtrl QP] constrained allocation failed; command not updated");
      return false;
    }
  } else {
    if (alloc_lambda_ > 0.0 && secondary_ref.size() == integrated_map_.cols()) {
      Eigen::MatrixXd lhs = integrated_map_ * integrated_map_.transpose()
                           + alloc_lambda_ * Eigen::MatrixXd::Identity(integrated_map_.rows(),
                                                                       integrated_map_.rows());
      target_vectoring_f_ = secondary_ref
          + integrated_map_.transpose()
              * lhs.ldlt().solve(desired_wrench_acc - integrated_map_ * secondary_ref);
    } else {
      target_vectoring_f_ = pinv_vectoring_f;
    }
  }
  pinv_vectoring_f =
      integrated_map_inv_ * (control_wrench_acc + task_wrench_acc + feedback_wrench_acc);
  publishAllocationPwmPredictions(target_vectoring_f_, pinv_vectoring_f, assembled_ids);

  // Target attitude for spinal inner loop is the operator's commanded attitude
  // (navigator->target_rpy_), NOT atan2(target_acc) derived from XY-PID.
  //
  // Rationale: unified mode is fully-actuated — gimbal vectoring already produces
  // body-x/y acceleration via the 6-DOF allocation, so the cascade must NOT also
  // tilt the body to generate that same acceleration (double-actuation positive
  // feedback). Real-hw log (pitch≈0.4 hover) showed the atan2 path coupled with
  // XY-PID drove monotonic pitch divergence; outer PITCH-I wound up to +5 N·m
  // without ever correcting the body angle. See gimbalrotor_controller.cpp
  // fully-actuated branch for the equivalent pattern.
  // (Per-frame value is latched by BeetleController after follower leader-reference override.)

  // Compute candidate yaw term for spinal yaw reconstruction.
  // When yaw already participates in unified allocation, do NOT reconstruct the
  // same yaw command again on spinal, otherwise the yaw effect is applied twice.
  if (yaw_in_allocation_) {
    candidate_yaw_term_ = 0.0;
  } else {
    int yaw_col = 5; // yaw is the 6th column (index 5) in the 6-DOF wrench
    double max_yaw_scale = 0;
    int total_motors = assembled_ids.size() * motor_num_per_module_;
    for (int i = 0; i < total_motors; i++) {
      if (integrated_map_inv_(i * rotor_coef_, yaw_col) > max_yaw_scale)
        max_yaw_scale = integrated_map_inv_(i * rotor_coef_, yaw_col);
    }
    candidate_yaw_term_ = yaw_pid_raw * max_yaw_scale;
  }

  // Extract per-rotor scalar thrust + gimbal angles (for debug/visualization)
  extractThrustAndGimbal(target_vectoring_f_, assembled_ids);
  publishInterfaceLoadDiagnostics(interface_load_matrix, interface_cuts,
                                  target_vectoring_f_, interface_load_reference);

  ROS_INFO_THROTTLE(5.0, "[UnifiedCtrl] N=%d mass=%.3f cog_offset=(%.4f,%.4f,%.4f) "
                    "control_plus_task_acc=(%.3f,%.3f,%.3f,%.4f,%.4f,%.4f)",
                    N, formation_mass_,
                    formation_cog_offset_.x(), formation_cog_offset_.y(), formation_cog_offset_.z(),
                    control_wrench_acc(0) + task_wrench_acc(0),
                    control_wrench_acc(1) + task_wrench_acc(1),
                    control_wrench_acc(2) + task_wrench_acc(2),
                    control_wrench_acc(3) + task_wrench_acc(3),
                    control_wrench_acc(4) + task_wrench_acc(4),
                    control_wrench_acc(5) + task_wrench_acc(5));

  // ---- One-shot deferred cascade setup ----
  // sendCascadeSetup() at mode switch runs BEFORE computeUnifiedAllocation(),
  // so integrated_map_inv_rot_ is still empty at that point. The allocation
  // matrix send silently fails, leaving spinal with the OLD independent-mode
  // matrix. thrustGainMapping() then maps cascade gains through the wrong
  // matrix → roll/pitch P/D ≈ 0 → pitch divergence.
  //
  // Fix: on the FIRST successful computation of integrated_map_inv_rot_,
  // resend the allocation matrix followed by cascade gains to this module's spinal.
  // Order matters: matrix first, then gains, so thrustGainMapping() uses
  // the correct matrix when processing the new gains.
  if (!cascade_alloc_sent_ && integrated_map_inv_rot_.rows() > 0 && has_cascade_gain_cache_) {
    bool matrix_sent = sendTorqueAllocationMatrixInvLocked();
    if (matrix_sent) {
      sendCascadeGains(cached_cascade_roll_p_, cached_cascade_roll_i_, cached_cascade_roll_d_,
                       cached_cascade_pitch_p_, cached_cascade_pitch_i_, cached_cascade_pitch_d_,
                       cached_cascade_yaw_d_);
      cascade_alloc_sent_ = true;
      ROS_INFO("[UnifiedCtrl] One-shot cascade resend: local allocation matrix (%ldx%ld) + "
               "gains(P_r=%.1f I_r=%.2f D_r=%.1f P_p=%.1f I_p=%.2f D_p=%.1f D_y=%.1f) sent to own spinal",
               integrated_map_inv_rot_.rows(), integrated_map_inv_rot_.cols(),
               cached_cascade_roll_p_, cached_cascade_roll_i_, cached_cascade_roll_d_,
               cached_cascade_pitch_p_, cached_cascade_pitch_i_, cached_cascade_pitch_d_,
               cached_cascade_yaw_d_);
    }
  }

  return true;
}

Eigen::VectorXd BeetleUnifiedController::buildSecondaryAllocationReference(
    const std::vector<int>& assembled_ids) const
{
  const int n_rotors = static_cast<int>(assembled_ids.size()) * motor_num_per_module_;
  Eigen::VectorXd ref = Eigen::VectorXd::Zero(rotor_coef_ * n_rotors);
  if (n_rotors <= 0 || rotor_coef_ <= 0 || formation_mass_ <= 0.0) return ref;

  for (size_t m = 0; m < assembled_ids.size(); m++) {
    ModuleModelDescriptor model;
    if (!getModuleModelDescriptor(assembled_ids[m], model)) return ref;
    const double hover_per_rotor = model.mass * aerial_robot_estimation::G / motor_num_per_module_;
    const double bounded_hover = std::max(0.0, std::min(hover_per_rotor, alloc_t_max_));
    const int module_col = static_cast<int>(m) * motor_num_per_module_ * rotor_coef_;
    for (int r = 0; r < motor_num_per_module_; r++) {
      ref(module_col + r * rotor_coef_ + rotor_coef_ - 1) = bounded_hover;
    }
  }

  if (internal_wrench_secondary_gain_ <= 0.0 || module_internal_wrench_comp_.empty()) {
    return ref;
  }

  const int cols_per_module = motor_num_per_module_ * rotor_coef_;
  const bool has_allocation_block =
      (integrated_map_.rows() == 6 && integrated_map_.cols() == ref.size());
  const double delta_limit = 0.25 * alloc_t_max_;
  Eigen::VectorXd internal_delta = Eigen::VectorXd::Zero(ref.size());

  for (size_t m = 0; m < assembled_ids.size(); m++) {
    auto it = module_internal_wrench_comp_.find(assembled_ids[m]);
    if (it == module_internal_wrench_comp_.end() || it->second.size() < 6) continue;

    const Eigen::VectorXd& comp = it->second;
    if (!comp.allFinite()) continue;

    const int module_col = static_cast<int>(m) * motor_num_per_module_ * rotor_coef_;
    Eigen::VectorXd delta = Eigen::VectorXd::Zero(cols_per_module);

    if (has_allocation_block) {
      Eigen::VectorXd comp_acc = Eigen::VectorXd::Zero(6);
      comp_acc.head(3) = comp.head(3) / formation_mass_;
      comp_acc.tail(3) = formation_inertia_.inverse() * comp.tail(3);
      const Eigen::MatrixXd module_map =
          integrated_map_.middleCols(module_col, cols_per_module);
      delta = aerial_robot_model::pseudoinverse(module_map) *
              (internal_wrench_secondary_gain_ * comp_acc);
    } else if (rotor_coef_ >= 2) {
      // Fallback for early calls before the formation map is ready.
      for (int r = 0; r < motor_num_per_module_; r++) {
        delta(r * rotor_coef_) =
            internal_wrench_secondary_gain_ * comp(0) / motor_num_per_module_;
        delta(r * rotor_coef_ + rotor_coef_ - 1) =
            internal_wrench_secondary_gain_ * comp(2) / motor_num_per_module_;
      }
    }

    for (int r = 0; r < motor_num_per_module_; r++) {
      const int base = module_col + r * rotor_coef_;
      for (int c = 0; c < rotor_coef_; c++) {
        const int idx = base + c;
        internal_delta(idx) +=
            std::max(-delta_limit, std::min(delta(r * rotor_coef_ + c), delta_limit));
      }
    }
  }
  if (has_allocation_block && internal_delta.size() == integrated_map_.cols()) {
    // Reuse the member pseudoinverse already computed in
    // computeUnifiedAllocation() for this same integrated_map_; only recompute
    // if it is not dimensionally consistent (e.g. early/stale call).
    const bool inv_consistent =
        (integrated_map_inv_.rows() == integrated_map_.cols() &&
         integrated_map_inv_.cols() == integrated_map_.rows());
    const Eigen::MatrixXd recomputed_pinv =
        inv_consistent ? Eigen::MatrixXd()
                       : aerial_robot_model::pseudoinverse(integrated_map_);
    const Eigen::MatrixXd& map_pinv =
        inv_consistent ? integrated_map_inv_ : recomputed_pinv;
    internal_delta =
        (Eigen::MatrixXd::Identity(internal_delta.size(), internal_delta.size()) -
         map_pinv * integrated_map_) * internal_delta;
  }
  for (int idx = 0; idx < ref.size(); idx++) {
    if (rotor_coef_ == 2 && (idx % rotor_coef_) == rotor_coef_ - 1) {
      ref(idx) = std::max(0.0, std::min(ref(idx) + internal_delta(idx), alloc_t_max_));
    } else {
      ref(idx) = std::max(-alloc_t_max_, std::min(ref(idx) + internal_delta(idx), alloc_t_max_));
    }
  }
  return ref;
}

double BeetleUnifiedController::getModuleAllocationWeight(int module_id) const
{
  if (module_id > 0 &&
      module_id <= static_cast<int>(alloc_module_weights_.size()) &&
      std::isfinite(alloc_module_weights_[module_id - 1])) {
    return std::max(0.0, alloc_module_weights_[module_id - 1]);
  }
  return 1.0;
}

bool BeetleUnifiedController::buildInterfaceLoadMatrix(
    const std::vector<int>& assembled_ids,
    Eigen::MatrixXd& interface_load_matrix,
    std::vector<std::pair<int, int>>& interface_cuts) const
{
  interface_load_matrix.resize(0, 0);
  interface_cuts.clear();

  const int n_modules = static_cast<int>(assembled_ids.size());
  if (n_modules < 2 || rotor_coef_ <= 0 || motor_num_per_module_ <= 0) {
    return true;
  }

  const int total_cols = n_modules * motor_num_per_module_ * rotor_coef_;
  interface_load_matrix = Eigen::MatrixXd::Zero(6 * (n_modules - 1), total_cols);
  interface_cuts.reserve(n_modules - 1);

  std::vector<Eigen::MatrixXd> masked_rot_single = buildRotorMask();
  if (static_cast<int>(masked_rot_single.size()) < motor_num_per_module_) {
    interface_load_matrix.resize(0, 0);
    return false;
  }

  std::vector<Eigen::Vector3d> module_offsets(n_modules, Eigen::Vector3d::Zero());
  for (int m = 0; m < n_modules; m++) {
    if (!getCachedModuleOffsetFromLeader(assembled_ids[m], module_offsets[m])) {
      interface_load_matrix.resize(0, 0);
      interface_cuts.clear();
      return false;
    }
  }

  for (int cut = 0; cut < n_modules - 1; cut++) {
    const int left_id = assembled_ids[cut];
    const int right_id = assembled_ids[cut + 1];
    interface_cuts.emplace_back(left_id, right_id);

    // Proxy cut point: midpoint between adjacent module CoGs, expressed relative
    // to formation CoG in body coordinates. The sign is "right-side actuator
    // wrench about the cut"; diagnostics use norms, and symmetric bounds use
    // absolute components.
    const Eigen::Vector3d cut_point =
        0.5 * (module_offsets[cut] + module_offsets[cut + 1]) - formation_cog_offset_;
    const int row_start = 6 * cut;

    for (int m = cut + 1; m < n_modules; m++) {
      ModuleModelDescriptor model;
      if (!getModuleModelDescriptor(assembled_ids[m], model)) {
        interface_load_matrix.resize(0, 0);
        interface_cuts.clear();
        return false;
      }

      const Eigen::Vector3d module_offset_from_formation_cog =
          module_offsets[m] - formation_cog_offset_;
      const int module_col = m * motor_num_per_module_ * rotor_coef_;

      for (int r = 0; r < motor_num_per_module_; r++) {
        const Eigen::Vector3d rotor_pos =
            module_offset_from_formation_cog + model.rotor_origins_from_cog.at(r);
        const int dir = model.rotor_direction.at(r + 1);

        Eigen::MatrixXd wrench_map = Eigen::MatrixXd::Zero(6, 3);
        wrench_map.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();
        wrench_map.block(3, 0, 3, 3) =
            aerial_robot_model::skew(rotor_pos - cut_point)
            + dir * model.mf_rate * Eigen::Matrix3d::Identity();

        const int col_start = module_col + r * rotor_coef_;
        interface_load_matrix.block(row_start, col_start, 6, rotor_coef_) =
            wrench_map * masked_rot_single[r];
      }
    }
  }

  return true;
}

void BeetleUnifiedController::publishInterfaceLoadDiagnostics(
    const Eigen::MatrixXd& interface_load_matrix,
    const std::vector<std::pair<int, int>>& interface_cuts,
    const Eigen::VectorXd& vectoring_f,
    const Eigen::VectorXd& interface_load_reference)
{
  if (interface_load_matrix.rows() == 0 ||
      interface_load_matrix.cols() != vectoring_f.size() ||
      static_cast<int>(interface_cuts.size()) * 6 != interface_load_matrix.rows()) {
    return;
  }
  const bool interface_diag_enabled =
      alloc_interface_force_weight_ > 0.0 ||
      alloc_interface_torque_weight_ > 0.0 ||
      alloc_interface_force_limit_ > 0.0 ||
      alloc_interface_torque_limit_ > 0.0;
  const bool has_subscriber = interface_load_pub_.getNumSubscribers() > 0;
  if (!interface_diag_enabled && !has_subscriber) return;

  Eigen::VectorXd load = interface_load_matrix * vectoring_f;
  Eigen::VectorXd load_err = load;
  const bool has_reference = (interface_load_reference.size() == load.size());
  if (has_reference) load_err -= interface_load_reference;
  std_msgs::Float32MultiArray msg;
  if (has_subscriber) msg.data.reserve(interface_cuts.size() * 10);

  double max_force = 0.0;
  double max_torque = 0.0;
  double max_force_err = 0.0;
  double max_torque_err = 0.0;
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(2);

  for (size_t i = 0; i < interface_cuts.size(); i++) {
    const Eigen::Vector3d force = load.segment<3>(6 * i);
    const Eigen::Vector3d torque = load.segment<3>(6 * i + 3);
    Eigen::Vector3d force_err = Eigen::Vector3d::Zero();
    Eigen::Vector3d torque_err = Eigen::Vector3d::Zero();
    if (has_reference) {
      force_err = load_err.segment<3>(6 * i);
      torque_err = load_err.segment<3>(6 * i + 3);
    }
    const double fn = force.norm();
    const double tn = torque.norm();
    max_force = std::max(max_force, fn);
    max_torque = std::max(max_torque, tn);
    max_force_err = std::max(max_force_err, force_err.norm());
    max_torque_err = std::max(max_torque_err, torque_err.norm());

    if (has_subscriber) {
      msg.data.push_back(static_cast<float>(interface_cuts[i].first));
      msg.data.push_back(static_cast<float>(interface_cuts[i].second));
      msg.data.push_back(static_cast<float>(force.x()));
      msg.data.push_back(static_cast<float>(force.y()));
      msg.data.push_back(static_cast<float>(force.z()));
      msg.data.push_back(static_cast<float>(torque.x()));
      msg.data.push_back(static_cast<float>(torque.y()));
      msg.data.push_back(static_cast<float>(torque.z()));
      msg.data.push_back(static_cast<float>(fn));
      msg.data.push_back(static_cast<float>(tn));
    }

    if (interface_diag_enabled) {
      if (i > 0) ss << " ";
      ss << interface_cuts[i].first << "-" << interface_cuts[i].second
         << ":F=(" << force.x() << "," << force.y() << "," << force.z()
         << ")|F|=" << fn
         << " T=(" << torque.x() << "," << torque.y() << "," << torque.z()
         << ")|T|=" << tn;
      if (has_reference) {
        ss << " err|F/T|=" << force_err.norm() << "/" << torque_err.norm();
      }
    }
  }

  if (has_subscriber) interface_load_pub_.publish(msg);
  if (!interface_diag_enabled) return;

  ROS_INFO_THROTTLE(
      5.0,
      "[UnifiedCtrl InterfaceLoad] actuator_cut_proxy max|F|=%.2fN max|T|=%.2fNm "
      "max|errF|=%.2fN max|errT|=%.2fNm limits(F/T)=%.2f/%.2f "
      "weights(F/T)=%.3g/%.3g cuts=[%s]",
      max_force, max_torque, max_force_err, max_torque_err,
      alloc_interface_force_limit_, alloc_interface_torque_limit_,
      alloc_interface_force_weight_, alloc_interface_torque_weight_,
      ss.str().c_str());
}

void BeetleUnifiedController::batteryVoltageCallback(const std_msgs::Float32ConstPtr& msg)
{
  if (!msg || !std::isfinite(msg->data) || msg->data <= 0.0f) return;
  battery_voltage_ = msg->data;
  battery_voltage_received_ = true;
}

double BeetleUnifiedController::convertThrustToPwmDuty(double thrust) const
{
  if (!std::isfinite(thrust)) thrust = 0.0;
  thrust = std::max(0.0, thrust);

  double voltage = 0.0;
  if (battery_voltage_received_ && std::isfinite(battery_voltage_) && battery_voltage_ > 0.0) {
    voltage = battery_voltage_;
  } else if (!pinv_motor_info_.empty() &&
             std::isfinite(pinv_motor_info_.front().voltage) &&
             pinv_motor_info_.front().voltage > 0.0f) {
    voltage = pinv_motor_info_.front().voltage;
  }

  int motor_ref_index = -1;
  double min_voltage_diff = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < pinv_motor_info_.size(); i++) {
    const double ref_voltage = pinv_motor_info_[i].voltage;
    if (!std::isfinite(ref_voltage) || ref_voltage <= 0.0) continue;
    const double voltage_diff = std::abs(voltage - ref_voltage);
    if (voltage_diff < min_voltage_diff) {
      motor_ref_index = static_cast<int>(i);
      min_voltage_diff = voltage_diff;
    }
  }

  const double pwm_min = std::max(0.0, pinv_pwm_min_);
  const double pwm_max = std::max(pwm_min, pinv_pwm_max_);

  if (motor_ref_index >= 0 && voltage > 0.0) {
    const spinal::MotorInfo& info = pinv_motor_info_[motor_ref_index];
    double v_factor = 1.0;
    switch (pinv_pwm_conversion_mode_) {
      case spinal::MotorInfo::SQRT_MODE:
        v_factor = (info.voltage / voltage) * (info.voltage / voltage);
        break;
      case spinal::MotorInfo::POLYNOMINAL_MODE:
        v_factor = (info.voltage / voltage) * std::sqrt(info.voltage / voltage);
        break;
      default:
        break;
    }

    const double scaled_thrust = std::max(0.0, v_factor * thrust);
    double target_pwm_percent = std::numeric_limits<double>::quiet_NaN();
    switch (pinv_pwm_conversion_mode_) {
      case spinal::MotorInfo::SQRT_MODE: {
        const double c0 = info.polynominal[0];
        const double c1 = info.polynominal[1];
        const double c2 = info.polynominal[2];
        const double disc = c1 * c1 - 40.0 * c2 * (c0 - scaled_thrust);
        if (std::abs(c2) > 1e-9 && disc >= 0.0) {
          target_pwm_percent = (-c1 + std::sqrt(disc)) / (2.0 * c2);
        }
        break;
      }
      case spinal::MotorInfo::POLYNOMINAL_MODE: {
        const double tenth_scaled_thrust = scaled_thrust * 0.1;
        target_pwm_percent = info.polynominal[4];
        for (int j = 3; j >= 0; j--) {
          target_pwm_percent = target_pwm_percent * tenth_scaled_thrust + info.polynominal[j];
        }
        break;
      }
      default:
        break;
    }

    if (std::isfinite(target_pwm_percent)) {
      return target_pwm_percent / 100.0;
    }
  }

  const double t_max = std::max(alloc_t_max_, 1e-6);
  const double ratio = std::max(0.0, std::min(thrust / t_max, 1.0));
  return pwm_min + (pwm_max - pwm_min) * std::sqrt(ratio);
}

double BeetleUnifiedController::predictThrustLimit() const
{
  double voltage = 0.0;
  if (battery_voltage_received_ && std::isfinite(battery_voltage_) && battery_voltage_ > 0.0) {
    voltage = battery_voltage_;
  } else if (!pinv_motor_info_.empty() &&
             std::isfinite(pinv_motor_info_.front().voltage) &&
             pinv_motor_info_.front().voltage > 0.0f) {
    voltage = pinv_motor_info_.front().voltage;
  }

  int motor_ref_index = -1;
  double min_voltage_diff = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < pinv_motor_info_.size(); i++) {
    const double ref_voltage = pinv_motor_info_[i].voltage;
    if (!std::isfinite(ref_voltage) || ref_voltage <= 0.0) continue;
    const double voltage_diff = std::abs(voltage - ref_voltage);
    if (voltage_diff < min_voltage_diff) {
      motor_ref_index = static_cast<int>(i);
      min_voltage_diff = voltage_diff;
    }
  }

  if (motor_ref_index >= 0 && voltage > 0.0) {
    const spinal::MotorInfo& info = pinv_motor_info_[motor_ref_index];
    double v_factor = 1.0;
    switch (pinv_pwm_conversion_mode_) {
      case spinal::MotorInfo::SQRT_MODE:
        v_factor = (info.voltage / voltage) * (info.voltage / voltage);
        break;
      case spinal::MotorInfo::POLYNOMINAL_MODE:
        v_factor = (info.voltage / voltage) * std::sqrt(info.voltage / voltage);
        break;
      default:
        break;
    }

    if (std::isfinite(info.max_thrust) && info.max_thrust > 0.0 &&
        std::isfinite(v_factor) && v_factor > 0.0) {
      return info.max_thrust / v_factor;
    }
  }

  return std::max(alloc_t_max_, 0.0);
}

uint16_t BeetleUnifiedController::predictPwmFromThrust(double thrust) const
{
  double pwm_min = std::max(0.0, pinv_pwm_min_);
  if (pinv_pwm_min_thrust_ > 0.0) {
    const double min_thrust_pwm = convertThrustToPwmDuty(pinv_pwm_min_thrust_);
    if (std::isfinite(min_thrust_pwm) && min_thrust_pwm > 0.0) {
      pwm_min = min_thrust_pwm;
    }
  }
  const double pwm_max = std::max(pwm_min, pinv_pwm_max_);
  const double pwm_norm =
      std::max(pwm_min, std::min(convertThrustToPwmDuty(thrust), pwm_max));
  const double pwm_us = std::max(0.0, std::min(2000.0 * pwm_norm, 65535.0));
  return static_cast<uint16_t>(std::lround(pwm_us));
}

bool BeetleUnifiedController::buildPwmPredictionMsg(
    const Eigen::VectorXd& vectoring_f,
    const std::vector<int>& assembled_ids,
    spinal::Pwms& msg) const
{
  if (!vectoring_f.allFinite()) return false;

  const int my_id = navigator_ ? navigator_->getMyID() : 0;
  int module_index = -1;
  for (size_t i = 0; i < assembled_ids.size(); i++) {
    if (assembled_ids[i] == my_id) {
      module_index = static_cast<int>(i);
      break;
    }
  }
  if (module_index < 0) return false;

  const int elems_per_module = motor_num_per_module_ * rotor_coef_;
  const int col_start = module_index * elems_per_module;
  if (vectoring_f.size() < col_start + elems_per_module) return false;

  msg.motor_value.resize(motor_num_per_module_);
  for (int r = 0; r < motor_num_per_module_; r++) {
    const int idx = col_start + r * rotor_coef_;
    const double thrust = vectoring_f.segment(idx, rotor_coef_).norm();
    msg.motor_value[r] = predictPwmFromThrust(thrust);
  }
  return true;
}

bool BeetleUnifiedController::buildThrustMarginMsg(
    const Eigen::VectorXd& vectoring_f,
    const std::vector<int>& assembled_ids,
    std_msgs::Float32MultiArray& msg) const
{
  if (!vectoring_f.allFinite()) return false;

  const int my_id = navigator_ ? navigator_->getMyID() : 0;
  int module_index = -1;
  for (size_t i = 0; i < assembled_ids.size(); i++) {
    if (assembled_ids[i] == my_id) {
      module_index = static_cast<int>(i);
      break;
    }
  }
  if (module_index < 0) return false;

  const int elems_per_module = motor_num_per_module_ * rotor_coef_;
  const int col_start = module_index * elems_per_module;
  if (vectoring_f.size() < col_start + elems_per_module) return false;

  const double thrust_limit = predictThrustLimit();
  if (!std::isfinite(thrust_limit)) return false;

  msg.data.resize(motor_num_per_module_);
  for (int r = 0; r < motor_num_per_module_; r++) {
    const int idx = col_start + r * rotor_coef_;
    const double thrust = vectoring_f.segment(idx, rotor_coef_).norm();
    msg.data[r] = static_cast<float>(thrust_limit - thrust);
  }
  return true;
}

void BeetleUnifiedController::publishAllocationPwmPredictions(
    const Eigen::VectorXd& qp_vectoring_f,
    const Eigen::VectorXd& pinv_vectoring_f,
    const std::vector<int>& assembled_ids)
{
  if (pinv_pwm_pred_pub_interval_ <= 0.0) return;

  const double now = ros::Time::now().toSec();
  if (last_pinv_pwm_pred_pub_time_ >= 0.0 &&
      now - last_pinv_pwm_pred_pub_time_ < pinv_pwm_pred_pub_interval_) {
    return;
  }

  bool published = false;
  if (use_constrained_alloc_) {
    spinal::Pwms qp_msg;
    if (buildPwmPredictionMsg(qp_vectoring_f, assembled_ids, qp_msg)) {
      qp_pwm_pred_pub_.publish(qp_msg);
      published = true;
    }

    std_msgs::Float32MultiArray qp_margin_msg;
    if (buildThrustMarginMsg(qp_vectoring_f, assembled_ids, qp_margin_msg)) {
      qp_thrust_margin_pub_.publish(qp_margin_msg);
      published = true;
    }
  }

  spinal::Pwms pinv_msg;
  if (buildPwmPredictionMsg(pinv_vectoring_f, assembled_ids, pinv_msg)) {
    pinv_pwm_pred_pub_.publish(pinv_msg);
    published = true;
  }

  std_msgs::Float32MultiArray pinv_margin_msg;
  if (buildThrustMarginMsg(pinv_vectoring_f, assembled_ids, pinv_margin_msg)) {
    pinv_thrust_margin_pub_.publish(pinv_margin_msg);
    published = true;
  }

  if (published) last_pinv_pwm_pred_pub_time_ = now;
}

bool BeetleUnifiedController::buildAllocationConstraints(
    const Eigen::MatrixXd& alloc_matrix,
    const Eigen::MatrixXd& interface_load_matrix,
    int n_cols, int n_rotors, int n_constraints,
    double cos_limit, double sin_limit, int thrust_poly_edges,
    bool use_rate_bound, bool use_direction_rate_bound,
    int n_interface_rows,
    const std::vector<int>& task_priority_rows,
    const Eigen::VectorXd& task_priority_target,
    const std::vector<int>& priority_rows,
    const Eigen::VectorXd& priority_target,
    std::vector<Eigen::Triplet<double>>& C_trips,
    Eigen::VectorXd& lb, Eigen::VectorXd& ub) const
{
  // C * f ∈ [lb, ub]
  const int n_gimbal_rows = (rotor_coef_ == 2) ? 2 * n_rotors : 0;
  const int n_thrust_rows = thrust_poly_edges * n_rotors;
  const int n_bound_rows = n_cols;
  const int n_rate_rows = use_rate_bound ? n_cols : 0;
  const int n_direction_rate_rows = use_direction_rate_bound ? 2 * n_rotors : 0;
  const int n_task_priority_rows = static_cast<int>(task_priority_rows.size());
  const int n_priority_rows = static_cast<int>(priority_rows.size());
  C_trips.clear();
  C_trips.reserve(n_gimbal_rows * 2 + n_thrust_rows * 2 + n_bound_rows +
                  n_rate_rows + n_direction_rate_rows * 2 +
                  n_interface_rows * n_cols +
                  (n_task_priority_rows + n_priority_rows) * n_cols);
  lb.resize(n_constraints);
  ub.resize(n_constraints);

  int row = 0;

  // (a) Gimbal angle constraints (only for rotor_coef == 2)
  // Convention: f_i = [f_x, f_z], gimbal angle θ = atan2(-f_x, f_z)
  // |θ| ≤ θ_max  ⟺  cos(θ_max)*f_x + sin(θ_max)*f_z ≥ 0
  //              AND -cos(θ_max)*f_x + sin(θ_max)*f_z ≥ 0
  // (valid when f_z ≥ 0, which is enforced by component bounds)
  if (rotor_coef_ == 2) {
    for (int i = 0; i < n_rotors; i++) {
      int fx_idx = rotor_coef_ * i;      // f_x index
      int fz_idx = rotor_coef_ * i + 1;  // f_z index

      // Row: cos_limit * f_x + sin_limit * f_z ≥ 0
      C_trips.emplace_back(row, fx_idx, cos_limit);
      C_trips.emplace_back(row, fz_idx, sin_limit);
      lb(row) = 0.0;
      ub(row) = OsqpEigen::INFTY;
      row++;

      // Row: -cos_limit * f_x + sin_limit * f_z ≥ 0
      C_trips.emplace_back(row, fx_idx, -cos_limit);
      C_trips.emplace_back(row, fz_idx, sin_limit);
      lb(row) = 0.0;
      ub(row) = OsqpEigen::INFTY;
      row++;
    }
  }

  // (b) Per-rotor thrust magnitude constraints.
  // OSQP accepts only linear constraints, so approximate the circle
  // sqrt(f_x^2 + f_z^2) <= T_max with an inscribed regular polygon.
  if (rotor_coef_ == 2 && thrust_poly_edges > 0) {
    const double thrust_poly_bound = alloc_t_max_ * std::cos(M_PI / thrust_poly_edges);
    for (int i = 0; i < n_rotors; i++) {
      int fx_idx = rotor_coef_ * i;
      int fz_idx = rotor_coef_ * i + 1;
      for (int k = 0; k < thrust_poly_edges; k++) {
        const double angle = 2.0 * M_PI * static_cast<double>(k) /
                             static_cast<double>(thrust_poly_edges);
        const double nx = std::cos(angle);
        const double nz = std::sin(angle);
        if (std::abs(nx) > 1e-12) C_trips.emplace_back(row, fx_idx, nx);
        if (std::abs(nz) > 1e-12) C_trips.emplace_back(row, fz_idx, nz);
        lb(row) = -OsqpEigen::INFTY;
        ub(row) = thrust_poly_bound;
        row++;
      }
    }
  }

  // (c) Component bounds: identity rows
  for (int j = 0; j < n_cols; j++) {
    C_trips.emplace_back(row, j, 1.0);
    if (rotor_coef_ == 2 && (j % rotor_coef_ == 1)) {
      // f_z: must be non-negative (thrust points "up" in rotor frame)
      lb(row) = 0.0;
      ub(row) = alloc_t_max_;
    } else {
      // f_x (lateral component): symmetric bounds
      lb(row) = -alloc_t_max_;
      ub(row) = alloc_t_max_;
    }
    row++;
  }

  // (d) Optional per-cycle component rate bounds around previous allocation.
  if (use_rate_bound) {
    for (int j = 0; j < n_cols; j++) {
      C_trips.emplace_back(row, j, 1.0);
      lb(row) = prev_vectoring_f_(j) - alloc_rate_limit_;
      ub(row) = prev_vectoring_f_(j) + alloc_rate_limit_;
      row++;
    }
  }

  // (e) Optional per-cycle direction bounds. Unlike component rate limits,
  // these suppress servo direction flips without hard-limiting thrust magnitude.
  if (use_direction_rate_bound) {
    for (int i = 0; i < n_rotors; i++) {
      const int fx_idx = rotor_coef_ * i;
      const int fz_idx = rotor_coef_ * i + 1;
      const double prev_fx = prev_vectoring_f_(fx_idx);
      const double prev_fz = prev_vectoring_f_(fz_idx);
      const double theta_prev = std::atan2(-prev_fx, prev_fz);
      const double theta_center =
          std::max(-alloc_gimbal_limit_rad_,
                   std::min(theta_prev, alloc_gimbal_limit_rad_));
      const double theta_lo =
          std::max(theta_center - alloc_direction_rate_limit_rad_,
                   -alloc_gimbal_limit_rad_);
      const double theta_hi =
          std::min(theta_center + alloc_direction_rate_limit_rad_,
                   alloc_gimbal_limit_rad_);

      // Same cos/sin form as the gimbal-limit rows, to stay well-conditioned as
      // theta_hi/theta_lo approach +/- pi/2 (cos(theta) >= 0 there).
      C_trips.emplace_back(row, fx_idx, std::cos(theta_hi));
      C_trips.emplace_back(row, fz_idx, std::sin(theta_hi));
      lb(row) = 0.0;
      ub(row) = OsqpEigen::INFTY;
      row++;

      C_trips.emplace_back(row, fx_idx, -std::cos(theta_lo));
      C_trips.emplace_back(row, fz_idx, -std::sin(theta_lo));
      lb(row) = 0.0;
      ub(row) = OsqpEigen::INFTY;
      row++;
    }
  }

  // (f) Optional component-wise interface cut-load proxy bounds.
  if (n_interface_rows > 0) {
    for (int r = 0; r < interface_load_matrix.rows(); r++) {
      const bool force_row = (r % 6) < 3;
      const double limit = force_row ? alloc_interface_force_limit_
                                     : alloc_interface_torque_limit_;
      if (limit <= 0.0) continue;
      for (int c = 0; c < n_cols; c++) {
        const double v = interface_load_matrix(r, c);
        if (std::abs(v) > 1e-12) C_trips.emplace_back(row, c, v);
      }
      lb(row) = -limit;
      ub(row) = limit;
      row++;
    }
  }

  // (g) Task hard bands: satisfy active feedforward wrench axes first, then let
  // effort/rate/balancing costs choose the actuator distribution.
  if (n_task_priority_rows > 0) {
    for (int i = 0; i < n_task_priority_rows; i++) {
      const int task_row = task_priority_rows[i];
      const double tol = alloc_priority_tolerances_(task_row);
      for (int c = 0; c < n_cols; c++) {
        const double v = alloc_matrix(task_row, c);
        if (std::abs(v) > 1e-12) C_trips.emplace_back(row, c, v);
      }
      lb(row) = task_priority_target(task_row) - tol;
      ub(row) = task_priority_target(task_row) + tol;
      row++;
    }
  }

  // (h) Optional hard priority bands for non-task high-output allocation.
  // The band center can differ from the soft target so fast feedback artifacts
  // do not become hard constraints.
  if (n_priority_rows > 0) {
    for (int i = 0; i < n_priority_rows; i++) {
      const int priority_row = priority_rows[i];
      const double tol = alloc_priority_tolerances_(priority_row);
      for (int c = 0; c < n_cols; c++) {
        const double v = alloc_matrix(priority_row, c);
        if (std::abs(v) > 1e-12) C_trips.emplace_back(row, c, v);
      }
      lb(row) = priority_target(priority_row) - tol;
      ub(row) = priority_target(priority_row) + tol;
      row++;
    }
  }

  if (row != n_constraints) {
    ROS_WARN_THROTTLE(1.0, "[UnifiedCtrl QP] constraint row mismatch: built=%d expected=%d",
                      row, n_constraints);
    return false;
  }
  return true;
}

bool BeetleUnifiedController::solveFullVectorQP(
    const Eigen::MatrixXd& alloc_matrix,
    const Eigen::VectorXd& w_control,
    const Eigen::VectorXd& w_task,
    const Eigen::VectorXd& task_weights,
    const Eigen::VectorXd& w_feedback,
    const Eigen::VectorXd& feedback_weights,
    const Eigen::VectorXd& w_priority,
    const Eigen::VectorXd& secondary_ref,
    const std::vector<int>& assembled_ids,
    const Eigen::MatrixXd& interface_load_matrix,
    const Eigen::VectorXd& interface_load_reference,
    Eigen::VectorXd& vectoring_f_out)
{
  // Full-vector QP: decision variables are all force components f ∈ R^{n_cols}.
  // For 1-DOF gimbal: each rotor contributes 2 variables [f_x, f_z].
  //
  // Objective:  min_f  0.5 * f' * P * f + q' * f
  //   where w_des = w_control + w_task + w_feedback,
  //         W_eff is a single per-axis wrench-tracking weight (alloc_wrench_weights_);
  //         task/feedback weights only select hard-band rows, not soft tracking.
  //         P = A'W_eff A + effort + R + optional rate/lateral-rate/interface terms,
  //         q = -A'W_eff*w_des - R*f_ref - D'Wi*d_ref - smoothness refs.
  //         Active task-priority rows are represented as hard bands around
  //         w_control+w_task and removed from W_eff.
  //
  // This separates stabilization/control tracking, explicit task feedforward,
  // and low-weight observer residual feedback. Optional interface terms are a
  // model-based actuator-side cut-load proxy target, not a measured physical
  // connector load.
  //
  // Constraints (all linear, OSQP-compatible):
  //   Per rotor i (rotor_coef=2, gimbal_dof=1):
  //     (a) Gimbal angle:  cos(θ_max)*f_x + sin(θ_max)*f_z ≥ 0   (angle ≥ -θ_max)
  //                       -cos(θ_max)*f_x + sin(θ_max)*f_z ≥ 0   (angle ≤ +θ_max)
  //     (b) Thrust magnitude: inner polygon approximation of sqrt(f_x^2+f_z^2) ≤ T_max
  //     (c) Component bounds: -T_max ≤ f_x ≤ T_max,  0 ≤ f_z ≤ T_max
  //   Optional:
  //     (d) Optional component rate bounds: |f - f_prev| ≤ Δf_max
  //     (e) Optional gimbal direction bounds: θ ∈ [θ_prev - Δθ, θ_prev + Δθ]
  //     (f) Interface cut-load component bounds: |D*f| ≤ load_max
  //     (g) Task-priority bands: active task rows must remain near w_control+w_task
  //     (h) Priority bands: selected 6D wrench rows must remain near target

  const int n_cols = alloc_matrix.cols();
  if (n_cols == 0 || rotor_coef_ == 0 || n_cols % rotor_coef_ != 0) {
    ROS_WARN_THROTTLE(2.0, "[UnifiedCtrl QP] Invalid alloc_matrix cols=%d, rotor_coef=%d",
                      n_cols, rotor_coef_);
    return false;
  }
  if (w_control.size() != alloc_matrix.rows() ||
      !alloc_matrix.allFinite() ||
      !w_control.allFinite() ||
      (w_task.size() > 0 && !w_task.allFinite()) ||
      (task_weights.size() > 0 && !task_weights.allFinite()) ||
      (w_feedback.size() > 0 && !w_feedback.allFinite()) ||
      (feedback_weights.size() > 0 && !feedback_weights.allFinite()) ||
      (w_priority.size() > 0 && !w_priority.allFinite()) ||
      (secondary_ref.size() > 0 && !secondary_ref.allFinite()) ||
      (interface_load_matrix.size() > 0 && !interface_load_matrix.allFinite()) ||
      (interface_load_reference.size() > 0 && !interface_load_reference.allFinite()) ||
      !std::isfinite(alloc_t_max_) ||
      !std::isfinite(alloc_gimbal_limit_rad_)) {
    ROS_WARN_THROTTLE(
        1.0,
        "[UnifiedCtrl QP] reject non-finite or mismatched input: A=%dx%d wc=%d "
        "wt=%d tw=%d wf=%d fw=%d pr=%d sec=%d D=%dx%d dref=%d",
        static_cast<int>(alloc_matrix.rows()),
        static_cast<int>(alloc_matrix.cols()),
        static_cast<int>(w_control.size()),
        static_cast<int>(w_task.size()),
        static_cast<int>(task_weights.size()),
        static_cast<int>(w_feedback.size()),
        static_cast<int>(feedback_weights.size()),
        static_cast<int>(w_priority.size()),
        static_cast<int>(secondary_ref.size()),
        static_cast<int>(interface_load_matrix.rows()),
        static_cast<int>(interface_load_matrix.cols()),
        static_cast<int>(interface_load_reference.size()));
    return false;
  }

  const int n_rotors = n_cols / rotor_coef_;
  // Gimbal-angle constraint coefficients. Use (cos, sin) rather than (1, tan):
  // tan(theta_max) -> 1.6e16 as theta_max -> pi/2 (the default 90 deg limit),
  // which makes that constraint row astronomically scaled and wrecks the QP
  // conditioning. cos/sin stays bounded in [0,1] and is the same constraint
  // (multiply f_x + tan(theta)*f_z >= 0 by cos(theta) >= 0 for theta in [0,pi/2]).
  const double cos_limit = std::cos(alloc_gimbal_limit_rad_);
  const double sin_limit = std::sin(alloc_gimbal_limit_rad_);
  if (!std::isfinite(cos_limit) || !std::isfinite(sin_limit)) {
    ROS_WARN_THROTTLE(1.0, "[UnifiedCtrl QP] reject invalid gimbal limit %.3f rad",
                      alloc_gimbal_limit_rad_);
    return false;
  }

  // --- Count constraints ---
  // For rotor_coef == 2:
  //   2 gimbal angle rows + thrust polygon rows + 2 component-bound rows per rotor
  int n_gimbal_rows = (rotor_coef_ == 2) ? 2 * n_rotors : 0;
  const int thrust_poly_edges = (rotor_coef_ == 2 && alloc_t_max_ > 0.0) ? 16 : 0;
  int n_thrust_rows = thrust_poly_edges * n_rotors;
  int n_bound_rows = n_cols;  // one bound per variable
  const bool has_prev_alloc = (prev_vectoring_f_.size() == n_cols);
  const bool use_rate_bound = has_prev_alloc && alloc_rate_limit_ > 0.0;
  const bool use_direction_rate_bound =
      has_prev_alloc && rotor_coef_ == 2 && alloc_direction_rate_limit_rad_ > 0.0;
  const int n_rate_rows = use_rate_bound ? n_cols : 0;
  const int n_direction_rate_rows = use_direction_rate_bound ? 2 * n_rotors : 0;
  const bool has_interface_map =
      (interface_load_matrix.cols() == n_cols && interface_load_matrix.rows() > 0);
  int n_interface_rows = 0;
  if (has_interface_map &&
      (alloc_interface_force_limit_ > 0.0 || alloc_interface_torque_limit_ > 0.0)) {
    for (int r = 0; r < interface_load_matrix.rows(); r++) {
      const bool force_row = (r % 6) < 3;
      const double limit = force_row ? alloc_interface_force_limit_
                                     : alloc_interface_torque_limit_;
      if (limit > 0.0) n_interface_rows++;
    }
  }
  Eigen::VectorXd f_ref = Eigen::VectorXd::Zero(n_cols);
  if (secondary_ref.size() == n_cols) {
    f_ref = secondary_ref;
  }

  // --- Build Hessian P = A'WA + effort + weighted secondary/rate/interface terms ---
  Eigen::VectorXd wrench_weights = Eigen::VectorXd::Ones(alloc_matrix.rows());
  if (alloc_wrench_weights_.size() == alloc_matrix.rows()) {
    wrench_weights = alloc_wrench_weights_;
  }
  Eigen::VectorXd effective_task_weights = Eigen::VectorXd::Zero(alloc_matrix.rows());
  if (task_weights.size() == alloc_matrix.rows()) {
    effective_task_weights = task_weights;
  }
  for (int r = 0; r < effective_task_weights.size(); r++) {
    wrench_weights(r) = std::max(0.0, wrench_weights(r));
    effective_task_weights(r) = std::max(0.0, effective_task_weights(r));
  }
  Eigen::VectorXd desired_tracking_target = w_control;
  if (w_task.size() == alloc_matrix.rows()) {
    desired_tracking_target += w_task;
  }
  if (w_feedback.size() == alloc_matrix.rows()) {
    desired_tracking_target += w_feedback;
  }
  Eigen::VectorXd task_priority_target = w_control;
  if (w_task.size() == alloc_matrix.rows()) {
    task_priority_target += w_task;
  }

  std::vector<int> task_priority_rows;
  std::vector<bool> task_priority_selected(alloc_matrix.rows(), false);
  if (alloc_task_priority_enabled_ &&
      w_task.size() == alloc_matrix.rows() &&
      alloc_priority_tolerances_.size() == alloc_matrix.rows()) {
    for (int r = 0; r < alloc_matrix.rows(); r++) {
      if (alloc_priority_tolerances_(r) > 0.0 &&
          effective_task_weights(r) >= alloc_task_priority_min_weight_ &&
          std::abs(w_task(r)) > 1e-6) {
        task_priority_rows.push_back(r);
        task_priority_selected[r] = true;
      }
    }
  }

  const Eigen::VectorXd& priority_target =
      (w_priority.size() == alloc_matrix.rows()) ? w_priority : w_control;
  std::vector<int> priority_rows;
  if (alloc_priority_enabled_ && alloc_priority_tolerances_.size() == alloc_matrix.rows()) {
    for (int r = 0; r < alloc_matrix.rows(); r++) {
      if (task_priority_selected[r]) continue;
      // A zero priority center means "do not make this local feedback axis hard".
      if (alloc_priority_tolerances_(r) > 0.0 &&
          std::abs(priority_target(r)) > 1e-6) {
        priority_rows.push_back(r);
      }
    }
  }
  const int n_task_priority_rows = static_cast<int>(task_priority_rows.size());
  const int n_priority_rows = static_cast<int>(priority_rows.size());
  int n_constraints = n_gimbal_rows + n_thrust_rows + n_bound_rows + n_rate_rows +
                      n_direction_rate_rows + n_interface_rows +
                      n_task_priority_rows + n_priority_rows;

  // Single per-axis wrench-tracking weight on the composed target w_des.
  // Task/feedback weights only gate which rows are promoted to hard bands
  // (below); they no longer scale the soft tracking, so the per-axis weight is
  // a deliberate priority rather than a function of how many sources contribute.
  Eigen::VectorXd tracking_weights = wrench_weights;
  for (int row_idx : task_priority_rows) {
    tracking_weights(row_idx) = 0.0;
  }
  Eigen::MatrixXd tracking_weight_diag = tracking_weights.asDiagonal();
  Eigen::MatrixXd P_dense =
      alloc_matrix.transpose() * tracking_weight_diag * alloc_matrix;
  Eigen::VectorXd q_vec =
      -alloc_matrix.transpose() *
          (tracking_weight_diag * desired_tracking_target);

  if (alloc_effort_weight_ > 0.0) {
    // Center the effort penalty on the balanced+internal reference f_ref, not on
    // zero. Penalizing ||f||^2 toward zero would fight any internal force (f_int
    // in N(A) raises ||f|| without producing net wrench); centering on f_ref
    // leaves the intended internal force unpenalized while still regularizing
    // the wrench-producing part toward the balanced reference.
    P_dense.diagonal().array() += alloc_effort_weight_;
    q_vec -= alloc_effort_weight_ * f_ref;
  }

  for (int m = 0; m < static_cast<int>(assembled_ids.size()); m++) {
    const double module_weight = getModuleAllocationWeight(assembled_ids[m]);
    const double reg_weight = alloc_lambda_ * module_weight;
    const int module_col = m * motor_num_per_module_ * rotor_coef_;
    for (int r = 0; r < motor_num_per_module_ * rotor_coef_; r++) {
      const int idx = module_col + r;
      if (idx >= n_cols) continue;
      P_dense(idx, idx) += reg_weight;
      q_vec(idx) -= reg_weight * f_ref(idx);
    }
  }

  if (has_prev_alloc && alloc_rate_weight_ > 0.0) {
    P_dense.diagonal().array() += alloc_rate_weight_;
    q_vec -= alloc_rate_weight_ * prev_vectoring_f_;
  }

  if (has_prev_alloc && rotor_coef_ == 2 && alloc_lateral_rate_weight_ > 0.0) {
    for (int i = 0; i < n_rotors; i++) {
      const int fx_idx = rotor_coef_ * i;
      P_dense(fx_idx, fx_idx) += alloc_lateral_rate_weight_;
      q_vec(fx_idx) -= alloc_lateral_rate_weight_ * prev_vectoring_f_(fx_idx);
    }
  }

  // Module thrust-balance: discourage one module from doing all the work while
  // another idles, WITHOUT forcing a distribution or capping cooperation.
  // Penalize the spread of per-module vertical-thrust sums s_m = sum_{r in m} f_z
  // about their mean:  beta * sum_m (s_m - mean)^2 = beta * ||M S f||^2, where S
  // sums each module's f_z and M = I - (1/N) 11^T centers them. Pure quadratic
  // (the mean is intrinsic), so it only adds to the Hessian; the connector load
  // is bounded separately by the hard interface limits, leaving cooperation free
  // below those limits.
  if (alloc_module_balance_weight_ > 0.0 && rotor_coef_ == 2) {
    const int n_modules = static_cast<int>(assembled_ids.size());
    if (n_modules > 1) {
      Eigen::MatrixXd S = Eigen::MatrixXd::Zero(n_modules, n_cols);
      for (int m = 0; m < n_modules; m++) {
        const int module_col = m * motor_num_per_module_ * rotor_coef_;
        for (int r = 0; r < motor_num_per_module_; r++) {
          const int fz_idx = module_col + r * rotor_coef_ + rotor_coef_ - 1;
          if (fz_idx < n_cols) S(m, fz_idx) = 1.0;
        }
      }
      const Eigen::MatrixXd M =
          Eigen::MatrixXd::Identity(n_modules, n_modules) -
          (1.0 / n_modules) * Eigen::MatrixXd::Ones(n_modules, n_modules);
      P_dense += alloc_module_balance_weight_ * S.transpose() * M * S;
    }
  }

  if (has_interface_map &&
      (alloc_interface_force_weight_ > 0.0 || alloc_interface_torque_weight_ > 0.0)) {
    Eigen::VectorXd row_weights = Eigen::VectorXd::Zero(interface_load_matrix.rows());
    Eigen::VectorXd interface_target = Eigen::VectorXd::Zero(interface_load_matrix.rows());
    if (interface_load_reference.size() == interface_load_matrix.rows()) {
      interface_target = interface_load_reference;
    }
    for (int r = 0; r < interface_load_matrix.rows(); r++) {
      row_weights(r) = ((r % 6) < 3) ? alloc_interface_force_weight_
                                     : alloc_interface_torque_weight_;
    }
    P_dense += interface_load_matrix.transpose()
             * row_weights.asDiagonal()
             * interface_load_matrix;
    q_vec -= interface_load_matrix.transpose()
           * row_weights.asDiagonal()
           * interface_target;
  }

  // --- Build constraint matrix C and bounds [lb, ub] (C * f in [lb, ub]) ---
  std::vector<Eigen::Triplet<double>> C_trips;
  Eigen::VectorXd lb, ub;
  if (!buildAllocationConstraints(alloc_matrix, interface_load_matrix,
                                  n_cols, n_rotors, n_constraints,
                                  cos_limit, sin_limit, thrust_poly_edges,
                                  use_rate_bound, use_direction_rate_bound, n_interface_rows,
                                  task_priority_rows, task_priority_target,
                                  priority_rows, priority_target,
                                  C_trips, lb, ub)) {
    return false;
  }
  bool bounds_have_nan = false;
  for (int i = 0; i < n_constraints; i++) {
    if (std::isnan(lb(i)) || std::isnan(ub(i))) {
      bounds_have_nan = true;
      break;
    }
  }
  if (!P_dense.allFinite() || !q_vec.allFinite() || bounds_have_nan) {
    ROS_WARN_THROTTLE(1.0, "[UnifiedCtrl QP] reject non-finite dense problem data");
    return false;
  }

  // --- Build sparse matrices ---
  Eigen::SparseMatrix<double> P_sparse(n_cols, n_cols);
  {
    std::vector<Eigen::Triplet<double>> P_trips;
    for (int r = 0; r < n_cols; r++) {
      for (int c = r; c < n_cols; c++) {
        if (std::abs(P_dense(r, c)) > 1e-12)
          P_trips.emplace_back(r, c, P_dense(r, c));
      }
    }
    P_sparse.setFromTriplets(P_trips.begin(), P_trips.end());
    P_sparse.makeCompressed();
  }

  Eigen::SparseMatrix<double> C_sparse(n_constraints, n_cols);
  C_sparse.setFromTriplets(C_trips.begin(), C_trips.end());
  C_sparse.makeCompressed();

  auto sparsePatternMatches =
      [](const Eigen::SparseMatrix<double>& matrix,
         const std::vector<int>& outer,
         const std::vector<int>& inner,
         int prev_nnz) {
        if (prev_nnz != matrix.nonZeros()) return false;
        if (static_cast<int>(outer.size()) != matrix.outerSize() + 1) return false;
        if (static_cast<int>(inner.size()) != matrix.nonZeros()) return false;
        for (int i = 0; i < matrix.outerSize() + 1; i++) {
          if (outer[i] != matrix.outerIndexPtr()[i]) return false;
        }
        for (int i = 0; i < matrix.nonZeros(); i++) {
          if (inner[i] != matrix.innerIndexPtr()[i]) return false;
        }
        return true;
      };
  auto storeSparsePattern =
      [](const Eigen::SparseMatrix<double>& matrix,
         std::vector<int>& outer,
         std::vector<int>& inner,
         int& nnz) {
        nnz = matrix.nonZeros();
        outer.resize(matrix.outerSize() + 1);
        for (int i = 0; i < matrix.outerSize() + 1; i++) {
          outer[i] = matrix.outerIndexPtr()[i];
        }
        inner.resize(matrix.nonZeros());
        for (int i = 0; i < matrix.nonZeros(); i++) {
          inner[i] = matrix.innerIndexPtr()[i];
        }
      };

  // --- Init or update solver ---
  const bool same_dimensions =
      (qp_n_vars_ == n_cols && qp_n_constraints_ == n_constraints);
  const bool same_pattern =
      same_dimensions &&
      sparsePatternMatches(P_sparse, qp_hessian_outer_, qp_hessian_inner_, qp_hessian_nnz_) &&
      sparsePatternMatches(C_sparse, qp_constraint_outer_, qp_constraint_inner_, qp_constraint_nnz_);
  bool need_init = !qp_solver_ || !same_pattern;
  if (need_init) {
    if (same_dimensions && qp_hessian_nnz_ >= 0 && qp_constraint_nnz_ >= 0) {
      ROS_INFO_THROTTLE(
          2.0,
          "[UnifiedCtrl QP] reinit solver because sparse pattern changed "
          "(P nnz %d->%d, C nnz %d->%d)",
          qp_hessian_nnz_, static_cast<int>(P_sparse.nonZeros()),
          qp_constraint_nnz_, static_cast<int>(C_sparse.nonZeros()));
    }
    // Recreate solver to avoid OsqpEigen "already set" stderr warnings
    // (clearSolver() does not reset the Data object's internal flags)
    qp_solver_ = std::make_unique<OsqpEigen::Solver>();
    qp_solver_->settings()->setVerbosity(false);
    qp_solver_->settings()->setWarmStart(true);
    qp_solver_->settings()->setMaxIteraction(500);
    qp_solver_->settings()->setAbsoluteTolerance(1e-5);
    qp_solver_->settings()->setRelativeTolerance(1e-4);
    qp_solver_->settings()->setPolish(true);
    qp_solver_->data()->setNumberOfVariables(n_cols);
    qp_solver_->data()->setNumberOfConstraints(n_constraints);

    if (!qp_solver_->data()->setHessianMatrix(P_sparse)) {
      ROS_WARN_THROTTLE(1.0, "[UnifiedCtrl QP] stage=qp_init_fail op=hessian");
      return false;
    }
    if (!qp_solver_->data()->setGradient(q_vec)) {
      ROS_WARN_THROTTLE(1.0, "[UnifiedCtrl QP] stage=qp_init_fail op=gradient");
      return false;
    }
    if (!qp_solver_->data()->setLinearConstraintsMatrix(C_sparse)) {
      ROS_WARN_THROTTLE(1.0, "[UnifiedCtrl QP] stage=qp_init_fail op=constraint_matrix");
      return false;
    }
    if (!qp_solver_->data()->setLowerBound(lb)) {
      ROS_WARN_THROTTLE(1.0, "[UnifiedCtrl QP] stage=qp_init_fail op=lower_bound");
      return false;
    }
    if (!qp_solver_->data()->setUpperBound(ub)) {
      ROS_WARN_THROTTLE(1.0, "[UnifiedCtrl QP] stage=qp_init_fail op=upper_bound");
      return false;
    }

    if (!qp_solver_->initSolver()) {
      ROS_WARN("[UnifiedCtrl QP] initSolver failed (n_vars=%d, n_constr=%d)",
               n_cols, n_constraints);
      return false;
    }
    qp_n_vars_ = n_cols;
    qp_n_constraints_ = n_constraints;
    storeSparsePattern(P_sparse, qp_hessian_outer_, qp_hessian_inner_, qp_hessian_nnz_);
    storeSparsePattern(C_sparse, qp_constraint_outer_, qp_constraint_inner_, qp_constraint_nnz_);
  } else {
    if (!qp_solver_->updateHessianMatrix(P_sparse)) {
      ROS_WARN_THROTTLE(1.0, "[UnifiedCtrl QP] stage=qp_update_fail op=hessian");
      resetQPState();
      return false;
    }
    if (!qp_solver_->updateGradient(q_vec)) {
      ROS_WARN_THROTTLE(1.0, "[UnifiedCtrl QP] stage=qp_update_fail op=gradient");
      resetQPState();
      return false;
    }
    if (!qp_solver_->updateLinearConstraintsMatrix(C_sparse)) {
      ROS_WARN_THROTTLE(1.0, "[UnifiedCtrl QP] stage=qp_update_fail op=constraint_matrix");
      resetQPState();
      return false;
    }
    if (!qp_solver_->updateBounds(lb, ub)) {
      ROS_WARN_THROTTLE(1.0, "[UnifiedCtrl QP] stage=qp_update_fail op=bounds");
      resetQPState();
      return false;
    }
  }

  // --- Solve ---
  if (!qp_solver_->solve()) {
    ROS_WARN_THROTTLE(
        1.0,
        "[UnifiedCtrl QP] stage=qp_solve_fail rows(task_priority=%d,priority=%d,rate=%d,dir=%d) "
        "wz=(soft=%.3f,priority=%.3f) rate=(w=%.1e,fx=%.1e,lim=%.2f,dir=%.1fdeg)",
        n_task_priority_rows, n_priority_rows, n_rate_rows, n_direction_rate_rows,
        w_control.size() > 2 ? w_control(2) : 0.0,
        priority_target.size() > 2 ? priority_target(2) : 0.0,
        alloc_rate_weight_, alloc_lateral_rate_weight_, alloc_rate_limit_,
        alloc_direction_rate_limit_rad_ * 180.0 / M_PI);
    return false;
  }
  Eigen::VectorXd f_sol = qp_solver_->getSolution();
  if (f_sol.size() != n_cols) {
    ROS_WARN_THROTTLE(1.0,
                      "[UnifiedCtrl QP] stage=qp_solution_reject size=%d expected=%d",
                      static_cast<int>(f_sol.size()), n_cols);
    return false;
  }
  if (!f_sol.allFinite()) {
    ROS_WARN_THROTTLE(1.0, "[UnifiedCtrl QP] stage=qp_solution_reject non_finite=1");
    resetQPState();
    return false;
  }

  vectoring_f_out = f_sol;
  prev_vectoring_f_ = f_sol;

  logQpDiagnostics(f_sol, alloc_matrix, desired_tracking_target, task_priority_target,
                   task_priority_rows, priority_target, priority_rows, n_rotors);

  return true;
}

void BeetleUnifiedController::logQpDiagnostics(
    const Eigen::VectorXd& f_sol,
    const Eigen::MatrixXd& alloc_matrix,
    const Eigen::VectorXd& desired_tracking_target,
    const Eigen::VectorXd& task_priority_target,
    const std::vector<int>& task_priority_rows,
    const Eigen::VectorXd& priority_target,
    const std::vector<int>& priority_rows,
    int n_rotors)
{
  // Short-term hardware diagnostic: expose hidden actuator saturation.
  // QP constrains fx/fz components, while the spinal receives sqrt(fx^2+fz^2).
  if (rotor_coef_ != 2) return;

  const double low_voltage_limit = 17.2;   // MotorInfo ref4 (21.2V)
  const double mid_voltage_limit = 18.44;  // MotorInfo ref3 (22.2V)
  const double model_limit = robot_model_ ? robot_model_->getThrustUpperLimit() : alloc_t_max_;
  double max_t = 0.0;
  double max_abs_angle_deg = 0.0;
  double max_abs_fx = 0.0;
  double max_fz = 0.0;
  double min_component_margin = std::numeric_limits<double>::infinity();
  int over_low = 0;
  int over_mid = 0;
  int over_alloc = 0;
  int over_model = 0;
  int near_component_bound = 0;
  for (int i = 0; i < n_rotors; i++) {
    double fx = f_sol(2 * i), fz = f_sol(2 * i + 1);
    double tmag = std::sqrt(fx * fx + fz * fz);
    double angle_deg = std::atan2(-fx, fz) * 180.0 / M_PI;

    max_t = std::max(max_t, tmag);
    max_abs_angle_deg = std::max(max_abs_angle_deg, std::abs(angle_deg));
    max_abs_fx = std::max(max_abs_fx, std::abs(fx));
    max_fz = std::max(max_fz, fz);
    const double component_margin = std::min(alloc_t_max_ - std::abs(fx), alloc_t_max_ - fz);
    min_component_margin = std::min(min_component_margin, component_margin);
    if (component_margin < 0.2) near_component_bound++;
    if (tmag > low_voltage_limit) over_low++;
    if (tmag > mid_voltage_limit) over_mid++;
    if (tmag > alloc_t_max_) over_alloc++;
    if (tmag > model_limit) over_model++;
  }

  const bool actuator_suspicious = over_low > 0 || over_alloc > 0 ||
                                   over_model > 0 || near_component_bound > 0;
  const double now = ros::Time::now().toSec();
  const bool qp_diag_due =
      last_qp_diag_log_time_ < 0.0 || now - last_qp_diag_log_time_ >= 0.5;
  if (!qp_diag_due) return;
  last_qp_diag_log_time_ = now;

  Eigen::VectorXd realized_acc = alloc_matrix * f_sol;
  Eigen::VectorXd desired_residual = realized_acc - desired_tracking_target;
  Eigen::VectorXd task_priority_residual = realized_acc - task_priority_target;
  double task_priority_residual_norm = 0.0;
  for (int task_row : task_priority_rows) {
    task_priority_residual_norm +=
        task_priority_residual(task_row) * task_priority_residual(task_row);
  }
  task_priority_residual_norm = std::sqrt(task_priority_residual_norm);
  Eigen::VectorXd priority_residual = realized_acc - priority_target;
  double priority_residual_norm = 0.0;
  for (int priority_row : priority_rows) {
    priority_residual_norm += priority_residual(priority_row) * priority_residual(priority_row);
  }
  priority_residual_norm = std::sqrt(priority_residual_norm);

  std::ostringstream thrust_stream;
  std::ostringstream angle_stream;
  thrust_stream << std::fixed << std::setprecision(1);
  angle_stream << std::fixed << std::setprecision(0);
  for (int i = 0; i < n_rotors; i++) {
    const double fx = f_sol(2 * i);
    const double fz = f_sol(2 * i + 1);
    if (i > 0) {
      thrust_stream << ",";
      angle_stream << ",";
    }
    thrust_stream << std::sqrt(fx * fx + fz * fz);
    angle_stream << std::atan2(-fx, fz) * 180.0 / M_PI;
  }

  const char* fmt =
      "[UnifiedCtrl QPDiag] desired_res=%.3f "
      "task_prio_res=%.3f task_rows=%d prio_res=%.3f prio_rows=%d "
      "max_t=%.2f max_angle=%.1fdeg "
      "max|fx|=%.2f max_fz=%.2f comp_margin_min=%.2f "
      "over_t(17.2/18.44/alloc/model)=%d/%d/%d/%d "
      "alloc_t_max=%.2f model_t_max=%.2f t=[%s] angle_deg=[%s]";
  if (actuator_suspicious) {
    ROS_WARN(fmt,
             desired_residual.norm(),
             task_priority_residual_norm, static_cast<int>(task_priority_rows.size()),
             priority_residual_norm, static_cast<int>(priority_rows.size()),
             max_t, max_abs_angle_deg,
             max_abs_fx, max_fz, min_component_margin,
             over_low, over_mid, over_alloc, over_model,
             alloc_t_max_, model_limit,
             thrust_stream.str().c_str(), angle_stream.str().c_str());
  } else {
    ROS_INFO(fmt,
             desired_residual.norm(),
             task_priority_residual_norm, static_cast<int>(task_priority_rows.size()),
             priority_residual_norm, static_cast<int>(priority_rows.size()),
             max_t, max_abs_angle_deg,
             max_abs_fx, max_fz, min_component_margin,
             over_low, over_mid, over_alloc, over_model,
             alloc_t_max_, model_limit,
             thrust_stream.str().c_str(), angle_stream.str().c_str());
  }
}

void BeetleUnifiedController::extractThrustAndGimbal(
    const Eigen::VectorXd& vectoring_f,
    const std::vector<int>& assembled_ids)
{
  int N = assembled_ids.size();
  module_commands_.clear();

  int col = 0;
  for (int m = 0; m < N; m++) {
    int module_id = assembled_ids[m];
    ModuleCommand cmd;
    cmd.full_thrusts.resize(motor_num_per_module_);
    cmd.gimbal_angles.resize(motor_num_per_module_ * gimbal_dof_);

    for (int r = 0; r < motor_num_per_module_; r++) {
      Eigen::VectorXd f_i = vectoring_f.segment(col, rotor_coef_);

      if (gimbal_dof_ == 1) {
        // Same as GimbalrotorController: no clamping, let atan2 handle all quadrants
        double thrust_mag = f_i.norm();
        double gimbal_angle = atan2(-f_i[0], f_i[1]);

        cmd.full_thrusts[r] = static_cast<float>(thrust_mag);
        cmd.gimbal_angles[r] = gimbal_angle;
      } else if (gimbal_dof_ == 2) {
        double thrust_mag = f_i.norm();
        cmd.full_thrusts[r] = static_cast<float>(thrust_mag);
        double gimbal_roll = atan2(-f_i[1], f_i[2]);
        double gimbal_pitch = atan2(f_i[0], -f_i[1] * sin(gimbal_roll) + f_i[2] * cos(gimbal_roll));
        cmd.gimbal_angles[2*r] = gimbal_roll;
        cmd.gimbal_angles[2*r+1] = gimbal_pitch;
      }

      col += rotor_coef_;
    }

    module_commands_[module_id] = cmd;
  }
}

int BeetleUnifiedController::getModuleIndex(int module_id) const
{
  std::vector<int> assembled_ids = navigator_->getAssemblyIds();
  for (size_t index = 0; index < assembled_ids.size(); index++) {
    if (assembled_ids[index] == module_id) return static_cast<int>(index);
  }
  return -1;
}

Eigen::VectorXd BeetleUnifiedController::getTargetVectoringForce() const
{
  std::lock_guard<std::mutex> lock(allocation_mutex_);
  return target_vectoring_f_;
}

std::map<int, BeetleUnifiedController::ModuleCommand> BeetleUnifiedController::getModuleCommands() const
{
  std::lock_guard<std::mutex> lock(allocation_mutex_);
  return module_commands_;
}

void BeetleUnifiedController::setCommandTargetRPY(const tf::Vector3& rpy)
{
  std::lock_guard<std::mutex> lock(allocation_mutex_);
  command_target_rpy_ = rpy;
}

bool BeetleUnifiedController::buildModuleThrustCommand(
    int module_id,
    spinal::FourAxisCommand& thrust_msg) const
{
  tf::Vector3 target_rpy;
  {
    std::lock_guard<std::mutex> lock(allocation_mutex_);

    int module_index = getModuleIndex(module_id);
    if (module_index < 0) return false;

    int elems_per_module = motor_num_per_module_ * rotor_coef_;
    int col_start = module_index * elems_per_module;
    if (target_vectoring_f_.size() < col_start + elems_per_module) return false;

    thrust_msg.base_thrust.resize(elems_per_module);
    for (int i = 0; i < elems_per_module; i++) {
      thrust_msg.base_thrust[i] = static_cast<float>(target_vectoring_f_(col_start + i));
    }
    thrust_msg.angles[0] = static_cast<float>(command_target_rpy_.x());
    thrust_msg.angles[1] = static_cast<float>(command_target_rpy_.y());
    thrust_msg.angles[2] = candidate_yaw_term_;
    target_rpy = command_target_rpy_;
  }

  const tf::Vector3 final_baselink_rpy = navigator_->getFinalTargetBaselinkRPY();
  const tf::Vector3 curr_baselink_rpy = navigator_->getCurrTargetBaselinkRPY();
  ROS_DEBUG_THROTTLE(
      2.0,
      "[UnifiedCtrl PitchChain id=%d] cmd_angles=(%.3f,%.3f,%.3f) "
      "target_rpy=(%.3f,%.3f,%.3f) final_baselink_rp=(%.3f,%.3f) curr_baselink_rp=(%.3f,%.3f)",
      module_id,
      thrust_msg.angles[0], thrust_msg.angles[1], thrust_msg.angles[2],
      target_rpy.x(), target_rpy.y(), target_rpy.z(),
      final_baselink_rpy.x(), final_baselink_rpy.y(),
      curr_baselink_rpy.x(), curr_baselink_rpy.y());
  return true;
}

bool BeetleUnifiedController::getRealizedModuleWrenchBody(
    int module_id,
    Eigen::VectorXd& realized) const
{
  realized = Eigen::VectorXd::Zero(6);
  std::lock_guard<std::mutex> lock(allocation_mutex_);

  int module_index = getModuleIndex(module_id);
  if (module_index < 0) return false;

  int elems_per_module = motor_num_per_module_ * rotor_coef_;
  int col_start = module_index * elems_per_module;
  if (target_vectoring_f_.size() < col_start + elems_per_module) return false;

  ModuleModelDescriptor model;
  if (!getModuleModelDescriptor(module_id, model)) return false;

  std::vector<Eigen::MatrixXd> masked_rot_single = buildRotorMask();
  if (static_cast<int>(masked_rot_single.size()) < motor_num_per_module_) return false;

  for (int r = 0; r < motor_num_per_module_; r++) {
    int block_start = col_start + r * rotor_coef_;
    Eigen::Vector3d f_i =
        masked_rot_single[r] * target_vectoring_f_.segment(block_start, rotor_coef_);
    realized.head(3) += f_i;
    realized.tail(3) += aerial_robot_model::skew(model.rotor_origins_from_cog.at(r)) * f_i
                        + model.rotor_direction.at(r + 1) * model.mf_rate * f_i;
  }

  return true;
}

bool BeetleUnifiedController::buildModuleTorqueAllocationMatrixInv(
    int module_id,
    spinal::TorqueAllocationMatrixInv& msg) const
{
  std::lock_guard<std::mutex> lock(allocation_mutex_);
  return buildModuleTorqueAllocationMatrixInvLocked(module_id, msg);
}

bool BeetleUnifiedController::buildModuleTorqueAllocationMatrixInvLocked(
    int module_id,
    spinal::TorqueAllocationMatrixInv& msg) const
{
  if (integrated_map_inv_rot_.rows() == 0) return false;

  int module_index = getModuleIndex(module_id);
  if (module_index < 0) return false;

  int rows_per_module = motor_num_per_module_ * rotor_coef_;
  int row_start = module_index * rows_per_module;
  if (integrated_map_inv_rot_.rows() < row_start + rows_per_module) return false;

  msg.rows.resize(rows_per_module);
  for (int i = 0; i < rows_per_module; i++) {
    msg.rows[i].x = static_cast<int16_t>(integrated_map_inv_rot_(row_start + i, 0) * 1000);
    msg.rows[i].y = static_cast<int16_t>(integrated_map_inv_rot_(row_start + i, 1) * 1000);
    msg.rows[i].z = static_cast<int16_t>(integrated_map_inv_rot_(row_start + i, 2) * 1000);
  }
  return true;
}

Eigen::VectorXd BeetleUnifiedController::getRealizedWrenchBody() const
{
  // Compute realized wrench from allocation: w_acc = A * f, then scale back to force/torque.
  //
  // integrated_map_ is in "acc-space":
  //   top 3 rows: (1/M) * force_allocation
  //   bottom 3 rows: I^{-1} * torque_allocation
  //
  // So: w_acc = integrated_map_ * target_vectoring_f_
  //     F = M * w_acc.head(3)
  //     T = I * w_acc.tail(3)

  Eigen::VectorXd realized = Eigen::VectorXd::Zero(6);
  std::lock_guard<std::mutex> lock(allocation_mutex_);

  if (integrated_map_.rows() != 6 || target_vectoring_f_.size() == 0) {
    return realized;
  }

  if (integrated_map_.cols() != target_vectoring_f_.size()) {
    ROS_WARN_THROTTLE(2.0, "[UnifiedCtrl] getRealizedWrenchBody: dimension mismatch: "
                      "map=%ldx%ld, vf=%ld",
                      integrated_map_.rows(), integrated_map_.cols(),
                      target_vectoring_f_.size());
    return realized;
  }

  // w_acc = A * f  (6D acceleration-space wrench)
  Eigen::VectorXd w_acc = integrated_map_ * target_vectoring_f_;

  // Convert from acc-space back to force/torque:
  //   F_body = M * w_acc.head(3)
  //   T_body = I * w_acc.tail(3)
  realized.head(3) = formation_mass_ * w_acc.head(3);
  realized.tail(3) = formation_inertia_ * w_acc.tail(3);

  return realized;
}

bool BeetleUnifiedController::sendTorqueAllocationMatrixInv()
{
  std::lock_guard<std::mutex> lock(allocation_mutex_);
  return sendTorqueAllocationMatrixInvLocked();
}

bool BeetleUnifiedController::sendTorqueAllocationMatrixInvLocked()
{
  // Send this module's rotational sub-block of the formation-level allocation
  // pseudoinverse to its spinal:
  // rows [m*motor_per_module*rotor_coef .. (m+1)*motor_per_module*rotor_coef) × 3 cols.
  //
  // Spinal uses this in thrustGainMapping():
  //   thrust_p_gain[i][axis] = torque_alloc_inv[i][axis] * torque_p_gain[axis]
  //   thrust_d_gain[i][axis] = torque_alloc_inv[i][axis] * torque_d_gain[axis]

  if (integrated_map_inv_rot_.rows() == 0) {
    ROS_WARN_THROTTLE(1.0, "[UnifiedCtrl] sendTorqueAllocationMatrixInv: inv_rot not computed yet");
    return false;
  }

  int rows_per_module = motor_num_per_module_ * rotor_coef_;
  int module_id = navigator_->getMyID();

  if (!module_torque_alloc_inv_pubs_.count(module_id)) {
    ROS_WARN_THROTTLE(1.0, "[UnifiedCtrl] sendTorqueAllocationMatrixInv: no publisher for module %d", module_id);
    return false;
  }

  spinal::TorqueAllocationMatrixInv msg;

  if (integrated_map_inv_rot_.cwiseAbs().maxCoeff() > INT16_MAX * 0.001f) {
    ROS_ERROR_THROTTLE(1.0, "[UnifiedCtrl] Torque Allocation Matrix overflow for module %d", module_id);
  }

  if (!buildModuleTorqueAllocationMatrixInvLocked(module_id, msg)) {
    return false;
  }

  module_torque_alloc_inv_pubs_[module_id].publish(msg);

  ROS_INFO("[UnifiedCtrl] Sent local torque_alloc_inv to module %d (%d rows), "
           "inv_rot total rows=%ld cols=%ld",
           module_id, rows_per_module,
           integrated_map_inv_rot_.rows(), integrated_map_inv_rot_.cols());
  return true;
}

void BeetleUnifiedController::sendCascadeGains(
    double roll_p, double roll_i, double roll_d,
    double pitch_p, double pitch_i, double pitch_d,
    double yaw_d)
{
  // Send torque-level P/I/D gains to this module's spinal.
  // Using motors.resize(1) → spinal stores as torque_{p,i,d}_gain and runs
  // thrustGainMapping() to compute per-motor gains using the
  // torque_allocation_matrix_inv.
  // v5: PC owns roll/pitch I and passes zero roll_i/pitch_i to spinal.
  int module_id = navigator_->getMyID();
  if (!module_rpy_gain_pubs_.count(module_id)) {
    ROS_WARN_THROTTLE(1.0, "[UnifiedCtrl] sendCascadeGains: no publisher for module %d", module_id);
    return;
  }

  spinal::RollPitchYawTerms rpy_gain_msg;
  rpy_gain_msg.motors.resize(1);
  rpy_gain_msg.motors[0].roll_p  = static_cast<int16_t>(roll_p  * 1000);
  rpy_gain_msg.motors[0].roll_i  = static_cast<int16_t>(roll_i  * 1000);
  rpy_gain_msg.motors[0].roll_d  = static_cast<int16_t>(roll_d  * 1000);
  rpy_gain_msg.motors[0].pitch_p = static_cast<int16_t>(pitch_p * 1000);
  rpy_gain_msg.motors[0].pitch_i = static_cast<int16_t>(pitch_i * 1000);
  rpy_gain_msg.motors[0].pitch_d = static_cast<int16_t>(pitch_d * 1000);
  rpy_gain_msg.motors[0].yaw_d   = static_cast<int16_t>(yaw_d   * 1000);

  module_rpy_gain_pubs_[module_id].publish(rpy_gain_msg);

  ROS_INFO_THROTTLE(2.0, "[UnifiedCtrl] Sent local cascade gains to module %d: "
                    "roll(P=%.2f,I=%.2f,D=%.2f) pitch(P=%.2f,I=%.2f,D=%.2f) yaw(D=%.2f)",
                    module_id, roll_p, roll_i, roll_d, pitch_p, pitch_i, pitch_d, yaw_d);
}

void BeetleUnifiedController::cacheCascadeGains(
    double roll_p, double roll_i, double roll_d,
    double pitch_p, double pitch_i, double pitch_d,
    double yaw_d)
{
  cached_cascade_roll_p_  = roll_p;
  cached_cascade_roll_i_  = roll_i;
  cached_cascade_roll_d_  = roll_d;
  cached_cascade_pitch_p_ = pitch_p;
  cached_cascade_pitch_i_ = pitch_i;
  cached_cascade_pitch_d_ = pitch_d;
  cached_cascade_yaw_d_   = yaw_d;
  has_cascade_gain_cache_ = true;
}

// ---- Formation Allocation Matrix (same math as Plan A / GimbalrotorController) ----

Eigen::MatrixXd BeetleUnifiedController::buildFormationAllocationMatrix(
    const std::vector<int>& assembled_ids,
    double formation_mass,
    const Eigen::Matrix3d& formation_inertia,
    const Eigen::Vector3d& formation_cog_offset)
{
  int N = assembled_ids.size();
  int total_rotors = N * motor_num_per_module_;
  double mass_inv = 1.0 / formation_mass;
  Eigen::Matrix3d inertia_inv = formation_inertia.inverse();

  Eigen::MatrixXd full_q_mat = Eigen::MatrixXd::Zero(6, 3 * total_rotors);

  Eigen::MatrixXd wrench_map = Eigen::MatrixXd::Zero(6, 3);
  wrench_map.block(0, 0, 3, 3) = Eigen::MatrixXd::Identity(3, 3);

  int col = 0;
  for (int m = 0; m < N; m++) {
    int module_id = assembled_ids[m];
    ModuleModelDescriptor model;
    if (!getModuleModelDescriptor(module_id, model)) {
      ROS_WARN_THROTTLE(1.0, "[UnifiedCtrl] Allocation blocked: missing ModuleModel id=%d", module_id);
      return Eigen::MatrixXd::Zero(6, rotor_coef_ * total_rotors);
    }

    Eigen::Vector3d module_offset = Eigen::Vector3d::Zero();
    if (!getCachedModuleOffsetFromLeader(module_id, module_offset)) {
      return Eigen::MatrixXd::Zero(6, rotor_coef_ * total_rotors);
    }

    for (int r = 0; r < motor_num_per_module_; r++) {
      Eigen::Vector3d rotor_pos =
          module_offset - formation_cog_offset + model.rotor_origins_from_cog.at(r);
      int dir = model.rotor_direction.at(r + 1);
      wrench_map.block(3, 0, 3, 3) =
          aerial_robot_model::skew(rotor_pos) + dir * model.mf_rate * Eigen::Matrix3d::Identity();
      full_q_mat.middleCols(col, 3) = wrench_map;
      col += 3;
    }
  }

  // Scale: force rows by 1/M, torque rows by I^{-1}
  full_q_mat.topRows(3) = mass_inv * full_q_mat.topRows(3);
  full_q_mat.bottomRows(3) = inertia_inv * full_q_mat.bottomRows(3);

  std::vector<Eigen::MatrixXd> masked_rot_single = buildRotorMask();

  // Block-diagonal integrated_rot
  int total_cols = rotor_coef_ * total_rotors;
  Eigen::MatrixXd integrated_rot = Eigen::MatrixXd::Zero(3 * total_rotors, total_cols);
  for (int m = 0; m < N; m++) {
    for (int r = 0; r < motor_num_per_module_; r++) {
      int rotor_idx = m * motor_num_per_module_ + r;
      integrated_rot.block(3 * rotor_idx, rotor_coef_ * rotor_idx,
                           3, rotor_coef_) = masked_rot_single[r];
    }
  }

  return full_q_mat * integrated_rot;
}

Eigen::Matrix3d BeetleUnifiedController::computeFormationInertia(
    const std::vector<int>& assembled_ids,
    const Eigen::Vector3d& formation_cog_offset)
{
  Eigen::Matrix3d formation_inertia = Eigen::Matrix3d::Zero();

  for (int module_id : assembled_ids) {
    ModuleModelDescriptor model;
    if (!getModuleModelDescriptor(module_id, model)) {
      ROS_WARN_THROTTLE(1.0, "[UnifiedCtrl] Inertia synthesis blocked: missing ModuleModel id=%d", module_id);
      return Eigen::Matrix3d::Zero();
    }
    Eigen::Vector3d d = Eigen::Vector3d::Zero();
    if (!getCachedModuleOffsetFromLeader(module_id, d)) {
      return Eigen::Matrix3d::Identity();
    }
    d -= formation_cog_offset;
    formation_inertia += model.inertia
                       + model.mass * (d.dot(d) * Eigen::Matrix3d::Identity() - d * d.transpose());
  }

  return formation_inertia;
}

} // namespace aerial_robot_control
