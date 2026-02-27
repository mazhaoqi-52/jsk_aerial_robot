// Unified controller for assembled beetle formation.
// PC does full 6-DOF PID + formation-wide pseudoinverse allocation.
// Spinal acts as pure PWM executor (zero attitude gains).

#include <beetle/control/beetle_unified_controller.h>
#include <tf_conversions/tf_kdl.h>
#include <tf_conversions/tf_eigen.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/SVD>
#include <sstream>
#include <iomanip>

namespace aerial_robot_control
{

BeetleUnifiedController::BeetleUnifiedController()
  : motor_num_per_module_(4),
    gimbal_dof_(1),
    rotor_coef_(2),
    gimbal_calc_in_fc_(false),
    formation_mass_(0),
    formation_cog_offset_(Eigen::Vector3d::Zero()),
    formation_inertia_(Eigen::Matrix3d::Zero())
{
}

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
    module_thrust_pubs_[i] = nh_.advertise<spinal::FourAxisCommand>(ns + "/unified_thrust_cmd", 1);
    module_gimbal_pubs_[i] = nh_.advertise<sensor_msgs::JointState>(ns + "/unified_gimbal_cmd", 1);
  }

  formation_wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("unified_control/formation_wrench", 1);
  formation_vectoring_f_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("unified_control/vectoring_force", 1);

  ROS_INFO("[UnifiedCtrl] Initialized: motor_per_module=%d, gimbal_dof=%d, rotor_coef=%d, gimbal_calc_in_fc=%d",
           motor_num_per_module_, gimbal_dof_, rotor_coef_, gimbal_calc_in_fc_);
}

void BeetleUnifiedController::rosParamInit()
{
  ros::NodeHandle control_nh(nh_, "controller");
  control_nh.param<int>("gimbal_dof", gimbal_dof_, 1);
  control_nh.param<bool>("gimbal_calc_in_fc", gimbal_calc_in_fc_, false);
}

bool BeetleUnifiedController::updateFormationGeometry()
{
  std::vector<int> assembled_ids = navigator_->getAssemblyIds();
  if (assembled_ids.empty()) return false;

  int N = assembled_ids.size();
  double single_mass = robot_model_->getMass();
  formation_mass_ = single_mass * N;
  int leader_id = navigator_->getLeaderID();
  std::string leader_cog_frame = navigator_->getMyName() + std::to_string(leader_id) + "/cog";

  Eigen::Vector3d cog_offset_sum = Eigen::Vector3d::Zero();
  for (int i = 0; i < N; i++) {
    int module_id = assembled_ids[i];
    if (module_id != leader_id) {
      try {
        std::string module_cog_frame = navigator_->getMyName() + std::to_string(module_id) + "/cog";
        geometry_msgs::TransformStamped tf_stamped =
            navigator_->getTfBuffer().lookupTransform(leader_cog_frame, module_cog_frame, ros::Time(0));
        cog_offset_sum.x() += tf_stamped.transform.translation.x;
        cog_offset_sum.y() += tf_stamped.transform.translation.y;
        cog_offset_sum.z() += tf_stamped.transform.translation.z;
      } catch (tf2::TransformException& ex) {
        ROS_WARN_THROTTLE(1.0, "[UnifiedCtrl] TF lookup for formation geometry failed: %s", ex.what());
        return false;
      }
    }
  }
  formation_cog_offset_ = cog_offset_sum / N;
  formation_inertia_ = computeFormationInertia(assembled_ids, formation_cog_offset_);
  return true;
}

bool BeetleUnifiedController::computeUnifiedAllocation(
    const Eigen::VectorXd& target_wrench_acc_cog,
    const Eigen::VectorXd& desired_ext_wrench)
{
  std::vector<int> assembled_ids = navigator_->getAssemblyIds();
  if (assembled_ids.empty()) return false;

  int N = assembled_ids.size();

  // Update formation geometry
  if (!updateFormationGeometry()) return false;

  // Build formation-wide allocation matrix (6 x rotor_coef*total_rotors)
  integrated_map_ = buildFormationAllocationMatrix(assembled_ids, formation_mass_,
                                                    formation_inertia_, formation_cog_offset_);

  // Pseudoinverse
  integrated_map_inv_ = aerial_robot_model::pseudoinverse(integrated_map_);

  // Add external wrench feedforward (convert from force/torque to acceleration)
  Eigen::VectorXd total_wrench_acc = target_wrench_acc_cog;
  if (desired_ext_wrench.size() == 6 && desired_ext_wrench.norm() > 1e-6) {
    double mass_inv = 1.0 / formation_mass_;
    Eigen::Matrix3d inertia_inv = formation_inertia_.inverse();
    total_wrench_acc.head(3) += mass_inv * desired_ext_wrench.head(3);
    total_wrench_acc.tail(3) += inertia_inv * desired_ext_wrench.tail(3);
  }

  // Allocate: vectoring_f = pseudoinverse * 6D_wrench_acc
  target_vectoring_f_ = integrated_map_inv_ * total_wrench_acc;

  // Extract per-rotor scalar thrust + gimbal angles
  extractThrustAndGimbal(target_vectoring_f_, assembled_ids);

  // --- Allocation diagnostics ---
  {
    // TODO#1: Verify allocation accuracy: realized vs demanded wrench
    Eigen::VectorXd realized = integrated_map_ * target_vectoring_f_;
    Eigen::VectorXd residual = realized - total_wrench_acc;
    ROS_WARN_THROTTLE(0.5, "[ALLOC_DIAG] demanded=(%.4f,%.4f,%.4f,%.5f,%.5f,%.5f) "
                      "realized=(%.4f,%.4f,%.4f,%.5f,%.5f,%.5f) "
                      "residual=(%.4f,%.4f,%.4f,%.5f,%.5f,%.5f)",
                      total_wrench_acc(0), total_wrench_acc(1), total_wrench_acc(2),
                      total_wrench_acc(3), total_wrench_acc(4), total_wrench_acc(5),
                      realized(0), realized(1), realized(2),
                      realized(3), realized(4), realized(5),
                      residual(0), residual(1), residual(2),
                      residual(3), residual(4), residual(5));

    // TODO#2: Per-rotor thrust and gimbal angle
    std::stringstream ss;
    ss << "[ROTOR_DIAG] ";
    int col = 0;
    for (int m = 0; m < N; m++) {
      int mid = assembled_ids[m];
      for (int r = 0; r < motor_num_per_module_; r++) {
        auto it = module_commands_.find(mid);
        if (it != module_commands_.end()) {
          ss << "m" << mid << "r" << r << ":(T=" 
             << std::fixed << std::setprecision(2) << it->second.full_thrusts[r]
             << ",g=" << std::setprecision(3) << it->second.gimbal_angles[r] << ") ";
        }
        col += rotor_coef_;
      }
    }
    ROS_WARN_THROTTLE(0.5, "%s", ss.str().c_str());

    // TODO#3: Condition number of integrated_map
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(integrated_map_);
    double cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size()-1);
    // TODO#4: Formation inertia diagonal
    ROS_WARN_THROTTLE(1.0, "[ALLOC_DIAG] cond_num=%.2f I_diag=(%.4f,%.4f,%.4f) mass=%.3f",
                      cond,
                      formation_inertia_(0,0), formation_inertia_(1,1), formation_inertia_(2,2),
                      formation_mass_);
  }

  ROS_INFO_THROTTLE(1.0, "[UnifiedCtrl] N=%d mass=%.3f cog_offset=(%.4f,%.4f,%.4f) "
                    "wrench_acc=(%.3f,%.3f,%.3f,%.4f,%.4f,%.4f)",
                    N, formation_mass_,
                    formation_cog_offset_.x(), formation_cog_offset_.y(), formation_cog_offset_.z(),
                    total_wrench_acc(0), total_wrench_acc(1), total_wrench_acc(2),
                    total_wrench_acc(3), total_wrench_acc(4), total_wrench_acc(5));

  return true;
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

void BeetleUnifiedController::publishCommands()
{
  for (const auto& kv : module_commands_) {
    int module_id = kv.first;
    const ModuleCommand& cmd = kv.second;

    // Publish scalar thrusts as FourAxisCommand.base_thrust
    // angles=[0,0,0]: spinal's attitude PID is zeroed, so these are ignored
    if (module_thrust_pubs_.count(module_id)) {
      spinal::FourAxisCommand thrust_msg;
      thrust_msg.base_thrust = cmd.full_thrusts;
      thrust_msg.angles[0] = 0;
      thrust_msg.angles[1] = 0;
      thrust_msg.angles[2] = 0;
      module_thrust_pubs_[module_id].publish(thrust_msg);
    }

    // Publish gimbal angles
    if (module_gimbal_pubs_.count(module_id)) {
      sensor_msgs::JointState gimbal_msg;
      gimbal_msg.header.stamp = ros::Time::now();
      gimbal_msg.position = cmd.gimbal_angles;
      module_gimbal_pubs_[module_id].publish(gimbal_msg);
    }
  }
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

  std::vector<Eigen::Vector3d> single_rotors_from_cog =
      robot_model_->getRotorsOriginFromCog<Eigen::Vector3d>();
  const auto& rotor_direction = robot_model_->getRotorDirection();
  const double m_f_rate = robot_model_->getMFRate();

  Eigen::MatrixXd wrench_map = Eigen::MatrixXd::Zero(6, 3);
  wrench_map.block(0, 0, 3, 3) = Eigen::MatrixXd::Identity(3, 3);

  int leader_id = navigator_->getLeaderID();
  std::string leader_cog_frame = navigator_->getMyName() + std::to_string(leader_id) + "/cog";

  int col = 0;
  for (int m = 0; m < N; m++) {
    int module_id = assembled_ids[m];

    Eigen::Vector3d module_offset = Eigen::Vector3d::Zero();
    if (module_id != leader_id) {
      try {
        std::string module_cog_frame = navigator_->getMyName() + std::to_string(module_id) + "/cog";
        geometry_msgs::TransformStamped tf_stamped =
            navigator_->getTfBuffer().lookupTransform(leader_cog_frame, module_cog_frame, ros::Time(0));
        module_offset << tf_stamped.transform.translation.x,
                         tf_stamped.transform.translation.y,
                         tf_stamped.transform.translation.z;
      } catch (tf2::TransformException& ex) {
        ROS_WARN_THROTTLE(1.0, "[UnifiedCtrl] TF lookup failed: %s", ex.what());
        return Eigen::MatrixXd::Zero(6, rotor_coef_ * total_rotors);
      }
    }

    for (int r = 0; r < motor_num_per_module_; r++) {
      Eigen::Vector3d rotor_pos = module_offset - formation_cog_offset + single_rotors_from_cog.at(r);
      int dir = rotor_direction.at(r + 1);
      wrench_map.block(3, 0, 3, 3) =
          aerial_robot_model::skew(rotor_pos) + dir * m_f_rate * Eigen::Matrix3d::Identity();
      full_q_mat.middleCols(col, 3) = wrench_map;
      col += 3;
    }
  }

  // Scale: force rows by 1/M, torque rows by I^{-1}
  full_q_mat.topRows(3) = mass_inv * full_q_mat.topRows(3);
  full_q_mat.bottomRows(3) = inertia_inv * full_q_mat.bottomRows(3);

  // Gimbal mask rotation matrix (same as GimbalrotorController)
  std::vector<KDL::Rotation> thrust_coords_rot =
      robot_model_->getThrustCoordRot<KDL::Rotation>();

  std::vector<Eigen::MatrixXd> masked_rot_single;
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
  int N = assembled_ids.size();
  double single_mass = robot_model_->getMass();
  Eigen::Matrix3d single_inertia = robot_model_->getInertia<Eigen::Matrix3d>();

  int leader_id = navigator_->getLeaderID();
  std::string leader_cog_frame = navigator_->getMyName() + std::to_string(leader_id) + "/cog";

  Eigen::Matrix3d formation_inertia = Eigen::Matrix3d::Zero();

  for (int i = 0; i < N; i++) {
    int module_id = assembled_ids[i];
    Eigen::Vector3d d = Eigen::Vector3d::Zero();
    if (module_id != leader_id) {
      try {
        std::string module_cog_frame = navigator_->getMyName() + std::to_string(module_id) + "/cog";
        geometry_msgs::TransformStamped tf_stamped =
            navigator_->getTfBuffer().lookupTransform(leader_cog_frame, module_cog_frame, ros::Time(0));
        d << tf_stamped.transform.translation.x,
             tf_stamped.transform.translation.y,
             tf_stamped.transform.translation.z;
      } catch (tf2::TransformException& ex) {
        ROS_WARN_THROTTLE(1.0, "[UnifiedCtrl] TF lookup for inertia failed: %s", ex.what());
        return Eigen::Matrix3d::Identity();
      }
    }
    d -= formation_cog_offset;
    formation_inertia += single_inertia
                       + single_mass * (d.dot(d) * Eigen::Matrix3d::Identity() - d * d.transpose());
  }

  return formation_inertia;
}

} // namespace aerial_robot_control
