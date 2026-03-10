// Unified controller for assembled beetle formation.
// PC outer loop (40Hz): 6-DOF PID + formation-wide pseudoinverse allocation.
// Spinal inner loop (1000Hz): P+D attitude tracking per-motor.
// PC retains I-term only for roll/pitch.

#include <beetle/control/beetle_unified_controller.h>
#include <tf_conversions/tf_kdl.h>
#include <tf_conversions/tf_eigen.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/TransformStamped.h>

namespace aerial_robot_control
{

BeetleUnifiedController::BeetleUnifiedController()
  : motor_num_per_module_(4),
    gimbal_dof_(1),
    rotor_coef_(2),
    gimbal_calc_in_fc_(false),
    formation_mass_(0),
    formation_cog_offset_(Eigen::Vector3d::Zero()),
    formation_inertia_(Eigen::Matrix3d::Zero()),
    target_roll_(0),
    target_pitch_(0),
    candidate_yaw_term_(0)
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
    module_torque_alloc_inv_pubs_[i] = nh_.advertise<spinal::TorqueAllocationMatrixInv>(
        ns + "/torque_allocation_matrix_inv", 1);
    module_rpy_gain_pubs_[i] = nh_.advertise<spinal::RollPitchYawTerms>(ns + "/rpy/gain", 1);
    module_gimbal_dof_pubs_[i] = nh_.advertise<std_msgs::UInt8>(ns + "/gimbal_dof", 1);
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

  // Split: rotational part (last 3 cols) for torque_allocation_matrix_inv
  integrated_map_inv_rot_ = integrated_map_inv_.rightCols(3);

  // Add external wrench feedforward (convert from force/torque to acceleration)
  Eigen::VectorXd total_wrench_acc = target_wrench_acc_cog;
  if (desired_ext_wrench.size() == 6 && desired_ext_wrench.norm() > 1e-6) {
    double mass_inv = 1.0 / formation_mass_;
    Eigen::Matrix3d inertia_inv = formation_inertia_.inverse();
    total_wrench_acc.head(3) += mass_inv * desired_ext_wrench.head(3);
    total_wrench_acc.tail(3) += inertia_inv * desired_ext_wrench.tail(3);
  }

  // Allocate: vectoring_f = pseudoinverse * 6D_wrench_acc
  // In cascade mode, the wrench_acc torque channels contain ONLY I-term
  // (P+D done by spinal). So base_thrust = allocation of (position PID + I-term only).
  target_vectoring_f_ = integrated_map_inv_ * total_wrench_acc;

  // Compute target angles for spinal inner loop
  // (underactuated: derive from position PID target acceleration)
  {
    // target_acc in body frame (first 3 elements of total_wrench_acc)
    Eigen::Vector3d target_acc_body = total_wrench_acc.head(3);
    target_roll_ = atan2(-target_acc_body.y(),
                         sqrt(target_acc_body.x() * target_acc_body.x() +
                              target_acc_body.z() * target_acc_body.z()));
    target_pitch_ = atan2(target_acc_body.x(), target_acc_body.z());
  }

  // Compute candidate yaw term for spinal yaw reconstruction
  {
    // Find max yaw column entry to scale yaw PID result (same logic as gimbalrotor)
    int yaw_col = 5; // yaw is the 6th column (index 5) in the 6-DOF wrench
    double max_yaw_scale = 0;
    int total_motors = assembled_ids.size() * motor_num_per_module_;
    for (int i = 0; i < total_motors; i++) {
      if (integrated_map_inv_(i * rotor_coef_, yaw_col) > max_yaw_scale)
        max_yaw_scale = integrated_map_inv_(i * rotor_coef_, yaw_col);
    }
    // candidate_yaw_term = yaw_pid_result * max_yaw_scale
    // (yaw PID result is wrench_acc(5), but we need the raw PID result before allocation)
    candidate_yaw_term_ = total_wrench_acc(5) * max_yaw_scale;
  }

  // Extract per-rotor scalar thrust + gimbal angles (for debug/visualization)
  extractThrustAndGimbal(target_vectoring_f_, assembled_ids);

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
  std::vector<int> assembled_ids = navigator_->getAssemblyIds();
  int leader_id = navigator_->getLeaderID();

  // In cascade mode (gimbal_calc_in_fc=true), send:
  //   base_thrust[motor_num * rotor_coef] = vectoring force components (NOT scalar magnitude)
  //   angles[0] = target_roll
  //   angles[1] = target_pitch
  //   angles[2] = candidate_yaw_term
  // Spinal will add roll_pitch_term (from its own P+D) to base_thrust,
  // then do sqrt+atan2 to decompose into scalar thrust + gimbal angle.

  int col = 0;
  for (size_t m = 0; m < assembled_ids.size(); m++) {
    int module_id = assembled_ids[m];

    // Skip LEADER — LEADER sends its own command in beetle_controller.cpp
    if (module_id == leader_id) {
      col += motor_num_per_module_ * rotor_coef_;
      continue;
    }

    if (module_thrust_pubs_.count(module_id)) {
      spinal::FourAxisCommand thrust_msg;

      // base_thrust: vectoring force components for this module's motors
      // size = motor_num_per_module_ * rotor_coef_ (e.g. 4*2=8 for gimbal_dof=1)
      thrust_msg.base_thrust.resize(motor_num_per_module_ * rotor_coef_);
      for (int r = 0; r < motor_num_per_module_; r++) {
        for (int c = 0; c < rotor_coef_; c++) {
          thrust_msg.base_thrust[r * rotor_coef_ + c] =
              static_cast<float>(target_vectoring_f_(col + r * rotor_coef_ + c));
        }
      }

      // Target angles for spinal inner-loop P+D tracking
      thrust_msg.angles[0] = target_roll_;
      thrust_msg.angles[1] = target_pitch_;
      thrust_msg.angles[2] = candidate_yaw_term_;

      module_thrust_pubs_[module_id].publish(thrust_msg);
    }

    // Also publish gimbal_dof=1 every frame to ensure spinal is in vectoring mode
    if (module_gimbal_dof_pubs_.count(module_id)) {
      std_msgs::UInt8 dof_msg;
      dof_msg.data = gimbal_dof_;
      module_gimbal_dof_pubs_[module_id].publish(dof_msg);
    }

    col += motor_num_per_module_ * rotor_coef_;
  }
}

bool BeetleUnifiedController::isAllocationSaturated() const
{
  if (module_commands_.empty()) return false;
  const double t_max = robot_model_->getThrustUpperLimit();
  const double t_min = robot_model_->getThrustLowerLimit();
  const double sat_margin = 0.05;
  const double t_upper = t_max * (1.0 - sat_margin);
  const double t_lower = t_min + t_max * sat_margin;
  for (const auto& kv : module_commands_) {
    for (float t : kv.second.full_thrusts) {
      if (t >= t_upper || t <= t_lower) return true;
    }
  }
  return false;
}

void BeetleUnifiedController::sendTorqueAllocationMatrixInv()
{
  // Send the rotational part of the formation-level allocation pseudoinverse
  // to each module's spinal. Each module receives only its sub-block:
  // rows [m*motor_per_module*rotor_coef .. (m+1)*motor_per_module*rotor_coef) × 3 cols.
  //
  // Spinal uses this in thrustGainMapping():
  //   thrust_p_gain[i][axis] = torque_alloc_inv[i][axis] * torque_p_gain[axis]
  //   thrust_d_gain[i][axis] = torque_alloc_inv[i][axis] * torque_d_gain[axis]

  if (integrated_map_inv_rot_.rows() == 0) {
    ROS_WARN_THROTTLE(1.0, "[UnifiedCtrl] sendTorqueAllocationMatrixInv: inv_rot not computed yet");
    return;
  }

  std::vector<int> assembled_ids = navigator_->getAssemblyIds();
  int rows_per_module = motor_num_per_module_ * rotor_coef_;

  for (size_t m = 0; m < assembled_ids.size(); m++) {
    int module_id = assembled_ids[m];
    if (!module_torque_alloc_inv_pubs_.count(module_id)) continue;

    spinal::TorqueAllocationMatrixInv msg;
    msg.rows.resize(rows_per_module);

    int row_start = m * rows_per_module;
    for (int i = 0; i < rows_per_module; i++) {
      if (integrated_map_inv_rot_.cwiseAbs().maxCoeff() > INT16_MAX * 0.001f) {
        ROS_ERROR_THROTTLE(1.0, "[UnifiedCtrl] Torque Allocation Matrix overflow for module %d", module_id);
      }
      msg.rows[i].x = static_cast<int16_t>(integrated_map_inv_rot_(row_start + i, 0) * 1000);
      msg.rows[i].y = static_cast<int16_t>(integrated_map_inv_rot_(row_start + i, 1) * 1000);
      msg.rows[i].z = static_cast<int16_t>(integrated_map_inv_rot_(row_start + i, 2) * 1000);
    }

    module_torque_alloc_inv_pubs_[module_id].publish(msg);
    ROS_INFO_THROTTLE(2.0, "[UnifiedCtrl] Sent torque_alloc_inv to module %d: %d rows, "
                      "inv_rot[0]=(%.4f,%.4f,%.4f)",
                      module_id, rows_per_module,
                      integrated_map_inv_rot_(row_start, 0),
                      integrated_map_inv_rot_(row_start, 1),
                      integrated_map_inv_rot_(row_start, 2));
  }
}

void BeetleUnifiedController::sendCascadeGains(
    double roll_p, double roll_d,
    double pitch_p, double pitch_d,
    double yaw_d)
{
  // Send torque-level P/D gains to each module's spinal.
  // Using motors.resize(1) → spinal stores as torque_p/d_gain and runs thrustGainMapping()
  // to compute per-motor gains using the torque_allocation_matrix_inv.
  // I=0: PC handles I-term in the outer loop.

  std::vector<int> assembled_ids = navigator_->getAssemblyIds();

  for (size_t m = 0; m < assembled_ids.size(); m++) {
    int module_id = assembled_ids[m];
    if (!module_rpy_gain_pubs_.count(module_id)) continue;

    spinal::RollPitchYawTerms rpy_gain_msg;
    rpy_gain_msg.motors.resize(1);
    rpy_gain_msg.motors[0].roll_p  = static_cast<int16_t>(roll_p * 1000);
    rpy_gain_msg.motors[0].roll_i  = 0;  // I-term handled by PC
    rpy_gain_msg.motors[0].roll_d  = static_cast<int16_t>(roll_d * 1000);
    rpy_gain_msg.motors[0].pitch_p = static_cast<int16_t>(pitch_p * 1000);
    rpy_gain_msg.motors[0].pitch_i = 0;  // I-term handled by PC
    rpy_gain_msg.motors[0].pitch_d = static_cast<int16_t>(pitch_d * 1000);
    rpy_gain_msg.motors[0].yaw_d   = static_cast<int16_t>(yaw_d * 1000);

    module_rpy_gain_pubs_[module_id].publish(rpy_gain_msg);
  }

  ROS_INFO_THROTTLE(2.0, "[UnifiedCtrl] Sent cascade gains to %zu modules: "
                    "roll(P=%.2f,D=%.2f) pitch(P=%.2f,D=%.2f) yaw(D=%.2f)",
                    assembled_ids.size(), roll_p, roll_d, pitch_p, pitch_d, yaw_d);
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
