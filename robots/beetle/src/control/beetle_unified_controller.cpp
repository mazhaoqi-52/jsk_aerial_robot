// Unified controller for assembled beetle formation (Plan B).
// See beetle_unified_controller.h for design overview.

#include <beetle/control/beetle_unified_controller.h>
#include <tf_conversions/tf_kdl.h>
#include <tf_conversions/tf_eigen.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/SVD>

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
    torque_alloc_pub_stamp_(0)
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
    // These publish directly to each module's spinal topics
    module_rpy_gain_pubs_[i] = nh_.advertise<spinal::RollPitchYawTerms>(ns + "/rpy/gain", 1);
    module_torque_alloc_pubs_[i] = nh_.advertise<spinal::TorqueAllocationMatrixInv>(ns + "/torque_allocation_matrix_inv", 1);
    module_p_matrix_pubs_[i] = nh_.advertise<spinal::PMatrixPseudoInverseWithInertia>(ns + "/p_matrix_pseudo_inverse_inertia", 1);
  }

  formation_wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("unified_control/formation_wrench", 1);
  formation_vectoring_f_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("unified_control/vectoring_force", 1);

  ROS_INFO("[UnifiedCtrl-B] Initialized: motor_per_module=%d, gimbal_dof=%d, rotor_coef=%d, gimbal_calc_in_fc=%d",
           motor_num_per_module_, gimbal_dof_, rotor_coef_, gimbal_calc_in_fc_);
}

void BeetleUnifiedController::rosParamInit()
{
  ros::NodeHandle control_nh(nh_, "controller");
  control_nh.param<int>("gimbal_dof", gimbal_dof_, 1);
  control_nh.param<bool>("gimbal_calc_in_fc", gimbal_calc_in_fc_, false);
  control_nh.param<double>("torque_allocation_matrix_inv_pub_interval", torque_alloc_pub_interval_, 0.05);
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
        ROS_WARN_THROTTLE(1.0, "[UnifiedCtrl-B] TF lookup for formation geometry failed: %s", ex.what());
        return false;
      }
    }
  }
  formation_cog_offset_ = cog_offset_sum / N;
  formation_inertia_ = computeFormationInertia(assembled_ids, formation_cog_offset_);
  return true;
}

bool BeetleUnifiedController::computePositionAllocation(
    const Eigen::Vector3d& target_acc_cog,
    double candidate_yaw_term,
    const Eigen::Vector3d& target_rpy)
{
  std::vector<int> assembled_ids = navigator_->getAssemblyIds();
  if (assembled_ids.empty()) return false;

  int N = assembled_ids.size();

  // Update formation geometry
  if (!updateFormationGeometry()) return false;

  // Build formation-wide allocation matrix
  integrated_map_ = buildFormationAllocationMatrix(assembled_ids, formation_mass_,
                                                    formation_inertia_, formation_cog_offset_);

  // Pseudoinverse of full 6-DOF allocation matrix
  integrated_map_inv_ = aerial_robot_model::pseudoinverse(integrated_map_);

  // Split into position (cols 0-2) and attitude (cols 3-5) parts
  q_mat_inv_trans_ = integrated_map_inv_.leftCols(3);
  q_mat_inv_rot_ = integrated_map_inv_.rightCols(3);

  // Split attitude part per module for torque_allocation_matrix_inv
  splitTorqueAllocationPerModule(assembled_ids);

  // Position allocation: vectoring_f_trans = q_mat_inv_trans * target_acc_cog
  Eigen::VectorXd vectoring_f_trans = q_mat_inv_trans_ * target_acc_cog;

  // Extract per-module base_thrust from position-only allocation
  extractBaseThrust(vectoring_f_trans, assembled_ids, candidate_yaw_term, target_rpy);

  // Debug
  ROS_INFO_THROTTLE(1.0, "[UnifiedCtrl-B DIAG] formation_mass=%.3f, N=%d, cog_offset=(%.4f,%.4f,%.4f)",
                    formation_mass_, N,
                    formation_cog_offset_.x(), formation_cog_offset_.y(), formation_cog_offset_.z());
  ROS_INFO_THROTTLE(1.0, "[UnifiedCtrl-B DIAG] target_acc_cog=(%.4f,%.4f,%.4f) target_rpy=(%.4f,%.4f,%.4f) yaw_term=%.4f",
                    target_acc_cog.x(), target_acc_cog.y(), target_acc_cog.z(),
                    target_rpy.x(), target_rpy.y(), target_rpy.z(), candidate_yaw_term);

  return true;
}

void BeetleUnifiedController::splitTorqueAllocationPerModule(const std::vector<int>& assembled_ids)
{
  int N = assembled_ids.size();
  int rotors_per_module = motor_num_per_module_ * rotor_coef_;
  per_module_torque_alloc_inv_.clear();

  for (int m = 0; m < N; m++) {
    int module_id = assembled_ids[m];
    // Extract this module's rows from q_mat_inv_rot_
    // q_mat_inv_rot_ is (N * motor * rotor_coef) x 3
    int start_row = m * rotors_per_module;
    per_module_torque_alloc_inv_[module_id] =
        q_mat_inv_rot_.middleRows(start_row, rotors_per_module);
  }
}

void BeetleUnifiedController::sendFormationTorqueAllocationMatrixInv()
{
  double now = ros::Time::now().toSec();
  if (now - torque_alloc_pub_stamp_ < torque_alloc_pub_interval_) return;
  torque_alloc_pub_stamp_ = now;

  for (const auto& kv : per_module_torque_alloc_inv_) {
    int module_id = kv.first;
    const Eigen::MatrixXd& alloc = kv.second;

    if (!module_torque_alloc_pubs_.count(module_id)) continue;

    // For gimbal_calc_in_fc=false: spinal has gimbal_dof=0, motor_number=4
    // So torque_allocation_matrix_inv should have motor_number rows (4 rows x 3 cols)
    // But our alloc is (motor_num * rotor_coef) x 3 = 8 x 3
    // We need to sum/combine the rotor_coef components into scalar per-motor values.
    //
    // When gimbal_calc_in_fc=false and spinal gimbal_dof=0:
    //   spinal uses rpyGainCallback with motors.size()==1 path:
    //     torque_p_gain[axis] = motors[0].roll_p etc
    //   then thrustGainMapping:
    //     thrust_p_gain[i][axis] = torque_alloc_inv[i][axis] * torque_p_gain[axis]
    //   spinal motor_number = physical motors (4), expects 4-row torque_alloc_inv
    //
    // For gimbal_calc_in_fc=true (spinal gimbal_dof=1):
    //   spinal motor_number = 8, expects 8-row torque_alloc_inv
    //
    // We handle both cases:
    int spinal_motor_num;
    if (gimbal_calc_in_fc_) {
      spinal_motor_num = motor_num_per_module_ * rotor_coef_;
    } else {
      spinal_motor_num = motor_num_per_module_;
    }

    spinal::TorqueAllocationMatrixInv msg;
    msg.rows.resize(spinal_motor_num);

    if (gimbal_calc_in_fc_) {
      // Direct: alloc rows map 1:1 to spinal motors
      for (int i = 0; i < spinal_motor_num; i++) {
        msg.rows[i].x = alloc(i, 0) * 1000;
        msg.rows[i].y = alloc(i, 1) * 1000;
        msg.rows[i].z = alloc(i, 2) * 1000;
      }
    } else {
      // gimbal_calc_in_fc=false: spinal has scalar motors (gimbal_dof=0, rotor_coef=1)
      // We need to project the rotor_coef-dimensional allocation onto the scalar thrust direction.
      // For each physical motor i, its vectoring force components are alloc(i*rotor_coef .. i*rotor_coef+rotor_coef-1, :)
      // The effective scalar torque allocation for motor i is the projection of these
      // vectoring components onto the current thrust direction.
      //
      // However, since spinal doesn't know about gimbal angles when gimbal_calc_in_fc=false,
      // we use the simple approach: sum the squared norms across rotor_coef dimensions.
      // Actually, the correct approach for scalar motors:
      //   torque_alloc_inv[i][axis] should represent how much scalar thrust change
      //   at motor i produces a unit angular acceleration on axis.
      // This is the norm of the vectoring allocation vector for that motor/axis pair.
      //
      // Simpler: since beetle's gimbal has 1-DOF (tilt in a plane), and in the current
      // configuration the vertical component dominates, we just use the magnitude with sign.
      for (int i = 0; i < motor_num_per_module_; i++) {
        // Take the rotor_coef rows for this motor
        Eigen::MatrixXd motor_alloc = alloc.middleRows(i * rotor_coef_, rotor_coef_);
        // For each axis (roll, pitch, yaw), compute signed magnitude
        for (int axis = 0; axis < 3; axis++) {
          Eigen::VectorXd col = motor_alloc.col(axis);
          // Use the vertical (last) component as dominant direction for sign
          double sign = (col(rotor_coef_ - 1) >= 0) ? 1.0 : -1.0;
          double val = sign * col.norm();
          switch (axis) {
            case 0: msg.rows[i].x = val * 1000; break;
            case 1: msg.rows[i].y = val * 1000; break;
            case 2: msg.rows[i].z = val * 1000; break;
          }
        }
      }
    }

    if (msg.rows.size() > 0 && fabs(msg.rows[0].x) + fabs(msg.rows[0].y) + fabs(msg.rows[0].z) > 0.001) {
      module_torque_alloc_pubs_[module_id].publish(msg);
      ROS_INFO_THROTTLE(2.0, "[UnifiedCtrl-B] Sent torque_alloc_inv to m%d: r0=(%.3f,%.3f,%.3f) r1=(%.3f,%.3f,%.3f) ...",
                        module_id,
                        msg.rows[0].x/1000.0, msg.rows[0].y/1000.0, msg.rows[0].z/1000.0,
                        msg.rows.size()>1 ? msg.rows[1].x/1000.0 : 0,
                        msg.rows.size()>1 ? msg.rows[1].y/1000.0 : 0,
                        msg.rows.size()>1 ? msg.rows[1].z/1000.0 : 0);
    }
  }
}

void BeetleUnifiedController::sendFormationAttitudeGains(
    double roll_p, double roll_i, double roll_d,
    double pitch_p, double pitch_i, double pitch_d,
    double yaw_d)
{
  spinal::RollPitchYawTerms rpy_gain_msg;
  rpy_gain_msg.motors.resize(1);
  rpy_gain_msg.motors[0].roll_p = roll_p * 1000;
  rpy_gain_msg.motors[0].roll_i = roll_i * 1000;
  rpy_gain_msg.motors[0].roll_d = roll_d * 1000;
  rpy_gain_msg.motors[0].pitch_p = pitch_p * 1000;
  rpy_gain_msg.motors[0].pitch_i = pitch_i * 1000;
  rpy_gain_msg.motors[0].pitch_d = pitch_d * 1000;
  rpy_gain_msg.motors[0].yaw_d = yaw_d * 1000;

  for (const auto& kv : module_rpy_gain_pubs_) {
    // Only send to assembled modules
    if (per_module_torque_alloc_inv_.count(kv.first)) {
      kv.second.publish(rpy_gain_msg);
    }
  }

  ROS_INFO_THROTTLE(2.0, "[UnifiedCtrl-B] Sent RPY gains: rp=(%.2f,%.2f,%.2f) pp=(%.2f,%.2f,%.2f) yd=%.2f",
                    roll_p, roll_i, roll_d, pitch_p, pitch_i, pitch_d, yaw_d);
}

void BeetleUnifiedController::sendFormationPMatrixInertia()
{
  // For gyro moment compensation inside spinal.
  // spinal uses: gyro_compensate = p_matrix_inv * (omega x (I * omega))
  // We need to send formation-wide p_matrix_pseudo_inverse and formation inertia.
  //
  // For now, since spinal's gyro compensation is per-motor and uses p_matrix_pseudo_inverse_
  // (which maps wrench to per-motor force), we send zeros to disable it.
  // The gyro compensation will be handled in the PC-side PID (if needed).
  //
  // TODO: properly compute per-module p_matrix for formation
  for (const auto& kv : per_module_torque_alloc_inv_) {
    int module_id = kv.first;
    if (!module_p_matrix_pubs_.count(module_id)) continue;

    int spinal_motor_num = gimbal_calc_in_fc_ ? motor_num_per_module_ * rotor_coef_ : motor_num_per_module_;

    spinal::PMatrixPseudoInverseWithInertia msg;
    msg.pseudo_inverse.resize(spinal_motor_num);
    for (int i = 0; i < spinal_motor_num; i++) {
      msg.pseudo_inverse[i].r = 0;
      msg.pseudo_inverse[i].p = 0;
      msg.pseudo_inverse[i].y = 0;
    }
    // Send zero inertia to disable gyro compensation in spinal
    // inertia is int16[6]: xx, yy, zz, xy, yz, xz
    for (int i = 0; i < 6; i++) msg.inertia[i] = 0;

    module_p_matrix_pubs_[module_id].publish(msg);
  }
}

void BeetleUnifiedController::extractBaseThrust(
    const Eigen::VectorXd& vectoring_f_trans,
    const std::vector<int>& assembled_ids,
    double candidate_yaw_term,
    const Eigen::Vector3d& target_rpy)
{
  int N = assembled_ids.size();
  module_commands_.clear();

  int col = 0;
  for (int m = 0; m < N; m++) {
    int module_id = assembled_ids[m];
    ModuleCommand cmd;
    cmd.target_roll = target_rpy.x();
    cmd.target_pitch = target_rpy.y();
    cmd.candidate_yaw_term = candidate_yaw_term;

    if (gimbal_calc_in_fc_) {
      // spinal expects vectoring force components (rotor_coef per motor)
      cmd.base_thrust.resize(motor_num_per_module_ * rotor_coef_);
      for (int r = 0; r < motor_num_per_module_; r++) {
        for (int c = 0; c < rotor_coef_; c++) {
          cmd.base_thrust[r * rotor_coef_ + c] = static_cast<float>(vectoring_f_trans(col + c));
        }
        col += rotor_coef_;
      }
    } else {
      // gimbal_calc_in_fc=false: spinal expects scalar thrusts (1 per motor)
      // We decompose vectoring force into scalar thrust + gimbal angle (PC side)
      cmd.base_thrust.resize(motor_num_per_module_);
      cmd.gimbal_angles.resize(motor_num_per_module_ * gimbal_dof_);

      for (int r = 0; r < motor_num_per_module_; r++) {
        Eigen::VectorXd f_i = vectoring_f_trans.segment(col, rotor_coef_);

        if (gimbal_dof_ == 1) {
          // Clamp negative vertical thrust
          if (f_i[1] < 0) {
            f_i[0] = 0;
            f_i[1] = std::abs(f_i[1]);
          }
          cmd.base_thrust[r] = static_cast<float>(f_i.norm());
          cmd.gimbal_angles[r] = atan2(-f_i[0], f_i[1]);
        } else if (gimbal_dof_ == 2) {
          cmd.base_thrust[r] = static_cast<float>(f_i.norm());
          double gimbal_roll = atan2(-f_i[1], f_i[2]);
          double gimbal_pitch = atan2(f_i[0], -f_i[1] * sin(gimbal_roll) + f_i[2] * cos(gimbal_roll));
          cmd.gimbal_angles[2*r] = gimbal_roll;
          cmd.gimbal_angles[2*r+1] = gimbal_pitch;
        }
        col += rotor_coef_;
      }
    }

    module_commands_[module_id] = cmd;
  }
}

void BeetleUnifiedController::publishCommands()
{
  for (const auto& kv : module_commands_) {
    int module_id = kv.first;
    const ModuleCommand& cmd = kv.second;

    if (module_thrust_pubs_.count(module_id)) {
      spinal::FourAxisCommand thrust_msg;
      thrust_msg.base_thrust = cmd.base_thrust;
      // KEY DIFFERENCE from Plan A: send target RPY to spinal for its attitude PID
      thrust_msg.angles[0] = cmd.target_roll;
      thrust_msg.angles[1] = cmd.target_pitch;
      thrust_msg.angles[2] = cmd.candidate_yaw_term;
      module_thrust_pubs_[module_id].publish(thrust_msg);
    }

    // If gimbal_calc_in_fc=false, send gimbal angles separately
    if (!gimbal_calc_in_fc_ && module_gimbal_pubs_.count(module_id)) {
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
        ROS_WARN_THROTTLE(1.0, "[UnifiedCtrl-B] TF lookup failed: %s", ex.what());
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
        ROS_WARN_THROTTLE(1.0, "[UnifiedCtrl-B] TF lookup for inertia failed: %s", ex.what());
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
