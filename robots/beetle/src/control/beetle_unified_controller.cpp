// Unified 4N-rotor controller for assembled beetle formation.
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
    candidate_yaw_term_(0),
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

  // Read actual values from robot model
  motor_num_per_module_ = robot_model_->getRotorNum();
  rosParamInit();
  rotor_coef_ = gimbal_dof_ + 1;

  // Create per-module publishers for all possible modules
  // In unified mode we send full spinal commands to each module:
  //   - FourAxisCommand (base_thrust 2D vectoring + target RPY angles)
  //   - TorqueAllocationMatrixInv (per-module allocation for spinal's attitude PID)
  //   - RollPitchYawTerms (RPY gains for spinal)
  //   - DesireCoord (CoG frame orientation for spinal)
  //   - UInt8 gimbal_dof (tell spinal to operate in gimbal_dof=1 mode)
  int max_modules = navigator_->getMaxModuleNum();
  std::string my_name = navigator_->getMyName();
  for (int i = 1; i <= max_modules; i++) {
    std::string ns = std::string("/") + my_name + std::to_string(i);
    module_thrust_pubs_[i] = nh_.advertise<spinal::FourAxisCommand>(ns + "/unified_thrust_cmd", 1);
    module_torque_alloc_pubs_[i] = nh_.advertise<spinal::TorqueAllocationMatrixInv>(ns + "/unified_torque_alloc_inv", 1);
    module_rpy_gain_pubs_[i] = nh_.advertise<spinal::RollPitchYawTerms>(ns + "/unified_rpy_gain", 1);
    module_desire_coord_pubs_[i] = nh_.advertise<spinal::DesireCoord>(ns + "/unified_desire_coord", 1);
    module_gimbal_dof_pubs_[i] = nh_.advertise<std_msgs::UInt8>(ns + "/unified_gimbal_dof", 1);
  }

  formation_wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("unified_control/formation_wrench", 1);
  formation_vectoring_f_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("unified_control/vectoring_force", 1);

  ROS_INFO("[UnifiedCtrl] Initialized: motor_per_module=%d, gimbal_dof=%d, rotor_coef=%d",
           motor_num_per_module_, gimbal_dof_, rotor_coef_);
}

void BeetleUnifiedController::rosParamInit()
{
  ros::NodeHandle control_nh(nh_, "controller");

  // gimbal_dof: read from existing gimbalrotor param
  control_nh.param<int>("gimbal_dof", gimbal_dof_, 1);
}

bool BeetleUnifiedController::updateFormationGeometry()
{
  std::vector<int> assembled_ids = navigator_->getAssemblyIds();
  if (assembled_ids.empty()) return false;

  int N = assembled_ids.size();
  double single_mass = robot_model_->getMass();
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
  if (target_wrench_acc_cog.size() < 6) {
    ROS_WARN_THROTTLE(1.0, "[UnifiedCtrl] target_wrench_acc_cog size < 6, skip");
    return false;
  }

  std::vector<int> assembled_ids = navigator_->getAssemblyIds();
  if (assembled_ids.empty()) return false;

  int N = assembled_ids.size();
  double single_mass = robot_model_->getMass();
  double formation_mass = single_mass * N;

  // ---- Compute formation CoG offset from LEADER CoG ----
  // All modules have same mass, so formation CoG = average of module CoG positions
  int leader_id = navigator_->getLeaderID();
  std::string leader_cog_frame = navigator_->getMyName() + std::to_string(leader_id) + "/cog";
  Eigen::Vector3d formation_cog_offset = Eigen::Vector3d::Zero();
  for (int i = 0; i < N; i++) {
    int module_id = assembled_ids[i];
    if (module_id != leader_id) {
      try {
        std::string module_cog_frame = navigator_->getMyName() + std::to_string(module_id) + "/cog";
        geometry_msgs::TransformStamped tf_stamped =
            navigator_->getTfBuffer().lookupTransform(leader_cog_frame, module_cog_frame, ros::Time(0));
        formation_cog_offset.x() += tf_stamped.transform.translation.x;
        formation_cog_offset.y() += tf_stamped.transform.translation.y;
        formation_cog_offset.z() += tf_stamped.transform.translation.z;
      } catch (tf2::TransformException& ex) {
        ROS_WARN_THROTTLE(1.0, "[UnifiedCtrl] TF lookup for formation CoG failed: %s", ex.what());
        return false;
      }
    }
    // LEADER offset = (0,0,0), already in sum
  }
  formation_cog_offset_ = formation_cog_offset / N;

  formation_inertia_ = computeFormationInertia(assembled_ids, formation_cog_offset_);

  // Build the formation-wide allocation matrix (relative to formation CoG)
  integrated_map_ = buildFormationAllocationMatrix(assembled_ids, formation_mass, formation_inertia_, formation_cog_offset_);

  // ---- Wrench acc: PID output is already in acceleration space ----
  // Allocation matrix is built relative to formation CoG, and PID angular
  // acceleration output is frame-independent (rigid body). No wrench
  // transform (d × F) is needed — same approach as assemble_quadrotors.
  Eigen::VectorXd total_wrench_acc = target_wrench_acc_cog;
  if (desired_ext_wrench.size() >= 6 && desired_ext_wrench.norm() > 1e-6) {
    double mass_inv = 1.0 / formation_mass;
    Eigen::Matrix3d inertia_inv = formation_inertia_.inverse();
    total_wrench_acc.head(3) += mass_inv * desired_ext_wrench.head(3);
    total_wrench_acc.tail(3) += inertia_inv * desired_ext_wrench.tail(3);
  }

  // ===== DIAGNOSTIC: formation inertia & allocation matrix =====
  {
    ROS_INFO_THROTTLE(1.0, "[UnifiedCtrl DIAG] formation_cog_offset=(%.4f,%.4f,%.4f)",
                      formation_cog_offset_.x(), formation_cog_offset_.y(), formation_cog_offset_.z());
    ROS_INFO_THROTTLE(1.0, "[UnifiedCtrl DIAG] formation_mass=%.3f, N=%d, single_mass=%.3f",
                      formation_mass, N, single_mass);
    ROS_INFO_THROTTLE(1.0, "[UnifiedCtrl DIAG] formation_inertia diag=(%.4f, %.4f, %.4f)",
                      formation_inertia_(0,0), formation_inertia_(1,1), formation_inertia_(2,2));
    ROS_INFO_THROTTLE(1.0, "[UnifiedCtrl DIAG] formation_inertia off-diag=(%.4f, %.4f, %.4f)",
                      formation_inertia_(0,1), formation_inertia_(0,2), formation_inertia_(1,2));

    // Allocation matrix condition number via SVD
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(integrated_map_);
    double cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size()-1);
    ROS_INFO_THROTTLE(1.0, "[UnifiedCtrl DIAG] integrated_map size=(%ld x %ld), cond_num=%.4f, sv_max=%.6f, sv_min=%.6f",
                      integrated_map_.rows(), integrated_map_.cols(), cond,
                      svd.singularValues()(0), svd.singularValues()(svd.singularValues().size()-1));
  }

  // Solve allocation: pseudoinverse
  integrated_map_inv_ = aerial_robot_model::pseudoinverse(integrated_map_);

  // Split allocation into translational (XYZ) and rotational (RPY) parts.
  // This follows the same approach as GimbalrotorController (gimbal_calc_in_fc=true):
  //   integrated_map_inv_trans_ = leftCols(3)  → maps XYZ acc to 2D vectoring force (base_thrust)
  //   integrated_map_inv_rot_  = rightCols(3) → maps RPY acc to 2D vectoring force (TorqueAllocationMatrixInv)
  // The RPY part is sent to spinal so it can do high-freq attitude PID at 1kHz.
  integrated_map_inv_trans_ = integrated_map_inv_.leftCols(3);
  integrated_map_inv_rot_ = integrated_map_inv_.rightCols(3);

  // Position-only vectoring force = only from XYZ acceleration commands
  target_vectoring_f_trans_ = integrated_map_inv_trans_ * total_wrench_acc.head(3);

  // Full vectoring force (for debug / monitoring)
  target_vectoring_f_ = integrated_map_inv_ * total_wrench_acc;

  // Compute candidate_yaw_term for spinal's yaw reconstruction
  // (same approach as GimbalrotorController: find max yaw scale factor)
  double max_yaw_scale = 0;
  int total_entries = integrated_map_inv_.rows();
  int yaw_col = 5;  // YAW is the last column (index 5 in 6D wrench)
  for (int i = 0; i < total_entries; i++) {
    if (integrated_map_inv_(i, yaw_col) > max_yaw_scale)
      max_yaw_scale = integrated_map_inv_(i, yaw_col);
  }
  // The yaw PID result is the 6th element (index 5) of target_wrench_acc
  candidate_yaw_term_ = total_wrench_acc(5) * max_yaw_scale;

  // ===== DIAGNOSTIC: verify allocation roundtrip =====
  {
    Eigen::VectorXd reconstructed = integrated_map_ * target_vectoring_f_;
    Eigen::VectorXd residual = reconstructed - total_wrench_acc;
    ROS_INFO_THROTTLE(1.0, "[UnifiedCtrl DIAG] allocation residual norm=%.6e", residual.norm());
    ROS_INFO_THROTTLE(1.0, "[UnifiedCtrl DIAG] reconstructed wrench_acc=(%.4f,%.4f,%.4f,%.4f,%.4f,%.4f)",
                      reconstructed(0), reconstructed(1), reconstructed(2),
                      reconstructed(3), reconstructed(4), reconstructed(5));
  }

  // Extract per-module base_thrust (2D vectoring, position-only) and TorqueAllocationMatrixInv
  extractModuleCommands(target_vectoring_f_trans_, integrated_map_inv_rot_, assembled_ids);

  // Publish debug info
  if (formation_wrench_pub_.getNumSubscribers() > 0) {
    geometry_msgs::WrenchStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.wrench.force.x = total_wrench_acc(0);
    msg.wrench.force.y = total_wrench_acc(1);
    msg.wrench.force.z = total_wrench_acc(2);
    msg.wrench.torque.x = total_wrench_acc(3);
    msg.wrench.torque.y = total_wrench_acc(4);
    msg.wrench.torque.z = total_wrench_acc(5);
    formation_wrench_pub_.publish(msg);
  }

  if (formation_vectoring_f_pub_.getNumSubscribers() > 0) {
    std_msgs::Float32MultiArray vmsg;
    for (int i = 0; i < target_vectoring_f_.size(); i++) {
      vmsg.data.push_back(target_vectoring_f_(i));
    }
    formation_vectoring_f_pub_.publish(vmsg);
  }

  return true;
}

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

  // ---- Step 1: Build full_q_mat (6 x 3*total_rotors) ----
  // Each column-block (3 cols) represents one rotor's [force; torque] contribution.
  Eigen::MatrixXd full_q_mat = Eigen::MatrixXd::Zero(6, 3 * total_rotors);

  // Get per-module rotor geometry from robot model
  // These are rotor positions relative to the single module's own CoG
  std::vector<Eigen::Vector3d> single_rotors_from_cog =
      robot_model_->getRotorsOriginFromCog<Eigen::Vector3d>();
  const auto& rotor_direction = robot_model_->getRotorDirection();
  const double m_f_rate = robot_model_->getMFRate();

  Eigen::MatrixXd wrench_map = Eigen::MatrixXd::Zero(6, 3);
  wrench_map.block(0, 0, 3, 3) = Eigen::MatrixXd::Identity(3, 3);

  // ===== DIAGNOSTIC: rotor model parameters =====
  {
    std::string dir_str;
    for (int r = 0; r < motor_num_per_module_; r++) {
      dir_str += std::to_string(rotor_direction.at(r + 1)) + " ";
    }
    std::stringstream rpos_ss;
    for (int r = 0; r < motor_num_per_module_; r++) {
      char rb[128];
      snprintf(rb, sizeof(rb), " r%d=(%.4f,%.4f,%.4f)", r+1,
               single_rotors_from_cog.at(r).x(), single_rotors_from_cog.at(r).y(), single_rotors_from_cog.at(r).z());
      rpos_ss << rb;
    }
    ROS_INFO_THROTTLE(1.0, "[UnifiedCtrl DIAG] m_f_rate=%.6f, dirs=[%s], n_rot=%d, single_rotors:%s",
                      m_f_rate, dir_str.c_str(), motor_num_per_module_, rpos_ss.str().c_str());
  }

  // LEADER's CoG frame is treated as formation CoG frame
  int leader_id = navigator_->getLeaderID();
  std::string leader_cog_frame = navigator_->getMyName() + std::to_string(leader_id) + "/cog";

  std::stringstream diag_tf_ss, diag_rotor_ss;  // collect all per-module/per-rotor info
  char buf[256];

  int col = 0;
  for (int m = 0; m < N; m++) {
    int module_id = assembled_ids[m];

    // Get module CoG offset from LEADER CoG via TF (real physical position)
    Eigen::Vector3d module_offset = Eigen::Vector3d::Zero();
    if (module_id != leader_id) {
      try {
        std::string module_cog_frame = navigator_->getMyName() + std::to_string(module_id) + "/cog";
        geometry_msgs::TransformStamped tf_stamped =
            navigator_->getTfBuffer().lookupTransform(leader_cog_frame, module_cog_frame, ros::Time(0));
        module_offset << tf_stamped.transform.translation.x,
                         tf_stamped.transform.translation.y,
                         tf_stamped.transform.translation.z;

        auto& q = tf_stamped.transform.rotation;
        snprintf(buf, sizeof(buf), "  m%d: t=(%.4f,%.4f,%.4f) q=(%.4f,%.4f,%.4f,%.4f)",
                 module_id, module_offset.x(), module_offset.y(), module_offset.z(),
                 q.x, q.y, q.z, q.w);
        diag_tf_ss << buf;
      } catch (tf2::TransformException& ex) {
        ROS_WARN_THROTTLE(1.0, "[UnifiedCtrl] TF lookup %s->module%d failed: %s",
                          leader_cog_frame.c_str(), module_id, ex.what());
        return Eigen::MatrixXd::Zero(6, rotor_coef_ * total_rotors);
      }
    } else {
      snprintf(buf, sizeof(buf), "  m%d(LEADER): t=(0,0,0)", module_id);
      diag_tf_ss << buf;
    }

    for (int r = 0; r < motor_num_per_module_; r++) {
      // Rotor position relative to formation CoG (not LEADER CoG)
      Eigen::Vector3d rotor_pos = module_offset - formation_cog_offset + single_rotors_from_cog.at(r);

      // rotor_direction is 1-indexed in the robot model (index 0 is unused)
      int dir = rotor_direction.at(r + 1);

      wrench_map.block(3, 0, 3, 3) =
          aerial_robot_model::skew(rotor_pos) + dir * m_f_rate * Eigen::Matrix3d::Identity();

      full_q_mat.middleCols(col, 3) = wrench_map;

      snprintf(buf, sizeof(buf), "  m%d_r%d: pos=(%.3f,%.3f,%.3f) d=%d",
               module_id, r+1, rotor_pos.x(), rotor_pos.y(), rotor_pos.z(), dir);
      diag_rotor_ss << buf;

      col += 3;
    }
  }

  ROS_INFO_THROTTLE(1.0, "[UnifiedCtrl DIAG] TF offsets (leader=%d):%s", leader_id, diag_tf_ss.str().c_str());
  ROS_INFO_THROTTLE(1.0, "[UnifiedCtrl DIAG] rotor positions:%s", diag_rotor_ss.str().c_str());

  // Scale: force rows by 1/M, torque rows by I^{-1}
  full_q_mat.topRows(3) = mass_inv * full_q_mat.topRows(3);
  full_q_mat.bottomRows(3) = inertia_inv * full_q_mat.bottomRows(3);

  // ---- Step 2: Build gimbal mask rotation matrix ----
  // Each rotor has a thrust coordinate rotation from the robot model.
  // For assembled formation, we assume all modules have the same rotor configuration
  // (same URDF), so thrust_coords_rot is reused per module.
  std::vector<KDL::Rotation> thrust_coords_rot =
      robot_model_->getThrustCoordRot<KDL::Rotation>();

  std::vector<Eigen::MatrixXd> masked_rot_single;
  std::stringstream diag_mask_ss;
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

    {
      double roll, pitch, yaw;
      tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
      Eigen::MatrixXd& mr = masked_rot_single.back();
      char b[128];
      if (gimbal_dof_ == 1) {
        snprintf(b, sizeof(b), "  r%d:yaw=%.3f col0=(%.3f,%.3f,%.3f) col1=(%.3f,%.3f,%.3f)",
                 r+1, yaw, mr(0,0), mr(1,0), mr(2,0), mr(0,1), mr(1,1), mr(2,1));
      } else {
        snprintf(b, sizeof(b), "  r%d:rpy=(%.3f,%.3f,%.3f)", r+1, roll, pitch, yaw);
      }
      diag_mask_ss << b;
    }
  }
  ROS_INFO_THROTTLE(1.0, "[UnifiedCtrl DIAG] masked_rot:%s", diag_mask_ss.str().c_str());

  // ---- Step 3: Build block-diagonal integrated_rot ----
  // Size: (3 * total_rotors) x (rotor_coef * total_rotors)
  Eigen::MatrixXd integrated_rot =
      Eigen::MatrixXd::Zero(3 * total_rotors, rotor_coef_ * total_rotors);

  for (int m = 0; m < N; m++) {
    for (int r = 0; r < motor_num_per_module_; r++) {
      int rotor_idx = m * motor_num_per_module_ + r;
      integrated_rot.block(3 * rotor_idx, rotor_coef_ * rotor_idx,
                           3, rotor_coef_) = masked_rot_single[r];
    }
  }

  // ---- Step 4: Final integrated allocation map ----
  // integrated_map = full_q_mat * integrated_rot
  // Size: 6 x (rotor_coef * total_rotors)
  // For 3-module beetle with gimbal_dof=1: 6 x (2 * 12) = 6 x 24
  return full_q_mat * integrated_rot;
}

Eigen::Matrix3d BeetleUnifiedController::computeFormationInertia(
    const std::vector<int>& assembled_ids,
    const Eigen::Vector3d& formation_cog_offset)
{
  int N = assembled_ids.size();
  double single_mass = robot_model_->getMass();
  Eigen::Matrix3d single_inertia = robot_model_->getInertia<Eigen::Matrix3d>();

  // ===== DIAGNOSTIC: single module inertia =====
  ROS_INFO_THROTTLE(1.0, "[UnifiedCtrl DIAG] single_inertia diag=(%.6f, %.6f, %.6f) mass=%.4f",
                    single_inertia(0,0), single_inertia(1,1), single_inertia(2,2), single_mass);

  int leader_id = navigator_->getLeaderID();
  std::string leader_cog_frame = navigator_->getMyName() + std::to_string(leader_id) + "/cog";

  Eigen::Matrix3d formation_inertia = Eigen::Matrix3d::Zero();

  for (int i = 0; i < N; i++) {
    int module_id = assembled_ids[i];

    // Get module CoG offset from LEADER CoG via TF
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
      }
    }
    // Shift to formation CoG reference
    d -= formation_cog_offset;

    // Parallel axis theorem: I_total += I_cm + m * (d^T*d * I_3 - d*d^T)
    formation_inertia += single_inertia
        + single_mass * (d.dot(d) * Eigen::Matrix3d::Identity() - d * d.transpose());
  }

  return formation_inertia;
}

void BeetleUnifiedController::extractModuleCommands(
    const Eigen::VectorXd& vectoring_f_trans,
    const Eigen::MatrixXd& map_inv_rot,
    const std::vector<int>& assembled_ids)
{
  // Extract per-module commands for spinal's gimbal_calc_in_fc=true mode:
  //
  // For each module m with motor_num_per_module_ rotors (rotor_coef_=2 entries each):
  //   base_thrust_2d[8] = position-only vectoring force for this module's 4 rotors
  //                       (from vectoring_f_trans, 2 entries per rotor: [lateral, vertical])
  //   torque_alloc_inv[8x3] = attitude allocation sub-matrix for this module's rotors
  //                           (from map_inv_rot, maps RPY acceleration to 2D vectoring force)
  //
  // Spinal will use: thrust[i] = base_thrust[i] + roll_pitch_term[i] + yaw_term[i]
  //   where roll_pitch_term is computed from torque_alloc_inv × RPY_PID_output at 1kHz.
  //   Final scalar thrust and gimbal angle are computed by spinal:
  //     thrust = norm(fx, fz), angle = atan2(-fx, fz)

  int N = assembled_ids.size();
  module_commands_.clear();

  std::stringstream diag_ss;
  char abuf[256];

  int entry_offset = 0;  // offset into vectoring_f_trans / map_inv_rot rows
  int entries_per_module = motor_num_per_module_ * rotor_coef_;  // 4*2=8

  for (int m = 0; m < N; m++) {
    int module_id = assembled_ids[m];
    ModuleCommand cmd;

    // base_thrust_2d: position-only vectoring force for this module's rotors
    cmd.base_thrust_2d.resize(entries_per_module);
    for (int i = 0; i < entries_per_module; i++) {
      cmd.base_thrust_2d[i] = static_cast<float>(vectoring_f_trans(entry_offset + i));
    }

    // torque_alloc_inv: sub-matrix of map_inv_rot for this module's rotors
    // Shape: entries_per_module x 3 (maps RPY acc → 2D vectoring corrections)
    cmd.torque_alloc_inv = map_inv_rot.block(entry_offset, 0, entries_per_module, 3);

    snprintf(abuf, sizeof(abuf), " m%d: bt2d=[", module_id);
    diag_ss << abuf;
    for (int i = 0; i < entries_per_module; i++) {
      snprintf(abuf, sizeof(abuf), "%.3f%s", cmd.base_thrust_2d[i],
               (i < entries_per_module-1) ? "," : "");
      diag_ss << abuf;
    }
    diag_ss << "]";

    module_commands_[module_id] = cmd;
    entry_offset += entries_per_module;
  }
  ROS_INFO_THROTTLE(1.0, "[UnifiedCtrl DIAG] extractModuleCommands:%s", diag_ss.str().c_str());
}

void BeetleUnifiedController::publishCommands(
    double target_roll, double target_pitch, double candidate_yaw_term,
    const std::vector<double>& rpy_p_gains,
    const std::vector<double>& rpy_i_gains,
    const std::vector<double>& rpy_d_gains)
{
  // Publish full spinal commands to each module in gimbal_calc_in_fc=true mode.
  //
  // Each module's spinal (SimulationAttitudeController → FlightControl → AttitudeController)
  // receives these on its standard topics (the beetle_controller forwards from unified_* → spinal):
  //
  // 1. FourAxisCommand (→ four_axes/command):
  //      base_thrust[8] = position-only 2D vectoring force (4 rotors × rotor_coef 2)
  //      angles[0] = target_roll
  //      angles[1] = target_pitch
  //      angles[2] = candidate_yaw_term
  //
  // 2. TorqueAllocationMatrixInv (→ torque_allocation_matrix_inv):
  //      rows[8] of Vector3Int16: per-motor RPY allocation, scaled ×1000
  //      Spinal uses: thrust_p_gain[i][axis] = rows[i].axis * 0.001 * torque_p_gain[axis]
  //
  // 3. RollPitchYawTerms (→ rpy/gain):
  //      motors[1] of RollPitchYawTerm: RPY PID gains, scaled ×1000
  //      With i_term_rp_calc_in_pc=true: roll_i/pitch_i = 0 (I-term computed in FC)
  //
  // 4. DesireCoord (→ desire_coordinate):
  //      roll=0, pitch=0, yaw=0 (formation frame = world, no offset)
  //
  // 5. UInt8 gimbal_dof = 1 (→ gimbal_dof):
  //      Ensure spinal operates in 2D vectoring mode.

  for (const auto& kv : module_commands_) {
    int module_id = kv.first;
    const ModuleCommand& cmd = kv.second;

    // --- 1. FourAxisCommand: base_thrust (2D vectoring) + target angles ---
    if (module_thrust_pubs_.count(module_id)) {
      spinal::FourAxisCommand thrust_msg;
      thrust_msg.base_thrust = cmd.base_thrust_2d;  // size 8
      thrust_msg.angles[0] = static_cast<float>(target_roll);
      thrust_msg.angles[1] = static_cast<float>(target_pitch);
      thrust_msg.angles[2] = static_cast<float>(candidate_yaw_term);
      module_thrust_pubs_[module_id].publish(thrust_msg);
    }

    // --- 2. TorqueAllocationMatrixInv (8 rows × 3 axes, int16 scaled ×1000) ---
    if (module_torque_alloc_pubs_.count(module_id)) {
      spinal::TorqueAllocationMatrixInv alloc_msg;
      int rows = cmd.torque_alloc_inv.rows();  // 8 = motor_num * rotor_coef
      alloc_msg.rows.resize(rows);
      if (cmd.torque_alloc_inv.cwiseAbs().maxCoeff() > INT16_MAX * 0.001)
        ROS_ERROR("[UnifiedCtrl] TorqueAllocationMatrixInv overflow for module %d", module_id);
      for (int i = 0; i < rows; i++) {
        alloc_msg.rows[i].x = static_cast<int16_t>(cmd.torque_alloc_inv(i, 0) * 1000);
        alloc_msg.rows[i].y = static_cast<int16_t>(cmd.torque_alloc_inv(i, 1) * 1000);
        alloc_msg.rows[i].z = static_cast<int16_t>(cmd.torque_alloc_inv(i, 2) * 1000);
      }
      module_torque_alloc_pubs_[module_id].publish(alloc_msg);
    }

    // --- 3. RollPitchYawTerms: RPY PID gains for spinal, scaled ×1000 ---
    // Same format as GimbalrotorController::setAttitudeGains() with i_term_rp_calc_in_pc=true
    if (module_rpy_gain_pubs_.count(module_id)) {
      spinal::RollPitchYawTerms gain_msg;
      gain_msg.motors.resize(1);
      gain_msg.motors[0].roll_p  = static_cast<int16_t>(rpy_p_gains.size() > 0 ? rpy_p_gains[0] * 1000 : 0);
      gain_msg.motors[0].roll_i  = static_cast<int16_t>(rpy_i_gains.size() > 0 ? rpy_i_gains[0] * 1000 : 0);
      gain_msg.motors[0].roll_d  = static_cast<int16_t>(rpy_d_gains.size() > 0 ? rpy_d_gains[0] * 1000 : 0);
      gain_msg.motors[0].pitch_p = static_cast<int16_t>(rpy_p_gains.size() > 1 ? rpy_p_gains[1] * 1000 : 0);
      gain_msg.motors[0].pitch_i = static_cast<int16_t>(rpy_i_gains.size() > 1 ? rpy_i_gains[1] * 1000 : 0);
      gain_msg.motors[0].pitch_d = static_cast<int16_t>(rpy_d_gains.size() > 1 ? rpy_d_gains[1] * 1000 : 0);
      gain_msg.motors[0].yaw_d   = static_cast<int16_t>(rpy_d_gains.size() > 2 ? rpy_d_gains[2] * 1000 : 0);
      module_rpy_gain_pubs_[module_id].publish(gain_msg);
    }

    // --- 4. DesireCoord: zero (formation frame = world) ---
    if (module_desire_coord_pubs_.count(module_id)) {
      spinal::DesireCoord coord_msg;
      coord_msg.roll = 0;
      coord_msg.pitch = 0;
      coord_msg.yaw = 0;
      module_desire_coord_pubs_[module_id].publish(coord_msg);
    }

    // --- 5. UInt8 gimbal_dof = 1: ensure 2D vectoring mode ---
    if (module_gimbal_dof_pubs_.count(module_id)) {
      std_msgs::UInt8 dof_msg;
      dof_msg.data = 1;
      module_gimbal_dof_pubs_[module_id].publish(dof_msg);
    }
  }

  ROS_DEBUG_THROTTLE(1.0, "[UnifiedCtrl] Published 2D vectoring commands for %zu modules "
                     "(entries_per_module=%d, roll=%.3f, pitch=%.3f, yaw_term=%.3f)",
                     module_commands_.size(), motor_num_per_module_ * rotor_coef_,
                     target_roll, target_pitch, candidate_yaw_term);
}

} // namespace aerial_robot_control
