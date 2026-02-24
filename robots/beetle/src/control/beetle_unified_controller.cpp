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
    rotor_coef_(2)
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
  int max_modules = navigator_->getMaxModuleNum();
  std::string my_name = navigator_->getMyName();
  for (int i = 1; i <= max_modules; i++) {
    std::string ns = std::string("/") + my_name + std::to_string(i);
    module_thrust_pubs_[i] = nh_.advertise<spinal::FourAxisCommand>(ns + "/unified_thrust_cmd", 1);
    module_gimbal_pubs_[i] = nh_.advertise<sensor_msgs::JointState>(ns + "/unified_gimbal_cmd", 1);
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
  formation_cog_offset /= N;

  Eigen::Matrix3d formation_inertia = computeFormationInertia(assembled_ids, formation_cog_offset);

  // Build the formation-wide allocation matrix (relative to formation CoG)
  integrated_map_ = buildFormationAllocationMatrix(assembled_ids, formation_mass, formation_inertia, formation_cog_offset);

  // ---- Wrench acc: PID output is already in acceleration space ----
  // Allocation matrix is built relative to formation CoG, and PID angular
  // acceleration output is frame-independent (rigid body). No wrench
  // transform (d × F) is needed — same approach as assemble_quadrotors.
  Eigen::VectorXd total_wrench_acc = target_wrench_acc_cog;
  if (desired_ext_wrench.size() >= 6 && desired_ext_wrench.norm() > 1e-6) {
    double mass_inv = 1.0 / formation_mass;
    Eigen::Matrix3d inertia_inv = formation_inertia.inverse();
    total_wrench_acc.head(3) += mass_inv * desired_ext_wrench.head(3);
    total_wrench_acc.tail(3) += inertia_inv * desired_ext_wrench.tail(3);
  }

  // ===== DIAGNOSTIC: formation inertia & allocation matrix =====
  {
    ROS_INFO_THROTTLE(1.0, "[UnifiedCtrl DIAG] formation_cog_offset=(%.4f,%.4f,%.4f)",
                      formation_cog_offset.x(), formation_cog_offset.y(), formation_cog_offset.z());
    ROS_INFO_THROTTLE(1.0, "[UnifiedCtrl DIAG] formation_mass=%.3f, N=%d, single_mass=%.3f",
                      formation_mass, N, single_mass);
    ROS_INFO_THROTTLE(1.0, "[UnifiedCtrl DIAG] formation_inertia diag=(%.4f, %.4f, %.4f)",
                      formation_inertia(0,0), formation_inertia(1,1), formation_inertia(2,2));
    ROS_INFO_THROTTLE(1.0, "[UnifiedCtrl DIAG] formation_inertia off-diag=(%.4f, %.4f, %.4f)",
                      formation_inertia(0,1), formation_inertia(0,2), formation_inertia(1,2));

    // Allocation matrix condition number via SVD
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(integrated_map_);
    double cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size()-1);
    ROS_INFO_THROTTLE(1.0, "[UnifiedCtrl DIAG] integrated_map size=(%ld x %ld), cond_num=%.4f, sv_max=%.6f, sv_min=%.6f",
                      integrated_map_.rows(), integrated_map_.cols(), cond,
                      svd.singularValues()(0), svd.singularValues()(svd.singularValues().size()-1));
  }

  // Solve allocation: pseudoinverse
  Eigen::MatrixXd integrated_map_inv = aerial_robot_model::pseudoinverse(integrated_map_);
  target_vectoring_f_ = integrated_map_inv * total_wrench_acc;

  // ===== DIAGNOSTIC: verify allocation roundtrip =====
  {
    Eigen::VectorXd reconstructed = integrated_map_ * target_vectoring_f_;
    Eigen::VectorXd residual = reconstructed - total_wrench_acc;
    ROS_INFO_THROTTLE(1.0, "[UnifiedCtrl DIAG] allocation residual norm=%.6e", residual.norm());
    ROS_INFO_THROTTLE(1.0, "[UnifiedCtrl DIAG] reconstructed wrench_acc=(%.4f,%.4f,%.4f,%.4f,%.4f,%.4f)",
                      reconstructed(0), reconstructed(1), reconstructed(2),
                      reconstructed(3), reconstructed(4), reconstructed(5));
  }

  // Extract per-rotor thrust magnitudes and gimbal angles
  extractThrustAndGimbal(target_vectoring_f_, assembled_ids);

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

void BeetleUnifiedController::extractThrustAndGimbal(
    const Eigen::VectorXd& vectoring_f,
    const std::vector<int>& assembled_ids)
{
  int N = assembled_ids.size();
  module_commands_.clear();

  std::stringstream diag_alloc_ss;
  char abuf[256];

  int col = 0;
  for (int m = 0; m < N; m++) {
    int module_id = assembled_ids[m];
    ModuleCommand cmd;
    cmd.full_thrusts.resize(motor_num_per_module_);
    cmd.gimbal_angles.resize(motor_num_per_module_ * gimbal_dof_);

    for (int r = 0; r < motor_num_per_module_; r++) {
      Eigen::VectorXd f_i = vectoring_f.segment(col, rotor_coef_);

      if (gimbal_dof_ == 1) {
        // Clamp: if f_i[1] < 0 (thrust pointing down), flip to ensure upward thrust.
        // This keeps gimbal within [-π/2, π/2], matching URDF joint limits.
        if (f_i[1] < 0) {
          f_i[0] = 0;
          f_i[1] = std::abs(f_i[1]);
        }
        cmd.full_thrusts[r] = f_i.norm();
        cmd.gimbal_angles[r] = atan2(-f_i[0], f_i[1]);

        snprintf(abuf, sizeof(abuf), " m%d_r%d:fi=(%.3f,%.3f)T=%.3f g=%.1fdeg",
                 module_id, r+1, f_i[0], f_i[1],
                 cmd.full_thrusts[r], cmd.gimbal_angles[r] * 180.0 / M_PI);
        diag_alloc_ss << abuf;
      } else if (gimbal_dof_ == 2) {
        cmd.full_thrusts[r] = f_i.norm();
        if (f_i[0] != 0 && f_i[2] != 0) {
          cmd.gimbal_angles[2 * r] = atan2(-f_i[1], f_i[2]);
          double gimbal_roll = cmd.gimbal_angles[2 * r];
          cmd.gimbal_angles[2 * r + 1] =
              atan2(f_i[0], -f_i[1] * sin(gimbal_roll) + f_i[2] * cos(gimbal_roll));
        }
        snprintf(abuf, sizeof(abuf), " m%d_r%d:T=%.3f g2=(%.1f,%.1f)deg",
                 module_id, r+1, cmd.full_thrusts[r],
                 cmd.gimbal_angles[2*r] * 180.0 / M_PI,
                 cmd.gimbal_angles[2*r+1] * 180.0 / M_PI);
        diag_alloc_ss << abuf;
      }

      col += rotor_coef_;
    }

    module_commands_[module_id] = cmd;
  }
  ROS_INFO_THROTTLE(1.0, "[UnifiedCtrl DIAG] alloc results:%s", diag_alloc_ss.str().c_str());
}

void BeetleUnifiedController::publishCommands()
{
  ros::Time now = ros::Time::now();

  for (const auto& kv : module_commands_) {
    int module_id = kv.first;
    const ModuleCommand& cmd = kv.second;

    // Publish thrust command (same format as spinal::FourAxisCommand)
    if (module_thrust_pubs_.count(module_id)) {
      spinal::FourAxisCommand thrust_msg;
      // FourAxisCommand.base_thrust is vector<float>
      thrust_msg.base_thrust = cmd.full_thrusts;
      // Set roll/pitch to 0 — unified controller handles everything via thrust vectoring
      thrust_msg.angles[0] = 0;
      thrust_msg.angles[1] = 0;
      thrust_msg.angles[2] = 0;
      module_thrust_pubs_[module_id].publish(thrust_msg);
    }

    // Publish gimbal command
    if (module_gimbal_pubs_.count(module_id)) {
      sensor_msgs::JointState gimbal_msg;
      gimbal_msg.header.stamp = now;
      for (int r = 0; r < motor_num_per_module_; r++) {
        if (gimbal_dof_ == 1) {
          gimbal_msg.position.push_back(cmd.gimbal_angles[r]);
          gimbal_msg.name.push_back("gimbal" + std::to_string(r + 1));
        } else if (gimbal_dof_ == 2) {
          gimbal_msg.position.push_back(cmd.gimbal_angles[2 * r]);
          gimbal_msg.position.push_back(cmd.gimbal_angles[2 * r + 1]);
          gimbal_msg.name.push_back("gimbal" + std::to_string(r + 1) + "_roll");
          gimbal_msg.name.push_back("gimbal" + std::to_string(r + 1) + "_pitch");
        }
      }
      module_gimbal_pubs_[module_id].publish(gimbal_msg);
    }
  }

  ROS_DEBUG_THROTTLE(1.0, "[UnifiedCtrl] Published commands for %zu modules",
                     module_commands_.size());
}

} // namespace aerial_robot_control
