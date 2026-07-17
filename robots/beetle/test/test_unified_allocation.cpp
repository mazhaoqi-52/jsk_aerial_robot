// Offline unit test for the unified-mode allocation QP (solveFullVectorQP).
//
// Runs without ROS node / hardware: it builds a synthetic allocation matrix,
// drives the private QP solver through a friend fixture, and checks the
// *properties* the solver must guarantee (wrench tracking, actuator bounds,
// gimbal-angle limit, module-balance shaping). This is the regression guard
// that makes deeper refactoring of solveFullVectorQP safe.
//
// Note: friendship is granted to this fixture only, and is not inherited by the
// TEST_F subclasses, so every access to private members of the controller goes
// through fixture methods (configure / solve) defined here.

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <cmath>
#include <limits>
#include <map>
#include <vector>
#include <beetle/control/beetle_unified_controller.h>

namespace aerial_robot_control
{

class BeetleUnifiedAllocTest : public ::testing::Test
{
protected:
  BeetleUnifiedController ctrl_;
  static constexpr int kModules = 2;
  static constexpr int kRotorsPerModule = 4;
  static constexpr int kRotorCoef = 2;
  static constexpr int kCols = kModules * kRotorsPerModule * kRotorCoef;  // 16

  // Tunables the tests may change before calling configure(); also read back in
  // assertions so the tests never touch the controller's private members.
  double t_max_ = 20.0;
  // A realistic finite limit. Avoid exactly pi/2: tan(pi/2) ~ 1.6e16 produces a
  // pathologically scaled gimbal-constraint row that wrecks QP conditioning.
  double gimbal_limit_ = 1.0;  // ~57 deg
  double lambda_ = 1e-4;
  double balance_weight_ = 0.0;
  double wrench_weight_ = 10.0;
  double interface_force_limit_ = 0.0;
  double interface_torque_limit_ = 0.0;

  void SetUp() override
  {
    if (!ros::Time::isValid()) ros::Time::init();  // logQpDiagnostics uses ros::Time::now()
    configure();
  }

  // Push the fixture tunables into the controller and reset the QP solver state.
  void configure()
  {
    ctrl_.rotor_coef_ = kRotorCoef;
    ctrl_.motor_num_per_module_ = kRotorsPerModule;
    ctrl_.alloc_t_max_ = t_max_;
    ctrl_.alloc_gimbal_limit_rad_ = gimbal_limit_;
    ctrl_.alloc_lambda_ = lambda_;
    ctrl_.alloc_effort_weight_ = 0.0;
    ctrl_.alloc_module_balance_weight_ = balance_weight_;
    ctrl_.alloc_interface_force_limit_ = interface_force_limit_;
    ctrl_.alloc_interface_torque_limit_ = interface_torque_limit_;
    ctrl_.alloc_wrench_weights_ = Eigen::VectorXd::Ones(6) * wrench_weight_;
    ctrl_.prev_vectoring_f_.resize(0);
    ctrl_.resetQPState();
    // Push the throttled QP diagnostic far into the future so logQpDiagnostics
    // returns before emitting any ROS log (which would try to start a node and
    // contact a master). Keeps the test purely offline.
    ctrl_.last_qp_diag_log_time_ = std::numeric_limits<double>::max();
  }

  // Synthetic allocation matrix: rotor i at position p_i contributes force
  // (fx along body-x, fz along body-z) and torque p_i x force. Two modules
  // offset along body-y; four rotors each at +/-(0.15) corners.
  Eigen::MatrixXd buildA() const
  {
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(6, kCols);
    const double mod_y[kModules] = {-0.3, 0.3};
    const double lx[kRotorsPerModule] = {0.15, 0.15, -0.15, -0.15};
    const double ly[kRotorsPerModule] = {0.15, -0.15, 0.15, -0.15};
    for (int m = 0; m < kModules; ++m) {
      for (int r = 0; r < kRotorsPerModule; ++r) {
        const double px = lx[r];
        const double py = mod_y[m] + ly[r];
        const int base = (m * kRotorsPerModule + r) * kRotorCoef;
        const int fx = base, fz = base + 1;
        A(0, fx) = 1.0;   // Fx
        A(5, fx) = -py;   // Tz
        A(2, fz) = 1.0;   // Fz
        A(3, fz) = py;    // Tx
        A(4, fz) = -px;   // Ty
      }
    }
    return A;
  }

  bool solve(const Eigen::MatrixXd& A, const Eigen::VectorXd& w_control,
             const Eigen::VectorXd& secondary_ref, Eigen::VectorXd& f_out)
  {
    const std::vector<int> ids = {1, 2};
    const Eigen::VectorXd empty;
    const Eigen::MatrixXd empty_matrix;
    return ctrl_.solveFullVectorQP(A, w_control, empty, empty, empty, empty, empty,
                                   secondary_ref, ids, empty_matrix, empty, f_out);
  }

  // Solve with an explicit task wrench whose active rows are promoted to hard
  // priority bands (tolerances > 0, weight >= min task weight).
  bool solveWithTaskBand(const Eigen::MatrixXd& A, const Eigen::VectorXd& w_control,
                         const Eigen::VectorXd& w_task, const Eigen::VectorXd& task_weights,
                         const Eigen::VectorXd& tolerances, Eigen::VectorXd& f_out)
  {
    ctrl_.alloc_task_priority_enabled_ = true;
    ctrl_.alloc_priority_tolerances_ = tolerances;
    const std::vector<int> ids = {1, 2};
    const Eigen::VectorXd empty;
    const Eigen::MatrixXd empty_matrix;
    return ctrl_.solveFullVectorQP(A, w_control, w_task, task_weights,
                                   empty, empty, empty,
                                   Eigen::VectorXd::Zero(kCols), ids,
                                   empty_matrix, empty,
                                   f_out);
  }

  bool solveWithInterfaceModel(const Eigen::MatrixXd& A,
                               const Eigen::VectorXd& w_control,
                               const Eigen::MatrixXd& D,
                               const Eigen::VectorXd& d,
                               Eigen::VectorXd& f_out)
  {
    const std::vector<int> ids = {1, 2};
    const Eigen::VectorXd empty;
    return ctrl_.solveFullVectorQP(A, w_control, empty, empty, empty, empty, empty,
                                   Eigen::VectorXd::Zero(kCols), ids, D, d, f_out);
  }

  bool solveWithTaskAndInterfaceModel(const Eigen::MatrixXd& A,
                                      const Eigen::VectorXd& w_control,
                                      const Eigen::VectorXd& w_task,
                                      const Eigen::VectorXd& task_weights,
                                      const Eigen::VectorXd& tolerances,
                                      const Eigen::MatrixXd& D,
                                      const Eigen::VectorXd& d,
                                      Eigen::VectorXd& f_out)
  {
    ctrl_.alloc_task_priority_enabled_ = true;
    ctrl_.alloc_priority_tolerances_ = tolerances;
    const std::vector<int> ids = {1, 2};
    const Eigen::VectorXd empty;
    return ctrl_.solveFullVectorQP(
        A, w_control, w_task, task_weights, empty, empty, empty,
        Eigen::VectorXd::Zero(kCols), ids, D, d, f_out);
  }

  bool physicalChainOrder(const std::vector<int>& allocation_ids,
                          const std::map<int, Eigen::Vector3d>& offsets,
                          std::vector<int>& chain_ids,
                          std::map<int, int>& allocation_indices)
  {
    ctrl_.cached_module_offsets_from_leader_ = offsets;
    return ctrl_.getPhysicalChainOrder(allocation_ids, chain_ids, allocation_indices);
  }

  bool buildThreeModuleInterfaceModel(const Eigen::VectorXd& control_wrench_acc,
                                      Eigen::MatrixXd& D,
                                      Eigen::VectorXd& d,
                                      std::vector<std::pair<int, int>>& cuts)
  {
    const std::vector<int> allocation_ids = {1, 2, 3};
    ctrl_.formation_cog_offset_.setZero();
    ctrl_.cached_module_offsets_from_leader_ = {
        {1, Eigen::Vector3d(1.0, 0.0, 0.0)},
        {2, Eigen::Vector3d(-1.0, 0.0, 0.0)},
        {3, Eigen::Vector3d(0.0, 0.0, 0.0)}};

    BeetleUnifiedController::ModuleModelDescriptor model;
    model.mass = 1.0;
    model.inertia = 0.1 * Eigen::Matrix3d::Identity();
    model.rotor_origins_from_cog.assign(kRotorsPerModule, Eigen::Vector3d::Zero());
    model.mf_rate = 0.0;
    for (int r = 0; r < kRotorsPerModule; r++) model.rotor_direction[r + 1] = 1;
    for (int id : allocation_ids) ctrl_.setModuleModelDescriptor(id, model);

    Eigen::MatrixXd mask(3, 2);
    mask << 1.0, 0.0,
            0.0, 0.0,
            0.0, 1.0;
    const std::vector<Eigen::MatrixXd> masks(kRotorsPerModule, mask);
    return ctrl_.buildInterfaceLoadModelWithMasks(
        allocation_ids, control_wrench_acc, masks, D, d, cuts);
  }

  static double moduleFzSum(const Eigen::VectorXd& f, int m)
  {
    double s = 0.0;
    for (int r = 0; r < kRotorsPerModule; ++r)
      s += f((m * kRotorsPerModule + r) * kRotorCoef + 1);
    return s;
  }
};

// (1) The solver tracks a feasible desired wrench and respects actuator bounds.
TEST_F(BeetleUnifiedAllocTest, TracksFeasibleWrench)
{
  const Eigen::MatrixXd A = buildA();
  Eigen::VectorXd w(6);
  w << 1.0, 0.0, 16.0, 0.5, 0.5, 0.2;
  Eigen::VectorXd f;
  ASSERT_TRUE(solve(A, w, Eigen::VectorXd::Zero(kCols), f));
  ASSERT_EQ(f.size(), kCols);

  const Eigen::VectorXd realized = A * f;
  // Fy (row 1) is unactuated in this synthetic model; its target is 0.
  for (int row : {0, 2, 3, 4, 5}) {
    EXPECT_NEAR(realized(row), w(row), 0.3) << "wrench row " << row;
  }
  for (int i = 0; i < kCols / kRotorCoef; ++i) {
    const double fx = f(2 * i), fz = f(2 * i + 1);
    EXPECT_GE(fz, -1e-6) << "rotor " << i;
    EXPECT_LE(std::sqrt(fx * fx + fz * fz), t_max_ + 1e-3) << "rotor " << i;
    EXPECT_LE(std::abs(std::atan2(-fx, fz)), gimbal_limit_ + 1e-3) << "rotor " << i;
  }
}

// (2) The per-rotor gimbal-angle limit is enforced.
TEST_F(BeetleUnifiedAllocTest, RespectsGimbalLimit)
{
  gimbal_limit_ = 0.3;  // ~17 deg
  configure();
  const Eigen::MatrixXd A = buildA();
  Eigen::VectorXd w(6);
  w << 1.0, 0.0, 16.0, 0.0, 0.0, 0.0;  // modest lateral demand, stays feasible
  Eigen::VectorXd f;
  ASSERT_TRUE(solve(A, w, Eigen::VectorXd::Zero(kCols), f));
  for (int i = 0; i < kCols / kRotorCoef; ++i) {
    const double fx = f(2 * i), fz = f(2 * i + 1);
    EXPECT_LE(std::abs(std::atan2(-fx, fz)), gimbal_limit_ + 1e-3) << "rotor " << i;
  }
}

// (3) The module-balance penalty reduces inter-module thrust spread that an
//     asymmetric secondary reference would otherwise induce.
TEST_F(BeetleUnifiedAllocTest, ModuleBalanceReducesSpread)
{
  const Eigen::MatrixXd A = buildA();
  Eigen::VectorXd w(6);
  w << 0.0, 0.0, 16.0, 0.0, 0.0, 0.0;  // symmetric lift, no torque demand

  Eigen::VectorXd fref = Eigen::VectorXd::Zero(kCols);
  for (int r = 0; r < kRotorsPerModule; ++r) {
    fref((0 * kRotorsPerModule + r) * kRotorCoef + 1) = 4.0;  // module 1 biased high
    fref((1 * kRotorsPerModule + r) * kRotorCoef + 1) = 0.5;  // module 2 biased low
  }
  wrench_weight_ = 0.1;  // weak tracking so the reference bias is visible
  lambda_ = 0.5;         // strong pull toward the asymmetric reference

  balance_weight_ = 0.0;
  configure();
  Eigen::VectorXd f_off;
  ASSERT_TRUE(solve(A, w, fref, f_off));
  const double spread_off = std::abs(moduleFzSum(f_off, 0) - moduleFzSum(f_off, 1));

  balance_weight_ = 10.0;
  configure();
  Eigen::VectorXd f_on;
  ASSERT_TRUE(solve(A, w, fref, f_on));
  const double spread_on = std::abs(moduleFzSum(f_on, 0) - moduleFzSum(f_on, 1));

  EXPECT_LT(spread_on, spread_off) << "spread off=" << spread_off << " on=" << spread_on;
}

// (4) Hard-banded task rows keep their soft tracking: the solution must sit
//     near the band CENTER (w_control + w_task), not be dragged to the band's
//     lower edge by the effort cost. Regression guard for the 2026-07-03
//     pushing log where task_prio_res was pinned at the tolerance for the
//     whole force ramp (contact force under-delivered by exactly tol).
TEST_F(BeetleUnifiedAllocTest, TaskBandTracksCenterNotEdge)
{
  configure();
  const Eigen::MatrixXd A = buildA();
  Eigen::VectorXd w_control(6), w_task(6), task_weights(6), tolerances(6);
  w_control << 0.0, 0.0, 16.0, 0.0, 0.0, 0.0;
  w_task << 2.0, 0.0, 0.0, 0.0, 0.0, 0.0;   // active task row: Fx
  task_weights << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  tolerances << 0.5, 0.0, 0.0, 0.0, 0.0, 0.0;

  Eigen::VectorXd f;
  ASSERT_TRUE(solveWithTaskBand(A, w_control, w_task, task_weights, tolerances, f));
  const double realized_fx = (A * f)(0);
  // Must stay inside the hard band ...
  EXPECT_GE(realized_fx, 2.0 - 0.5 - 1e-3);
  EXPECT_LE(realized_fx, 2.0 + 0.5 + 1e-3);
  // ... and near its center, not at the lower edge (1.5).
  EXPECT_NEAR(realized_fx, 2.0, 0.15);
}

// (5) A vertical gimbal limit (pi/2) must not break QP conditioning. Before the
//     cos/sin reformulation of the gimbal-angle constraint, tan(pi/2) ~ 1.6e16
//     produced a pathologically scaled row and the solver failed here.
TEST_F(BeetleUnifiedAllocTest, HandlesVerticalGimbalLimit)
{
  gimbal_limit_ = M_PI / 2.0;
  configure();
  const Eigen::MatrixXd A = buildA();
  Eigen::VectorXd w(6);
  w << 1.0, 0.0, 16.0, 0.5, 0.5, 0.2;
  Eigen::VectorXd f;
  ASSERT_TRUE(solve(A, w, Eigen::VectorXd::Zero(kCols), f));
  for (int i = 0; i < kCols / kRotorCoef; ++i) {
    const double fx = f(2 * i), fz = f(2 * i + 1);
    EXPECT_GE(fz, -1e-6) << "rotor " << i;
    EXPECT_LE(std::sqrt(fx * fx + fz * fz), t_max_ + 1e-3) << "rotor " << i;
  }
}

// (6) The affine interface constraint is centered on the model demand d, not
//     on raw actuator wrench D*f. This is the discrete QP form of
//     -F_bar <= d - D*f <= F_bar.
TEST_F(BeetleUnifiedAllocTest, InterfaceForceBoundUsesAffineDemand)
{
  interface_force_limit_ = 0.02;
  configure();
  const Eigen::MatrixXd A = buildA();
  Eigen::VectorXd w(6);
  w << 1.0, 0.0, 16.0, 0.0, 0.0, 0.0;

  Eigen::MatrixXd D = Eigen::MatrixXd::Zero(6, kCols);
  D(0, 0) = 1.0;
  Eigen::VectorXd d = Eigen::VectorXd::Zero(6);
  d(0) = 0.4;

  Eigen::VectorXd f;
  ASSERT_TRUE(solveWithInterfaceModel(A, w, D, d, f));
  EXPECT_LE(std::abs(d(0) - (D * f)(0)), interface_force_limit_ + 1e-4);
}

// (7) Force and torque limits gate different rows of each 6D interface block.
TEST_F(BeetleUnifiedAllocTest, InterfaceTorqueBoundUsesTorqueRowsOnly)
{
  interface_torque_limit_ = 0.015;
  configure();
  const Eigen::MatrixXd A = buildA();
  Eigen::VectorXd w(6);
  w << 1.0, 0.0, 16.0, 0.0, 0.0, 0.0;

  Eigen::MatrixXd D = Eigen::MatrixXd::Zero(6, kCols);
  D(3, 0) = 1.0;
  Eigen::VectorXd d = Eigen::VectorXd::Zero(6);
  d(3) = 0.3;

  Eigen::VectorXd f;
  ASSERT_TRUE(solveWithInterfaceModel(A, w, D, d, f));
  EXPECT_LE(std::abs(d(3) - (D * f)(3)), interface_torque_limit_ + 1e-4);
}

// (8) A task hard band must not bypass an incompatible structural hard bound.
TEST_F(BeetleUnifiedAllocTest, InfeasibleTaskAndInterfaceBoundsReturnFalse)
{
  interface_force_limit_ = 0.05;
  configure();
  const Eigen::MatrixXd A = buildA();
  Eigen::VectorXd w_control(6), w_task(6), task_weights(6);
  w_control << 0.0, 0.0, 16.0, 0.0, 0.0, 0.0;
  w_task << 2.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  task_weights << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  Eigen::VectorXd tolerances = Eigen::VectorXd::Zero(6);
  tolerances(0) = 0.1;

  Eigen::MatrixXd D = Eigen::MatrixXd::Zero(6, kCols);
  D.row(0) = A.row(0);
  const Eigen::VectorXd d = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd f;
  EXPECT_FALSE(solveWithTaskAndInterfaceModel(
      A, w_control, w_task, task_weights, tolerances, D, d, f));
}

// (9) Navigator assembly IDs are numerically sorted, but the physical chain is
//     defined by increasing body-X position (e.g. the real formation 2-3-1).
TEST_F(BeetleUnifiedAllocTest, PhysicalChainOrderDoesNotFollowNumericIds)
{
  const std::map<int, Eigen::Vector3d> offsets = {
      {1, Eigen::Vector3d(1.0, 0.0, 0.0)},
      {2, Eigen::Vector3d(-1.0, 0.0, 0.0)},
      {3, Eigen::Vector3d(0.0, 0.0, 0.0)}};
  std::vector<int> chain_ids;
  std::map<int, int> allocation_indices;
  ASSERT_TRUE(physicalChainOrder({1, 2, 3}, offsets,
                                 chain_ids, allocation_indices));
  ASSERT_EQ(chain_ids.size(), 3u);
  EXPECT_EQ(chain_ids[0], 2);
  EXPECT_EQ(chain_ids[1], 3);
  EXPECT_EQ(chain_ids[2], 1);
  EXPECT_EQ(allocation_indices.at(1), 0);
  EXPECT_EQ(allocation_indices.at(2), 1);
  EXPECT_EQ(allocation_indices.at(3), 2);
}

// (10) The actual cut model must use the contact-free negative-X subtree. For
//      allocation columns [id1,id2,id3] and physical chain 2-3-1, the two cuts
//      contain {2} and {2,3}; hover cancels d-Df, while equal task shares create
//      one and two units of transmitted interface load respectively.
TEST_F(BeetleUnifiedAllocTest, InterfaceModelUsesContactFreePhysicalSubtree)
{
  Eigen::VectorXd control = Eigen::VectorXd::Zero(6);
  control(2) = aerial_robot_estimation::G;
  Eigen::MatrixXd D;
  Eigen::VectorXd d;
  std::vector<std::pair<int, int>> cuts;
  ASSERT_TRUE(buildThreeModuleInterfaceModel(control, D, d, cuts));
  ASSERT_EQ(D.rows(), 12);
  ASSERT_EQ(D.cols(), 24);
  ASSERT_EQ(cuts.size(), 2u);
  EXPECT_EQ(cuts[0], std::make_pair(2, 3));
  EXPECT_EQ(cuts[1], std::make_pair(3, 1));

  // Allocation block order remains numeric [1,2,3]. Cut 2-3 contains only id2;
  // cut 3-1 contains id2 and id3, never the +X contact module id1.
  EXPECT_NEAR(D.block(0, 0, 6, 8).norm(), 0.0, 1e-12);
  EXPECT_GT(D.block(0, 8, 6, 8).norm(), 0.0);
  EXPECT_NEAR(D.block(0, 16, 6, 8).norm(), 0.0, 1e-12);
  EXPECT_NEAR(D.block(6, 0, 6, 8).norm(), 0.0, 1e-12);
  EXPECT_GT(D.block(6, 8, 6, 16).norm(), 0.0);

  Eigen::VectorXd f_hover = Eigen::VectorXd::Zero(24);
  for (int module = 0; module < 3; module++) {
    for (int rotor = 0; rotor < kRotorsPerModule; rotor++) {
      f_hover((module * kRotorsPerModule + rotor) * kRotorCoef + 1) =
          aerial_robot_estimation::G / kRotorsPerModule;
    }
  }
  EXPECT_NEAR((d - D * f_hover).norm(), 0.0, 1e-10);

  Eigen::VectorXd f_push = f_hover;
  for (int module = 0; module < 3; module++) {
    for (int rotor = 0; rotor < kRotorsPerModule; rotor++) {
      f_push((module * kRotorsPerModule + rotor) * kRotorCoef) =
          1.0 / kRotorsPerModule;
    }
  }
  const Eigen::VectorXd interface_wrench = d - D * f_push;
  EXPECT_NEAR(interface_wrench(0), -1.0, 1e-10);
  EXPECT_NEAR(interface_wrench(6), -2.0, 1e-10);
}

}  // namespace aerial_robot_control

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_unified_allocation");
  return RUN_ALL_TESTS();
}
