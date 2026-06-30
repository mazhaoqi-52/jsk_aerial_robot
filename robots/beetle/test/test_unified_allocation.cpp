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
    const Eigen::MatrixXd empty_mat(0, 0);
    return ctrl_.solveFullVectorQP(A, w_control, empty, empty, empty, empty, empty,
                                   secondary_ref, ids, empty_mat, empty, f_out);
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

// (4) A vertical gimbal limit (pi/2) must not break QP conditioning. Before the
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

}  // namespace aerial_robot_control

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_unified_allocation");
  return RUN_ALL_TESTS();
}
