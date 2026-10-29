#include <gtest/gtest.h>
#include <Eigen/Eigenvalues>
#include "ros2_bot_estimator_lidar_vshape/filter.hpp"
using namespace vshape;

std::vector<Observation> observations(const SE2& truth, const Config& c, int only = -1) {
  std::vector<Observation> obs;
  for (int arm = 0; arm < 2; ++arm) {
    if (only >= 0 && arm != only) continue;
    for (int i = 0; i < 20; ++i) {
      Point p; p.q = truth.inverse() * (armDirection(c, arm) * (.04 + i * .02));
      p.covariance = Mat2::Identity() * c.range_sigma * c.range_sigma;
      obs.push_back({p, arm});
    }
  }
  return obs;
}
TEST(Filter, RecoversPoseAndKeepsCovariancePSD) {
  Config c; Filter f(c); SE2 truth(2., .1, 3.0);
  f.initialize(0, truth * SE2::exp(Vec3(.1, -.05, .08)), Mat3::Identity() * .1);
  auto obs = observations(truth, c);
  for (int i = 0; i < 10; ++i) {
    ASSERT_TRUE(f.update(obs));
    EXPECT_GT(Eigen::SelfAdjointEigenSolver<Mat3>(f.state().covariance).eigenvalues().minCoeff(), 0);
  }
  EXPECT_LT((truth.inverse() * f.state().robot_in_marker).log().norm(), .002);
  EXPECT_LT((f.state().covariance - f.state().covariance.transpose()).norm(), 1e-12);
}
TEST(Filter, PartialArmPreservesUnobservedTangent) {
  Config c; Filter f(c); SE2 truth(0, 0, 0);
  f.initialize(0, truth, Mat3::Identity() * .1);
  Vec3 tangent; tangent << armDirection(c, 0), 0.;
  ASSERT_TRUE(f.update(observations(truth, c, 0)));
  EXPECT_NEAR(tangent.dot(f.state().covariance * tangent), .1, 1e-10);
  EXPECT_EQ(f.state().diagnostics.mode, "PARTIAL");
  EXPECT_LT(std::abs(f.state().diagnostics.information_eigenvalues[0]), 1e-8);
}
TEST(Filter, MissingInvalidAndRejectedMeasurementsAreAtomic) {
  Config c; Filter f(c); f.initialize(0, SE2(), Mat3::Identity() * .001);
  auto prior = f.state();
  EXPECT_FALSE(f.update({}));
  auto obs = observations(SE2(3, 2, 1), c);
  EXPECT_FALSE(f.update(obs));
  EXPECT_LT((f.state().robot_in_marker.inverse() * prior.robot_in_marker).log().norm(), 1e-12);
  EXPECT_LT((f.state().covariance - prior.covariance).norm(), 1e-12);
  obs[0].point.q.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(f.update(obs));
}
TEST(Filter, ExactMotionAndSubdivisionCovariance) {
  Config c; Filter f(c), g(c); SE2 initial(1, -2, .7);
  f.initialize(0, initial, Mat3::Identity() * .1); g = f;
  Vec3 u(.3, -.1, .4);
  f.predict(1., u);
  for (int k = 1; k <= 100; ++k) g.predict(k * .01, u);
  EXPECT_LT((f.state().robot_in_marker.inverse() * (initial * SE2::exp(u))).log().norm(), 1e-10);
  EXPECT_LT((f.state().covariance - g.state().covariance).norm(), 1e-9);
  auto p = f.state().covariance; auto x = f.state().robot_in_marker;
  f.predict(2., u, false);
  EXPECT_LT((f.state().robot_in_marker.inverse() * x).log().norm(), 1e-12);
  EXPECT_GT(f.state().covariance.trace(), p.trace());
}
TEST(Filter, InvalidConfigurationAndInitializationFail) {
  Config c; c.arm_length = -1;
  EXPECT_THROW(Filter f(c), std::invalid_argument);
  Filter f;
  EXPECT_THROW(f.initialize(0, SE2(), -Mat3::Identity()), std::invalid_argument);
}
TEST(Filter, UnconvergedIterationLimitDoesNotCommitPosterior) {
  Config c; c.max_iterations = 1;
  Filter f(c); SE2 truth(2, .1, 3.);
  f.initialize(0, truth * SE2::exp(Vec3(.1, -.05, .08)), Mat3::Identity() * .1);
  auto p = f.state().covariance; auto x = f.state().robot_in_marker;
  EXPECT_FALSE(f.update(observations(truth, c)));
  EXPECT_LT((p - f.state().covariance).norm(), 1e-12);
  EXPECT_LT((x.inverse() * f.state().robot_in_marker).log().norm(), 1e-12);
}
