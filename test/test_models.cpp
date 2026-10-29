#include <gtest/gtest.h>
#include "ros2_bot_estimator_lidar_vshape/se2.hpp"
#include <random>

using namespace vshape;

TEST(SE2, ExpLogAndInverse) {
  for (double angle : {0.0, 1e-10, -1.2, 3.141592653589 - 1e-6}) {
    Vec3 x(0.7, -0.4, angle);
    auto t = SE2::exp(x);
    EXPECT_LT((t.log() - x).norm(), 1e-9);
    EXPECT_LT((t * t.inverse()).log().norm(), 1e-9);
  }
}

TEST(SE2, AdjointTransportsPerturbation) {
  SE2 t(1.2, -0.5, 0.9);
  Vec3 d(1e-6, -2e-6, 3e-6);
  EXPECT_LT(((t * SE2::exp(d) * t.inverse()).log() - t.adjoint() * d).norm(), 1e-12);
}

TEST(SE2, MeasurementAndRetractionJacobians) {
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> unif(-2.0, 2.0);
  for (int k = 0; k < 100; ++k) {
    SE2 t(unif(rng), unif(rng), unif(rng));
    Vec2 p(unif(rng), unif(rng)), n(0.6, 0.8);
    Eigen::RowVector3d numerical;
    for (int j = 0; j < 3; ++j) {
      Vec3 e = Vec3::Zero(); e[j] = 1e-6;
      numerical[j] = (n.dot((t * SE2::exp(e)) * p) -
                      n.dot((t * SE2::exp(-e)) * p)) / 2e-6;
    }
    EXPECT_LT((pointLineJacobian(t, p, n) - numerical).norm(), 1e-6);
    Vec3 delta(unif(rng) * .1, unif(rng) * .1, unif(rng) * .1);
    Mat3 reset, prior;
    for (int j = 0; j < 3; ++j) {
      Vec3 e = Vec3::Zero(); e[j] = 1e-6;
      reset.col(j) = ((SE2::exp(delta).inverse() * SE2::exp(delta + e)).log() -
                      (SE2::exp(delta).inverse() * SE2::exp(delta - e)).log()) / 2e-6;
      prior.col(j) = ((SE2::exp(delta) * SE2::exp(e)).log() -
                      (SE2::exp(delta) * SE2::exp(-e)).log()) / 2e-6;
    }
    EXPECT_LT((rightJacobian(delta) - reset).norm(), 1e-6);
    EXPECT_LT((rightJacobianInverse(delta) - prior).norm(), 1e-6);
  }
}

TEST(SE2, PoseCovarianceAndInversionUseCorrectFrames) {
  SE2 t(1.2, -.7, 1.1);
  Mat3 p; p << .04, .01, .002, .01, .03, -.001, .002, -.001, .005;
  Mat3 j;
  for (int k = 0; k < 3; ++k) {
    Vec3 e = Vec3::Zero(); e[k] = 1e-6;
    j.col(k) = ((t * SE2::exp(e)).vector() - (t * SE2::exp(-e)).vector()) / 2e-6;
  }
  EXPECT_LT((poseCovariance(t, p) - j * p * j.transpose()).norm(), 1e-8);
  Mat3 ji;
  for (int k = 0; k < 3; ++k) {
    Vec3 e = Vec3::Zero(); e[k] = 1e-6;
    ji.col(k) = ((t * SE2::exp(e)).inverse().vector() -
                 (t * SE2::exp(-e)).inverse().vector()) / 2e-6;
  }
  EXPECT_LT((inversePoseCovariance(t, p) - ji * p * ji.transpose()).norm(), 1e-8);
}
TEST(SE2, MotionTransitionMatchesTangentFiniteDifferences) {
  SE2 x(1.2, -.7, 1.1); Vec3 u(.3, -.1, .6); const double dt = .17;
  SE2 predicted = x * SE2::exp(u * dt);
  Mat3 numerical;
  for (int j = 0; j < 3; ++j) {
    Vec3 e = Vec3::Zero(); e[j] = 1e-6;
    numerical.col(j) = ((predicted.inverse() * (x * SE2::exp(e) * SE2::exp(u * dt))).log() -
                       (predicted.inverse() * (x * SE2::exp(-e) * SE2::exp(u * dt))).log()) / 2e-6;
  }
  EXPECT_LT((numerical - SE2::exp(-u * dt).adjoint()).norm(), 1e-6);
}
