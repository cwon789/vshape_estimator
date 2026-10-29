#include <gtest/gtest.h>
#include "ros2_bot_estimator_lidar_vshape/output.hpp"
using namespace vshape;
TEST(Output, ChoosesHeadingAwayFromAMROnFirstAcquisition) {
  const double pi = 3.141592653589793;
  for (const SE2& geometric : {SE2(2, 0, pi), SE2(-2, 0, 0), SE2(0, 2, -pi/2), SE2(0, -2, pi/2)}) {
    DockingConvention convention;
    SE2 result = convention.apply(geometric);
    EXPECT_LT((result.p - geometric.p).norm(), 1e-12);
    EXPECT_GT(Vec2(std::cos(result.yaw), std::sin(result.yaw)).dot(result.p), 1.9);
  }
}
TEST(Output, PreservesRealRotationInsteadOfFlippingPiAgain) {
  DockingConvention convention;
  EXPECT_NEAR(convention.apply(SE2(2,0,3.141592653589793)).yaw, 0., 1e-12);
  EXPECT_NEAR(convention.apply(SE2(2,0,3.141592653589793+2.)).yaw, 2., 1e-12);
  convention.reset();
  EXPECT_GT(std::cos(convention.apply(SE2(2,0,3.141592653589793+2.)).yaw), 0.);
}
TEST(Output, AdditiveCovarianceIsUnaffectedByFixedHeadingBranch) {
  DockingConvention convention;
  SE2 x(2, -.3, 2.9); Mat3 p = Mat3::Identity() * .01;
  convention.apply(x.inverse());
  Mat3 numerical;
  for (int j = 0; j < 3; ++j) {
    Vec3 e = Vec3::Zero(); e[j] = 1e-6;
    Vec3 plus = convention.apply((x*SE2::exp(e)).inverse()).vector();
    Vec3 minus = convention.apply((x*SE2::exp(-e)).inverse()).vector();
    Vec3 difference=plus-minus; difference.z()=wrap(difference.z());
    numerical.col(j)=difference/2e-6;
  }
  EXPECT_LT((inversePoseCovariance(x,p)-numerical*p*numerical.transpose()).norm(),1e-8);
}
TEST(Output, DockingOriginIsTheOpeningFaceMidpoint) {
  const double arm = .15, half_angle = 3.141592653589793 / 3.;
  const double depth = arm * std::cos(half_angle);
  for (double yaw : {0., .7, 3.141592653589793, -2.}) {
    SE2 marker(2., -.3, yaw);
    Vec2 end0 = marker * Vec2(arm * std::cos(half_angle), -arm * std::sin(half_angle));
    Vec2 end1 = marker * Vec2(arm * std::cos(half_angle), arm * std::sin(half_angle));
    auto face = openingFacePose(marker, depth);
    EXPECT_LT((face.p - (end0 + end1) / 2).norm(), 1e-12);
    EXPECT_NEAR(face.yaw, marker.yaw, 1e-12);
  }
  DockingConvention convention;
  auto docking = convention.apply(openingFacePose(SE2(2.,0.,3.141592653589793), depth));
  EXPECT_NEAR(docking.p.x(), 1.925, 1e-12);
  EXPECT_NEAR(docking.yaw, 0., 1e-12);
}
TEST(Output, FaceMidpointCovarianceIncludesYawLeverArm) {
  SE2 x(1.2, -.4, 2.8);
  Mat3 p; p << .03, .004, .001, .004, .02, -.002, .001, -.002, .006;
  const double depth = .075;
  const Mat3 actual = openingFaceCovariance(x.inverse(), inversePoseCovariance(x,p), depth);
  Mat3 j;
  for (int k = 0; k < 3; ++k) {
    Vec3 e = Vec3::Zero(); e[k] = 1e-6;
    Vec3 a = openingFacePose((x * SE2::exp(e)).inverse(), depth).vector();
    Vec3 b = openingFacePose((x * SE2::exp(-e)).inverse(), depth).vector();
    Vec3 diff = a-b; diff.z() = wrap(diff.z()); j.col(k) = diff / 2e-6;
  }
  EXPECT_LT((actual-j*p*j.transpose()).norm(), 1e-8);
  EXPECT_GT((actual-inversePoseCovariance(x,p)).norm(), 1e-5);
}
