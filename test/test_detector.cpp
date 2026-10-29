#include <gtest/gtest.h>
#include "ros2_bot_estimator_lidar_vshape/detector.hpp"
using namespace vshape;
Points cloud(const Config& c, const SE2& marker) {
  Points points;
  for (int arm = 0; arm < 2; ++arm) {
    for (int i = 0; i < 35; ++i) {
      Point p; p.q = marker * (armDirection(c, arm) * (.03 + i * .013)); p.beam = points.size();
      points.push_back(p);
    }
  }
  return points;
}
TEST(Detector, InitializesOnlyCorrectFiniteVInTargetROI) {
  Config c; SE2 marker(2, .1, 3.0); c.target_y = .1; c.target_yaw = 3.;
  auto p = cloud(c, marker);
  auto result = detectV(p, c);
  ASSERT_TRUE(result.valid);
  EXPECT_LT((result.marker_in_base.inverse() * marker).log().norm(), 1e-6);
  c.opening_angle_deg = 100;
  EXPECT_FALSE(detectV(p, c).valid);
  c.opening_angle_deg = 70; c.target_x = -2;
  EXPECT_FALSE(detectV(p, c).valid);
}
TEST(Detector, AssociationHonorsSupportEndpointsAndArmLabels) {
  Config c; SE2 marker(2, 0, 3.14);
  auto points = cloud(c, marker);
  Point extension; extension.q = marker * (armDirection(c, 0) * 1.0); points.push_back(extension);
  auto observations = associate(points, marker.inverse(), c);
  ASSERT_LE(observations.size(), 2u * c.max_points_per_arm);
  for (const auto& obs : observations) {
    Vec2 q = marker.inverse() * obs.point.q;
    EXPECT_LE(armDirection(c, obs.arm).dot(q), c.arm_length + c.endpoint_margin);
    EXPECT_LT(std::abs(perpendicular(armDirection(c, obs.arm)).dot(q)), 1e-8);
  }
  Points one(points.begin(), points.begin() + 35);
  EXPECT_FALSE(detectV(one, c).valid);
  EXPECT_GE(associate(one, marker.inverse(), c).size(), std::size_t(c.min_points));
}
TEST(Detector, RemoteCollinearBackgroundDoesNotPolluteFiniteMarker) {
  Config c; SE2 marker(2, .1, 3.0); c.target_y = .1; c.target_yaw = 3.;
  auto points = cloud(c, marker);
  for (int i = 0; i < 100; ++i) {
    Point p; p.q = marker * (armDirection(c, i % 2) * (2 + i * .03)); points.push_back(p);
  }
  auto result = detectV(points, c);
  ASSERT_TRUE(result.valid);
  EXPECT_LT((result.marker_in_base.inverse() * marker).log().norm(), 1e-6);
}
TEST(Detector, ContinuousOverlongBackgroundArmsAreRejected) {
  Config c; SE2 marker(2, 0, 3.141592653589793); Points points;
  for (int arm = 0; arm < 2; ++arm) for (int k = 0; k < 100; ++k) {
    Point p; p.q = marker * (armDirection(c, arm) * (.03 + k * .01));
    p.beam = points.size(); points.push_back(p);
  }
  EXPECT_FALSE(detectV(points, c).valid);
}
