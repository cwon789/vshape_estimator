#include <gtest/gtest.h>
#include "ros2_bot_estimator_lidar_vshape/tracker.hpp"
using namespace vshape;
namespace {
// Independent analytic integration of a body twist; no production SE2 exp/log.
Vec3 circularMotion(const Vec3& twist, double dt) {
  const double yaw = twist.z() * dt;
  if (std::abs(twist.z()) < 1e-12) return {twist.x() * dt, twist.y() * dt, yaw};
  return {(twist.x() * std::sin(yaw) + twist.y() * (std::cos(yaw) - 1)) / twist.z(),
          (twist.x() * (1 - std::cos(yaw)) + twist.y() * std::sin(yaw)) / twist.z(), yaw};
}

void expectDeskewedWall(const Vec3& first_twist, const Vec3& second_twist) {
  Config c; c.lidar_in_base = SE2(.35, -.18, .23); c.odom_timeout = .5;
  Tracker tracker(c);
  constexpr double switch_time = .157; // Deliberately between adjacent beams.
  ASSERT_TRUE(tracker.addOdometry({2, first_twist}));
  ASSERT_TRUE(tracker.addOdometry({2 + switch_time, second_twist}));
  Scan scan; scan.stamp = 2; scan.angle_min = -.6; scan.angle_increment = .01;
  scan.time_increment = .003; scan.ranges.resize(121);
  std::vector<Vec2> truth;
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const double dt = i * scan.time_increment;
    const Vec3 a = circularMotion(first_twist, std::min(dt, switch_time));
    const Vec3 b = circularMotion(second_twist, std::max(0., dt - switch_time));
    const double cosine = std::cos(a.z()), sine = std::sin(a.z());
    const double x = a.x() + cosine * b.x() - sine * b.y();
    const double y = a.y() + sine * b.x() + cosine * b.y();
    const double yaw = a.z() + b.z();
    const double laser_x = x + std::cos(yaw) * c.lidar_in_base.p.x() - std::sin(yaw) * c.lidar_in_base.p.y();
    const double laser_y = y + std::sin(yaw) * c.lidar_in_base.p.x() + std::cos(yaw) * c.lidar_in_base.p.y();
    const double direction = yaw + c.lidar_in_base.yaw + scan.angle_min + i * scan.angle_increment;
    const double range = (4 - laser_x) / std::cos(direction);
    scan.ranges[i] = range;
    truth.emplace_back(4, laser_y + range * std::sin(direction));
  }
  Points corrected; std::string reason = "previous failure";
  ASSERT_TRUE(tracker.deskewScan(scan, corrected, reason)) << reason;
  EXPECT_TRUE(reason.empty());
  ASSERT_EQ(corrected.size(), truth.size());
  double raw_squared_error = 0, corrected_squared_error = 0;
  for (std::size_t i = 0; i < corrected.size(); ++i) {
    EXPECT_EQ(corrected[i].beam, int(i));
    EXPECT_LT((corrected[i].q - truth[i]).norm(), 1e-11);
    EXPECT_TRUE(corrected[i].covariance.allFinite());
    const double angle = scan.angle_min + i * scan.angle_increment;
    const Vec2 raw = c.lidar_in_base * Vec2(scan.ranges[i] * std::cos(angle), scan.ranges[i] * std::sin(angle));
    raw_squared_error += std::pow(raw.x() - 4, 2);
    corrected_squared_error += std::pow(corrected[i].q.x() - 4, 2);
  }
  EXPECT_GT(std::sqrt(raw_squared_error / truth.size()), .1);
  EXPECT_LT(std::sqrt(corrected_squared_error / truth.size()), 1e-11);
}
}  // namespace

TEST(Tracker, DeskewRecoversStationaryWallDuringCircularTranslation) {
  expectDeskewedWall(Vec3(.8, -.15, .7), Vec3(.8, -.15, .7));
}
TEST(Tracker, DeskewAccountsForRotatingLidarLeverArm) {
  expectDeskewedWall(Vec3(0, 0, 1.3), Vec3(0, 0, 1.3));
}
TEST(Tracker, DeskewIntegratesOdometryChangesBetweenBeams) {
  expectDeskewedWall(Vec3(.8, -.15, .7), Vec3(-.2, .3, 1.2));
}
TEST(Tracker, DeskewRejectsMissingOrStaleOdometryWithoutPartialOutput) {
  Config c; c.odom_timeout = .06;
  Tracker tracker(c);
  Scan scan; scan.stamp = 1; scan.time_increment = .05; scan.ranges = {1, 2, 3};
  Points points(1); std::string reason;
  EXPECT_FALSE(tracker.deskewScan(scan, points, reason));
  EXPECT_EQ(reason, "deskew lacks odometry at scan start");
  EXPECT_TRUE(points.empty());
  ASSERT_TRUE(tracker.addOdometry({1, Vec3(.8, .1, .7)}));
  EXPECT_FALSE(tracker.deskewScan(scan, points, reason));
  EXPECT_EQ(reason, "deskew odometry stale");
  EXPECT_TRUE(points.empty());
  scan.stamp = 1.1;
  EXPECT_FALSE(tracker.deskewScan(scan, points, reason));
  EXPECT_EQ(reason, "deskew lacks odometry at scan start");
}
TEST(Tracker, InstantaneousScanUsesOnlyExtrinsicsWithoutOdometry) {
  Config c; c.lidar_in_base = SE2(.3, -.1, .4);
  Tracker tracker(c);
  Scan scan; scan.stamp = 1; scan.angle_min = -.2; scan.angle_increment = .2; scan.ranges = {1, 2, 3};
  Points points; std::string reason;
  ASSERT_TRUE(tracker.deskewScan(scan, points, reason)) << reason;
  ASSERT_EQ(points.size(), scan.ranges.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    const double angle = scan.angle_min + i * scan.angle_increment;
    EXPECT_LT((points[i].q - c.lidar_in_base * Vec2(scan.ranges[i] * std::cos(angle),
      scan.ranges[i] * std::sin(angle))).norm(), 1e-12);
  }
}
TEST(Tracker, DeskewDoesNotUseFutureCheckpointOdometryForOldScan) {
  Config c; c.max_events = 2;
  Tracker tracker(c);
  for (double stamp : {.2, .3, .4}) ASSERT_TRUE(tracker.addOdometry({stamp, Vec3(1, 0, .5)}));
  Scan scan; scan.stamp = .1; scan.time_increment = .01; scan.ranges = {1, 2, 3};
  Points points; std::string reason;
  EXPECT_FALSE(tracker.deskewScan(scan, points, reason));
  EXPECT_EQ(reason, "deskew lacks odometry at scan start");
  EXPECT_TRUE(points.empty());
}
TEST(Tracker, PublicDeskewRejectsInvalidScanMetadata) {
  Tracker tracker;
  Scan scan; scan.ranges = {1, 2, 3}; scan.time_increment = -.01;
  Points points(1); std::string reason;
  EXPECT_FALSE(tracker.deskewScan(scan, points, reason));
  EXPECT_EQ(reason, "invalid or oversized scan");
  EXPECT_TRUE(points.empty());
}
TEST(Tracker, MissingScansPropagateAndStaleOdometryStopsMean) {
  Config c; c.odom_timeout = .2;
  Tracker t(c); t.initialize(0, SE2(), Mat3::Identity() * .01);
  ASSERT_TRUE(t.addOdometry({0, Vec3(1, 0, 0)}));
  auto early = t.estimate(.1), late = t.estimate(.5);
  EXPECT_NEAR(early.robot_in_marker.p.x(), .1, 1e-9);
  EXPECT_NEAR(late.robot_in_marker.p.x(), .2, 1e-9);
  EXPECT_EQ(late.diagnostics.mode, "ODOMETRY_STALE");
  EXPECT_GT(late.covariance.trace(), early.covariance.trace());
}
TEST(Tracker, LateOdometryReplaysAndDuplicateDoesNotChangeState) {
  Tracker a, b; a.initialize(0, SE2(), Mat3::Identity()); b = a;
  a.addOdometry({0, Vec3(.2, 0, .1)});
  a.addOdometry({.1, Vec3(.5, 0, -.2)});
  a.addOdometry({.2, Vec3(.1, .1, .2)});
  b.addOdometry({0, Vec3(.2, 0, .1)});
  b.addOdometry({.2, Vec3(.1, .1, .2)});
  EXPECT_TRUE(b.addOdometry({.1, Vec3(.5, 0, -.2)}));
  auto x = a.estimate(.25), y = b.estimate(.25);
  EXPECT_LT((x.robot_in_marker.inverse() * y.robot_in_marker).log().norm(), 1e-10);
  EXPECT_LT((x.covariance - y.covariance).norm(), 1e-10);
  EXPECT_FALSE(b.addOdometry({.1, Vec3(.5, 0, -.2)}));
  EXPECT_LT((y.covariance - b.estimate(.25).covariance).norm(), 1e-10);
}
TEST(Tracker, OldHistoryInvalidInputAndReset) {
  Config c; c.history_sec = .2; c.max_events = 10;
  Tracker t(c); t.initialize(0, SE2(), Mat3::Identity());
  for (int i = 0; i < 100; ++i) EXPECT_TRUE(t.addOdometry({i * .01, Vec3(.1, 0, 0)}));
  EXPECT_FALSE(t.addOdometry({.1, Vec3::Zero()}));
  EXPECT_FALSE(t.addOdometry({1, Vec3(NAN, 0, 0)}));
  Scan s; s.stamp = 1; s.ranges.resize(c.max_scan_points + 1, 1.);
  EXPECT_FALSE(t.addScan(s));
  t.reset(); EXPECT_FALSE(t.estimate(2).initialized);
}
TEST(Tracker, DuplicateAtPrunedCheckpointIsRejected) {
  Config c; c.max_events = 2;
  Tracker t(c); t.initialize(0, SE2(), Mat3::Identity());
  Scan s; s.angle_increment = .01; s.ranges.assign(10, INFINITY);
  for (double time : {.1, .2, .3}) { s.stamp = time; EXPECT_TRUE(t.addScan(s)); }
  s.stamp = .1;
  EXPECT_FALSE(t.addScan(s));
}
TEST(Filter, RecoveryRequiresConsistentRepeatedHypotheses) {
  Filter f; f.initialize(0, SE2(), Mat3::Identity());
  EXPECT_FALSE(f.recoveryReady(SE2(1, 0, 0)));
  EXPECT_FALSE(f.recoveryReady(SE2(1, 0, 0)));
  EXPECT_FALSE(f.recoveryReady(SE2(2, 0, 0)));
  f.predict(.1, Vec3(1, 0, 0));
  EXPECT_FALSE(f.recoveryReady(SE2(2.1, 0, 0)));
  EXPECT_TRUE(f.recoveryReady(SE2(2.1, 0, 0)));
}
TEST(Tracker, OdometryInsideSweptScanTriggersReplayEvenWhenAppended) {
  Tracker a, b; a.initialize(0, SE2(.7, 0, 3.14), Mat3::Identity() * .1); b = a;
  Odom first{0, Vec3::Zero()}, middle{.15, Vec3(.2, 0, 0)};
  Scan s; s.stamp = .1; s.time_increment = .1 / 1440.; s.angle_min = -2.35;
  s.angle_increment = 4.7 / 1440.; s.ranges.assign(1441, INFINITY);
  const SE2 marker(.7, 0, 3.14);
  Config c;
  for (std::size_t i = 0; i < s.ranges.size(); ++i) {
    const double angle = s.angle_min + i * s.angle_increment;
    const Vec2 ray(std::cos(angle), std::sin(angle));
    for (int arm = 0; arm < 2; ++arm) {
      Mat2 system; system.col(0) = ray; system.col(1) = -marker.rotation() * armDirection(c, arm);
      if (std::abs(system.determinant()) < 1e-8) continue;
      Vec2 hit = system.partialPivLu().solve(marker.p);
      if (hit[0] > .05 && hit[1] >= 0 && hit[1] <= c.arm_length) s.ranges[i] = std::min(s.ranges[i], hit[0]);
    }
  }
  a.addOdometry(first); a.addOdometry(middle); a.addScan(s);
  b.addOdometry(first); b.addScan(s); b.addOdometry(middle);
  EXPECT_GE(b.estimate(.2).diagnostics.replays, 1u);
  auto x = a.estimate(.2), y = b.estimate(.2);
  EXPECT_GT(y.diagnostics.updates, 0u);
  EXPECT_LT((x.robot_in_marker.inverse() * y.robot_in_marker).log().norm(), 1e-12);
  EXPECT_LT((x.covariance - y.covariance).norm(), 1e-12);
}
TEST(Tracker, DelayedUnfinishedSweepCannotExceedEventCapacity) {
  Config c; c.max_events = 2;
  Tracker t(c); t.initialize(0, SE2(), Mat3::Identity());
  t.addOdometry({.2, Vec3::Zero()}); t.addOdometry({.3, Vec3::Zero()});
  Scan s; s.stamp = .1; s.angle_increment = .1; s.time_increment = .2; s.ranges.assign(3, INFINITY);
  EXPECT_FALSE(t.addScan(s));
}
