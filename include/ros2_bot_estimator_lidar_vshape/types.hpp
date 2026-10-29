#pragma once
#include "se2.hpp"
#include <array>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace vshape {
struct Config {
  double arm_length = 0.5, opening_angle_deg = 70;
  double range_sigma = 0.01, angle_sigma = 0.001, model_sigma = 0.002;
  double q_linear = 0.02, q_angular = 0.01;
  double odom_timeout = 0.2, unknown_linear = 0.5, unknown_angular = 0.5;
  double association_distance = 0.08, endpoint_margin = 0.025, min_support = 0.1;
  double gate_sigma = 5.0, huber_delta = 2.5;
  double history_sec = 1.0, init_position_sigma = 0.05, init_yaw_sigma = 0.1;
  double target_x = 2.0, target_y = 0.0, target_yaw = 3.141592653589793;
  double target_position_tolerance = 0.8, target_yaw_tolerance = 0.7;
  int min_points = 5, max_points_per_arm = 24, max_iterations = 10;
  std::size_t max_events = 5000, max_scan_points = 20000;
  SE2 lidar_in_base;
  void validate() const;
};
struct Scan {
  double stamp = 0, angle_min = 0, angle_increment = 0, time_increment = 0;
  double range_min = 0.05, range_max = 30.0;
  std::vector<double> ranges;
};
struct Odom { double stamp = 0; Vec3 twist = Vec3::Zero(); };
struct Point {
  Vec2 q = Vec2::Zero();                 // point in base at scan reference time
  Mat2 covariance = Mat2::Identity() * 1e-4;
  int beam = 0;
};
struct Observation { Point point; int arm = 0; };
using Points = std::vector<Point>;
struct Diagnostics {
  std::string mode = "UNINITIALIZED", reason = "waiting for V marker";
  double last_measurement = -std::numeric_limits<double>::infinity();
  double odom_age = std::numeric_limits<double>::infinity();
  double nis = std::numeric_limits<double>::quiet_NaN();
  int dof = 0;
  std::array<int, 2> points{{0, 0}};
  Vec3 information_eigenvalues = Vec3::Zero();
  std::size_t updates = 0, rejected = 0, replays = 0;
  double update_ms = 0, processing_ms = 0;
  double candidate_score = 0, support_fraction = 0, normal_condition = 0;
};
struct Estimate {
  bool initialized = false;
  double stamp = 0;
  SE2 robot_in_marker;
  Mat3 covariance = Mat3::Identity();   // right body tangent, NOT ROS pose coordinates
  Diagnostics diagnostics;
};
inline Vec2 armDirection(const Config& c, int arm) {
  const double h = c.opening_angle_deg * 3.141592653589793 / 360.;
  return {std::cos(h), (arm == 0 ? -1.0 : 1.0) * std::sin(h)};
}
}  // namespace vshape
