#include "ros2_bot_estimator_lidar_vshape/tracker.hpp"
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>

namespace {
double number(std::istream& input) {
  std::string token; if (!(input >> token)) throw std::invalid_argument("missing number");
  std::size_t used = 0; double value = std::stod(token, &used);
  if (used != token.size()) throw std::invalid_argument("invalid number");
  return value;
}
void setting(vshape::Config& c, const std::string& name, double value) {
#define REAL(key) if (name == #key) { c.key = value; return; }
  REAL(arm_length) REAL(opening_angle_deg) REAL(range_sigma) REAL(angle_sigma) REAL(model_sigma)
  REAL(q_linear) REAL(q_angular) REAL(odom_timeout) REAL(unknown_linear) REAL(unknown_angular)
  REAL(association_distance) REAL(endpoint_margin) REAL(min_support) REAL(gate_sigma) REAL(huber_delta)
  REAL(history_sec) REAL(init_position_sigma) REAL(init_yaw_sigma) REAL(target_x) REAL(target_y)
  REAL(target_yaw) REAL(target_position_tolerance) REAL(target_yaw_tolerance)
#undef REAL
  if (!std::isfinite(value)) throw std::invalid_argument("nonfinite setting");
  if (name == "lidar_x") c.lidar_in_base.p.x() = value;
  else if (name == "lidar_y") c.lidar_in_base.p.y() = value;
  else if (name == "lidar_yaw") c.lidar_in_base.yaw = value;
  else {
    if (value < 0 || value > 10000000 || std::floor(value) != value) throw std::invalid_argument("invalid integer setting");
    if (name == "min_points") c.min_points = int(value);
    else if (name == "max_points_per_arm") c.max_points_per_arm = int(value);
    else if (name == "max_iterations") c.max_iterations = int(value);
    else if (name == "max_events") c.max_events = std::size_t(value);
    else if (name == "max_scan_points") c.max_scan_points = std::size_t(value);
    else throw std::invalid_argument("unknown configuration key: " + name);
  }
}
void jsonNumber(double value) { if (std::isfinite(value)) std::cout << value; else std::cout << "null"; }
void output(const vshape::Estimate& e) {
  const auto& d = e.diagnostics;
  std::cout << "{\"stamp\":" << e.stamp << ",\"initialized\":" << (e.initialized ? "true" : "false") << ",\"pose\":[";
  for (int i = 0; i < 3; ++i) { if (i) std::cout << ','; jsonNumber(e.robot_in_marker.vector()[i]); }
  std::cout << "],\"covariance\":[";
  for (int i = 0; i < 9; ++i) { if (i) std::cout << ','; jsonNumber(e.covariance(i / 3, i % 3)); }
  std::cout << "],\"mode\":" << std::quoted(d.mode) << ",\"reason\":" << std::quoted(d.reason) << ",\"last_measurement\":";
  jsonNumber(d.last_measurement); std::cout << ",\"odom_age\":"; jsonNumber(d.odom_age);
  std::cout << ",\"nis\":"; jsonNumber(d.nis);
  std::cout << ",\"dof\":" << d.dof << ",\"points\":[" << d.points[0] << ',' << d.points[1] << "],\"eigenvalues\":[";
  for (int i = 0; i < 3; ++i) { if (i) std::cout << ','; jsonNumber(d.information_eigenvalues[i]); }
  std::cout << "],\"updates\":" << d.updates << ",\"rejected\":" << d.rejected << ",\"replays\":" << d.replays;
  std::cout << ",\"update_ms\":"; jsonNumber(d.update_ms);
  std::cout << ",\"processing_ms\":"; jsonNumber(d.processing_ms);
  std::cout << ",\"candidate_score\":"; jsonNumber(d.candidate_score);
  std::cout << ",\"support_fraction\":"; jsonNumber(d.support_fraction);
  std::cout << ",\"normal_condition\":"; jsonNumber(d.normal_condition);
  std::cout << "}\n";
}
}
int main() {
  vshape::Config config; std::unique_ptr<vshape::Tracker> tracker;
  std::string line; std::size_t line_number = 0;
  std::cout << std::setprecision(17);
  try {
    while (std::getline(std::cin, line)) {
      ++line_number;
      if (line.empty() || line[0] == '#') continue;
      std::istringstream input(line); std::string command; input >> command;
      if (command == "C") {
        if (tracker) throw std::invalid_argument("C must precede all events");
        std::string key; input >> key; setting(config, key, number(input)); continue;
      }
      if (!tracker) tracker = std::make_unique<vshape::Tracker>(config);
      if (command == "I") {
        const double t = number(input), x = number(input), y = number(input), yaw = number(input);
        const double pos_sigma = number(input), yaw_sigma = number(input);
        if (!(pos_sigma > 0) || !(yaw_sigma > 0)) throw std::invalid_argument("initial sigma must be positive");
        vshape::Mat3 covariance = vshape::Mat3::Zero();
        covariance.diagonal() << pos_sigma * pos_sigma, pos_sigma * pos_sigma, yaw_sigma * yaw_sigma;
        tracker->initialize(t, vshape::SE2(x, y, yaw), covariance);
      } else if (command == "O") {
        vshape::Odom o; o.stamp = number(input); for (int i = 0; i < 3; ++i) o.twist[i] = number(input);
        tracker->addOdometry(o);
      } else if (command == "S") {
        vshape::Scan scan; scan.stamp = number(input); scan.angle_min = number(input);
        scan.angle_increment = number(input); scan.time_increment = number(input);
        const double count = number(input);
        if (!std::isfinite(count) || count < 3 || count > config.max_scan_points || count != std::floor(count))
          throw std::invalid_argument("invalid scan count");
        scan.ranges.reserve(std::size_t(count));
        for (int i = 0; i < int(count); ++i) scan.ranges.push_back(number(input));
        tracker->addScan(scan);
      } else if (command == "P") output(tracker->estimate(number(input)));
      else if (command == "R") tracker->reset();
      else throw std::invalid_argument("unknown command: " + command);
    }
  } catch (const std::exception& e) {
    std::cerr << "Replay input line " << line_number << ": " << e.what() << '\n'; return 2;
  }
  return 0;
}
