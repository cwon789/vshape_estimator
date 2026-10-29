#pragma once
#include "filter.hpp"
#include <deque>

namespace vshape {
// Single owner, not thread safe. ROS uses SingleThreadedExecutor.
class Tracker {
 public:
  explicit Tracker(Config config = {});
  void reset();
  void initialize(double stamp, const SE2& robot_in_marker, const Mat3& covariance);
  bool addOdometry(const Odom& odometry);
  bool addScan(const Scan& scan);
  // Same points used by filtering, in base at scan.stamp (the first ray).
  // Read-only; failure clears points and sets reason, success clears reason.
  bool deskewScan(const Scan& scan, Points& points, std::string& reason) const;
  Estimate estimate(double stamp); // predicts output without modifying history posterior
  const Config& config() const { return config_; }
 private:
  struct Event { double stamp; bool is_scan; Odom odom; Scan scan; std::size_t sequence; };
  Config config_;
  Filter anchor_, filter_;
  std::deque<Event> events_;
  Odom anchor_odom_, active_odom_;
  bool anchor_has_odom_ = false, has_odom_ = false;
  std::size_t sequence_ = 0, replays_ = 0;
  double closed_through_ = -std::numeric_limits<double>::infinity();
  double newest_ = -std::numeric_limits<double>::infinity();
  void advance(Filter& f, Odom& active, bool has_odom, double stamp) const;
  bool insert(Event event);
  void process(Filter& f, Odom& active, bool& has_odom, const Event& event) const;
};
}  // namespace vshape
