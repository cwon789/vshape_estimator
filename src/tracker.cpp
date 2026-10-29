#include "ros2_bot_estimator_lidar_vshape/tracker.hpp"
#include "ros2_bot_estimator_lidar_vshape/detector.hpp"
#include <algorithm>
#include <chrono>
#include <stdexcept>

namespace vshape {
namespace {
bool validScan(const Scan& scan, const Config& config) {
  return std::isfinite(scan.stamp) && scan.stamp >= 0 && std::isfinite(scan.angle_min) &&
    std::isfinite(scan.angle_increment) && std::isfinite(scan.time_increment) &&
    scan.time_increment >= 0 && scan.ranges.size() >= 3 && scan.ranges.size() <= config.max_scan_points &&
    std::isfinite(scan.range_min) && std::isfinite(scan.range_max) && scan.range_min >= 0 &&
    scan.range_max > scan.range_min && scan.time_increment * double(scan.ranges.size() - 1) <= .5;
}
}  // namespace
Tracker::Tracker(Config config) : config_(std::move(config)), anchor_(config_), filter_(config_) {}
void Tracker::reset() {
  anchor_ = Filter(config_); filter_ = anchor_; events_.clear();
  anchor_odom_ = Odom{}; active_odom_ = Odom{};
  anchor_has_odom_ = false; has_odom_ = false; sequence_ = replays_ = 0;
  newest_ = closed_through_ = -std::numeric_limits<double>::infinity();
}
void Tracker::initialize(double stamp, const SE2& x, const Mat3& p) {
  reset(); anchor_.initialize(stamp, x, p); filter_ = anchor_; newest_ = stamp;
}
void Tracker::advance(Filter& f, Odom& active, bool has_odom, double stamp) const {
  const double now = f.state().stamp;
  const std::string previous_mode = f.state().diagnostics.mode;
  if (stamp <= now + 1e-10) return;
  const double valid_until = has_odom ? active.stamp + config_.odom_timeout : now;
  if (has_odom && valid_until > now) f.predict(std::min(stamp, valid_until), active.twist, true);
  if (stamp > f.state().stamp + 1e-10) f.predict(stamp, Vec3::Zero(), false);
  if (has_odom && stamp <= valid_until && stamp - f.state().diagnostics.last_measurement <= .15 &&
      (previous_mode == "TRACKING" || previous_mode == "PARTIAL"))
    f.mutableState().diagnostics.mode = previous_mode;
  f.mutableState().diagnostics.odom_age = has_odom ? std::max(0., stamp - active.stamp) :
    std::numeric_limits<double>::infinity();
}

bool Tracker::deskewScan(const Scan& scan, Points& points, std::string& reason) const {
  points.clear(); reason.clear();
  if (!validScan(scan, config_)) { reason = "invalid or oversized scan"; return false; }
  // Relative base motion from first ray to each beam, independent of target state.
  SE2 relative; double cursor = scan.stamp, travel = 0;
  Odom active = anchor_odom_; bool have = anchor_has_odom_;
  std::vector<Odom> odoms;
  for (const auto& event : events_) if (!event.is_scan) {
    if (event.stamp <= scan.stamp + 1e-10) { active = event.odom; have = true; }
    else odoms.push_back(event.odom);
  }
  std::size_t next_odom = 0;
  if (scan.time_increment > 0 && (!have || active.stamp > scan.stamp + 1e-10 ||
      active.stamp + config_.odom_timeout < scan.stamp)) {
    reason = "deskew lacks odometry at scan start"; return false;
  }
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const double time = scan.stamp + scan.time_increment * double(i);
    while (cursor < time - 1e-10) {
      const double boundary = next_odom < odoms.size() ? std::min(time, odoms[next_odom].stamp) : time;
      if (!have || active.stamp + config_.odom_timeout < boundary - 1e-9) {
        points.clear(); reason = "deskew odometry stale"; return false;
      }
      travel += active.twist.head<2>().norm() * (boundary - cursor);
      relative = relative * SE2::exp(active.twist * (boundary - cursor)); cursor = boundary;
      if (next_odom < odoms.size() && cursor >= odoms[next_odom].stamp - 1e-10) active = odoms[next_odom++];
    }
    const double range = scan.ranges[i];
    if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max) continue;
    const double angle = scan.angle_min + scan.angle_increment * double(i);
    const Vec2 ray(std::cos(angle), std::sin(angle));
    const SE2 transform = relative * config_.lidar_in_base;
    const Vec2 body_point = config_.lidar_in_base * (range * ray);
    Point p; p.q = relative * body_point; p.beam = int(i);
    Mat2 polar; polar.col(0) = ray; polar.col(1) = range * perpendicular(ray);
    Mat2 noise = Mat2::Zero(); noise(0, 0) = config_.range_sigma * config_.range_sigma;
    noise(1, 1) = config_.angle_sigma * config_.angle_sigma;
    p.covariance = transform.rotation() * polar * noise * polar.transpose() * transform.rotation().transpose();
    p.covariance.diagonal().array() += 1e-10;
    // Cauchy bound for up to 2*max_points_per_arm correlated retained returns:
    // C_joint <= N*diag(C_i). This avoids averaging shared deskew noise away.
    // Cross-correlation with subsequent process noise is still approximate;
    // formal covariance consistency is tested separately on instantaneous scans.
    if (scan.time_increment > 0) {
      const double dt = time - scan.stamp;
      p.covariance.diagonal().array() += (2 * config_.max_points_per_arm) * dt * (config_.q_linear * config_.q_linear +
        std::pow(body_point.norm() + travel, 2) * config_.q_angular * config_.q_angular);
    }
    points.push_back(p);
  }
  return true;
}
void Tracker::process(Filter& f, Odom& active, bool& have, const Event& event) const {
  advance(f, active, have, event.stamp);
  if (!event.is_scan) { active = event.odom; have = true; return; }
  Points points; std::string reason;
  if (!deskewScan(event.scan, points, reason)) { f.reject(reason); return; }
  if (!f.state().initialized) {
    auto candidate = detectV(points, config_);
    if (!candidate.valid) { f.reject("no unambiguous V in target ROI"); return; }
    Mat3 p = Mat3::Zero();
    p.diagonal() << config_.init_position_sigma * config_.init_position_sigma,
      config_.init_position_sigma * config_.init_position_sigma,
      config_.init_yaw_sigma * config_.init_yaw_sigma;
    // Init pose is inverse of detected marker-in-base; transform conservative
    // marker pose uncertainty into the robot pose right tangent.
    const Mat3 a = candidate.marker_in_base.adjoint();
    f.initialize(event.stamp, candidate.marker_in_base.inverse(), a * p * a.transpose());
    f.mutableState().diagnostics.candidate_score = candidate.score;
    return; // Never reuse the initialization scan for a second update.
  }
  SE2 association_pose = f.state().robot_in_marker;
  if (event.stamp - f.state().diagnostics.last_measurement > .5) {
    Config search = config_;
    const SE2 marker = association_pose.inverse();
    search.target_x = marker.p.x(); search.target_y = marker.p.y(); search.target_yaw = marker.yaw;
    const double sigma = std::sqrt(f.state().covariance.topLeftCorner<2, 2>().trace());
    search.target_position_tolerance = std::min(config_.target_position_tolerance, std::max(.15, 3 * sigma));
    auto candidate = detectV(points, search);
    if (!candidate.valid) { f.reject("recovery: full V not found"); return; }
    if (!f.recoveryReady(candidate.marker_in_base.inverse())) { f.reject("recovery: awaiting confirmation"); return; }
    association_pose = candidate.marker_in_base.inverse();
    f.mutableState().diagnostics.candidate_score = candidate.score;
  }
  auto obs = associate(points, association_pose, config_);
  f.update(obs);
}
bool Tracker::insert(Event event) {
  if (event.stamp <= closed_through_ + 1e-9) { filter_.reject("input at/before closed checkpoint"); return false; }
  if (std::isfinite(newest_) && (event.stamp < newest_ - config_.history_sec - 1e-9 ||
      event.stamp < anchor_.state().stamp - 1e-9)) { filter_.reject("input older than retained history"); return false; }
  for (const auto& e : events_)
    if (e.is_scan == event.is_scan && std::abs(e.stamp - event.stamp) < 1e-9) {
      filter_.reject("duplicate input"); return false;
    }
  auto support_end = [](const Event& e) {
    return e.is_scan ? e.scan.stamp + e.scan.time_increment * double(e.scan.ranges.size() - 1) : e.stamp;
  };
  if (events_.size() >= config_.max_events && !events_.empty() &&
      (event.stamp < events_.front().stamp ? support_end(event) : support_end(events_.front())) >
        std::max(newest_, event.stamp) + 1e-9) {
    filter_.reject("history capacity: unfinished sweep"); return false;
  }
  bool affects_sweep = false;
  if (!event.is_scan) for (const auto& e : events_)
    if (e.is_scan && event.stamp >= e.stamp - 1e-9 && event.stamp <= support_end(e) + 1e-9)
      affects_sweep = true;
  const bool delayed = affects_sweep || event.stamp < newest_ - 1e-9 ||
    (!event.is_scan && !events_.empty() && events_.back().is_scan && std::abs(event.stamp - newest_) < 1e-9);
  event.sequence = sequence_++;
  auto order = [](const Event& a, const Event& b) {
    if (a.stamp != b.stamp) return a.stamp < b.stamp;
    if (a.is_scan != b.is_scan) return !a.is_scan;
    return a.sequence < b.sequence;
  };
  auto it = std::upper_bound(events_.begin(), events_.end(), event, order);
  const bool appended = it == events_.end();
  events_.insert(it, event);
  newest_ = std::max(newest_, event.stamp);
  if (appended && !delayed) process(filter_, active_odom_, has_odom_, event);
  else {
    ++replays_; filter_ = anchor_; active_odom_ = anchor_odom_; has_odom_ = anchor_has_odom_;
    for (const auto& e : events_) process(filter_, active_odom_, has_odom_, e);
  }
  while (!events_.empty() && support_end(events_.front()) <= newest_ + 1e-9 &&
         (support_end(events_.front()) < newest_ - config_.history_sec - 1e-9 ||
          events_.size() > config_.max_events)) {
    process(anchor_, anchor_odom_, anchor_has_odom_, events_.front());
    closed_through_ = std::max(closed_through_, support_end(events_.front())); events_.pop_front();
  }
  filter_.mutableState().diagnostics.replays = replays_;
  return true;
}
bool Tracker::addOdometry(const Odom& odom) {
  if (!std::isfinite(odom.stamp) || odom.stamp < 0 || !odom.twist.allFinite() ||
      odom.twist.head<2>().norm() > 10 || std::abs(odom.twist.z()) > 10) {
    filter_.reject("invalid odometry"); return false;
  }
  Event event{}; event.stamp = odom.stamp; event.odom = odom; event.is_scan = false;
  return insert(std::move(event));
}
bool Tracker::addScan(const Scan& scan) {
  const auto started = std::chrono::steady_clock::now();
  if (!validScan(scan, config_)) {
    filter_.reject("invalid or oversized scan"); return false;
  }
  Event event{}; event.stamp = scan.stamp; event.scan = scan; event.is_scan = true;
  const bool accepted = insert(std::move(event));
  filter_.mutableState().diagnostics.processing_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - started).count();
  return accepted;
}
Estimate Tracker::estimate(double stamp) {
  if (!std::isfinite(stamp) || stamp < filter_.state().stamp - 1e-9)
    throw std::invalid_argument("output time precedes posterior");
  Filter output = filter_; Odom active = active_odom_;
  const auto previous_mode = output.state().diagnostics.mode;
  advance(output, active, has_odom_, stamp);
  auto& d = output.mutableState().diagnostics;
  d.odom_age = has_odom_ ? std::max(0., stamp - active.stamp) : std::numeric_limits<double>::infinity();
  if (output.state().initialized) {
    if (d.odom_age > config_.odom_timeout) d.mode = "ODOMETRY_STALE";
    else if (stamp - d.last_measurement <= .15 && (previous_mode == "TRACKING" || previous_mode == "PARTIAL")) d.mode = previous_mode;
    else d.mode = "PROPAGATION_ONLY";
  }
  return output.state();
}
}  // namespace vshape
