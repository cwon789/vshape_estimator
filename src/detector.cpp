#include "ros2_bot_estimator_lidar_vshape/detector.hpp"
#include <Eigen/Eigenvalues>
#include <Eigen/LU>
#include <algorithm>
#include <random>

namespace vshape {
namespace {
struct Line { Vec2 mean, direction; std::vector<int> indices; };
Line fit(const Points& points, const std::vector<int>& indices) {
  Line line; line.indices = indices; line.mean = Vec2::Zero();
  for (int i : indices) line.mean += points[i].q;
  line.mean /= double(indices.size()); Mat2 scatter = Mat2::Zero();
  for (int i : indices) { Vec2 d = points[i].q - line.mean; scatter += d * d.transpose(); }
  line.direction = Eigen::SelfAdjointEigenSolver<Mat2>(scatter).eigenvectors().col(1);
  return line;
}
}
std::vector<Observation> associate(const Points& points, const SE2& x, const Config& c) {
  std::array<std::vector<std::pair<double, int>>, 2> groups;
  for (std::size_t i = 0; i < points.size(); ++i) {
    if (!points[i].q.allFinite()) continue;
    const Vec2 q = x * points[i].q;
    double distance[2], along[2];
    for (int a = 0; a < 2; ++a) {
      const Vec2 d = armDirection(c, a); along[a] = d.dot(q);
      distance[a] = std::abs(perpendicular(d).dot(q));
      if (along[a] < .02 || along[a] > c.arm_length + c.endpoint_margin) distance[a] = 1e9;
    }
    int a = distance[0] < distance[1] ? 0 : 1;
    if (distance[a] > c.association_distance || std::abs(distance[0] - distance[1]) < .005) continue;
    groups[a].push_back({along[a], int(i)});
  }
  std::vector<Observation> out;
  for (int a = 0; a < 2; ++a) {
    auto& g = groups[a]; std::sort(g.begin(), g.end());
    if (g.size() < std::size_t(c.min_points) || g.back().first - g.front().first < c.min_support) continue;
    const int count = std::min(int(g.size()), c.max_points_per_arm);
    for (int k = 0; k < count; ++k) {
      const std::size_t i = std::size_t(k) * (g.size() - 1) / std::size_t(count - 1);
      out.push_back({points[g[i].second], a});
    }
  }
  return out;
}
Detection detectV(const Points& points, const Config& c) {
  std::vector<int> remaining;
  const Vec2 target(c.target_x, c.target_y);
  for (std::size_t i = 0; i < points.size(); ++i)
    if (points[i].q.allFinite() && (points[i].q - target).norm() < c.target_position_tolerance + c.arm_length)
      remaining.push_back(int(i));
  std::vector<Line> lines;
  std::mt19937 rng(42);
  const double threshold = std::max(.006, 2.5 * c.range_sigma);
  for (int extraction = 0; extraction < 6 && remaining.size() >= std::size_t(c.min_points); ++extraction) {
    std::vector<int> best;
    for (int trial = 0; trial < 240; ++trial) {
      const int i = remaining[rng() % remaining.size()], j = remaining[rng() % remaining.size()];
      Vec2 d = points[j].q - points[i].q;
      if (d.norm() < c.min_support) continue;
      Vec2 n = perpendicular(d.normalized()); std::vector<int> inliers;
      for (int k : remaining) if (std::abs(n.dot(points[k].q - points[i].q)) < threshold) inliers.push_back(k);
      if (inliers.size() > best.size()) best = std::move(inliers);
    }
    if (best.size() < std::size_t(c.min_points)) break;
    Line line = fit(points, best);
    best.clear();
    for (int k : remaining)
      if (std::abs(perpendicular(line.direction).dot(points[k].q - line.mean)) < threshold) best.push_back(k);
    if (best.size() < std::size_t(c.min_points)) break;
    line = fit(points, best); lines.push_back(line);
    remaining.erase(std::remove_if(remaining.begin(), remaining.end(), [&](int k) {
      return std::abs(perpendicular(line.direction).dot(points[k].q - line.mean)) < threshold;
    }), remaining.end());
  }
  Detection result; double runner_up = 0;
  for (std::size_t i = 0; i < lines.size(); ++i) for (std::size_t j = i + 1; j < lines.size(); ++j) {
    Line a = lines[i], b = lines[j];
    // RANSAC strips overlap near the apex; jointly reassign before estimating
    // the intersection, so one arm's points cannot bias the other arm's fit.
    for (int refinement = 0; refinement < 5; ++refinement) {
      Mat2 normals0;
      normals0.row(0) = perpendicular(a.direction).transpose();
      normals0.row(1) = perpendicular(b.direction).transpose();
      if (std::abs(normals0.determinant()) < .15) break;
      Vec2 origin = normals0.partialPivLu().solve(Vec2(normals0.row(0).dot(a.mean), normals0.row(1).dot(b.mean)));
      Vec2 dir_a = a.direction, dir_b = b.direction;
      if (dir_a.dot(a.mean - origin) < 0) dir_a = -dir_a;
      if (dir_b.dot(b.mean - origin) < 0) dir_b = -dir_b;
      std::vector<int> ia, ib;
      for (std::size_t k = 0; k < points.size(); ++k) {
        if (!points[k].q.allFinite() || (points[k].q - target).norm() > c.target_position_tolerance + c.arm_length) continue;
        double da = std::abs(perpendicular(a.direction).dot(points[k].q - a.mean));
        double db = std::abs(perpendicular(b.direction).dot(points[k].q - b.mean));
        const double sa = dir_a.dot(points[k].q - origin), sb = dir_b.dot(points[k].q - origin);
        if (sa < -c.endpoint_margin || sa > c.arm_length + c.endpoint_margin) da = 1e9;
        if (sb < -c.endpoint_margin || sb > c.arm_length + c.endpoint_margin) db = 1e9;
        if (std::min(da, db) > threshold) continue;
        (da < db ? ia : ib).push_back(int(k));
      }
      if (ia.size() < std::size_t(c.min_points) || ib.size() < std::size_t(c.min_points)) break;
      a = fit(points, ia); b = fit(points, ib);
    }
    Mat2 normals; normals.row(0) = perpendicular(a.direction).transpose(); normals.row(1) = perpendicular(b.direction).transpose();
    if (std::abs(normals.determinant()) < .15) continue;
    Vec2 rhs(normals.row(0).dot(a.mean), normals.row(1).dot(b.mean));
    Vec2 apex = normals.partialPivLu().solve(rhs);
    if ((apex - target).norm() > c.target_position_tolerance) continue;
    Vec2 da = a.direction, db = b.direction;
    if (da.dot(a.mean - apex) < 0) da = -da;
    if (db.dot(b.mean - apex) < 0) db = -db;
    const double angle = std::acos(std::clamp(da.dot(db), -1., 1.));
    if (std::abs(angle - c.opening_angle_deg * 3.141592653589793 / 180.) > .14) continue;
    Vec2 bisector = da + db;
    const double yaw = std::atan2(bisector.y(), bisector.x());
    if (std::abs(wrap(yaw - c.target_yaw)) > c.target_yaw_tolerance) continue;
    bool valid = true;
    for (const auto* l : {&a, &b}) {
      Vec2 d = l->direction; if (d.dot(l->mean - apex) < 0) d = -d;
      double low = 1e9, high = -1e9;
      for (int k : l->indices) { const double s = d.dot(points[k].q - apex); low = std::min(low, s); high = std::max(high, s); }
      if (low < -c.endpoint_margin - threshold || high > c.arm_length + c.endpoint_margin + threshold || high - low < c.min_support) valid = false;
    }
    // Do not manufacture a short marker by cropping a continuous long wall.
    // Inspect the connected projected support, retaining evidence just beyond
    // either finite endpoint while ignoring remote collinear objects after a gap.
    for (const Vec2& d : {da, db}) {
      std::vector<double> support;
      for (const auto& point : points) {
        const Vec2 q = point.q - apex;
        if (q.allFinite() && std::abs(perpendicular(d).dot(q)) < threshold)
          support.push_back(d.dot(q));
      }
      std::sort(support.begin(), support.end());
      std::size_t begin = 0;
      while (begin < support.size()) {
        std::size_t end = begin;
        while (end + 1 < support.size() && support[end + 1] - support[end] < .08) ++end;
        if (support[begin] < c.arm_length && support[end] > .03 &&
            (support[begin] < -c.endpoint_margin - threshold ||
             support[end] > c.arm_length + c.endpoint_margin + threshold)) valid = false;
        begin = end + 1;
      }
    }
    if (!valid) continue;
    const double score = double(a.indices.size() + b.indices.size()) /
      (1 + (apex - target).norm() + std::abs(wrap(yaw - c.target_yaw)));
    if (score > result.score) { runner_up = result.score; result = {true, SE2(apex.x(), apex.y(), yaw), score}; }
    else runner_up = std::max(runner_up, score);
  }
  if (runner_up > result.score * .85) result.valid = false;
  return result;
}
}  // namespace vshape
