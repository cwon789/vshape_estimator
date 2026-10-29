#include "ros2_bot_estimator_lidar_vshape/filter.hpp"
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <boost/math/distributions/chi_squared.hpp>
#include <algorithm>
#include <chrono>
#include <stdexcept>

namespace vshape {
void Config::validate() const {
  const double positives[] = {arm_length, opening_angle_deg, range_sigma, q_linear,
    q_angular, odom_timeout, unknown_linear, unknown_angular, association_distance,
    min_support, gate_sigma, history_sec, init_position_sigma, init_yaw_sigma,
    target_position_tolerance, target_yaw_tolerance};
  for (double v : positives)
    if (!std::isfinite(v) || v <= 0) throw std::invalid_argument("configuration must be finite and positive");
  const double nonnegative[] = {angle_sigma, model_sigma, endpoint_margin, huber_delta};
  for (double v : nonnegative)
    if (!std::isfinite(v) || v < 0) throw std::invalid_argument("configuration must be finite and nonnegative");
  if (opening_angle_deg >= 170 || opening_angle_deg < 10 || min_support >= arm_length ||
      min_points < 3 || max_points_per_arm < min_points || max_points_per_arm > 256 ||
      max_iterations < 1 || max_iterations > 50 || max_events < 2 || max_scan_points < 3 ||
      !lidar_in_base.finite() || !std::isfinite(target_x) || !std::isfinite(target_y) ||
      !std::isfinite(target_yaw)) throw std::invalid_argument("invalid geometry, bounds or target pose");
}
Filter::Filter(Config config) : config_(std::move(config)) { config_.validate(); }
void Filter::initialize(double stamp, const SE2& x, const Mat3& p) {
  if (!std::isfinite(stamp) || !x.finite() || !p.allFinite() ||
      (p - p.transpose()).norm() > 1e-9 || Eigen::LLT<Mat3>(p).info() != Eigen::Success)
    throw std::invalid_argument("invalid initialization pose/covariance/time");
  recovery_count_ = 0;
  state_ = Estimate{}; state_.initialized = true; state_.stamp = stamp;
  state_.robot_in_marker = x; state_.covariance = p;
  state_.diagnostics.mode = "TRACKING"; state_.diagnostics.reason = "initialized";
  state_.diagnostics.last_measurement = stamp;
}
void Filter::predict(double stamp, const Vec3& twist, bool fresh) {
  if (!std::isfinite(stamp) || !twist.allFinite() || stamp < state_.stamp - 1e-9)
    throw std::invalid_argument("invalid propagation time/twist");
  double remaining = stamp - state_.stamp;
  if (!state_.initialized) { state_.stamp = stamp; return; }
  if (remaining > 1e-9) {
    if (!fresh) {
      // Unknown motion during a gap: conservative white twist model, mean held.
      state_.covariance.diagonal() += Vec3(config_.unknown_linear * config_.unknown_linear,
        config_.unknown_linear * config_.unknown_linear,
        config_.unknown_angular * config_.unknown_angular) * remaining;
    } else {
      Mat3 qc = Mat3::Zero(); qc.diagonal() << config_.q_linear * config_.q_linear,
        config_.q_linear * config_.q_linear, config_.q_angular * config_.q_angular;
      while (remaining > 1e-10) {
        const double dt = std::min(remaining, .02);
        Mat3 phi = SE2::exp(-twist * dt).adjoint(), qd = Mat3::Zero();
        // Three-point Gauss-Legendre integration of continuous body-twist density.
        const double nodes[] = {.1127016653792583, .5, .8872983346207417};
        const double weights[] = {5./18., 4./9., 5./18.};
        for (int j = 0; j < 3; ++j) {
          Mat3 a = SE2::exp(-twist * (dt * nodes[j])).adjoint();
          qd += weights[j] * dt * a * qc * a.transpose();
        }
        state_.covariance = phi * state_.covariance * phi.transpose() + qd;
        state_.robot_in_marker = state_.robot_in_marker * SE2::exp(twist * dt);
        if (recovery_count_ > 0) recovery_pose_ = recovery_pose_ * SE2::exp(twist * dt);
        remaining -= dt;
      }
    }
    state_.covariance = (.5 * (state_.covariance + state_.covariance.transpose())).eval();
    state_.diagnostics.mode = fresh ? "PROPAGATION_ONLY" : "ODOMETRY_STALE";
  }
  state_.stamp = stamp;
}
void Filter::reject(const std::string& reason) {
  state_.diagnostics.reason = reason; ++state_.diagnostics.rejected;
}

bool Filter::recoveryReady(const SE2& hypothesis) {
  const Vec3 error = (recovery_pose_.inverse() * hypothesis).log();
  if (recovery_count_ > 0 && error.head<2>().norm() < .08 && std::abs(error.z()) < .1)
    ++recovery_count_;
  else recovery_count_ = 1;
  recovery_pose_ = hypothesis;
  return recovery_count_ >= 3;
}

bool Filter::update(const std::vector<Observation>& measurements) {
  const auto start = std::chrono::steady_clock::now();
  auto& diag = state_.diagnostics;
  diag.nis = std::numeric_limits<double>::quiet_NaN(); diag.dof = 0;
  diag.points = {{0, 0}}; diag.information_eigenvalues.setZero();
  if (!state_.initialized || measurements.empty()) { reject("no supported arm"); return false; }
  const int m = static_cast<int>(measurements.size());
  if (m > 2 * config_.max_points_per_arm) { reject("too many measurements"); return false; }
  for (const auto& obs : measurements) {
    if (obs.arm < 0 || obs.arm > 1 || !obs.point.q.allFinite() ||
        !obs.point.covariance.allFinite() ||
        (obs.point.covariance - obs.point.covariance.transpose()).norm() > 1e-9 ||
        Eigen::LLT<Mat2>(obs.point.covariance).info() != Eigen::Success) {
      reject("invalid point covariance/arm"); return false;
    }
    ++diag.points[obs.arm];
  }
  const SE2 prior = state_.robot_in_marker;
  const Mat3 p0 = state_.covariance;
  Eigen::LDLT<Mat3> psolver(p0);
  if (psolver.info() != Eigen::Success || !psolver.isPositive()) { reject("invalid prior covariance"); return false; }
  const Mat3 precision = psolver.solve(Mat3::Identity());
  Eigen::MatrixXd r = Eigen::MatrixXd::Zero(m, m), h0(m, 3);
  Eigen::VectorXd residual0(m);
  for (int i = 0; i < m; ++i) {
    const auto& obs = measurements[i]; const Vec2 n = perpendicular(armDirection(config_, obs.arm));
    Eigen::RowVector2d a = n.transpose() * prior.rotation();
    r(i, i) = std::max(1e-10, (a * obs.point.covariance * a.transpose())(0, 0));
    for (int j = 0; j < m; ++j)
      if (measurements[j].arm == obs.arm) r(i, j) += config_.model_sigma * config_.model_sigma;
    h0.row(i) = pointLineJacobian(prior, obs.point.q, n);
    residual0[i] = n.dot(prior * obs.point.q);
  }
  Eigen::LDLT<Eigen::MatrixXd> rsolver(r), ssolver(h0 * p0 * h0.transpose() + r);
  if (rsolver.info() != Eigen::Success || !rsolver.isPositive() ||
      ssolver.info() != Eigen::Success || !ssolver.isPositive()) { reject("innovation factorization failed"); return false; }
  diag.nis = residual0.dot(ssolver.solve(residual0)); diag.dof = m;
  const double probability = std::min(1. - 1e-12, .5 * (1. + std::erf(config_.gate_sigma / std::sqrt(2.))));
  const double gate = boost::math::quantile(boost::math::chi_squared(m), probability);
  if (!std::isfinite(diag.nis) || diag.nis > gate) { reject("innovation gate"); return false; }
  Vec3 delta = Vec3::Zero();
  bool converged = false;
  Eigen::MatrixXd final_a(m, 3), final_r = r;
  auto linearize = [&](const Vec3& d, Eigen::VectorXd& residual, Eigen::MatrixXd& a, Eigen::MatrixXd& reff) {
    const SE2 x = prior * SE2::exp(d); const Mat3 jr = rightJacobian(d);
    reff = r;
    for (int i = 0; i < m; ++i) {
      const auto& obs = measurements[i]; const Vec2 n = perpendicular(armDirection(config_, obs.arm));
      residual[i] = n.dot(x * obs.point.q);
      a.row(i) = pointLineJacobian(x, obs.point.q, n) * jr;
      if (config_.huber_delta > 0) {
        const double z = std::abs(residual[i]) / std::sqrt(r(i, i));
        // Inflate independent variance; retain common arm correlation once.
        if (z > config_.huber_delta) reff(i, i) += r(i, i) * (z / config_.huber_delta - 1);
      }
    }
  };
  for (int iteration = 0; iteration < config_.max_iterations; ++iteration) {
    Eigen::VectorXd residual(m); Eigen::MatrixXd a(m, 3), reff;
    linearize(delta, residual, a, reff);
    Eigen::LDLT<Eigen::MatrixXd> noise(reff);
    Mat3 normal = precision + a.transpose() * noise.solve(a);
    Eigen::SelfAdjointEigenSolver<Mat3> spectrum(normal);
    diag.normal_condition = spectrum.eigenvalues().maxCoeff() / spectrum.eigenvalues().minCoeff();
    if (!std::isfinite(diag.normal_condition) || spectrum.eigenvalues().minCoeff() <= 0 ||
        diag.normal_condition > 1e12) { reject("ill-conditioned normal system"); return false; }
    Eigen::LDLT<Mat3> normal_solver(normal);
    if (noise.info() != Eigen::Success || !noise.isPositive() ||
        normal_solver.info() != Eigen::Success || !normal_solver.isPositive()) { reject("update factorization failed"); return false; }
    Vec3 step = -normal_solver.solve(precision * delta + a.transpose() * noise.solve(residual));
    if (!step.allFinite()) { reject("nonfinite update"); return false; }
    const double old_cost = delta.dot(precision * delta) + residual.dot(noise.solve(residual));
    bool accepted = false;
    for (int backtrack = 0; backtrack < 10; ++backtrack) {
      const Vec3 next = delta + step;
      Eigen::VectorXd nr(m);
      const SE2 nx = prior * SE2::exp(next);
      for (int i = 0; i < m; ++i)
        nr[i] = perpendicular(armDirection(config_, measurements[i].arm)).dot(nx * measurements[i].point.q);
      double cost = next.dot(precision * next) + nr.dot(noise.solve(nr));
      if (cost <= old_cost + 1e-10) { delta = next; accepted = true; break; }
      step *= .5;
    }
    if (!accepted) { reject("update cost increased"); return false; }
    if (step.norm() < 1e-7) { converged = true; break; }
  }
  if (!converged) { reject("iteration limit without convergence"); return false; }
  Eigen::VectorXd residual(m);
  linearize(delta, residual, final_a, final_r);
  Eigen::LDLT<Eigen::MatrixXd> posterior_s(final_a * p0 * final_a.transpose() + final_r);
  if (posterior_s.info() != Eigen::Success || !posterior_s.isPositive()) { reject("posterior factorization failed"); return false; }
  Eigen::MatrixXd k = posterior_s.solve(final_a * p0).transpose();
  Mat3 a = Mat3::Identity() - k * final_a;
  Mat3 posterior = a * p0 * a.transpose() + k * final_r * k.transpose();
  const Mat3 reset = rightJacobian(delta);
  posterior = reset * posterior * reset.transpose();
  posterior = (.5 * (posterior + posterior.transpose())).eval();
  const SE2 next = prior * SE2::exp(delta);
  if (!next.finite() || !posterior.allFinite() || Eigen::LLT<Mat3>(posterior).info() != Eigen::Success) {
    reject("invalid posterior"); return false;
  }
  state_.robot_in_marker = next; state_.covariance = posterior; recovery_count_ = 0;
  // Report information in the current tangent, independent of the prior.
  Eigen::MatrixXd hc(m, 3);
  for (int i = 0; i < m; ++i)
    hc.row(i) = pointLineJacobian(next, measurements[i].point.q,
                                  perpendicular(armDirection(config_, measurements[i].arm)));
  diag.information_eigenvalues = Eigen::SelfAdjointEigenSolver<Mat3>(hc.transpose() * rsolver.solve(hc)).eigenvalues();
  diag.support_fraction = double(measurements.size()) / (2 * config_.max_points_per_arm);
  diag.mode = diag.points[0] && diag.points[1] ? "TRACKING" : "PARTIAL";
  diag.reason = "accepted"; diag.last_measurement = state_.stamp; ++diag.updates;
  diag.update_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
  return true;
}
}  // namespace vshape
