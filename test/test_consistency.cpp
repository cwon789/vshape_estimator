#include "ros2_bot_estimator_lidar_vshape/filter.hpp"
#include <Eigen/Cholesky>
#include <boost/math/distributions/chi_squared.hpp>
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <random>

using namespace vshape;
int main() {
  constexpr int seeds = 200, per_arm = 24, dof = 2 * per_arm;
  Config c; c.huber_delta = 0; c.angle_sigma = 0; c.gate_sigma = 8;
  const SE2 truth(1.3, -.3, 2.8);
  Vec3 stddev(.015, .015, .006);
  const Mat3 p0 = stddev.array().square().matrix().asDiagonal();
  double nees_sum = 0, nis_sum = 0; int covered = 0, accepted = 0;
  Eigen::VectorXd innovation_sum = Eigen::VectorXd::Zero(dof);
  double max_skew = 0, min_eigenvalue = 1e9;
  for (int seed = 0; seed < seeds; ++seed) {
    std::mt19937 rng(seed + 1234); std::normal_distribution<double> gaussian;
    Vec3 error;
    for (int j = 0; j < 3; ++j) error[j] = gaussian(rng) * stddev[j];
    const SE2 prior = truth * SE2::exp(error);
    Filter filter(c); filter.initialize(0, prior, p0);
    std::vector<Observation> obs;
    for (int arm = 0; arm < 2; ++arm) {
      const double common = gaussian(rng) * c.model_sigma;
      const Vec2 direction = armDirection(c, arm), n = perpendicular(direction);
      for (int j = 0; j < per_arm; ++j) {
        Point p;
        p.q = truth.inverse() * (direction * (.04 + .4 * j / (per_arm - 1.)));
        p.q += truth.rotation().transpose() * n * common;
        Vec2 independent; independent << gaussian(rng), gaussian(rng);
        p.q += c.range_sigma * independent;
        p.covariance = Mat2::Identity() * c.range_sigma * c.range_sigma;
        obs.push_back({p, arm});
      }
    }
    Eigen::MatrixXd h(dof, 3), r = Eigen::MatrixXd::Zero(dof, dof);
    Eigen::VectorXd residual(dof);
    for (int i = 0; i < dof; ++i) {
      const Vec2 n = perpendicular(armDirection(c, obs[i].arm));
      h.row(i) = pointLineJacobian(prior, obs[i].point.q, n);
      residual[i] = n.dot(prior * obs[i].point.q);
      r(i, i) = c.range_sigma * c.range_sigma;
      for (int j = 0; j < dof; ++j) if (obs[i].arm == obs[j].arm) r(i, j) += c.model_sigma * c.model_sigma;
    }
    Eigen::LLT<Eigen::MatrixXd> s(h * p0 * h.transpose() + r);
    innovation_sum += s.matrixL().solve(residual);
    if (!filter.update(obs)) continue;
    ++accepted;
    const auto& state = filter.state();
    const Vec3 e = (state.robot_in_marker.inverse() * truth).log();
    double nees = e.dot(state.covariance.ldlt().solve(e));
    nees_sum += nees; nis_sum += state.diagnostics.nis;
    auto chi = boost::math::chi_squared(3);
    if (nees >= boost::math::quantile(chi, .025) && nees <= boost::math::quantile(chi, .975)) ++covered;
    max_skew = std::max(max_skew, (state.covariance - state.covariance.transpose()).norm());
    Eigen::LDLT<Mat3> ps(state.covariance);
    min_eigenvalue = std::min(min_eigenvalue, ps.vectorD().minCoeff());
  }
  auto bound = [](int dimension, double probability) {
    return boost::math::quantile(boost::math::chi_squared(dimension * seeds), probability) / seeds;
  };
  const double nees = nees_sum / seeds, nis = nis_sum / seeds;
  const double nees_low = bound(3, .025), nees_high = bound(3, .975);
  const double nis_low = bound(dof, .025), nis_high = bound(dof, .975);
  const double coverage = double(covered) / seeds;
  const double max_mean = (innovation_sum / seeds).cwiseAbs().maxCoeff();
  // Simultaneous per-channel mean check; 0.3 exceeds four standard errors.
  const bool pass = accepted == seeds && nees >= nees_low && nees <= nees_high &&
    nis >= nis_low && nis <= nis_high && coverage >= .90 && coverage <= .99 &&
    min_eigenvalue > 0 && max_skew < 1e-12 && max_mean < .3;
  std::cout << std::setprecision(12)
    << "{\"experiment\":\"fixed_association_gaussian_single_update\",\"seeds\":" << seeds
    << ",\"accepted\":" << accepted << ",\"state_dof\":3,\"measurement_dof\":" << dof
    << ",\"mean_nees\":" << nees << ",\"nees_95_interval\":[" << nees_low << ',' << nees_high
    << "],\"mean_nis\":" << nis << ",\"nis_95_interval\":[" << nis_low << ',' << nis_high
    << "],\"coverage_95\":" << coverage << ",\"max_abs_whitened_innovation_mean\":" << max_mean
    << ",\"max_covariance_skew\":" << max_skew << ",\"min_ldlt_pivot\":" << min_eigenvalue
    << ",\"passed\":" << (pass ? "true" : "false") << "}\n";
  return pass ? 0 : 1;
}
