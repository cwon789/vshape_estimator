#pragma once
#include <Eigen/Core>
#include <Eigen/LU>
#include <cmath>

namespace vshape {
using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using Mat2 = Eigen::Matrix2d;
using Mat3 = Eigen::Matrix3d;
inline double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }
inline Vec2 perpendicular(const Vec2& p) { return {-p.y(), p.x()}; }

// T_AB maps points B -> A. All filter errors are RIGHT tangent [rho_x,rho_y,yaw].
struct SE2 {
  Vec2 p = Vec2::Zero();
  double yaw = 0.0;
  SE2() = default;
  SE2(double x, double y, double a) : p(x, y), yaw(wrap(a)) {}
  Mat2 rotation() const {
    const double c = std::cos(yaw), s = std::sin(yaw);
    Mat2 r; r << c, -s, s, c; return r;
  }
  Vec3 vector() const { return {p.x(), p.y(), yaw}; }
  bool finite() const { return p.allFinite() && std::isfinite(yaw); }
  Vec2 operator*(const Vec2& q) const { return rotation() * q + p; }
  SE2 operator*(const SE2& b) const {
    Vec2 t = *this * b.p; return {t.x(), t.y(), yaw + b.yaw};
  }
  SE2 inverse() const {
    Vec2 t = -rotation().transpose() * p; return {t.x(), t.y(), -yaw};
  }
  static SE2 exp(const Vec3& x) {
    const double a = x.z(), a2 = a * a;
    const double s = std::abs(a) < 1e-5 ? 1 - a2 / 6 + a2 * a2 / 120 : std::sin(a) / a;
    const double c = std::abs(a) < 1e-5 ? a / 2 - a * a2 / 24 + a * a2 * a2 / 720 : (1 - std::cos(a)) / a;
    return {s * x.x() - c * x.y(), c * x.x() + s * x.y(), a};
  }
  Vec3 log() const {
    const double a = wrap(yaw), a2 = a * a;
    const double k = std::abs(a) < 1e-5 ? 1 - a2 / 12 - a2 * a2 / 720 : (a / 2) / std::tan(a / 2);
    return {k * p.x() + a / 2 * p.y(), -a / 2 * p.x() + k * p.y(), a};
  }
  Mat3 adjoint() const {
    Mat3 a = Mat3::Identity(); a.topLeftCorner<2, 2>() = rotation();
    a(0, 2) = p.y(); a(1, 2) = -p.x(); return a;
  }
};

inline Mat3 rightJacobian(const Vec3& x) {
  Mat3 ad = Mat3::Zero();
  ad(0, 1) = -x.z(); ad(1, 0) = x.z();
  ad(0, 2) = x.y(); ad(1, 2) = -x.x();
  Mat3 sum = Mat3::Identity(), term = Mat3::Identity();
  for (int n = 1; n <= 30; ++n) {
    term = (-term * ad / double(n + 1)).eval(); sum += term;
  }
  return sum;
}
inline Mat3 rightJacobianInverse(const Vec3& x) {
  return rightJacobian(x).partialPivLu().solve(Mat3::Identity());
}
inline Eigen::RowVector3d pointLineJacobian(const SE2& x, const Vec2& q, const Vec2& n) {
  Eigen::Matrix<double, 2, 3> b;
  b.leftCols<2>() = Mat2::Identity(); b.col(2) = perpendicular(q);
  return n.transpose() * x.rotation() * b;
}
inline Mat3 poseCovariance(const SE2& x, const Mat3& p) {
  Mat3 j = Mat3::Identity(); j.topLeftCorner<2, 2>() = x.rotation();
  return j * p * j.transpose();
}
inline Mat3 inversePoseCovariance(const SE2& x, const Mat3& p) {
  Mat3 a = x.adjoint();
  return poseCovariance(x.inverse(), a * p * a.transpose());
}
}  // namespace vshape
