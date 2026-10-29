#pragma once
#include "se2.hpp"

namespace vshape {
// A recessed V is controlled from its opening plane, halfway between arm tips.
// `depth` is arm_length*cos(opening_angle/2), along the geometric opening axis.
inline SE2 openingFacePose(const SE2& marker_in_base, double depth) {
  const Vec2 center = marker_in_base * Vec2(depth, 0.);
  return {center.x(), center.y(), marker_in_base.yaw};
}

inline Mat3 openingFaceCovariance(const SE2& marker_in_base,
                                 const Mat3& apex_covariance, double depth) {
  Mat3 j = Mat3::Identity();
  j.topRightCorner<2, 1>() = perpendicular(marker_in_base.rotation() * Vec2(depth, 0.));
  return j * apex_covariance * j.transpose();
}

// Match reflector: choose the outward pi branch ONCE at acquisition, then
// preserve real relative rotation. Input is the requested docking reference pose.
class DockingConvention {
 public:
  void reset() { valid_ = false; offset_ = 0.; }
  SE2 apply(const SE2& marker_in_base) {
    if (!valid_) {
      if (Vec2(std::cos(marker_in_base.yaw), std::sin(marker_in_base.yaw)).dot(marker_in_base.p) < 0)
        offset_ = 3.141592653589793;
      valid_ = true;
    }
    return {marker_in_base.p.x(), marker_in_base.p.y(), marker_in_base.yaw + offset_};
  }
 private:
  bool valid_ = false;
  double offset_ = 0.;
};
}  // namespace vshape
