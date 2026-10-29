#pragma once
#include "types.hpp"

namespace vshape {
class Filter {
 public:
  explicit Filter(Config config = {});
  const Estimate& state() const { return state_; }
  void initialize(double stamp, const SE2& robot_in_marker, const Mat3& covariance);
  void predict(double stamp, const Vec3& twist, bool fresh = true);
  bool update(const std::vector<Observation>& measurements);
  void reject(const std::string& reason);
  bool recoveryReady(const SE2& hypothesis);
  Estimate& mutableState() { return state_; } // only Tracker owns mutations/replay
 private:
  Config config_;
  Estimate state_;
  int recovery_count_ = 0;
  SE2 recovery_pose_;
};
}  // namespace vshape
