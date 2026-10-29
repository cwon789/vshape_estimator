#pragma once
#include "types.hpp"
namespace vshape {
struct Detection { bool valid = false; SE2 marker_in_base; double score = 0; };
Detection detectV(const Points& points, const Config& config);
std::vector<Observation> associate(const Points& points, const SE2& robot_in_marker, const Config& config);
}  // namespace vshape
