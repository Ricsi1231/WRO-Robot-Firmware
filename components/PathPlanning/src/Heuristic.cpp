#include "Heuristic.hpp"

#include <algorithm>
#include <cmath>

namespace WroRobotSoftware {
namespace PathPlanning {

uint32_t Heuristic::manhattan(GridPosition source, GridPosition target) {
  int32_t dx = std::abs(source.x - target.x);
  int32_t dy = std::abs(source.y - target.y);
  return static_cast<uint32_t>(dx + dy);
}

uint32_t Heuristic::euclidean(GridPosition source, GridPosition target) {
  int64_t dx = std::abs(source.x - target.x);
  int64_t dy = std::abs(source.y - target.y);
  return static_cast<uint32_t>(std::sqrt(static_cast<double>(dx * dx + dy * dy)) + 0.5);
}

uint32_t Heuristic::octagonal(GridPosition source, GridPosition target) {
  int32_t dx = std::abs(source.x - target.x);
  int32_t dy = std::abs(source.y - target.y);
  return static_cast<uint32_t>((dx + dy) - std::min(dx, dy));
}

}  // namespace PathPlanning
}  // namespace WroRobotSoftware
