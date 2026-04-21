/**
 * @file IPathPlanner.cpp
 * @brief Implementation of GridPosition operators.
 */

#include "IPathPlanner.hpp"

namespace WroRobotSoftware {

bool GridPosition::operator==(const GridPosition& other) const {
  return x == other.x && y == other.y;
}

bool GridPosition::operator!=(const GridPosition& other) const {
  return !(*this == other);
}

}  // namespace WroRobotSoftware
