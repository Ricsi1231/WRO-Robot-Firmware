/**
 * @file Heuristic.hpp
 * @brief Heuristic functions for pathfinding algorithms.
 *
 * Provides common distance heuristics (Manhattan, Euclidean, Octagonal) used
 * by A* and similar algorithms to estimate the cost to reach the target.
 */

#pragma once

#include <cstdint>

#include "IPathPlanner.hpp"

namespace WroRobotSoftware {
namespace PathPlanning {

/**
 * @enum HeuristicType
 * @brief Available heuristic functions for pathfinding.
 */
enum class HeuristicType {
  Manhattan,   ///< Sum of absolute differences (L1 norm)
  Euclidean,   ///< Straight-line distance (L2 norm)
  Octagonal    ///< Chebyshev-like distance
};

/**
 * @struct Heuristic
 * @brief Static utility providing distance heuristic calculations.
 */
struct Heuristic {
  Heuristic() = delete;

  /**
   * @brief Manhattan distance (L1 norm).
   * @param source Starting position.
   * @param target Goal position.
   * @return Estimated cost as sum of absolute coordinate differences.
   */
  static uint32_t manhattan(GridPosition source, GridPosition target);

  /**
   * @brief Euclidean distance (L2 norm).
   * @param source Starting position.
   * @param target Goal position.
   * @return Estimated cost as straight-line distance.
   */
  static uint32_t euclidean(GridPosition source, GridPosition target);

  /**
   * @brief Octagonal distance (Chebyshev-like).
   * @param source Starting position.
   * @param target Goal position.
   * @return Estimated cost suitable for diagonal movement.
   */
  static uint32_t octagonal(GridPosition source, GridPosition target);
};

}  // namespace PathPlanning
}  // namespace WroRobotSoftware
