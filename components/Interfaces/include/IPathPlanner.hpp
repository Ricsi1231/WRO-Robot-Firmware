/**
 * @file IPathPlanner.hpp
 * @brief Hardware-agnostic interface for waypoint-based pathfinding algorithms.
 *
 * Defines a generic path planner API that works with any pathfinding
 * implementation (A*, Dijkstra, etc.) operating on a graph of waypoints
 * at arbitrary 2D coordinates connected by explicit edges.
 */

#pragma once

#include <cstdint>
#include <vector>

#include "esp_err.h"

namespace WroRobotSoftware {

/**
 * @struct GridPosition
 * @brief Represents a 2D coordinate.
 */
struct GridPosition {
    int32_t x;  ///< X coordinate
    int32_t y;  ///< Y coordinate

    /**
     * @brief Equality comparison.
     * @param other Position to compare with.
     * @return true if both coordinates match.
     */
    bool operator==(const GridPosition& other) const;

    /**
     * @brief Inequality comparison.
     * @param other Position to compare with.
     * @return true if coordinates differ.
     */
    bool operator!=(const GridPosition& other) const;
};

/// Ordered sequence of positions forming a path
using Path = std::vector<GridPosition>;

/// Sentinel value indicating an invalid node index
static constexpr int32_t INVALID_NODE = -1;

/**
 * @class IPathPlanner
 * @brief Abstract interface for waypoint-based pathfinding algorithms.
 */
class IPathPlanner {
  public:
    /**
     * @brief Virtual destructor.
     */
    virtual ~IPathPlanner() = default;

    /**
     * @brief Add a waypoint node at the given position.
     * @param position 2D coordinate of the waypoint.
     * @return Node index (0-based, auto-assigned sequentially).
     */
    virtual int32_t addNode(GridPosition position) = 0;

    /**
     * @brief Add an edge between two existing nodes.
     *
     * Edge weight is automatically calculated from Euclidean distance.
     * @param from Source node index.
     * @param to Destination node index.
     * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if indices are
     *         out of range.
     */
    virtual esp_err_t addEdge(int32_t from, int32_t to) = 0;

    /**
     * @brief Block a node (mark as impassable obstacle).
     * @param node Node index to block.
     */
    virtual void blockNode(int32_t node) = 0;

    /**
     * @brief Unblock a previously blocked node.
     * @param node Node index to unblock.
     */
    virtual void unblockNode(int32_t node) = 0;

    /**
     * @brief Remove all nodes, edges, and obstacles.
     */
    virtual void clearMap() = 0;

    /**
     * @brief Find the shortest path between two nodes.
     * @param startNode Source node index.
     * @param targetNode Destination node index.
     * @param outPath Output: ordered sequence of positions from start to target.
     * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG for invalid
     *         indices, ESP_ERR_NOT_FOUND if no path exists.
     */
    virtual esp_err_t findPath(int32_t startNode, int32_t targetNode, Path& outPath) = 0;
};

}  // namespace WroRobotSoftware
