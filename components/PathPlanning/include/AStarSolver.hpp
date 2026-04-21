/**
 * @file AStarSolver.hpp
 * @brief A* pathfinding algorithm for waypoint graph navigation.
 *
 * Finds the shortest path through a graph of waypoints at arbitrary 2D
 * coordinates connected by explicit edges. Edge weights are automatically
 * computed from Euclidean distance between waypoints.
 *
 * Based on the A* algorithm by Damian Barczynski (ISC license), adapted for
 * ESP32 embedded use with memory-safe patterns.
 */

#pragma once

#include <cstdint>
#include <vector>

#include "Heuristic.hpp"
#include "IPathPlanner.hpp"

namespace WroRobotSoftware {
namespace PathPlanning {

/**
 * @struct AStarConfig
 * @brief Configuration parameters for the A* solver.
 */
struct AStarConfig {
    HeuristicType heuristicType = HeuristicType::Euclidean;  ///< Heuristic function selection
};

/**
 * @class AStarSolver
 * @brief Implements A* shortest path on a waypoint graph.
 *
 * Waypoints are added at arbitrary positions, connected by explicit edges.
 * Uses index-based node tracking with a contiguous node pool for memory safety.
 */
class AStarSolver : public IPathPlanner {
  public:
    /**
     * @brief Construct the solver with the given configuration.
     * @param config A* configuration (heuristic type).
     */
    explicit AStarSolver(const AStarConfig& config);

    /**
     * @brief Destructor.
     */
    ~AStarSolver() override;

    AStarSolver(const AStarSolver&) = delete;
    AStarSolver& operator=(const AStarSolver&) = delete;

    /**
     * @brief Move constructor.
     * @param other Solver to move from.
     */
    AStarSolver(AStarSolver&& other) noexcept;

    /**
     * @brief Move assignment operator.
     * @param other Solver to move from.
     * @return Reference to this solver.
     */
    AStarSolver& operator=(AStarSolver&& other) noexcept;

    /**
     * @brief Add a waypoint node at the given position.
     * @param position 2D coordinate of the waypoint.
     * @return Node index (0-based, auto-assigned sequentially).
     */
    int32_t addNode(GridPosition position) override;

    /**
     * @brief Add an edge between two existing nodes.
     * @param from Source node index.
     * @param to Destination node index.
     * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if indices are
     *         out of range.
     */
    esp_err_t addEdge(int32_t from, int32_t to) override;

    /**
     * @brief Block a node (mark as impassable obstacle).
     * @param node Node index to block.
     */
    void blockNode(int32_t node) override;

    /**
     * @brief Unblock a previously blocked node.
     * @param node Node index to unblock.
     */
    void unblockNode(int32_t node) override;

    /**
     * @brief Remove all nodes, edges, and obstacles.
     */
    void clearMap() override;

    /**
     * @brief Find the shortest path between two nodes using A*.
     * @param startNode Source node index.
     * @param targetNode Destination node index.
     * @param outPath Output: ordered sequence of positions from start to target.
     * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG for invalid
     *         indices, ESP_ERR_NOT_FOUND if no path exists.
     */
    esp_err_t findPath(int32_t startNode, int32_t targetNode, Path& outPath) override;

    /**
     * @brief Set the heuristic function used for path estimation.
     * @param type Heuristic type to use.
     */
    void setHeuristic(HeuristicType type);

    /**
     * @brief Get the current configuration.
     * @return Current A* configuration.
     */
    AStarConfig getConfig() const;

  private:
    /**
     * @struct WaypointEdge
     * @brief A weighted connection to another waypoint.
     */
    struct WaypointEdge {
        int32_t targetNode;  ///< Index of the connected waypoint
        uint32_t weight;     ///< Edge weight (Euclidean distance)
    };

    /**
     * @struct Waypoint
     * @brief A node in the waypoint graph.
     */
    struct Waypoint {
        GridPosition position;            ///< 2D coordinate
        bool blocked;                     ///< true if marked as obstacle
        std::vector<WaypointEdge> edges;  ///< Connections to other waypoints
    };

    /**
     * @struct SearchNode
     * @brief Internal node representation for the A* search.
     */
    struct SearchNode {
        uint32_t g;           ///< Cost from start to this node
        uint32_t h;           ///< Heuristic cost from this node to target
        int32_t waypointIdx;  ///< Index into waypoints vector
        int32_t parentIndex;  ///< Index into searchPool, or NO_PARENT

        /**
         * @brief Get the total estimated cost (f = g + h).
         * @return Combined cost score.
         */
        uint32_t getScore() const;
    };

    /**
     * @brief Check if a node index is valid.
     * @param node Node index to validate.
     * @return true if within bounds.
     */
    bool isValidNode(int32_t node) const;

    /**
     * @brief Find a search node for the given waypoint index in a list.
     * @param list List of search node indices.
     * @param waypointIdx Waypoint index to find.
     * @return Index into searchPool if found, NO_PARENT if not found.
     */
    int32_t findInSearchList(const std::vector<int32_t>& list, int32_t waypointIdx) const;

    /**
     * @brief Compute the heuristic cost between two positions.
     * @param source Starting position.
     * @param target Goal position.
     * @return Estimated cost.
     */
    uint32_t computeHeuristic(GridPosition source, GridPosition target) const;

    /**
     * @brief Compute Euclidean distance between two positions as edge weight.
     * @param a First position.
     * @param b Second position.
     * @return Distance as uint32_t.
     */
    static uint32_t computeEdgeWeight(GridPosition a, GridPosition b);

    /**
     * @brief Reconstruct the path from the search pool using parent indices.
     * @param targetIndex Index of the target search node in searchPool.
     * @return Ordered path from source to target.
     */
    Path reconstructPath(int32_t targetIndex) const;

    /// Sentinel value indicating no parent node exists
    static constexpr int32_t NO_PARENT = -1;

    static constexpr const char* TAG = "AStarSolver";

    AStarConfig config;                  ///< Solver configuration
    std::vector<Waypoint> waypoints;     ///< The waypoint graph
    std::vector<SearchNode> searchPool;  ///< Arena for A* search nodes
};

}  // namespace PathPlanning
}  // namespace WroRobotSoftware
