#include "AStarSolver.hpp"

#include <algorithm>
#include <cmath>

#include "esp_log.h"

namespace WroRobotSoftware {
namespace PathPlanning {

AStarSolver::AStarSolver(const AStarConfig& config) : config(config) {}

AStarSolver::~AStarSolver() = default;

AStarSolver::AStarSolver(AStarSolver&& other) noexcept
    : config(other.config),
      waypoints(std::move(other.waypoints)),
      searchPool(std::move(other.searchPool)) {}

AStarSolver& AStarSolver::operator=(AStarSolver&& other) noexcept {
  if (this != &other) {
    config = other.config;
    waypoints = std::move(other.waypoints);
    searchPool = std::move(other.searchPool);
  }
  return *this;
}

uint32_t AStarSolver::SearchNode::getScore() const {
  return g + h;
}

bool AStarSolver::isValidNode(int32_t node) const {
  return node >= 0 && node < static_cast<int32_t>(waypoints.size());
}

int32_t AStarSolver::findInSearchList(const std::vector<int32_t>& list,
                                      int32_t waypointIdx) const {
  for (int32_t idx : list) {
    if (searchPool[idx].waypointIdx == waypointIdx) {
      return idx;
    }
  }
  return NO_PARENT;
}

uint32_t AStarSolver::computeHeuristic(GridPosition source,
                                       GridPosition target) const {
  switch (config.heuristicType) {
    case HeuristicType::Manhattan:
      return Heuristic::manhattan(source, target);
    case HeuristicType::Octagonal:
      return Heuristic::octagonal(source, target);
    case HeuristicType::Euclidean:
    default:
      return Heuristic::euclidean(source, target);
  }
}

uint32_t AStarSolver::computeEdgeWeight(GridPosition a, GridPosition b) {
  int64_t dx = a.x - b.x;
  int64_t dy = a.y - b.y;
  return static_cast<uint32_t>(
      std::sqrt(static_cast<double>(dx * dx + dy * dy)) + 0.5);
}

Path AStarSolver::reconstructPath(int32_t targetIndex) const {
  Path path;
  int32_t current = targetIndex;

  while (current != NO_PARENT) {
    int32_t wpIdx = searchPool[current].waypointIdx;
    path.push_back(waypoints[wpIdx].position);
    current = searchPool[current].parentIndex;
  }

  std::reverse(path.begin(), path.end());
  return path;
}

int32_t AStarSolver::addNode(GridPosition position) {
  Waypoint wp;
  wp.position = position;
  wp.blocked = false;

  int32_t index = static_cast<int32_t>(waypoints.size());
  waypoints.push_back(wp);

  ESP_LOGI(TAG, "Added node %ld at (%ld,%ld)",
           static_cast<long>(index),
           static_cast<long>(position.x),
           static_cast<long>(position.y));

  return index;
}

esp_err_t AStarSolver::addEdge(int32_t from, int32_t to) {
  if (!isValidNode(from) || !isValidNode(to)) {
    ESP_LOGE(TAG, "Invalid edge indices: from=%ld, to=%ld (nodeCount=%zu)",
             static_cast<long>(from), static_cast<long>(to),
             waypoints.size());
    return ESP_ERR_INVALID_ARG;
  }

  if (from == to) {
    ESP_LOGE(TAG, "Cannot add self-loop: node %ld", static_cast<long>(from));
    return ESP_ERR_INVALID_ARG;
  }


  for (const auto& edge : waypoints[from].edges) {
    if (edge.targetNode == to) {
      ESP_LOGW(TAG, "Edge %ld->%ld already exists",
               static_cast<long>(from), static_cast<long>(to));
      return ESP_OK;
    }
  }

  uint32_t weight =
      computeEdgeWeight(waypoints[from].position, waypoints[to].position);

  waypoints[from].edges.push_back({to, weight});
  waypoints[to].edges.push_back({from, weight});

  return ESP_OK;
}

void AStarSolver::blockNode(int32_t node) {
  if (isValidNode(node)) {
    waypoints[node].blocked = true;
  } else {
    ESP_LOGW(TAG, "Cannot block invalid node %ld", static_cast<long>(node));
  }
}

void AStarSolver::unblockNode(int32_t node) {
  if (isValidNode(node)) {
    waypoints[node].blocked = false;
  } else {
    ESP_LOGW(TAG, "Cannot unblock invalid node %ld", static_cast<long>(node));
  }
}

void AStarSolver::clearMap() {
  waypoints.clear();
  searchPool.clear();
}

esp_err_t AStarSolver::findPath(int32_t startNode, int32_t targetNode,
                                Path& outPath) {
  outPath.clear();

  if (!isValidNode(startNode) || !isValidNode(targetNode)) {
    ESP_LOGE(TAG, "Invalid node indices: start=%ld, target=%ld (nodeCount=%zu)",
             static_cast<long>(startNode), static_cast<long>(targetNode),
             waypoints.size());
    return ESP_ERR_INVALID_ARG;
  }

  if (waypoints[startNode].blocked) {
    ESP_LOGE(TAG, "Start node %ld is blocked", static_cast<long>(startNode));
    return ESP_ERR_INVALID_ARG;
  }

  if (waypoints[targetNode].blocked) {
    ESP_LOGE(TAG, "Target node %ld is blocked", static_cast<long>(targetNode));
    return ESP_ERR_INVALID_ARG;
  }

  if (startNode == targetNode) {
    outPath.push_back(waypoints[startNode].position);
    return ESP_OK;
  }


  searchPool.clear();
  searchPool.reserve(waypoints.size());


  std::vector<int32_t> openList;
  std::vector<int32_t> closedList;


  SearchNode startSearch;
  startSearch.g = 0;
  startSearch.h = computeHeuristic(waypoints[startNode].position,
                                   waypoints[targetNode].position);
  startSearch.waypointIdx = startNode;
  startSearch.parentIndex = NO_PARENT;

  searchPool.push_back(startSearch);
  openList.push_back(0);

  while (!openList.empty()) {

    size_t bestOpenIdx = 0;
    uint32_t bestScore = searchPool[openList[0]].getScore();

    for (size_t i = 1; i < openList.size(); i++) {
      uint32_t score = searchPool[openList[i]].getScore();
      if (score < bestScore) {
        bestScore = score;
        bestOpenIdx = i;
      }
    }

    int32_t currentSearchIdx = openList[bestOpenIdx];
    int32_t currentWpIdx = searchPool[currentSearchIdx].waypointIdx;


    if (currentWpIdx == targetNode) {
      outPath = reconstructPath(currentSearchIdx);
      ESP_LOGI(TAG, "Path found from node %ld to %ld, length=%u",
               static_cast<long>(startNode), static_cast<long>(targetNode),
               static_cast<unsigned>(outPath.size()));
      return ESP_OK;
    }


    openList.erase(openList.begin() + bestOpenIdx);
    closedList.push_back(currentSearchIdx);


    for (const auto& edge : waypoints[currentWpIdx].edges) {
      int32_t neighborWpIdx = edge.targetNode;


      if (waypoints[neighborWpIdx].blocked) {
        continue;
      }


      if (findInSearchList(closedList, neighborWpIdx) != NO_PARENT) {
        continue;
      }

      uint32_t tentativeG = searchPool[currentSearchIdx].g + edge.weight;


      int32_t existingIdx = findInSearchList(openList, neighborWpIdx);

      if (existingIdx != NO_PARENT) {

        if (tentativeG < searchPool[existingIdx].g) {
          searchPool[existingIdx].g = tentativeG;
          searchPool[existingIdx].parentIndex = currentSearchIdx;
        }
      } else {

        SearchNode neighborSearch;
        neighborSearch.g = tentativeG;
        neighborSearch.h = computeHeuristic(waypoints[neighborWpIdx].position,
                                            waypoints[targetNode].position);
        neighborSearch.waypointIdx = neighborWpIdx;
        neighborSearch.parentIndex = currentSearchIdx;

        int32_t newIdx = static_cast<int32_t>(searchPool.size());
        searchPool.push_back(neighborSearch);
        openList.push_back(newIdx);
      }
    }
  }

  ESP_LOGW(TAG, "No path found from node %ld to %ld",
           static_cast<long>(startNode), static_cast<long>(targetNode));
  return ESP_ERR_NOT_FOUND;
}

void AStarSolver::setHeuristic(HeuristicType type) {
  config.heuristicType = type;
}

AStarConfig AStarSolver::getConfig() const {
  return config;
}

}  // namespace PathPlanning
}  // namespace WroRobotSoftware
