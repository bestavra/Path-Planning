#include "path/AStarPlanner.h"

#include "map/Map.h"
#include "path/PlannerUtils.h"

#include <algorithm>
#include <cstddef>
#include <functional>
#include <glm/glm.hpp>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace {

struct Node {
    glm::ivec2 cell;
    float g = 0.0f;
    float f = 0.0f;
};

struct NodeCompare {
    bool operator()(const Node& lhs, const Node& rhs) const {
        return lhs.f > rhs.f;
    }
};

struct IVec2Hash {
    std::size_t operator()(const glm::ivec2& v) const noexcept {
        std::size_t h1 = std::hash<int>()(v.x);
        std::size_t h2 = std::hash<int>()(v.y);
        return h1 ^ (h2 << 1);
    }
};

float heuristic(const glm::ivec2& a, const glm::ivec2& b) {
    const int dx = std::abs(b.x - a.x);
    const int dy = std::abs(b.y - a.y);
    const int minDelta = std::min(dx, dy);
    const int maxDelta = std::max(dx, dy);
    constexpr float sqrt2 = 1.41421356237f;
    return (maxDelta - minDelta) + sqrt2 * static_cast<float>(minDelta);
}

std::vector<glm::ivec2> reconstructPath(
    const std::unordered_map<glm::ivec2, glm::ivec2, IVec2Hash>& cameFrom, glm::ivec2 current) {
    std::vector<glm::ivec2> cells;
    cells.push_back(current);
    auto it = cameFrom.find(current);
    while (it != cameFrom.end()) {
        current = it->second;
        cells.push_back(current);
        it = cameFrom.find(current);
    }
    std::reverse(cells.begin(), cells.end());
    return cells;
}

} // namespace

namespace planner {

void AStarPlanner::solve(PlannedPath& path) {
    const auto& grid = getMap();

    const PlannerPosition& startPos = getStart();
    const PlannerPosition& goalPos = getGoal();

    if (!startPos.isCell() || !goalPos.isCell()) {
        throw std::runtime_error("AStarPlanner currently supports only cell-based start/goal");
    }

    glm::ivec2 startCell = startPos.cell;
    glm::ivec2 goalCell = goalPos.cell;

    if (!planner::utils::isCellWithinBounds(grid, startCell) || !planner::utils::isCellWithinBounds(grid, goalCell)) {
        path.exploredCells.clear();
        path.waypoints.clear();
        return;
    }

    if (!planner::utils::isTraversableCell(grid, startCell) || !planner::utils::isTraversableCell(grid, goalCell)) {
        path.exploredCells.clear();
        path.waypoints.clear();
        return;
    }

    if (startCell == goalCell) {
        path.exploredCells = {startCell};
        path.waypoints = {glm::vec2(startCell) + glm::vec2(0.5f, 0.5f)};
        return;
    }

    std::priority_queue<Node, std::vector<Node>, NodeCompare> openSet;
    openSet.push(Node{startCell, 0.0f, heuristic(startCell, goalCell)});

    std::unordered_map<glm::ivec2, glm::ivec2, IVec2Hash> cameFrom;
    std::unordered_map<glm::ivec2, float, IVec2Hash> gScore;
    gScore[startCell] = 0.0f;

    std::unordered_map<glm::ivec2, float, IVec2Hash> fScore;
    fScore[startCell] = heuristic(startCell, goalCell);

    std::unordered_set<glm::ivec2, IVec2Hash> closedSet;
    path.exploredCells.clear();
    path.waypoints.clear();

    const glm::ivec2 neighbors[8] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

    while (!openSet.empty()) {
        Node current = openSet.top();
        openSet.pop();

        if (closedSet.find(current.cell) != closedSet.end()) {
            continue;
        }
        closedSet.insert(current.cell);
        path.exploredCells.push_back(current.cell);

        if (current.cell == goalCell) {
            auto cells = reconstructPath(cameFrom, current.cell);
            std::vector<glm::vec2> points;
            points.reserve(cells.size());
            for (const auto& cell : cells) {
                points.emplace_back(static_cast<float>(cell.x) + 0.5f, static_cast<float>(cell.y) + 0.5f);
            }
            path.waypoints = std::move(points);
            return;
        }

        for (const auto& offset : neighbors) {
            glm::ivec2 neighbor = current.cell + offset;
            if (!planner::utils::isCellWithinBounds(grid, neighbor) ||
                !planner::utils::isTraversableCell(grid, neighbor)) {
                continue;
            }

            const bool isDiagonal = offset.x != 0 && offset.y != 0;
            if (isDiagonal) {
                const glm::ivec2 adjA{current.cell.x + offset.x, current.cell.y};
                const glm::ivec2 adjB{current.cell.x, current.cell.y + offset.y};
                if (!planner::utils::isCellWithinBounds(grid, adjA) ||
                    !planner::utils::isCellWithinBounds(grid, adjB) || !planner::utils::isTraversableCell(grid, adjA) ||
                    !planner::utils::isTraversableCell(grid, adjB)) {
                    continue;
                }
            }

            const float tentativeG = gScore[current.cell] + planner::utils::traversalCost(grid, neighbor, isDiagonal);
            auto gIt = gScore.find(neighbor);
            if (gIt == gScore.end() || tentativeG < gIt->second) {
                cameFrom[neighbor] = current.cell;
                gScore[neighbor] = tentativeG;
                float f = tentativeG + heuristic(neighbor, goalCell);
                fScore[neighbor] = f;
                openSet.push(Node{neighbor, tentativeG, f});
            }
        }
    }

    // No path found; exploredCells already captures visited nodes
}

} // namespace planner
