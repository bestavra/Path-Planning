#include "path/DStarLitePlanner.h"

#include "path/PlannerUtils.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>

namespace {

float computeHeuristic(const glm::ivec2& a, const glm::ivec2& b) {
    const int dx = std::abs(b.x - a.x);
    const int dy = std::abs(b.y - a.y);
    const int minDelta = std::min(dx, dy);
    const int maxDelta = std::max(dx, dy);
    constexpr float sqrt2 = 1.41421356237f;
    return (maxDelta - minDelta) + sqrt2 * static_cast<float>(minDelta);
}

constexpr float s_infinity = std::numeric_limits<float>::infinity();

} // namespace

namespace planner {

bool DStarLitePlanner::QueueEntryCompare::operator()(const QueueEntry& lhs, const QueueEntry& rhs) const noexcept {
    return DStarLitePlanner::keyLess(rhs.key, lhs.key);
}

DStarLitePlanner::DStarLitePlanner() {
    resetPlannerState();
}

void DStarLitePlanner::resetPlannerState() {
    while (!m_openList.empty()) {
        m_openList.pop();
    }
    m_nodeInfo.clear();
    m_openTable.clear();
    m_expandedNodes.clear();
    m_keyModifier = 0.0f;
    m_queueSequence = 0;
    m_initialized = false;
}

void DStarLitePlanner::setMap(const map::Grid& grid) {
    if (!m_hasMap) {
        m_map = grid;
        m_hasMap = true;
        resetPlannerState();
        m_pendingUpdates.clear();
        return;
    }

    const auto& oldMeta = m_map.metadata();
    const auto& newMeta = grid.metadata();
    const bool dimensionChanged = oldMeta.width != newMeta.width || oldMeta.height != newMeta.height;

    if (dimensionChanged) {
        m_map = grid;
        m_pendingUpdates.clear();
        resetPlannerState();
        m_hasMap = true;
        return;
    }

    const auto& oldCells = m_map.cells();
    const auto& newCells = grid.cells();
    const std::size_t total = std::min(oldCells.size(), newCells.size());

    for (std::size_t idx = 0; idx < total; ++idx) {
        const bool oldBlocked = oldCells[idx] >= 1.0f || oldCells[idx] == map::Grid::s_missingData;
        const bool newBlocked = newCells[idx] >= 1.0f || newCells[idx] == map::Grid::s_missingData;
        if (oldBlocked != newBlocked) {
            const int x = static_cast<int>(idx % newMeta.width);
            const int y = static_cast<int>(idx / newMeta.width);
            m_pendingUpdates.insert(glm::ivec2{x, y});
        }
    }

    m_map = grid;
    m_hasMap = true;
}

void DStarLitePlanner::setStart(const PlannerPosition& position) {
    if (!position.isCell()) {
        throw std::runtime_error("D* Lite planner currently supports only cell-based starts");
    }
    if (!m_hasMap) {
        m_startCell = position.cell;
        m_lastStart = m_startCell;
        m_hasStart = true;
        return;
    }

    if (m_hasStart) {
        m_keyModifier += computeHeuristic(m_lastStart, position.cell);
    }

    m_startCell = position.cell;
    if (!m_initialized) {
        m_lastStart = m_startCell;
    }
    m_hasStart = true;
}

void DStarLitePlanner::setGoal(const PlannerPosition& position) {
    if (!position.isCell()) {
        throw std::runtime_error("D* Lite planner currently supports only cell-based goals");
    }
    m_goalCell = position.cell;
    m_hasGoal = true;
    m_initialized = false;
}

bool DStarLitePlanner::isTrav(const glm::ivec2& cell) const {
    return utils::isTraversableCell(m_map, cell);
}

DStarLitePlanner::NodeData& DStarLitePlanner::dataFor(const glm::ivec2& cell) {
    return m_nodeInfo[cell];
}

float DStarLitePlanner::g(const glm::ivec2& cell) const {
    const auto it = m_nodeInfo.find(cell);
    if (it == m_nodeInfo.end()) {
        return s_infinity;
    }
    return it->second.g;
}

float DStarLitePlanner::rhs(const glm::ivec2& cell) const {
    const auto it = m_nodeInfo.find(cell);
    if (it == m_nodeInfo.end()) {
        return s_infinity;
    }
    return it->second.rhs;
}

void DStarLitePlanner::setG(const glm::ivec2& cell, float value) {
    dataFor(cell).g = value;
}

void DStarLitePlanner::setRhs(const glm::ivec2& cell, float value) {
    dataFor(cell).rhs = value;
}

bool DStarLitePlanner::keyLess(const Key& lhs, const Key& rhs) {
    if (lhs.k1 < rhs.k1) {
        return true;
    }
    if (lhs.k1 > rhs.k1) {
        return false;
    }
    return lhs.k2 < rhs.k2;
}

DStarLitePlanner::Key DStarLitePlanner::calculateKey(const glm::ivec2& cell) const {
    const float gVal = g(cell);
    const float rhsVal = rhs(cell);
    const float minVal = std::min(gVal, rhsVal);
    return Key{minVal + computeHeuristic(m_startCell, cell) + m_keyModifier, minVal};
}

void DStarLitePlanner::pushOpen(const glm::ivec2& cell, const Key& key) {
    m_openList.push(QueueEntry{cell, key, m_queueSequence++});
    m_openTable[cell] = key;
}

std::vector<glm::ivec2> DStarLitePlanner::getNeighbors(const glm::ivec2& cell) const {
    static const glm::ivec2 offsets[8] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

    std::vector<glm::ivec2> neighbors;
    neighbors.reserve(8);

    for (const auto& offset : offsets) {
        const glm::ivec2 candidate = cell + offset;
        if (!utils::isCellWithinBounds(m_map, candidate)) {
            continue;
        }

        const bool diagonal = offset.x != 0 && offset.y != 0;
        if (diagonal) {
            const glm::ivec2 adjA{cell.x + offset.x, cell.y};
            const glm::ivec2 adjB{cell.x, cell.y + offset.y};
            if (!utils::isCellWithinBounds(m_map, adjA) || !utils::isCellWithinBounds(m_map, adjB) || !isTrav(adjA) ||
                !isTrav(adjB)) {
                continue;
            }
        }

        if (!isTrav(candidate)) {
            continue;
        }

        neighbors.push_back(candidate);
    }

    return neighbors;
}

float DStarLitePlanner::edgeCost(const glm::ivec2& from, const glm::ivec2& to) const {
    if (!utils::isCellWithinBounds(m_map, to)) {
        return s_infinity;
    }

    const bool diagonal = (from.x != to.x) && (from.y != to.y);
    if (diagonal) {
        const glm::ivec2 adjA{from.x, to.y};
        const glm::ivec2 adjB{to.x, from.y};
        if (!utils::isCellWithinBounds(m_map, adjA) || !utils::isCellWithinBounds(m_map, adjB) || !isTrav(adjA) ||
            !isTrav(adjB)) {
            return s_infinity;
        }
    }

    if (!isTrav(to)) {
        return s_infinity;
    }

    return utils::traversalCost(m_map, to, diagonal);
}

void DStarLitePlanner::initializePlanner() {
    resetPlannerState();

    setRhs(m_goalCell, 0.0f);
    const Key key = calculateKey(m_goalCell);
    pushOpen(m_goalCell, key);
    m_lastStart = m_startCell;
    m_initialized = true;
}

void DStarLitePlanner::applyPendingUpdates() {
    if (m_pendingUpdates.empty()) {
        return;
    }

    std::vector<glm::ivec2> toProcess;
    toProcess.reserve(m_pendingUpdates.size() * 5);

    for (const auto& cell : m_pendingUpdates) {
        toProcess.push_back(cell);
        const auto neighbors = getNeighbors(cell);
        toProcess.insert(toProcess.end(), neighbors.begin(), neighbors.end());
    }

    m_pendingUpdates.clear();

    for (const auto& cell : toProcess) {
        updateVertex(cell);
    }
}

void DStarLitePlanner::updateVertex(const glm::ivec2& cell) {
    if (cell == m_goalCell) {
        setRhs(cell, 0.0f);
    } else {
        float minRhs = s_infinity;
        const auto neighbors = getNeighbors(cell);
        for (const auto& neighbor : neighbors) {
            const float cost = edgeCost(cell, neighbor);
            if (!std::isfinite(cost)) {
                continue;
            }
            minRhs = std::min(minRhs, cost + g(neighbor));
        }
        setRhs(cell, minRhs);
    }

    if (g(cell) != rhs(cell)) {
        pushOpen(cell, calculateKey(cell));
    } else {
        m_openTable.erase(cell);
    }
}

void DStarLitePlanner::computeShortestPath() {
    m_expandedNodes.clear();
    std::unordered_set<glm::ivec2, IVec2Hash> expandedSet;

    while (!m_openList.empty()) {
        const QueueEntry top = m_openList.top();
        const auto tableIt = m_openTable.find(top.cell);
        if (tableIt == m_openTable.end() || tableIt->second.k1 != top.key.k1 || tableIt->second.k2 != top.key.k2) {
            m_openList.pop();
            continue;
        }

        const Key startKey = calculateKey(m_startCell);
        if (!keyLess(top.key, startKey) && rhs(m_startCell) == g(m_startCell)) {
            break;
        }

        m_openList.pop();
        m_openTable.erase(top.cell);

        if (expandedSet.insert(top.cell).second) {
            m_expandedNodes.push_back(top.cell);
        }

        const float gOld = g(top.cell);
        const float rhsVal = rhs(top.cell);

        if (gOld > rhsVal) {
            setG(top.cell, rhsVal);
            const auto predecessors = getNeighbors(top.cell);
            for (const auto& pred : predecessors) {
                updateVertex(pred);
            }
        } else {
            setG(top.cell, s_infinity);
            updateVertex(top.cell);
            const auto predecessors = getNeighbors(top.cell);
            for (const auto& pred : predecessors) {
                updateVertex(pred);
            }
        }
    }
}

bool DStarLitePlanner::isValidStartGoal() const {
    if (!m_hasMap || !m_hasStart || !m_hasGoal) {
        return false;
    }
    if (!utils::isCellWithinBounds(m_map, m_startCell) || !utils::isCellWithinBounds(m_map, m_goalCell)) {
        return false;
    }
    if (!isTrav(m_goalCell)) {
        return false;
    }
    if (m_startCell != m_goalCell && !isTrav(m_startCell)) {
        return false;
    }
    return true;
}

PlannedPath DStarLitePlanner::computePath() {
    if (!isValidStartGoal()) {
        PlannedPath empty;
        empty.style = PathStyle::Polyline;
        empty.success = false;
        return empty;
    }

    if (!m_initialized) {
        initializePlanner();
    }

    if (m_lastStart != m_startCell) {
        m_keyModifier += computeHeuristic(m_lastStart, m_startCell);
        m_lastStart = m_startCell;
    }

    applyPendingUpdates();
    updateVertex(m_startCell);
    computeShortestPath();

    PlannedPath path;
    path.style = PathStyle::Polyline;
    path.exploredCells = m_expandedNodes;

    if (!std::isfinite(rhs(m_startCell))) {
        path.success = false;
        return path;
    }

    std::vector<glm::ivec2> cells;
    cells.reserve(m_map.metadata().cellCount());
    cells.push_back(m_startCell);

    glm::ivec2 current = m_startCell;
    const std::size_t limit = m_map.metadata().cellCount() + 1;

    for (std::size_t i = 0; i < limit && current != m_goalCell; ++i) {
        float bestScore = s_infinity;
        glm::ivec2 bestCell = current;
        const auto neighbors = getNeighbors(current);
        for (const auto& neighbor : neighbors) {
            const float cost = edgeCost(current, neighbor);
            if (!std::isfinite(cost)) {
                continue;
            }
            const float score = cost + g(neighbor);
            if (score < bestScore) {
                bestScore = score;
                bestCell = neighbor;
            }
        }

        if (bestCell == current || !std::isfinite(bestScore)) {
            path.success = false;
            path.exploredCells.clear();
            return path;
        }

        current = bestCell;
        cells.push_back(current);
    }

    if (cells.back() != m_goalCell) {
        path.success = false;
        path.exploredCells.clear();
        return path;
    }

    std::vector<glm::vec2> waypoints;
    waypoints.reserve(cells.size());
    for (const auto& cell : cells) {
        waypoints.emplace_back(static_cast<float>(cell.x) + 0.5f, static_cast<float>(cell.y) + 0.5f);
    }

    path.waypoints = std::move(waypoints);
    path.success = true;
    return path;
}

} // namespace planner
