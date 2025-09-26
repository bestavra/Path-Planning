#include "viewer/Grid.h"

#include "map/Map.h"
#include "viewer/Constants.h"

#include <algorithm>
#include <cmath>
#include <optional>

namespace {

const glm::vec3 s_visibleObstacleColor{0.0f, 0.0f, 0.0f};
const glm::vec3 s_hiddenObstacleColor{0.1f, 0.4f, 0.9f};
constexpr float s_historyPointEpsilon = 1e-3f;

glm::vec3 baseColorForCell(const map::Grid& mapData, const glm::ivec2& cell) {
    if (cell.x < 0 || cell.y < 0) {
        return glm::vec3(0.0f);
    }

    const auto& metadata = mapData.metadata();
    if (cell.x >= static_cast<int>(metadata.width) || cell.y >= static_cast<int>(metadata.height)) {
        return glm::vec3(0.0f);
    }

    const std::size_t ux = static_cast<std::size_t>(cell.x);
    const std::size_t uy = static_cast<std::size_t>(cell.y);

    const bool missing = mapData.isMissing(ux, uy);
    const float value = missing ? map::Grid::s_missingData : mapData.at(ux, uy);
    return Grid::colorForValue(value);
}

bool pointsApproximatelyEqual(const glm::vec2& a, const glm::vec2& b) {
    return glm::length(a - b) <= s_historyPointEpsilon;
}

bool circleTouchesObstacleCorner(const glm::vec2& circleCenter, float radiusSquared, const glm::ivec2& cell) {
    const float left = static_cast<float>(cell.x);
    const float bottom = static_cast<float>(cell.y);
    const float right = left + 1.0f;
    const float top = bottom + 1.0f;

    if (circleCenter.x >= left && circleCenter.x <= right && circleCenter.y >= bottom && circleCenter.y <= top) {
        return true;
    }

    const glm::vec2 corners[] = {{left, bottom}, {right, bottom}, {left, top}, {right, top}};

    for (const auto& corner : corners) {
        const glm::vec2 diff = corner - circleCenter;
        const float distSquared = glm::dot(diff, diff);
        if (distSquared <= radiusSquared) {
            return true;
        }
    }

    return false;
}

} // namespace

Grid::Grid() : m_width(GRID_WIDTH), m_height(GRID_HEIGHT) {
    m_lineRenderer.setDimensions(m_width, m_height);
    m_quadRenderer.resize(m_width, m_height);
    m_pathRenderer.clear();
    m_exploredRenderer.setColor(glm::vec4(1.0f, 0.85f, 0.2f, 0.35f));
    m_exploredRenderer.clear();
    m_inflationRenderer.setColor(glm::vec4(0.0f, 1.0f, 1.0f, 0.35f));
    m_inflationRenderer.clear();
}

void Grid::resize(std::size_t newWidth, std::size_t newHeight) {
    m_width = newWidth;
    m_height = newHeight;

    m_lineRenderer.setDimensions(m_width, m_height);
    m_quadRenderer.resize(m_width, m_height);
    m_quadRenderer.update();
}

void Grid::addCell(const glm::vec2& gridPos, const glm::vec3& color, bool updateImmediately) {
    // Ignore clicks outside the grid
    if (m_width == 0 || m_height == 0) {
        return;
    }
    if (gridPos.x < 0 || gridPos.x > static_cast<float>(m_width == 0 ? 0 : m_width - 1) || gridPos.y < 0 ||
        gridPos.y > static_cast<float>(m_height == 0 ? 0 : m_height - 1)) {
        return;
    }

    m_quadRenderer.addQuad(gridPos, color);
    if (updateImmediately) {
        m_quadRenderer.update();
    }
}

void Grid::removeCell(const glm::vec2& gridPos, bool updateImmediately) {
    // Ignore clicks outside the grid
    if (m_width == 0 || m_height == 0) {
        return;
    }
    if (gridPos.x < 0 || gridPos.x > static_cast<float>(m_width == 0 ? 0 : m_width - 1) || gridPos.y < 0 ||
        gridPos.y > static_cast<float>(m_height == 0 ? 0 : m_height - 1)) {
        return;
    }

    m_quadRenderer.removeQuad(gridPos);
    if (updateImmediately) {
        m_quadRenderer.update();
    }
}

void Grid::setCamera(const glm::mat4& viewProjectionMatrix) {
    m_lastViewProjection = viewProjectionMatrix;
    m_lineRenderer.setCamera(viewProjectionMatrix);
    m_quadRenderer.setCamera(viewProjectionMatrix);
    m_pathRenderer.setCamera(viewProjectionMatrix);
    m_agentRenderer.setCamera(viewProjectionMatrix);
    m_exploredRenderer.setCamera(viewProjectionMatrix);
    m_inflationRenderer.setCamera(viewProjectionMatrix);
    if (m_agent) {
        m_agent->setViewProjection(viewProjectionMatrix);
    }
}

void Grid::calculateFrustum(const glm::mat4& projection, const glm::mat4& view, const glm::vec3& cameraPos,
    float viewportWidth, float viewportHeight) {
    m_quadRenderer.calculateFrustum(projection, view, cameraPos, viewportWidth, viewportHeight);
}

void Grid::setAgent(agent::IAgent* agentPtr) {
    m_agent = agentPtr;
    if (m_agent) {
        m_agent->attachRenderer(&m_agentRenderer);
        m_agent->setViewProjection(m_lastViewProjection);
        if (m_latestPath.has_value() && m_latestPath->success && m_latestPath->waypoints.size() >= 2) {
            m_agent->onNewPath(*m_latestPath);
        }
    }
}

void Grid::tick(float deltaSeconds) {
    if (m_agent) {
        m_agent->update(deltaSeconds);
        const float ratio = m_agent->hasPath() ? m_agent->getPathTraversalRatio() : 0.0f;
        m_pathRenderer.setTravelProgress(ratio);
    } else {
        m_pathRenderer.setTravelProgress(0.0f);
    }
}

void Grid::update() {
    m_quadRenderer.update();
}

void Grid::draw() {
    m_quadRenderer.draw();
    if (m_gridLinesVisible) {
        m_lineRenderer.draw();
    }
    m_inflationRenderer.draw();
    m_exploredRenderer.draw();
    m_pathRenderer.draw();
    if (m_agent) {
        m_agent->draw();
    }
}

void Grid::clear() {
    m_quadRenderer.clear();
    m_lineRenderer.setDimensions(m_width, m_height);
    clearPath();
    clearInflationOverlay();
    m_startCell = glm::ivec2(-1, -1);
    m_goalCell = glm::ivec2(-1, -1);
    m_visibleDynamicObstacles.clear();
    m_hiddenDynamicObstacles.clear();
}

void Grid::paint(const map::Grid& mapData) {
    m_quadRenderer.clear();

    if (m_width == 0 || m_height == 0) {
        return;
    }

    const auto& metadata = mapData.metadata();
    const std::size_t maxWidth = std::min<std::size_t>(metadata.width, m_width);
    const std::size_t maxHeight = std::min<std::size_t>(metadata.height, m_height);

    for (std::size_t y = 0; y < maxHeight; ++y) {
        for (std::size_t x = 0; x < maxWidth; ++x) {
            const float value = mapData.isMissing(x, y) ? map::Grid::s_missingData : mapData.at(x, y);
            const glm::vec3 color = colorForValue(value);
            m_quadRenderer.addQuad(glm::vec2(static_cast<float>(x), static_cast<float>(y)), color);
        }
    }

    m_quadRenderer.update();

    if (containsCell(m_startCell)) {
        setCellColor(m_startCell, glm::vec3(0.0f, 1.0f, 0.0f));
    }
    if (containsCell(m_goalCell)) {
        setCellColor(m_goalCell, glm::vec3(1.0f, 0.0f, 0.0f));
    }

    repaintDynamicObstacles();
}

void Grid::setCellColor(const glm::ivec2& cell, const glm::vec3& color) {
    if (!containsCell(cell)) {
        return;
    }

    m_quadRenderer.addQuad(glm::vec2(static_cast<float>(cell.x), static_cast<float>(cell.y)), color);
    m_quadRenderer.update();
}

void Grid::setPath(const planner::PlannedPath& path, const std::vector<glm::vec2>& history) {
    m_latestPath = path;
    setExploredCells(path.exploredCells);

    if (!history.empty()) {
        appendTravelHistory(history);
    } else if (!m_travelHistory.empty() && !path.waypoints.empty()) {
        const glm::vec2 newStart = path.waypoints.front();
        if (!pointsApproximatelyEqual(m_travelHistory.back(), newStart)) {
            m_travelHistory.clear();
        }
    }

    if (!path.success || path.waypoints.size() < 2) {
        m_pathRenderer.clear();
        if (!m_travelHistory.empty()) {
            m_pathRenderer.setHistory(m_travelHistory);
        }
        if (m_agent) {
            m_agent->reset();
        }
        return;
    }

    m_pathRenderer.setPath(path, glm::vec3(1.0f, 0.0f, 0.0f));
    if (m_travelHistory.empty()) {
        m_pathRenderer.clearHistory();
    } else {
        m_pathRenderer.setHistory(m_travelHistory);
    }

    if (m_agent) {
        m_agent->onNewPath(path);
    }
}

void Grid::clearPath() {
    m_pathRenderer.clear();
    m_travelHistory.clear();
    m_latestPath.reset();
    clearExploredCells();
    if (m_agent) {
        m_agent->reset();
    }
}

void Grid::setStartMarker(const glm::ivec2& cell, const map::Grid& mapData) {
    if (!containsCell(cell)) {
        return;
    }

    if (containsCell(m_startCell)) {
        if (isDynamicObstacle(m_startCell)) {
            setCellColorForObstacle(m_startCell);
        } else {
            setCellColor(m_startCell, baseColorForCell(mapData, m_startCell));
        }
    }

    m_startCell = cell;
    setCellColor(m_startCell, glm::vec3(0.0f, 1.0f, 0.0f));
}

void Grid::setGoalMarker(const glm::ivec2& cell, const map::Grid& mapData) {
    if (!containsCell(cell)) {
        return;
    }

    if (containsCell(m_goalCell)) {
        if (isDynamicObstacle(m_goalCell)) {
            setCellColorForObstacle(m_goalCell);
        } else {
            setCellColor(m_goalCell, baseColorForCell(mapData, m_goalCell));
        }
    }

    m_goalCell = cell;
    setCellColor(m_goalCell, glm::vec3(1.0f, 0.0f, 0.0f));
}

void Grid::clearStartMarker(const map::Grid& mapData) {
    if (!containsCell(m_startCell)) {
        return;
    }

    if (isDynamicObstacle(m_startCell)) {
        setCellColorForObstacle(m_startCell);
    } else {
        setCellColor(m_startCell, baseColorForCell(mapData, m_startCell));
    }
    m_startCell = glm::ivec2(-1, -1);
}

void Grid::clearGoalMarker(const map::Grid& mapData) {
    if (!containsCell(m_goalCell)) {
        return;
    }

    if (isDynamicObstacle(m_goalCell)) {
        setCellColorForObstacle(m_goalCell);
    } else {
        setCellColor(m_goalCell, baseColorForCell(mapData, m_goalCell));
    }
    m_goalCell = glm::ivec2(-1, -1);
}

glm::vec3 Grid::colorForValue(float value) {
    if (value <= map::Grid::s_missingData) {
        return glm::vec3(0.0f, 0.0f, 0.5f);
    }

    const float clamped = std::clamp(value, 0.0f, 1.0f);
    const float intensity = 1.0f - clamped;
    return glm::vec3(intensity);
}

bool Grid::containsCell(const glm::ivec2& cell) const noexcept {
    return cell.x >= 0 && cell.y >= 0 && cell.x < static_cast<int>(m_width) && cell.y < static_cast<int>(m_height);
}

std::optional<glm::ivec2> Grid::getStartCell() const noexcept {
    if (containsCell(m_startCell)) {
        return m_startCell;
    }
    return std::nullopt;
}

std::optional<glm::ivec2> Grid::getGoalCell() const noexcept {
    if (containsCell(m_goalCell)) {
        return m_goalCell;
    }
    return std::nullopt;
}

void Grid::setExploredCells(const std::vector<glm::ivec2>& cells) {
    if (cells.empty()) {
        m_exploredRenderer.clear();
        return;
    }

    std::vector<glm::vec2> centers;
    centers.reserve(cells.size());
    for (const auto& cell : cells) {
        if (!containsCell(cell)) {
            continue;
        }
        centers.emplace_back(static_cast<float>(cell.x) + 0.5f, static_cast<float>(cell.y) + 0.5f);
    }
    m_exploredRenderer.setCells(centers);
}

void Grid::clearExploredCells() {
    m_exploredRenderer.clear();
}

bool Grid::addDynamicObstacle(const glm::ivec2& cell, ObstacleVisibility visibility) {
    if (!containsCell(cell)) {
        return false;
    }
    if (cell == m_startCell || cell == m_goalCell) {
        return false;
    }
    if (m_visibleDynamicObstacles.count(cell) > 0 || m_hiddenDynamicObstacles.count(cell) > 0) {
        return false;
    }

    if (visibility == ObstacleVisibility::Visible) {
        m_visibleDynamicObstacles.insert(cell);
    } else {
        m_hiddenDynamicObstacles.insert(cell);
    }

    setCellColorForObstacle(cell);
    return true;
}

bool Grid::revealDynamicObstacle(const glm::ivec2& cell) {
    if (!containsCell(cell)) {
        return false;
    }

    const auto it = m_hiddenDynamicObstacles.find(cell);
    if (it == m_hiddenDynamicObstacles.end()) {
        return false;
    }

    m_hiddenDynamicObstacles.erase(it);
    m_visibleDynamicObstacles.insert(cell);
    setCellColorForObstacle(cell);
    return true;
}

bool Grid::removeDynamicObstacle(const glm::ivec2& cell, const map::Grid& mapData) {
    if (!containsCell(cell)) {
        return false;
    }

    bool removed = false;
    if (m_visibleDynamicObstacles.erase(cell) > 0) {
        removed = true;
    } else if (m_hiddenDynamicObstacles.erase(cell) > 0) {
        removed = true;
    }

    if (!removed) {
        return false;
    }

    if (cell == m_startCell) {
        setCellColor(cell, glm::vec3(0.0f, 1.0f, 0.0f));
    } else if (cell == m_goalCell) {
        setCellColor(cell, glm::vec3(1.0f, 0.0f, 0.0f));
    } else {
        setCellColor(cell, baseColorForCell(mapData, cell));
    }

    return true;
}

void Grid::clearDynamicObstacles(const map::Grid& mapData) {
    if (m_visibleDynamicObstacles.empty() && m_hiddenDynamicObstacles.empty()) {
        return;
    }

    const auto restoreColor = [&](const glm::ivec2& cell) {
        if (!containsCell(cell)) {
            return;
        }

        if (cell == m_startCell) {
            setCellColor(cell, glm::vec3(0.0f, 1.0f, 0.0f));
        } else if (cell == m_goalCell) {
            setCellColor(cell, glm::vec3(1.0f, 0.0f, 0.0f));
        } else {
            setCellColor(cell, baseColorForCell(mapData, cell));
        }
    };

    for (const auto& cell : m_visibleDynamicObstacles) {
        restoreColor(cell);
    }
    for (const auto& cell : m_hiddenDynamicObstacles) {
        restoreColor(cell);
    }

    m_visibleDynamicObstacles.clear();
    m_hiddenDynamicObstacles.clear();
}

bool Grid::isDynamicObstacle(const glm::ivec2& cell) const noexcept {
    return isVisibleDynamicObstacle(cell) || isHiddenDynamicObstacle(cell);
}

bool Grid::isVisibleDynamicObstacle(const glm::ivec2& cell) const noexcept {
    return m_visibleDynamicObstacles.find(cell) != m_visibleDynamicObstacles.end();
}

bool Grid::isHiddenDynamicObstacle(const glm::ivec2& cell) const noexcept {
    return m_hiddenDynamicObstacles.find(cell) != m_hiddenDynamicObstacles.end();
}

bool Grid::hasDynamicObstacles() const noexcept {
    return !m_visibleDynamicObstacles.empty() || !m_hiddenDynamicObstacles.empty();
}

std::vector<glm::ivec2> Grid::revealDynamicObstaclesWithinRadius(const glm::vec2& center, float radius) {
    std::vector<glm::ivec2> discovered;
    if (m_hiddenDynamicObstacles.empty() || radius <= 0.0f) {
        return discovered;
    }

    const float radiusSquared = radius * radius;
    std::vector<glm::ivec2> toReveal;
    toReveal.reserve(m_hiddenDynamicObstacles.size());

    for (const auto& cell : m_hiddenDynamicObstacles) {
        if (circleTouchesObstacleCorner(center, radiusSquared, cell)) {
            toReveal.push_back(cell);
        }
    }

    for (const auto& cell : toReveal) {
        if (revealDynamicObstacle(cell)) {
            discovered.push_back(cell);
        }
    }

    return discovered;
}

std::optional<Grid::AgentObservation> Grid::getAgentObservation() const {
    if (!m_agent || !m_agent->hasPath()) {
        return std::nullopt;
    }

    return AgentObservation{m_agent->getCurrentPosition(), m_agent->getObservationRadius()};
}

std::vector<glm::vec2> Grid::getAgentTravelHistory() const {
    if (!m_agent) {
        return {};
    }
    return m_agent->getTravelledPolyline();
}

float Grid::getAgentFootprintRadius() const {
    if (!m_agent) {
        return 0.0f;
    }
    return m_agent->getFootprintRadiusCells();
}

void Grid::setInflationOverlay(const std::vector<glm::vec2>& cells) {
    m_inflationCells = cells;
    if (m_inflationCells.empty()) {
        m_inflationRenderer.clear();
    } else {
        m_inflationRenderer.setCells(m_inflationCells);
    }
}

void Grid::clearInflationOverlay() {
    m_inflationCells.clear();
    m_inflationRenderer.clear();
}

void Grid::appendTravelHistory(const std::vector<glm::vec2>& polyline) {
    if (polyline.empty()) {
        return;
    }

    const auto pushIfNew = [this](const glm::vec2& point) {
        if (m_travelHistory.empty() || !pointsApproximatelyEqual(m_travelHistory.back(), point)) {
            m_travelHistory.push_back(point);
        }
    };

    if (m_travelHistory.empty()) {
        for (const auto& point : polyline) {
            pushIfNew(point);
        }
        return;
    }

    const glm::vec2 lastStored = m_travelHistory.back();
    std::size_t overlapIndex = 0;
    for (; overlapIndex < polyline.size(); ++overlapIndex) {
        if (pointsApproximatelyEqual(polyline[overlapIndex], lastStored)) {
            break;
        }
    }

    if (overlapIndex == polyline.size()) {
        m_travelHistory.clear();
        for (const auto& point : polyline) {
            pushIfNew(point);
        }
        return;
    }

    for (std::size_t i = overlapIndex + 1; i < polyline.size(); ++i) {
        pushIfNew(polyline[i]);
    }
}

std::optional<glm::ivec2> Grid::getAgentCurrentCell() const {
    if (!m_agent || !m_agent->hasPath()) {
        return std::nullopt;
    }

    const glm::vec2 position = m_agent->getCurrentPosition();
    const glm::ivec2 cell{static_cast<int>(std::floor(position.x)), static_cast<int>(std::floor(position.y))};

    if (!containsCell(cell)) {
        return std::nullopt;
    }

    return cell;
}

void Grid::repaintDynamicObstacles() {
    if (m_visibleDynamicObstacles.empty() && m_hiddenDynamicObstacles.empty()) {
        return;
    }

    for (const auto& cell : m_visibleDynamicObstacles) {
        if (containsCell(cell)) {
            setCellColor(cell, s_visibleObstacleColor);
        }
    }

    for (const auto& cell : m_hiddenDynamicObstacles) {
        if (containsCell(cell)) {
            setCellColor(cell, s_hiddenObstacleColor);
        }
    }
}

void Grid::setCellColorForObstacle(const glm::ivec2& cell) {
    if (m_visibleDynamicObstacles.count(cell) > 0) {
        setCellColor(cell, s_visibleObstacleColor);
    } else if (m_hiddenDynamicObstacles.count(cell) > 0) {
        setCellColor(cell, s_hiddenObstacleColor);
    }
}