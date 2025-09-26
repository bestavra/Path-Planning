#include "agent/SimpleAStarAgent.h"

#include "viewer/AgentRenderer.h"

#include <algorithm>
#include <glm/glm.hpp>

namespace {
constexpr glm::vec3 s_agentColor{1.0f, 0.0f, 1.0f};
}

namespace agent {

SimpleAStarAgent::SimpleAStarAgent(float speedCellsPerSecond) : m_speed(speedCellsPerSecond) {
    recalculateRadii();
}

void SimpleAStarAgent::attachRenderer(AgentRenderer* rendererPtr) {
    m_renderer = rendererPtr;
    if (m_renderer) {
        m_renderer->setCamera(m_cachedViewProjection);
    }
}

void SimpleAStarAgent::setViewProjection(const glm::mat4& viewProjectionMatrix) {
    m_cachedViewProjection = viewProjectionMatrix;
    if (m_renderer) {
        m_renderer->setCamera(viewProjectionMatrix);
    }
}

void SimpleAStarAgent::onNewPath(const planner::PlannedPath& path) {
    if (!path.success || path.waypoints.size() < 2) {
        reset();
        return;
    }

    m_waypoints = path.waypoints;
    recomputeSegments();
    m_currentPosition = m_waypoints.front();
    m_distanceTravelled = 0.0f;
    m_pathAvailable = true;
    start();
}

void SimpleAStarAgent::start() {
    if (!m_pathAvailable || m_segmentLengths.empty()) {
        return;
    }
    m_distanceTravelled = 0.0f;
    m_currentPosition = m_waypoints.front();
    m_playing = true;
}

void SimpleAStarAgent::reset() {
    m_waypoints.clear();
    m_segmentLengths.clear();
    m_totalLength = 0.0f;
    m_distanceTravelled = 0.0f;
    m_pathAvailable = false;
    m_playing = false;
}

bool SimpleAStarAgent::hasPath() const {
    return m_pathAvailable;
}

bool SimpleAStarAgent::isActive() const {
    return m_playing;
}

void SimpleAStarAgent::update(float deltaSeconds) {
    if (!m_playing) {
        return;
    }

    if (m_segmentLengths.empty() || m_totalLength <= 0.0f) {
        m_playing = false;
        return;
    }

    m_distanceTravelled = std::min(m_distanceTravelled + m_speed * deltaSeconds, m_totalLength);
    updateCurrentPosition();

    if (m_distanceTravelled >= m_totalLength) {
        m_playing = false;
    }
}

void SimpleAStarAgent::draw() const {
    if (!m_renderer || !m_pathAvailable) {
        return;
    }

    const bool shouldRender = m_playing || m_distanceTravelled >= m_totalLength;
    if (!shouldRender) {
        return;
    }

    m_renderer->drawObservationArea(m_currentPosition, m_observationRadiusCells, m_observationColor);
    m_renderer->drawAgent(m_currentPosition, s_agentColor, m_footprintRadiusCells);
}

void SimpleAStarAgent::recomputeSegments() {
    m_segmentLengths.clear();
    m_segmentLengths.reserve(m_waypoints.size() > 1 ? m_waypoints.size() - 1 : 0);
    m_totalLength = 0.0f;

    for (std::size_t i = 0; i + 1 < m_waypoints.size(); ++i) {
        const float length = glm::distance(m_waypoints[i + 1], m_waypoints[i]);
        m_segmentLengths.push_back(length);
        m_totalLength += length;
    }
}

void SimpleAStarAgent::updateCurrentPosition() {
    if (m_segmentLengths.empty()) {
        m_currentPosition = m_waypoints.empty() ? glm::vec2(0.0f) : m_waypoints.back();
        return;
    }

    float remaining = m_distanceTravelled;

    for (std::size_t i = 0; i < m_segmentLengths.size(); ++i) {
        const float segmentLength = m_segmentLengths[i];
        if (segmentLength <= 0.0f) {
            continue;
        }

        if (remaining > segmentLength) {
            remaining -= segmentLength;
            continue;
        }

        const float t = std::clamp(remaining / segmentLength, 0.0f, 1.0f);
        m_currentPosition = m_waypoints[i] + t * (m_waypoints[i + 1] - m_waypoints[i]);
        return;
    }

    m_currentPosition = m_waypoints.back();
}

glm::vec2 SimpleAStarAgent::getCurrentPosition() const {
    return m_currentPosition;
}

float SimpleAStarAgent::getObservationRadius() const {
    return m_observationRadiusCells;
}

float SimpleAStarAgent::getPathTraversalRatio() const {
    if (!m_pathAvailable || m_totalLength <= 1e-6f) {
        return 0.0f;
    }

    const float ratio = m_distanceTravelled / std::max(m_totalLength, 1e-6f);
    return std::clamp(ratio, 0.0f, 1.0f);
}

std::vector<glm::vec2> SimpleAStarAgent::getTravelledPolyline() const {
    std::vector<glm::vec2> history;
    if (!m_pathAvailable || m_waypoints.empty()) {
        return history;
    }

    history.push_back(m_waypoints.front());

    if (m_segmentLengths.empty() || m_distanceTravelled <= 0.0f) {
        history.push_back(m_currentPosition);
        return history;
    }

    float remaining = std::clamp(m_distanceTravelled, 0.0f, m_totalLength);

    for (std::size_t i = 0; i < m_segmentLengths.size(); ++i) {
        const float segLen = m_segmentLengths[i];
        if (segLen <= 1e-6f) {
            history.push_back(m_waypoints[i + 1]);
            continue;
        }

        if (remaining >= segLen) {
            history.push_back(m_waypoints[i + 1]);
            remaining -= segLen;
            continue;
        }

        const float t = std::clamp(remaining / segLen, 0.0f, 1.0f);
        const glm::vec2 interpolated = m_waypoints[i] + t * (m_waypoints[i + 1] - m_waypoints[i]);
        history.push_back(interpolated);
        remaining = 0.0f;
        break;
    }

    if (remaining > 0.0f && m_waypoints.size() >= 2) {
        history.push_back(m_waypoints.back());
    }

    if (history.empty() || history.back() != m_currentPosition) {
        history.push_back(m_currentPosition);
    }

    return history;
}

void SimpleAStarAgent::configurePhysicalSize(float diameterMeters, float resolutionMetersPerCell) {
    m_agentDiameterMeters = std::max(0.0f, diameterMeters);
    m_mapResolutionMetersPerCell = std::max(1e-3f, resolutionMetersPerCell);
    recalculateRadii();
}

void SimpleAStarAgent::setObservationRangeMeters(float rangeMeters) {
    m_observationRangeMeters = std::max(rangeMeters, 0.0f);
    recalculateRadii();
}

void SimpleAStarAgent::recalculateRadii() {
    const float radiusMeters = m_agentDiameterMeters * 0.5f;
    const float radiusCells = radiusMeters / std::max(m_mapResolutionMetersPerCell, 1e-3f);
    m_footprintRadiusCells = std::max(radiusCells, 0.0f);
    const float observationMeters = std::max(m_observationRangeMeters, radiusMeters);
    m_observationRadiusCells =
        std::max(observationMeters / std::max(m_mapResolutionMetersPerCell, 1e-3f), m_footprintRadiusCells);
}

} // namespace agent
