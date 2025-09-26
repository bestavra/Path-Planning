#pragma once

#include "agent/IAgent.h"
#include "viewer/Constants.h"

#include <algorithm>
#include <vector>

namespace agent {

/**
 * @brief Minimal path-following agent that animates A* results.
 */
class SimpleAStarAgent : public IAgent {
  public:
    explicit SimpleAStarAgent(float speedCellsPerSecond = 4.0f);

    /**
     * @brief Associates an optional renderer used for visualization.
     * @param renderer Renderer that will draw the agent, or nullptr to detach rendering support.
     */
    void attachRenderer(AgentRenderer* renderer) override;

    /**
     * @brief Updates the cached view-projection matrix for rendering helpers.
     * @param viewProjectionMatrix Combined view-projection transform provided by the active camera.
     */
    void setViewProjection(const glm::mat4& viewProjectionMatrix) override;

    /**
     * @brief Responds to a new planned path and resets traversal state.
     * @param path Planned path that the agent should begin following.
     */
    void onNewPath(const planner::PlannedPath& path) override;

    /**
     * @brief Starts animating along the current path.
     */
    void start() override;

    /**
     * @brief Resets traversal progress and clears the current path.
     */
    void reset() override;

    [[nodiscard]] bool hasPath() const override;
    [[nodiscard]] bool isActive() const override;

    /**
     * @brief Advances the traversal based on elapsed time.
     * @param deltaSeconds Elapsed simulation time in seconds since the previous update.
     */
    void update(float deltaSeconds) override;

    /**
     * @brief Delegates drawing to the attached renderer when active.
     */
    void draw() const override;

    /**
     * @brief Adjusts the agent physical footprint using map resolution.
     * @param diameterMeters Agent diameter expressed in meters.
     * @param mapResolutionMetersPerCell Map resolution measured in meters per grid cell.
     */
    void configurePhysicalSize(float diameterMeters, float mapResolutionMetersPerCell);

    /**
     * @brief Sets the observation range in meters used for visualization.
     * @param rangeMeters Observation radius in meters around the agent.
     */
    void setObservationRangeMeters(float rangeMeters);

    [[nodiscard]] glm::vec2 getCurrentPosition() const override;
    [[nodiscard]] float getObservationRadius() const override;
    [[nodiscard]] float getPathTraversalRatio() const override;
    [[nodiscard]] std::vector<glm::vec2> getTravelledPolyline() const override;

    [[nodiscard]] float getFootprintRadiusCells() const override {
        return m_footprintRadiusCells;
    }

  private:
    AgentRenderer* m_renderer = nullptr;
    glm::mat4 m_cachedViewProjection{1.0f};

    std::vector<glm::vec2> m_waypoints;
    std::vector<float> m_segmentLengths;
    float m_totalLength = 0.0f;
    float m_distanceTravelled = 0.0f;
    float m_speed = 4.0f;
    bool m_pathAvailable = false;
    bool m_playing = false;
    glm::vec2 m_currentPosition{0.0f, 0.0f};
    float m_agentDiameterMeters = DEFAULT_AGENT_DIAMETER_METERS;
    float m_mapResolutionMetersPerCell = 1.0f;
    float m_footprintRadiusCells = 0.5f;
    float m_observationRadiusCells = 0.5f;
    float m_observationRangeMeters = DEFAULT_AGENT_OBSERVATION_RADIUS_METERS;
    glm::vec4 m_observationColor{0.4f, 0.4f, 0.4f, 0.35f};

    void recomputeSegments();
    void updateCurrentPosition();
    void recalculateRadii();
};

} // namespace agent
