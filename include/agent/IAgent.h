#pragma once

#include "path/PlannerTypes.h"

#include <glm/glm.hpp>
#include <vector>

class AgentRenderer;

namespace agent {

/**
 * @brief Abstract interface for agents that traverse planned paths and render their state.
 */
class IAgent {
  public:
    virtual ~IAgent() = default;

    /**
     * @brief Binds the renderer responsible for drawing the agent.
     * @param renderer Renderer that will visualize the agent; pass nullptr to detach rendering.
     */
    virtual void attachRenderer(AgentRenderer* renderer) = 0;

    /**
     * @brief Updates the renderer with the latest view-projection matrix.
     * @param viewProjectionMatrix Combined view-projection transform supplied by the active camera.
     */
    virtual void setViewProjection(const glm::mat4& viewProjectionMatrix) = 0;

    /**
     * @brief Supplies a new path for the agent to follow.
     * @param path Planned path that should be traversed by the agent.
     */
    virtual void onNewPath(const planner::PlannedPath& path) = 0;

    /**
     * @brief Begins traversal of the current path.
     */
    virtual void start() = 0;

    /**
     * @brief Resets traversal progress and clears internal state.
     */
    virtual void reset() = 0;

    /**
     * @brief Indicates whether the agent currently has a path.
     */
    virtual bool hasPath() const = 0;

    /**
     * @brief Returns true while the agent is actively moving along a path.
     */
    virtual bool isActive() const = 0;

    /**
     * @brief Advances internal state by the specified delta time.
     * @param deltaSeconds Elapsed simulation time in seconds since the previous update.
     */
    virtual void update(float deltaSeconds) = 0;

    /**
     * @brief Renders the agent using the attached renderer.
     */
    virtual void draw() const = 0;

    /**
     * @brief Returns the current agent position in map coordinates.
     */
    virtual glm::vec2 getCurrentPosition() const = 0;

    /**
     * @brief Returns the radius of the observation area in map cells.
     */
    virtual float getObservationRadius() const = 0;

    /**
     * @brief Reports traversal progress in the [0, 1] range.
     */
    virtual float getPathTraversalRatio() const = 0;

    /**
     * @brief Returns the polyline representing the travelled portion of the path.
     */
    virtual std::vector<glm::vec2> getTravelledPolyline() const = 0;

    /**
     * @brief Provides the agent footprint radius expressed in cells.
     */
    virtual float getFootprintRadiusCells() const = 0;
};

} // namespace agent
