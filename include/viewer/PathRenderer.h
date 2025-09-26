#pragma once

#include "path/PlannerTypes.h"

#include <glm/glm.hpp>
#include <vector>

/**
 * @brief Renders the active planned path, its travel progress, and the agent history.
 */
class PathRenderer {
  public:
    PathRenderer();
    ~PathRenderer();

    PathRenderer(const PathRenderer&) = delete;
    PathRenderer& operator=(const PathRenderer&) = delete;
    PathRenderer(PathRenderer&&) = delete;
    PathRenderer& operator=(PathRenderer&&) = delete;

    /**
     * @brief Resets all cached geometry and state.
     */
    void clear();

    /**
     * @brief Uploads a new planned path along with the color used for the remaining segment.
     * @param path Planned path data containing waypoints to render.
     * @param color RGB color applied to the remaining portion of the path.
     */
    void setPath(const planner::PlannedPath& path, const glm::vec3& color);

    /**
     * @brief Updates the camera matrices used for rendering.
     * @param viewProjectionMatrix Combined view-projection matrix supplied by the active camera.
     */
    void setCamera(const glm::mat4& viewProjectionMatrix);

    /**
     * @brief Sets the travel progress along the path in the range [0, 1].
     * @param ratio Fraction of the path that has been traversed in the [0, 1] range.
     */
    void setTravelProgress(float ratio);

    /**
     * @brief Stores the agent's historical positions to be rendered as a trail.
     * @param historyPoints Ordered list of previous agent positions in world coordinates.
     */
    void setHistory(const std::vector<glm::vec2>& historyPoints);

    /**
     * @brief Removes any recorded history trail.
     */
    void clearHistory();

    /**
     * @brief Issues draw commands for the current path state.
     */
    void draw() const;

  private:
    /**
     * @brief Uploads path vertex data to GPU buffers.
     */
    void upload() const;

    /**
     * @brief Recomputes travelled and remaining path segments based on current progress.
     */
    void rebuildSegments() const;

    unsigned int m_shaderProgram{0};
    unsigned int m_vertexBuffer{0};
    unsigned int m_vertexArray{0};

    planner::PathStyle m_style{planner::PathStyle::Polyline};
    glm::vec3 m_pathColor{1.0f, 0.0f, 0.0f};
    std::vector<glm::vec3> m_vertices;
    std::vector<float> m_segmentLengths;
    float m_totalLength{0.0f};
    float m_travelRatio{0.0f};
    mutable bool m_segmentsDirty{false};
    mutable std::vector<glm::vec3> m_travelledVertices;
    mutable std::vector<glm::vec3> m_remainingVertices;
    std::vector<glm::vec3> m_historyVertices;
    glm::mat4 m_viewProjection{1.0f};
};
