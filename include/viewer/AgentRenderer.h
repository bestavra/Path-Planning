#pragma once

#include <glm/glm.hpp>

/**
 * @brief Renders the agent body and observation footprint as simple quads.
 */
class AgentRenderer {
  public:
    AgentRenderer();
    ~AgentRenderer();

    AgentRenderer(const AgentRenderer&) = delete;
    AgentRenderer& operator=(const AgentRenderer&) = delete;
    AgentRenderer(AgentRenderer&&) = delete;
    AgentRenderer& operator=(AgentRenderer&&) = delete;

    /**
     * @brief Updates the camera matrices used for all subsequent draws.
     * @param viewProjectionMatrix Combined view-projection matrix supplied by the active camera.
     */
    void setCamera(const glm::mat4& viewProjectionMatrix);

    /**
     * @brief Draws an opaque circle representing the agent footprint.
     * @param center Agent center position in world coordinates.
     * @param color RGB color used for the agent body.
     * @param radius Radius of the agent circle in world units.
     */
    void drawAgent(const glm::vec2& center, const glm::vec3& color, float radius) const;

    /**
     * @brief Draws a translucent observation disk around the agent.
     * @param center Agent center position in world coordinates.
     * @param radius Observation radius in world units.
     * @param color RGBA color applied to the observation area.
     */
    void drawObservationArea(const glm::vec2& center, float radius, const glm::vec4& color) const;

  private:
    /**
     * @brief Helper for drawing filled circles using the current shader.
     * @param center Circle center in world coordinates.
     * @param radius Circle radius in world units.
     * @param color RGBA color to apply.
     */
    void drawCircle(const glm::vec2& center, float radius, const glm::vec4& color) const;

    unsigned int m_shaderProgram = 0;
    unsigned int m_vertexBuffer = 0;
    unsigned int m_vertexArray = 0;
    glm::mat4 m_viewProjection{1.0f};
};
