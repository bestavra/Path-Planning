#pragma once

#include <glm/glm.hpp>
#include <vector>

/**
 * @brief Renders explored planner cells as instanced quads with shared shader state.
 */
class ExploredCellsRenderer {
  public:
    ExploredCellsRenderer();
    ~ExploredCellsRenderer();

    /**
     * @brief Updates the camera matrices applied to future draw calls.
     * @param viewProjectionMatrix Combined view-projection matrix supplied by the active camera.
     */
    void setCamera(const glm::mat4& viewProjectionMatrix);

    /**
     * @brief Adjusts the overlay color used for all instances.
     * @param colorValue RGBA color to apply to the explored cells overlay.
     */
    void setColor(const glm::vec4& colorValue) {
        m_color = colorValue;
    }

    /**
     * @brief Uploads the centers of the explored cells for instanced rendering.
     * @param centers Collection of explored cell centers in world coordinates.
     */
    void setCells(const std::vector<glm::vec2>& centers);

    /**
     * @brief Clears all uploaded instances.
     */
    void clear();

    /**
     * @brief Draws the explored cell quads when any instances are present.
     */
    void draw() const;

  private:
    unsigned int m_shaderProgram = 0;
    unsigned int m_vertexArray = 0;
    unsigned int m_quadVertexBuffer = 0;
    unsigned int m_instanceVertexBuffer = 0;

    glm::mat4 m_viewProjection{1.0f};
    glm::vec4 m_color{1.0f, 0.85f, 0.2f, 0.35f};
    std::size_t m_instanceCount = 0;
};
