#pragma once

#include <cstddef>
#include <glm/glm.hpp>
#include <vector>

/**
 * @brief Generates the grid line overlay using dynamic vertex buffers.
 */
class LineRenderer {
  public:
    LineRenderer();
    ~LineRenderer();

    /**
     * @brief Adds an arbitrary line segment to the current buffer.
     * @param start Starting vertex of the line in world coordinates.
     * @param end Ending vertex of the line in world coordinates.
     */
    void addLine(const glm::vec3& start, const glm::vec3& end);

    /**
     * @brief Updates the default grid dimensions and rebuilds the vertex data.
     * @param newWidth Grid width expressed in cells.
     * @param newHeight Grid height expressed in cells.
     */
    void setDimensions(std::size_t newWidth, std::size_t newHeight);

    /**
     * @brief Returns the current grid width.
     */
    [[nodiscard]] std::size_t getWidth() const noexcept {
        return m_width;
    }

    /**
     * @brief Returns the current grid height.
     */
    [[nodiscard]] std::size_t getHeight() const noexcept {
        return m_height;
    }

    /**
     * @brief Updates the camera matrices used during rendering.
     * @param viewProjectionMatrix Combined view-projection matrix supplied by the active camera.
     */
    void setCamera(const glm::mat4& viewProjectionMatrix);

    /**
     * @brief Issues draw commands for the buffered lines.
     */
    void draw();

    /**
     * @brief Clears all stored vertices.
     */
    void clear();

  private:
    /**
     * @brief Uploads vertex data to GPU buffers.
     */
    void upload() const;

    unsigned int m_shaderProgram;
    unsigned int m_vertexBuffer;
    unsigned int m_vertexArray;
    std::vector<float> m_vertices;
    glm::mat4 m_viewProjection;
    std::size_t m_width;
    std::size_t m_height;
};