#pragma once

#include <cstddef>
#include <glm/glm.hpp>
#include <vector>

class QuadRenderer {
  public:
    QuadRenderer();
    ~QuadRenderer();

    /**
     * @brief Resizes the quad grid to the supplied dimensions.
     * @param newWidth Grid width expressed in cells.
     * @param newHeight Grid height expressed in cells.
     */
    void resize(std::size_t newWidth, std::size_t newHeight);

    /**
     * @brief Returns the current grid width in cells.
     */
    [[nodiscard]] std::size_t getWidth() const noexcept {
        return m_width;
    }

    /**
     * @brief Returns the current grid height in cells.
     */
    [[nodiscard]] std::size_t getHeight() const noexcept {
        return m_height;
    }

    /**
     * @brief Adds a quad instance at the specified grid position.
     * @param pos Grid coordinate where the quad should appear.
     * @param color RGB color applied to the quad.
     */
    void addQuad(const glm::vec2& pos, const glm::vec3& color);

    /**
     * @brief Removes a quad instance at the specified grid position.
     * @param pos Grid coordinate to clear.
     */
    void removeQuad(const glm::vec2& pos);

    /**
     * @brief Updates the camera matrices used for rendering.
     * @param viewProjectionMatrix Combined view-projection matrix supplied by the active camera.
     */
    void setCamera(const glm::mat4& viewProjectionMatrix);

    /**
     * @brief Recomputes the visible frustum region to cull quads.
     * @param projection Camera projection matrix.
     * @param view Camera view matrix.
     * @param cameraPos Camera position in world coordinates.
     * @param viewportWidth Width of the viewport in pixels.
     * @param viewportHeight Height of the viewport in pixels.
     */
    void calculateFrustum(const glm::mat4& projection, const glm::mat4& view, const glm::vec3& cameraPos,
        float viewportWidth, float viewportHeight);

    /**
     * @brief Uploads any pending buffer updates to the GPU.
     */
    void update();

    /**
     * @brief Draws all visible quad instances.
     */
    void draw();

    /**
     * @brief Clears all stored quad instances.
     */
    void clear();

  private:
    unsigned int m_shaderProgram = 0;
    unsigned int m_vbo = 0;
    unsigned int m_vao = 0;
    unsigned int m_ebo = 0;
    unsigned int m_offsetBuffer = 0;
    unsigned int m_colorBuffer = 0;

    std::size_t m_activeInstanceCount = 0;
    std::size_t m_width = 0;
    std::size_t m_height = 0;

    std::vector<std::vector<bool>> m_active;
    std::vector<std::vector<glm::vec3>> m_positions;
    std::vector<std::vector<glm::vec3>> m_colors;
    std::vector<glm::vec3> m_instancePositions;
    std::vector<glm::vec3> m_instanceColors;

    glm::mat4 m_viewProjection{};
    glm::vec2 m_bottomLeft{0.0f};
    glm::vec2 m_topRight{-1.0f};
};