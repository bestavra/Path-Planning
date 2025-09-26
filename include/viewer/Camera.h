#pragma once

#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

/**
 * @brief Maintains camera state for grid visualization, including projection and input handling.
 */
class Camera {
  public:
    /**
     * @brief Constructs a camera positioned at the supplied world coordinate.
     * @param startPos Initial camera position expressed in world coordinates.
     */
    explicit Camera(glm::vec3 startPos = glm::vec3(0.0f, 0.0f, 15.0f));

    /**
     * @brief Returns the cached view matrix.
     */
    [[nodiscard]] const glm::mat4& getViewMatrix() const {
        return m_viewMatrix;
    }

    /**
     * @brief Returns the cached projection matrix.
     */
    [[nodiscard]] const glm::mat4& getProjectionMatrix() const {
        return m_projectionMatrix;
    }

    /**
     * @brief Computes the combined view-projection matrix.
     */
    [[nodiscard]] glm::mat4 getViewProjectionMatrix() const {
        return m_projectionMatrix * m_viewMatrix;
    }

    /**
     * @brief Returns the camera position in world space.
     */
    [[nodiscard]] const glm::vec3& getPosition() const {
        return m_position;
    }

    /**
     * @brief Returns the forward looking direction of the camera.
     */
    [[nodiscard]] const glm::vec3& getFront() const {
        return m_front;
    }

    /**
     * @brief Processes mouse drag input to pan the camera.
     * @param xpos Current mouse x position in pixels.
     * @param ypos Current mouse y position in pixels.
     * @param middleButtonPressed Indicates whether the middle mouse button is held down.
     */
    void handleMouseMovement(double xpos, double ypos, bool middleButtonPressed);

    /**
     * @brief Processes scroll wheel input to zoom the camera toward a cursor ray.
     * @param yoffset Scroll wheel offset provided by GLFW.
     * @param mouseX Mouse x position in pixels at the time of scrolling.
     * @param mouseY Mouse y position in pixels at the time of scrolling.
     */
    void handleScroll(double yoffset, double mouseX, double mouseY);

    /**
     * @brief Sets the camera position explicitly.
     * @param newPos Desired camera position in world coordinates.
     */
    void setPosition(const glm::vec3& newPos);

    /**
     * @brief Updates the aspect ratio and recomputes the projection matrix.
     * @param ratio New viewport aspect ratio (width divided by height).
     */
    void setAspectRatio(float ratio);

    /**
     * @brief Adjusts the field of view in degrees.
     * @param newFov Field of view in degrees.
     */
    void setFOV(float newFov);

    /**
     * @brief Updates the viewport dimensions used for ray calculations.
     * @param width Width of the viewport in pixels.
     * @param height Height of the viewport in pixels.
     */
    void setViewportSize(float width, float height);

    /**
     * @brief Returns the current viewport width in pixels.
     */
    [[nodiscard]] float getViewportWidth() const {
        return m_viewportWidth;
    }

    /**
     * @brief Returns the current viewport height in pixels.
     */
    [[nodiscard]] float getViewportHeight() const {
        return m_viewportHeight;
    }

    /**
     * @brief Calculates the number of pixels covered by a single world unit at the current zoom.
     */
    [[nodiscard]] float getPixelsPerUnit() const;

    /**
     * @brief Resets mouse-tracking state to avoid sudden jumps on the next move event.
     */
    void resetMouseTracking();

    /**
     * @brief Optional global pointer for legacy access patterns.
     */
    static Camera* s_instance;

  private:
    void updateViewMatrix();
    void updateProjectionMatrix();

    glm::vec3 m_position;
    glm::vec3 m_front;
    glm::vec3 m_right;
    glm::vec3 m_up;

    glm::mat4 m_viewMatrix;
    glm::mat4 m_projectionMatrix;

    float m_fieldOfView;
    float m_nearPlane;
    float m_farPlane;
    float m_aspectRatio;
    float m_viewportWidth;
    float m_viewportHeight;

    float m_lastCursorX;
    float m_lastCursorY;
    bool m_isFirstMouseMove;
    float m_scrollSpeed;
};