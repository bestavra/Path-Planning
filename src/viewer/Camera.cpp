#include "viewer/Camera.h"

#include "viewer/Constants.h"
#include "viewer/MathUtils.h"

#include <algorithm>

Camera* Camera::s_instance = nullptr;

Camera::Camera(glm::vec3 startPos)
    : m_position(startPos), m_front(glm::vec3(0, 0, -1)), m_right(glm::vec3(1, 0, 0)), m_up(glm::vec3(0, 1, 0)),
      m_fieldOfView(DEFAULT_FOV), m_nearPlane(NEAR_DIST), m_farPlane(FAR_DIST),
      m_aspectRatio(static_cast<float>(SCR_WIDTH) / static_cast<float>(SCR_HEIGHT)),
      m_viewportWidth(static_cast<float>(SCR_WIDTH)), m_viewportHeight(static_cast<float>(SCR_HEIGHT)),
      m_lastCursorX(SCR_WIDTH / 2.0f), m_lastCursorY(SCR_HEIGHT / 2.0f), m_isFirstMouseMove(true), m_scrollSpeed(2.0f) {
    updateViewMatrix();
    updateProjectionMatrix();
    Camera::s_instance = this;
}

void Camera::updateViewMatrix() {
    m_viewMatrix = glm::lookAt(m_position, m_position + m_front, m_up);
}

void Camera::updateProjectionMatrix() {
    m_projectionMatrix = glm::perspective(glm::radians(m_fieldOfView), m_aspectRatio, m_nearPlane, m_farPlane);
}

void Camera::handleMouseMovement(double xpos, double ypos, bool middleButtonPressed) {
    if (m_isFirstMouseMove) {
        m_lastCursorX = static_cast<float>(xpos);
        m_lastCursorY = static_cast<float>(ypos);
        m_isFirstMouseMove = false;
    }

    const float xoffset = static_cast<float>(xpos) - m_lastCursorX;
    const float yoffset = m_lastCursorY - static_cast<float>(ypos);

    m_lastCursorX = static_cast<float>(xpos);
    m_lastCursorY = static_cast<float>(ypos);

    if (middleButtonPressed) {
        // Pan the camera
        const float safeWidth = std::max(m_viewportWidth, 1.0f);
        const float safeHeight = std::max(m_viewportHeight, 1.0f);
        m_position -= m_scrollSpeed * glm::vec3(xoffset / safeWidth, yoffset / safeHeight, 0.0f);
        updateViewMatrix();
    }
}

void Camera::handleScroll(double yoffset, double mouseX, double mouseY) {
    m_scrollSpeed = m_position.z * 0.1f;
    const glm::vec3 rayDirection =
        MathUtils::rayCast(mouseX, mouseY, m_viewportWidth, m_viewportHeight, m_projectionMatrix, m_viewMatrix);
    m_position += static_cast<float>(yoffset) * m_scrollSpeed * rayDirection;
    updateViewMatrix();
}

void Camera::setPosition(const glm::vec3& newPos) {
    m_position = newPos;
    updateViewMatrix();
}

void Camera::setAspectRatio(float ratio) {
    m_aspectRatio = ratio;
    updateProjectionMatrix();
}

void Camera::setFOV(float newFov) {
    m_fieldOfView = std::clamp(newFov, 1.0f, 179.0f);
    updateProjectionMatrix();
}

void Camera::setViewportSize(float width, float height) {
    m_viewportWidth = std::max(width, 1.0f);
    m_viewportHeight = std::max(height, 1.0f);
    m_aspectRatio = m_viewportWidth / m_viewportHeight;
    m_lastCursorX = m_viewportWidth / 2.0f;
    m_lastCursorY = m_viewportHeight / 2.0f;
    m_isFirstMouseMove = true;
    updateProjectionMatrix();
}

float Camera::getPixelsPerUnit() const {
    const float distance = std::max(m_position.z, 1e-3f);
    const float tanHalfFov = std::tan(glm::radians(m_fieldOfView) * 0.5f);
    if (tanHalfFov <= 0.0f) {
        return m_viewportHeight;
    }

    const float verticalWorldSpan = 2.0f * distance * tanHalfFov;
    const float horizontalWorldSpan = verticalWorldSpan * m_aspectRatio;

    const float pixelsPerUnitY = m_viewportHeight / std::max(verticalWorldSpan, 1e-3f);
    const float pixelsPerUnitX = m_viewportWidth / std::max(horizontalWorldSpan, 1e-3f);

    return std::min(pixelsPerUnitX, pixelsPerUnitY);
}

void Camera::resetMouseTracking() {
    m_isFirstMouseMove = true;
}