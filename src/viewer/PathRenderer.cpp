#include "viewer/PathRenderer.h"

#include "viewer/Constants.h"
#include "viewer/ShaderUtils.h"

#include <algorithm>
#include <cmath>
// clang-format off
#include <glad/gl.h>
#include <GLFW/glfw3.h>
// clang-format on

namespace {
constexpr float kEpsilon = 1e-6f;
}

PathRenderer::PathRenderer() {
    const char* vertexShaderSource = "#version 330 core\n"
                                     "layout (location = 0) in vec3 aPos;\n"
                                     "uniform mat4 viewProjection;\n"
                                     "uniform vec3 uColor;\n"
                                     "out vec3 fragColor;\n"
                                     "void main() {\n"
                                     "    gl_Position = viewProjection * vec4(aPos, 1.0);\n"
                                     "    fragColor = uColor;\n"
                                     "}\n";

    const char* fragmentShaderSource = "#version 330 core\n"
                                       "in vec3 fragColor;\n"
                                       "out vec4 FragColor;\n"
                                       "void main() {\n"
                                       "    FragColor = vec4(fragColor, 1.0);\n"
                                       "}\n";

    m_shaderProgram = ShaderUtils::createShaderProgram(vertexShaderSource, fragmentShaderSource);

    glGenVertexArrays(1, &m_vertexArray);
    glGenBuffers(1, &m_vertexBuffer);

    glBindVertexArray(m_vertexArray);
    glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer);
    glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), reinterpret_cast<void*>(0));
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

PathRenderer::~PathRenderer() {
    if (!glfwGetCurrentContext()) {
        return;
    }

    if (m_vertexArray)
        glDeleteVertexArrays(1, &m_vertexArray);
    if (m_vertexBuffer)
        glDeleteBuffers(1, &m_vertexBuffer);
    if (m_shaderProgram)
        glDeleteProgram(m_shaderProgram);
}

void PathRenderer::clear() {
    m_style = planner::PathStyle::Polyline;
    m_vertices.clear();
    m_segmentLengths.clear();
    m_totalLength = 0.0f;
    m_travelRatio = 0.0f;
    m_travelledVertices.clear();
    m_remainingVertices.clear();
    m_historyVertices.clear();
    m_segmentsDirty = true;
}

void PathRenderer::setPath(const planner::PlannedPath& path, const glm::vec3& color) {
    m_style = path.style;
    m_pathColor = color;

    m_vertices.clear();
    m_segmentLengths.clear();
    m_totalLength = 0.0f;
    m_travelRatio = 0.0f;
    m_travelledVertices.clear();
    m_remainingVertices.clear();

    if (path.style == planner::PathStyle::Polyline) {
        m_vertices.reserve(path.waypoints.size());
        for (const auto& point : path.waypoints) {
            m_vertices.emplace_back(point, 0.0f);
        }

        if (m_vertices.size() >= 2) {
            m_segmentLengths.reserve(m_vertices.size() - 1);
            for (std::size_t i = 0; i + 1 < m_vertices.size(); ++i) {
                const float len = glm::distance(m_vertices[i + 1], m_vertices[i]);
                m_segmentLengths.push_back(len);
                m_totalLength += len;
            }
        }
    }

    m_segmentsDirty = true;
}

void PathRenderer::setCamera(const glm::mat4& viewProjectionMatrix) {
    m_viewProjection = viewProjectionMatrix;
}

void PathRenderer::setTravelProgress(float ratio) {
    const float clamped = std::clamp(ratio, 0.0f, 1.0f);
    if (std::abs(clamped - m_travelRatio) > 1e-4f) {
        m_travelRatio = clamped;
        m_segmentsDirty = true;
    }
}

void PathRenderer::setHistory(const std::vector<glm::vec2>& historyPoints) {
    m_historyVertices.clear();
    if (historyPoints.size() < 2) {
        return;
    }

    m_historyVertices.reserve(historyPoints.size());
    glm::vec2 lastPoint = historyPoints.front();
    m_historyVertices.emplace_back(lastPoint, 0.0f);

    for (std::size_t i = 1; i < historyPoints.size(); ++i) {
        const glm::vec2& point = historyPoints[i];
        if (glm::length(point - lastPoint) <= kEpsilon) {
            continue;
        }
        m_historyVertices.emplace_back(point, 0.0f);
        lastPoint = point;
    }

    if (m_historyVertices.size() < 2) {
        m_historyVertices.clear();
    }
}

void PathRenderer::clearHistory() {
    m_historyVertices.clear();
}

void PathRenderer::draw() const {
    const bool hasHistory = m_historyVertices.size() >= 2;
    const bool hasPathGeometry = m_style == planner::PathStyle::Polyline && m_vertices.size() >= 2;

    if (!hasHistory && !hasPathGeometry) {
        return;
    }

    if (hasPathGeometry) {
        rebuildSegments();
    } else {
        m_travelledVertices.clear();
        m_remainingVertices.clear();
    }

    const bool hasTravelled = hasPathGeometry && m_travelledVertices.size() >= 2;
    const bool hasRemaining = hasPathGeometry && m_remainingVertices.size() >= 2;

    if (!hasHistory && !hasTravelled && !hasRemaining) {
        return;
    }

    glUseProgram(m_shaderProgram);
    glUniformMatrix4fv(glGetUniformLocation(m_shaderProgram, "viewProjection"), 1, GL_FALSE, &m_viewProjection[0][0]);

    glBindVertexArray(m_vertexArray);
    glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer);
    glLineWidth(PATH_LINE_WIDTH);

    const glm::vec3 black(0.0f, 0.0f, 0.0f);

    if (hasHistory) {
        glUniform3fv(glGetUniformLocation(m_shaderProgram, "uColor"), 1, &black[0]);
        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(m_historyVertices.size() * sizeof(glm::vec3)),
            m_historyVertices.data(), GL_DYNAMIC_DRAW);
        glDrawArrays(GL_LINE_STRIP, 0, static_cast<GLsizei>(m_historyVertices.size()));
    }

    if (hasTravelled) {
        glUniform3fv(glGetUniformLocation(m_shaderProgram, "uColor"), 1, &black[0]);
        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(m_travelledVertices.size() * sizeof(glm::vec3)),
            m_travelledVertices.data(), GL_DYNAMIC_DRAW);
        glDrawArrays(GL_LINE_STRIP, 0, static_cast<GLsizei>(m_travelledVertices.size()));
    }

    if (hasRemaining) {
        glUniform3fv(glGetUniformLocation(m_shaderProgram, "uColor"), 1, &m_pathColor[0]);
        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(m_remainingVertices.size() * sizeof(glm::vec3)),
            m_remainingVertices.data(), GL_DYNAMIC_DRAW);
        glDrawArrays(GL_LINE_STRIP, 0, static_cast<GLsizei>(m_remainingVertices.size()));
    }

    glLineWidth(1.0f);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void PathRenderer::upload() const {
    m_segmentsDirty = true;
}

void PathRenderer::rebuildSegments() const {
    if (!m_segmentsDirty) {
        return;
    }

    m_segmentsDirty = false;
    m_travelledVertices.clear();
    m_remainingVertices.clear();

    if (m_style != planner::PathStyle::Polyline || m_vertices.size() < 2) {
        return;
    }

    const float clampedRatio = std::clamp(m_travelRatio, 0.0f, 1.0f);

    if (clampedRatio <= 0.0f) {
        m_remainingVertices = m_vertices;
        return;
    }

    if (clampedRatio >= 1.0f || m_totalLength <= kEpsilon) {
        m_travelledVertices = m_vertices;
        return;
    }

    const float targetDistance = clampedRatio * m_totalLength;
    float accumulated = 0.0f;

    m_travelledVertices.push_back(m_vertices.front());

    for (std::size_t i = 0; i < m_segmentLengths.size(); ++i) {
        const glm::vec3& v0 = m_vertices[i];
        const glm::vec3& v1 = m_vertices[i + 1];
        const float segLen = m_segmentLengths[i];
        const float nextAccum = accumulated + segLen;

        if (targetDistance > nextAccum) {
            m_travelledVertices.push_back(v1);
            accumulated = nextAccum;
            continue;
        }

        const float segmentTravel = std::clamp(targetDistance - accumulated, 0.0f, segLen);
        const float t = segLen > kEpsilon ? segmentTravel / segLen : 0.0f;
        const glm::vec3 splitPoint = v0 + t * (v1 - v0);

        m_travelledVertices.push_back(splitPoint);

        m_remainingVertices.push_back(splitPoint);
        for (std::size_t j = i + 1; j < m_vertices.size(); ++j) {
            m_remainingVertices.push_back(m_vertices[j]);
        }
        return;
    }

    m_travelledVertices = m_vertices;
}
