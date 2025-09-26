#include "viewer/AgentRenderer.h"

#include "viewer/ShaderUtils.h"

#include <algorithm>
#include <glm/glm.hpp>
// clang-format off
#include <glad/gl.h>
#include <GLFW/glfw3.h>
// clang-format on

AgentRenderer::AgentRenderer() {
    const char* vertexShaderSource = "#version 330 core\n"
                                     "layout (location = 0) in vec2 aOffset;\n"
                                     "uniform vec2 uCenter;\n"
                                     "uniform float uRadius;\n"
                                     "uniform mat4 viewProjection;\n"
                                     "out vec2 vLocal;\n"
                                     "void main() {\n"
                                     "    vec2 scaledOffset = aOffset * (uRadius * 2.0);\n"
                                     "    vLocal = scaledOffset;\n"
                                     "    vec3 worldPos = vec3(uCenter + scaledOffset, 0.0);\n"
                                     "    gl_Position = viewProjection * vec4(worldPos, 1.0);\n"
                                     "}\n";

    const char* fragmentShaderSource = "#version 330 core\n"
                                       "uniform vec4 uColor;\n"
                                       "uniform float uRadius;\n"
                                       "in vec2 vLocal;\n"
                                       "out vec4 FragColor;\n"
                                       "void main() {\n"
                                       "    if (length(vLocal) > uRadius) {\n"
                                       "        discard;\n"
                                       "    }\n"
                                       "    FragColor = uColor;\n"
                                       "}\n";

    m_shaderProgram = ShaderUtils::createShaderProgram(vertexShaderSource, fragmentShaderSource);

    const float vertices[] = {-0.5f, -0.5f, 0.5f, -0.5f, -0.5f, 0.5f, 0.5f, 0.5f};

    glGenVertexArrays(1, &m_vertexArray);
    glGenBuffers(1, &m_vertexBuffer);

    glBindVertexArray(m_vertexArray);
    glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), reinterpret_cast<void*>(0));
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

AgentRenderer::~AgentRenderer() {
    if (!glfwGetCurrentContext()) {
        return;
    }

    if (m_vertexArray) {
        glDeleteVertexArrays(1, &m_vertexArray);
    }
    if (m_vertexBuffer) {
        glDeleteBuffers(1, &m_vertexBuffer);
    }
    if (m_shaderProgram) {
        glDeleteProgram(m_shaderProgram);
    }
}

void AgentRenderer::setCamera(const glm::mat4& viewProjectionMatrix) {
    m_viewProjection = viewProjectionMatrix;
}

void AgentRenderer::drawAgent(const glm::vec2& center, const glm::vec3& color, float radius) const {
    const float safeRadius = std::max(radius, 0.0f);
    drawCircle(center, safeRadius, glm::vec4(color, 1.0f));
}

void AgentRenderer::drawObservationArea(const glm::vec2& center, float radius, const glm::vec4& color) const {
    if (radius <= 0.0f || color.a <= 0.0f) {
        return;
    }

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    drawCircle(center, radius, color);
    glDisable(GL_BLEND);
}

void AgentRenderer::drawCircle(const glm::vec2& center, float radius, const glm::vec4& color) const {
    glUseProgram(m_shaderProgram);
    glUniformMatrix4fv(glGetUniformLocation(m_shaderProgram, "viewProjection"), 1, GL_FALSE, &m_viewProjection[0][0]);
    glUniform2fv(glGetUniformLocation(m_shaderProgram, "uCenter"), 1, &center[0]);
    glUniform1f(glGetUniformLocation(m_shaderProgram, "uRadius"), radius);
    glUniform4fv(glGetUniformLocation(m_shaderProgram, "uColor"), 1, &color[0]);

    glBindVertexArray(m_vertexArray);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    glBindVertexArray(0);
}
