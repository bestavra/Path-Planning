#include "viewer/ExploredCellsRenderer.h"

#include "viewer/ShaderUtils.h"

// clang-format off
#include <glad/gl.h>
#include <GLFW/glfw3.h>
// clang-format on

ExploredCellsRenderer::ExploredCellsRenderer() {
    const char* vertexShaderSource = "#version 330 core\n"
                                     "layout (location = 0) in vec2 aOffset;\n"
                                     "layout (location = 1) in vec2 aCenter;\n"
                                     "uniform mat4 viewProjection;\n"
                                     "void main() {\n"
                                     "    vec3 worldPos = vec3(aCenter + aOffset, 0.0);\n"
                                     "    gl_Position = viewProjection * vec4(worldPos, 1.0);\n"
                                     "}\n";

    const char* fragmentShaderSource = "#version 330 core\n"
                                       "uniform vec4 uColor;\n"
                                       "out vec4 FragColor;\n"
                                       "void main() {\n"
                                       "    FragColor = uColor;\n"
                                       "}\n";

    m_shaderProgram = ShaderUtils::createShaderProgram(vertexShaderSource, fragmentShaderSource);

    const float quadVertices[] = {-0.5f, -0.5f, 0.5f, -0.5f, -0.5f, 0.5f, 0.5f, 0.5f};

    glGenVertexArrays(1, &m_vertexArray);
    glGenBuffers(1, &m_quadVertexBuffer);
    glGenBuffers(1, &m_instanceVertexBuffer);

    glBindVertexArray(m_vertexArray);

    glBindBuffer(GL_ARRAY_BUFFER, m_quadVertexBuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), quadVertices, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), reinterpret_cast<void*>(0));
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, m_instanceVertexBuffer);
    glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), reinterpret_cast<void*>(0));
    glEnableVertexAttribArray(1);
    glVertexAttribDivisor(1, 1);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

ExploredCellsRenderer::~ExploredCellsRenderer() {
    if (!glfwGetCurrentContext()) {
        return;
    }

    if (m_vertexArray) {
        glDeleteVertexArrays(1, &m_vertexArray);
    }
    if (m_quadVertexBuffer) {
        glDeleteBuffers(1, &m_quadVertexBuffer);
    }
    if (m_instanceVertexBuffer) {
        glDeleteBuffers(1, &m_instanceVertexBuffer);
    }
    if (m_shaderProgram) {
        glDeleteProgram(m_shaderProgram);
    }
}

void ExploredCellsRenderer::setCamera(const glm::mat4& viewProjectionMatrix) {
    m_viewProjection = viewProjectionMatrix;
}

void ExploredCellsRenderer::setCells(const std::vector<glm::vec2>& centers) {
    m_instanceCount = centers.size();
    glBindBuffer(GL_ARRAY_BUFFER, m_instanceVertexBuffer);
    if (m_instanceCount > 0) {
        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(m_instanceCount * sizeof(glm::vec2)), centers.data(),
            GL_DYNAMIC_DRAW);
    } else {
        glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);
    }
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void ExploredCellsRenderer::clear() {
    m_instanceCount = 0;
    glBindBuffer(GL_ARRAY_BUFFER, m_instanceVertexBuffer);
    glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void ExploredCellsRenderer::draw() const {
    if (m_instanceCount == 0) {
        return;
    }

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glUseProgram(m_shaderProgram);
    glUniformMatrix4fv(glGetUniformLocation(m_shaderProgram, "viewProjection"), 1, GL_FALSE, &m_viewProjection[0][0]);
    glUniform4fv(glGetUniformLocation(m_shaderProgram, "uColor"), 1, &m_color[0]);

    glBindVertexArray(m_vertexArray);
    glDrawArraysInstanced(GL_TRIANGLE_STRIP, 0, 4, static_cast<GLsizei>(m_instanceCount));
    glBindVertexArray(0);

    glDisable(GL_BLEND);
}
