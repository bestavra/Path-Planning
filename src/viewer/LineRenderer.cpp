#include "viewer/LineRenderer.h"

#include "viewer/Constants.h"
#include "viewer/ShaderUtils.h"

// clang-format off
#include <glad/gl.h>
#include <GLFW/glfw3.h>
// clang-format on

LineRenderer::LineRenderer() : m_shaderProgram(0), m_vertexBuffer(0), m_vertexArray(0), m_width(0), m_height(0) {
    const char* vertexShaderSource = "#version 330 core\n"
                                     "layout (location = 0) in vec3 aPos;\n"
                                     "uniform mat4 viewProjection;\n"
                                     "void main()\n"
                                     "{\n"
                                     "   gl_Position = viewProjection * vec4(aPos.x, aPos.y, aPos.z, 1.0);\n"
                                     "}\0";

    const char* fragmentShaderSource = "#version 330 core\n"
                                       "out vec4 FragColor;\n"
                                       "void main()\n"
                                       "{\n"
                                       "   FragColor = vec4(0,0,0,1);\n"
                                       "}\n\0";

    m_shaderProgram = ShaderUtils::createShaderProgram(vertexShaderSource, fragmentShaderSource);

    glGenVertexArrays(1, &m_vertexArray);
    glGenBuffers(1, &m_vertexBuffer);
    glBindVertexArray(m_vertexArray);

    glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer);
    glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    setDimensions(GRID_WIDTH, GRID_HEIGHT);
}

LineRenderer::~LineRenderer() {
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

void LineRenderer::addLine(const glm::vec3& start, const glm::vec3& end) {
    std::vector<float> lineVertices = {start.x, start.y, start.z, end.x, end.y, end.z};

    m_vertices.insert(m_vertices.end(), lineVertices.begin(), lineVertices.end());
    upload();
}

void LineRenderer::setDimensions(std::size_t newWidth, std::size_t newHeight) {
    m_width = newWidth;
    m_height = newHeight;

    m_vertices.clear();

    if (m_width == 0 || m_height == 0) {
        upload();
        return;
    }

    m_vertices.reserve(((m_width + m_height) * 2 + 4) * 3);

    for (std::size_t j = 0; j <= m_height; ++j) {
        const float y = static_cast<float>(j);
        m_vertices.push_back(0.0f);
        m_vertices.push_back(y);
        m_vertices.push_back(0.0f);
        m_vertices.push_back(static_cast<float>(m_width));
        m_vertices.push_back(y);
        m_vertices.push_back(0.0f);
    }

    for (std::size_t i = 0; i <= m_width; ++i) {
        const float x = static_cast<float>(i);
        m_vertices.push_back(x);
        m_vertices.push_back(0.0f);
        m_vertices.push_back(0.0f);
        m_vertices.push_back(x);
        m_vertices.push_back(static_cast<float>(m_height));
        m_vertices.push_back(0.0f);
    }

    upload();
}

void LineRenderer::setCamera(const glm::mat4& viewProjectionMatrix) {
    m_viewProjection = viewProjectionMatrix;
}

void LineRenderer::draw() {
    if (m_vertices.empty())
        return;

    glUseProgram(m_shaderProgram);
    glUniformMatrix4fv(glGetUniformLocation(m_shaderProgram, "viewProjection"), 1, GL_FALSE, &m_viewProjection[0][0]);

    glBindVertexArray(m_vertexArray);
    glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(m_vertices.size() / 3));
    glBindVertexArray(0);
}

void LineRenderer::clear() {
    m_vertices.clear();
    upload();
}

void LineRenderer::upload() const {
    glBindVertexArray(m_vertexArray);
    glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer);
    if (!m_vertices.empty()) {
        glBufferData(GL_ARRAY_BUFFER, sizeof(float) * m_vertices.size(), m_vertices.data(), GL_DYNAMIC_DRAW);
    } else {
        glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);
    }
    glBindVertexArray(0);
}