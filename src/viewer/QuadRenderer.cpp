#include "viewer/QuadRenderer.h"

#include "viewer/Constants.h"
#include "viewer/MathUtils.h"
#include "viewer/ShaderUtils.h"

#include <algorithm>
#include <cmath>
// clang-format off
#include <glad/gl.h>
#include <GLFW/glfw3.h>
// clang-format on

QuadRenderer::QuadRenderer()
    : m_shaderProgram(0), m_vbo(0), m_vao(0), m_ebo(0), m_offsetBuffer(0), m_colorBuffer(0), m_activeInstanceCount(0),
      m_width(0), m_height(0), m_bottomLeft(glm::vec2(0.0f)), m_topRight(glm::vec2(-1.0f)) {
    const char* vertexShaderSource = "#version 330 core\n"
                                     "layout (location = 0) in vec3 aPos;\n"
                                     "layout (location = 1) in vec3 aOffset;\n"
                                     "layout (location = 2) in vec3 aCol;\n"
                                     "uniform mat4 viewProjection;\n"
                                     "out vec3 color;\n"
                                     "void main()\n"
                                     "{\n"
                                     "   vec3 worldPos = vec3(aPos.xy + aOffset.xy, aPos.z + aOffset.z);\n"
                                     "   gl_Position = viewProjection * vec4(worldPos, 1.0);\n"
                                     "   color = aCol;\n"
                                     "}\0";

    const char* fragmentShaderSource = "#version 330 core\n"
                                       "out vec4 FragColor;\n"
                                       "in vec3 color;\n"
                                       "void main()\n"
                                       "{\n"
                                       "   FragColor = vec4(color,1);\n"
                                       "}\n\0";

    m_shaderProgram = ShaderUtils::createShaderProgram(vertexShaderSource, fragmentShaderSource);

    float vertices[] = {
        1.0f, 1.0f, 0.0f, // top right
        1.0f, 0.0f, 0.0f, // bottom right
        0.0f, 0.0f, 0.0f, // bottom left
        0.0f, 1.0f, 0.0f  // top left
    };

    unsigned int indices[] = {
        0, 1, 3, // first Triangle
        1, 2, 3  // second Triangle
    };

    m_instanceColors = MathUtils::flatten(m_colors);

    glGenVertexArrays(1, &m_vao);
    glGenBuffers(1, &m_vbo);
    glGenBuffers(1, &m_ebo);
    glGenBuffers(1, &m_offsetBuffer);
    glGenBuffers(1, &m_colorBuffer);

    glBindVertexArray(m_vao);

    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // Offset buffer setup
    glBindBuffer(GL_ARRAY_BUFFER, m_offsetBuffer);
    glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);

    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribDivisor(1, 1);

    // Color buffer setup
    glBindBuffer(GL_ARRAY_BUFFER, m_colorBuffer);
    if (!m_instanceColors.empty()) {
        glBufferData(
            GL_ARRAY_BUFFER, m_instanceColors.size() * sizeof(glm::vec3), &m_instanceColors.front(), GL_DYNAMIC_DRAW);
    } else {
        glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);
    }

    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(2);
    glVertexAttribDivisor(2, 1);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    resize(GRID_WIDTH, GRID_HEIGHT);
}

QuadRenderer::~QuadRenderer() {
    if (!glfwGetCurrentContext()) {
        return;
    }

    if (m_vao)
        glDeleteVertexArrays(1, &m_vao);
    if (m_vbo)
        glDeleteBuffers(1, &m_vbo);
    if (m_ebo)
        glDeleteBuffers(1, &m_ebo);
    if (m_offsetBuffer)
        glDeleteBuffers(1, &m_offsetBuffer);
    if (m_colorBuffer)
        glDeleteBuffers(1, &m_colorBuffer);
    if (m_shaderProgram)
        glDeleteProgram(m_shaderProgram);
}

void QuadRenderer::addQuad(const glm::vec2& pos, const glm::vec3& color) {
    int x = static_cast<int>(pos.x);
    int y = static_cast<int>(pos.y);

    if (x < 0 || x >= static_cast<int>(m_width) || y < 0 || y >= static_cast<int>(m_height))
        return;

    m_positions[x][y] = glm::vec3(pos, 0.0f);
    m_active[x][y] = true;
    m_colors[x][y] = color;
}

void QuadRenderer::removeQuad(const glm::vec2& pos) {
    int x = static_cast<int>(pos.x);
    int y = static_cast<int>(pos.y);

    if (x < 0 || x >= static_cast<int>(m_width) || y < 0 || y >= static_cast<int>(m_height))
        return;

    m_active[x][y] = false;
}

void QuadRenderer::setCamera(const glm::mat4& viewProjectionMatrix) {
    m_viewProjection = viewProjectionMatrix;
}

void QuadRenderer::calculateFrustum(const glm::mat4& projection, const glm::mat4& view, const glm::vec3& cameraPos,
    float viewportWidth, float viewportHeight) {
    if (m_width == 0 || m_height == 0) {
        m_bottomLeft = glm::vec2(0.0f);
        m_topRight = glm::vec2(-1.0f);
        return;
    }

    const double safeWidth = std::max(1.0, static_cast<double>(viewportWidth));
    const double safeHeight = std::max(1.0, static_cast<double>(viewportHeight));

    glm::vec3 rayWorld = MathUtils::rayCast(0.0, safeHeight, safeWidth, safeHeight, projection, view);
    glm::vec3 worldPos = MathUtils::rayPlaneIntersection(cameraPos, rayWorld, glm::vec3(0, 0, 1), glm::vec3(0, 0, 0));
    m_bottomLeft = glm::vec2(static_cast<int>(worldPos.x), static_cast<int>(worldPos.y));

    rayWorld = MathUtils::rayCast(safeWidth, 0.0, safeWidth, safeHeight, projection, view);
    worldPos = MathUtils::rayPlaneIntersection(cameraPos, rayWorld, glm::vec3(0, 0, 1), glm::vec3(0, 0, 0));
    m_topRight = glm::vec2(static_cast<int>(worldPos.x), static_cast<int>(worldPos.y));

    const glm::vec2 minBounds(0.0f);
    const glm::vec2 maxBounds(
        static_cast<float>(m_width ? m_width - 1 : 0), static_cast<float>(m_height ? m_height - 1 : 0));
    m_bottomLeft = glm::clamp(m_bottomLeft, minBounds, maxBounds);
    m_topRight = glm::clamp(m_topRight, minBounds, maxBounds);
}

void QuadRenderer::update() {
    m_instancePositions.clear();
    m_instanceColors.clear();

    if (m_width == 0 || m_height == 0) {
        m_activeInstanceCount = 0;
        return;
    }

    const int startX = std::clamp(static_cast<int>(std::floor(m_bottomLeft.x)), 0, static_cast<int>(m_width) - 1);
    const int endX = std::clamp(static_cast<int>(std::floor(m_topRight.x)), 0, static_cast<int>(m_width) - 1);
    const int startY = std::clamp(static_cast<int>(std::floor(m_bottomLeft.y)), 0, static_cast<int>(m_height) - 1);
    const int endY = std::clamp(static_cast<int>(std::floor(m_topRight.y)), 0, static_cast<int>(m_height) - 1);

    if (startX > endX || startY > endY) {
        m_activeInstanceCount = 0;
        return;
    }

    const std::size_t maxCells = static_cast<std::size_t>((endX - startX + 1) * (endY - startY + 1));
    m_instancePositions.reserve(maxCells);
    m_instanceColors.reserve(maxCells);

    for (int x = startX; x <= endX; ++x) {
        for (int y = startY; y <= endY; ++y) {
            if (!m_active[x][y]) {
                continue;
            }
            m_instancePositions.push_back(m_positions[x][y]);
            m_instanceColors.push_back(m_colors[x][y]);
        }
    }

    if (!m_instancePositions.empty()) {
        glBindBuffer(GL_ARRAY_BUFFER, m_offsetBuffer);
        glBufferData(GL_ARRAY_BUFFER, m_instancePositions.size() * sizeof(glm::vec3), m_instancePositions.data(),
            GL_DYNAMIC_DRAW);

        glBindBuffer(GL_ARRAY_BUFFER, m_colorBuffer);
        glBufferData(
            GL_ARRAY_BUFFER, m_instanceColors.size() * sizeof(glm::vec3), m_instanceColors.data(), GL_DYNAMIC_DRAW);
        m_activeInstanceCount = m_instancePositions.size();
    } else {
        m_activeInstanceCount = 0;
    }
}

void QuadRenderer::draw() {
    if (m_activeInstanceCount == 0)
        return;

    glUseProgram(m_shaderProgram);
    glUniformMatrix4fv(glGetUniformLocation(m_shaderProgram, "viewProjection"), 1, GL_FALSE, &m_viewProjection[0][0]);

    glBindVertexArray(m_vao);
    glDrawElementsInstanced(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0, static_cast<GLsizei>(m_activeInstanceCount));
    glBindVertexArray(0);
}

void QuadRenderer::clear() {
    for (auto& column : m_active) {
        std::fill(column.begin(), column.end(), false);
    }
    for (auto& column : m_colors) {
        std::fill(column.begin(), column.end(), glm::vec3(0.0f));
    }
    for (auto& column : m_positions) {
        std::fill(column.begin(), column.end(), glm::vec3(0.0f));
    }
    m_activeInstanceCount = 0;
}

void QuadRenderer::resize(std::size_t newWidth, std::size_t newHeight) {
    m_width = newWidth;
    m_height = newHeight;

    m_colors.assign(m_width, std::vector<glm::vec3>(m_height, glm::vec3(0.0f)));
    m_positions.assign(m_width, std::vector<glm::vec3>(m_height, glm::vec3(0.0f)));
    m_active.assign(m_width, std::vector<bool>(m_height, false));
    m_instanceColors.clear();
    m_instancePositions.clear();

    m_bottomLeft = glm::vec2(0.0f);
    if (m_width == 0 || m_height == 0) {
        m_topRight = glm::vec2(-1.0f);
    } else {
        m_topRight = glm::vec2(static_cast<float>(m_width - 1), static_cast<float>(m_height - 1));
    }

    m_activeInstanceCount = 0;

    const std::size_t cellCount = m_width * m_height;

    glBindBuffer(GL_ARRAY_BUFFER, m_offsetBuffer);
    glBufferData(GL_ARRAY_BUFFER, cellCount * sizeof(glm::vec3), nullptr, GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, m_colorBuffer);
    glBufferData(GL_ARRAY_BUFFER, cellCount * sizeof(glm::vec3), nullptr, GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
}