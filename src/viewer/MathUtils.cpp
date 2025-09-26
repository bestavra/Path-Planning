#include "viewer/MathUtils.h"

#include "viewer/Constants.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <glm/gtc/matrix_transform.hpp>

namespace MathUtils {

glm::vec3 rayCast(double xpos, double ypos, double viewportWidth, double viewportHeight, const glm::mat4& projection,
    const glm::mat4& view) {
    // converts a position from the 2d xpos, ypos to a normalized 3d direction
    const float safeWidth = static_cast<float>(std::max(1.0, viewportWidth));
    const float safeHeight = static_cast<float>(std::max(1.0, viewportHeight));

    float x = (2.0f * static_cast<float>(xpos)) / safeWidth - 1.0f;
    float y = 1.0f - (2.0f * static_cast<float>(ypos)) / safeHeight;
    float z = 1.0f;
    glm::vec3 ray_nds = glm::vec3(x, y, z);
    glm::vec4 ray_clip = glm::vec4(ray_nds.x, ray_nds.y, -1.0f, 1.0f);
    // eye space to clip we would multiply by projection so
    // clip space to eye space is the inverse projection
    glm::vec4 ray_eye = glm::inverse(projection) * ray_clip;
    // convert point to forwards
    ray_eye = glm::vec4(ray_eye.x, ray_eye.y, -1.0f, 0.0f);
    // world space to eye space is usually multiply by view so
    // eye space to world space is inverse view
    glm::vec4 inv_ray_wor = (glm::inverse(view) * ray_eye);
    glm::vec3 ray_wor = glm::vec3(inv_ray_wor.x, inv_ray_wor.y, inv_ray_wor.z);
    ray_wor = glm::normalize(ray_wor);
    return ray_wor;
}

glm::vec3 rayPlaneIntersection(const glm::vec3& ray_position, const glm::vec3& ray_direction,
    const glm::vec3& plane_normal, const glm::vec3& plane_position) {
    float d = glm::dot(plane_normal, plane_position - ray_position) / (0.001f + glm::dot(ray_direction, plane_normal));
    return ray_position + ray_direction * d;
}

template <typename T>
std::vector<T> flatten(const std::vector<std::vector<T>>& orig, glm::vec2 bottomLeft, glm::vec2 topRight) {
    std::vector<T> ret;

    const std::size_t width = orig.size();
    if (width == 0) {
        return ret;
    }

    const std::size_t height = orig.front().size();
    if (height == 0) {
        return ret;
    }

    glm::vec2 effectiveTopRight = topRight;
    if (effectiveTopRight.x < 0.0f) {
        effectiveTopRight.x = static_cast<float>(width - 1);
    }
    if (effectiveTopRight.y < 0.0f) {
        effectiveTopRight.y = static_cast<float>(height - 1);
    }

    int lx = static_cast<int>(std::floor(bottomLeft.x));
    int ly = static_cast<int>(std::floor(bottomLeft.y));
    int rx = static_cast<int>(std::floor(effectiveTopRight.x)) + 1;
    int ry = static_cast<int>(std::floor(effectiveTopRight.y)) + 1;

    lx = std::clamp(lx, 0, static_cast<int>(width));
    ly = std::clamp(ly, 0, static_cast<int>(height));
    rx = std::clamp(rx, 0, static_cast<int>(width));
    ry = std::clamp(ry, 0, static_cast<int>(height));

    if (lx >= rx || ly >= ry) {
        return ret;
    }

    ret.reserve(static_cast<std::size_t>((rx - lx) * (ry - ly)));

    for (int i = lx; i < rx; ++i) {
        const auto& row = orig[static_cast<std::size_t>(i)];
        if (row.empty()) {
            continue;
        }

        const std::size_t rowLimit = std::min<std::size_t>(row.size(), static_cast<std::size_t>(ry));
        const std::size_t rowStart = std::min<std::size_t>(row.size(), static_cast<std::size_t>(ly));

        if (rowLimit <= rowStart) {
            continue;
        }

        ret.insert(ret.end(), row.begin() + static_cast<std::ptrdiff_t>(rowStart),
            row.begin() + static_cast<std::ptrdiff_t>(rowLimit));
    }
    return ret;
}

// Explicit template instantiations
template std::vector<glm::vec3> flatten(
    const std::vector<std::vector<glm::vec3>>& orig, glm::vec2 bottomLeft, glm::vec2 topRight);
template std::vector<glm::mat4> flatten(
    const std::vector<std::vector<glm::mat4>>& orig, glm::vec2 bottomLeft, glm::vec2 topRight);

} // namespace MathUtils