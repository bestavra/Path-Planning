#pragma once

#include <glm/glm.hpp>
#include <vector>

namespace MathUtils {

/**
 * @brief Converts a screen position to a normalized 3D direction for ray casting.
 * @param xpos Screen-space x position in pixels.
 * @param ypos Screen-space y position in pixels.
 * @param viewportWidth Width of the active viewport in pixels.
 * @param viewportHeight Height of the active viewport in pixels.
 * @param projection Projection matrix used for rendering the scene.
 * @param view View matrix describing the camera transform.
 * @return Normalized ray direction expressed in world coordinates.
 */
glm::vec3 rayCast(double xpos, double ypos, double viewportWidth, double viewportHeight, const glm::mat4& projection,
    const glm::mat4& view);

/**
 * @brief Calculates the intersection point between a ray and a plane.
 * @param ray_position Starting point of the ray in world coordinates.
 * @param ray_direction Normalized direction vector of the ray.
 * @param plane_normal Normal vector describing the plane orientation.
 * @param plane_position Any point located on the target plane.
 * @return Intersection point expressed in world coordinates.
 */
glm::vec3 rayPlaneIntersection(const glm::vec3& ray_position, const glm::vec3& ray_direction,
    const glm::vec3& plane_normal, const glm::vec3& plane_position);

/**
 * @brief Flattens a two-dimensional container inside an optional rectangular region.
 * @param orig Source 2D container to flatten into a single vector.
 * @param bottomLeft Inclusive lower-left bound of the region to copy.
 * @param topRight Exclusive upper-right bound of the region to copy; negative values include all cells.
 * @return Flattened 1D vector containing copies of the selected elements.
 */
template <typename T>
std::vector<T> flatten(const std::vector<std::vector<T>>& orig, glm::vec2 bottomLeft = glm::vec2(0.0f, 0.0f),
    glm::vec2 topRight = glm::vec2(-1.0f, -1.0f));

} // namespace MathUtils