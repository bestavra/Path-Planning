#pragma once

#include "map/Map.h"
#include "path/IPathPlanner.h"
#include "path/PlannerTypes.h"

#include <glm/glm.hpp>
#include <string_view>

namespace planner::utils {

/**
 * @brief Checks whether a given cell lies within the provided grid bounds.
 * @param grid Source grid that defines valid bounds.
 * @param cell Cell coordinate to test against the grid dimensions.
 * @return True if the cell index is valid, false otherwise.
 */
bool isCellWithinBounds(const map::Grid& grid, const glm::ivec2& cell) noexcept;

/**
 * @brief Returns true when a cell is traversable according to the grid values.
 * @param grid Source grid that stores traversal costs or occupancy data.
 * @param cell Cell coordinate whose traversability should be evaluated.
 * @return True if the cell is considered traversable, false otherwise.
 */
bool isTraversableCell(const map::Grid& grid, const glm::ivec2& cell);

/**
 * @brief Computes the traversal cost for the provided cell, accounting for diagonal moves.
 * @param grid Source grid containing traversal costs.
 * @param cell Cell coordinate for which to compute the traversal cost.
 * @param diagonal Indicates whether the move into the cell is diagonal.
 * @return Cost associated with entering the specified cell.
 */
float traversalCost(const map::Grid& grid, const glm::ivec2& cell, bool diagonal);

/**
 * @brief Executes the planner while measuring execution time and logging it.
 * @param planner Planner instance that will compute the path.
 * @param label Optional label used in timing output for identification.
 * @return Planned path along with timing metadata.
 */
PlannedPath computePathWithTiming(IPathPlanner& planner, std::string_view label = {});

} // namespace planner::utils
