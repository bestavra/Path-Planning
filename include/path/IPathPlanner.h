#pragma once

#include "path/PlannerTypes.h"

namespace planner {

/**
 * @brief Interface for all path planning algorithms.
 */
class IPathPlanner {
  public:
    virtual ~IPathPlanner() = default;

    /**
     * @brief Sets the occupancy grid that the planner should operate on.
     * @param grid Source map containing traversal costs or occupancy values.
     */
    virtual void setMap(const map::Grid& grid) = 0;

    /**
     * @brief Defines the planner start position.
     * @param position Start location expressed in planner coordinates.
     */
    virtual void setStart(const PlannerPosition& position) = 0;

    /**
     * @brief Defines the planner goal position.
     * @param position Goal location expressed in planner coordinates.
     */
    virtual void setGoal(const PlannerPosition& position) = 0;

    /**
     * @brief Executes the planning algorithm and returns the full path result.
     */
    virtual PlannedPath computePath() = 0;
};

} // namespace planner
