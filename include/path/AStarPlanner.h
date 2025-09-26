#pragma once

#include "path/PathPlannerBase.h"

namespace planner {

/**
 * @brief Classic A* implementation producing polyline paths on 8-connected grids.
 */
class AStarPlanner : public PathPlannerBase<PathStyle::Polyline> {
  protected:
    void solve(PlannedPath& path) override;
};

} // namespace planner
