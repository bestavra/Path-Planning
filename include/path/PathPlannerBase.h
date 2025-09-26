#pragma once

#include "map/Map.h"
#include "path/IPathPlanner.h"

#include <optional>
#include <stdexcept>

namespace planner {

/**
 * @brief Provides shared state handling for concrete path planner implementations.
 */
template <PathStyle Style> class PathPlannerBase : public IPathPlanner {
  public:
    void setMap(const map::Grid& grid) override {
        m_map = grid;
    }

    void setStart(const PlannerPosition& position) override {
        m_start = position;
    }

    void setGoal(const PlannerPosition& position) override {
        m_goal = position;
    }

    PlannedPath computePath() override {
        ensureConfigured();
        PlannedPath path;
        path.style = Style;
        solve(path);
        path.success = !path.waypoints.empty();
        return path;
    }

  protected:
    /**
     * @brief Provides read-only access to the configured map instance.
     */
    const map::Grid& getMap() const {
        return m_map;
    }

    /**
     * @brief Retrieves the configured planner start position.
     */
    const PlannerPosition& getStart() const {
        return *m_start;
    }

    /**
     * @brief Retrieves the configured planner goal position.
     */
    const PlannerPosition& getGoal() const {
        return *m_goal;
    }

    /**
     * @brief Implemented by derived classes to populate the computed path.
     * @param path Output path structure that should be filled with planner results.
     */
    virtual void solve(PlannedPath& path) = 0;

  private:
    void ensureConfigured() const {
        if (m_map.metadata().width == 0 || m_map.metadata().height == 0) {
            throw std::runtime_error("Path planner map is not set");
        }
        if (!m_start.has_value()) {
            throw std::runtime_error("Path planner start position is not set");
        }
        if (!m_goal.has_value()) {
            throw std::runtime_error("Path planner goal position is not set");
        }
    }

    map::Grid m_map{};
    std::optional<PlannerPosition> m_start{};
    std::optional<PlannerPosition> m_goal{};
};

} // namespace planner
