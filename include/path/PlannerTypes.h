#pragma once

#include <glm/glm.hpp>
#include <vector>

namespace map {
class Grid;
}

namespace planner {

/**
 * @brief Enumerates supported path serialization styles.
 */
enum class PathStyle {
    Polyline,
    // Future path styles (e.g., Bezier, Continuous) can be added here
};

/**
 * @brief Distinguishes whether a planner position is stored as a grid cell or a world-space point.
 */
enum class PlannerPositionKind { Cell, Point };

/**
 * @brief Represents either a discrete grid cell or a continuous point.
 */
struct PlannerPosition {
    PlannerPositionKind kind = PlannerPositionKind::Cell;
    glm::ivec2 cell{0, 0};
    glm::vec2 point{0.0f, 0.0f};

    /**
     * @brief Creates a planner position backed by an integer grid cell.
     * @param c Grid cell coordinates that define the position.
     */
    static PlannerPosition Cell(const glm::ivec2& c) {
        PlannerPosition pos;
        pos.kind = PlannerPositionKind::Cell;
        pos.cell = c;
        return pos;
    }

    /**
     * @brief Creates a planner position for an arbitrary point.
     * @param p Continuous point position in map coordinates.
     */
    static PlannerPosition Point(const glm::vec2& p) {
        PlannerPosition pos;
        pos.kind = PlannerPositionKind::Point;
        pos.point = p;
        return pos;
    }

    /**
     * @brief Returns true when the position encodes a grid cell.
     */
    [[nodiscard]] bool isCell() const noexcept {
        return kind == PlannerPositionKind::Cell;
    }

    /**
     * @brief Returns true when the position encodes a continuous coordinate.
     */
    [[nodiscard]] bool isPoint() const noexcept {
        return kind == PlannerPositionKind::Point;
    }
};

/**
 * @brief Stores the outcome of a planner invocation, including waypoints and debug info.
 */
struct PlannedPath {
    PathStyle style = PathStyle::Polyline; //!< Representation format of the generated path.
    std::vector<glm::vec2> waypoints;      //!< Ordered list of waypoints in map coordinates.
    std::vector<glm::ivec2> exploredCells; //!< Cells expanded during planning useful for visualization.
    bool success = false;                  //!< Indicates whether a valid path was found.
};

} // namespace planner
