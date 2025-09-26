#pragma once

#include "AgentRenderer.h"
#include "ExploredCellsRenderer.h"
#include "LineRenderer.h"
#include "PathRenderer.h"
#include "QuadRenderer.h"
#include "agent/IAgent.h"
#include "path/PlannerTypes.h"

#include <cstddef>
#include <functional>
#include <glm/glm.hpp>
#include <optional>
#include <unordered_set>
#include <vector>

namespace map {
class Grid;
}

/**
 * @brief Central coordination class for rendering and interacting with the grid map.
 */
class Grid {
  public:
    /**
     * @brief Distinguishes whether dynamic obstacles should be rendered.
     */
    enum class ObstacleVisibility { Hidden, Visible };

    /**
     * @brief Captures the instantaneous agent observation footprint.
     */
    struct AgentObservation {
        glm::vec2 center;
        float radius;
    };

    /**
     * @brief Hash functor for glm::ivec2 coordinates.
     */
    struct IVec2Hash {
        std::size_t operator()(const glm::ivec2& v) const noexcept {
            const std::size_t hashX = std::hash<int>()(v.x);
            const std::size_t hashY = std::hash<int>()(v.y);
            return hashX ^ (hashY << 1);
        }
    };

    /**
     * @brief Constructs the grid with default dimensions.
     */
    Grid();
    ~Grid() = default;

    /**
     * @brief Resizes the grid to match the map dimensions.
     * @param newWidth Desired width expressed in grid cells.
     * @param newHeight Desired height expressed in grid cells.
     */
    void resize(std::size_t newWidth, std::size_t newHeight);

    /**
     * @brief Returns the current grid width in cells.
     */
    [[nodiscard]] std::size_t getWidth() const noexcept {
        return m_width;
    }

    /**
     * @brief Returns the current grid height in cells.
     */
    [[nodiscard]] std::size_t getHeight() const noexcept {
        return m_height;
    }

    /**
     * @brief Checks whether a cell coordinate lies within the grid.
     * @param cell Cell coordinate to test.
     * @return True if the coordinate lies within bounds, false otherwise.
     */
    [[nodiscard]] bool containsCell(const glm::ivec2& cell) const noexcept;

    /**
     * @brief Returns the currently selected start cell if valid.
     */
    [[nodiscard]] std::optional<glm::ivec2> getStartCell() const noexcept;

    /**
     * @brief Returns the currently selected goal cell if valid.
     */
    [[nodiscard]] std::optional<glm::ivec2> getGoalCell() const noexcept;

    /**
     * @brief Adds a cell overlay quad with the supplied color.
     * @param gridPos Cell coordinate expressed in grid space.
     * @param color RGB color to apply to the overlay quad.
     * @param updateImmediately Whether to update GPU buffers immediately after insertion.
     */
    void addCell(const glm::vec2& gridPos, const glm::vec3& color, bool updateImmediately = true);

    /**
     * @brief Removes an overlay quad at the specified cell.
     * @param gridPos Cell coordinate expressed in grid space.
     * @param updateImmediately Whether to update GPU buffers immediately after removal.
     */
    void removeCell(const glm::vec2& gridPos, bool updateImmediately = true);

    /**
     * @brief Updates the camera matrices across internal renderers.
     * @param viewProjectionMatrix Combined view-projection matrix supplied by the active camera.
     */
    void setCamera(const glm::mat4& viewProjectionMatrix);

    /**
     * @brief Recomputes the culling frustum for the quad renderer.
     * @param projection Camera projection matrix.
     * @param view Camera view matrix.
     * @param cameraPos Camera position in world coordinates.
     * @param viewportWidth Width of the viewport in pixels.
     * @param viewportHeight Height of the viewport in pixels.
     */
    void calculateFrustum(const glm::mat4& projection, const glm::mat4& view, const glm::vec3& cameraPos,
        float viewportWidth, float viewportHeight);

    /**
     * @brief Enables or disables rendering of grid lines.
     * @param visible True to render grid lines, false to hide them.
     */
    void setGridLinesVisible(bool visible) noexcept {
        m_gridLinesVisible = visible;
    }

    /**
     * @brief Returns true when grid lines will be drawn.
     */
    [[nodiscard]] bool getGridLinesVisible() const noexcept {
        return m_gridLinesVisible;
    }

    /**
     * @brief Associates an agent for rendering and tick updates.
     * @param agentPtr Agent instance to manage, or nullptr to detach.
     */
    void setAgent(agent::IAgent* agentPtr);

    /**
     * @brief Advances internal animations and agent traversal.
     * @param deltaSeconds Elapsed simulation time in seconds since the previous tick.
     */
    void tick(float deltaSeconds);

    /**
     * @brief Flushes pending GPU buffer changes.
     */
    void update();

    /**
     * @brief Issues draw calls for all renderers.
     */
    void draw();

    /**
     * @brief Clears overlays, markers, and dynamic obstacles.
     */
    void clear();

    /**
     * @brief Provides the agent footprint radius used during inflation.
     */
    [[nodiscard]] float getAgentFootprintRadius() const;

    /**
     * @brief Returns the last path rendered on the grid.
     */
    [[nodiscard]] const std::optional<planner::PlannedPath>& getLatestPath() const noexcept {
        return m_latestPath;
    }

    /**
     * @brief Highlights the explored cells from the planner.
     * @param cells Collection of explored cell coordinates in grid space.
     */
    void setExploredCells(const std::vector<glm::ivec2>& cells);

    /**
     * @brief Clears the explored cell overlay.
     */
    void clearExploredCells();

    /**
     * @brief Adds a dynamic obstacle and colors it according to visibility.
     * @param cell Cell coordinate to mark as a dynamic obstacle.
     * @param visibility Whether the new obstacle should be visible or hidden.
     * @return True if the obstacle was added, false if it already existed.
     */
    bool addDynamicObstacle(const glm::ivec2& cell, ObstacleVisibility visibility);

    /**
     * @brief Reveals a previously hidden dynamic obstacle.
     * @param cell Cell coordinate of the obstacle to reveal.
     * @return True if the obstacle transitioned to visible, false otherwise.
     */
    bool revealDynamicObstacle(const glm::ivec2& cell);

    /**
     * @brief Removes a dynamic obstacle and restores the base map color.
     * @param cell Cell coordinate of the obstacle to remove.
     * @param mapData Map data used to restore the cell color.
     * @return True if an obstacle was removed from the cell.
     */
    bool removeDynamicObstacle(const glm::ivec2& cell, const map::Grid& mapData);

    /**
     * @brief Clears all dynamic obstacles while restoring map colors.
     * @param mapData Map data used to restore base colors for all cells.
     */
    void clearDynamicObstacles(const map::Grid& mapData);

    /**
     * @brief Returns true if a cell contains any dynamic obstacle.
     */
    [[nodiscard]] bool isDynamicObstacle(const glm::ivec2& cell) const noexcept;

    /**
     * @brief Returns true when a cell contains a visible dynamic obstacle.
     */
    [[nodiscard]] bool isVisibleDynamicObstacle(const glm::ivec2& cell) const noexcept;

    /**
     * @brief Returns true when a cell contains a hidden dynamic obstacle.
     */
    [[nodiscard]] bool isHiddenDynamicObstacle(const glm::ivec2& cell) const noexcept;

    /**
     * @brief Indicates whether any dynamic obstacles exist.
     */
    [[nodiscard]] bool hasDynamicObstacles() const noexcept;

    /**
     * @brief Exposes the set of visible dynamic obstacles.
     */
    [[nodiscard]] const std::unordered_set<glm::ivec2, IVec2Hash>& getVisibleDynamicObstacles() const noexcept {
        return m_visibleDynamicObstacles;
    }

    /**
     * @brief Exposes the set of hidden dynamic obstacles.
     */
    [[nodiscard]] const std::unordered_set<glm::ivec2, IVec2Hash>& getHiddenDynamicObstacles() const noexcept {
        return m_hiddenDynamicObstacles;
    }

    /**
     * @brief Reveals hidden obstacles within the supplied radius and returns them.
     * @param center World-space center used to test obstacle distance.
     * @param radius Radius within which hidden obstacles should be revealed.
     * @return Collection of obstacles that transitioned to visible.
     */
    std::vector<glm::ivec2> revealDynamicObstaclesWithinRadius(const glm::vec2& center, float radius);

    /**
     * @brief Returns the agent observation footprint when available.
     */
    [[nodiscard]] std::optional<AgentObservation> getAgentObservation() const;

    /**
     * @brief Returns the cell currently occupied by the agent.
     */
    [[nodiscard]] std::optional<glm::ivec2> getAgentCurrentCell() const;

    /**
     * @brief Paints the static map onto the grid.
     * @param mapData Map data providing occupancy or traversal values.
     */
    void paint(const map::Grid& mapData);

    /**
     * @brief Sets the color of a specific map cell overlay.
     * @param cell Cell coordinate whose overlay color will change.
     * @param color RGB color to apply to the cell overlay.
     */
    void setCellColor(const glm::ivec2& cell, const glm::vec3& color);

    /**
     * @brief Displays a path and optional traversal history.
     * @param path Planned path to render on the grid.
     * @param history Optional travelled history polyline.
     */
    void setPath(const planner::PlannedPath& path, const std::vector<glm::vec2>& history = {});

    /**
     * @brief Clears the rendered path and travel history.
     */
    void clearPath();

    /**
     * @brief Returns the stored agent travel history polyline.
     */
    [[nodiscard]] std::vector<glm::vec2> getAgentTravelHistory() const;

    /**
     * @brief Visualizes inflation cells for debugging.
     * @param cells Collection of inflation cell centers in world coordinates.
     */
    void setInflationOverlay(const std::vector<glm::vec2>& cells);

    /**
     * @brief Clears the inflation cell overlay.
     */
    void clearInflationOverlay();

    /**
     * @brief Places the start marker on the map, respecting existing overlays.
     * @param cell Cell coordinate where the start marker should be placed.
     * @param mapData Map data used to adjust underlying colors.
     */
    void setStartMarker(const glm::ivec2& cell, const map::Grid& mapData);

    /**
     * @brief Places the goal marker on the map, respecting existing overlays.
     * @param cell Cell coordinate where the goal marker should be placed.
     * @param mapData Map data used to adjust underlying colors.
     */
    void setGoalMarker(const glm::ivec2& cell, const map::Grid& mapData);

    /**
     * @brief Removes the start marker and restores the base color.
     * @param mapData Map data used to restore the cell color.
     */
    void clearStartMarker(const map::Grid& mapData);

    /**
     * @brief Removes the goal marker and restores the base color.
     * @param mapData Map data used to restore the cell color.
     */
    void clearGoalMarker(const map::Grid& mapData);

    /**
     * @brief Converts a map occupancy value into an RGB color.
     * @param value Occupancy or cost value to convert.
     * @return Converted RGB color.
     */
    static glm::vec3 colorForValue(float value);

  private:
    LineRenderer m_lineRenderer;
    QuadRenderer m_quadRenderer;
    PathRenderer m_pathRenderer;
    AgentRenderer m_agentRenderer;
    ExploredCellsRenderer m_exploredRenderer;
    agent::IAgent* m_agent{nullptr};
    ExploredCellsRenderer m_inflationRenderer;
    std::size_t m_width;
    std::size_t m_height;
    bool m_gridLinesVisible{true};
    glm::mat4 m_lastViewProjection{1.0f};
    std::optional<planner::PlannedPath> m_latestPath;
    std::unordered_set<glm::ivec2, IVec2Hash> m_visibleDynamicObstacles;
    std::unordered_set<glm::ivec2, IVec2Hash> m_hiddenDynamicObstacles;
    std::vector<glm::vec2> m_travelHistory;
    std::vector<glm::vec2> m_inflationCells;

    glm::ivec2 m_startCell{-1, -1};
    glm::ivec2 m_goalCell{-1, -1};

    void repaintDynamicObstacles();
    void setCellColorForObstacle(const glm::ivec2& cell);
    void appendTravelHistory(const std::vector<glm::vec2>& polyline);
};