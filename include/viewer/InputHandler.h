#pragma once

#include "map/CostmapLayer.h"

#include <GLFW/glfw3.h>
#include <cstddef>
#include <glm/glm.hpp>
#include <optional>
#include <vector>

namespace map {
class Grid;
}

namespace planner {
class IPathPlanner;
}

class Camera; // Forward declaration
class Grid;   // Forward declaration

class InputHandler {
  public:
    enum class AlgorithmSelection { AStar, DStarLite };

    /**
     * @brief Initializes the input handler with core application dependencies.
     * @param cam Active camera instance used for navigation.
     * @param g Grid instance responsible for rendering and map interaction.
     * @param map Map data currently loaded into the application.
     * @param plannerInstance Planner implementation used for path generation.
     * @param algorithm Planner algorithm selected for runtime execution.
     */
    static void initialize(Camera* cam, Grid* g, const map::Grid* map, planner::IPathPlanner* plannerInstance,
        AlgorithmSelection algorithm);

    // GLFW callback functions
    /**
     * @brief Handles keyboard events forwarded from GLFW.
     * @param window GLFW window that triggered the callback.
     * @param key Keyboard key identifier.
     * @param scancode Hardware scancode for the key.
     * @param action Key action such as press or release.
     * @param mods Modifier flags active during the event.
     */
    static void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);

    /**
     * @brief Handles mouse movement events forwarded from GLFW.
     * @param window GLFW window that triggered the callback.
     * @param xpos Cursor x position in pixels.
     * @param ypos Cursor y position in pixels.
     */
    static void mouseCallback(GLFWwindow* window, double xpos, double ypos);

    /**
     * @brief Handles mouse button events forwarded from GLFW.
     * @param window GLFW window that triggered the callback.
     * @param button Mouse button identifier.
     * @param action Button action such as press or release.
     * @param mods Modifier flags active during the event.
     */
    static void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods);

    /**
     * @brief Handles scroll wheel events forwarded from GLFW.
     * @param window GLFW window that triggered the callback.
     * @param xoffset Horizontal scroll offset.
     * @param yoffset Vertical scroll offset.
     */
    static void scrollCallback(GLFWwindow* window, double xoffset, double yoffset);

    // Input processing
    /**
     * @brief Processes per-frame input state and applies camera/grid updates.
     * @param window GLFW window providing input polling context.
     */
    static void processInput(GLFWwindow* window);

  private:
    static Camera* s_camera;
    static Grid* s_grid;
    static bool s_realTimeUpdating;
    static const map::Grid* s_mapData;
    static planner::IPathPlanner* s_planner;
    enum class PlannerAlgorithm { AStar, DStarLite };
    static PlannerAlgorithm s_plannerAlgorithm;
    static std::vector<float> s_runtimeCells;
    static map::CostmapLayer s_costmapLayer;
    static std::size_t s_mapWidth;
    static std::size_t s_mapHeight;
    enum class PaintMode { None, Add, Remove };

    static PaintMode s_paintingMode;
    static std::optional<glm::ivec2> s_lastPaintedCell;

    static void updatePlannerMap();
    static void ensureRuntimeCellsInitialized();
    static void recomputeCostmap();
    static float costmapInflationRadius();
    static bool blockRuntimeCell(const glm::ivec2& cell);
    static bool resetRuntimeCell(const glm::ivec2& cell);
    static bool addObstacleFromInput(const glm::ivec2& cell);
    static void clearDynamicObstacles();
    static bool removeDynamicObstacle(const glm::ivec2& cell);
    static bool obstacleBlocksCurrentPath(const glm::ivec2& cell);
    static bool currentPathBlockedByCostmap();
    static bool isTraversableForSelection(const glm::ivec2& cell);
    static void handleCellSelection(const glm::ivec2& cell, bool isStart);
    static void runPlanner();
};