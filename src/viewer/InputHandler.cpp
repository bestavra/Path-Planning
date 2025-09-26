#include "viewer/InputHandler.h"

#include "map/Map.h"
#include "path/IPathPlanner.h"
#include "path/PlannerTypes.h"
#include "path/PlannerUtils.h"
#include "viewer/Camera.h"
#include "viewer/Constants.h"
#include "viewer/Grid.h"
#include "viewer/MathUtils.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <optional>
#include <sstream>
#include <vector>

// Static member definitions
Camera* InputHandler::s_camera = nullptr;
Grid* InputHandler::s_grid = nullptr;
bool InputHandler::s_realTimeUpdating = true;
const map::Grid* InputHandler::s_mapData = nullptr;
planner::IPathPlanner* InputHandler::s_planner = nullptr;
InputHandler::PlannerAlgorithm InputHandler::s_plannerAlgorithm = InputHandler::PlannerAlgorithm::AStar;
std::vector<float> InputHandler::s_runtimeCells{};
map::CostmapLayer InputHandler::s_costmapLayer{};
std::size_t InputHandler::s_mapWidth = 0;
std::size_t InputHandler::s_mapHeight = 0;
InputHandler::PaintMode InputHandler::s_paintingMode = InputHandler::PaintMode::None;
std::optional<glm::ivec2> InputHandler::s_lastPaintedCell = std::nullopt;

namespace {

constexpr float kInflationMultiplier = 1.5f;
constexpr float kMinimumInflationRadius = 1.0f;

float distanceSquaredToSegment(const glm::vec2& point, const glm::vec2& a, const glm::vec2& b) {
    const glm::vec2 ab = b - a;
    const float abLenSq = glm::dot(ab, ab);
    if (abLenSq <= 1e-6f) {
        const glm::vec2 diff = point - a;
        return glm::dot(diff, diff);
    }

    const float t = std::clamp(glm::dot(point - a, ab) / abLenSq, 0.0f, 1.0f);
    const glm::vec2 projection = a + t * ab;
    const glm::vec2 diff = point - projection;
    return glm::dot(diff, diff);
}

std::optional<glm::ivec2> cursorCell(GLFWwindow* window, Camera* camera, Grid* grid) {
    if (!camera || !grid) {
        return std::nullopt;
    }

    double xpos = 0.0;
    double ypos = 0.0;
    glfwGetCursorPos(window, &xpos, &ypos);

    const glm::vec3 rayDir = MathUtils::rayCast(xpos, ypos, camera->getViewportWidth(), camera->getViewportHeight(),
        camera->getProjectionMatrix(), camera->getViewMatrix());
    const glm::vec3 hit =
        MathUtils::rayPlaneIntersection(camera->getPosition(), rayDir, glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(0.0f));
    if (!std::isfinite(hit.x) || !std::isfinite(hit.y)) {
        return std::nullopt;
    }

    glm::ivec2 cell(static_cast<int>(std::floor(hit.x)), static_cast<int>(std::floor(hit.y)));
    if (!grid->containsCell(cell)) {
        return std::nullopt;
    }
    return cell;
}

bool shiftKeyPressed(GLFWwindow* window) {
    return glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
           glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
}

bool ctrlKeyPressed(GLFWwindow* window) {
    return glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS ||
           glfwGetKey(window, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS;
}

} // namespace

void InputHandler::updatePlannerMap() {
    if (!s_planner || !s_mapData) {
        return;
    }

    ensureRuntimeCellsInitialized();

    if (s_runtimeCells.empty()) {
        if (s_grid) {
            s_grid->clearInflationOverlay();
        }
        return;
    }

    recomputeCostmap();

    const auto& costCells = s_costmapLayer.cells();
    map::Grid runtime(s_mapData->metadata(), costCells);
    s_planner->setMap(runtime);
}

void InputHandler::ensureRuntimeCellsInitialized() {
    if (!s_mapData) {
        s_runtimeCells.clear();
        if (s_grid) {
            s_grid->clearInflationOverlay();
        }
        return;
    }

    const auto& metadata = s_mapData->metadata();
    const std::size_t cellCount = metadata.cellCount();
    if (!s_costmapLayer.isInitialized() || s_costmapLayer.cells().size() != cellCount) {
        s_costmapLayer.initialize(metadata);
    }

    const std::size_t expectedSize = s_mapWidth * s_mapHeight;
    if (expectedSize == 0) {
        s_runtimeCells = s_mapData->cells();
        return;
    }

    if (s_runtimeCells.size() != expectedSize) {
        s_runtimeCells = s_mapData->cells();
    }
}

void InputHandler::recomputeCostmap() {
    if (!s_mapData || s_runtimeCells.empty() || !s_costmapLayer.isInitialized()) {
        if (s_grid) {
            s_grid->clearInflationOverlay();
        }
        return;
    }

    const float radius = costmapInflationRadius();
    s_costmapLayer.update(s_runtimeCells, radius);

    if (s_grid) {
        s_grid->setInflationOverlay(s_costmapLayer.inflationCenters());
    }
}

float InputHandler::costmapInflationRadius() {
    if (!s_grid) {
        return kMinimumInflationRadius;
    }

    const float agentRadius = s_grid->getAgentFootprintRadius();
    if (agentRadius <= 0.0f) {
        return kMinimumInflationRadius;
    }

    return std::max(agentRadius * kInflationMultiplier, kMinimumInflationRadius);
}

bool InputHandler::blockRuntimeCell(const glm::ivec2& cell) {
    if (!s_mapData || s_mapWidth == 0 || s_mapHeight == 0) {
        return false;
    }

    if (cell.x < 0 || cell.y < 0 || static_cast<std::size_t>(cell.x) >= s_mapWidth ||
        static_cast<std::size_t>(cell.y) >= s_mapHeight) {
        return false;
    }

    ensureRuntimeCellsInitialized();
    if (s_runtimeCells.empty()) {
        return false;
    }

    const std::size_t idx = static_cast<std::size_t>(cell.y) * s_mapWidth + static_cast<std::size_t>(cell.x);
    if (idx >= s_runtimeCells.size()) {
        return false;
    }

    if (s_runtimeCells[idx] >= 1.0f) {
        return false;
    }

    s_runtimeCells[idx] = 1.0f;
    return true;
}

bool InputHandler::resetRuntimeCell(const glm::ivec2& cell) {
    if (!s_mapData || s_mapWidth == 0 || s_mapHeight == 0) {
        return false;
    }

    if (cell.x < 0 || cell.y < 0 || static_cast<std::size_t>(cell.x) >= s_mapWidth ||
        static_cast<std::size_t>(cell.y) >= s_mapHeight) {
        return false;
    }

    ensureRuntimeCellsInitialized();
    if (s_runtimeCells.empty()) {
        return false;
    }

    const std::size_t idx = static_cast<std::size_t>(cell.y) * s_mapWidth + static_cast<std::size_t>(cell.x);
    if (idx >= s_runtimeCells.size()) {
        return false;
    }

    const auto& baseCells = s_mapData->cells();
    const float baseValue = idx < baseCells.size() ? baseCells[idx] : 0.0f;
    if (std::abs(s_runtimeCells[idx] - baseValue) < 1e-6f) {
        return false;
    }

    s_runtimeCells[idx] = baseValue;
    return true;
}

bool InputHandler::addObstacleFromInput(const glm::ivec2& cell) {
    if (!s_grid || !s_mapData) {
        return false;
    }

    s_lastPaintedCell = cell;

    if (!s_grid->addDynamicObstacle(cell, Grid::ObstacleVisibility::Hidden)) {
        return false;
    }

    std::cout << "Dynamic obstacle placed (hidden) at (" << cell.x << ", " << cell.y << ")" << std::endl;
    return true;
}

bool InputHandler::removeDynamicObstacle(const glm::ivec2& cell) {
    if (!s_grid || !s_mapData) {
        return false;
    }

    s_lastPaintedCell = cell;

    const bool wasVisible = s_grid->isVisibleDynamicObstacle(cell);

    if (!s_grid->removeDynamicObstacle(cell, *s_mapData)) {
        return false;
    }

    if (!wasVisible) {
        std::cout << "Hidden dynamic obstacle removed at (" << cell.x << ", " << cell.y << ")" << std::endl;
        return true;
    }

    const bool changed = resetRuntimeCell(cell);
    if (changed) {
        updatePlannerMap();
    }

    std::cout << "Discovered dynamic obstacle removed at (" << cell.x << ", " << cell.y << ")" << std::endl;
    runPlanner();
    return true;
}

void InputHandler::clearDynamicObstacles() {
    if (!s_grid || !s_mapData) {
        return;
    }

    if (!s_grid->hasDynamicObstacles()) {
        return;
    }

    s_grid->clearDynamicObstacles(*s_mapData);
    s_paintingMode = PaintMode::None;
    s_lastPaintedCell.reset();

    ensureRuntimeCellsInitialized();
    if (!s_runtimeCells.empty()) {
        s_runtimeCells = s_mapData->cells();
    }

    updatePlannerMap();
    runPlanner();
}

bool InputHandler::obstacleBlocksCurrentPath(const glm::ivec2& cell) {
    if (!s_grid) {
        return false;
    }

    if (const auto agentCell = s_grid->getAgentCurrentCell(); agentCell.has_value()) {
        if (agentCell.value() == cell) {
            return true;
        }
    }

    const auto& latest = s_grid->getLatestPath();
    if (!latest.has_value() || !latest->success || latest->waypoints.size() < 2) {
        return false;
    }

    const glm::vec2 cellCenter(static_cast<float>(cell.x) + 0.5f, static_cast<float>(cell.y) + 0.5f);

    for (const auto& waypoint : latest->waypoints) {
        const glm::ivec2 waypointCell{
            static_cast<int>(std::floor(waypoint.x)), static_cast<int>(std::floor(waypoint.y))};
        if (waypointCell == cell) {
            return true;
        }
    }

    constexpr float kHalfCell = 0.5f;
    const float maxDistanceSq = (kHalfCell + 1e-3f) * (kHalfCell + 1e-3f);

    for (std::size_t i = 0; i + 1 < latest->waypoints.size(); ++i) {
        const glm::vec2& a = latest->waypoints[i];
        const glm::vec2& b = latest->waypoints[i + 1];
        if (distanceSquaredToSegment(cellCenter, a, b) <= maxDistanceSq) {
            return true;
        }
    }

    return false;
}

bool InputHandler::currentPathBlockedByCostmap() {
    if (!s_grid || !s_mapData || !s_costmapLayer.isInitialized()) {
        return false;
    }

    const auto& latestPath = s_grid->getLatestPath();
    if (!latestPath.has_value() || !latestPath->success) {
        return false;
    }

    const auto& costCells = s_costmapLayer.cells();
    const auto& metadata = s_mapData->metadata();
    if (costCells.size() != metadata.cellCount()) {
        return false;
    }

    const auto isBlocked = [&](const glm::ivec2& cell) {
        if (cell.x < 0 || cell.y < 0 || cell.x >= static_cast<int>(metadata.width) ||
            cell.y >= static_cast<int>(metadata.height)) {
            return false;
        }

        const std::size_t idx = static_cast<std::size_t>(cell.y) * metadata.width + static_cast<std::size_t>(cell.x);
        return costCells[idx] >= 1.0f;
    };

    for (const auto& waypoint : latestPath->waypoints) {
        const glm::ivec2 cell{static_cast<int>(std::floor(waypoint.x)), static_cast<int>(std::floor(waypoint.y))};
        if (isBlocked(cell)) {
            return true;
        }
    }

    return false;
}

bool InputHandler::isTraversableForSelection(const glm::ivec2& cell) {
    if (!s_mapData || !s_costmapLayer.isInitialized()) {
        return false;
    }

    if (!s_costmapLayer.isTraversableCell(cell)) {
        return false;
    }

    if (s_grid && s_grid->isDynamicObstacle(cell)) {
        return false;
    }

    return true;
}

void InputHandler::handleCellSelection(const glm::ivec2& cell, bool isStart) {
    if (!s_mapData || !s_grid) {
        return;
    }

    if (!isTraversableForSelection(cell)) {
        std::cout << "Cell (" << cell.x << ", " << cell.y << ") is not traversable." << std::endl;
        return;
    }

    if (isStart) {
        bool startChanged = true;
        if (const auto existingStart = s_grid->getStartCell(); existingStart.has_value()) {
            startChanged = existingStart.value() != cell;
        }

        if (startChanged && s_plannerAlgorithm == PlannerAlgorithm::DStarLite) {
            s_grid->clearPath();
        }

        s_grid->setStartMarker(cell, *s_mapData);
        std::cout << "Start cell set to (" << cell.x << ", " << cell.y << ")" << std::endl;
    } else {
        s_grid->setGoalMarker(cell, *s_mapData);
        std::cout << "Goal cell set to (" << cell.x << ", " << cell.y << ")" << std::endl;
    }

    runPlanner();
}

void InputHandler::runPlanner() {
    if (!s_planner || !s_grid) {
        return;
    }

    const auto startMarker = s_grid->getStartCell();
    const auto goalCell = s_grid->getGoalCell();

    std::optional<glm::ivec2> plannerStart = startMarker;
    if (s_plannerAlgorithm == PlannerAlgorithm::DStarLite) {
        if (const auto agentCell = s_grid->getAgentCurrentCell(); agentCell.has_value()) {
            plannerStart = agentCell;
        }
    }

    if (!plannerStart.has_value() || !goalCell.has_value()) {
        s_grid->clearPath();
        return;
    }

    updatePlannerMap();
    s_planner->setStart(planner::PlannerPosition::Cell(plannerStart.value()));
    s_planner->setGoal(planner::PlannerPosition::Cell(goalCell.value()));

    try {
        std::ostringstream label;
        const auto& startForLabel = plannerStart.value();
        label << "interactive start=(" << startForLabel.x << ", " << startForLabel.y << ") goal=(" << goalCell->x
              << ", " << goalCell->y << ")";
        const std::string labelStr = label.str();

        const auto path = planner::utils::computePathWithTiming(*s_planner, labelStr);
        if (path.success) {
            std::vector<glm::vec2> historyToApply;
            if (s_plannerAlgorithm == PlannerAlgorithm::DStarLite) {
                historyToApply = s_grid->getAgentTravelHistory();
                if (!historyToApply.empty() && path.waypoints.size() >= 1) {
                    const glm::vec2 newStart = path.waypoints.front();
                    if (glm::length(historyToApply.back() - newStart) > 1e-3f) {
                        historyToApply.push_back(newStart);
                    }
                }
            }
            s_grid->setPath(path, historyToApply);
            std::cout << "Planner: path updated with " << path.waypoints.size() << " waypoints" << std::endl;
        } else {
            s_grid->clearPath();
            std::cout << "Planner: no path found for current start/goal" << std::endl;
        }
    } catch (const std::exception& ex) {
        std::cerr << "Planner error: " << ex.what() << std::endl;
    }
}

void InputHandler::initialize(
    Camera* cam, Grid* g, const map::Grid* map, planner::IPathPlanner* plannerInstance, AlgorithmSelection algorithm) {
    s_camera = cam;
    s_grid = g;
    s_mapData = map;
    s_planner = plannerInstance;
    s_plannerAlgorithm =
        (algorithm == AlgorithmSelection::DStarLite) ? PlannerAlgorithm::DStarLite : PlannerAlgorithm::AStar;

    if (s_mapData) {
        const auto& metadata = s_mapData->metadata();
        s_mapWidth = metadata.width;
        s_mapHeight = metadata.height;
        s_runtimeCells = s_mapData->cells();
        s_costmapLayer.initialize(metadata);
        recomputeCostmap();
    } else {
        s_mapWidth = s_mapHeight = 0;
        s_runtimeCells.clear();
        s_costmapLayer = map::CostmapLayer();
        if (s_grid) {
            s_grid->clearInflationOverlay();
        }
    }

    if (s_planner) {
        updatePlannerMap();
        if (s_grid && s_grid->getStartCell().has_value() && s_grid->getGoalCell().has_value()) {
            runPlanner();
        }
    }

    s_paintingMode = PaintMode::None;
    s_lastPaintedCell.reset();
}

void InputHandler::keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    (void)scancode;
    (void)mods;
    if (action == GLFW_PRESS) {
        if (key == GLFW_KEY_ESCAPE) {
            glfwSetWindowShouldClose(window, true);
        } else if (key == GLFW_KEY_C) {
            clearDynamicObstacles();
            std::cout << "Dynamic obstacles cleared" << std::endl;
        }
    }
}

void InputHandler::mouseCallback(GLFWwindow* window, double xpos, double ypos) {
    if (!s_camera || !s_grid)
        return;

    bool middleButtonPressed = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
    s_camera->handleMouseMovement(xpos, ypos, middleButtonPressed);

    if (middleButtonPressed) {
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

        // Update camera matrices for grid
        s_grid->setCamera(s_camera->getViewProjectionMatrix());
        s_grid->calculateFrustum(s_camera->getProjectionMatrix(), s_camera->getViewMatrix(), s_camera->getPosition(),
            s_camera->getViewportWidth(), s_camera->getViewportHeight());
        const bool showGridLines = s_camera->getPixelsPerUnit() >= GRID_LINE_MIN_PIXEL_SIZE;
        s_grid->setGridLinesVisible(showGridLines);

        // Adjust real-time updating based on camera distance
        s_realTimeUpdating = (s_camera->getPosition().z <= 15.0f);

        if (s_realTimeUpdating) {
            s_grid->update();
        }
    } else {
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
    }

    if (s_paintingMode != PaintMode::None) {
        const bool requiredModifierHeld = (s_paintingMode == PaintMode::Add && shiftKeyPressed(window)) ||
                                          (s_paintingMode == PaintMode::Remove && ctrlKeyPressed(window));

        if (!requiredModifierHeld) {
            s_paintingMode = PaintMode::None;
            s_lastPaintedCell.reset();
            return;
        }

        const auto cell = cursorCell(window, s_camera, s_grid);
        if (cell.has_value()) {
            const bool newCell = !s_lastPaintedCell.has_value() || cell.value() != s_lastPaintedCell.value();
            if (newCell) {
                const bool success = (s_paintingMode == PaintMode::Add) ? addObstacleFromInput(cell.value())
                                                                        : removeDynamicObstacle(cell.value());
                if (!success) {
                    s_lastPaintedCell = cell;
                }
            }
        }
    }
}

void InputHandler::mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
    if (button == GLFW_MOUSE_BUTTON_MIDDLE && action == GLFW_RELEASE) {
        if (!s_realTimeUpdating && s_grid) {
            s_grid->update();
        }
    }

    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE) {
        s_paintingMode = PaintMode::None;
        s_lastPaintedCell.reset();
        return;
    }

    if (!s_camera || !s_grid || !s_mapData || !s_planner) {
        return;
    }

    if (action != GLFW_PRESS) {
        return;
    }

    const auto cell = cursorCell(window, s_camera, s_grid);
    if (!cell.has_value()) {
        return;
    }

    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        const bool shiftHeld = (mods & GLFW_MOD_SHIFT) != 0;
        const bool ctrlHeld = (mods & GLFW_MOD_CONTROL) != 0;

        if (shiftHeld) {
            const bool success = addObstacleFromInput(cell.value());
            s_paintingMode = success ? PaintMode::Add : PaintMode::None;
            return;
        }

        if (ctrlHeld) {
            const bool success = removeDynamicObstacle(cell.value());
            s_paintingMode = success ? PaintMode::Remove : PaintMode::None;
            return;
        }

        handleCellSelection(cell.value(), true);
    } else if (button == GLFW_MOUSE_BUTTON_RIGHT) {
        handleCellSelection(cell.value(), false);
    }
}

void InputHandler::scrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
    if (!s_camera || !s_grid)
        return;

    const bool ctrlPressed = glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS ||
                             glfwGetKey(window, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS;
#ifdef __APPLE__
    const bool superPressed =
        glfwGetKey(window, GLFW_KEY_LEFT_SUPER) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_RIGHT_SUPER) == GLFW_PRESS;
#else
    const bool superPressed = false;
#endif
    const bool shiftPressed =
        glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;

    bool cameraChanged = false;

    if ((ctrlPressed || superPressed) && yoffset != 0.0) {
        double xpos, ypos;
        glfwGetCursorPos(window, &xpos, &ypos);
        s_camera->handleScroll(yoffset, xpos, ypos);
        cameraChanged = true;
    } else {
        glm::vec3 cameraPos = s_camera->getPosition();
        const float panBaseSpeed = KEYBOARD_PAN_SPEED;
        const float panStep = panBaseSpeed * std::max(0.1f, cameraPos.z * 0.05f);

        if (shiftPressed) {
            float horizontal = static_cast<float>(xoffset);
            if (horizontal == 0.0f) {
                horizontal = static_cast<float>(yoffset);
            }
            if (horizontal != 0.0f) {
                cameraPos.x += horizontal * panStep;
                cameraChanged = true;
            }
        } else {
            float vertical = static_cast<float>(yoffset);
            if (vertical != 0.0f) {
                cameraPos.y += vertical * panStep;
                cameraChanged = true;
            }
        }

        if (cameraChanged) {
            s_camera->setPosition(cameraPos);
        }
    }

    if (cameraChanged) {
        s_realTimeUpdating = (s_camera->getPosition().z <= 15.0f);
        s_grid->setCamera(s_camera->getViewProjectionMatrix());
        s_grid->calculateFrustum(s_camera->getProjectionMatrix(), s_camera->getViewMatrix(), s_camera->getPosition(),
            s_camera->getViewportWidth(), s_camera->getViewportHeight());
        const bool showGridLines = s_camera->getPixelsPerUnit() >= GRID_LINE_MIN_PIXEL_SIZE;
        s_grid->setGridLinesVisible(showGridLines);
        s_grid->update();
    }
}

void InputHandler::processInput(GLFWwindow* window) {
    if (!s_camera || !s_grid)
        return;
    (void)window;

    if (!s_mapData || !s_planner) {
        return;
    }

    const auto observation = s_grid->getAgentObservation();
    if (!observation.has_value()) {
        return;
    }

    const auto newlyDiscovered = s_grid->revealDynamicObstaclesWithinRadius(observation->center, observation->radius);
    if (newlyDiscovered.empty()) {
        return;
    }

    std::cout << "Discovered " << newlyDiscovered.size() << " dynamic obstacle"
              << (newlyDiscovered.size() == 1 ? "" : "s") << " within observation radius." << std::endl;

    bool runtimeUpdated = false;
    bool requiresReplan = false;

    for (const auto& cell : newlyDiscovered) {
        runtimeUpdated = blockRuntimeCell(cell) || runtimeUpdated;
        if (obstacleBlocksCurrentPath(cell)) {
            requiresReplan = true;
        }
    }

    if (runtimeUpdated) {
        updatePlannerMap();
        if (!requiresReplan && currentPathBlockedByCostmap()) {
            requiresReplan = true;
        }
    }

    const bool incrementalReplanNeeded = runtimeUpdated && s_plannerAlgorithm == PlannerAlgorithm::DStarLite;

    if (requiresReplan || incrementalReplanNeeded) {
        if (requiresReplan) {
            std::cout << "Newly discovered obstacle blocks current path; replanning..." << std::endl;
        } else {
            std::cout << "Map updated with newly discovered obstacle; refreshing plan." << std::endl;
        }
        runPlanner();
    }
}