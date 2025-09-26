#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <vector>

// clang-format off
#include <glad/gl.h>
#include <GLFW/glfw3.h>
// clang-format on
#include "agent/SimpleAStarAgent.h"
#include "map/CostmapLayer.h"
#include "map/MapLoader.h"
#include "path/AStarPlanner.h"
#include "path/DStarLitePlanner.h"
#include "path/PlannerUtils.h"
#include "viewer/Camera.h"
#include "viewer/Constants.h"
#include "viewer/Grid.h"
#include "viewer/InputHandler.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace {

struct WindowContext {
    Camera* camera{nullptr};
    Grid* grid{nullptr};
};

void framebufferSizeCallback(GLFWwindow* window, int width, int height) {
    const int safeWidth = std::max(width, 1);
    const int safeHeight = std::max(height, 1);
    glViewport(0, 0, safeWidth, safeHeight);

    auto* context = static_cast<WindowContext*>(glfwGetWindowUserPointer(window));
    if (!context) {
        return;
    }

    if (context->camera) {
        context->camera->setViewportSize(static_cast<float>(safeWidth), static_cast<float>(safeHeight));
    }

    if (context->grid && context->camera) {
        context->grid->setCamera(context->camera->getViewProjectionMatrix());
        context->grid->calculateFrustum(context->camera->getProjectionMatrix(), context->camera->getViewMatrix(),
            context->camera->getPosition(), context->camera->getViewportWidth(), context->camera->getViewportHeight());
        const bool showGridLines = context->camera->getPixelsPerUnit() >= GRID_LINE_MIN_PIXEL_SIZE;
        context->grid->setGridLinesVisible(showGridLines);
        context->grid->update();
    }
}

std::vector<std::filesystem::path> collectMapFiles(const std::filesystem::path& directory) {
    std::vector<std::filesystem::path> maps;
    if (!std::filesystem::exists(directory)) {
        return maps;
    }

    for (const auto& entry : std::filesystem::directory_iterator(directory)) {
        if (!entry.is_regular_file()) {
            continue;
        }
        if (entry.path().extension() == ".map") {
            maps.push_back(entry.path());
        }
    }

    std::sort(maps.begin(), maps.end());
    return maps;
}

std::string prettyName(const std::filesystem::path& mapPath) {
    std::string stem = mapPath.stem().string();
    std::replace(stem.begin(), stem.end(), '_', ' ');
    return stem;
}

std::optional<glm::ivec2> findTraversableFromCorner(const map::Grid& grid, bool reverseOrder) {
    const auto& metadata = grid.metadata();
    if (metadata.width == 0 || metadata.height == 0) {
        return std::nullopt;
    }

    const int xBegin = reverseOrder ? static_cast<int>(metadata.width) - 1 : 0;
    const int yBegin = reverseOrder ? static_cast<int>(metadata.height) - 1 : 0;
    const int xEnd = reverseOrder ? -1 : static_cast<int>(metadata.width);
    const int yEnd = reverseOrder ? -1 : static_cast<int>(metadata.height);
    const int xStep = reverseOrder ? -1 : 1;
    const int yStep = reverseOrder ? -1 : 1;

    for (int y = yBegin; y != yEnd; y += yStep) {
        for (int x = xBegin; x != xEnd; x += xStep) {
            const glm::ivec2 cell{x, y};
            if (planner::utils::isTraversableCell(grid, cell)) {
                return cell;
            }
        }
    }

    return std::nullopt;
}

} // namespace

int main() {
    // Load maps
    const std::filesystem::path dataDirectory{"data"};
    const auto mapFiles = collectMapFiles(dataDirectory);
    if (mapFiles.empty()) {
        std::cerr << "No map files found in " << dataDirectory << ". Please add *.map files." << std::endl;
        return -1;
    }

    // Display available maps
    std::cout << "Available maps:" << std::endl;
    for (std::size_t i = 0; i < mapFiles.size(); ++i) {
        std::cout << "  [" << i << "] " << prettyName(mapFiles[i]) << " (" << mapFiles[i].filename().string() << ")"
                  << std::endl;
    }

    // Select map
    std::size_t selectedIndex = 0;
    std::cout << "Select map index [0-" << (mapFiles.size() - 1) << "] (default 0): ";
    std::string input;
    std::getline(std::cin, input);
    if (!input.empty()) {
        try {
            std::size_t value = std::stoul(input);
            if (value < mapFiles.size()) {
                selectedIndex = value;
            } else {
                std::cerr << "Index out of range. Using default map 0." << std::endl;
            }
        } catch (const std::exception&) {
            std::cerr << "Invalid input. Using default map 0." << std::endl;
        }
    }

    const auto selectedMap = mapFiles[selectedIndex];

    // Select planner
    std::cout << "Available planners:" << std::endl;
    std::cout << "  [0] A* (default)" << std::endl;
    std::cout << "  [1] D* Lite" << std::endl;
    std::cout << "Select planner index [0-1] (default 0): ";
    std::size_t plannerIndex = 0;
    std::string plannerInput;
    std::getline(std::cin, plannerInput);
    if (!plannerInput.empty()) {
        try {
            std::size_t value = std::stoul(plannerInput);
            if (value <= 1) {
                plannerIndex = value;
            } else {
                std::cerr << "Planner index out of range. Using default (A*)." << std::endl;
            }
        } catch (const std::exception&) {
            std::cerr << "Invalid planner input. Using default (A*)." << std::endl;
        }
    }

    InputHandler::AlgorithmSelection plannerSelection =
        plannerIndex == 1 ? InputHandler::AlgorithmSelection::DStarLite : InputHandler::AlgorithmSelection::AStar;

    std::unique_ptr<planner::IPathPlanner> planner;
    if (plannerIndex == 1) {
        planner = std::make_unique<planner::DStarLitePlanner>();
        std::cout << "Planner: D* Lite" << std::endl;
    } else {
        planner = std::make_unique<planner::AStarPlanner>();
        std::cout << "Planner: A*" << std::endl;
    }

    // Initialize GLFW
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return -1;
    }

    // Configure GLFW
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    // Create window
    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "PathPlanning", nullptr, nullptr);
    if (!window) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);

    // Initialize GLAD
    if (!gladLoadGL((GLADloadfunc)glfwGetProcAddress)) {
        std::cerr << "Failed to initialize GLAD" << std::endl;
        glfwTerminate();
        return -1;
    }

    int framebufferWidth = SCR_WIDTH;
    int framebufferHeight = SCR_HEIGHT;
    glfwGetFramebufferSize(window, &framebufferWidth, &framebufferHeight);
    glViewport(0, 0, framebufferWidth, framebufferHeight);

    // Create core objects
    auto camera = std::make_unique<Camera>(glm::vec3(GRID_WIDTH / 2.0f, GRID_HEIGHT / 2.0f, 15.0f));
    auto grid = std::make_unique<Grid>();
    auto agent = std::make_unique<agent::SimpleAStarAgent>();
    grid->setAgent(agent.get());

    // Load selected map
    map::Loader mapLoader;
    map::Grid loadedMap;
    try {
        loadedMap = mapLoader.load(selectedMap);
        std::cout << "Loaded map: " << prettyName(selectedMap) << " -> " << loadedMap.metadata().summary() << std::endl;
    } catch (const std::exception& ex) {
        std::cerr << "Failed to load map file: " << ex.what() << std::endl;
        glfwTerminate();
        return -1;
    }

    const auto& meta = loadedMap.metadata();
    agent->configurePhysicalSize(DEFAULT_AGENT_DIAMETER_METERS, meta.resolution);
    grid->resize(meta.width, meta.height);
    grid->paint(loadedMap);

    const float centerX = static_cast<float>(meta.width) / 2.0f;
    const float centerY = static_cast<float>(meta.height) / 2.0f;

    const float padding = MAP_VIEW_PADDING_CELLS * 2.0f;
    const float targetWidth = static_cast<float>(meta.width) + padding;
    const float targetHeight = static_cast<float>(meta.height) + padding;

    const float halfWidth = targetWidth * 0.5f;
    const float halfHeight = targetHeight * 0.5f;

    const float fovRadians = glm::radians(DEFAULT_FOV);
    const float aspectRatio = static_cast<float>(framebufferWidth) / static_cast<float>(std::max(framebufferHeight, 1));
    const float tanHalfFov = std::tan(fovRadians * 0.5f);
    const float distanceY = halfHeight / tanHalfFov;
    const float distanceX = halfWidth / (tanHalfFov * aspectRatio);
    const float cameraZ = std::max({distanceX, distanceY, 1.0f});

    camera->setPosition(glm::vec3(centerX, centerY, cameraZ));
    camera->setViewportSize(static_cast<float>(framebufferWidth), static_cast<float>(framebufferHeight));

    WindowContext windowContext{camera.get(), grid.get()};
    glfwSetWindowUserPointer(window, &windowContext);
    glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);

    // Inflate obstacles
    const float inflationRadiusCells = std::max(grid->getAgentFootprintRadius() * 1.5f, 1.0f);
    map::CostmapLayer initialCostmap;
    initialCostmap.initialize(meta);
    initialCostmap.update(loadedMap.cells(), inflationRadiusCells);
    map::Grid inflatedGrid(meta, initialCostmap.cells());

    planner->setMap(inflatedGrid);
    grid->setInflationOverlay(initialCostmap.inflationCenters());

    auto startCell = findTraversableFromCorner(inflatedGrid, false);
    auto goalCell = findTraversableFromCorner(inflatedGrid, true);

    if (startCell && goalCell && startCell == goalCell) {
        goalCell = std::nullopt;
    }

    if (startCell) {
        grid->setStartMarker(*startCell, loadedMap);
    }

    if (!goalCell && startCell) {
        bool found = false;
        for (std::size_t y = 0; y < meta.height && !found; ++y) {
            for (std::size_t x = 0; x < meta.width; ++x) {
                const glm::ivec2 candidate{static_cast<int>(x), static_cast<int>(y)};
                if (candidate == *startCell) {
                    continue;
                }
                if (planner::utils::isTraversableCell(inflatedGrid, candidate)) {
                    goalCell = candidate;
                    found = true;
                    break;
                }
            }
        }

        if (!goalCell) {
            goalCell = startCell;
        }
    }

    if (goalCell) {
        grid->setGoalMarker(*goalCell, loadedMap);
    }

    if (!startCell) {
        std::cout << "Planner: unable to auto-select a start cell; choose one with Left Click" << std::endl;
    }
    if (!goalCell) {
        std::cout << "Planner: unable to auto-select a goal cell; choose one with Right Click" << std::endl;
    }

    if (startCell && goalCell) {
        planner->setStart(planner::PlannerPosition::Cell(*startCell));
        planner->setGoal(planner::PlannerPosition::Cell(*goalCell));
        const auto initialPath = planner::utils::computePathWithTiming(*planner, "initial");
        if (initialPath.success) {
            grid->setPath(initialPath);
            std::cout << "Planner: initial path with " << initialPath.waypoints.size() << " waypoints" << std::endl;
        } else {
            std::cout << "Planner: unable to compute initial path with default start/goal" << std::endl;
        }
    } else {
        std::cout << "Planner: select start and goal cells to compute a path" << std::endl;
    }

    // Initialize input handler
    InputHandler::initialize(camera.get(), grid.get(), &loadedMap, planner.get(), plannerSelection);

    // set up GLFW callbacks
    glfwSetKeyCallback(window, InputHandler::keyCallback);
    glfwSetCursorPosCallback(window, InputHandler::mouseCallback);
    glfwSetMouseButtonCallback(window, InputHandler::mouseButtonCallback);
    glfwSetScrollCallback(window, InputHandler::scrollCallback);

    // Set initial camera matrices
    grid->setCamera(camera->getViewProjectionMatrix());
    grid->calculateFrustum(camera->getProjectionMatrix(), camera->getViewMatrix(), camera->getPosition(),
        camera->getViewportWidth(), camera->getViewportHeight());
    grid->setGridLinesVisible(camera->getPixelsPerUnit() >= GRID_LINE_MIN_PIXEL_SIZE);

    // Set OpenGL state
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    double lastFrameTime = glfwGetTime();

    std::cout << "----------------------------------------------" << std::endl;
    std::cout << "PathPlanning - Modular Edition" << std::endl;
    std::cout << "----------------------------------------------" << std::endl;
    std::cout << std::endl;
    std::cout << "Controls:" << std::endl;
    std::cout << "  Scroll: Zoom in/out" << std::endl;
    std::cout << "  Shift + Scroll or Horizontal Scroll: Pan horizontally" << std::endl;
    std::cout << "  Ctrl  + Scroll: Pan vertically" << std::endl;
    std::cout << "  Left Click: Set start cell" << std::endl;
    std::cout << "  Right Click: Set goal cell" << std::endl;
    std::cout << "  Shift + Left Click + Drag: Add obstacles" << std::endl;
    std::cout << "  Ctrl + Left Click + Drag: Remove obstacles" << std::endl;
    std::cout << "  C: Remove all added obstacles" << std::endl;
    std::cout << "  Explored cells appear highlighted (yellow) once planning runs" << std::endl;
    std::cout << "  Inflated obstacles appear highlighted (cyan)" << std::endl;
    std::cout << "  ESC: Exit" << std::endl;
    std::cout << "----------------------------------------------" << std::endl;
    std::cout << std::endl;

    // Main render loop
    while (!glfwWindowShouldClose(window)) {
        const double currentTime = glfwGetTime();
        const float deltaSeconds = static_cast<float>(currentTime - lastFrameTime);
        lastFrameTime = currentTime;

        InputHandler::processInput(window);

        grid->tick(deltaSeconds);

        // clear screen
        glClear(GL_COLOR_BUFFER_BIT);

        grid->draw();

        // Swap buffers and poll events
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // Cleanup OpenGL resources before terminating GLFW
    // (prevents from invoking callbacks that reference deleted objects)
    glfwSetFramebufferSizeCallback(window, nullptr);
    glfwSetKeyCallback(window, nullptr);
    glfwSetCursorPosCallback(window, nullptr);
    glfwSetMouseButtonCallback(window, nullptr);
    glfwSetScrollCallback(window, nullptr);
    windowContext.camera = nullptr;
    windowContext.grid = nullptr;
    glfwSetWindowUserPointer(window, nullptr);

    // cleanup core objects
    if (grid) {
        grid->setAgent(nullptr);
    }
    agent.reset();
    camera.reset();
    grid.reset();

    // safe to terminate GLFW
    glfwTerminate();
    return 0;
}