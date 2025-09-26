#include "path/PlannerUtils.h"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <string_view>

namespace planner::utils {

bool isCellWithinBounds(const map::Grid& grid, const glm::ivec2& cell) noexcept {
    if (cell.x < 0 || cell.y < 0) {
        return false;
    }
    const auto& metadata = grid.metadata();
    if (cell.x >= static_cast<int>(metadata.width) || cell.y >= static_cast<int>(metadata.height)) {
        return false;
    }
    return true;
}

bool isTraversableCell(const map::Grid& grid, const glm::ivec2& cell) {
    if (!isCellWithinBounds(grid, cell)) {
        return false;
    }

    const std::size_t x = static_cast<std::size_t>(cell.x);
    const std::size_t y = static_cast<std::size_t>(cell.y);

    if (grid.isMissing(x, y)) {
        return false;
    }

    const float value = grid.at(x, y);
    if (value >= 1.0f) {
        return false;
    }

    return true;
}

float traversalCost(const map::Grid& grid, const glm::ivec2& cell, bool diagonal) {
    const std::size_t x = static_cast<std::size_t>(cell.x);
    const std::size_t y = static_cast<std::size_t>(cell.y);
    const float value = grid.at(x, y);
    const float clamped = value < 0.0f ? 0.0f : value;
    const float baseCost = diagonal ? std::sqrt(2.0f) : 1.0f;
    return baseCost * (1.0f + clamped);
}

PlannedPath computePathWithTiming(IPathPlanner& planner, std::string_view label) {
    const auto start = std::chrono::steady_clock::now();
    PlannedPath result = planner.computePath();
    const auto end = std::chrono::steady_clock::now();
    const double milliseconds = std::chrono::duration<double, std::milli>(end - start).count();

    if (!label.empty()) {
        std::cout << "Planner [" << label << "] took " << milliseconds << " ms" << std::endl;
    } else {
        std::cout << "Planner run took " << milliseconds << " ms" << std::endl;
    }

    return result;
}

} // namespace planner::utils
