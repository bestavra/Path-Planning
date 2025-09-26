#include "map/CostmapLayer.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace map {

namespace {
constexpr float s_obstacleThreshold = 1.0f;
}

void CostmapLayer::initialize(const Metadata& metadata) {
    m_metadata = metadata;
    m_cells.assign(m_metadata.cellCount(), 0.0f);
    m_inflationMask.assign(m_metadata.cellCount(), 0);
    m_inflationCenters.clear();
    m_initialized = true;
}

void CostmapLayer::update(const std::vector<float>& baseCells, float inflationRadiusCells) {
    if (!m_initialized) {
        throw std::runtime_error("CostmapLayer must be initialized before update");
    }

    if (baseCells.size() != m_metadata.cellCount()) {
        throw std::runtime_error("CostmapLayer update size mismatch");
    }

    m_cells = baseCells;
    std::fill(m_inflationMask.begin(), m_inflationMask.end(), 0u);
    m_inflationCenters.clear();

    if (m_metadata.width == 0 || m_metadata.height == 0) {
        return;
    }

    const float radius = std::max(inflationRadiusCells, 0.0f);
    if (radius <= 0.0f) {
        return;
    }

    const float radiusSquared = radius * radius;
    const int radiusCeil = std::max(1, static_cast<int>(std::ceil(radius)));

    for (std::size_t idx = 0; idx < baseCells.size(); ++idx) {
        if (baseCells[idx] < s_obstacleThreshold) {
            continue;
        }

        const int baseX = static_cast<int>(idx % m_metadata.width);
        const int baseY = static_cast<int>(idx / m_metadata.width);

        for (int dy = -radiusCeil; dy <= radiusCeil; ++dy) {
            const int ny = baseY + dy;
            if (ny < 0 || ny >= static_cast<int>(m_metadata.height)) {
                continue;
            }

            for (int dx = -radiusCeil; dx <= radiusCeil; ++dx) {
                const int nx = baseX + dx;
                if (nx < 0 || nx >= static_cast<int>(m_metadata.width)) {
                    continue;
                }

                const float distSquared = static_cast<float>(dx * dx + dy * dy);
                if (distSquared > radiusSquared) {
                    continue;
                }

                const std::size_t nIdx = static_cast<std::size_t>(ny) * m_metadata.width + static_cast<std::size_t>(nx);
                const float baseValue = baseCells[nIdx];

                if (baseValue >= s_obstacleThreshold) {
                    continue;
                }

                if (baseValue <= map::Grid::s_missingData) {
                    continue;
                }

                if (m_cells[nIdx] >= s_obstacleThreshold) {
                    if (!m_inflationMask[nIdx]) {
                        m_inflationMask[nIdx] = 1u;
                        m_inflationCenters.emplace_back(static_cast<float>(nx) + 0.5f, static_cast<float>(ny) + 0.5f);
                    }
                    continue;
                }

                m_cells[nIdx] = s_obstacleThreshold;
                if (!m_inflationMask[nIdx]) {
                    m_inflationMask[nIdx] = 1u;
                    m_inflationCenters.emplace_back(static_cast<float>(nx) + 0.5f, static_cast<float>(ny) + 0.5f);
                }
            }
        }
    }
}

bool CostmapLayer::isTraversableCell(const glm::ivec2& cell) const noexcept {
    if (!m_initialized) {
        return false;
    }

    if (!inBounds(cell.x, cell.y)) {
        return false;
    }

    const std::size_t idx = index(cell.x, cell.y);
    const float value = m_cells[idx];
    if (value <= map::Grid::s_missingData) {
        return false;
    }

    return value < s_obstacleThreshold;
}

bool CostmapLayer::inBounds(int x, int y) const noexcept {
    if (x < 0 || y < 0) {
        return false;
    }
    if (x >= static_cast<int>(m_metadata.width) || y >= static_cast<int>(m_metadata.height)) {
        return false;
    }
    return true;
}

std::size_t CostmapLayer::index(int x, int y) const noexcept {
    return static_cast<std::size_t>(y) * m_metadata.width + static_cast<std::size_t>(x);
}

} // namespace map
