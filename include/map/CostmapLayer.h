#pragma once

#include "map/Map.h"

#include <cstddef>
#include <cstdint>
#include <glm/glm.hpp>
#include <vector>

namespace map {

/**
 * @brief Maintains an inflated representation of a grid including traversal metadata.
 */
class CostmapLayer {
  public:
    CostmapLayer() = default;

    /**
     * @brief Configures the layer with map metadata and resets internal buffers.
     * @param metadata Dimensions and resolution describing the input grid map.
     */
    void initialize(const Metadata& metadata);

    /**
     * @brief Indicates whether the layer has been initialised and is ready for updates.
     */
    [[nodiscard]] bool isInitialized() const noexcept {
        return m_initialized;
    }

    /**
     * @brief Recomputes the inflated cost map using the provided base grid values.
     * @param baseCells Raw traversal cost per cell prior to inflation.
     * @param inflationRadiusCells Inflation radius expressed in grid cells.
     */
    void update(const std::vector<float>& baseCells, float inflationRadiusCells);

    /**
     * @brief Returns the current inflated traversal cost values.
     */
    [[nodiscard]] const std::vector<float>& cells() const noexcept {
        return m_cells;
    }

    /**
     * @brief Returns the centers of the cells affected by inflation for visualization.
     */
    [[nodiscard]] const std::vector<glm::vec2>& inflationCenters() const noexcept {
        return m_inflationCenters;
    }

    /**
     * @brief Checks whether a cell remains traversable after inflation.
     * @param cell Grid cell coordinates to inspect.
     * @return True if the cell is still traversable once inflated, false otherwise.
     */
    [[nodiscard]] bool isTraversableCell(const glm::ivec2& cell) const noexcept;

  private:
    Metadata m_metadata{};
    std::vector<float> m_cells{};
    std::vector<std::uint8_t> m_inflationMask{};
    std::vector<glm::vec2> m_inflationCenters{};
    bool m_initialized = false;

    [[nodiscard]] bool inBounds(int x, int y) const noexcept;
    [[nodiscard]] std::size_t index(int x, int y) const noexcept;
};

} // namespace map
