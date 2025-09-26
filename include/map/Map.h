#pragma once

#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

namespace map {

/**
 * @brief Describes the geometric properties of a grid map.
 *
 * Metadata encapsulates the width and height expressed in cells alongside the resolution in meters per cell.
 */
struct Metadata {
    std::size_t width = 0;
    std::size_t height = 0;
    float resolution = 1.0f; //!< Physical size represented by a single grid cell.

    /**
     * @brief Returns the total number of cells represented by the metadata.
     */
    [[nodiscard]] std::size_t cellCount() const noexcept {
        return width * height;
    }

    /**
     * @brief Produces a concise human-readable summary of the metadata.
     */
    [[nodiscard]] std::string summary() const;
};

/**
 * @brief Immutable grid map storing occupancy or traversability values per cell.
 */
class Grid {
  public:
    static constexpr float s_missingData = -1.0f;

    Grid() = default;

    /**
     * @brief Constructs a grid with the provided metadata and cell values.
     * @param metadata Dimensions and resolution information describing the grid.
     * @param cells Row-major ordered cell values sized according to the metadata.
     */
    Grid(Metadata metadata, std::vector<float> cells);

    /**
     * @brief Provides read-only access to the grid metadata.
     */
    [[nodiscard]] const Metadata& metadata() const noexcept {
        return m_metadata;
    }

    /**
     * @brief Provides read-only access to the raw cell values.
     */
    [[nodiscard]] const std::vector<float>& cells() const noexcept {
        return m_cells;
    }

    /**
     * @brief Returns the value stored at a given cell coordinate.
     * @param x Column index of the cell to query.
     * @param y Row index of the cell to query.
     * @return Stored traversal or occupancy value at the specified cell.
     * @throw std::out_of_range if the coordinate is outside the grid bounds.
     */
    [[nodiscard]] float at(std::size_t x, std::size_t y) const;

    /**
     * @brief Checks whether a given cell represents missing data.
     * @param x Column index of the cell to inspect.
     * @param y Row index of the cell to inspect.
     * @return True if the cell is marked as missing data, false otherwise.
     */
    [[nodiscard]] bool isMissing(std::size_t x, std::size_t y) const;

  private:
    Metadata m_metadata{};        //!< Describes the grid shape and resolution.
    std::vector<float> m_cells{}; //!< Flattened storage for grid values using row-major order.
};

} // namespace map
