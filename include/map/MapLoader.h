#pragma once

#include "map/Map.h"

#include <filesystem>
#include <string>

namespace map {

/**
 * @brief Utility responsible for loading occupancy grids from disk.
 */
class Loader {
  public:
    Loader() = default;

    /**
     * @brief Loads a grid map from the provided file path.
     *
     * @param filepath Path to the *.map file on disk.
     * @return Fully constructed @ref Grid instance.
     * @throw std::runtime_error When the file cannot be opened or parsed.
     */
    [[nodiscard]] Grid load(const std::filesystem::path& filepath) const;
};

} // namespace map
