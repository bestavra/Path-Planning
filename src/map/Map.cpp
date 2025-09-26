#include "map/Map.h"

#include <sstream>
#include <utility>

namespace map {

std::string Metadata::summary() const {
    std::ostringstream oss;
    oss << width << "x" << height << " @ " << resolution << "m";
    return oss.str();
}

Grid::Grid(Metadata metadata, std::vector<float> cells) : m_metadata(metadata), m_cells(std::move(cells)) {
    if (m_metadata.width == 0 || m_metadata.height == 0) {
        throw std::invalid_argument("Map dimensions must be positive");
    }
    if (m_cells.size() != m_metadata.cellCount()) {
        throw std::invalid_argument("Cell count does not match metadata dimensions");
    }
}

float Grid::at(std::size_t x, std::size_t y) const {
    if (x >= m_metadata.width || y >= m_metadata.height) {
        throw std::out_of_range("Requested cell is out of bounds");
    }
    return m_cells[y * m_metadata.width + x];
}

bool Grid::isMissing(std::size_t x, std::size_t y) const {
    return at(x, y) == s_missingData;
}

} // namespace map
