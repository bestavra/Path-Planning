#include "map/MapLoader.h"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

namespace map {

namespace {

Metadata parseHeader(std::istream& input) {
    Metadata metadata;
    std::string headerLine;

    while (std::getline(input, headerLine)) {
        if (headerLine.empty()) {
            continue;
        }
        if (headerLine.front() == '#') {
            continue;
        }

        std::istringstream iss(headerLine);
        if (!(iss >> metadata.width >> metadata.height >> metadata.resolution)) {
            throw std::runtime_error("Failed to parse map header. Expected: <width> <height> <resolution>");
        }

        if (metadata.width == 0 || metadata.height == 0) {
            throw std::runtime_error("Map dimensions must be positive");
        }
        if (metadata.resolution <= 0.0f) {
            throw std::runtime_error("Map resolution must be positive");
        }
        return metadata;
    }

    throw std::runtime_error("Map header not found");
}

std::vector<float> parseCells(std::istream& input, const Metadata& metadata) {
    std::vector<float> cells;
    cells.reserve(metadata.cellCount());

    std::string line;
    while (std::getline(input, line)) {
        if (line.empty() || line.front() == '#') {
            continue;
        }
        std::istringstream iss(line);
        float value;
        while (iss >> value) {
            cells.push_back(value);
        }
        if (!iss.eof()) {
            throw std::runtime_error("Failed to parse map cell value");
        }
        if (cells.size() == metadata.cellCount()) {
            break;
        }
    }

    if (cells.size() != metadata.cellCount()) {
        throw std::runtime_error("Map file ended before expected number of cells were read");
    }

    return cells;
}

} // namespace

Grid Loader::load(const std::filesystem::path& filepath) const {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        throw std::runtime_error("Unable to open map file: " + filepath.string());
    }

    auto metadata = parseHeader(file);
    auto cells = parseCells(file, metadata);

    return Grid(metadata, std::move(cells));
}

} // namespace map
