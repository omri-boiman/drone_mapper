#pragma once

#include <filesystem>
#include <vector>
#include "types/MapValue.h"
#include "types/Units.h"
#include "io/ErrorLogger.h"

namespace drone {

// Bounding box as read from / written to file
struct MapBounds {
    Centi xmin{0 * si::centi<si::metre>}, xmax{0 * si::centi<si::metre>};
    Centi ymin{0 * si::centi<si::metre>}, ymax{0 * si::centi<si::metre>};
    Centi hmin{0 * si::centi<si::metre>}, hmax{0 * si::centi<si::metre>};
};

// A single cell entry as read from / written to file
struct MapCell {
    Centi    x{0 * si::centi<si::metre>};
    Centi    y{0 * si::centi<si::metre>};
    Centi    height{0 * si::centi<si::metre>};
    MapValue value{MapValue::NotMapped};
};

// Result of parsing a map file
struct ParsedMap {
    bool             valid{false};   // false if the file could not be read at all
    MapBounds        bounds;
    std::vector<MapCell> cells;
};

// Parse <path>/map_input.txt (or any file in the CELL format).
// Recoverable errors (bad lines, unknown values) are logged via ErrorLogger.
// Returns valid=false only for unrecoverable failures (file not found, no VERSION line).
ParsedMap ParseMapFile(const std::filesystem::path& filePath,
                       ErrorLogger& logger);

// Write cells to <filePath> in the CELL format.
// Returns false if the file could not be opened.
bool WriteMapFile(const std::filesystem::path& filePath,
                  const MapBounds&              bounds,
                  const std::vector<MapCell>&   cells);

} // namespace drone
