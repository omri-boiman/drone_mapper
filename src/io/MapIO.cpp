#include "io/MapIO.h"

#include <fstream>
#include <sstream>
#include <iostream>
#include <string>

using namespace mp_units;
using namespace mp_units::si::unit_symbols;

namespace drone {

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static MapValue IntToMapValue(int v)
{
    switch (v) {
        case  0: return MapValue::Empty;
        case  1: return MapValue::Occupied;
        case -1: return MapValue::NotMapped;
        case -2: return MapValue::BeyondBounds;
        default: return MapValue::NotMapped;   // treated as unmapped
    }
}

static int MapValueToInt(MapValue v)
{
    return static_cast<int>(v);
}

// Parse "key=value" tokens on the BOUNDS line, e.g. xmin=0.00
static bool ParseBoundsToken(const std::string& token,
                              const std::string& key,
                              double&            out)
{
    const std::string prefix = key + "=";
    if (token.rfind(prefix, 0) != 0) {
        return false;
    }
    try {
        out = std::stod(token.substr(prefix.size()));
        return true;
    } catch (...) {
        return false;
    }
}

// ---------------------------------------------------------------------------
// ParseMapFile
// ---------------------------------------------------------------------------

ParsedMap ParseMapFile(const std::filesystem::path& filePath,
                       ErrorLogger& logger)
{
    ParsedMap result;

    std::ifstream file(filePath);
    if (!file.is_open()) {
        std::cerr << "Error: cannot open map file: " << filePath << "\n";
        return result;   // valid = false
    }

    bool foundVersion = false;
    bool foundBounds  = false;
    int  lineNum      = 0;
    std::string line;

    while (std::getline(file, line)) {
        ++lineNum;

        // Strip leading whitespace
        const auto start = line.find_first_not_of(" \t\r");
        if (start == std::string::npos || line[start] == '#') {
            continue;   // blank or comment
        }
        const std::string trimmed = line.substr(start);

        // VERSION
        if (trimmed.rfind("VERSION", 0) == 0) {
            foundVersion = true;
            continue;
        }

        // RESOLUTION XY <xy_cm> H <h_cm>
        if (trimmed.rfind("RESOLUTION", 0) == 0) {
            std::istringstream ss(trimmed.substr(10));
            std::string xylabel, hlabel;
            double xy{1.0}, h{1.0};
            if (ss >> xylabel >> xy >> hlabel >> h) {
                result.resolutionXY = xy;
                result.resolutionH  = h;
            } else {
                logger.Log("map file line " + std::to_string(lineNum) +
                           ": malformed RESOLUTION line — using defaults (1.0 cm)");
            }
            continue;
        }

        // END
        if (trimmed == "END") {
            break;
        }

        // BOUNDS xmin=.. xmax=.. ymin=.. ymax=.. hmin=.. hmax=..
        if (trimmed.rfind("BOUNDS", 0) == 0) {
            std::istringstream ss(trimmed.substr(6));
            std::string token;
            // Parse raw doubles first, then wrap in Centi
            double xmin{0}, xmax{0}, ymin{0}, ymax{0}, hmin{0}, hmax{0};
            while (ss >> token) {
                bool matched =
                    ParseBoundsToken(token, "xmin", xmin) ||
                    ParseBoundsToken(token, "xmax", xmax) ||
                    ParseBoundsToken(token, "ymin", ymin) ||
                    ParseBoundsToken(token, "ymax", ymax) ||
                    ParseBoundsToken(token, "hmin", hmin) ||
                    ParseBoundsToken(token, "hmax", hmax);
                if (!matched) {
                    logger.Log("map file line " + std::to_string(lineNum) +
                               ": unrecognised BOUNDS token '" + token + "' — ignored");
                }
            }
            result.bounds = {
                xmin * si::centi<si::metre>, xmax * si::centi<si::metre>,
                ymin * si::centi<si::metre>, ymax * si::centi<si::metre>,
                hmin * si::centi<si::metre>, hmax * si::centi<si::metre>
            };
            foundBounds = true;
            continue;
        }

        // CELL x y h value
        if (trimmed.rfind("CELL", 0) == 0) {
            std::istringstream ss(trimmed.substr(4));
            double rawX{}, rawY{}, rawH{};
            int    rawValue{};
            if (!(ss >> rawX >> rawY >> rawH >> rawValue)) {
                logger.Log("map file line " + std::to_string(lineNum) +
                           ": malformed CELL entry — skipped");
                continue;
            }
            result.cells.push_back({
                rawX * si::centi<si::metre>,
                rawY * si::centi<si::metre>,
                rawH * si::centi<si::metre>,
                IntToMapValue(rawValue)
            });
            continue;
        }

        // Unknown line
        logger.Log("map file line " + std::to_string(lineNum) +
                   ": unrecognised token '" + trimmed.substr(0, 20) + "' — skipped");
    }

    if (!foundVersion) {
        std::cerr << "Error: map file missing VERSION header: " << filePath << "\n";
        return result;   // valid = false
    }

    if (!foundBounds) {
        logger.Log("map file: BOUNDS line missing — defaulting to all zeros");
    }

    result.valid = true;
    return result;
}

// ---------------------------------------------------------------------------
// WriteMapFile
// ---------------------------------------------------------------------------

bool WriteMapFile(const std::filesystem::path& filePath,
                  const MapBounds&              bounds,
                  const std::vector<MapCell>&   cells,
                  double                        resolutionXY,
                  double                        resolutionH)
{
    std::ofstream file(filePath);
    if (!file.is_open()) {
        std::cerr << "Error: cannot write map file: " << filePath << "\n";
        return false;
    }

    // Determine decimal places from resolution (e.g. 1.0->0, 0.1->1, 0.01->2)
    auto decimals = [](double res) -> int {
        if (res >= 1.0) return 0;
        return static_cast<int>(std::ceil(-std::log10(res)));
    };
    const int dpXY = decimals(resolutionXY);
    const int dpH  = decimals(resolutionH);

    const auto cm = [](Centi v) {
        return v.numerical_value_in(si::centi<si::metre>);
    };

    file << "VERSION 2\n";
    file << std::fixed;
    file.precision(6);
    file << "RESOLUTION XY " << resolutionXY << " H " << resolutionH << "\n";

    file.precision(dpXY);
    file << "BOUNDS"
         << " xmin=" << cm(bounds.xmin) << " xmax=" << cm(bounds.xmax)
         << " ymin=" << cm(bounds.ymin) << " ymax=" << cm(bounds.ymax);
    file.precision(dpH);
    file << " hmin=" << cm(bounds.hmin) << " hmax=" << cm(bounds.hmax)
         << "\n";

    for (const auto& cell : cells) {
        file.precision(dpXY);
        file << "CELL "
             << cm(cell.x) << " "
             << cm(cell.y) << " ";
        file.precision(dpH);
        file << cm(cell.height) << " "
             << MapValueToInt(cell.value) << "\n";
    }

    return true;
}

} // namespace drone
