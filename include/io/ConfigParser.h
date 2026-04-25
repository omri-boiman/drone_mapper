#pragma once

#include <filesystem>
#include <vector>
#include <utility>   // std::pair
#include "io/ErrorLogger.h"
#include "types/Units.h"

namespace drone {

// ---------------------------------------------------------------------------
// DroneConfig — capabilities loaded from drone_config.txt
// ---------------------------------------------------------------------------
struct DroneConfig {
    // Minimum passage dimensions the drone will attempt to enter
    Centi   minPassWidth  {60.0  * si::centi<si::metre>};
    Centi   minPassLength {60.0  * si::centi<si::metre>};
    Centi   minPassHeight {120.0 * si::centi<si::metre>};

    // Lidar range (Z-min / Z-max)
    Centi   lidarMinRange {20.0   * si::centi<si::metre>};  // Z-min
    Centi   lidarMaxRange {1000.0 * si::centi<si::metre>};  // Z-max

    // v2 circular beam model parameters
    Centi   lidarD    {5.0 * si::centi<si::metre>}; // beam-circle spacing at Z-min
    int     lidarFovc {5};                           // number of beam circles (0=centre only)

    // v1 fields kept for backward-compat parsing (not used by the v2 sensor)
    Degrees lidarFov        {90.0   * si::degree};
    Centi   lidarResAtDist1 {5.0    * si::centi<si::metre>};
    Centi   lidarDist1      {100.0  * si::centi<si::metre>};
    Centi   lidarResAtDist2 {20.0   * si::centi<si::metre>};
    Centi   lidarDist2      {800.0  * si::centi<si::metre>};

    // Maximum movement per single command
    Degrees maxRotate {45.0 * si::degree};
    Centi   maxAdvance{50.0 * si::centi<si::metre>};
    Centi   maxElevate{30.0 * si::centi<si::metre>};
};

// ---------------------------------------------------------------------------
// MissionConfig — per-mission settings loaded from mission_config.txt
// ---------------------------------------------------------------------------
struct MissionConfig {
    // Boundary polygon as a list of (x, y) vertices in cm.
    // Stored as raw doubles for use in the geometric ray-casting algorithm.
    std::vector<std::pair<double, double>> boundaryPolygon;

    // Height limits
    Centi minHeight {0.0   * si::centi<si::metre>};
    Centi maxHeight {300.0 * si::centi<si::metre>};

    // Output map resolution (decimal places — dimensionless counts)
    int outputResXYDecimals {2};
    int outputResHDecimals  {2};

    // Drone start position
    Centi startX      {0.0   * si::centi<si::metre>};
    Centi startY      {0.0   * si::centi<si::metre>};
    Centi startHeight {150.0 * si::centi<si::metre>};
};

// ---------------------------------------------------------------------------
// Parsing functions
// ---------------------------------------------------------------------------

// Parse drone_config.txt from filePath.
// Missing or malformed keys are replaced by defaults and logged.
// Returns false only if the file cannot be opened (unrecoverable).
bool ParseDroneConfig(const std::filesystem::path& filePath,
                      DroneConfig&                 out,
                      ErrorLogger&                 logger);

// Parse mission_config.txt from filePath.
// Missing or malformed keys are replaced by defaults and logged.
// Returns false only if the file cannot be opened (unrecoverable).
bool ParseMissionConfig(const std::filesystem::path& filePath,
                        MissionConfig&               out,
                        ErrorLogger&                 logger);

} // namespace drone
