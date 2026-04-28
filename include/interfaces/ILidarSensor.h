#pragma once

#include <vector>
#include <optional>
#include "types/Units.h"

namespace drone {

// One lidar beam that hit an object within the scan range.
// horizontal — relative horizontal angle of the beam (degrees, relative to drone heading)
// altitude   — relative vertical angle of the beam (degrees, 0 = horizontal)
// distance   — cm to the hit surface; 0.0 means hit is within Z-min (too close
//              to measure accurately)
struct LidarBeamHit {
    Degrees horizontal;
    Degrees altitude;
    double  distance;
};

// Sparse result of a single Scan: only beams that hit something are listed.
// Empty vector means no beams detected anything within Z-max.
using LidarScanResult = std::vector<LidarBeamHit>;

class ILidarSensor {
public:
    virtual ~ILidarSensor() = default;

    // Fire the lidar in the given direction.
    // xy_angle absent -> scan along current drone heading (0 offset).
    // pitch    absent -> horizontal scan (0 deg).
    // Returns the list of beams that hit something.
    virtual LidarScanResult Scan(
        std::optional<Degrees> xy_angle = std::nullopt,
        std::optional<Degrees> pitch    = std::nullopt) = 0;
};

} // namespace drone
