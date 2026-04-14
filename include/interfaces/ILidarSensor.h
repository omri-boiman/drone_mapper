#pragma once

#include <vector>
#include <optional>
#include "types/Units.h"

namespace drone {

// One row of lidar distance readings (inner = horizontal cells)
// Each value is a distance in cm, or:
//   -1.0 : no element detected within effective scan range
//   -2.0 : element detected below minimum scan distance
using LidarRow    = std::vector<double>;
using LidarMatrix = std::vector<LidarRow>;

// Full result of a single Scan command
struct LidarScanResult {
    LidarMatrix cells;     // [row][col] — row = vertical cells, col = horizontal cells
    Degrees     xy_angle;  // actual horizontal scan direction used
    Degrees     pitch;     // actual vertical scan direction used
};

class ILidarSensor {
public:
    virtual ~ILidarSensor() = default;

    // Scan in the given direction.
    // xy_angle absent -> use current drone heading (0 offset).
    // pitch absent    -> horizontal scan (0 deg).
    virtual LidarScanResult Scan(
        std::optional<Degrees> xy_angle = std::nullopt,
        std::optional<Degrees> pitch    = std::nullopt) = 0;
};

} // namespace drone
