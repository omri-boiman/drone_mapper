#pragma once

#include <variant>
#include <optional>
#include "types/Units.h"

namespace drone {

// Rotate the drone left (negative angle) or right (positive angle)
struct RotateCmd {
    Degrees angle;
};

// Move forward (positive) or backward (negative) along current heading
struct AdvanceCmd {
    Centi distance;
};

// Move up (positive) or down (negative) along the vertical axis
struct ElevateCmd {
    Centi distance;
};

// Fire the lidar in a given direction relative to the current heading.
// xy_angle absent -> use current heading (0 offset).
// pitch absent    -> horizontal scan (0 degrees).
struct ScanCmd {
    std::optional<Degrees> xy_angle;
    std::optional<Degrees> pitch;
};

// Request current position from the position sensor
struct GetLocationCmd {};

// Signal that mapping is complete; the algorithm's Run() method should return after issuing this
struct FinishedCmd {};

// All commands the mapping algorithm can issue
using DroneCommand = std::variant<
    RotateCmd,
    AdvanceCmd,
    ElevateCmd,
    ScanCmd,
    GetLocationCmd,
    FinishedCmd
>;

} // namespace drone