#pragma once

#include "types/Units.h"

namespace drone {

// ---------------------------------------------------------------------------
// SimulationState
//
// Shared mutable state carried by all three mock objects.
// MockMovementDriver writes to it; MockPositionSensor and MockLidarSensor
// read from it.  All three hold a shared_ptr<SimulationState>.
// ---------------------------------------------------------------------------
struct SimulationState {
    Position3D  position;     // current drone position in cm
    Orientation orientation;  // heading (0-360 deg, clockwise from +X) and pitch
};

} // namespace drone
