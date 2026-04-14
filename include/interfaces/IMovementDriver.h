#pragma once

#include "types/Units.h"

namespace drone {

// Returned by every movement call to indicate whether the move succeeded
enum class MoveResult {
    Success,          // move completed normally
    CollisionDetected // drone hit an element — simulation must end with failure notice
};

class IMovementDriver {
public:
    virtual ~IMovementDriver() = default;

    // Rotate in the XY plane. Positive = clockwise, negative = counter-clockwise.
    // Clamped to max_rotate_deg per call.
    virtual MoveResult Rotate(Degrees angle) = 0;

    // Move along current heading. Positive = forward, negative = backward.
    // Clamped to max_advance_cm per call.
    virtual MoveResult Advance(Centi distance) = 0;

    // Move vertically. Positive = up, negative = down.
    // Clamped to max_elevate_cm per call.
    virtual MoveResult Elevate(Centi distance) = 0;
};

} // namespace drone
