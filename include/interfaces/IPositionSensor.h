#pragma once

#include "types/Units.h"

namespace drone {

class IPositionSensor {
public:
    virtual ~IPositionSensor() = default;

    // Returns the drone's exact current position
    virtual Position3D GetPosition() const = 0;
};

} // namespace drone
