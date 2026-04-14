#pragma once

#include "types/Units.h"
#include "types/MapValue.h"

namespace drone {

class IBuildingMap {
public:
    virtual ~IBuildingMap() = default;

    // Returns the mapped value at the given 3D position.
    // Result is based ONLY on values previously set by the drone — never on
    // the ground-truth simulation input.
    virtual MapValue Get(Centi x, Centi y, Centi height) const = 0;

    // Record a mapped value at the given 3D position.
    // Positions outside the mission boundary/height range are silently ignored.
    virtual void Set(Centi x, Centi y, Centi height, MapValue value) = 0;
};

} // namespace drone
