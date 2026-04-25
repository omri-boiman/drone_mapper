#pragma once

#include <mp-units/systems/si.h>
#include <mp-units/systems/isq.h>
#include <mp-units/math.h>

namespace drone {

using namespace mp_units;
using namespace mp_units::si::unit_symbols;  // m, cm, deg, etc.

// Convenience quantity types
using Meters    = quantity<si::metre,        double>;
using Centi     = quantity<si::centi<si::metre>, double>;
using Degrees   = quantity<si::degree,       double>;

// 3D position: X, Y in the horizontal plane, Height on the vertical axis.
// heading is also included per v2 spec (position sensor now reports XY-Angle).
struct Position3D {
    Centi   x;
    Centi   y;
    Centi   height;
    Degrees heading {0.0 * si::degree};
};

// Drone orientation: heading angle in the XY plane (0 = positive X axis, clockwise)
struct Orientation {
    Degrees heading;   // horizontal heading, 0-360
    Degrees pitch;     // vertical tilt (0 = horizontal)
};

} // namespace drone
