#include "simulation/MockLidarSensor.h"

#include <cmath>
#include <numbers>

namespace drone {

MockLidarSensor::MockLidarSensor(std::shared_ptr<SimulationState> state,
                                 const DroneConfig&               config,
                                 const GroundTruthMap&            groundTruth)
    : m_state(std::move(state))
    , m_config(config)
    , m_groundTruth(groundTruth)
{}

// ---------------------------------------------------------------------------

double MockLidarSensor::CastRay(double originX, double originY, double originH,
                                double dx,      double dy,      double dz) const
{
    const double minRange = m_config.lidarMinRange.numerical_value_in(si::centi<si::metre>);
    const double maxRange = m_config.lidarMaxRange.numerical_value_in(si::centi<si::metre>);

    const double step     = 0.1;
    const int    maxSteps = static_cast<int>(maxRange / step) + 1;

    for (int t = 1; t <= maxSteps; ++t) {
        const double dist = t * step;
        const double cx = originX + dist * dx;
        const double cy = originY + dist * dy;
        const double ch = originH + dist * dz;

        if (m_groundTruth.IsOccupied(
                cx * si::centi<si::metre>,
                cy * si::centi<si::metre>,
                ch * si::centi<si::metre>)) {
            if (dist < minRange) return -2.0;
            if (dist > maxRange) return -1.0;
            return dist;
        }
    }
    return -1.0;
}

// ---------------------------------------------------------------------------

LidarScanResult MockLidarSensor::Scan(std::optional<Degrees> xy_angle,
                                       std::optional<Degrees> pitch)
{
    const double headingDeg  = m_state->orientation.heading.numerical_value_in(si::degree);
    const double relHorizDeg = xy_angle.has_value() ? xy_angle->numerical_value_in(si::degree) : 0.0;
    const double relAltDeg   = pitch.has_value()    ? pitch->numerical_value_in(si::degree)    : 0.0;

    const double zmin = m_config.lidarMinRange.numerical_value_in(si::centi<si::metre>);
    const double D    = m_config.lidarD.numerical_value_in(si::centi<si::metre>);
    const int    fovc = m_config.lidarFovc;

    const double ox = m_state->position.x.numerical_value_in(si::centi<si::metre>);
    const double oy = m_state->position.y.numerical_value_in(si::centi<si::metre>);
    const double oh = m_state->position.height.numerical_value_in(si::centi<si::metre>);

    LidarScanResult hits;

    for (int circle = 0; circle < fovc; ++circle) {
        const int    numBeams = (circle == 0) ? 1 : static_cast<int>(std::round(std::pow(4.0, circle)));
        const double radius   = static_cast<double>(circle) * D;

        for (int beam = 0; beam < numBeams; ++beam) {
            const double phi = (numBeams > 1)
                ? (2.0 * std::numbers::pi * beam / numBeams)
                : 0.0;

            // Professor's 2D angle-space formula
            const double horizOffset  = radius * std::cos(phi);
            const double altOffset    = radius * std::sin(phi);
            const double horizDeltaDeg = std::atan2(horizOffset, zmin) * (180.0 / std::numbers::pi);
            const double altDeltaDeg   = std::atan2(altOffset,   zmin) * (180.0 / std::numbers::pi);

            // Absolute direction for ray tracing through the map
            const double absHorizRad = (headingDeg + relHorizDeg + horizDeltaDeg) * std::numbers::pi / 180.0;
            const double absAltRad   = (relAltDeg  + altDeltaDeg)                 * std::numbers::pi / 180.0;

            const double cosAlt = std::cos(absAltRad);
            const double dx = cosAlt * std::cos(absHorizRad);
            const double dy = cosAlt * std::sin(absHorizRad);
            const double dz = std::sin(absAltRad);

            const double dist = CastRay(ox, oy, oh, dx, dy, dz);
            if (dist == -1.0) continue;

            hits.push_back({
                (relHorizDeg + horizDeltaDeg) * si::degree,
                (relAltDeg   + altDeltaDeg)   * si::degree,
                (dist == -2.0) ? 0.0 : dist
            });
        }
    }

    return hits;
}

} // namespace drone
