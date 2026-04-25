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

    const int maxSteps = static_cast<int>(maxRange) + 1;
    for (int t = 1; t <= maxSteps; ++t) {
        const double cx = originX + t * dx;
        const double cy = originY + t * dy;
        const double ch = originH + t * dz;

        if (m_groundTruth.IsOccupied(
                cx * si::centi<si::metre>,
                cy * si::centi<si::metre>,
                ch * si::centi<si::metre>)) {
            const double dist = static_cast<double>(t);
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
    // 1. Resolve absolute scan direction
    const double headingDeg  = m_state->orientation.heading.numerical_value_in(si::degree);
    const double xyOffsetDeg = xy_angle.has_value()
        ? xy_angle->numerical_value_in(si::degree) : 0.0;
    const double azimuthDeg  = headingDeg + xyOffsetDeg;
    const double azimuthRad  = azimuthDeg * std::numbers::pi / 180.0;

    const double pitchDeg = pitch.has_value()
        ? pitch->numerical_value_in(si::degree) : 0.0;
    const double pitchRad = pitchDeg * std::numbers::pi / 180.0;

    // 2. Central beam unit vector  (cx, cy, cz)
    const double cosPitch = std::cos(pitchRad);
    const double sinPitch = std::sin(pitchRad);
    const double cosAz    = std::cos(azimuthRad);
    const double sinAz    = std::sin(azimuthRad);

    const double cx = cosPitch * cosAz;
    const double cy = cosPitch * sinAz;
    const double cz = sinPitch;

    // 3. Two orthogonal vectors in the plane perpendicular to central direction.
    //    u = horizontal perpendicular; w = central × u.
    double ux, uy, uz;
    if (std::abs(cz) < 0.9) {
        const double len = std::sqrt(cx * cx + cy * cy);
        ux = -cy / len;  uy = cx / len;  uz = 0.0;
    } else {
        // Near-vertical scan: anchor to +X to avoid degeneracy
        ux = 1.0;  uy = 0.0;  uz = 0.0;
    }
    const double wx = cy * uz - cz * uy;
    const double wy = cz * ux - cx * uz;
    const double wz = cx * uy - cy * ux;

    // 4. Lidar parameters
    const double zmin = m_config.lidarMinRange.numerical_value_in(si::centi<si::metre>);
    const double D    = m_config.lidarD.numerical_value_in(si::centi<si::metre>);
    const int    fovc = m_config.lidarFovc;

    // Angular step between consecutive circles = atan(D / Z-min)
    const double angStep = std::atan(D / zmin);

    // 5. Drone origin
    const double ox = m_state->position.x.numerical_value_in(si::centi<si::metre>);
    const double oy = m_state->position.y.numerical_value_in(si::centi<si::metre>);
    const double oh = m_state->position.height.numerical_value_in(si::centi<si::metre>);

    // 6. Emit beams circle by circle
    LidarScanResult hits;

    for (int circle = 0; circle < fovc; ++circle) {
        const double theta    = circle * angStep;
        const int    numBeams = (circle == 0)
            ? 1
            : static_cast<int>(std::round(std::pow(4.0, circle)));

        for (int beam = 0; beam < numBeams; ++beam) {
            const double phi = (numBeams > 1)
                ? (2.0 * std::numbers::pi * beam / numBeams)
                : 0.0;

            // Beam direction: rotate central by theta around axis at azimuth phi
            const double sinTheta = std::sin(theta);
            const double cosTheta = std::cos(theta);

            const double bx = cosTheta * cx + sinTheta * (std::cos(phi) * ux + std::sin(phi) * wx);
            const double by = cosTheta * cy + sinTheta * (std::cos(phi) * uy + std::sin(phi) * wy);
            const double bz = cosTheta * cz + sinTheta * (std::cos(phi) * uz + std::sin(phi) * wz);

            const double dist = CastRay(ox, oy, oh, bx, by, bz);
            if (dist == -1.0) continue;  // no hit within Z-max

            // Absolute azimuth and elevation of this beam
            const double beamAz = std::atan2(by, bx);
            const double beamEl = std::atan2(bz, std::sqrt(bx * bx + by * by));

            hits.push_back({
                beamAz * (180.0 / std::numbers::pi) * si::degree,
                beamEl * (180.0 / std::numbers::pi) * si::degree,
                (dist == -2.0) ? 0.0 : dist  // 0.0 = within Z-min
            });
        }
    }

    return hits;
}

} // namespace drone
