#include "simulation/MockLidarSensor.h"

#include <cmath>
#include <numbers>

namespace drone {

// ---------------------------------------------------------------------------

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

    // Step in 1 cm increments (matches GroundTruthMap cm quantisation)
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
    return -1.0; // no hit within maxRange
}

// ---------------------------------------------------------------------------

LidarScanResult MockLidarSensor::Scan(std::optional<Degrees> xy_angle,
                                       std::optional<Degrees> pitch)
{
    // 1. Resolve scan direction.
    //    xy_angle is an OFFSET from the current heading (assignment:
    //    "X-Y angle in reference to current direction"). Absent = 0 offset.
    const double headingDeg  = m_state->orientation.heading.numerical_value_in(si::degree);
    const double xyOffsetDeg = xy_angle.has_value()
        ? xy_angle->numerical_value_in(si::degree)
        : 0.0;
    const double xyDeg    = headingDeg + xyOffsetDeg;
    const double xyRad    = xyDeg * std::numbers::pi / 180.0;

    const double pitchDeg = pitch.has_value()
        ? pitch->numerical_value_in(si::degree)
        : 0.0;
    const double pitchRad = pitchDeg * std::numbers::pi / 180.0;

    // 2. Compute matrix dimensions from FOV and resolution at dist1
    const double fovRad        = m_config.lidarFov.numerical_value_in(si::degree)
                                 * std::numbers::pi / 180.0;
    const double res1          = m_config.lidarResAtDist1.numerical_value_in(si::centi<si::metre>);
    const double dist1         = m_config.lidarDist1.numerical_value_in(si::centi<si::metre>);
    const double deltaAngleRad = std::atan(res1 / dist1);
    const int    half          = static_cast<int>(std::floor(fovRad / (2.0 * deltaAngleRad)));
    const int    numCells      = 2 * half + 1; // odd, centred

    // 3. Drone origin
    const double ox = m_state->position.x.numerical_value_in(si::centi<si::metre>);
    const double oy = m_state->position.y.numerical_value_in(si::centi<si::metre>);
    const double oh = m_state->position.height.numerical_value_in(si::centi<si::metre>);

    // 4. Build matrix [row = vertical index, col = horizontal index]
    LidarMatrix matrix(static_cast<std::size_t>(numCells),
                       LidarRow(static_cast<std::size_t>(numCells), -1.0));

    for (int row = 0; row < numCells; ++row) {
        const double rowOffset  = (row - half) * deltaAngleRad; // + = upward
        const double totalPitch = pitchRad + rowOffset;

        for (int col = 0; col < numCells; ++col) {
            const double colOffset = (col - half) * deltaAngleRad;
            const double totalXY   = xyRad + colOffset;

            const double horiz = std::cos(totalPitch);
            const double dx    = horiz * std::cos(totalXY);
            const double dy    = horiz * std::sin(totalXY);
            const double dz    = std::sin(totalPitch);

            matrix[static_cast<std::size_t>(row)][static_cast<std::size_t>(col)] =
                CastRay(ox, oy, oh, dx, dy, dz);
        }
    }

    // Return actual absolute scan angles so the caller can interpret the matrix
    return LidarScanResult{
        .cells    = std::move(matrix),
        .xy_angle = xyDeg    * si::degree,
        .pitch    = pitchDeg * si::degree
    };
}

} // namespace drone
