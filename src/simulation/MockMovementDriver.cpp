#include "simulation/MockMovementDriver.h"

#include <cmath>
#include <numbers>

namespace drone {

namespace {

// Clamp a Degrees quantity to [-limit, +limit]
Degrees ClampDeg(Degrees value, Degrees limit)
{
    if (value >  limit) return  limit;
    if (value < -limit) return -limit;
    return value;
}

// Clamp a Centi quantity to [-limit, +limit]
Centi ClampCm(Centi value, Centi limit)
{
    if (value >  limit) return  limit;
    if (value < -limit) return -limit;
    return value;
}

// Wrap heading into [0, 360) degrees
Degrees WrapHeading(Degrees deg)
{
    double raw = std::fmod(deg.numerical_value_in(si::degree), 360.0);
    if (raw < 0.0) raw += 360.0;
    return raw * si::degree;
}

} // anonymous namespace

// ---------------------------------------------------------------------------

MockMovementDriver::MockMovementDriver(std::shared_ptr<SimulationState> state,
                                       const DroneConfig&               config,
                                       const GroundTruthMap&            groundTruth)
    : m_state(std::move(state))
    , m_config(config)
    , m_groundTruth(groundTruth)
{}

// ---------------------------------------------------------------------------

MoveResult MockMovementDriver::Rotate(Degrees angle)
{
    m_state->orientation.heading =
        WrapHeading(m_state->orientation.heading + ClampDeg(angle, m_config.maxRotate));
    return MoveResult::Success;
}

// ---------------------------------------------------------------------------

MoveResult MockMovementDriver::Advance(Centi distance)
{
    const Centi distClamped = ClampCm(distance, m_config.maxAdvance);
    const double distCm     = distClamped.numerical_value_in(si::centi<si::metre>);

    const double headingRad =
        m_state->orientation.heading.numerical_value_in(si::degree)
        * std::numbers::pi / 180.0;

    const double dx = distCm * std::cos(headingRad);
    const double dy = distCm * std::sin(headingRad);

    const double startX = m_state->position.x.numerical_value_in(si::centi<si::metre>);
    const double startY = m_state->position.y.numerical_value_in(si::centi<si::metre>);
    const double height = m_state->position.height.numerical_value_in(si::centi<si::metre>);

    const int steps = static_cast<int>(std::abs(distCm)) + 1;
    for (int i = 1; i <= steps; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(steps);
        if (m_groundTruth.IsOccupied(
                (startX + t * dx) * si::centi<si::metre>,
                (startY + t * dy) * si::centi<si::metre>,
                height            * si::centi<si::metre>)) {
            return MoveResult::CollisionDetected;
        }
    }

    m_state->position.x = (startX + dx) * si::centi<si::metre>;
    m_state->position.y = (startY + dy) * si::centi<si::metre>;
    return MoveResult::Success;
}

// ---------------------------------------------------------------------------

MoveResult MockMovementDriver::Elevate(Centi distance)
{
    const Centi  distClamped = ClampCm(distance, m_config.maxElevate);
    const double distCm      = distClamped.numerical_value_in(si::centi<si::metre>);

    const double x      = m_state->position.x.numerical_value_in(si::centi<si::metre>);
    const double y      = m_state->position.y.numerical_value_in(si::centi<si::metre>);
    const double startH = m_state->position.height.numerical_value_in(si::centi<si::metre>);

    const int steps = static_cast<int>(std::abs(distCm)) + 1;
    for (int i = 1; i <= steps; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(steps);
        if (m_groundTruth.IsOccupied(
                x              * si::centi<si::metre>,
                y              * si::centi<si::metre>,
                (startH + t * distCm) * si::centi<si::metre>)) {
            return MoveResult::CollisionDetected;
        }
    }

    m_state->position.height = (startH + distCm) * si::centi<si::metre>;
    return MoveResult::Success;
}

} // namespace drone
