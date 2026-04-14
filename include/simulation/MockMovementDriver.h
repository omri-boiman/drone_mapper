#pragma once

#include <memory>
#include "interfaces/IMovementDriver.h"
#include "simulation/SimulationState.h"
#include "simulation/GroundTruthMap.h"
#include "io/ConfigParser.h"

namespace drone {

// ---------------------------------------------------------------------------
// MockMovementDriver
//
// Implements IMovementDriver against the shared SimulationState.
// Each call is clamped to the configured maximums before execution.
// The ground-truth map is walked in 1 cm steps; a collision returns
// CollisionDetected without updating the state (the main loop must then
// issue a failure notice and end the simulation).
// ---------------------------------------------------------------------------
class MockMovementDriver : public IMovementDriver {
public:
    MockMovementDriver(std::shared_ptr<SimulationState> state,
                       const DroneConfig&               config,
                       const GroundTruthMap&            groundTruth);

    // Rotate in the XY plane. Clamped to maxRotateDeg.
    // Positive = clockwise; always succeeds (no geometry check needed).
    MoveResult Rotate(Degrees angle) override;

    // Advance along current heading. Clamped to maxAdvanceCm.
    // Walks in 1 cm steps; returns CollisionDetected if any step is occupied.
    MoveResult Advance(Centi distance) override;

    // Move vertically. Clamped to maxElevateCm.
    // Walks in 1 cm steps; returns CollisionDetected if any step is occupied.
    MoveResult Elevate(Centi distance) override;

private:
    std::shared_ptr<SimulationState> m_state;
    const DroneConfig&               m_config;
    const GroundTruthMap&            m_groundTruth;
};

} // namespace drone
