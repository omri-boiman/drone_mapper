#pragma once

#include <memory>
#include "interfaces/IPositionSensor.h"
#include "simulation/SimulationState.h"

namespace drone {

// ---------------------------------------------------------------------------
// MockPositionSensor
//
// Reads the drone's current position directly from the shared SimulationState.
// The drone algorithm only sees the IPositionSensor interface.
// ---------------------------------------------------------------------------
class MockPositionSensor : public IPositionSensor {
public:
    explicit MockPositionSensor(std::shared_ptr<SimulationState> state);

    Position3D GetPosition() const override;

private:
    std::shared_ptr<SimulationState> m_state;
};

} // namespace drone
