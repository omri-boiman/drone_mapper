#include "simulation/MockPositionSensor.h"

namespace drone {

MockPositionSensor::MockPositionSensor(std::shared_ptr<SimulationState> state)
    : m_state(std::move(state))
{}

Position3D MockPositionSensor::GetPosition() const
{
    return m_state->position;
}

} // namespace drone
