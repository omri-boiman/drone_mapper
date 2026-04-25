#include "simulation/MockPositionSensor.h"

namespace drone {

MockPositionSensor::MockPositionSensor(std::shared_ptr<SimulationState> state)
    : m_state(std::move(state))
{}

Position3D MockPositionSensor::GetPosition() const
{
    Position3D pos = m_state->position;
    pos.heading = m_state->orientation.heading;
    return pos;
}

} // namespace drone
