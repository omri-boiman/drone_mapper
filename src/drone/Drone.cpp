#include "drone/Drone.h"

namespace drone {

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

Drone::Drone(ILidarSensor&    lidar,
             IPositionSensor& position,
             IMovementDriver& driver,
             IBuildingMap&    map)
    : m_lidar(lidar)
    , m_position(position)
    , m_driver(driver)
    , m_map(map)
{}

// ---------------------------------------------------------------------------
// Movement — delegate directly to the movement driver
// ---------------------------------------------------------------------------

MoveResult Drone::Rotate(Degrees angle)
{
    return m_driver.Rotate(angle);
}

MoveResult Drone::Advance(Centi distance)
{
    return m_driver.Advance(distance);
}

MoveResult Drone::Elevate(Centi distance)
{
    return m_driver.Elevate(distance);
}

// ---------------------------------------------------------------------------
// Sensing — delegate directly to the sensors
// ---------------------------------------------------------------------------

LidarScanResult Drone::Scan(std::optional<Degrees> xy_angle,
                             std::optional<Degrees> pitch)
{
    return m_lidar.Scan(xy_angle, pitch);
}

Position3D Drone::GetLocation() const
{
    return m_position.GetPosition();
}

// ---------------------------------------------------------------------------
// Map — delegate directly to the building map
// ---------------------------------------------------------------------------

void Drone::RecordCell(Centi x, Centi y, Centi height, MapValue value)
{
    m_map.Set(x, y, height, value);
}

MapValue Drone::QueryCell(Centi x, Centi y, Centi height) const
{
    return m_map.Get(x, y, height);
}

} // namespace drone
