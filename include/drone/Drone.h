#pragma once

#include <optional>
#include "interfaces/ILidarSensor.h"
#include "interfaces/IPositionSensor.h"
#include "interfaces/IMovementDriver.h"
#include "interfaces/IBuildingMap.h"
#include "types/Units.h"
#include "types/MapValue.h"

namespace drone {

// ---------------------------------------------------------------------------
// Drone — hardware abstraction layer
//
// A thin wrapper around the four interface dependencies.
// It does NOT contain any exploration algorithm or decision logic.
//
// Usage:
//   Drone drone(lidar, position, driver, map);
//   MoveResult r = drone.Rotate(45.0 * si::degree);
//   LidarScanResult scan = drone.Scan();
//   Position3D pos = drone.GetLocation();
//
// The exploration algorithm lives in a separate class that holds a Drone&
// and calls these methods.  This keeps the Drone reusable across assignments.
// ---------------------------------------------------------------------------

class Drone {
public:
    // Inject all four dependencies.
    // The Drone holds references — it does not own the objects.
    Drone(ILidarSensor&    lidar,
          IPositionSensor& position,
          IMovementDriver& driver,
          IBuildingMap&    map);

    // -----------------------------------------------------------------------
    // Movement
    // -----------------------------------------------------------------------

    // Rotate in the XY plane. Positive = clockwise, negative = counter-clockwise.
    // The underlying driver clamps the angle to the configured maximum per call.
    // Returns Success or CollisionDetected (rotation itself never collides, but
    // the interface keeps a uniform return type).
    MoveResult Rotate(Degrees angle);

    // Move along current heading. Positive = forward, negative = backward.
    // Clamped to max_advance_cm per call. Returns CollisionDetected if the
    // path is blocked — position is NOT updated in that case.
    MoveResult Advance(Centi distance);

    // Move vertically. Positive = up, negative = down.
    // Clamped to max_elevate_cm per call. Returns CollisionDetected if blocked.
    MoveResult Elevate(Centi distance);

    // -----------------------------------------------------------------------
    // Sensing
    // -----------------------------------------------------------------------

    // Fire the lidar and return a matrix of hit distances (cm).
    //   xy_angle absent -> scan along current heading.
    //   pitch    absent -> horizontal scan (0 deg).
    // Cell values: distance in cm, -1.0 = no hit, -2.0 = hit below min range.
    LidarScanResult Scan(std::optional<Degrees> xy_angle = std::nullopt,
                         std::optional<Degrees> pitch    = std::nullopt);

    // Ask the position sensor for the drone's current 3D position.
    Position3D GetLocation() const;

    // -----------------------------------------------------------------------
    // Map
    // -----------------------------------------------------------------------

    // Write a discovered cell value into the drone's building map.
    void RecordCell(Centi x, Centi y, Centi height, MapValue value);

    // Read back a previously recorded value (NotMapped if never set,
    // BeyondBounds if outside the mission boundary).
    MapValue QueryCell(Centi x, Centi y, Centi height) const;

private:
    ILidarSensor&    m_lidar;
    IPositionSensor& m_position;
    IMovementDriver& m_driver;
    IBuildingMap&    m_map;
};

} // namespace drone
