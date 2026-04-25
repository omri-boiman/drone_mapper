//
// sensor_test — smoke test for Phase 2 mocks and Phase 3 BuildingMapImpl.
//
// Scenario:
//   Ground-truth wall: a block of Occupied cells at x=300, y=80..120, h=100..200 cm.
//   Drone starts at (100, 100, 150) cm, heading 0 degrees (+X direction).
//   No GTest required — prints PASS / FAIL for each check.
//

#include <iostream>
#include <memory>
#include <cmath>
#include <string>

#include "simulation/SimulationState.h"
#include "simulation/MockPositionSensor.h"
#include "simulation/MockMovementDriver.h"
#include "simulation/MockLidarSensor.h"
#include "drone/BuildingMapImpl.h"
#include "drone/Drone.h"
#include "io/MapIO.h"
#include "io/ConfigParser.h"
#include "simulation/GroundTruthMap.h"

using namespace drone;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static int g_pass = 0;
static int g_fail = 0;

static void Check(const std::string& label, bool condition)
{
    if (condition) {
        std::cout << "  PASS  " << label << "\n";
        ++g_pass;
    } else {
        std::cout << "  FAIL  " << label << "\n";
        ++g_fail;
    }
}

// Build a DroneConfig with convenient defaults for testing
static DroneConfig MakeTestDroneConfig()
{
    DroneConfig c;
    // v2 lidar model
    c.lidarMinRange = 20.0  * si::centi<si::metre>;
    c.lidarMaxRange = 500.0 * si::centi<si::metre>;
    c.lidarD        = 5.0   * si::centi<si::metre>;
    c.lidarFovc     = 5;
    // v1 backward-compat fields (not used by the new sensor)
    c.lidarFov        = 90.0  * si::degree;
    c.lidarResAtDist1 = 5.0   * si::centi<si::metre>;
    c.lidarDist1      = 100.0 * si::centi<si::metre>;
    c.lidarResAtDist2 = 20.0  * si::centi<si::metre>;
    c.lidarDist2      = 800.0 * si::centi<si::metre>;
    c.maxRotate  = 45.0  * si::degree;
    c.maxAdvance = 100.0 * si::centi<si::metre>;
    c.maxElevate = 50.0  * si::centi<si::metre>;
    return c;
}

// Build a ParsedMap with a wall: Occupied cells at x=300, y=80..120 (step 1), h=100..200 (step 1)
static ParsedMap MakeWallMap()
{
    ParsedMap pm;
    pm.valid = true;
    pm.bounds = {
          0 * si::centi<si::metre>, 500 * si::centi<si::metre>,
          0 * si::centi<si::metre>, 500 * si::centi<si::metre>,
          0 * si::centi<si::metre>, 300 * si::centi<si::metre>
    };

    for (int y = 80; y <= 120; ++y)
        for (int h = 100; h <= 200; ++h)
            pm.cells.push_back({
                300.0 * si::centi<si::metre>,
                static_cast<double>(y) * si::centi<si::metre>,
                static_cast<double>(h) * si::centi<si::metre>,
                MapValue::Occupied
            });
    return pm;
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

static void TestPositionSensor()
{
    std::cout << "\n-- MockPositionSensor --\n";

    auto state = std::make_shared<SimulationState>();
    state->position = { 100.0 * si::centi<si::metre>,
                        200.0 * si::centi<si::metre>,
                        150.0 * si::centi<si::metre> };

    MockPositionSensor sensor(state);
    const Position3D pos = sensor.GetPosition();

    Check("x == 100 cm",
          pos.x.numerical_value_in(si::centi<si::metre>) == 100.0);
    Check("y == 200 cm",
          pos.y.numerical_value_in(si::centi<si::metre>) == 200.0);
    Check("height == 150 cm",
          pos.height.numerical_value_in(si::centi<si::metre>) == 150.0);

    // Mutate state and verify sensor reflects the change
    state->position.x = 999.0 * si::centi<si::metre>;
    Check("x tracks state mutation",
          sensor.GetPosition().x.numerical_value_in(si::centi<si::metre>) == 999.0);
}

static void TestMovementDriver()
{
    std::cout << "\n-- MockMovementDriver --\n";

    const DroneConfig  config = MakeTestDroneConfig();
    const GroundTruthMap gtMap(MakeWallMap());

    auto state = std::make_shared<SimulationState>();
    state->position    = { 100.0 * si::centi<si::metre>,
                           100.0 * si::centi<si::metre>,
                           150.0 * si::centi<si::metre> };
    state->orientation = { 0.0 * si::degree, 0.0 * si::degree };

    MockMovementDriver driver(state, config, gtMap);

    // --- Rotate: request 60 deg, max is 45 --- should clamp to 45
    const MoveResult rotResult = driver.Rotate(60.0 * si::degree);
    Check("Rotate returns Success",
          rotResult == MoveResult::Success);
    Check("Rotate 60 deg clamped to 45 deg",
          std::abs(state->orientation.heading.numerical_value_in(si::degree) - 45.0) < 0.001);

    // Reset heading to 0 for straight-line advance tests
    state->orientation.heading = 0.0 * si::degree;

    // --- Advance 100 cm into free space (wall is 200 cm away) ---
    const MoveResult advOk = driver.Advance(100.0 * si::centi<si::metre>);
    Check("Advance into free space returns Success",
          advOk == MoveResult::Success);
    Check("Position updated after free advance",
          std::abs(state->position.x.numerical_value_in(si::centi<si::metre>) - 200.0) < 0.5);

    // --- Advance another 100 cm: now 100 cm from the wall at x=300 ---
    // The wall starts at x=300 so a 100 cm advance should hit it
    const MoveResult advCollide = driver.Advance(100.0 * si::centi<si::metre>);
    Check("Advance into wall returns CollisionDetected",
          advCollide == MoveResult::CollisionDetected);
    Check("Position NOT updated on collision",
          std::abs(state->position.x.numerical_value_in(si::centi<si::metre>) - 200.0) < 0.5);

    // --- Elevate into free space ---
    const double startH = state->position.height.numerical_value_in(si::centi<si::metre>);
    const MoveResult elevOk = driver.Elevate(30.0 * si::centi<si::metre>);
    Check("Elevate into free space returns Success",
          elevOk == MoveResult::Success);
    Check("Height updated after elevate",
          std::abs(state->position.height.numerical_value_in(si::centi<si::metre>)
                   - (startH + 30.0)) < 0.5);
}

static void TestLidarSensor()
{
    std::cout << "\n-- MockLidarSensor --\n";

    const DroneConfig    config = MakeTestDroneConfig();
    const GroundTruthMap gtMap(MakeWallMap());

    auto state = std::make_shared<SimulationState>();
    // Drone at (100, 100, 150), heading 0 deg — pointing directly at the wall at x=300
    state->position    = { 100.0 * si::centi<si::metre>,
                           100.0 * si::centi<si::metre>,
                           150.0 * si::centi<si::metre> };
    state->orientation = { 0.0 * si::degree, 0.0 * si::degree };

    MockLidarSensor lidar(state, config, gtMap);

    // Default scan along heading (0 deg = +X direction). Wall is 200 cm away.
    const LidarScanResult result = lidar.Scan();

    Check("Scan returns some hits", !result.empty());

    // The centre beam (azimuth≈0°, elevation≈0°) must hit the wall at ~200 cm
    bool foundCentreHit = false;
    for (const auto& hit : result) {
        const double az = hit.azimuth.numerical_value_in(si::degree);
        const double el = hit.elevation.numerical_value_in(si::degree);
        if (std::abs(az) < 5.0 && std::abs(el) < 5.0 &&
            hit.distance >= 195.0 && hit.distance <= 205.0) {
            foundCentreHit = true;
            break;
        }
    }
    Check("Centre beam hits wall at ~200 cm", foundCentreHit);

    // 90-deg offset scan points in +Y; no wall in that direction → no hits
    const LidarScanResult result90 = lidar.Scan(90.0 * si::degree);
    Check("90-deg offset scan: no hits (no wall in +Y direction)", result90.empty());
}

static void TestBuildingMap()
{
    std::cout << "\n-- BuildingMapImpl --\n";

    MissionConfig mc;
    // Simple 500x500 cm square boundary, height 0–300 cm, 2 decimal places
    mc.boundaryPolygon   = { {0,0}, {500,0}, {500,500}, {0,500} };
    mc.minHeight = 0.0   * si::centi<si::metre>;
    mc.maxHeight = 300.0 * si::centi<si::metre>;
    mc.outputResXYDecimals = 2;
    mc.outputResHDecimals  = 2;

    BuildingMapImpl map(mc);

    // Fresh cell should be NotMapped
    Check("Unset cell returns NotMapped",
          map.Get(100.0 * si::centi<si::metre>,
                  100.0 * si::centi<si::metre>,
                  150.0 * si::centi<si::metre>) == MapValue::NotMapped);

    // Out of bounds cell should be BeyondBounds
    Check("Out-of-bounds cell returns BeyondBounds",
          map.Get(600.0 * si::centi<si::metre>,
                  100.0 * si::centi<si::metre>,
                  150.0 * si::centi<si::metre>) == MapValue::BeyondBounds);

    // Set a cell and read it back
    map.Set(100.0 * si::centi<si::metre>,
            100.0 * si::centi<si::metre>,
            150.0 * si::centi<si::metre>, MapValue::Occupied);
    Check("Set then Get returns Occupied",
          map.Get(100.0 * si::centi<si::metre>,
                  100.0 * si::centi<si::metre>,
                  150.0 * si::centi<si::metre>) == MapValue::Occupied);

    // Set outside bounds should be silently ignored
    map.Set(600.0 * si::centi<si::metre>,
            100.0 * si::centi<si::metre>,
            150.0 * si::centi<si::metre>, MapValue::Occupied);
    Check("Set out-of-bounds is silently ignored (still BeyondBounds)",
          map.Get(600.0 * si::centi<si::metre>,
                  100.0 * si::centi<si::metre>,
                  150.0 * si::centi<si::metre>) == MapValue::BeyondBounds);

    // Resolution round-trip: 100.001 should quantise to same cell as 100.00
    map.Set(100.001 * si::centi<si::metre>,
            100.0   * si::centi<si::metre>,
            150.0   * si::centi<si::metre>, MapValue::Empty);
    Check("100.001 cm quantises to same cell as 100.00 cm",
          map.Get(100.0 * si::centi<si::metre>,
                  100.0 * si::centi<si::metre>,
                  150.0 * si::centi<si::metre>) == MapValue::Empty);
}

// ---------------------------------------------------------------------------
// TestDrone — exercises the Drone hardware-abstraction wrapper
// ---------------------------------------------------------------------------
// Same scenario: wall at x=300, y=80..120, h=100..200.
// Drone starts at (100, 100, 150) heading 0 deg.

static void TestDrone()
{
    std::cout << "\n-- Drone (hardware abstraction layer) --\n";

    const DroneConfig    config = MakeTestDroneConfig();
    const GroundTruthMap gtMap(MakeWallMap());

    auto state = std::make_shared<SimulationState>();
    state->position    = { 100.0 * si::centi<si::metre>,
                           100.0 * si::centi<si::metre>,
                           150.0 * si::centi<si::metre> };
    state->orientation = { 0.0 * si::degree, 0.0 * si::degree };

    MockPositionSensor posSensor(state);
    MockMovementDriver driver(state, config, gtMap);
    MockLidarSensor    lidar(state, config, gtMap);

    MissionConfig mc;
    mc.boundaryPolygon  = { {0,0}, {500,0}, {500,500}, {0,500} };
    mc.minHeight        = 0.0   * si::centi<si::metre>;
    mc.maxHeight        = 300.0 * si::centi<si::metre>;
    mc.outputResXYDecimals = 2;
    mc.outputResHDecimals  = 2;
    BuildingMapImpl buildingMap(mc);

    Drone drone(lidar, posSensor, driver, buildingMap);

    // --- GetLocation returns the initial position ---
    const Position3D pos = drone.GetLocation();
    Check("GetLocation x == 100 cm",
          pos.x.numerical_value_in(si::centi<si::metre>) == 100.0);
    Check("GetLocation y == 100 cm",
          pos.y.numerical_value_in(si::centi<si::metre>) == 100.0);
    Check("GetLocation height == 150 cm",
          pos.height.numerical_value_in(si::centi<si::metre>) == 150.0);

    // --- Rotate delegates to driver (clamped to 45 deg) ---
    const MoveResult rotResult = drone.Rotate(60.0 * si::degree);
    Check("Rotate returns Success",
          rotResult == MoveResult::Success);
    Check("Rotate 60 deg clamped to 45 deg",
          std::abs(state->orientation.heading.numerical_value_in(si::degree) - 45.0) < 0.001);

    // Reset heading for straight-line tests
    state->orientation.heading = 0.0 * si::degree;

    // --- Advance into free space ---
    const MoveResult advOk = drone.Advance(100.0 * si::centi<si::metre>);
    Check("Advance into free space returns Success",
          advOk == MoveResult::Success);
    Check("Position updated after Advance",
          std::abs(drone.GetLocation().x.numerical_value_in(si::centi<si::metre>) - 200.0) < 0.5);

    // --- Advance into wall returns CollisionDetected ---
    const MoveResult advCollide = drone.Advance(100.0 * si::centi<si::metre>);
    Check("Advance into wall returns CollisionDetected",
          advCollide == MoveResult::CollisionDetected);
    Check("Position NOT updated on collision",
          std::abs(drone.GetLocation().x.numerical_value_in(si::centi<si::metre>) - 200.0) < 0.5);

    // --- Elevate ---
    const double startH = drone.GetLocation().height.numerical_value_in(si::centi<si::metre>);
    const MoveResult elevOk = drone.Elevate(30.0 * si::centi<si::metre>);
    Check("Elevate returns Success",
          elevOk == MoveResult::Success);
    Check("Height updated after Elevate",
          std::abs(drone.GetLocation().height.numerical_value_in(si::centi<si::metre>)
                   - (startH + 30.0)) < 0.5);

    // --- Scan returns hits toward the wall (~100 cm away after advance) ---
    const LidarScanResult scan = drone.Scan();
    Check("Scan returns some hits", !scan.empty());
    bool foundWallHit = false;
    for (const auto& hit : scan) {
        const double az = hit.azimuth.numerical_value_in(si::degree);
        const double el = hit.elevation.numerical_value_in(si::degree);
        if (std::abs(az) < 5.0 && std::abs(el) < 5.0 &&
            hit.distance >= 95.0 && hit.distance <= 105.0) {
            foundWallHit = true;
            break;
        }
    }
    Check("Scan centre beam hits wall at ~100 cm", foundWallHit);

    // --- RecordCell / QueryCell round-trip ---
    drone.RecordCell(200.0 * si::centi<si::metre>,
                     100.0 * si::centi<si::metre>,
                     150.0 * si::centi<si::metre>, MapValue::Occupied);
    Check("RecordCell then QueryCell returns Occupied",
          drone.QueryCell(200.0 * si::centi<si::metre>,
                          100.0 * si::centi<si::metre>,
                          150.0 * si::centi<si::metre>) == MapValue::Occupied);

    // --- QueryCell on unset location returns NotMapped ---
    Check("QueryCell on unset location returns NotMapped",
          drone.QueryCell(1.0 * si::centi<si::metre>,
                          1.0 * si::centi<si::metre>,
                          1.0 * si::centi<si::metre>) == MapValue::NotMapped);
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main()
{
    std::cout << "=== Sensor & Map smoke tests ===\n";

    TestPositionSensor();
    TestMovementDriver();
    TestLidarSensor();
    TestBuildingMap();
    TestDrone();

    std::cout << "\n================================\n";
    std::cout << "  Passed: " << g_pass << "\n";
    std::cout << "  Failed: " << g_fail << "\n";
    std::cout << "================================\n";

    return (g_fail == 0) ? 0 : 1;
}
