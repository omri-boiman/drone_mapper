//
// test_mapping — smoke test for Phase 5 MappingAlgorithm.
//
// Scenario A (wall): wall at x=300, y=80..120, h=100..200.
//   Drone starts at (100, 100, 150), heading 0 deg (+X).
//   outputResXYDecimals=0 → m_resolution=1.0 → BFS steps = 100 cm world.
//
// Scenario B (open): no obstacles, 600×600 cm room.
//   Drone starts at (300, 300, 150).
//
// No GTest required — prints PASS / FAIL for each check.
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
#include "drone/MappingAlgorithm.h"
#include "simulation/GroundTruthMap.h"
#include "io/MapIO.h"
#include "io/ConfigParser.h"

using namespace drone;

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

// ---------------------------------------------------------------------------
// Shared config builders
// ---------------------------------------------------------------------------

static DroneConfig MakeTestDroneConfig()
{
    DroneConfig c;
    c.lidarFov        = 90.0  * si::degree;
    c.lidarMinRange   = 20.0  * si::centi<si::metre>;
    c.lidarMaxRange   = 500.0 * si::centi<si::metre>;
    c.lidarResAtDist1 = 5.0   * si::centi<si::metre>;
    c.lidarDist1      = 100.0 * si::centi<si::metre>;
    c.lidarResAtDist2 = 20.0  * si::centi<si::metre>;
    c.lidarDist2      = 800.0 * si::centi<si::metre>;
    c.maxRotate       = 45.0  * si::degree;
    c.maxAdvance      = 100.0 * si::centi<si::metre>;
    c.maxElevate      = 50.0  * si::centi<si::metre>;
    return c;
}

// Wall: Occupied at x=300, y=80..120, h=100..200 cm
static ParsedMap MakeWallMap()
{
    ParsedMap pm;
    pm.valid  = true;
    pm.bounds = {
        0   * si::centi<si::metre>, 500 * si::centi<si::metre>,
        0   * si::centi<si::metre>, 500 * si::centi<si::metre>,
        0   * si::centi<si::metre>, 300 * si::centi<si::metre>
    };
    for (int y = 80; y <= 120; ++y)
        for (int h = 100; h <= 200; ++h)
            pm.cells.push_back({
                300.0                         * si::centi<si::metre>,
                static_cast<double>(y)        * si::centi<si::metre>,
                static_cast<double>(h)        * si::centi<si::metre>,
                MapValue::Occupied
            });
    return pm;
}

// Empty room — no obstacles
static ParsedMap MakeEmptyRoomMap()
{
    ParsedMap pm;
    pm.valid  = true;
    pm.bounds = {
        0   * si::centi<si::metre>, 600 * si::centi<si::metre>,
        0   * si::centi<si::metre>, 600 * si::centi<si::metre>,
        0   * si::centi<si::metre>, 300 * si::centi<si::metre>
    };
    return pm;
}

// Mission config helper (outputResXYDecimals=0 → m_resolution=1.0 → 100 cm BFS steps)
static MissionConfig MakeMissionConfig(
    std::vector<std::pair<double,double>> poly,
    double minH, double maxH)
{
    MissionConfig mc;
    mc.boundaryPolygon     = std::move(poly);
    mc.minHeight           = minH * si::centi<si::metre>;
    mc.maxHeight           = maxH * si::centi<si::metre>;
    mc.outputResXYCm = 1.0;
    mc.outputResHCm  = 1.0;
    return mc;
}

// ---------------------------------------------------------------------------
// Test 1: ScanAndUpdate records Occupied cells at the wall
// ---------------------------------------------------------------------------
static void TestScanRecordsWall()
{
    std::cout << "\n-- MappingAlgorithm: ScanAndUpdate records Occupied wall cells --\n";

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

    MissionConfig mc = MakeMissionConfig(
        {{0,0},{500,0},{500,500},{0,500}}, 150.0, 150.0);
    BuildingMapImpl buildingMap(mc);
    Drone           drone(lidar, posSensor, driver, buildingMap);

    MappingAlgorithm algo(drone, &config, &mc);
    algo.Run();

    // Wall is at x=300.  Coarse lidar detection fires ~5 cm before the surface,
    // so the hit lands in [295, 305].  Check the whole ±5 cm band.
    bool wallDetected = false;
    for (int dx = -5; dx <= 5 && !wallDetected; ++dx) {
        wallDetected = (drone.QueryCell(
            (300.0 + dx) * si::centi<si::metre>,
            100.0        * si::centi<si::metre>,
            150.0        * si::centi<si::metre>) == MapValue::Occupied);
    }
    Check("Wall cell near (300,100,150) recorded as Occupied", wallDetected);
}

// ---------------------------------------------------------------------------
// Test 2: ScanAndUpdate records Empty cells along the ray to the wall
// ---------------------------------------------------------------------------
static void TestScanRecordsEmptyAlongRay()
{
    std::cout << "\n-- MappingAlgorithm: ScanAndUpdate records Empty cells along ray --\n";

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

    MissionConfig mc = MakeMissionConfig(
        {{0,0},{500,0},{500,500},{0,500}}, 150.0, 150.0);
    BuildingMapImpl buildingMap(mc);
    Drone           drone(lidar, posSensor, driver, buildingMap);

    MappingAlgorithm algo(drone, &config, &mc);
    algo.Run();

    // Midpoint at x=200, y=100, h=150 lies on the ray from drone to wall
    const MapValue midCell = drone.QueryCell(
        200.0 * si::centi<si::metre>,
        100.0 * si::centi<si::metre>,
        150.0 * si::centi<si::metre>);

    Check("Midpoint cell (200,100,150) along ray recorded as Empty",
          midCell == MapValue::Empty);
}

// ---------------------------------------------------------------------------
// Test 3: BFS moves the drone from its start position
// ---------------------------------------------------------------------------
static void TestBFSMovesFromStart()
{
    std::cout << "\n-- MappingAlgorithm: BFS exploration moves drone from start --\n";

    const DroneConfig    config = MakeTestDroneConfig();
    const GroundTruthMap gtMap(MakeEmptyRoomMap());

    auto state = std::make_shared<SimulationState>();
    state->position    = { 300.0 * si::centi<si::metre>,
                           300.0 * si::centi<si::metre>,
                           150.0 * si::centi<si::metre> };
    state->orientation = { 0.0 * si::degree, 0.0 * si::degree };

    MockPositionSensor posSensor(state);
    MockMovementDriver driver(state, config, gtMap);
    MockLidarSensor    lidar(state, config, gtMap);

    MissionConfig mc = MakeMissionConfig(
        {{0,0},{600,0},{600,600},{0,600}}, 150.0, 150.0);
    BuildingMapImpl buildingMap(mc);
    Drone           drone(lidar, posSensor, driver, buildingMap);

    MappingAlgorithm algo(drone, &config, &mc);
    algo.Run();

    const Position3D finalPos = drone.GetLocation();
    const double finalX = finalPos.x.numerical_value_in(si::centi<si::metre>);
    const double finalY = finalPos.y.numerical_value_in(si::centi<si::metre>);
    const double dist   = std::sqrt((finalX - 300.0) * (finalX - 300.0) +
                                    (finalY - 300.0) * (finalY - 300.0));

    Check("Drone moved from start (300,300) during BFS exploration", dist > 0.5);
}

// ---------------------------------------------------------------------------
// Test 4: Multiple cells mapped after full BFS exploration (open room)
// ---------------------------------------------------------------------------
static void TestMultipleCellsMapped()
{
    std::cout << "\n-- MappingAlgorithm: multiple cells recorded after exploration --\n";

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

    MissionConfig mc = MakeMissionConfig(
        {{0,0},{500,0},{500,500},{0,500}}, 150.0, 150.0);
    BuildingMapImpl buildingMap(mc);
    Drone           drone(lidar, posSensor, driver, buildingMap);

    MappingAlgorithm algo(drone, &config, &mc);
    algo.Run();

    // At least the initial scan should have recorded both Occupied and Empty cells
    int occupiedCount = 0;
    int emptyCount    = 0;

    // BFS step is 100cm so only y=100 is ever visited; check x=[295,305] at y=100, h=150.
    for (int dx = -5; dx <= 5; ++dx) {
        if (drone.QueryCell((300.0 + dx) * si::centi<si::metre>,
                            100.0 * si::centi<si::metre>,
                            150.0 * si::centi<si::metre>) == MapValue::Occupied) {
            ++occupiedCount;
            break;
        }
    }

    // Empty cells between start and wall along y=100 at h=150, step 10 cm
    for (int x = 110; x < 300; x += 10) {
        if (drone.QueryCell(static_cast<double>(x) * si::centi<si::metre>,
                            100.0 * si::centi<si::metre>,
                            150.0 * si::centi<si::metre>) == MapValue::Empty)
            ++emptyCount;
    }

    Check("At least 1 Occupied wall cell detected",  occupiedCount >= 1);
    Check("At least 5 Empty cells along scan ray",    emptyCount    >= 5);
}

// ---------------------------------------------------------------------------
// Test 5: Run() terminates (no infinite loop) in a bounded scenario
// ---------------------------------------------------------------------------
static void TestRunTerminates()
{
    std::cout << "\n-- MappingAlgorithm: Run() terminates cleanly --\n";

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

    MissionConfig mc = MakeMissionConfig(
        {{0,0},{500,0},{500,500},{0,500}}, 150.0, 150.0);
    BuildingMapImpl buildingMap(mc);
    Drone           drone(lidar, posSensor, driver, buildingMap);

    MappingAlgorithm algo(drone, &config, &mc);

    bool completed = false;
    algo.Run();
    completed = true;

    Check("Run() returns without crashing", completed);
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main()
{
    std::cout << "=== Phase 5 MappingAlgorithm smoke tests ===\n";

    TestScanRecordsWall();
    TestScanRecordsEmptyAlongRay();
    TestBFSMovesFromStart();
    TestMultipleCellsMapped();
    TestRunTerminates();

    std::cout << "\n============================================\n";
    std::cout << "  Passed: " << g_pass << "\n";
    std::cout << "  Failed: " << g_fail << "\n";
    std::cout << "============================================\n";

    return (g_fail == 0) ? 0 : 1;
}
