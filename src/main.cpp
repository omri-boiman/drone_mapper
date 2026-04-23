#include <filesystem>
#include <iostream>
#include <memory>

#include "types/Units.h"
#include "io/ErrorLogger.h"
#include "io/ConfigParser.h"
#include "io/MapIO.h"
#include "simulation/GroundTruthMap.h"
#include "simulation/SimulationState.h"
#include "simulation/MockLidarSensor.h"
#include "simulation/MockPositionSensor.h"
#include "simulation/MockMovementDriver.h"
#include "drone/BuildingMapImpl.h"
#include "drone/Drone.h"
#include "drone/MappingAlgorithm.h"
#include "scoring/Scorer.h"

using namespace mp_units;
using namespace mp_units::si::unit_symbols;

int main(int argc, char* argv[])
{
    // -----------------------------------------------------------------------
    // 1. Resolve input/output path
    // -----------------------------------------------------------------------
    const std::filesystem::path ioPath =
        (argc >= 2) ? std::filesystem::path(argv[1])
                    : std::filesystem::current_path();

    // -----------------------------------------------------------------------
    // 2. Load all three input files
    // -----------------------------------------------------------------------
    drone::ErrorLogger logger;

    // --- drone_config.txt ---
    drone::DroneConfig droneConfig;
    const bool droneConfigOk =
        drone::ParseDroneConfig(ioPath / "drone_config.txt", droneConfig, logger);

    if (!droneConfigOk) {
        std::cerr << "Fatal: could not open drone_config.txt in " << ioPath << "\n";
        logger.Flush(ioPath);
        return 1;
    }

    // --- mission_config.txt ---
    drone::MissionConfig missionConfig;
    const bool missionConfigOk =
        drone::ParseMissionConfig(ioPath / "mission_config.txt", missionConfig, logger);

    if (!missionConfigOk) {
        std::cerr << "Fatal: could not open mission_config.txt in " << ioPath << "\n";
        logger.Flush(ioPath);
        return 1;
    }

    // --- map_input.txt ---
    const drone::ParsedMap parsedMap =
        drone::ParseMapFile(ioPath / "map_input.txt", logger);

    if (!parsedMap.valid) {
        std::cerr << "Fatal: could not parse map_input.txt in " << ioPath << "\n";
        logger.Flush(ioPath);
        return 1;
    }

    // -----------------------------------------------------------------------
    // 3. Write input_errors.txt if any recoverable errors were found
    // -----------------------------------------------------------------------
    if (logger.HasErrors()) {
        logger.Flush(ioPath);
    }

    // -----------------------------------------------------------------------
    // 4. Build ground-truth map (visible only to mock sensors)
    // -----------------------------------------------------------------------
    drone::GroundTruthMap groundTruth(parsedMap);

    // -----------------------------------------------------------------------
    // 5. Wire up the simulation: create SimulationState + all mocks
    // -----------------------------------------------------------------------
    auto simulationState = std::make_shared<drone::SimulationState>();

    // Initialize drone position from mission config
    simulationState->position = {
        missionConfig.startX,
        missionConfig.startY,
        missionConfig.startHeight
    };
    simulationState->orientation = {
        0.0 * si::degree,  // initially facing +X (0 degrees)
        0.0 * si::degree   // horizontal pitch (no elevation)
    };

    // Create the three mock sensors
    drone::MockLidarSensor      lidar(simulationState, droneConfig, groundTruth);
    drone::MockPositionSensor   position(simulationState);
    drone::MockMovementDriver   driver(simulationState, droneConfig, groundTruth);

    // Create the drone's own building map
    drone::BuildingMapImpl       map(missionConfig);

    // Create the Drone (hardware abstraction layer)
    drone::Drone                drone(lidar, position, driver, map);

    // -----------------------------------------------------------------------
    // 6. Run the mapping algorithm
    // -----------------------------------------------------------------------
    std::cout << "\nStarting mapping algorithm...\n";
    drone::MappingAlgorithm algo(drone, &droneConfig, &missionConfig);

    try {
        algo.Run();
        std::cout << "Mapping algorithm completed successfully.\n";
    } catch (const std::exception& e) {
        std::cerr << "Mapping algorithm failed with exception: " << e.what() << "\n";
        return 1;
    }

    // -----------------------------------------------------------------------
    // 7. Write the output map
    // -----------------------------------------------------------------------
    const std::vector<drone::MapCell> discoveredCells = map.GetAllCells();
    std::cout << "Cells recorded: " << discoveredCells.size() << "\n";
    const drone::MapBounds outputBounds = parsedMap.bounds;

    const std::filesystem::path outputPath = ioPath / "map_output.txt";
    if (!drone::WriteMapFile(outputPath, outputBounds, discoveredCells)) {
        std::cerr << "Failed to write output map to " << outputPath << "\n";
        return 1;
    }
    std::cout << "Output map written to " << outputPath << "\n";

    // -----------------------------------------------------------------------
    // 8. Compute and print score
    // -----------------------------------------------------------------------
    const double score = drone::Scorer::ComputeScore(map, discoveredCells, parsedMap, outputBounds);
    std::cout << "\nScore: " << score << "%\n";

    // -----------------------------------------------------------------------
    // 9. Done
    // -----------------------------------------------------------------------
    std::cout << "Simulation completed.\n";

    return 0;
}
