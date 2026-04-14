#include <filesystem>
#include <iostream>

#include "types/Units.h"
#include "io/ErrorLogger.h"
#include "io/ConfigParser.h"
#include "io/MapIO.h"
#include "simulation/GroundTruthMap.h"

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
    // 5. Simulation loop placeholder — to be filled in Phase 4
    // -----------------------------------------------------------------------
    std::cout << "Input files loaded successfully.\n";
    std::cout << "  Lidar FOV:      "
              << droneConfig.lidarFov.numerical_value_in(si::degree) << " deg\n";
    std::cout << "  Lidar range:    "
              << droneConfig.lidarMinRange.numerical_value_in(si::centi<si::metre>)
              << " - "
              << droneConfig.lidarMaxRange.numerical_value_in(si::centi<si::metre>) << " cm\n";
    std::cout << "  Mission height: "
              << missionConfig.minHeight.numerical_value_in(si::centi<si::metre>)
              << " - "
              << missionConfig.maxHeight.numerical_value_in(si::centi<si::metre>) << " cm\n";
    std::cout << "  Map cells:      " << parsedMap.cells.size() << "\n";
    std::cout << "  Start pos:      ("
              << missionConfig.startX.numerical_value_in(si::centi<si::metre>)      << ", "
              << missionConfig.startY.numerical_value_in(si::centi<si::metre>)      << ", "
              << missionConfig.startHeight.numerical_value_in(si::centi<si::metre>) << ") cm\n";

    // TODO Phase 4: initialise mocks, drone, and run simulation loop

    return 0;
}
