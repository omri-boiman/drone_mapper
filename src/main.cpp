#include <filesystem>
#include <iostream>

#include "types/Units.h"
#include "io/ErrorLogger.h"
#include "io/ConfigParser.h"
#include "io/MapIO.h"

int main(int argc, char* argv[])
{
    const std::filesystem::path ioPath =
        (argc >= 2) ? std::filesystem::path(argv[1])
                    : std::filesystem::current_path();

    drone::ErrorLogger logger;

    drone::DroneConfig droneConfig;
    const bool droneConfigOk =
        drone::ParseDroneConfig(ioPath / "drone_config.txt", droneConfig, logger);
    if (!droneConfigOk) {
        std::cerr << "Fatal: could not open drone_config.txt in " << ioPath << "\n";
        logger.Flush(ioPath);
        return 1;
    }

    drone::MissionConfig missionConfig;
    const bool missionConfigOk =
        drone::ParseMissionConfig(ioPath / "mission_config.txt", missionConfig, logger);
    if (!missionConfigOk) {
        std::cerr << "Fatal: could not open mission_config.txt in " << ioPath << "\n";
        logger.Flush(ioPath);
        return 1;
    }

    const drone::ParsedMap parsedMap =
        drone::ParseMapFile(ioPath / "map_input.txt", logger);
    if (!parsedMap.valid) {
        std::cerr << "Fatal: could not parse map_input.txt in " << ioPath << "\n";
        logger.Flush(ioPath);
        return 1;
    }

    if (logger.HasErrors()) {
        logger.Flush(ioPath);
    }

    // TODO Phase 6: wire SimulationState + CellMap + mocks + Drone + algo

    std::cout << "Input files loaded successfully.\n";
    std::cout << "  Lidar beam range: "
              << droneConfig.lidarBeamMin.numerical_value_in(drone::cm)
              << " - "
              << droneConfig.lidarBeamMax.numerical_value_in(drone::cm) << " cm\n";
    std::cout << "  Lidar circles:    " << droneConfig.lidarFovCircles << "\n";
    std::cout << "  Mission height:   "
              << missionConfig.minHeight.numerical_value_in(drone::cm)
              << " - "
              << missionConfig.maxHeight.numerical_value_in(drone::cm) << " cm\n";
    std::cout << "  Map cells:        " << parsedMap.cells.size() << "\n";
    std::cout << "  Start pos:        ("
              << missionConfig.startX.numerical_value_in(drone::cm)      << ", "
              << missionConfig.startY.numerical_value_in(drone::cm)      << ", "
              << missionConfig.startHeight.numerical_value_in(drone::cm) << ") cm\n";

    return 0;
}
