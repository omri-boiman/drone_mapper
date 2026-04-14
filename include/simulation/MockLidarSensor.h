#pragma once

#include <memory>
#include "interfaces/ILidarSensor.h"
#include "simulation/SimulationState.h"
#include "simulation/GroundTruthMap.h"
#include "io/ConfigParser.h"

namespace drone {

// ---------------------------------------------------------------------------
// MockLidarSensor
//
// Implements ILidarSensor by ray-casting against the GroundTruthMap.
//
// Matrix dimensions are derived from the FOV and the resolution at dist1:
//   deltaAngle = atan(resAtDist1 / dist1)
//   numCells   = 2*floor(fov_rad / (2*deltaAngle)) + 1   (odd, centred)
//
// Each ray is stepped in 1 cm increments (matches GroundTruthMap quantisation).
// Return values per cell:
//   positive  : distance in cm to first occupied voxel
//   -1.0      : no hit within maxRange
//   -2.0      : hit detected below minRange
// ---------------------------------------------------------------------------
class MockLidarSensor : public ILidarSensor {
public:
    MockLidarSensor(std::shared_ptr<SimulationState> state,
                    const DroneConfig&               config,
                    const GroundTruthMap&            groundTruth);

    LidarScanResult Scan(
        std::optional<Degrees> xy_angle = std::nullopt,
        std::optional<Degrees> pitch    = std::nullopt) override;

private:
    std::shared_ptr<SimulationState> m_state;
    const DroneConfig&               m_config;
    const GroundTruthMap&            m_groundTruth;

    // Cast a single ray; returns distance in cm, -1 or -2
    double CastRay(double originX, double originY, double originH,
                   double dx, double dy, double dz) const;
};

} // namespace drone
