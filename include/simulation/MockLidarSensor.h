#pragma once

#include <memory>
#include "interfaces/ILidarSensor.h"
#include "simulation/SimulationState.h"
#include "simulation/GroundTruthMap.h"
#include "io/ConfigParser.h"

namespace drone {

// ---------------------------------------------------------------------------
// MockLidarSensor — v2 circular beam model
//
// The lidar emits beams arranged in concentric circles around the scan centre:
//   Circle 0 : 1 beam (centre)
//   Circle N : 4^N beams, angular radius = N * atan(D / Z-min) from centre
//              beams are evenly distributed around the circumference
//
// Only beams that hit something within [Z-min, Z-max] appear in the result.
// A hit within Z-min is included with distance=0 (too close to measure).
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

    // Cast a single ray; returns distance in cm, -1 (no hit), or -2 (< Z-min)
    double CastRay(double originX, double originY, double originH,
                   double dx, double dy, double dz) const;
};

} // namespace drone
