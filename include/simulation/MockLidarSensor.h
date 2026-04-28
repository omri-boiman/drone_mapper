#pragma once

#include <memory>
#include "interfaces/ILidarSensor.h"
#include "simulation/SimulationState.h"
#include "simulation/GroundTruthMap.h"
#include "io/ConfigParser.h"

namespace drone {

// ---------------------------------------------------------------------------
// MockLidarSensor — professor's circular beam model
//
// Beams are arranged in concentric circles around the scan centre:
//   Circle 0 : 1 beam (centre)
//   Circle N : 4^N beams; radius at Z-min = N*D; angles computed via atan2
//              (professor's 2D angle-space formula)
//
// Returns RELATIVE angles (relative to drone heading).
// The drone adds its heading to reconstruct absolute world-frame direction.
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
