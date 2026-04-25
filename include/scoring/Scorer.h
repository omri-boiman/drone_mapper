#pragma once

#include <vector>
#include "types/MapValue.h"
#include "types/Units.h"
#include "io/MapIO.h"

namespace drone {

class IBuildingMap;

// ---------------------------------------------------------------------------
// Scorer
//
// Compares the drone's discovered map against the ground truth map
// and computes a score.
// 
// Score formula:
//   score = (correctly_occupied + correctly_empty) /
//           (ground_truth_occupied + reachable_empty_cells) * 100
//
// For Phase 4: stub that returns 0. Phase 6+ will compute actual scores.
// ---------------------------------------------------------------------------
class Scorer {
public:
    // Compute the score comparing the drone's map to ground truth.
    //
    // droneMap   — used solely for the mission-boundary polygon check
    //              (Get() returns BeyondBounds for out-of-bounds positions)
    // droneCells — the complete set of cells the drone recorded
    // groundTruth / bounds — the reference map
    // xyScale = 10^outputResXYDecimals, hScale = 10^outputResHDecimals
    // Both maps are rasterised to this grid before comparison, as per the v2 spec.
    static double ComputeScore(const IBuildingMap&         droneMap,
                               const std::vector<MapCell>& droneCells,
                               const ParsedMap&            groundTruth,
                               const MapBounds&            bounds,
                               double                      xyScale,
                               double                      hScale);
};

} // namespace drone
