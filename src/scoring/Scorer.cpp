#include "scoring/Scorer.h"
#include "interfaces/IBuildingMap.h"
#include "types/MapValue.h"

#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <iostream>

using namespace mp_units;
using namespace mp_units::si::unit_symbols;

namespace drone {

namespace {

struct IKey3 {
    int x, y, h;
    bool operator==(const IKey3& o) const noexcept {
        return x == o.x && y == o.y && h == o.h;
    }
};
struct IKey3Hash {
    std::size_t operator()(const IKey3& k) const noexcept {
        std::size_t s = static_cast<std::size_t>(k.x);
        s ^= static_cast<std::size_t>(k.y) + 0x9e3779b9u + (s << 6) + (s >> 2);
        s ^= static_cast<std::size_t>(k.h) + 0x9e3779b9u + (s << 6) + (s >> 2);
        return s;
    }
};

IKey3 MakeKey(double xCm, double yCm, double hCm, double xyScale, double hScale)
{
    return { static_cast<int>(std::round(xCm * xyScale)),
             static_cast<int>(std::round(yCm * xyScale)),
             static_cast<int>(std::round(hCm * hScale)) };
}

} // anonymous namespace

// ---------------------------------------------------------------------------
// Scorer::ComputeScore
//
// Both maps are rasterised to the output-resolution grid (per v2 spec) before
// comparison.  xyScale = 10^outputResXYDecimals, hScale = 10^outputResHDecimals.
//
// Steps
// -----
// 1. Build gtOccupied: GT Occupied cells reachable by the drone, keyed at
//    output resolution.  Only these cells contribute to gt_occupied.
// 2. Deduplicate the drone's recorded cells at output resolution: when both
//    Occupied and Empty fall in the same voxel, Occupied wins.
// 3. Walk the deduplicated drone voxels:
//    - Occupied + in gtOccupied → correctly_occupied
//    - Empty    + not in gtOccupied → correctly_empty; total → reachable_empty
// 4. score = (correctly_occupied + correctly_empty) /
//            (gt_occupied + reachable_empty) * 100
// ---------------------------------------------------------------------------
double Scorer::ComputeScore(const IBuildingMap&         droneMap,
                            const std::vector<MapCell>& droneCells,
                            const ParsedMap&            groundTruth,
                            const MapBounds&            bounds [[maybe_unused]],
                            double                      xyScale,
                            double                      hScale)
{
    std::cerr << "[Scorer Debug]  output resolution: xy=" << xyScale
              << "  h=" << hScale << "\n";

    // 1. Build GT occupied set (only cells the drone map considers reachable).
    std::unordered_set<IKey3, IKey3Hash> gtOccupied;
    long long gt_occupied = 0;

    for (const auto& cell : groundTruth.cells) {
        if (cell.value != MapValue::Occupied) continue;
        if (droneMap.Get(cell.x, cell.y, cell.height) == MapValue::BeyondBounds)
            continue;
        ++gt_occupied;
        gtOccupied.insert(MakeKey(
            cell.x.numerical_value_in(si::centi<si::metre>),
            cell.y.numerical_value_in(si::centi<si::metre>),
            cell.height.numerical_value_in(si::centi<si::metre>),
            xyScale, hScale));
    }

    // 2. Deduplicate drone cells at output resolution. Occupied beats Empty.
    std::unordered_map<IKey3, MapValue, IKey3Hash> droneCoarse;
    droneCoarse.reserve(droneCells.size());

    for (const auto& cell : droneCells) {
        IKey3 k = MakeKey(
            cell.x.numerical_value_in(si::centi<si::metre>),
            cell.y.numerical_value_in(si::centi<si::metre>),
            cell.height.numerical_value_in(si::centi<si::metre>),
            xyScale, hScale);

        auto it = droneCoarse.find(k);
        if (it == droneCoarse.end()) {
            droneCoarse[k] = cell.value;
        } else if (cell.value == MapValue::Occupied) {
            it->second = MapValue::Occupied;
        }
    }

    // 3. Score the deduplicated voxels.
    long long correctly_occupied = 0;
    long long correctly_empty    = 0;
    long long reachable_empty    = 0;

    for (const auto& [key, value] : droneCoarse) {
        const bool inGT = gtOccupied.count(key) > 0;

        if (value == MapValue::Occupied) {
            if (inGT) ++correctly_occupied;
        } else if (value == MapValue::Empty) {
            ++reachable_empty;
            if (!inGT) ++correctly_empty;
        }
    }

    const long long denom = gt_occupied + reachable_empty;
    if (denom == 0) return 0.0;

    const double result =
        static_cast<double>(correctly_occupied + correctly_empty) * 100.0
        / static_cast<double>(denom);

    std::cerr << "[Scorer Debug]\n"
              << "  GT occupied (reachable): " << gt_occupied << "\n"
              << "  Correctly occupied:      " << correctly_occupied << "\n"
              << "  Reachable empty:         " << reachable_empty << "\n"
              << "  Correctly empty:         " << correctly_empty << "\n"
              << "  Denom:                   " << denom << "\n"
              << "  Numer:                   " << (correctly_occupied + correctly_empty) << "\n"
              << "  Score:                   " << result << "%\n";

    return result;
}

} // namespace drone
