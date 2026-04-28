#pragma once

#include <unordered_map>
#include <tuple>
#include <cmath>
#include "io/MapIO.h"
#include "types/MapValue.h"
#include "types/Units.h"

namespace drone {

// ---------------------------------------------------------------------------
// GroundTruthMap
//
// Stores the "real" building loaded from map_input.txt.
// Used ONLY by the mock sensors — the Drone never accesses this directly.
//
// Positions are quantised to whole centimetres for O(1) lookup.
// The lidar mock steps through space in 1 cm increments and calls IsOccupied()
// to detect walls, floors, ceilings and obstacles.
// ---------------------------------------------------------------------------

class GroundTruthMap {
public:
    // Build from an already-parsed map file
    explicit GroundTruthMap(const ParsedMap& parsed);

    // Returns true if the position is occupied in the ground-truth map.
    // Snaps the query to the map's resolution before lookup, so a ray
    // stepping at 1 cm always detects a wall cell regardless of beam angle.
    bool IsOccupied(Centi x, Centi y, Centi height) const;

    // Returns the raw MapValue at the position (BeyondBounds / NotMapped / etc.)
    MapValue Query(Centi x, Centi y, Centi height) const;

    const MapBounds& Bounds() const { return m_bounds; }

private:
    // Key: (x, y, h) rounded to nearest cm, stored as integers
    struct Key {
        int x, y, h;
        bool operator==(const Key& o) const noexcept {
            return x == o.x && y == o.y && h == o.h;
        }
    };

    struct KeyHash {
        std::size_t operator()(const Key& k) const noexcept {
            // Simple but effective hash combining three ints
            std::size_t seed = static_cast<std::size_t>(k.x);
            seed ^= static_cast<std::size_t>(k.y)  + 0x9e3779b9u + (seed << 6) + (seed >> 2);
            seed ^= static_cast<std::size_t>(k.h)  + 0x9e3779b9u + (seed << 6) + (seed >> 2);
            return seed;
        }
    };

    static Key MakeKey(double xCm, double yCm, double hCm) {
        return { static_cast<int>(std::floor(xCm)),
                 static_cast<int>(std::floor(yCm)),
                 static_cast<int>(std::floor(hCm)) };
    }

    std::unordered_map<Key, MapValue, KeyHash> m_cells;
    MapBounds m_bounds;
};

} // namespace drone
