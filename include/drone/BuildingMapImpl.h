#pragma once

#include <unordered_map>
#include <vector>
#include <utility>
#include "interfaces/IBuildingMap.h"
#include "io/ConfigParser.h"
#include "io/MapIO.h"

namespace drone {

// ---------------------------------------------------------------------------
// BuildingMapImpl
//
// The drone's own sparse 3D occupancy map — implements IBuildingMap.
// The drone is the only writer; it never sees the ground-truth input map.
//
// Storage:
//   std::unordered_map keyed on quantised (ix, iy, ih) integer indices.
//   Missing key  → NotMapped  (-1)
//   Out-of-bounds → BeyondBounds (-2)
//
// Quantisation:
//   ix = round(x_cm  * 10^outputResXYDecimals)
//   iy = round(y_cm  * 10^outputResXYDecimals)
//   ih = round(h_cm  * 10^outputResHDecimals)
//
// Boundary check:
//   XY plane : point-in-polygon test against MissionConfig::boundaryPolygon
//   Height   : [minHeightCm, maxHeightCm]
// ---------------------------------------------------------------------------
class BuildingMapImpl : public IBuildingMap {
public:
    explicit BuildingMapImpl(const MissionConfig& mission);

    // Returns BeyondBounds if outside polygon/height range.
    // Returns NotMapped if inside but not yet recorded.
    // Returns stored value otherwise.
    MapValue Get(Centi x, Centi y, Centi height) const override;

    // Silently ignores positions outside the mission boundary.
    void Set(Centi x, Centi y, Centi height, MapValue value) override;

    // Return every recorded cell so the output file can be written.
    std::vector<MapCell> GetAllCells() const;

private:
    // Integer key for the sparse hash map
    struct Key {
        int ix, iy, ih;
        bool operator==(const Key& o) const noexcept {
            return ix == o.ix && iy == o.iy && ih == o.ih;
        }
    };
    struct KeyHash {
        std::size_t operator()(const Key& k) const noexcept {
            std::size_t seed = static_cast<std::size_t>(k.ix);
            seed ^= static_cast<std::size_t>(k.iy) + 0x9e3779b9u + (seed << 6) + (seed >> 2);
            seed ^= static_cast<std::size_t>(k.ih) + 0x9e3779b9u + (seed << 6) + (seed >> 2);
            return seed;
        }
    };

    Key MakeKey(double xCm, double yCm, double hCm) const;
    bool IsInBounds(Centi x, Centi y, Centi height) const;

    // XY boundary test: uses inclusive rect check for axis-aligned rectangles,
    // falls back to ray-casting for arbitrary polygons.
    bool IsInsidePolygon(double xCm, double yCm) const;

    std::unordered_map<Key, MapValue, KeyHash> m_cells;

    const std::vector<std::pair<double,double>>& m_polygon;
    Centi  m_minHeight;
    Centi  m_maxHeight;
    double m_xyScale;  // 10^outputResXYDecimals
    double m_hScale;   // 10^outputResHDecimals

    // Set to true when the polygon is an axis-aligned rectangle so we can use
    // an inclusive range check (ray-casting misses points on the far edges).
    bool   m_isRect {false};
    double m_rectXmin {0}, m_rectXmax {0};
    double m_rectYmin {0}, m_rectYmax {0};
};

} // namespace drone
