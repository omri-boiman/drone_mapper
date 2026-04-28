#include "drone/BuildingMapImpl.h"
#include "io/MapIO.h"

#include <algorithm>
#include <cmath>

namespace drone {

// ---------------------------------------------------------------------------

BuildingMapImpl::BuildingMapImpl(const MissionConfig& mission)
    : m_polygon(mission.boundaryPolygon)
    , m_minHeight(mission.minHeight)
    , m_maxHeight(mission.maxHeight)
{
    m_xyScale = 1.0 / mission.outputResXYCm;
    m_hScale  = 1.0 / mission.outputResHCm;

    // Detect axis-aligned rectangle: 4 vertices where every x is xmin or xmax
    // and every y is ymin or ymax.  Ray-casting misses points on the far edges
    // (x=xmax, y=ymax), so we switch to an inclusive range check in that case.
    if (m_polygon.size() == 4) {
        double xs[4], ys[4];
        for (int i = 0; i < 4; ++i) { xs[i] = m_polygon[i].first; ys[i] = m_polygon[i].second; }
        m_rectXmin = *std::min_element(xs, xs+4);
        m_rectXmax = *std::max_element(xs, xs+4);
        m_rectYmin = *std::min_element(ys, ys+4);
        m_rectYmax = *std::max_element(ys, ys+4);

        bool isRect = true;
        for (double x : xs) if (std::abs(x - m_rectXmin) > 0.01 && std::abs(x - m_rectXmax) > 0.01) { isRect = false; break; }
        if (isRect) for (double y : ys) if (std::abs(y - m_rectYmin) > 0.01 && std::abs(y - m_rectYmax) > 0.01) { isRect = false; break; }
        m_isRect = isRect;
    }
}

// ---------------------------------------------------------------------------

BuildingMapImpl::Key BuildingMapImpl::MakeKey(double xCm, double yCm, double hCm) const
{
    return {
        static_cast<int>(std::round(xCm * m_xyScale)),
        static_cast<int>(std::round(yCm * m_xyScale)),
        static_cast<int>(std::round(hCm * m_hScale))
    };
}

// ---------------------------------------------------------------------------

bool BuildingMapImpl::IsInsidePolygon(double xCm, double yCm) const
{
    // Axis-aligned rectangle: use inclusive range check so all four walls
    // (including x=xmax and y=ymax) are treated as inside.
    if (m_isRect) {
        return xCm >= m_rectXmin && xCm <= m_rectXmax &&
               yCm >= m_rectYmin && yCm <= m_rectYmax;
    }

    // General polygon: ray-casting algorithm.
    const std::size_t n = m_polygon.size();
    if (n < 3) return false;

    bool inside = false;
    for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
        const double xi = m_polygon[i].first,  yi = m_polygon[i].second;
        const double xj = m_polygon[j].first,  yj = m_polygon[j].second;

        const bool crosses = ((yi > yCm) != (yj > yCm)) &&
                             (xCm < (xj - xi) * (yCm - yi) / (yj - yi) + xi);
        if (crosses) inside = !inside;
    }
    return inside;
}

// ---------------------------------------------------------------------------

bool BuildingMapImpl::IsInBounds(Centi x, Centi y, Centi height) const
{
    if (height < m_minHeight || height > m_maxHeight) return false;
    return IsInsidePolygon(x.numerical_value_in(si::centi<si::metre>),
                           y.numerical_value_in(si::centi<si::metre>));
}

// ---------------------------------------------------------------------------

MapValue BuildingMapImpl::Get(Centi x, Centi y, Centi height) const
{
    if (!IsInBounds(x, y, height)) return MapValue::BeyondBounds;

    const double xCm = x.numerical_value_in(si::centi<si::metre>);
    const double yCm = y.numerical_value_in(si::centi<si::metre>);
    const double hCm = height.numerical_value_in(si::centi<si::metre>);

    const auto it = m_cells.find(MakeKey(xCm, yCm, hCm));
    return (it != m_cells.end()) ? it->second : MapValue::NotMapped;
}

// ---------------------------------------------------------------------------

void BuildingMapImpl::Set(Centi x, Centi y, Centi height, MapValue value)
{
    if (!IsInBounds(x, y, height)) return; // silently ignore

    m_cells[MakeKey(x.numerical_value_in(si::centi<si::metre>),
                    y.numerical_value_in(si::centi<si::metre>),
                    height.numerical_value_in(si::centi<si::metre>))] = value;
}

// ---------------------------------------------------------------------------

std::vector<MapCell> BuildingMapImpl::GetAllCells() const
{
    std::vector<MapCell> result;
    result.reserve(m_cells.size());

    for (const auto& [key, value] : m_cells) {
        MapCell cell;
        cell.x      = (static_cast<double>(key.ix) / m_xyScale) * si::centi<si::metre>;
        cell.y      = (static_cast<double>(key.iy) / m_xyScale) * si::centi<si::metre>;
        cell.height = (static_cast<double>(key.ih) / m_hScale)  * si::centi<si::metre>;
        cell.value  = value;
        result.push_back(cell);
    }

    return result;
}

} // namespace drone
