#include "drone/BuildingMapImpl.h"

#include <cmath>

namespace drone {

// ---------------------------------------------------------------------------

BuildingMapImpl::BuildingMapImpl(const MissionConfig& mission)
    : m_polygon(mission.boundaryPolygon)
    , m_minHeight(mission.minHeight)
    , m_maxHeight(mission.maxHeight)
{
    // Pre-compute scale factors from decimal-place settings
    m_xyScale = std::pow(10.0, static_cast<double>(mission.outputResXYDecimals));
    m_hScale  = std::pow(10.0, static_cast<double>(mission.outputResHDecimals));
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
    // Ray-casting algorithm: count crossings of a horizontal ray from (x,y)
    // going in the +X direction with each polygon edge.
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

} // namespace drone
