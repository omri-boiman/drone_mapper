#include "simulation/GroundTruthMap.h"

namespace drone {

GroundTruthMap::GroundTruthMap(const ParsedMap& parsed)
    : m_bounds(parsed.bounds)
{
    for (const auto& cell : parsed.cells) {
        m_cells[MakeKey(
            cell.x.numerical_value_in(si::centi<si::metre>),
            cell.y.numerical_value_in(si::centi<si::metre>),
            cell.height.numerical_value_in(si::centi<si::metre>)
        )] = cell.value;
    }
}

bool GroundTruthMap::IsOccupied(Centi x, Centi y, Centi height) const
{
    const auto it = m_cells.find(MakeKey(
        x.numerical_value_in(si::centi<si::metre>),
        y.numerical_value_in(si::centi<si::metre>),
        height.numerical_value_in(si::centi<si::metre>)
    ));
    if (it == m_cells.end()) return false;
    return it->second == MapValue::Occupied;
}

MapValue GroundTruthMap::Query(Centi x, Centi y, Centi height) const
{
    const auto it = m_cells.find(MakeKey(
        x.numerical_value_in(si::centi<si::metre>),
        y.numerical_value_in(si::centi<si::metre>),
        height.numerical_value_in(si::centi<si::metre>)
    ));
    if (it == m_cells.end()) return MapValue::NotMapped;
    return it->second;
}

} // namespace drone
