#include "drone/MappingAlgorithm.h"
#include "drone/Drone.h"
#include <cmath>
#include <algorithm>
#include <functional>
#include <limits>
#include <map>

namespace drone {

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
MappingAlgorithm::MappingAlgorithm(Drone& drone, const DroneConfig* config,
                                   const MissionConfig* mission)
    : m_drone(drone), m_config(config), m_mission(mission) {
    if (mission) {
        m_minHeight = static_cast<int>(
            mission->minHeight.numerical_value_in(si::centi<si::metre>));
        m_maxHeight = static_cast<int>(
            mission->maxHeight.numerical_value_in(si::centi<si::metre>));

        // Initialize at start height
        Position3D startPos = m_drone.GetLocation();
        m_currentHeightLevel = static_cast<int>(
            startPos.height.numerical_value_in(si::centi<si::metre>));
    }

    // Set resolution based on output config or use default 1 cm
    if (mission && mission->outputResXYDecimals >= 0) {
        m_resolution = std::pow(10.0, -mission->outputResXYDecimals);
    }

    // BFS step: one drone-advance-length in grid units.
    // This decouples the storage quantisation (m_resolution) from the
    // navigation step so that the drone physically moves each BFS iteration
    // regardless of how finely the map is stored.
    if (config) {
        const double advanceCm =
            config->maxAdvance.numerical_value_in(si::centi<si::metre>);
        m_bfsStep = std::max(1,
            static_cast<int>(std::ceil(advanceCm / m_resolution)));
    } else {
        m_bfsStep = static_cast<int>(std::ceil(50.0 / m_resolution));
    }
}

// ---------------------------------------------------------------------------
// Main algorithm entry point
// ---------------------------------------------------------------------------
void MappingAlgorithm::Run() {
    const double elevStep = m_config
        ? m_config->maxElevate.numerical_value_in(si::centi<si::metre>)
        : 30.0;

    // 0. Brute-force insert all 6 boundary faces as Occupied.
    //    The mission config gives us the exact room extents, so we treat every
    //    cell on the boundary box as a wall.  This covers surfaces the lidar
    //    physically cannot reach (floor/ceiling due to ±45° FOV limit) and
    //    surfaces it would sample too sparsely at long range (far side walls).
    //    Assumption: the room is an enclosed box — valid per the mission spec.
    //    Out-of-bounds cells are silently rejected by the building map.
    //
    // if (m_mission && !m_mission->boundaryPolygon.empty()) {
    //     double bboxMinX =  1e9, bboxMaxX = -1e9;
    //     double bboxMinY =  1e9, bboxMaxY = -1e9;
    //     for (const auto& [px, py] : m_mission->boundaryPolygon) {
    //         bboxMinX = std::min(bboxMinX, px);
    //         bboxMaxX = std::max(bboxMaxX, px);
    //         bboxMinY = std::min(bboxMinY, py);
    //         bboxMaxY = std::max(bboxMaxY, py);
    //     }
    //     const double floorH   = static_cast<double>(m_minHeight);
    //     const double ceilingH = static_cast<double>(m_maxHeight);
    //
    //     // Floor and ceiling: sweep all (x, y) at fixed h
    //     for (double x = bboxMinX; x <= bboxMaxX + 0.5; x += 1.0) {
    //         for (double y = bboxMinY; y <= bboxMaxY + 0.5; y += 1.0) {
    //             m_drone.RecordCell(x * si::centi<si::metre>,
    //                                y * si::centi<si::metre>,
    //                                floorH * si::centi<si::metre>,
    //                                MapValue::Occupied);
    //             m_drone.RecordCell(x * si::centi<si::metre>,
    //                                y * si::centi<si::metre>,
    //                                ceilingH * si::centi<si::metre>,
    //                                MapValue::Occupied);
    //         }
    //     }
    //
    //     // Four vertical walls: sweep all (y, h) at fixed x, and (x, h) at fixed y
    //     for (double h = floorH; h <= ceilingH + 0.5; h += 1.0) {
    //         for (double y = bboxMinY; y <= bboxMaxY + 0.5; y += 1.0) {
    //             m_drone.RecordCell(bboxMinX * si::centi<si::metre>,
    //                                y        * si::centi<si::metre>,
    //                                h        * si::centi<si::metre>,
    //                                MapValue::Occupied);
    //             m_drone.RecordCell(bboxMaxX * si::centi<si::metre>,
    //                                y        * si::centi<si::metre>,
    //                                h        * si::centi<si::metre>,
    //                                MapValue::Occupied);
    //         }
    //         for (double x = bboxMinX; x <= bboxMaxX + 0.5; x += 1.0) {
    //             m_drone.RecordCell(x        * si::centi<si::metre>,
    //                                bboxMinY * si::centi<si::metre>,
    //                                h        * si::centi<si::metre>,
    //                                MapValue::Occupied);
    //             m_drone.RecordCell(x        * si::centi<si::metre>,
    //                                bboxMaxY * si::centi<si::metre>,
    //                                h        * si::centi<si::metre>,
    //                                MapValue::Occupied);
    //         }
    //     }
    // }

    // 1. Full 360° scan at the starting position.
    ScanAndUpdate();

    // 2. Explore XY at the starting height slice.
    ExploreAtCurrentHeight();

    // 3. Descend from start height down to minHeight, exploring each slice.
    while (true) {
        Position3D pos = m_drone.GetLocation();
        double curH = pos.height.numerical_value_in(si::centi<si::metre>);

        double nextH = curH - elevStep;
        if (nextH < static_cast<double>(m_minHeight) - 1.0) break;

        m_currentHeightLevel = static_cast<int>(nextH);
        if (!AdjustHeight(nextH * si::centi<si::metre>)) break;

        ScanAndUpdate();
        ExploreAtCurrentHeight();
    }

    // 4. Climb upward from current position to maxHeight, exploring each slice.
    while (true) {
        Position3D pos = m_drone.GetLocation();
        double curH = pos.height.numerical_value_in(si::centi<si::metre>);

        double nextH = curH + elevStep;
        if (nextH > static_cast<double>(m_maxHeight) + 1.0) break;

        m_currentHeightLevel = static_cast<int>(nextH);
        if (!AdjustHeight(nextH * si::centi<si::metre>)) break;

        ScanAndUpdate();
        ExploreAtCurrentHeight();
    }
}

// ---------------------------------------------------------------------------
// Scan and update: fire lidar at current heading + three 90° rotations so
// every BFS position covers a full 360° sweep.
// ---------------------------------------------------------------------------
void MappingAlgorithm::ScanAndUpdate() {
    // Four cardinal scans (0°, 90°, 180°, 270° relative to current heading).
    // We rotate the drone in place and call the single-direction helper each
    // time, ending back at the original heading.
    const double maxRotateDeg = m_config
        ? m_config->maxRotate.numerical_value_in(si::degree)
        : 45.0;

    for (int quarter = 0; quarter < 4; ++quarter) {
        ScanSingleDirection();
        // Rotate 90° (in max-rotate-sized steps)
        double remaining = 90.0;
        while (remaining > 0.5) {
            double step = std::min(remaining, maxRotateDeg);
            m_drone.Rotate(step * si::degree);
            m_currentHeading += step;
            if (m_currentHeading >= 360.0) m_currentHeading -= 360.0;
            remaining -= step;
        }
    }
}

// Single-direction lidar scan using the v2 circular beam model.
// The result is a sparse hit list; azimuth and elevation are absolute angles,
// so there is no need to reconstruct ray directions from matrix offsets.
void MappingAlgorithm::ScanSingleDirection() {
    LidarScanResult hits = m_drone.Scan();
    Position3D dronePos = m_drone.GetLocation();

    const double droneX = dronePos.x.numerical_value_in(si::centi<si::metre>);
    const double droneY = dronePos.y.numerical_value_in(si::centi<si::metre>);
    const double droneZ = dronePos.height.numerical_value_in(si::centi<si::metre>);

    for (const auto& hit : hits) {
        if (hit.distance == 0.0) continue;  // within Z-min, position unknown

        const double azRad = hit.azimuth.numerical_value_in(si::radian);
        const double elRad = hit.elevation.numerical_value_in(si::radian);

        const double dx = std::cos(elRad) * std::cos(azRad);
        const double dy = std::cos(elRad) * std::sin(azRad);
        const double dz = std::sin(elRad);

        // Snap to nearest cm so boundary check matches GroundTruthMap precision
        const double hitX = std::round(droneX + hit.distance * dx);
        const double hitY = std::round(droneY + hit.distance * dy);
        const double hitZ = std::round(droneZ + hit.distance * dz);

        m_drone.RecordCell(hitX * si::centi<si::metre>,
                           hitY * si::centi<si::metre>,
                           hitZ * si::centi<si::metre>,
                           MapValue::Occupied);

        // Record empty cells along the ray at 1 cm steps
        for (double d = 1.0; d < hit.distance; d += 1.0) {
            m_drone.RecordCell(
                (droneX + d * dx) * si::centi<si::metre>,
                (droneY + d * dy) * si::centi<si::metre>,
                (droneZ + d * dz) * si::centi<si::metre>,
                MapValue::Empty);
        }
    }
}

// ---------------------------------------------------------------------------
// BFS frontier exploration at current height
// ---------------------------------------------------------------------------
void MappingAlgorithm::ExploreAtCurrentHeight() {
    Position3D currentPos = m_drone.GetLocation();
    GridCell3D currentCell = WorldToGrid(currentPos.x, currentPos.y, currentPos.height);

    m_visited.insert(currentCell);

    // Push unvisited walkable neighbors of `cell` onto the frontier,
    // skipping any whose passage is too narrow for the drone.
    const int S = m_bfsStep;
    auto expand = [&](const GridCell3D& cell) {
        for (const GridCell3D& nb : std::vector<GridCell3D>{
                {cell.x + S, cell.y,     cell.z},
                {cell.x - S, cell.y,     cell.z},
                {cell.x,     cell.y + S, cell.z},
                {cell.x,     cell.y - S, cell.z}})
            if (!m_visited.count(nb) && IsWalkable(nb) && HasClearance(cell, nb))
                m_frontier.push(nb);
    };

    expand(currentCell);

    while (!m_frontier.empty()) {
        GridCell3D target = m_frontier.front();
        m_frontier.pop();

        if (m_visited.count(target)) continue;

        // Use A* to find a path through known-empty space to the target
        std::vector<GridCell3D> path = FindPath(target);
        if (path.empty()) continue;  // currently unreachable — skip

        // Walk every step in the path (index 0 is current position, already visited)
        for (std::size_t i = 1; i < path.size(); ++i) {
            if (!MoveToCell(path[i])) return;  // collision — stop exploration
            m_visited.insert(path[i]);
            ScanAndUpdate();
            expand(path[i]);
        }
    }
}

// ---------------------------------------------------------------------------
// Height adjustment
// ---------------------------------------------------------------------------
bool MappingAlgorithm::ElevateToNextHeight() {
    m_currentHeightLevel += 30;
    return AdjustHeight(m_currentHeightLevel * si::centi<si::metre>);
}

// ---------------------------------------------------------------------------
// Helper: world to grid conversion
// ---------------------------------------------------------------------------
GridCell3D MappingAlgorithm::WorldToGrid(Centi x, Centi y, Centi z) const {
    int gridX = static_cast<int>(
        x.numerical_value_in(si::centi<si::metre>) / m_resolution);
    int gridY = static_cast<int>(
        y.numerical_value_in(si::centi<si::metre>) / m_resolution);
    int gridZ = static_cast<int>(
        z.numerical_value_in(si::centi<si::metre>) / m_resolution);
    return {gridX, gridY, gridZ};
}

// ---------------------------------------------------------------------------
// Helper: grid to world conversion
// ---------------------------------------------------------------------------
void MappingAlgorithm::GridToWorld(const GridCell3D& cell, Centi& x, Centi& y,
                                   Centi& z) const {
    x = (cell.x * m_resolution) * si::centi<si::metre>;
    y = (cell.y * m_resolution) * si::centi<si::metre>;
    z = (cell.z * m_resolution) * si::centi<si::metre>;
}

// ---------------------------------------------------------------------------
// Helper: check if cell is walkable (empty or unmapped)
// ---------------------------------------------------------------------------
bool MappingAlgorithm::IsWalkable(const GridCell3D& cell) const {
    Centi x, y, z;
    GridToWorld(cell, x, y, z);

    MapValue val = m_drone.QueryCell(x, y, z);
    return val != MapValue::Occupied && val != MapValue::BeyondBounds;
}

// ---------------------------------------------------------------------------
// Helper: check the passage from `from` to `to` fits the drone's minimum
// pass dimensions (width and height).  Only uses cells already recorded in
// the drone's map — unknown cells are assumed passable.
// ---------------------------------------------------------------------------
bool MappingAlgorithm::HasClearance(const GridCell3D& from,
                                    const GridCell3D& to) const {
    if (!m_config) return true;

    const double halfW = m_config->minPassWidth.numerical_value_in(
                             si::centi<si::metre>) / 2.0;
    const double halfH = m_config->minPassHeight.numerical_value_in(
                             si::centi<si::metre>) / 2.0;

    Centi toX, toY, toZ;
    GridToWorld(to, toX, toY, toZ);
    const double tx = toX.numerical_value_in(si::centi<si::metre>);
    const double ty = toY.numerical_value_in(si::centi<si::metre>);
    const double tz = toZ.numerical_value_in(si::centi<si::metre>);

    // Determine lateral axis: movement in X → lateral is Y; movement in Y → lateral is X
    const bool movingInX = (to.x != from.x);

    // Four boundary points of the minimum-pass cross-section at the target cell
    const double latOff1 = movingInX ? ty - halfW : tx - halfW;
    const double latOff2 = movingInX ? ty + halfW : tx + halfW;

    auto occupied = [&](double cx, double cy, double cz) {
        return m_drone.QueryCell(cx * si::centi<si::metre>,
                                 cy * si::centi<si::metre>,
                                 cz * si::centi<si::metre>) == MapValue::Occupied;
    };

    if (movingInX) {
        return !occupied(tx, latOff1, tz)        // left edge
            && !occupied(tx, latOff2, tz)        // right edge
            && !occupied(tx, ty,      tz - halfH) // bottom edge
            && !occupied(tx, ty,      tz + halfH); // top edge
    } else {
        return !occupied(latOff1, ty, tz)        // left edge
            && !occupied(latOff2, ty, tz)        // right edge
            && !occupied(tx,      ty, tz - halfH) // bottom edge
            && !occupied(tx,      ty, tz + halfH); // top edge
    }
}

// ---------------------------------------------------------------------------
// A* pathfinding — finds shortest path through known-empty cells
// ---------------------------------------------------------------------------
std::vector<GridCell3D> MappingAlgorithm::FindPath(const GridCell3D& target) {
    Position3D currentPos = m_drone.GetLocation();
    GridCell3D start = WorldToGrid(currentPos.x, currentPos.y, currentPos.height);

    if (start == target) return {start};

    // Manhattan distance heuristic (admissible for uniform-cost grid)
    auto heuristic = [](const GridCell3D& a, const GridCell3D& b) {
        return std::abs(a.x - b.x) + std::abs(a.y - b.y) + std::abs(a.z - b.z);
    };

    // Min-heap ordered by f = g + h
    using Entry = std::pair<int, GridCell3D>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> openSet;

    std::map<GridCell3D, int>        gScore;
    std::map<GridCell3D, GridCell3D> parent;
    std::set<GridCell3D>             closed;

    gScore[start] = 0;
    parent[start] = start;
    openSet.push({heuristic(start, target), start});

    while (!openSet.empty()) {
        auto [f, current] = openSet.top();
        openSet.pop();

        if (closed.count(current)) continue;
        closed.insert(current);

        if (current == target) {
            // Reconstruct path from target back to start
            std::vector<GridCell3D> path;
            for (GridCell3D node = target; !(node == start); node = parent[node])
                path.push_back(node);
            path.push_back(start);
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (const GridCell3D& nb : std::vector<GridCell3D>{
                {current.x + m_bfsStep, current.y,           current.z},
                {current.x - m_bfsStep, current.y,           current.z},
                {current.x,             current.y + m_bfsStep, current.z},
                {current.x,             current.y - m_bfsStep, current.z}}) {
            if (closed.count(nb) || !IsWalkable(nb)) continue;

            int tentativeG = gScore[current] + m_bfsStep;
            if (!gScore.count(nb) || tentativeG < gScore[nb]) {
                gScore[nb] = tentativeG;
                parent[nb] = current;
                openSet.push({tentativeG + heuristic(nb, target), nb});
            }
        }
    }

    return {};  // no path found
}

// ---------------------------------------------------------------------------
// Movement: navigate to a target cell
// ---------------------------------------------------------------------------
bool MappingAlgorithm::MoveToCell(const GridCell3D& target) {
    Centi targetX, targetY, targetZ;
    GridToWorld(target, targetX, targetY, targetZ);

    // Adjust height first
    if (!AdjustHeight(targetZ)) {
        return false;
    }

    // Rotate to face target
    if (!RotateToFace(targetX, targetY)) {
        return false;
    }

    // Calculate distance
    Position3D currentPos = m_drone.GetLocation();
    double dx = targetX.numerical_value_in(si::centi<si::metre>) -
                currentPos.x.numerical_value_in(si::centi<si::metre>);
    double dy = targetY.numerical_value_in(si::centi<si::metre>) -
                currentPos.y.numerical_value_in(si::centi<si::metre>);
    double distCm = std::sqrt(dx * dx + dy * dy);

    // Move towards target in chunks
    int maxAdvance = m_config ? static_cast<int>(
        m_config->maxAdvance.numerical_value_in(si::centi<si::metre>)) : 50;

    while (distCm > 5.0) {
        int advance = static_cast<int>(std::min(static_cast<double>(maxAdvance), distCm));
        if (m_drone.Advance(advance * si::centi<si::metre>) ==
            MoveResult::CollisionDetected) {
            return false;
        }

        // Rescan for safety
        ScanAndUpdate();

        // Recalculate distance
        currentPos = m_drone.GetLocation();
        dx = targetX.numerical_value_in(si::centi<si::metre>) -
             currentPos.x.numerical_value_in(si::centi<si::metre>);
        dy = targetY.numerical_value_in(si::centi<si::metre>) -
             currentPos.y.numerical_value_in(si::centi<si::metre>);
        distCm = std::sqrt(dx * dx + dy * dy);
    }

    return true;
}

// ---------------------------------------------------------------------------
// Rotation helper
// ---------------------------------------------------------------------------
bool MappingAlgorithm::RotateToFace(Centi targetX, Centi targetY) {
    Position3D currentPos = m_drone.GetLocation();

    double dx = targetX.numerical_value_in(si::centi<si::metre>) -
                currentPos.x.numerical_value_in(si::centi<si::metre>);
    double dy = targetY.numerical_value_in(si::centi<si::metre>) -
                currentPos.y.numerical_value_in(si::centi<si::metre>);

    // Desired heading (0 = +X, 90 = +Y)
    double desiredHeading = std::atan2(dy, dx) * 180.0 / 3.14159265359;
    if (desiredHeading < 0) desiredHeading += 360.0;

    double maxRotateDeg = m_config
        ? m_config->maxRotate.numerical_value_in(si::degree)
        : 45.0;

    double angleDiff = desiredHeading - m_currentHeading;

    // Normalize to [-180, 180]
    if (angleDiff > 180.0) angleDiff -= 360.0;
    if (angleDiff < -180.0) angleDiff += 360.0;

    // Step in maxRotate-sized chunks so m_currentHeading stays in sync with
    // the driver's actual heading (which clamps each call to maxRotate).
    while (std::abs(angleDiff) > 1.0) {
        double step = angleDiff > 0
            ? std::min(angleDiff,  maxRotateDeg)
            : std::max(angleDiff, -maxRotateDeg);

        if (m_drone.Rotate(step * si::degree) == MoveResult::CollisionDetected) {
            return false;
        }
        m_currentHeading += step;
        if (m_currentHeading >= 360.0) m_currentHeading -= 360.0;
        if (m_currentHeading <    0.0) m_currentHeading += 360.0;

        angleDiff = desiredHeading - m_currentHeading;
        if (angleDiff > 180.0) angleDiff -= 360.0;
        if (angleDiff < -180.0) angleDiff += 360.0;
    }

    return true;
}

// ---------------------------------------------------------------------------
// Height adjustment helper
// ---------------------------------------------------------------------------
bool MappingAlgorithm::AdjustHeight(Centi targetHeight) {
    const double maxElevate = m_config
        ? m_config->maxElevate.numerical_value_in(si::centi<si::metre>)
        : 30.0;

    while (true) {
        Position3D currentPos = m_drone.GetLocation();
        double currentH = currentPos.height.numerical_value_in(si::centi<si::metre>);
        double targetH  = targetHeight.numerical_value_in(si::centi<si::metre>);
        double diff     = targetH - currentH;

        if (std::abs(diff) <= 1.0) break;  // close enough

        double step = (diff > 0)
            ? std::min(diff,  maxElevate)
            : std::max(diff, -maxElevate);

        if (m_drone.Elevate(step * si::centi<si::metre>) ==
            MoveResult::CollisionDetected) {
            return false;
        }
    }
    return true;
}

}  // namespace drone
