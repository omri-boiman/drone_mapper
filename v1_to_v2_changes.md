# v1 → v2 Assignment Changes

## Overview

The v2 specification updates three sensor interfaces, one config format, and one config file schema. All existing v1 config keys remain parseable for backward compatibility. The changes below are listed from most impactful (sensor model rewrite) to least.

---

## 1. Lidar Model: Rectangular Matrix → Circular Ring Beams

### v1 Model

The lidar emitted an **N×N rectangular grid** of beams. Resolution was defined by two calibration points:

- `lidar_fov_deg` — total field-of-view cone angle
- `lidar_res_at_dist1_cm` / `lidar_dist1_cm` — beam spacing at distance 1
- `lidar_res_at_dist2_cm` / `lidar_dist2_cm` — beam spacing at distance 2

The result was a fixed-size 2D matrix of distances. Every beam always had a slot in the matrix (even if nothing was hit), which made result processing straightforward but wasteful.

### v2 Model

The lidar now emits beams arranged in **concentric circular rings** centred on the scan direction:

| Parameter | Config key | Meaning |
|-----------|-----------|---------|
| `lidarD` | `lidar_D_cm` | Spacing between adjacent ring beams at Z-min distance |
| `lidarFovc` | `lidar_fovc` | Number of rings (0 = centre beam only; 5 = centre + 4 rings) |

The angular step between consecutive rings is:

```
angStep = atan(D / Z-min)
```

Ring 0 has **1 beam** (the central beam). Ring N has **4^N beams** evenly spaced around the circle. For `lidarFovc = 5` this gives:

```
ring 0: 1 beam
ring 1: 4 beams
ring 2: 16 beams
ring 3: 64 beams
ring 4: 256 beams
Total: 341 beams
```

Each ring-N beam direction is computed by rotating the central direction vector by `N * angStep` around a random azimuth angle `phi ∈ [0, 2π)`. The rotation uses two orthogonal vectors **u** and **w** perpendicular to the central direction:

```
beam_direction = cos(theta)*central + sin(theta)*(cos(phi)*u + sin(phi)*w)
```

Near-vertical scans use a degeneracy guard — when `|cz| ≥ 0.9` the horizontal perpendicular vector is anchored to +X instead of being derived from the horizontal component.

**Implementation:** [MockLidarSensor.cpp:69-131](src/simulation/MockLidarSensor.cpp#L69)

```cpp
// u = horizontal perpendicular to central direction
if (std::abs(cz) < 0.9) {
    const double len = std::sqrt(cx * cx + cy * cy);
    ux = -cy / len;  uy = cx / len;  uz = 0.0;
} else {
    ux = 1.0;  uy = 0.0;  uz = 0.0;  // near-vertical degeneracy guard
}
const double wx = cy * uz - cz * uy;
const double wy = cz * ux - cx * uz;
const double wz = cx * uy - cy * ux;

for (int circle = 0; circle < fovc; ++circle) {
    const double theta    = circle * angStep;
    const int    numBeams = (circle == 0) ? 1
                          : static_cast<int>(std::round(std::pow(4.0, circle)));
    for (int beam = 0; beam < numBeams; ++beam) {
        const double phi = (numBeams > 1) ? (2π * beam / numBeams) : 0.0;
        double bx = cosTheta*cx + sinTheta*(cos(phi)*ux + sin(phi)*wx);
        // ... by, bz similarly ...
    }
}
```

**Config struct:** [ConfigParser.h:24-33](include/io/ConfigParser.h#L24)

```cpp
// v2 circular beam model parameters
Centi   lidarD    {5.0 * si::centi<si::metre>}; // beam-circle spacing at Z-min
int     lidarFovc {5};                           // number of beam circles

// v1 fields kept for backward-compat parsing (not used by v2 sensor)
Degrees lidarFov        {90.0 * si::degree};
Centi   lidarResAtDist1 {5.0  * si::centi<si::metre>};
// ...
```

---

## 2. Lidar Scan Result: Dense Matrix → Sparse Hit List

### v1 Result

`LidarScanResult` was a struct containing:

- `LidarMatrix cells` — an N×N 2D array of distances (raw doubles)
- `Degrees xy_angle` — heading the scan was fired at
- `Degrees pitch` — vertical tilt used

Sentinel values distinguished outcomes: `-1.0` = no hit (beyond Z-max), `-2.0` = too close (within Z-min).

Processing in the mapping algorithm iterated over every matrix cell by `[row][col]` index, reconstructed the ray direction from the matrix position, and then walked the ray.

### v2 Result

`LidarScanResult` is now a **`std::vector<LidarBeamHit>`** — only beams that *hit* something appear in the list. An empty vector means nothing was detected.

Each `LidarBeamHit` carries three fields:

```cpp
struct LidarBeamHit {
    Degrees azimuth;    // absolute horizontal angle (0 = +X axis)
    Degrees elevation;  // absolute vertical angle (0 = horizontal)
    double  distance;   // cm to hit surface; 0.0 = within Z-min
};
```

**Key implications:**

- No matrix size dependency — result length varies with scene geometry.
- Azimuth and elevation are **absolute** world angles (not offsets from the scan direction), so the mapping algorithm no longer needs to know what heading the scan was fired at.
- `distance == 0.0` replaces the old `-2.0` sentinel for within-Z-min hits.
- Missing hits (beyond Z-max) are simply absent from the list (no `-1.0` sentinel).

**Interface definition:** [ILidarSensor.h:9-22](include/interfaces/ILidarSensor.h#L9)

```cpp
struct LidarBeamHit {
    Degrees azimuth;
    Degrees elevation;
    double  distance;  // 0.0 = within Z-min
};
using LidarScanResult = std::vector<LidarBeamHit>;
```

**Mapping algorithm consumption** — old nested loop over matrix rows/cols replaced by a flat loop over hits. Ray direction is reconstructed from the absolute angles directly: [MappingAlgorithm.cpp:162-190](src/drone/MappingAlgorithm.cpp#L162)

```cpp
for (const auto& hit : hits) {
    if (hit.distance == 0.0) continue;  // within Z-min, position unknown

    const double azRad = hit.azimuth.numerical_value_in(si::radian);
    const double elRad = hit.elevation.numerical_value_in(si::radian);

    const double dx = std::cos(elRad) * std::cos(azRad);
    const double dy = std::cos(elRad) * std::sin(azRad);
    const double dz = std::sin(elRad);

    const double hitX = std::round(droneX + hit.distance * dx);
    m_drone.RecordCell(hitX * cm, hitY * cm, hitZ * cm, MapValue::Occupied);

    for (double d = 1.0; d < hit.distance; d += 1.0)
        m_drone.RecordCell((droneX + d*dx)*cm, ..., MapValue::Empty);
}
```

---

## 3. Position Sensor: Position Only → Position + Heading

### v1

`GetPosition()` returned `Position3D {x, y, height}` — three coordinates. The drone's heading was tracked internally by the mapping algorithm (`m_currentHeading`) by accumulating rotation commands.

### v2

The position sensor now **also reports the XY-Angle (heading)**. `Position3D` gains a fourth field:

**Types definition:** [Units.h:19-24](include/types/Units.h#L19)

```cpp
struct Position3D {
    Centi   x;
    Centi   y;
    Centi   height;
    Degrees heading {0.0 * si::degree};  // v2 addition
};
```

The mock implementation fills this from the shared simulation state:

**MockPositionSensor.cpp** — `GetPosition()` now reads:

```cpp
Position3D pos = m_state->position;
pos.heading = m_state->orientation.heading;
return pos;
```

This means the drone no longer needs dead-reckoning to know its heading — it can always ask the position sensor. The mapping algorithm still maintains `m_currentHeading` for its own rotation logic, but could use `GetLocation().heading` as a cross-check.

---

## 4. Mission Boundary: Arbitrary Polygon → Axis-Aligned Rectangle

### v1

The mission boundary was specified as an arbitrary polygon in `mission_config.txt`:

```
boundary_polygon = (0,0),(800,0),(800,600),(0,600)
```

The parser consumed this as a list of `(x, y)` vertex pairs and stored them in `MissionConfig::boundaryPolygon`. The mapping algorithm used ray-casting against this polygon for boundary checks.

### v2

The boundary is now specified as four axis-aligned limits:

```
boundary_xmin_cm = 0
boundary_xmax_cm = 800
boundary_ymin_cm = 0
boundary_ymax_cm = 600
```

Internally these are still stored as a polygon (a degenerate rectangle with four vertices), so all downstream geometry code — including the brute-force floor/ceiling injection — works unchanged.

**Config parser** — rectangle takes priority, falls back to `boundary_polygon` for v1 compatibility: [ConfigParser.cpp:224-237](src/io/ConfigParser.cpp#L224)

```cpp
const std::string rxmin = Find(kv, "boundary_xmin_cm");
// ...
if (!rxmin.empty() && !rxmax.empty() && !rymin.empty() && !rymax.empty()) {
    out.boundaryPolygon = {{xmin, ymin}, {xmax, ymin}, {xmax, ymax}, {xmin, ymax}};
} else {
    out.boundaryPolygon = ParsePolygon(Find(kv, "boundary_polygon"), f, logger);
}
```

**Scenario config files** — all four scenarios were converted from the polygon format to the rectangle format. See [scenario1/mission_config.txt](scenario1/mission_config.txt) and [scenario2/mission_config.txt](scenario2/mission_config.txt).

---

## 5. New Drone Config Fields

Two new keys appear in `drone_config.txt`:

```
lidar_D_cm = 5
lidar_fovc = 5
```

These map to `DroneConfig::lidarD` and `DroneConfig::lidarFovc` respectively. They are parsed in [ConfigParser.cpp:195-196](src/io/ConfigParser.cpp#L195):

```cpp
out.lidarD    = GetCenti(kv, "lidar_D_cm",  out.lidarD,    f, logger);
out.lidarFovc = GetInt  (kv, "lidar_fovc",  out.lidarFovc, f, logger);
```

All four scenario `drone_config.txt` files have been updated to include these keys.

---

## Summary Table

| Area | v1 | v2 |
|------|----|----|
| Lidar beam layout | N×N rectangular matrix | Concentric circular rings (1 + 4^1 + 4^2 + ...) |
| Lidar result type | `LidarScanResult` struct with `LidarMatrix cells` | `std::vector<LidarBeamHit>` sparse list |
| Hit sentinel (within Z-min) | `distance = -2.0` | `distance = 0.0` |
| Miss sentinel (beyond Z-max) | `distance = -1.0` in matrix | beam absent from list |
| Position sensor output | `{x, y, height}` | `{x, y, height, heading}` |
| Mission boundary format | `boundary_polygon = (x1,y1),...` | `boundary_xmin/xmax/ymin/ymax_cm` |
| New drone config keys | — | `lidar_D_cm`, `lidar_fovc` |
