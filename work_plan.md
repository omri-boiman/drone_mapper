# Drone Mapper — Assignment 1 Implementation Plan

## Context
TAU Advanced Topics in Programming, Semester B 2026 — Assignment 1 v2 (due May 17th).
Build a C++ simulator for an autonomous 3D-building-mapping drone. The drone uses mock
sensors (Lidar, Position, Movement) to navigate and construct a sparse 3D occupancy map.
No real hardware; everything is a software simulation. Next assignments will dictate a common
API, so design clean interfaces now to minimize future refactoring pain.

> **v2 vs v1 changes (April 2026):** deadline extended to May 17; position sensor now also
> returns XY-Angle (heading); lidar model replaced (matrix → circular beams, see Phase 2);
> lidar scan result changed (NxN matrix → sparse hit list with azimuth+distance); mission
> boundary simplified (polygon → bounded rectangle); FAQ clarifies blind spots cannot be
> interpolated — every cell must be actively mapped.

## Submission Requirements (from Submission Guidelines)
- **Compiler**: gcc 11.4+, flags: `g++ -std=c++20 -Wall -Wextra -Werror -pedantic`
- **Standard**: C++20 (not C++23)
- **Build**: Makefile or CMake — must compile on Linux
- **No binary files** in submission
- **Output file name**: `output_map.txt` (written to same path as input files)
- **External libraries**: only standard C++ allowed unless forum approval granted
  - `mp-unit` — **approved**, add usage explanation to readme
- **Three test input sets**: each in its own folder, each with a nested `original_output/` subfolder containing the pre-run output
- **readme file**: contributor names + IDs, input/output format description
- **HLD PDF**: in the main submission folder
- **bonus.txt**: if requesting any bonus (with file paths and line numbers)
- **Zip**: `ID1_ID2.zip`

---

## Project Structure

```
Drone Project/
├── CMakeLists.txt
├── include/
│   ├── types/
│   │   ├── Units.h           # mp-unit aliases: Meters, Degrees, Position3D, Orientation
│   │   ├── MapValue.h        # enum: Empty=0, Occupied=1, NotMapped=-1, BeyondBounds=-2
│   │   └── DroneCommand.h    # std::variant of all command types (for use by the mapping algorithm)
│   ├── interfaces/
│   │   ├── ILidarSensor.h
│   │   ├── IPositionSensor.h
│   │   ├── IMovementDriver.h
│   │   └── IBuildingMap.h
│   ├── simulation/
│   │   ├── SimulationState.h     # shared_ptr state between all 3 mocks
│   │   ├── MockLidarSensor.h
│   │   ├── MockPositionSensor.h
│   │   ├── MockMovementDriver.h
│   │   └── GroundTruthMap.h     # loads map_input.txt, NOT visible to Drone
│   ├── drone/
│   │   ├── DroneConfig.h
│   │   ├── MissionConfig.h
│   │   ├── Drone.h              # hardware abstraction layer — imperative robot API
│   │   └── BuildingMapImpl.h    # Drone's own IBuildingMap implementation
│   ├── io/
│   │   ├── ConfigParser.h
│   │   ├── MapIO.h
│   │   └── ErrorLogger.h        # writes input_errors.txt on demand
│   └── scoring/
│       └── Scorer.h
├── src/
│   ├── main.cpp
│   ├── simulation/{*.cpp}
│   ├── drone/{*.cpp}
│   ├── io/{*.cpp}
│   └── scoring/Scorer.cpp
├── scenario1/                  # test set 1 (e.g. simple rectangular room)
│   ├── drone_config.txt
│   ├── mission_config.txt
│   ├── map_input.txt
│   └── original_output/
│       └── output_map.txt
├── scenario2/                  # test set 2 (e.g. inaccessible parts)
│   ├── drone_config.txt
│   ├── mission_config.txt
│   ├── map_input.txt
│   └── original_output/
│       └── output_map.txt
├── scenario3/                  # test set 3 (e.g. complex boundary polygon)
│   ├── drone_config.txt
│   ├── mission_config.txt
│   ├── map_input.txt
│   └── original_output/
│       └── output_map.txt
├── tests/                      # GTest-based (bonus)
├── docs/                       # HLD PDF goes here AND in main folder
├── readme.txt                  # names, IDs, input/output format description
├── HLD.pdf                     # mandatory, in main folder
└── bonus.txt                   # only if requesting bonus
```

---

## Key Architectural Decisions

### Interfaces vs Mocks (strict separation)
The Drone class holds only interface references (`ILidarSensor&`, `IPositionSensor&`,
`IMovementDriver&`, `IBuildingMap&`). It never knows it's talking to mocks.

### Shared Simulation State
All three mocks hold a `shared_ptr<SimulationState>` which contains the drone's current
`Position3D` and `Orientation`. Movement driver writes to it; position sensor reads from it;
lidar sensor reads it to know where to cast rays from.

### Drone as Hardware Abstraction Layer
`Drone` is a thin, imperative wrapper over the four interfaces — no algorithm logic.
It exposes `Rotate()`, `Advance()`, `Elevate()`, `Scan()`, `GetLocation()`,
`RecordCell()`, and `QueryCell()`. Each method delegates to exactly one interface.
The mapping algorithm lives in a separate class that holds a `Drone&` and calls it.
`DroneCommand.h` is available for the algorithm to model its internal state machine.

### Sparse Map Storage
`std::unordered_map<std::tuple<int,int,int>, MapValue>` with integer grid indices.
Missing key = `NotMapped (-1)`. Out-of-bounds returns `BeyondBounds (-2)`.

### C++20 Constraint
Use C++20 (not C++23). Avoid C++23-only features. Key C++20 features available:
`std::variant`, `std::span`, concepts, ranges, `std::numbers::pi`. No `std::print`.

---

## File Formats (our choices for Ex1)

### drone_config.txt
```
min_pass_width_cm = 60
min_pass_length_cm = 60
min_pass_height_cm = 120
# Lidar — v2 circular beam model:
lidar_zmin_cm = 20           # min operational distance; hit reported as distance=0 below this
lidar_zmax_cm = 1000         # max operational distance; no detection beyond this
lidar_D_cm = 5               # spacing between consecutive beam circles at Z-min distance
                             # (circle 1 radius = D, circle 2 radius = 2D, etc.)
lidar_fovc = 5               # number of beam circles (0=centre only; N circles → 1+4+16+…+4^N beams)
max_rotate_deg = 45
max_advance_cm = 50
max_elevate_cm = 30
# v1 fields kept for backward-compat parsing (ignored if lidar_fovc present):
# lidar_fov_deg, lidar_res_at_dist1_cm, lidar_dist1_cm, lidar_res_at_dist2_cm, lidar_dist2_cm
```

### mission_config.txt
```
# v2: boundary is a simple rectangle (replaces v1 polygon)
boundary_xmin_cm = 0
boundary_xmax_cm = 2000
boundary_ymin_cm = 0
boundary_ymax_cm = 1500
min_height_cm = 0
max_height_cm = 300
output_resolution_xy_decimals = 2
output_resolution_h_decimals = 2
start_x_cm = 100
start_y_cm = 100
start_height_cm = 150
```

### map_input.txt / map_output.txt (identical format)
```
VERSION 1
BOUNDS xmin=0.00 xmax=20.00 ymin=0.00 ymax=15.00 hmin=0.00 hmax=3.00
# Only occupied cells stored (value=1). Absent cells = not mapped.
CELL <x_cm> <y_cm> <h_cm> <value>
CELL 10.00 0.00 150.00 1
...
END
```

---

## Implementation Phases (do in order, one by one)

### Phase 1 — Foundation (compile skeleton) ✅ COMPLETE
1. `CMakeLists.txt` — fetch mp-unit via `FetchContent`, set **C++20**, add flags `-Wall -Wextra -Werror -pedantic`. Also add a `Makefile` wrapper that calls cmake for Linux compatibility.
2. `include/types/Units.h` — mp-unit aliases (`Meters`, `Degrees`, `Position3D`, `Orientation`)
3. `include/types/MapValue.h` — enum class
4. `include/types/DroneCommand.h` — variant of all command structs
5. All 4 interface headers (pure abstract, header-only)
6. `src/io/ErrorLogger.cpp` — file writer for input_errors.txt
7. `src/io/MapIO.cpp` — parse/write the CELL-based format
8. `src/io/ConfigParser.cpp` — parse both config files, use defaults + log errors
9. `src/simulation/GroundTruthMap.cpp` — in-memory store from map_input.txt
10. `src/main.cpp` skeleton — load files, log errors, exit cleanly

**Milestone:** Compiles; reads all three files; prints any config errors.

### Phase 2 — Mock Sensors ✅ COMPLETE (v1 model; needs v2 lidar adaptation)
11. `SimulationState` — simple struct, `shared_ptr` shared by all mocks
12. `MockPositionSensor` — returns `state->current_position`
    - **v2 change:** must also return XY-Angle (heading) from `state->orientation`
13. `MockMovementDriver` — updates state position/orientation; enforces max-per-request; checks GroundTruthMap for collision (→ failure notice + finish)
14. `MockLidarSensor` — **v2 replaces matrix model with circular beam model:**
    - **Circle 0**: single beam along centre of scan direction
    - **Circle N** (N=1..FOVC-1): ring of `4^N` beams evenly distributed around circumference at angular radius `N * atan(D / Z-min)` from centre
    - Total beams = 1 + 4 + 16 + … + 4^(FOVC-1) = (4^FOVC − 1) / 3
    - Cast each beam as a ray; step from 1 cm to Z-max
    - **v2 result format:** sparse list of `{azimuth_3d, distance}` for each beam that hits; distance=0 if hit is within Z-min; list is empty if no beams hit anything
    - Beams that miss (no hit within Z-max) are omitted from the result
    - **Blind spots**: gaps between beams are real — the algorithm must actively map every required-resolution cell (no interpolation allowed per FAQ)

**Milestone:** Unit test Lidar on one-wall room; verify correct azimuth+distance returned.

### Phase 3 — Building Map (Drone's own map) ✅ COMPLETE
15. `BuildingMapImpl` — sparse map with mission-boundary check; coordinate-to-index using output resolution; `Get`/`Set` API
    - **v2 change:** boundary is a rectangle (xmin/xmax/ymin/ymax) instead of an arbitrary polygon; the existing polygon implementation handles this as a degenerate case and can be kept

**Milestone:** Unit test bounds checking and round-trip Set/Get.

### Phase 4 — Simulation Loop Wiring
16. Wire `main.cpp`: construct `SimulationState`, all four mocks, `BuildingMapImpl`,
    and `Drone`. Set `state->position` from `missionConfig.startX/Y/Height`.
17. Instantiate the mapping algorithm (Phase 5 class) with `Drone&`. Call `algo.Run()`.
    On `CollisionDetected` the algorithm must stop and print a failure notice.
18. After `algo.Run()` returns: write `output_map.txt` via `WriteMapFile()`,
    compute and print score.

**Milestone:** Algorithm stub that immediately returns runs without crash;
`output_map.txt` is created (empty map is fine at this stage).

### Phase 5 — Mapping Algorithm
Create a class (e.g. `MappingAlgorithm`) that takes `Drone&` in its constructor
and exposes a single `Run()` method. It calls the Drone API directly — no main-loop
dispatch, no callbacks needed.

19. Scan-and-update: call `drone.Scan()`, parse **v2 hit list** (azimuth+distance per hit),
    compute world hit positions from azimuth angles + current heading,
    call `drone.RecordCell()` for each hit; also record empty cells along each ray
20. BFS frontier: `std::queue<GridCell3D>` of cells to visit; `visited` set
21. XY-plane A* path planning on known-empty cells (query via `drone.QueryCell()`)
22. Movement execution: split large moves into max-size chunks; rotate to face target
    first; check return value of every `drone.Advance()` / `drone.Elevate()` call
23. Height exploration: after XY frontier exhausted at current height, `drone.Elevate()`
    to next unvisited height slice
24. Quick forward scan after each advance step (collision re-check)
25. Return from `Run()` when BFS frontier is empty and all height slices covered

**Milestone:** Drone maps a simple rectangular room; score > 70%.

### Phase 6 — Integration & Scoring
26. Test multi-room floor plan (door openings, corridors)
27. Test multi-height building
28. Score formula:
    ```
    score = (correctly_occupied + correctly_empty) /
            (ground_truth_occupied + reachable_empty_cells) * 100
    ```
    Rasterize both maps to output-resolution grid before comparison.
29. Test all error paths: missing fields, malformed map, out-of-bounds start

### Phase 7 — Polish & Docs
30. `readme.txt` — contributor names + IDs, description of all file formats
31. HLD document (PDF): UML class diagram, UML sequence diagram (main flow), design
    considerations, testing approach. Place in main folder as `HLD.pdf`
32. Create three scenario folders with varied test cases:
    - `scenario1/` — simple rectangular room (baseline)
    - `scenario2/` — building with inaccessible rooms (drone can't enter, score < 100 naturally)
    - `scenario3/` — non-square rectangular boundary with internal obstacles (v2: boundary is always a rectangle)
    Each with pre-run `original_output/output_map.txt` inside
33. Add `mp-unit` usage explanation to `readme.txt` (required since it's an approved external library)
34. (Bonus) GTest unit tests for ConfigParser, Lidar, BuildingMap, Scorer
35. (Bonus) Logging (timestamped log file)
36. (Bonus) Visual simulation utility (external tool reads input/output files)
37. `bonus.txt` with file paths and line numbers for any bonus features
38. Zip as `ID1_ID2.zip` — verify no binary files included

---

## Critical Files (in implementation order)
- `CMakeLists.txt`
- `include/types/Units.h`
- `include/interfaces/ILidarSensor.h`, `IPositionSensor.h`, `IMovementDriver.h`, `IBuildingMap.h`
- `src/io/ConfigParser.cpp` + `src/io/MapIO.cpp`
- `src/simulation/SimulationState.h` + `MockLidarSensor.cpp`
- `src/drone/BuildingMapImpl.cpp`
- `src/drone/Drone.cpp` (hardware abstraction — thin delegating wrapper, already done)
- `src/drone/MappingAlgorithm.cpp` (algorithm core — to be implemented in Phase 5)
- `src/main.cpp` (simulation loop)
- `src/scoring/Scorer.cpp`

---

## Verification
1. Build on Linux: `g++ -std=c++20 -Wall -Wextra -Werror -pedantic ...` or via `make` / `cmake -B build && cmake --build build`
2. Run: `./drone_mapper ./scenario1` → check `scenario1/output_map.txt` created
3. Check score printed to stdout (target > 70 for simple test room)
4. Verify no crash on malformed config → check `input_errors.txt` created alongside input files
5. Verify collision detection fires proper failure notice
6. Run all three scenarios and verify output makes sense for each
7. Check zip contains no binary files
8. (Bonus) Run GTest suite: `ctest --test-dir build`
