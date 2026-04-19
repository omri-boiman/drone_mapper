# Drone Mapper

A C++20 drone building-mapping simulator.  
The drone navigates an unknown building, fires a simulated LiDAR, and builds a voxel map of what it discovers.

---

## Project structure - After Omri's First steps.

```
drone_mapper/
├── .devcontainer/
│   └── devcontainer.json        # VS Code Dev Container config
├── CMakeLists.txt               # Builds project; bootstraps Conan + mp-units automatically
├── CMakeUserPresets.json        # Defines the "release" configure preset
├── conanfile.txt                # Dependencies (mp-units 2.5.0)
├── Makefile                     # Convenience wrapper around cmake
├── include/
│   ├── drone/                   # BuildingMapImpl
│   ├── interfaces/              # IBuildingMap, ILidarSensor, …
│   ├── io/                      # ConfigParser, MapIO, ErrorLogger
│   ├── simulation/              # Mock sensors, GroundTruthMap
│   └── types/                   # Units, DroneCommand, MapValue
└── src/
    ├── drone/
    ├── io/
    ├── simulation/
    ├── main.cpp
    └── test_sensors.cpp         # Phase 2/3 smoke test (no GTest needed)
```

---

## Prerequisites (local machine)

| Tool | Version |
|------|---------|
| [Docker Desktop](https://www.docker.com/products/docker-desktop/) | latest |
| [VS Code](https://code.visualstudio.com/) | latest |
| VS Code extension — **Dev Containers** (`ms-vscode-remote.remote-containers`) | latest |

No compiler, CMake, or Conan needs to be installed on the host.  
Everything runs inside the container.

---

## Opening the project in the Dev Container

1. From the course Moodle, download the example container ZIP file.
2. Copy only the `.devcontainer` folder from it into the project root.
3. Open the project folder in VS Code.
4. When prompted by the pop-up in the bottom-right, click **Reopen in Container**.  
   Alternatively: press `F1` → type `Dev Containers: Reopen in Container` → press Enter.
5. Clone this project inside the container.

---

## Building

No manual setup is needed. On the very first build CMake automatically installs Conan and fetches `mp-units/2.5.0`.

```bash
make
```

Or equivalently:

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

---

## When to re-run make

| What changed | Command needed |
|---|---|
| Any `.cpp` or `.h` source file | `make` — incremental, only recompiles what changed |
| `CMakeLists.txt` | `make` — cmake reconfigures automatically |
| `conanfile.txt` (dependencies) | `make rebuild` — must re-run Conan to fetch new packages |
| Fresh container / first clone | `make` — everything is bootstrapped automatically |

---

## Running the programs

### Smoke test (Phase 2 & 3 mocks)

Runs without any input files. Prints PASS / FAIL for each check:

```bash
./build/sensor_test
```

Expected output ends with:
```
================================
  Passed: 37
  Failed: 0
================================
```

### Main simulator

```bash
./build/drone_mapper <path-to-input-folder>
```

The input folder must contain:
- `drone_config.txt`
- `mission_config.txt`
- `map_input.txt`

Outputs are written to the same folder:
- `output_map.txt`
- `input_errors.txt` (only if parse errors occurred)

---

## Rebuilding from scratch

```bash
make rebuild
```

Or manually:

```bash
rm -rf build
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

---

## Compiler flags

All targets are built with `g++ -std=c++20 -Wall -Wextra -Werror -pedantic`.  
The build will fail on any warning — fix warnings before committing.

---

## Dependencies

| Library | Version | How used |
|---------|---------|----------|
| [mp-units](https://mpusz.github.io/mp-units/) | 2.5.0 | Strong physical units (`Centi`, `Degrees`, …) throughout all APIs |

Fetched automatically by Conan on first build — no manual installation needed.
