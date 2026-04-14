# Drone Mapper

A C++20 drone building-mapping simulator.  
The drone navigates an unknown building, fires a simulated LiDAR, and builds a voxel map of what it discovers.

---

## Project structure

```
drone_mapper/
├── Dockerfile                   # Container image definition
├── .devcontainer/
│   └── devcontainer.json        # VS Code Dev Container config
├── CMakeLists.txt
├── CMakeUserPresets.json        # Wires cmake --preset to Conan output
├── conanfile.txt                # Dependencies (mp-units 2.5.0)
├── Makefile                     # Convenience wrapper
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

Open the project folder in VS Code.

Ensure you have the following two files configured in your root directory:

### Dockerfile

```Dockerfile
FROM ubuntu:24.04

# Prevent interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Install only GCC 13 and necessary tools
RUN apt-get update && apt-get install -y \
    build-essential \
    gcc-13 \
    g++-13 \
    cmake \
    git \
    make \
    python3-pip \
    python3-venv \
    && rm -rf /var/lib/apt/lists/*

# Set GCC 13 as the absolute system default
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-13 100 \
    --slave /usr/bin/g++ g++ /usr/bin/g++-13

# Install Conan 2.x
RUN pip3 install --break-system-packages conan

WORKDIR /workspaces/drone_mapper

```
### .devcontainer/devcontainer.json
```JSON
{
    "name": "Drone Mapper GCC 13",
    "build": { "dockerfile": "../Dockerfile" },
    "customizations": {
        "vscode": {
            "extensions": [
                "ms-vscode.cpptools",
                "ms-vscode.cmake-tools"
            ]
        }
    },
    "remoteUser": "root",
    "workspaceFolder": "/workspaces/drone_mapper"
}
```
When prompted by the pop-up in the bottom-right, click Reopen in Container.

Alternatively, press F1, type "Dev Containers: Reopen in Container", and press Enter.

Once the container starts, verify the compiler by running g++ --version in the terminal. It should report version 13.x.x.


---

## Building

### Step 1 — Install dependencies with Conan

Run this whenever `conanfile.txt` changes (or on a fresh container):

```bash
conan install . --build=missing -s build_type=Release
```

Conan downloads `mp-units/2.5.0` (and its transitive deps), compiles anything that has no pre-built binary, and writes CMake toolchain files into `build/`.

### Step 2 — Configure CMake

```bash
cmake --preset conan-release
```

This reads the toolchain Conan generated and configures the project.

### Step 3 — Compile

```bash
cmake --build build/build/Release
```

**Or use the Makefile shortcut (steps 2 + 3 together):**

```bash
make
```

> `make` will not re-run `conan install`. Always run Step 1 manually when dependencies change.

---

## Running the programs

### Smoke test (Phase 2 & 3 mocks)

Runs without any input files. Prints PASS / FAIL for each check:

```bash
./build/build/Release/sensor_test
```

Expected output ends with:
```
================================
  Passed: 22
  Failed: 0
================================
```

### Main simulator

```bash
./build/build/Release/drone_mapper <path-to-input-folder>
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
cmake --build build/build/Release --target clean
cmake --preset conan-release
cmake --build build/build/Release
```

---

## Compiler flags

All targets are built with `-Wall -Wextra -Werror -pedantic`.  
The build will fail on any warning — fix warnings before committing.

---

## Dependencies

| Library | Version | How used |
|---------|---------|----------|
| [mp-units](https://mpusz.github.io/mp-units/) | 2.5.0 | Strong physical units (`Centi`, `Degrees`, …) throughout all APIs |

Managed by Conan; no manual installation needed.
