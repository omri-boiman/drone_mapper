cmake_minimum_required(VERSION 3.25)
project(drone_mapper LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(mp-units REQUIRED)

# Gather all .cpp sources under src/ (except main.cpp added separately)
file(GLOB_RECURSE DRONE_SOURCES
    "src/io/*.cpp"
    "src/simulation/*.cpp"
    "src/drone/*.cpp"
    "src/scoring/*.cpp"
)

add_executable(drone_mapper
    src/main.cpp
    ${DRONE_SOURCES}
)

target_include_directories(drone_mapper PRIVATE include)
target_compile_options(drone_mapper PRIVATE -Wall -Wextra -Werror -pedantic)
target_link_libraries(drone_mapper PRIVATE mp-units::mp-units)

# ----------------------------------------------------------------------------
# sensor_test — quick smoke test for Phase 2 & 3 mocks (no GTest required)
# ----------------------------------------------------------------------------
add_executable(sensor_test
    src/test_sensors.cpp
    ${DRONE_SOURCES}
)

target_include_directories(sensor_test PRIVATE include)
target_compile_options(sensor_test PRIVATE -Wall -Wextra -Werror -pedantic)
target_link_libraries(sensor_test PRIVATE mp-units::mp-units)
