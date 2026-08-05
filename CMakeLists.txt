cmake_minimum_required(VERSION 3.16)
project(partsim CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
endif()

# Compile-time capacities. The ESP32 build overrides these downward in platformio.ini;
# host/wasm can afford far more. See core/include/partsim/Config.h.
set(PARTSIM_MAX_PARTICLES 16384 CACHE STRING "Particle pool capacity")
set(PARTSIM_MAX_PANELS    8     CACHE STRING "Panel table capacity")
set(PARTSIM_MAX_PANEL_TEXELS 4096 CACHE STRING "Max texels (w*h) per panel")
set(PARTSIM_MAX_GRID_CELLS 32768 CACHE STRING "Neighbour-grid cell capacity")
set(PARTSIM_MAX_FIELD_CELLS 32768 CACHE STRING "Heat/smoke field cell capacity")

option(PARTSIM_SANITIZE "Build with ASan+UBSan" OFF)

add_subdirectory(core)

if(EMSCRIPTEN)
  add_subdirectory(platform/wasm)
else()
  add_subdirectory(platform/host)
  enable_testing()
  add_subdirectory(tests)
endif()
