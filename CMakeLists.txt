cmake_minimum_required(VERSION 3.16)
project(partsim CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
endif()

# Compile-time capacities. Left EMPTY by default so core/include/partsim/Config.h is the single
# place the numbers live -- otherwise the ESP32 profile below would be silently overridden by
# CMake's own copy of the host defaults, and a build claiming to measure the device would be
# measuring the host. Set one to force it.
set(PARTSIM_MAX_PARTICLES    "" CACHE STRING "Particle pool capacity (empty = Config.h)")
set(PARTSIM_MAX_PANELS       "" CACHE STRING "Panel table capacity")
set(PARTSIM_MAX_PANEL_TEXELS "" CACHE STRING "Max texels (w*h) per panel")
set(PARTSIM_MAX_GRID_CELLS   "" CACHE STRING "Neighbour-grid cell capacity")
set(PARTSIM_MAX_FIELD_CELLS  "" CACHE STRING "Heat/smoke field cell capacity")
set(PARTSIM_INTERNAL_PIXELS  "" CACHE STRING "Keep an RGBA copy of every panel (1/0)")

# "esp32" selects the device capacity profile from Config.h, so the host tools can be built
# against exactly the pools the firmware ships with. See scripts/check_esp32_budget.sh.
set(PARTSIM_PROFILE "" CACHE STRING "Capacity profile: empty (host) or esp32")

option(PARTSIM_SANITIZE "Build with ASan+UBSan" OFF)

add_subdirectory(core)

if(EMSCRIPTEN)
  add_subdirectory(platform/wasm)
else()
  add_subdirectory(platform/host)
  enable_testing()
  add_subdirectory(tests)
endif()
