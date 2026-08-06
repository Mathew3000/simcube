# Explicit source list, never GLOB: adding a file to the PlatformIO build and forgetting
# CMake (or vice versa) must fail loudly rather than silently diverge between targets.
add_library(partsim_core STATIC
  src/Geometry.cpp
  src/SimVolume.cpp
  src/SpatialHash.cpp
  src/Solver.cpp
  src/Renderer.cpp
  src/PaletteData.cpp
  src/Simulation.cpp
  src/FieldGrid.cpp
  src/ScenePresets.cpp
  src/MotionSource.cpp
  src/ChainMap.cpp
)

target_include_directories(partsim_core PUBLIC include)
target_compile_features(partsim_core PUBLIC cxx_std_17)

if(PARTSIM_PROFILE STREQUAL "esp32")
  target_compile_definitions(partsim_core PUBLIC PARTSIM_PROFILE_ESP32=1)
elseif(NOT PARTSIM_PROFILE STREQUAL "")
  message(FATAL_ERROR "Unknown PARTSIM_PROFILE '${PARTSIM_PROFILE}' (expected '' or 'esp32')")
endif()

# Only forward capacities that were explicitly set, so an unset one falls through to the
# profile defaults in Config.h rather than being pinned to a stale copy of them.
foreach(var PARTSIM_MAX_PARTICLES PARTSIM_MAX_PANELS PARTSIM_MAX_PANEL_TEXELS
            PARTSIM_MAX_GRID_CELLS PARTSIM_MAX_FIELD_CELLS PARTSIM_INTERNAL_PIXELS)
  if(NOT ${var} STREQUAL "")
    target_compile_definitions(partsim_core PUBLIC ${var}=${${var}})
  endif()
endforeach()

# -ffp-contract=off is load-bearing for cross-target determinism: Xtensa will fuse a*b+c
# into an FMA with different rounding than WASM's non-fused multiply-then-add, and the
# trajectories diverge within a few hundred steps.
target_compile_options(partsim_core PUBLIC -ffp-contract=off)
target_compile_options(partsim_core PRIVATE
  -Wall -Wextra -Werror -Wdouble-promotion -Wfloat-conversion)

if(PARTSIM_SANITIZE)
  target_compile_options(partsim_core PUBLIC -fsanitize=address,undefined -fno-omit-frame-pointer)
  target_link_options(partsim_core PUBLIC -fsanitize=address,undefined)
endif()
