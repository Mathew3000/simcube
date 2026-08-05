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
)

target_include_directories(partsim_core PUBLIC include)
target_compile_features(partsim_core PUBLIC cxx_std_17)

target_compile_definitions(partsim_core PUBLIC
  PARTSIM_MAX_PARTICLES=${PARTSIM_MAX_PARTICLES}
  PARTSIM_MAX_PANELS=${PARTSIM_MAX_PANELS}
  PARTSIM_MAX_PANEL_TEXELS=${PARTSIM_MAX_PANEL_TEXELS}
  PARTSIM_MAX_GRID_CELLS=${PARTSIM_MAX_GRID_CELLS}
  PARTSIM_MAX_FIELD_CELLS=${PARTSIM_MAX_FIELD_CELLS}
)

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
