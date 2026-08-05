add_executable(partsim_tests
  check.cpp
  test_math.cpp
  test_geometry.cpp
  test_hash.cpp
  test_solver.cpp
  test_renderer.cpp
  test_simulation.cpp
  test_sand.cpp
  test_field.cpp
  test_noalloc.cpp
)
target_link_libraries(partsim_tests PRIVATE partsim_core)
target_include_directories(partsim_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(partsim_tests PRIVATE -Wall -Wextra)

add_test(NAME unit COMMAND partsim_tests)

# core/ must never reach for libm transcendentals -- see core/include/partsim/Math.h.
add_test(NAME no_libm
         COMMAND ${CMAKE_COMMAND} -E env bash
                 ${CMAKE_SOURCE_DIR}/scripts/check_no_libm.sh ${CMAKE_SOURCE_DIR})

# Cross-target determinism: the WASM build must produce bit-identical particle state and
# pixels to the host. Skips (loudly) if scripts/build_wasm.sh has not been run.
add_test(NAME wasm_determinism
         COMMAND ${CMAKE_COMMAND} -E env bash
                 ${CMAKE_SOURCE_DIR}/scripts/check_wasm.sh)

# The golden hashes in scripts/golden_hash.txt must match what this build produces, or the
# WASM comparison is measuring against a stale reference.
add_test(NAME golden_hash_current
         COMMAND ${CMAKE_COMMAND} -E env bash
                 ${CMAKE_SOURCE_DIR}/scripts/check_golden.sh
                 $<TARGET_FILE:partsim_golden> ${CMAKE_SOURCE_DIR}/scripts/golden_hash.txt)
