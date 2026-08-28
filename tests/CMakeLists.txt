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
  test_motion.cpp
  test_chainmap.cpp
  test_device_loop.cpp
  test_resolution.cpp
  test_renderstate.cpp
  test_simframe.cpp
  test_multinode.cpp
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

# The device capacity profile: pools must fit internal SRAM, and the physics at those
# capacities must still be reproducible. Runs without any hardware attached.
if(PARTSIM_PROFILE STREQUAL "")
  add_test(NAME esp32_budget
           COMMAND ${CMAKE_COMMAND} -E env bash
                   ${CMAKE_SOURCE_DIR}/scripts/check_esp32_budget.sh)

  # The third leg of the determinism check: the golden sequence on emulated Xtensa. Slow (~90s of
  # emulated CPU) and skips loudly without Espressif's QEMU fork, so it is opt-in via
  # -DPARTSIM_QEMU_TEST=ON rather than part of every run.
  if(PARTSIM_QEMU_TEST)
    add_test(NAME qemu_determinism
             COMMAND ${CMAKE_COMMAND} -E env bash
                     ${CMAKE_SOURCE_DIR}/scripts/check_qemu.sh)
    set_tests_properties(qemu_determinism PROPERTIES TIMEOUT 1200)
  endif()

  # ...and the firmware must still compile and link for Xtensa. Skips if PlatformIO is absent.
  add_test(NAME esp32_build
           COMMAND ${CMAKE_COMMAND} -E env bash
                   ${CMAKE_SOURCE_DIR}/scripts/check_esp32_build.sh)
endif()

# The golden hashes in scripts/golden_hash.txt must match what this build produces, or the
# WASM comparison is measuring against a stale reference.
add_test(NAME golden_hash_current
         COMMAND ${CMAKE_COMMAND} -E env bash
                 ${CMAKE_SOURCE_DIR}/scripts/check_golden.sh
                 $<TARGET_FILE:partsim_golden> ${CMAKE_SOURCE_DIR}/scripts/golden_hash.txt)
