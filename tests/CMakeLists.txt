add_executable(partsim_tests
  check.cpp
  test_math.cpp
  test_geometry.cpp
  test_hash.cpp
  test_solver.cpp
  test_renderer.cpp
)
target_link_libraries(partsim_tests PRIVATE partsim_core)
target_include_directories(partsim_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(partsim_tests PRIVATE -Wall -Wextra)

add_test(NAME unit COMMAND partsim_tests)

# core/ must never reach for libm transcendentals -- see core/include/partsim/Math.h.
add_test(NAME no_libm
         COMMAND ${CMAKE_COMMAND} -E env bash
                 ${CMAKE_SOURCE_DIR}/scripts/check_no_libm.sh ${CMAKE_SOURCE_DIR})
