add_executable(partsim_bench bench.cpp)
target_link_libraries(partsim_bench PRIVATE partsim_core)
target_compile_options(partsim_bench PRIVATE -Wall -Wextra)

add_executable(partsim_ppm ppm_dump.cpp)
target_link_libraries(partsim_ppm PRIVATE partsim_core)
target_compile_options(partsim_ppm PRIVATE -Wall -Wextra)
