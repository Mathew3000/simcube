add_executable(partsim_bench bench.cpp)
target_link_libraries(partsim_bench PRIVATE partsim_core)
target_compile_options(partsim_bench PRIVATE -Wall -Wextra)

add_executable(partsim_ppm ppm_dump.cpp)
target_link_libraries(partsim_ppm PRIVATE partsim_core)
target_compile_options(partsim_ppm PRIVATE -Wall -Wextra)

add_executable(partsim_memreport memreport.cpp)
target_link_libraries(partsim_memreport PRIVATE partsim_core)
target_compile_options(partsim_memreport PRIVATE -Wall -Wextra)

add_executable(partsim_golden golden.cpp)
target_link_libraries(partsim_golden PRIVATE partsim_core)
target_compile_options(partsim_golden PRIVATE -Wall -Wextra)
