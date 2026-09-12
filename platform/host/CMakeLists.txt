add_executable(partsim_bench bench.cpp)
target_link_libraries(partsim_bench PRIVATE partsim_core)
target_compile_options(partsim_bench PRIVATE -Wall -Wextra)

add_executable(partsim_ppm ppm_dump.cpp)
target_link_libraries(partsim_ppm PRIVATE partsim_core)
target_compile_options(partsim_ppm PRIVATE -Wall -Wextra)

add_executable(partsim_dutyreport dutyreport.cpp)
target_link_libraries(partsim_dutyreport PRIVATE partsim_core)
target_compile_options(partsim_dutyreport PRIVATE -Wall -Wextra)

add_executable(partsim_memreport memreport.cpp)
target_link_libraries(partsim_memreport PRIVATE partsim_core)
target_compile_options(partsim_memreport PRIVATE -Wall -Wextra)

add_executable(partsim_golden golden.cpp)
target_link_libraries(partsim_golden PRIVATE partsim_core)
target_compile_options(partsim_golden PRIVATE -Wall -Wextra)

# The application layer on a second platform. See console_main.cpp for why this is the proof that
# the HAL extraction is real rather than a rename.
add_executable(partsim_console console_main.cpp)
target_link_libraries(partsim_console PRIVATE partsim_app)
target_compile_options(partsim_console PRIVATE -Wall -Wextra)

# The beaker overlay, as an image. See beaker_ppm.cpp: the hashes cannot tell you whether a
# glyph is legible at 3x5 on a 32x32 face.
add_executable(partsim_beaker_ppm beaker_ppm.cpp)
target_link_libraries(partsim_beaker_ppm PRIVATE partsim_core)
target_compile_options(partsim_beaker_ppm PRIVATE -Wall -Wextra)

# M4-B as an image: a beaker pouring into the next one. See spill_ppm.cpp -- a conserved count
# cannot tell you whether the liquid leaves as a stream or as a burst.
add_executable(partsim_spill_ppm spill_ppm.cpp)
target_link_libraries(partsim_spill_ppm PRIVATE partsim_core)
target_compile_options(partsim_spill_ppm PRIVATE -Wall -Wextra)
