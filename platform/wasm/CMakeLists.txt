add_executable(partsim bindings.cpp)
target_link_libraries(partsim PRIVATE partsim_core)
target_compile_options(partsim PRIVATE -Wall -Wextra -O3)

set_target_properties(partsim PROPERTIES
  OUTPUT_NAME "partsim"
  SUFFIX ".mjs")

# ALLOW_MEMORY_GROWTH is off on purpose: growth swaps in a new ArrayBuffer and detaches the
# old one, silently zero-lengthing the cached panel views (black textures, no error). Nothing
# is allocated after init, so growth would buy nothing.
#
# ENVIRONMENT includes node so the determinism harness can import the same artifact the
# browser loads -- testing a separate build would prove nothing.
target_link_options(partsim PRIVATE
  -O3
  --no-entry
  -sMODULARIZE=1
  -sEXPORT_ES6=1
  -sEXPORT_NAME=createPartsim
  -sINITIAL_MEMORY=33554432
  -sALLOW_MEMORY_GROWTH=0
  -sSTACK_SIZE=1048576
  -sENVIRONMENT=web,node
  -sFILESYSTEM=0
  -sMALLOC=emmalloc
  -sASSERTIONS=0
  -sEXPORTED_RUNTIME_METHODS=HEAPU8,HEAPF32
  "-sEXPORTED_FUNCTIONS=['_ps_init','_ps_particle_count','_ps_capacity','_ps_set_orientation','_ps_add_jerk','_ps_advance','_ps_render','_ps_panel_count','_ps_panel_w','_ps_panel_h','_ps_panel_ptr','_ps_panel_basis','_ps_stats','_ps_set_stats_timing','_ps_set_palette','_ps_palette_count','_ps_state_hash','_ps_golden_hash','_ps_pixel_hash']"
)
