# The artifact's name is a build-time choice, because two CONFIGURATIONS of this module ship side
# by side: the default one every page loads and the determinism check guards, and a beaker-tier one
# with chroma compiled in. Emscripten's MODULARIZE output locates its .wasm by a filename baked
# into the .mjs, so renaming on copy silently breaks the second artifact -- the name has to be set
# here, not by cp.
set(PARTSIM_WASM_NAME "partsim" CACHE STRING "Basename of the emitted .mjs/.wasm")

add_executable(partsim bindings.cpp)
target_link_libraries(partsim PRIVATE partsim_core)
target_compile_options(partsim PRIVATE -Wall -Wextra -O3)

set_target_properties(partsim PROPERTIES
  OUTPUT_NAME "${PARTSIM_WASM_NAME}"
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
  "-sEXPORTED_FUNCTIONS=['_ps_init','_ps_panel_res','_ps_panel_pitch','_ps_particle_count','_ps_capacity','_ps_set_orientation','_ps_add_jerk','_ps_advance','_ps_render','_ps_panel_count','_ps_panel_w','_ps_panel_h','_ps_panel_ptr','_ps_panel_basis','_ps_stats','_ps_set_stats_timing','_ps_set_palette','_ps_palette_count','_ps_scene_count','_ps_scene','_ps_set_scene','_ps_scene_name','_ps_set_auto_cycle','_ps_auto_cycle','_ps_init_scene','_ps_state_hash','_ps_golden_hash','_ps_pixel_hash','_malloc','_free','_ps_node_init','_ps_node_count','_ps_node_face_count','_ps_node_face','_ps_node_step','_ps_node_frame_ptr','_ps_node_frame_len','_ps_node_master_step','_ps_node_deliver','_ps_node_last_step','_ps_node_ever_received','_ps_node_render','_ps_node_panel_ptr','_ps_node_reset','_ps_beaker_has_chroma','_ps_beaker_max','_ps_beaker_stat_count','_ps_beaker_set_chain_position','_ps_beaker_chain_id','_ps_beaker_chain_len','_ps_beaker_upstream','_ps_beaker_init','_ps_beaker_count','_ps_beaker_set_dye','_ps_beaker_dye_r','_ps_beaker_dye_g','_ps_beaker_reset','_ps_beaker_orient','_ps_beaker_jerk','_ps_beaker_step','_ps_beaker_out_ptr','_ps_beaker_out_len','_ps_beaker_deliver','_ps_beaker_fill','_ps_beaker_capacity','_ps_beaker_stats_ptr','_ps_beaker_chroma_ptr','_ps_beaker_render','_ps_beaker_panel_ptr','_ps_beaker_panel_count','_ps_beaker_panel_w','_ps_beaker_panel_h','_ps_beaker_panel_basis','_ps_beaker_open_face','_ps_beaker_rest_spacing']"
)
