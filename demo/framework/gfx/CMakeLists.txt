set(GFX_FILES
    arena.c
    arena.h
    draw_overlay.h
    draw.c
    draw.h
    edges.c
    edges.h
    geometry_registry.c
    geometry_registry.h
    gtao.c
    gtao.h
    highlight_mask.c
    highlight_mask.h
    highlight_outline.c
    highlight_outline.h
    highlight_target.c
    highlight_target.h
    ibl.c
    ibl.h
    overlay.c
    overlay.h
	projection.h
    qsort.h
    renderer.c
    renderer.h
    scene_target.c
    scene_target.h
    shadow.c
    shadow.h
    sky.c
    sky.h
    sokol_impl.c
    text.c
    timer.c
    timer.h
    text.h
    tone_map.c
    tone_map.h
    utility.h
)

add_library(gfx STATIC ${GFX_FILES})

if(APPLE)
    set_source_files_properties(
        sokol_impl.c
        PROPERTIES LANGUAGE OBJC)
    target_compile_options(gfx PRIVATE -fobjc-arc)
endif()

target_include_directories(gfx PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/..")
target_link_libraries(gfx
    PUBLIC  sokol
    PRIVATE shader_gen
)

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${GFX_FILES})

if(UNIX AND NOT APPLE)
    target_link_libraries(gfx PUBLIC m)
endif()
