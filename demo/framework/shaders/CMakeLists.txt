set(SHADER_OUTPUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/generated"
    CACHE INTERNAL "Directory holding generated shader headers")

if(APP_BUILD_SHADERS)
    if(NOT SOKOL_SHDC_EXECUTABLE OR NOT EXISTS "${SOKOL_SHDC_EXECUTABLE}")
        message(FATAL_ERROR
            "SOKOL_SHDC_EXECUTABLE is not set or does not exist: "
            "${SOKOL_SHDC_EXECUTABLE}")
    endif()
    file(MAKE_DIRECTORY "${SHADER_OUTPUT_DIR}")

    file(GLOB SHADER_COMMON_INCLUDES CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_LIST_DIR}/common/*.glsl")
endif()

set_property(GLOBAL PROPERTY SHADER_OUTPUTS "")
set_property(GLOBAL PROPERTY SHADER_INPUTS  "")

set(SHADER_LANGUAGES "hlsl5:metal_macos:glsl430")

function(add_shader name input)
    if(NOT IS_ABSOLUTE "${input}")
        set(input "${CMAKE_CURRENT_SOURCE_DIR}/${input}")
    endif()
    set(output "${SHADER_OUTPUT_DIR}/${name}.glsl.h")

    if(APP_BUILD_SHADERS)
        add_custom_command(
            OUTPUT  "${output}"
            COMMAND "${SOKOL_SHDC_EXECUTABLE}"
                    --input  "${input}"
                    --output "${output}"
                    --slang  "${SHADER_LANGUAGES}"

            DEPENDS "${input}" ${SHADER_COMMON_INCLUDES}
            COMMENT "sokol-shdc ${name}.glsl"
            VERBATIM
        )
        set_source_files_properties("${output}" PROPERTIES GENERATED TRUE)
    endif()

    set_property(GLOBAL APPEND PROPERTY SHADER_OUTPUTS "${output}")
    set_property(GLOBAL APPEND PROPERTY SHADER_INPUTS "${input}")
endfunction()

add_shader(cube "shapes/cube.glsl")
add_shader(sphere "shapes/sphere.glsl")
add_shader(capsule "shapes/capsule.glsl")
add_shader(geom "shapes/geom.glsl")
add_shader(edge "shapes/edge.glsl")
add_shader(shadow_caster_cube "shapes/shadow_caster_cube.glsl")
add_shader(shadow_caster_geom "shapes/shadow_caster_geom.glsl")
add_shader(shadow_caster_sphere "shapes/shadow_caster_sphere.glsl")
add_shader(shadow_caster_capsule "shapes/shadow_caster_capsule.glsl")

add_shader(sky "ibl/sky.glsl")
add_shader(brdf_lut "ibl/brdf_lut.glsl")
add_shader(sky_to_cube "ibl/sky_to_cube.glsl")
add_shader(prefilter "ibl/prefilter.glsl")

add_shader(depth_only_cube "post/depth_only_cube.glsl")
add_shader(depth_only_sphere "post/depth_only_sphere.glsl")
add_shader(depth_only_capsule "post/depth_only_capsule.glsl")
add_shader(depth_only_geom "post/depth_only_geom.glsl")
add_shader(gtao_prefilter_depth "post/gtao_prefilter_depth.glsl")
add_shader(gtao_main_pass "post/gtao_main_pass.glsl")
add_shader(gtao_denoise "post/gtao_denoise.glsl")
add_shader(tonemap "post/tonemap.glsl")
add_shader(mask_sphere "post/mask_sphere.glsl")
add_shader(mask_capsule "post/mask_capsule.glsl")
add_shader(mask_hull "post/mask_hull.glsl")
add_shader(highlight_outline "post/highlight_outline.glsl")

add_shader(overlay_line "overlays/line.glsl")
add_shader(overlay_point "overlays/point.glsl")

get_property(_outputs GLOBAL PROPERTY SHADER_OUTPUTS)
get_property(_inputs  GLOBAL PROPERTY SHADER_INPUTS)

if(APP_BUILD_SHADERS)

    add_custom_target(shaders
        DEPENDS ${_outputs}
        SOURCES ${_inputs} ${SHADER_COMMON_INCLUDES} ${_outputs}
    )

    source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}"
        FILES ${_inputs} ${SHADER_COMMON_INCLUDES})
    source_group("Generated" FILES ${_outputs})

    source_group("CMake Rules" FILES "${CMAKE_CURRENT_BINARY_DIR}/CMakeFiles/shaders")
else()

    add_custom_target(shaders SOURCES ${_inputs} ${_outputs})
endif()

add_library(shader_gen INTERFACE)
add_dependencies(shader_gen shaders)
target_include_directories(shader_gen SYSTEM INTERFACE "${SHADER_OUTPUT_DIR}")
