if(APPLE)
    enable_language(OBJC)
endif()

FetchContent_Declare(imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        v1.92.7
    GIT_SHALLOW    TRUE
    GIT_PROGRESS   TRUE
)
FetchContent_MakeAvailable(imgui)

FetchContent_Declare(implot
    GIT_REPOSITORY https://github.com/epezent/implot.git
    GIT_TAG        v0.17
    GIT_SHALLOW    TRUE
    GIT_PROGRESS   TRUE
)
FetchContent_MakeAvailable(implot)

if(APP_BUILD_SHADERS)
    FetchContent_Declare(sokol_tools_bin
        GIT_REPOSITORY https://github.com/floooh/sokol-tools-bin.git
        GIT_TAG        1a9a4e54090fec42c5d13169b638f09f25474953
        GIT_PROGRESS   TRUE
    )
    FetchContent_MakeAvailable(sokol_tools_bin)
endif()

add_library(imgui STATIC
    "${imgui_SOURCE_DIR}/imgui.cpp"
    "${imgui_SOURCE_DIR}/imgui_draw.cpp"
    "${imgui_SOURCE_DIR}/imgui_tables.cpp"
    "${imgui_SOURCE_DIR}/imgui_widgets.cpp"
    "${imgui_SOURCE_DIR}/imgui_demo.cpp"
)
target_include_directories(imgui SYSTEM PUBLIC "${imgui_SOURCE_DIR}")
set_target_properties(imgui PROPERTIES CXX_STANDARD 20 CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF
    POSITION_INDEPENDENT_CODE ON FOLDER "demo/extern")

add_library(implot STATIC
    "${implot_SOURCE_DIR}/implot.cpp"
    "${implot_SOURCE_DIR}/implot_items.cpp"
    "${implot_SOURCE_DIR}/implot_demo.cpp"
)
target_link_libraries(implot PUBLIC imgui)
target_include_directories(implot SYSTEM PUBLIC "${implot_SOURCE_DIR}")
set_target_properties(implot PROPERTIES CXX_STANDARD 20 CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF
    POSITION_INDEPENDENT_CODE ON FOLDER "demo/extern")

set(sokol_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../extern/sokol")
add_library(sokol INTERFACE)
target_include_directories(sokol SYSTEM INTERFACE "${sokol_SOURCE_DIR}")
if(WIN32)
    target_compile_definitions(sokol INTERFACE SOKOL_D3D11)
    target_link_libraries(sokol INTERFACE d3d11 dxgi dxguid user32 gdi32 shell32 ole32)
elseif(APPLE)
    target_compile_definitions(sokol INTERFACE SOKOL_METAL)
    target_link_libraries(sokol INTERFACE
        "-framework Foundation" "-framework Cocoa" "-framework QuartzCore"
        "-framework Metal" "-framework MetalKit")
elseif(UNIX)
    target_compile_definitions(sokol INTERFACE SOKOL_GLCORE)
    find_package(OpenGL REQUIRED)
    find_package(Threads REQUIRED)
    find_package(X11 REQUIRED)
    if(NOT X11_Xi_FOUND OR NOT X11_Xcursor_FOUND)
        message(FATAL_ERROR "sokol_app needs Xi and Xcursor; install libxi-dev and libxcursor-dev.")
    endif()
    target_link_libraries(sokol INTERFACE
        OpenGL::GL Threads::Threads ${CMAKE_DL_LIBS} m X11::X11 X11::Xi X11::Xcursor)
else()
    message(FATAL_ERROR "Unsupported platform for sokol backend selection.")
endif()

if(APP_BUILD_SHADERS)
    if(WIN32)
        set(_shdc_subdir win32)
        set(_shdc_exe    sokol-shdc.exe)
    elseif(APPLE)
        if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm64|aarch64")
            set(_shdc_subdir osx_arm64)
        else()
            set(_shdc_subdir osx)
        endif()
        set(_shdc_exe sokol-shdc)
    else()
        set(_shdc_subdir linux)
        set(_shdc_exe    sokol-shdc)
    endif()
    set(SOKOL_SHDC_EXECUTABLE "${sokol_tools_bin_SOURCE_DIR}/bin/${_shdc_subdir}/${_shdc_exe}"
        CACHE FILEPATH "Path to sokol-shdc binary")
endif()

add_subdirectory(shaders)
add_subdirectory(gfx)
set_target_properties(gfx PROPERTIES FOLDER "demo/framework")

set(FRAMEWORK_FILES
    main.cpp
    sample.cpp
    sample.h
    math3d.h
    theme.h
    ui.cpp
    ui.h
    scene.cpp
    scene.h
    screenshot.cpp
    screenshot.h
    random.h
    jsmn.h

    host/camera.cpp
    host/camera.h
    host/gui.cpp
    host/gui.h
    host/imgui_impl.cpp
    host/sokol_app_impl.c
)

file(GLOB_RECURSE SCENE_FILES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/../scenes/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/../scenes/*.h"
)

add_executable(mech_ik_demo ${FRAMEWORK_FILES} ${SCENE_FILES})

if(APPLE)
    set_source_files_properties(host/sokol_app_impl.c PROPERTIES LANGUAGE OBJC)
    set_source_files_properties(host/sokol_app_impl.c PROPERTIES COMPILE_OPTIONS "-fobjc-arc")
endif()

target_include_directories(mech_ik_demo PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/../scenes")
target_include_directories(mech_ik_demo SYSTEM PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../extern/stb")

target_compile_definitions(mech_ik_demo PRIVATE APP_TITLE="${APP_TITLE}")
target_link_libraries(mech_ik_demo PRIVATE gfx imgui implot sokol)

if (MINGW)
    target_link_libraries(mech_ik_demo PRIVATE winmm)
endif()

set_target_properties(mech_ik_demo PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
    FOLDER "demo"
)

if(MSVC)
    target_compile_options(mech_ik_demo PRIVATE /W3 /utf-8)
endif()

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} PREFIX "framework" FILES ${FRAMEWORK_FILES})
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/../scenes" PREFIX "scenes" FILES ${SCENE_FILES})
