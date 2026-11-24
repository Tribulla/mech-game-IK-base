cmake_minimum_required(VERSION 3.22)

project(mech_ik_core
	VERSION 1.0.0
	DESCRIPTION "Multithreaded procedural mech IK and gait core (C11 + Python)"
	LANGUAGES C CXX
)

option(GAITCORE_BUILD_TESTS    "Build C unit and determinism tests"        ${PROJECT_IS_TOP_LEVEL})
option(GAITCORE_BUILD_EXAMPLES "Build minimal C integration example"       ${PROJECT_IS_TOP_LEVEL})
option(GAITCORE_BUILD_DEMO     "Build interactive 3D sokol/ImGui demo app" ${PROJECT_IS_TOP_LEVEL})
option(GAITCORE_ENABLE_PYTHON  "Embed Python 3.10+ in the demo when found" ON)

if(PROJECT_IS_TOP_LEVEL)
	set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
	set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
	set_property(GLOBAL PROPERTY USE_FOLDERS ON)

	set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin")
	set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin")

	if(NOT FETCHCONTENT_BASE_DIR)
		set(FETCHCONTENT_BASE_DIR "${CMAKE_SOURCE_DIR}/.fetchcontent-cache"
			CACHE PATH "Shared FetchContent populate cache")
	endif()

	if(GAITCORE_BUILD_TESTS)
		enable_testing()
	endif()
endif()

add_subdirectory(IK/c)

if(GAITCORE_BUILD_EXAMPLES)
	add_subdirectory(examples)
endif()

if(GAITCORE_BUILD_DEMO)
	set(APP_TITLE "Mech IK Core Demo" CACHE STRING "Window title shown by the demo app")
	include(FetchContent)
	add_subdirectory(demo)
	if(MSVC AND PROJECT_IS_TOP_LEVEL)
		set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT mech_ik_demo)
	endif()
endif()
