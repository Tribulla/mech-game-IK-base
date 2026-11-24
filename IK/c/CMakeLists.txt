cmake_minimum_required(VERSION 3.22)

project(gaitcore_c VERSION 1.0.0 LANGUAGES C)

option(GAITCORE_BUILD_TESTS "Build gaitcore_c unit and determinism tests" ${PROJECT_IS_TOP_LEVEL})

set(GAITCORE_C_SOURCES
	src/gc_vmath.c
	src/gc_thread.c
	src/gc_world.c
	src/gc_ik.c
	src/gc_gaits.c
	src/gc_mechdefs.c
	src/gc_leg.c
	src/gc_gaitsel.c
	src/gc_foothold.c
	src/gc_clearance.c
	src/gc_scheduler.c
	src/gc_mech.c
	src/gc_api.c
)

add_library(gaitcore_c STATIC ${GAITCORE_C_SOURCES})
add_library(gaitcore::c ALIAS gaitcore_c)

target_include_directories(gaitcore_c
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:include>
	PRIVATE
		"${CMAKE_CURRENT_SOURCE_DIR}/src"
)

set_target_properties(gaitcore_c PROPERTIES
	C_STANDARD 11
	C_STANDARD_REQUIRED ON
	C_EXTENSIONS OFF
	POSITION_INDEPENDENT_CODE ON
	FOLDER "IK/c"
)

if(MSVC)
	target_compile_definitions(gaitcore_c PRIVATE _CRT_SECURE_NO_WARNINGS)
	target_compile_options(gaitcore_c PRIVATE /W3 /utf-8)
else()
	find_package(Threads REQUIRED)
	target_link_libraries(gaitcore_c PUBLIC Threads::Threads)
	if(UNIX)
		target_link_libraries(gaitcore_c PUBLIC m)
	endif()
	target_compile_options(gaitcore_c PRIVATE -Wall -Wextra -Wno-unused-parameter)
endif()

if(GAITCORE_BUILD_TESTS)
	if(PROJECT_IS_TOP_LEVEL)
		enable_testing()
	endif()

	add_executable(gaitcore_c_tests tests/test_gaitcore_c.c)
	target_link_libraries(gaitcore_c_tests PRIVATE gaitcore::c)
	set_target_properties(gaitcore_c_tests PROPERTIES
		C_STANDARD 11
		C_STANDARD_REQUIRED ON
		C_EXTENSIONS OFF
		FOLDER "IK/c"
	)

	if(MSVC)
		target_compile_definitions(gaitcore_c_tests PRIVATE _CRT_SECURE_NO_WARNINGS)
		target_compile_options(gaitcore_c_tests PRIVATE /W3 /utf-8)
	else()
		target_compile_options(gaitcore_c_tests PRIVATE -Wall -Wextra -Wno-unused-parameter)
	endif()

	add_test(NAME gaitcore_c_tests COMMAND gaitcore_c_tests)
endif()
