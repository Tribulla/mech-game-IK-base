if(NOT DEFINED APP_TITLE)
	set(APP_TITLE "Mech IK Core Demo" CACHE STRING "Window title shown by the demo app")
endif()

option(APP_BUILD_SHADERS "Recompile shaders with sokol-shdc" OFF)

add_subdirectory(framework)

target_link_libraries(mech_ik_demo PRIVATE gaitcore::c)

if(GAITCORE_ENABLE_PYTHON)
	find_package(Python3 3.10 COMPONENTS Interpreter Development.Embed)
	if(Python3_Development.Embed_FOUND)
		execute_process(
			COMMAND "${Python3_EXECUTABLE}" -c "import sys; print(sys.base_prefix)"
			OUTPUT_VARIABLE MECH_PY_HOME OUTPUT_STRIP_TRAILING_WHITESPACE)
		file(TO_CMAKE_PATH "${MECH_PY_HOME}" MECH_PY_HOME)
		target_link_libraries(mech_ik_demo PRIVATE Python3::Python)
		target_compile_definitions(mech_ik_demo PRIVATE
			MECH_HAVE_PYTHON=1
			MECH_IK_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../IK/python"
			MECH_PY_HOME="${MECH_PY_HOME}")
		if(WIN32)
			set(_pydll "${MECH_PY_HOME}/python${Python3_VERSION_MAJOR}${Python3_VERSION_MINOR}.dll")
			add_custom_target(mech_python_dlls
				COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:mech_ik_demo>"
				COMMAND ${CMAKE_COMMAND} -E copy_if_different "${_pydll}" "$<TARGET_FILE_DIR:mech_ik_demo>"
				COMMAND ${CMAKE_COMMAND} -E copy_if_different "${MECH_PY_HOME}/python3.dll" "$<TARGET_FILE_DIR:mech_ik_demo>"
				COMMENT "Copying the Python runtime next to mech_ik_demo.exe"
				VERBATIM)
			set_target_properties(mech_python_dlls PROPERTIES FOLDER "demo")
			add_dependencies(mech_ik_demo mech_python_dlls)
		endif()
		message(STATUS "Demo app: C (multithreaded) + Python ${Python3_VERSION} embedded from ${MECH_PY_HOME}")
	else()
		message(STATUS "Demo app: Python 3.10+ development files not found; using native multithreaded C gait core only.")
	endif()
else()
	message(STATUS "Demo app: Python embedding disabled; using native multithreaded C gait core.")
endif()

if(MSVC AND PROJECT_IS_TOP_LEVEL)
	set_property(TARGET mech_ik_demo PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")
endif()
