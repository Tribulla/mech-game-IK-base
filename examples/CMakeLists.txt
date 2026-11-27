add_executable(custom_mech_example custom_mech_example.c)
target_link_libraries(custom_mech_example PRIVATE gaitcore::c)

set_target_properties(custom_mech_example PROPERTIES
	C_STANDARD 11
	C_STANDARD_REQUIRED ON
	C_EXTENSIONS OFF
	FOLDER "examples"
)

if(MSVC)
	target_compile_definitions(custom_mech_example PRIVATE _CRT_SECURE_NO_WARNINGS)
	target_compile_options(custom_mech_example PRIVATE /W3 /utf-8)
else()
	target_compile_options(custom_mech_example PRIVATE -Wall -Wextra -Wno-unused-parameter)
endif()

if(TARGET gaitcore_java)
	include(UseJava)
	set(CMAKE_JAVA_COMPILE_FLAGS --release 17 -Xlint:all -Werror)
	add_jar(custom_mech_example_java
		SOURCES CustomMechExample.java
		INCLUDE_JARS gaitcore_java
		ENTRY_POINT CustomMechExample
		OUTPUT_NAME custom_mech_example_java
	)
endif()
