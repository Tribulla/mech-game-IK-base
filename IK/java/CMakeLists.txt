cmake_minimum_required(VERSION 3.20)
project(gaitcore_java VERSION 1.0.0 LANGUAGES NONE)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
	enable_testing()
endif()

if(NOT DEFINED GAITCORE_BUILD_TESTS)
	option(GAITCORE_BUILD_TESTS "Build Java unit and determinism tests" ON)
endif()

find_package(Java 17 REQUIRED COMPONENTS Development Runtime)
include(UseJava)

set(CMAKE_JAVA_COMPILE_FLAGS --release 17 -Xlint:all -Werror)

set(GAITCORE_JAVA_SOURCES
	src/main/java/gaitcore/Clearance.java
	src/main/java/gaitcore/Foothold.java
	src/main/java/gaitcore/Gait.java
	src/main/java/gaitcore/GaitCore.java
	src/main/java/gaitcore/GaitSelector.java
	src/main/java/gaitcore/GcBenchConfig.java
	src/main/java/gaitcore/GcContext.java
	src/main/java/gaitcore/GcMechDesc.java
	src/main/java/gaitcore/GcMechInfo.java
	src/main/java/gaitcore/GcMechStats.java
	src/main/java/gaitcore/GcQueryStats.java
	src/main/java/gaitcore/GcStubWorld.java
	src/main/java/gaitcore/GcWorldCallbacks.java
	src/main/java/gaitcore/Leg.java
	src/main/java/gaitcore/LegIK.java
	src/main/java/gaitcore/Mech.java
	src/main/java/gaitcore/MechDef.java
	src/main/java/gaitcore/Scheduler.java
	src/main/java/gaitcore/ThreadPool.java
	src/main/java/gaitcore/Vec3.java
	src/main/java/gaitcore/WorldQueryCtx.java
)

add_jar(gaitcore_java
	SOURCES ${GAITCORE_JAVA_SOURCES}
	OUTPUT_NAME gaitcore
)

if(GAITCORE_BUILD_TESTS)
	add_jar(gaitcore_java_tests
		SOURCES src/test/java/gaitcore/GaitCoreTest.java
		INCLUDE_JARS gaitcore_java
		ENTRY_POINT gaitcore.GaitCoreTest
		OUTPUT_NAME gaitcore_java_tests
	)

	add_test(NAME gaitcore_java_tests
		COMMAND "${Java_JAVA_EXECUTABLE}"
			-cp "$<TARGET_PROPERTY:gaitcore_java_tests,JAR_FILE>$<IF:$<BOOL:${WIN32}>,$<SEMICOLON>,:>$<TARGET_PROPERTY:gaitcore_java,JAR_FILE>"
			gaitcore.GaitCoreTest
	)
endif()
