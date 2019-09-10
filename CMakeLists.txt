cmake_minimum_required(VERSION 3.5)

set(CMAKE_SUPPRESS_REGENERATION true)

project(CompFab)

# Enable C++ 11
set(CMAKE_CXX_STANDARD 11)

# Suppress warnings.
add_definitions(
  -D_CRT_SECURE_NO_WARNINGS
)
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
if(UNIX)
  set(GCC_COVERAGE_COMPILE_FLAGS "-Wno-format")
  set(GCC_COVERAGE_COMPILE_FLAGS "-Wno-unused-result")
  add_definitions(${GCC_COVERAGE_COMPILE_FLAGS})
endif()

# Expose PROJECT_SOURCE_DIR to the source code.
add_definitions(-DPROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

# Externals
set(EXTERNAL_ROOT ${PROJECT_SOURCE_DIR}/Externals)
add_subdirectory(${EXTERNAL_ROOT})
get_directory_property(EXTERNAL_HEADER
  DIRECTORY ${EXTERNAL_ROOT}
  DEFINITION EXTERNAL_HEADER)

# Common
set(COMMON_ROOT ${PROJECT_SOURCE_DIR}/Common)
add_subdirectory(${COMMON_ROOT})
get_directory_property(COMMON_HEADER
  DIRECTORY ${COMMON_ROOT}
  DEFINITION COMMON_HEADER)

# Include each assignment
if (NOT ${HW} OR ${HW} EQUAL "all")
    add_subdirectory(assignment1)
elseif (${HW} EQUAL "1")
    add_subdirectory(assignment1)
endif()
