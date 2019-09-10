set(ASSIGNMENT1_SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR})

file(GLOB_RECURSE ASSIGNMENT_HEADER ${CMAKE_CURRENT_LIST_DIR}/*.h ${CMAKE_CURRENT_LIST_DIR}/*.hpp)
file(GLOB_RECURSE ASSIGNMENT_SOURCE ${CMAKE_CURRENT_LIST_DIR}/*.cpp)

add_executable(run1 ${ASSIGNMENT_HEADER} ${ASSIGNMENT_SOURCE} ${COMMON_ROOT}/Geometry/BasicGeometry.hpp ${COMMON_ROOT}/Mesh/tri_mesh.hpp)

# Includes.
target_include_directories(run1 PRIVATE ${EXTERNAL_HEADER})
target_include_directories(run1 PRIVATE ${COMMON_HEADER})
target_include_directories(run1 PRIVATE ${CMAKE_CURRENT_LIST_DIR}/)
target_include_directories(run1 PRIVATE ${CMAKE_CURRENT_LIST_DIR}/Mesh)
