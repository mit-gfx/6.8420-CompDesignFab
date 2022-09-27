set(ASSIGNMENT5B_SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR})

file(GLOB_RECURSE ASSIGNMENT_HEADER ${CMAKE_CURRENT_LIST_DIR}/part2/*.h ${CMAKE_CURRENT_LIST_DIR}/part2/*.hpp)
file(GLOB_RECURSE ASSIGNMENT_SOURCE ${CMAKE_CURRENT_LIST_DIR}/part2/*.cpp)

add_executable(run5 ${ASSIGNMENT_HEADER} ${ASSIGNMENT_SOURCE})

# Includes.
target_include_directories(run5 PRIVATE ${EXTERNAL_HEADER})
target_include_directories(run5 PRIVATE ${COMMON_HEADER})
target_include_directories(run5 PRIVATE ${CMAKE_CURRENT_LIST_DIR}/)
target_include_directories(run5 PRIVATE ${CMAKE_CURRENT_LIST_DIR}/../assignment1)
