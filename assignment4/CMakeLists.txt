set(ASSIGNMENT4_SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR})

# Adds target OpenGL::GL
find_package(OpenGL REQUIRED)

file(GLOB_RECURSE ASSIGNMENT_HEADER ${CMAKE_CURRENT_LIST_DIR}/*.h ${CMAKE_CURRENT_LIST_DIR}/*.hpp)
file(GLOB_RECURSE ASSIGNMENT_SOURCE ${CMAKE_CURRENT_LIST_DIR}/*.cpp)

add_executable(run4 ${ASSIGNMENT_HEADER} ${ASSIGNMENT_SOURCE})

target_link_libraries(run4 PRIVATE OpenGL::GL)
target_link_libraries(run4 PRIVATE glad)
target_link_libraries(run4 PRIVATE glfw)
target_link_libraries(run4 PRIVATE imgui)
target_link_libraries(run4 PRIVATE tetgen)

# Includes.
target_include_directories(run4 PRIVATE ${CMAKE_CURRENT_LIST_DIR}/../assignment2/external)
target_include_directories(run4 PRIVATE ${CMAKE_CURRENT_LIST_DIR}/../assignment2/external/glad/include)
target_include_directories(run4 PRIVATE ${CMAKE_CURRENT_LIST_DIR}/../assignment2/external/glfw/include)
target_include_directories(run4 PRIVATE ${CMAKE_CURRENT_LIST_DIR}/../assignment2/external/tetgen)
target_include_directories(run4 PRIVATE ${EXTERNAL_HEADER})
target_include_directories(run4 PRIVATE ${COMMON_HEADER})
target_include_directories(run4 PRIVATE ${CMAKE_CURRENT_LIST_DIR}/)
