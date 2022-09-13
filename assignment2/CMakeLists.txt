set(ASSIGNMENT2_SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR})

# Adds library target imgui
add_subdirectory(external/libigl-imgui)
# Adds library target glfw
add_subdirectory(external/glfw)
# Adds library target glad
add_subdirectory(external/glad)
# Adds Library target tetgen
add_subdirectory(external/tetgen)
# Adds target OpenGL::GL
find_package(OpenGL REQUIRED)

file(GLOB ASSIGNMENT_SOURCE src/*.cpp src/*.hpp src/*.h src/*.natvis)

add_executable(run2 ${ASSIGNMENT_SOURCE})

target_link_libraries(run2 PRIVATE OpenGL::GL)
target_link_libraries(run2 PRIVATE glad)
target_link_libraries(run2 PRIVATE glfw)
target_link_libraries(run2 PRIVATE imgui)
target_link_libraries(run2 PRIVATE tetgen)

target_include_directories(run2 PRIVATE external)
target_include_directories(run2 PRIVATE external/glad/include)
target_include_directories(run2 PRIVATE external/glfw/include)
target_include_directories(run2 PRIVATE external/tetgen)
target_include_directories(run2 PRIVATE ${EXTERNAL_HEADER})
target_include_directories(run2 PRIVATE ${COMMON_HEADER})

if (WIN32)
	add_definitions(/bigobj)
endif()