cmake_minimum_required(VERSION 3.16)

project(
  PathPlanning
  VERSION 1.0.0
  DESCRIPTION "Interactive Path Planning with OpenGL"
  LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set default build type to Release if not specified
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Compiler-specific options
if(MSVC)
  add_compile_options(/W4)
else()
  add_compile_options(-Wall -Wextra -Wpedantic)
  if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_options(-g -O0)
  else()
    add_compile_options(-O3)
  endif()
endif()

# Find required packages
find_package(PkgConfig REQUIRED)

# Find GLFW
pkg_check_modules(GLFW REQUIRED glfw3)

# Find OpenGL
find_package(OpenGL REQUIRED)

# Find GLM
find_package(glm REQUIRED)

# Create viewer library
set(VIEWER_SOURCES
    src/viewer/Camera.cpp
    src/viewer/Grid.cpp
    src/viewer/InputHandler.cpp
    src/viewer/LineRenderer.cpp
    src/viewer/MathUtils.cpp
    src/viewer/PathRenderer.cpp
    src/viewer/QuadRenderer.cpp
    src/viewer/ShaderUtils.cpp
    src/map/CostmapLayer.cpp
    src/map/Map.cpp
    src/map/MapLoader.cpp
    src/path/AStarPlanner.cpp
    src/path/DStarLitePlanner.cpp
    src/path/PlannerUtils.cpp)

set(VIEWER_HEADERS
    include/viewer/Camera.h
    include/viewer/Constants.h
    include/viewer/Grid.h
    include/viewer/InputHandler.h
    include/viewer/LineRenderer.h
    include/viewer/MathUtils.h
    include/viewer/PathRenderer.h
    include/viewer/QuadRenderer.h
    include/viewer/ShaderUtils.h
    include/map/CostmapLayer.h
    include/map/Map.h
    include/map/MapLoader.h
    include/path/AStarPlanner.h
    include/path/DStarLitePlanner.h
    include/path/IPathPlanner.h
    include/path/PathPlannerBase.h
    include/path/PlannerTypes.h
    include/path/PlannerUtils.h)

# Create static library for core functionality
add_library(PathPlanningCore STATIC ${VIEWER_SOURCES} ${VIEWER_HEADERS})

target_include_directories(
  PathPlanningCore PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                          $<INSTALL_INTERFACE:include> ${GLFW_INCLUDE_DIRS})

target_link_libraries(
  PathPlanningCore PUBLIC third_party::glad ${GLFW_LIBRARIES} OpenGL::GL
                          ${CMAKE_DL_LIBS} glm::glm)

target_compile_definitions(PathPlanningCore PUBLIC ${GLFW_CFLAGS_OTHER})

# Add third-party dependencies
add_subdirectory(third_party)

# Create main executable
add_executable(path_planning src/main.cpp)

target_link_libraries(path_planning PRIVATE PathPlanningCore third_party::glad)

# Installation
install(TARGETS path_planning RUNTIME DESTINATION bin)

# Optional: Create a CMake package for the library
install(
  TARGETS PathPlanningCore glad
  EXPORT PathPlanningTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(DIRECTORY include/ DESTINATION include)

install(
  EXPORT PathPlanningTargets
  FILE PathPlanningTargets.cmake
  DESTINATION lib/cmake/PathPlanning)

# Package configuration
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/PathPlanningConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion)

configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/Config.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/PathPlanningConfig.cmake"
  INSTALL_DESTINATION lib/cmake/PathPlanning)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/PathPlanningConfig.cmake"
              "${CMAKE_CURRENT_BINARY_DIR}/PathPlanningConfigVersion.cmake"
        DESTINATION lib/cmake/PathPlanning)

# Print build information
message(STATUS "PathPlanning ${PROJECT_VERSION}")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ standard: ${CMAKE_CXX_STANDARD}")
message(
  STATUS "Compiler: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
