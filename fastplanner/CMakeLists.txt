cmake_minimum_required(VERSION 3.15)
project(FastPlanner VERSION 1.0.0 LANGUAGES CXX)

# Set C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler flags for optimization and warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")

# Set default build type to Release if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# ============================================================================
# Dependencies
# ============================================================================

# Eigen3 for linear algebra (required)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
message(STATUS "Found Eigen3: ${EIGEN3_INCLUDE_DIR}")

# Google Test (optional, for unit tests)
find_package(GTest QUIET)
if(GTest_FOUND)
    message(STATUS "Found GTest: ${GTEST_INCLUDE_DIRS}")
    enable_testing()
else()
    message(STATUS "GTest not found - tests will not be built")
endif()

# Google Benchmark (optional, for performance benchmarks)
find_package(benchmark QUIET)
if(benchmark_FOUND)
    message(STATUS "Found Google Benchmark")
else()
    message(STATUS "Google Benchmark not found - benchmarks will not be built")
endif()

# ============================================================================
# Main Library
# ============================================================================

# Collect source files
set(FASTPLANNER_SOURCES
    src/core/state.cpp
    src/core/collision_checker.cpp
)

# Collect header files
set(FASTPLANNER_HEADERS
    include/fastplanner/core/state.hpp
    include/fastplanner/core/collision_checker.hpp
)

# Create shared library
add_library(fastplanner SHARED ${FASTPLANNER_SOURCES})

# Set library properties
set_target_properties(fastplanner PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 1
    PUBLIC_HEADER "${FASTPLANNER_HEADERS}"
)

# Include directories
target_include_directories(fastplanner
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Link dependencies
target_link_libraries(fastplanner
    PUBLIC
        Eigen3::Eigen
)

# ============================================================================
# Tests
# ============================================================================

if(GTest_FOUND)
    add_subdirectory(tests)
endif()

# ============================================================================
# Benchmarks
# ============================================================================

if(benchmark_FOUND)
    add_subdirectory(benchmarks)
endif()

# ============================================================================
# Examples
# ============================================================================

# Build examples (optional)
option(BUILD_EXAMPLES "Build example programs" ON)
if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# ============================================================================
# WebAssembly Build
# ============================================================================

# Check if building with Emscripten
if(EMSCRIPTEN)
    message(STATUS "Emscripten build detected - configuring WebAssembly target")
    
    add_executable(fastplanner_web
        web/bindings.cpp
        ${FASTPLANNER_SOURCES}
    )
    
    target_include_directories(fastplanner_web PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    
    target_link_libraries(fastplanner_web PRIVATE
        Eigen3::Eigen
    )
    
    # Emscripten-specific flags
    set_target_properties(fastplanner_web PROPERTIES
        LINK_FLAGS "-s WASM=1 -s ALLOW_MEMORY_GROWTH=1 -s MODULARIZE=1 -s EXPORT_NAME='FastPlanner' --bind"
    )
endif()

# ============================================================================
# Installation
# ============================================================================

# Install library
install(TARGETS fastplanner
    EXPORT FastPlannerTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    PUBLIC_HEADER DESTINATION include/fastplanner/core
)

# Install headers
install(DIRECTORY include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.hpp"
)

# Export targets
install(EXPORT FastPlannerTargets
    FILE FastPlannerTargets.cmake
    NAMESPACE FastPlanner::
    DESTINATION lib/cmake/FastPlanner
)

# Create config file
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/FastPlannerConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/FastPlannerConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/FastPlannerConfig.cmake"
    INSTALL_DESTINATION lib/cmake/FastPlanner
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/FastPlannerConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/FastPlannerConfigVersion.cmake"
    DESTINATION lib/cmake/FastPlanner
)

# ============================================================================
# Summary
# ============================================================================

message(STATUS "")
message(STATUS "FastPlanner Configuration Summary")
message(STATUS "==================================")
message(STATUS "Version: ${PROJECT_VERSION}")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ standard: C++${CMAKE_CXX_STANDARD}")
message(STATUS "Compiler: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "Install prefix: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "")
message(STATUS "Components:")
message(STATUS "  - Core library: YES")
message(STATUS "  - Unit tests: ${GTest_FOUND}")
message(STATUS "  - Benchmarks: ${benchmark_FOUND}")
message(STATUS "  - Examples: ${BUILD_EXAMPLES}")
message(STATUS "  - WebAssembly: ${EMSCRIPTEN}")
message(STATUS "")
