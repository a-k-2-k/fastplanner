# Google Benchmark configuration

# Benchmark executable
add_executable(fastplanner_benchmarks
    benchmark_collision_checking.cpp
)

# Link against library and benchmark
target_link_libraries(fastplanner_benchmarks
    PRIVATE
        fastplanner
        benchmark::benchmark
        benchmark::benchmark_main
)

# Include directories
target_include_directories(fastplanner_benchmarks PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

message(STATUS "Configured benchmarks")

