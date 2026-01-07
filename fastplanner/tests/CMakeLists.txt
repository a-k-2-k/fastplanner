# Google Test configuration
include(GoogleTest)

# Common test settings
set(TEST_SOURCES
    test_state.cpp
    test_collision_checker.cpp
)

# Create test executable
add_executable(fastplanner_tests ${TEST_SOURCES})

# Link against library and GTest
target_link_libraries(fastplanner_tests
    PRIVATE
        fastplanner
        GTest::GTest
        GTest::Main
)

# Include directories
target_include_directories(fastplanner_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

# Discover tests
gtest_discover_tests(fastplanner_tests)

# Optional: Add custom test target
add_custom_target(check
    COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
    DEPENDS fastplanner_tests
    COMMENT "Running unit tests"
)
