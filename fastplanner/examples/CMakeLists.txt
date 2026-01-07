# Example programs demonstrating FastPlanner usage

# Simple state and trajectory example
add_executable(demo_state_trajectory
    demo_state_trajectory.cpp
)

target_link_libraries(demo_state_trajectory
    PRIVATE fastplanner
)

target_include_directories(demo_state_trajectory PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

# Collision checking example
add_executable(demo_collision_checking
    demo_collision_checking.cpp
)

target_link_libraries(demo_collision_checking
    PRIVATE fastplanner
)

target_include_directories(demo_collision_checking PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

message(STATUS "Configured example programs")
