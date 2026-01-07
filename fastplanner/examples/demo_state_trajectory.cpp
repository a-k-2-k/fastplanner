/**
 * @file demo_state_trajectory.cpp
 * @brief Simple demonstration of State and Trajectory classes
 * 
 * This example shows:
 * - Creating and manipulating states
 * - Building trajectories
 * - Computing trajectory metrics (length, curvature, cost)
 * - Interpolation and sampling
 */

#include "fastplanner/core/state.hpp"
#include <iostream>
#include <iomanip>

using namespace fastplanner;

void printState(const State& state, const std::string& name) {
    std::cout << name << ": "
              << "pos=(" << state.position.x() << ", " << state.position.y() << "), "
              << "heading=" << state.heading << " rad, "
              << "vel=" << state.velocity << " m/s, "
              << "time=" << state.time << " s\n";
}

void printTrajectoryMetrics(const Trajectory& traj) {
    std::cout << "\nTrajectory Metrics:\n";
    std::cout << "  States: " << traj.size() << "\n";
    std::cout << "  Length: " << std::fixed << std::setprecision(3) 
              << traj.getLength() << " m\n";
    std::cout << "  Duration: " << traj.getDuration() << " s\n";
    std::cout << "  Curvature: " << traj.getCurvature() << "\n";
    std::cout << "  Cost: " << traj.getCost(1.0) << "\n";
}

int main() {
    std::cout << "=== FastPlanner State & Trajectory Demo ===\n\n";
    
    // ========================================================================
    // Part 1: Creating and manipulating states
    // ========================================================================
    
    std::cout << "1. Creating States\n";
    std::cout << "------------------\n";
    
    State start(0.0, 0.0, 0.0, 1.0, 0.0);
    State goal(10.0, 5.0, M_PI/4, 1.0, 10.0);
    
    printState(start, "Start");
    printState(goal, "Goal");
    
    double distance = start.distanceTo(goal);
    std::cout << "\nEuclidean distance: " << distance << " m\n";
    
    double config_dist = start.configurationDistanceTo(goal, 0.5);
    std::cout << "Configuration space distance: " << config_dist << "\n";
    
    // ========================================================================
    // Part 2: State interpolation
    // ========================================================================
    
    std::cout << "\n2. State Interpolation\n";
    std::cout << "----------------------\n";
    
    for (double t = 0.0; t <= 1.0; t += 0.25) {
        State interp = start.interpolate(goal, t);
        std::cout << "t=" << t << ": ";
        printState(interp, "");
    }
    
    // ========================================================================
    // Part 3: Building a trajectory
    // ========================================================================
    
    std::cout << "\n3. Building a Trajectory\n";
    std::cout << "------------------------\n";
    
    Trajectory trajectory;
    
    // Create a curved path (L-shape)
    trajectory.addState(State(0, 0, 0, 1.0, 0.0));
    trajectory.addState(State(2, 0, 0, 1.0, 2.0));
    trajectory.addState(State(4, 0, 0, 1.0, 4.0));
    trajectory.addState(State(6, 0, M_PI/6, 1.0, 6.0));
    trajectory.addState(State(7, 1, M_PI/3, 1.0, 8.0));
    trajectory.addState(State(8, 2, M_PI/2, 1.0, 10.0));
    trajectory.addState(State(8, 4, M_PI/2, 1.0, 12.0));
    trajectory.addState(State(8, 6, M_PI/2, 0.5, 14.0));
    
    printTrajectoryMetrics(trajectory);
    
    // ========================================================================
    // Part 4: Trajectory operations
    // ========================================================================
    
    std::cout << "\n4. Trajectory Operations\n";
    std::cout << "------------------------\n";
    
    // Downsample
    Trajectory downsampled = trajectory.downsample(2);
    std::cout << "\nDownsampled (factor=2):\n";
    printTrajectoryMetrics(downsampled);
    
    // Reverse
    Trajectory reversed = trajectory.reverse();
    std::cout << "\nReversed:\n";
    std::cout << "  First state: ";
    printState(reversed[0], "");
    std::cout << "  Last state: ";
    printState(reversed[reversed.size() - 1], "");
    
    // Interpolate to fixed time step
    Trajectory interpolated = trajectory.interpolateToTimeStep(1.0);
    std::cout << "\nInterpolated (dt=1.0s):\n";
    printTrajectoryMetrics(interpolated);
    
    // ========================================================================
    // Part 5: Bounds and sampling
    // ========================================================================
    
    std::cout << "\n5. Bounds and Random Sampling\n";
    std::cout << "-----------------------------\n";
    
    Bounds bounds(Eigen::Vector2d(-10, -10), Eigen::Vector2d(10, 10),
                  -M_PI, M_PI, 0.0, 3.0);
    
    std::cout << "Configuration space bounds:\n";
    std::cout << "  Position: [" << bounds.position_min.transpose() 
              << "] to [" << bounds.position_max.transpose() << "]\n";
    std::cout << "  Heading: [" << bounds.heading_min << ", " << bounds.heading_max << "] rad\n";
    std::cout << "  Velocity: [" << bounds.velocity_min << ", " << bounds.velocity_max << "] m/s\n";
    
    std::cout << "\nRandom samples:\n";
    for (int i = 0; i < 5; ++i) {
        Eigen::Vector2d pos = bounds.samplePosition();
        double heading = bounds.sampleHeading();
        double velocity = bounds.sampleVelocity();
        
        std::cout << "  Sample " << i+1 << ": "
                  << "pos=(" << pos.transpose() << "), "
                  << "heading=" << std::fixed << std::setprecision(2) << heading << ", "
                  << "vel=" << velocity << "\n";
    }
    
    // ========================================================================
    // Part 6: Comparing trajectories
    // ========================================================================
    
    std::cout << "\n6. Comparing Different Paths\n";
    std::cout << "----------------------------\n";
    
    // Straight path
    Trajectory straight;
    for (double x = 0; x <= 10; x += 1.0) {
        straight.addState(State(x, 0, 0, 1.0, x));
    }
    
    // Curved path
    Trajectory curved;
    for (double t = 0; t <= 10; t += 1.0) {
        double x = t;
        double y = std::sin(t * M_PI / 10) * 2;
        double heading = std::cos(t * M_PI / 10) * M_PI / 5;
        curved.addState(State(x, y, heading, 1.0, t));
    }
    
    std::cout << "\nStraight path:\n";
    printTrajectoryMetrics(straight);
    
    std::cout << "\nCurved path:\n";
    printTrajectoryMetrics(curved);
    
    std::cout << "\nComparison:\n";
    std::cout << "  Straight path is " 
              << (straight.getLength() < curved.getLength() ? "shorter" : "longer")
              << " by " << std::abs(curved.getLength() - straight.getLength()) << " m\n";
    std::cout << "  Straight path is " 
              << (straight.getCurvature() < curved.getCurvature() ? "smoother" : "less smooth")
              << " (curvature difference: " 
              << std::abs(curved.getCurvature() - straight.getCurvature()) << ")\n";
    
    std::cout << "\n=== Demo Complete ===\n";
    
    return 0;
}

