/**
 * @file demo_state.cpp
 * @brief Demonstration of State, Bounds, and Trajectory usage
 * 
 * This example shows how to:
 * - Create and manipulate states
 * - Define configuration space bounds
 * - Build and analyze trajectories
 * - Use interpolation and resampling
 */

#include "fastplanner/core/state.hpp"
#include <iostream>
#include <iomanip>

using namespace fastplanner;

void printState(const State& s, const std::string& label) {
    std::cout << label << ": "
              << "pos=(" << s.position.x() << ", " << s.position.y() << "), "
              << "theta=" << s.theta << ", "
              << "v=" << s.velocity << ", "
              << "t=" << s.time << std::endl;
}

void printSeparator(const std::string& title = "") {
    std::cout << "\n" << std::string(60, '=') << std::endl;
    if (!title.empty()) {
        std::cout << title << std::endl;
        std::cout << std::string(60, '=') << std::endl;
    }
}

int main() {
    std::cout << std::fixed << std::setprecision(3);
    
    printSeparator("FastPlanner State Demo");
    
    // =============================================================================
    // Part 1: State Creation and Manipulation
    // =============================================================================
    printSeparator("Part 1: State Creation");
    
    State s1(0.0, 0.0);
    State s2(3.0, 4.0, M_PI/4);
    State s3(6.0, 8.0, M_PI/2, 5.0, 2.0);
    
    printState(s1, "State 1 (minimal)");
    printState(s2, "State 2 (with angle)");
    printState(s3, "State 3 (complete)");
    
    // Distance calculations
    std::cout << "\nDistance from s1 to s2: " << s1.distanceTo(s2) << " meters" << std::endl;
    std::cout << "Squared distance (faster): " << s1.distanceSquaredTo(s2) << " m^2" << std::endl;
    
    // Angle calculations
    std::cout << "\nAngle difference s1 to s2: " << s1.angleDifferenceTo(s2) << " rad" << std::endl;
    
    // Angle normalization
    double angle = 7.0;
    std::cout << "Normalize " << angle << " rad: " 
              << State::normalizeAngle(angle) << " rad" << std::endl;
    
    // =============================================================================
    // Part 2: Configuration Space Bounds
    // =============================================================================
    printSeparator("Part 2: Configuration Space Bounds");
    
    Bounds bounds(0.0, 50.0, 0.0, 50.0);
    std::cout << "Bounds: [" << bounds.x_min << ", " << bounds.x_max << "] x ["
              << bounds.y_min << ", " << bounds.y_max << "]" << std::endl;
    
    State center = bounds.getCenter();
    printState(center, "Center of bounds");
    
    std::cout << "Diagonal length: " << bounds.getDiagonal() << " meters" << std::endl;
    
    // Sampling
    std::cout << "\nRandom samples from bounds:" << std::endl;
    std::mt19937 rng(42);
    for (int i = 0; i < 5; ++i) {
        State sample = bounds.sample(rng);
        printState(sample, "  Sample " + std::to_string(i+1));
    }
    
    // Containment checking
    State inside(25.0, 25.0);
    State outside(100.0, 100.0);
    std::cout << "\nState at (25, 25) in bounds: " << (bounds.contains(inside) ? "YES" : "NO") << std::endl;
    std::cout << "State at (100, 100) in bounds: " << (bounds.contains(outside) ? "YES" : "NO") << std::endl;
    
    // =============================================================================
    // Part 3: Trajectory Building
    // =============================================================================
    printSeparator("Part 3: Trajectory Building");
    
    Trajectory traj;
    std::cout << "Creating trajectory with 5 waypoints..." << std::endl;
    
    traj.addState(State(0.0, 0.0, 0.0, 1.0, 0.0));
    traj.addState(State(5.0, 0.0, 0.0, 1.0, 5.0));
    traj.addState(State(10.0, 0.0, M_PI/4, 1.0, 10.0));
    traj.addState(State(15.0, 5.0, M_PI/2, 1.0, 15.0));
    traj.addState(State(15.0, 10.0, M_PI/2, 1.0, 20.0));
    
    std::cout << "Trajectory size: " << traj.size() << " states" << std::endl;
    std::cout << "Total length: " << traj.getLength() << " meters" << std::endl;
    std::cout << "Total cost: " << traj.getCost() << std::endl;
    
    std::cout << "\nTrajectory waypoints:" << std::endl;
    for (size_t i = 0; i < traj.size(); ++i) {
        printState(traj[i], "  State " + std::to_string(i));
    }
    
    // =============================================================================
    // Part 4: Trajectory Interpolation
    // =============================================================================
    printSeparator("Part 4: Trajectory Interpolation");
    
    std::cout << "Interpolating at various distances along path:" << std::endl;
    
    for (double dist = 0.0; dist <= traj.getLength(); dist += traj.getLength() / 5.0) {
        State interpolated = traj.interpolate(dist);
        printState(interpolated, "  At distance " + std::to_string(dist) + "m");
    }
    
    // =============================================================================
    // Part 5: Trajectory Analysis
    // =============================================================================
    printSeparator("Part 5: Trajectory Analysis");
    
    double max_curvature = traj.getMaxCurvature();
    std::cout << "Maximum curvature: " << max_curvature << " rad/m" << std::endl;
    
    if (max_curvature > 0.0) {
        double min_radius = 1.0 / max_curvature;
        std::cout << "Minimum turn radius: " << min_radius << " meters" << std::endl;
    }
    
    // =============================================================================
    // Part 6: Trajectory Resampling
    // =============================================================================
    printSeparator("Part 6: Trajectory Resampling");
    
    double spacing = 2.0;
    std::cout << "Resampling trajectory with " << spacing << "m spacing..." << std::endl;
    
    Trajectory resampled = traj.resample(spacing);
    std::cout << "Original: " << traj.size() << " states" << std::endl;
    std::cout << "Resampled: " << resampled.size() << " states" << std::endl;
    std::cout << "Resampled length: " << resampled.getLength() << " meters" << std::endl;
    
    std::cout << "\nResampled waypoints:" << std::endl;
    for (size_t i = 0; i < resampled.size(); ++i) {
        printState(resampled[i], "  State " + std::to_string(i));
    }
    
    // =============================================================================
    // Part 7: Trajectory Manipulation
    // =============================================================================
    printSeparator("Part 7: Trajectory Manipulation");
    
    Trajectory reversed = traj;
    reversed.reverse();
    
    std::cout << "Original first state: ";
    printState(traj[0], "");
    std::cout << "Reversed first state: ";
    printState(reversed[0], "");
    
    std::cout << "\nOriginal last state: ";
    printState(traj[traj.size()-1], "");
    std::cout << "Reversed last state: ";
    printState(reversed[reversed.size()-1], "");
    
    // =============================================================================
    // Part 8: State Comparisons
    // =============================================================================
    printSeparator("Part 8: State Comparisons");
    
    State exact1(1.0, 2.0, 0.5);
    State exact2(1.0, 2.0, 0.5);
    State approx(1.0001, 2.0001, 0.501);
    State different(5.0, 6.0, 1.0);
    
    std::cout << "exact1 == exact2: " << (exact1 == exact2 ? "TRUE" : "FALSE") << std::endl;
    std::cout << "exact1 == approx: " << (exact1 == approx ? "TRUE" : "FALSE") << std::endl;
    std::cout << "exact1.isApprox(approx, 0.001, 0.01): " 
              << (exact1.isApprox(approx, 0.001, 0.01) ? "TRUE" : "FALSE") << std::endl;
    std::cout << "exact1.isApprox(different, 0.001, 0.01): " 
              << (exact1.isApprox(different, 0.001, 0.01) ? "TRUE" : "FALSE") << std::endl;
    
    printSeparator();
    std::cout << "\n✓ State demo complete!\n" << std::endl;
    
    return 0;
}

