#pragma once

#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <cmath>

namespace fastplanner {

/**
 * @brief Configuration space bounds for state sampling and validation
 * 
 * Defines the valid ranges for state variables. Used for random sampling
 * and checking state validity during planning.
 */
struct Bounds {
    Eigen::Vector2d position_min;  ///< Minimum (x, y) position
    Eigen::Vector2d position_max;  ///< Maximum (x, y) position
    double heading_min;            ///< Minimum heading angle (radians)
    double heading_max;            ///< Maximum heading angle (radians)
    double velocity_min;           ///< Minimum velocity (m/s)
    double velocity_max;           ///< Maximum velocity (m/s)
    
    /**
     * @brief Constructor with default bounds
     * @param pos_min Minimum position
     * @param pos_max Maximum position
     * @param h_min Minimum heading (default: -π)
     * @param h_max Maximum heading (default: π)
     * @param v_min Minimum velocity (default: 0)
     * @param v_max Maximum velocity (default: 5.0 m/s)
     */
    Bounds(const Eigen::Vector2d& pos_min = Eigen::Vector2d(-10, -10),
           const Eigen::Vector2d& pos_max = Eigen::Vector2d(10, 10),
           double h_min = -M_PI,
           double h_max = M_PI,
           double v_min = 0.0,
           double v_max = 5.0);
    
    /**
     * @brief Check if a position is within bounds
     * @param position Position to check
     * @return true if within bounds, false otherwise
     */
    bool isPositionValid(const Eigen::Vector2d& position) const;
    
    /**
     * @brief Generate a random position within bounds
     * @return Random valid position
     */
    Eigen::Vector2d samplePosition() const;
    
    /**
     * @brief Generate a random heading within bounds
     * @return Random valid heading angle
     */
    double sampleHeading() const;
    
    /**
     * @brief Generate a random velocity within bounds
     * @return Random valid velocity
     */
    double sampleVelocity() const;
};

/**
 * @brief Represents a state in the configuration space
 * 
 * A state captures the complete configuration of a vehicle/robot at a given time.
 * For 2D planning, this includes position (x, y), heading angle, velocity, and timestamp.
 * 
 * Design choice: Using a struct instead of class for POD-like behavior.
 * This allows for efficient copying and vectorization.
 */
struct State {
    Eigen::Vector2d position;  ///< Position (x, y) in meters
    double heading;            ///< Heading angle in radians [-π, π]
    double velocity;           ///< Forward velocity in m/s
    double time;               ///< Timestamp in seconds
    
    /**
     * @brief Default constructor - initializes to zero state
     */
    State();
    
    /**
     * @brief Parameterized constructor
     * @param pos Position vector
     * @param head Heading angle (radians)
     * @param vel Velocity (m/s)
     * @param t Timestamp (seconds)
     */
    State(const Eigen::Vector2d& pos, double head = 0.0, double vel = 0.0, double t = 0.0);
    
    /**
     * @brief Convenience constructor from x, y coordinates
     * @param x X position
     * @param y Y position
     * @param head Heading angle (radians)
     * @param vel Velocity (m/s)
     * @param t Timestamp (seconds)
     */
    State(double x, double y, double head = 0.0, double vel = 0.0, double t = 0.0);
    
    /**
     * @brief Compute Euclidean distance to another state
     * @param other Target state
     * @return Euclidean distance in meters
     */
    double distanceTo(const State& other) const;
    
    /**
     * @brief Compute configuration space distance (includes heading)
     * 
     * Uses weighted combination of position and heading difference.
     * Useful for planning in SE(2) space.
     * 
     * @param other Target state
     * @param heading_weight Weight for heading difference (default: 0.5)
     * @return Weighted configuration space distance
     */
    double configurationDistanceTo(const State& other, double heading_weight = 0.5) const;
    
    /**
     * @brief Linear interpolation between states
     * @param other Target state
     * @param t Interpolation parameter [0, 1]
     * @return Interpolated state
     */
    State interpolate(const State& other, double t) const;
    
    /**
     * @brief Normalize heading angle to [-π, π]
     */
    void normalizeHeading();
    
    /**
     * @brief Check if state is within bounds
     * @param bounds Configuration space bounds
     * @return true if valid, false otherwise
     */
    bool isValid(const Bounds& bounds) const;
};

/**
 * @brief Represents a trajectory as a sequence of states
 * 
 * A trajectory is a time-parameterized path through configuration space.
 * Provides utilities for cost calculation, smoothness metrics, and validation.
 * 
 * Design pattern: This class encapsulates trajectory data and operations,
 * making it easy to pass around and analyze planning results.
 */
class Trajectory {
public:
    /**
     * @brief Default constructor - creates empty trajectory
     */
    Trajectory();
    
    /**
     * @brief Constructor from state sequence
     * @param states Vector of states forming the trajectory
     */
    explicit Trajectory(const std::vector<State>& states);
    
    /**
     * @brief Add a state to the end of the trajectory
     * @param state State to append
     */
    void addState(const State& state);
    
    /**
     * @brief Get the states in the trajectory
     * @return Const reference to state vector
     */
    const std::vector<State>& getStates() const { return states_; }
    
    /**
     * @brief Get mutable states (use with caution)
     * @return Reference to state vector
     */
    std::vector<State>& getStates() { return states_; }
    
    /**
     * @brief Get number of states in trajectory
     * @return Number of waypoints
     */
    size_t size() const { return states_.size(); }
    
    /**
     * @brief Check if trajectory is empty
     * @return true if no states, false otherwise
     */
    bool empty() const { return states_.empty(); }
    
    /**
     * @brief Clear all states
     */
    void clear() { states_.clear(); }
    
    /**
     * @brief Get state at index (with bounds checking)
     * @param index Index of state
     * @return Const reference to state
     * @throws std::out_of_range if index invalid
     */
    const State& at(size_t index) const { return states_.at(index); }
    
    /**
     * @brief Get state at index (no bounds checking)
     * @param index Index of state
     * @return Const reference to state
     */
    const State& operator[](size_t index) const { return states_[index]; }
    
    /**
     * @brief Calculate total path length (Euclidean)
     * 
     * Sums the Euclidean distances between consecutive waypoints.
     * This is a key metric for comparing path quality.
     * 
     * @return Total path length in meters
     */
    double getLength() const;
    
    /**
     * @brief Calculate total trajectory duration
     * @return Time difference between first and last state (seconds)
     */
    double getDuration() const;
    
    /**
     * @brief Calculate path curvature metric (smoothness)
     * 
     * Measures path smoothness by computing the sum of squared heading changes.
     * Lower values indicate smoother paths. Useful for comparing planners.
     * 
     * Formula: Σ(θ_i - θ_{i-1})² for all consecutive waypoints
     * 
     * @return Curvature metric (lower is smoother)
     */
    double getCurvature() const;
    
    /**
     * @brief Calculate trajectory cost (combined metric)
     * 
     * Weighted combination of length and curvature. This is often used
     * as the objective function in optimal planning (e.g., RRT*).
     * 
     * Cost = length + curvature_weight * curvature
     * 
     * @param curvature_weight Weight for curvature term (default: 1.0)
     * @return Total trajectory cost
     */
    double getCost(double curvature_weight = 1.0) const;
    
    /**
     * @brief Interpolate trajectory to fixed time step
     * 
     * Creates a new trajectory with waypoints at regular time intervals.
     * Useful for visualization and simulation.
     * 
     * @param dt Time step (seconds)
     * @return Interpolated trajectory
     */
    Trajectory interpolateToTimeStep(double dt) const;
    
    /**
     * @brief Downsample trajectory by keeping every n-th waypoint
     * @param factor Downsampling factor (keep every n-th point)
     * @return Downsampled trajectory
     */
    Trajectory downsample(size_t factor) const;
    
    /**
     * @brief Reverse the trajectory
     * @return Reversed trajectory
     */
    Trajectory reverse() const;

private:
    std::vector<State> states_;  ///< Sequence of states forming the trajectory
};

/**
 * @brief Compute angle difference in [-π, π]
 * 
 * Utility function for handling angle wraparound correctly.
 * Essential for heading-based distance metrics.
 * 
 * @param angle1 First angle (radians)
 * @param angle2 Second angle (radians)
 * @return Shortest angular difference (radians)
 */
double angleDifference(double angle1, double angle2);

/**
 * @brief Normalize angle to [-π, π]
 * @param angle Input angle (radians)
 * @return Normalized angle (radians)
 */
double normalizeAngle(double angle);

} // namespace fastplanner
