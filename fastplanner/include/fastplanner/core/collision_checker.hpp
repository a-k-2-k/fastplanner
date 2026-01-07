#pragma once

#include "fastplanner/core/state.hpp"
#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <unordered_map>

namespace fastplanner {

/**
 * @brief Represents a circular obstacle in 2D space
 * 
 * Design choice: Using circles for simplicity and efficient collision checking.
 * Circle-circle and point-circle collisions are O(1) with simple distance checks.
 * Can be extended to polygons later if needed.
 */
struct CircleObstacle {
    Eigen::Vector2d center;    ///< Center position (x, y)
    double radius;             ///< Radius in meters
    Eigen::Vector2d velocity;  ///< Velocity for dynamic obstacles (m/s)
    bool is_dynamic;           ///< Whether obstacle moves
    
    /**
     * @brief Constructor for static obstacle
     * @param c Center position
     * @param r Radius
     */
    CircleObstacle(const Eigen::Vector2d& c, double r);
    
    /**
     * @brief Constructor for dynamic obstacle
     * @param c Center position
     * @param r Radius
     * @param v Velocity vector
     */
    CircleObstacle(const Eigen::Vector2d& c, double r, const Eigen::Vector2d& v);
    
    /**
     * @brief Predict obstacle position at future time
     * 
     * Uses constant velocity model. More sophisticated prediction
     * (e.g., constant acceleration, learned models) can be added later.
     * 
     * @param dt Time delta (seconds)
     * @return Predicted center position
     */
    Eigen::Vector2d predictPosition(double dt) const;
    
    /**
     * @brief Check if point is inside obstacle (with margin)
     * @param point Point to check
     * @param margin Safety margin (default: 0.0)
     * @return true if collision, false otherwise
     */
    bool collidesWith(const Eigen::Vector2d& point, double margin = 0.0) const;
};

/**
 * @brief Grid cell for spatial hashing
 * 
 * Used for O(1) nearest obstacle queries. Instead of checking all N obstacles,
 * we only check obstacles in nearby grid cells, reducing collision checks by ~100x.
 */
struct SpatialHashGrid {
    double cell_size;                                      ///< Grid cell size
    std::unordered_map<int64_t, std::vector<size_t>> grid; ///< Hash -> obstacle indices
    
    /**
     * @brief Constructor
     * @param cell_sz Grid cell size (should be ~2x average obstacle radius)
     */
    explicit SpatialHashGrid(double cell_sz = 2.0);
    
    /**
     * @brief Compute hash key for position
     * @param pos Position to hash
     * @return Hash key
     */
    int64_t getHashKey(const Eigen::Vector2d& pos) const;
    
    /**
     * @brief Insert obstacle into grid
     * @param obstacle Obstacle to insert
     * @param index Index in obstacle list
     */
    void insert(const CircleObstacle& obstacle, size_t index);
    
    /**
     * @brief Get nearby obstacle indices
     * @param pos Query position
     * @param radius Search radius
     * @return Vector of obstacle indices near position
     */
    std::vector<size_t> getNearbyObstacles(const Eigen::Vector2d& pos, double radius) const;
    
    /**
     * @brief Clear the grid
     */
    void clear();
};

/**
 * @brief Collision checking with spatial optimization
 * 
 * Core component for validating states and paths during planning.
 * Uses spatial hashing to accelerate queries from O(n) to ~O(1).
 * 
 * Key responsibilities:
 * - Maintain obstacle representation
 * - Check point/path collisions efficiently
 * - Handle dynamic obstacle prediction
 * - Provide safety margins for robust planning
 * 
 * Performance optimization: Spatial hashing reduces collision checks by ~100x
 * in dense environments. Critical for real-time planning (20+ Hz).
 */
class CollisionChecker {
public:
    /**
     * @brief Constructor with robot radius
     * @param robot_radius Radius of circular robot footprint (meters)
     * @param use_spatial_hashing Enable spatial hashing optimization (default: true)
     */
    explicit CollisionChecker(double robot_radius = 0.5, bool use_spatial_hashing = true);
    
    /**
     * @brief Add a static circular obstacle
     * @param center Obstacle center position
     * @param radius Obstacle radius
     */
    void addObstacle(const Eigen::Vector2d& center, double radius);
    
    /**
     * @brief Add a dynamic circular obstacle
     * @param center Obstacle center position
     * @param radius Obstacle radius
     * @param velocity Obstacle velocity vector
     */
    void addDynamicObstacle(const Eigen::Vector2d& center, double radius,
                           const Eigen::Vector2d& velocity);
    
    /**
     * @brief Add obstacle object directly
     * @param obstacle Obstacle to add
     */
    void addObstacle(const CircleObstacle& obstacle);
    
    /**
     * @brief Remove all obstacles
     */
    void clearObstacles();
    
    /**
     * @brief Get all obstacles
     * @return Const reference to obstacle vector
     */
    const std::vector<CircleObstacle>& getObstacles() const { return obstacles_; }
    
    /**
     * @brief Check if a state is collision-free
     * 
     * Checks robot footprint (circle) against all obstacles.
     * For dynamic obstacles, predicts their position at state.time.
     * 
     * @param state State to check
     * @param margin Additional safety margin (default: 0.0)
     * @return true if collision-free, false if collision
     */
    bool isStateFree(const State& state, double margin = 0.0) const;
    
    /**
     * @brief Check if a path segment is collision-free
     * 
     * Interpolates between two states and checks collision at regular intervals.
     * Resolution determines trade-off between accuracy and speed.
     * 
     * Design choice: Fixed resolution instead of adaptive. Adaptive would be
     * more accurate but adds complexity. For 20Hz planning, fixed is sufficient.
     * 
     * @param state1 Start state
     * @param state2 End state
     * @param resolution Check interval (default: 0.1 meters)
     * @param margin Safety margin (default: 0.0)
     * @return true if entire path is collision-free
     */
    bool isPathFree(const State& state1, const State& state2,
                   double resolution = 0.1, double margin = 0.0) const;
    
    /**
     * @brief Check if entire trajectory is collision-free
     * @param trajectory Trajectory to validate
     * @param margin Safety margin (default: 0.0)
     * @return true if collision-free, false otherwise
     */
    bool isTrajectoryFree(const Trajectory& trajectory, double margin = 0.0) const;
    
    /**
     * @brief Get distance to nearest obstacle
     * 
     * Returns minimum distance from point to any obstacle surface.
     * Useful for potential field methods and gradient-based optimization.
     * 
     * @param position Query position
     * @param time Time for dynamic obstacle prediction (default: 0.0)
     * @return Distance to nearest obstacle (meters), or infinity if no obstacles
     */
    double distanceToNearestObstacle(const Eigen::Vector2d& position, double time = 0.0) const;
    
    /**
     * @brief Get gradient of obstacle distance function
     * 
     * Computes the direction of maximum clearance increase.
     * Used in trajectory optimization for obstacle avoidance term.
     * 
     * Uses numerical differentiation with small epsilon.
     * 
     * @param position Query position
     * @param time Time for dynamic obstacle prediction
     * @return Gradient vector (direction away from nearest obstacle)
     */
    Eigen::Vector2d getObstacleGradient(const Eigen::Vector2d& position, double time = 0.0) const;
    
    /**
     * @brief Rebuild spatial hash (call after adding many obstacles)
     * 
     * Spatial hash is built lazily, but if you add many obstacles,
     * manually rebuilding can improve query performance.
     */
    void rebuildSpatialHash();
    
    /**
     * @brief Get robot radius
     * @return Robot footprint radius (meters)
     */
    double getRobotRadius() const { return robot_radius_; }
    
    /**
     * @brief Set robot radius
     * @param radius New robot radius (meters)
     */
    void setRobotRadius(double radius) { robot_radius_ = radius; }
    
    /**
     * @brief Get number of obstacles
     * @return Total obstacle count
     */
    size_t getObstacleCount() const { return obstacles_.size(); }

private:
    std::vector<CircleObstacle> obstacles_;  ///< All obstacles in environment
    double robot_radius_;                    ///< Robot footprint radius
    bool use_spatial_hashing_;              ///< Whether to use spatial optimization
    mutable SpatialHashGrid spatial_hash_;  ///< Spatial hash for fast queries
    mutable bool hash_valid_;               ///< Whether hash is up-to-date
    
    /**
     * @brief Ensure spatial hash is built
     */
    void ensureSpatialHashBuilt() const;
    
    /**
     * @brief Check collision with all obstacles (brute force)
     * @param position Position to check
     * @param time Time for prediction
     * @param margin Safety margin
     * @return true if collision-free
     */
    bool checkCollisionBruteForce(const Eigen::Vector2d& position,
                                 double time, double margin) const;
    
    /**
     * @brief Check collision using spatial hash (optimized)
     * @param position Position to check
     * @param time Time for prediction
     * @param margin Safety margin
     * @return true if collision-free
     */
    bool checkCollisionSpatialHash(const Eigen::Vector2d& position,
                                  double time, double margin) const;
};

} // namespace fastplanner
