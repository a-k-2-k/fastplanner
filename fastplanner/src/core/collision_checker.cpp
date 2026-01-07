#include "fastplanner/core/collision_checker.hpp"
#include <cmath>
#include <limits>
#include <algorithm>

namespace fastplanner {

// ============================================================================
// CircleObstacle Implementation
// ============================================================================

CircleObstacle::CircleObstacle(const Eigen::Vector2d& c, double r)
    : center(c),
      radius(r),
      velocity(Eigen::Vector2d::Zero()),
      is_dynamic(false) {
}

CircleObstacle::CircleObstacle(const Eigen::Vector2d& c, double r, const Eigen::Vector2d& v)
    : center(c),
      radius(r),
      velocity(v),
      is_dynamic(true) {
}

Eigen::Vector2d CircleObstacle::predictPosition(double dt) const {
    if (!is_dynamic || dt <= 0.0) {
        return center;
    }
    
    // Constant velocity prediction
    // Extension idea: Add acceleration term for more accurate prediction
    return center + velocity * dt;
}

bool CircleObstacle::collidesWith(const Eigen::Vector2d& point, double margin) const {
    double distance = (point - center).norm();
    return distance <= (radius + margin);
}

// ============================================================================
// SpatialHashGrid Implementation
// ============================================================================

SpatialHashGrid::SpatialHashGrid(double cell_sz)
    : cell_size(cell_sz) {
}

int64_t SpatialHashGrid::getHashKey(const Eigen::Vector2d& pos) const {
    // Map position to grid cell
    int64_t x = static_cast<int64_t>(std::floor(pos.x() / cell_size));
    int64_t y = static_cast<int64_t>(std::floor(pos.y() / cell_size));
    
    // Cantor pairing function for 2D -> 1D hash
    // Alternative: use std::hash<std::pair<int, int>> but this is faster
    return (x + y) * (x + y + 1) / 2 + y;
}

void SpatialHashGrid::insert(const CircleObstacle& obstacle, size_t index) {
    // Insert obstacle into all cells it overlaps
    // Calculate bounding box in grid coordinates
    int x_min = static_cast<int>(std::floor((obstacle.center.x() - obstacle.radius) / cell_size));
    int x_max = static_cast<int>(std::ceil((obstacle.center.x() + obstacle.radius) / cell_size));
    int y_min = static_cast<int>(std::floor((obstacle.center.y() - obstacle.radius) / cell_size));
    int y_max = static_cast<int>(std::ceil((obstacle.center.y() + obstacle.radius) / cell_size));
    
    // Insert into all overlapping cells
    for (int x = x_min; x <= x_max; ++x) {
        for (int y = y_min; y <= y_max; ++y) {
            Eigen::Vector2d cell_center(x * cell_size, y * cell_size);
            int64_t key = getHashKey(cell_center);
            grid[key].push_back(index);
        }
    }
}

std::vector<size_t> SpatialHashGrid::getNearbyObstacles(const Eigen::Vector2d& pos,
                                                        double radius) const {
    std::vector<size_t> nearby;
    
    // Query all cells within radius
    int x_min = static_cast<int>(std::floor((pos.x() - radius) / cell_size));
    int x_max = static_cast<int>(std::ceil((pos.x() + radius) / cell_size));
    int y_min = static_cast<int>(std::floor((pos.y() - radius) / cell_size));
    int y_max = static_cast<int>(std::ceil((pos.y() + radius) / cell_size));
    
    // Use set to avoid duplicates (obstacle can be in multiple cells)
    std::unordered_map<size_t, bool> seen;
    
    for (int x = x_min; x <= x_max; ++x) {
        for (int y = y_min; y <= y_max; ++y) {
            Eigen::Vector2d cell_center(x * cell_size, y * cell_size);
            int64_t key = getHashKey(cell_center);
            
            auto it = grid.find(key);
            if (it != grid.end()) {
                for (size_t idx : it->second) {
                    if (seen.find(idx) == seen.end()) {
                        nearby.push_back(idx);
                        seen[idx] = true;
                    }
                }
            }
        }
    }
    
    return nearby;
}

void SpatialHashGrid::clear() {
    grid.clear();
}

// ============================================================================
// CollisionChecker Implementation
// ============================================================================

CollisionChecker::CollisionChecker(double robot_radius, bool use_spatial_hashing)
    : robot_radius_(robot_radius),
      use_spatial_hashing_(use_spatial_hashing),
      spatial_hash_(2.0),  // Default cell size = 2 * typical obstacle radius
      hash_valid_(false) {
}

void CollisionChecker::addObstacle(const Eigen::Vector2d& center, double radius) {
    obstacles_.emplace_back(center, radius);
    hash_valid_ = false;  // Invalidate spatial hash
}

void CollisionChecker::addDynamicObstacle(const Eigen::Vector2d& center, double radius,
                                         const Eigen::Vector2d& velocity) {
    obstacles_.emplace_back(center, radius, velocity);
    hash_valid_ = false;
}

void CollisionChecker::addObstacle(const CircleObstacle& obstacle) {
    obstacles_.push_back(obstacle);
    hash_valid_ = false;
}

void CollisionChecker::clearObstacles() {
    obstacles_.clear();
    spatial_hash_.clear();
    hash_valid_ = false;
}

void CollisionChecker::ensureSpatialHashBuilt() const {
    if (!use_spatial_hashing_ || hash_valid_) {
        return;
    }
    
    // Rebuild spatial hash
    spatial_hash_.clear();
    
    for (size_t i = 0; i < obstacles_.size(); ++i) {
        spatial_hash_.insert(obstacles_[i], i);
    }
    
    hash_valid_ = true;
}

bool CollisionChecker::checkCollisionBruteForce(const Eigen::Vector2d& position,
                                               double time, double margin) const {
    // Check against all obstacles
    for (const auto& obstacle : obstacles_) {
        // Predict obstacle position if dynamic
        Eigen::Vector2d obs_pos = obstacle.is_dynamic ?
            obstacle.predictPosition(time) : obstacle.center;
        
        // Check collision: distance < sum of radii
        double distance = (position - obs_pos).norm();
        double collision_distance = robot_radius_ + obstacle.radius + margin;
        
        if (distance < collision_distance) {
            return false;  // Collision detected
        }
    }
    
    return true;  // No collision
}

bool CollisionChecker::checkCollisionSpatialHash(const Eigen::Vector2d& position,
                                                double time, double margin) const {
    ensureSpatialHashBuilt();
    
    // Query nearby obstacles using spatial hash
    double query_radius = robot_radius_ + margin + 2.0;  // Conservative estimate
    std::vector<size_t> nearby = spatial_hash_.getNearbyObstacles(position, query_radius);
    
    // Check only nearby obstacles
    for (size_t idx : nearby) {
        const auto& obstacle = obstacles_[idx];
        
        // Predict obstacle position if dynamic
        Eigen::Vector2d obs_pos = obstacle.is_dynamic ?
            obstacle.predictPosition(time) : obstacle.center;
        
        // Check collision
        double distance = (position - obs_pos).norm();
        double collision_distance = robot_radius_ + obstacle.radius + margin;
        
        if (distance < collision_distance) {
            return false;  // Collision detected
        }
    }
    
    return true;  // No collision
}

bool CollisionChecker::isStateFree(const State& state, double margin) const {
    if (obstacles_.empty()) {
        return true;
    }
    
    if (use_spatial_hashing_) {
        return checkCollisionSpatialHash(state.position, state.time, margin);
    } else {
        return checkCollisionBruteForce(state.position, state.time, margin);
    }
}

bool CollisionChecker::isPathFree(const State& state1, const State& state2,
                                 double resolution, double margin) const {
    if (obstacles_.empty()) {
        return true;
    }
    
    double distance = state1.distanceTo(state2);
    
    // Early exit for very short paths
    if (distance < 1e-6) {
        return isStateFree(state1, margin);
    }
    
    // Calculate number of checks needed
    int num_checks = static_cast<int>(std::ceil(distance / resolution));
    num_checks = std::max(2, num_checks);  // At least check endpoints
    
    // Check collision at regular intervals along path
    for (int i = 0; i <= num_checks; ++i) {
        double t = static_cast<double>(i) / num_checks;
        State intermediate = state1.interpolate(state2, t);
        
        if (!isStateFree(intermediate, margin)) {
            return false;  // Collision detected
        }
    }
    
    return true;  // Path is collision-free
}

bool CollisionChecker::isTrajectoryFree(const Trajectory& trajectory, double margin) const {
    if (trajectory.empty()) {
        return true;
    }
    
    // Check each waypoint
    for (size_t i = 0; i < trajectory.size(); ++i) {
        if (!isStateFree(trajectory[i], margin)) {
            return false;
        }
    }
    
    // Check segments between waypoints
    for (size_t i = 1; i < trajectory.size(); ++i) {
        if (!isPathFree(trajectory[i - 1], trajectory[i], 0.1, margin)) {
            return false;
        }
    }
    
    return true;
}

double CollisionChecker::distanceToNearestObstacle(const Eigen::Vector2d& position,
                                                   double time) const {
    if (obstacles_.empty()) {
        return std::numeric_limits<double>::infinity();
    }
    
    double min_distance = std::numeric_limits<double>::infinity();
    
    if (use_spatial_hashing_) {
        ensureSpatialHashBuilt();
        
        // Query nearby obstacles
        double query_radius = 10.0;  // Search within 10m
        std::vector<size_t> nearby = spatial_hash_.getNearbyObstacles(position, query_radius);
        
        for (size_t idx : nearby) {
            const auto& obstacle = obstacles_[idx];
            Eigen::Vector2d obs_pos = obstacle.is_dynamic ?
                obstacle.predictPosition(time) : obstacle.center;
            
            // Distance to obstacle surface (not center)
            double distance = (position - obs_pos).norm() - obstacle.radius;
            min_distance = std::min(min_distance, distance);
        }
    } else {
        // Brute force check all obstacles
        for (const auto& obstacle : obstacles_) {
            Eigen::Vector2d obs_pos = obstacle.is_dynamic ?
                obstacle.predictPosition(time) : obstacle.center;
            
            double distance = (position - obs_pos).norm() - obstacle.radius;
            min_distance = std::min(min_distance, distance);
        }
    }
    
    return min_distance;
}

Eigen::Vector2d CollisionChecker::getObstacleGradient(const Eigen::Vector2d& position,
                                                      double time) const {
    // Numerical gradient using finite differences
    // This gives the direction of maximum clearance increase
    
    const double epsilon = 0.01;  // 1cm perturbation
    
    double dist_center = distanceToNearestObstacle(position, time);
    
    // Compute partial derivatives
    double dist_x_plus = distanceToNearestObstacle(
        position + Eigen::Vector2d(epsilon, 0), time);
    double dist_y_plus = distanceToNearestObstacle(
        position + Eigen::Vector2d(0, epsilon), time);
    
    double grad_x = (dist_x_plus - dist_center) / epsilon;
    double grad_y = (dist_y_plus - dist_center) / epsilon;
    
    return Eigen::Vector2d(grad_x, grad_y);
}

void CollisionChecker::rebuildSpatialHash() {
    if (!use_spatial_hashing_) {
        return;
    }
    
    hash_valid_ = false;
    ensureSpatialHashBuilt();
}

} // namespace fastplanner
