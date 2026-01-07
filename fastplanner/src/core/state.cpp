#include "fastplanner/core/state.hpp"
#include <random>
#include <stdexcept>
#include <algorithm>

namespace fastplanner {

// ============================================================================
// Utility Functions
// ============================================================================

double angleDifference(double angle1, double angle2) {
    double diff = angle1 - angle2;
    // Normalize to [-π, π]
    while (diff > M_PI) diff -= 2.0 * M_PI;
    while (diff < -M_PI) diff += 2.0 * M_PI;
    return diff;
}

double normalizeAngle(double angle) {
    double normalized = angle;
    while (normalized > M_PI) normalized -= 2.0 * M_PI;
    while (normalized < -M_PI) normalized += 2.0 * M_PI;
    return normalized;
}

// ============================================================================
// Bounds Implementation
// ============================================================================

Bounds::Bounds(const Eigen::Vector2d& pos_min,
               const Eigen::Vector2d& pos_max,
               double h_min,
               double h_max,
               double v_min,
               double v_max)
    : position_min(pos_min),
      position_max(pos_max),
      heading_min(h_min),
      heading_max(h_max),
      velocity_min(v_min),
      velocity_max(v_max) {
}

bool Bounds::isPositionValid(const Eigen::Vector2d& position) const {
    return position.x() >= position_min.x() && position.x() <= position_max.x() &&
           position.y() >= position_min.y() && position.y() <= position_max.y();
}

Eigen::Vector2d Bounds::samplePosition() const {
    // Use thread-local random engine for efficiency
    static thread_local std::random_device rd;
    static thread_local std::mt19937 gen(rd());
    
    std::uniform_real_distribution<double> dist_x(position_min.x(), position_max.x());
    std::uniform_real_distribution<double> dist_y(position_min.y(), position_max.y());
    
    return Eigen::Vector2d(dist_x(gen), dist_y(gen));
}

double Bounds::sampleHeading() const {
    static thread_local std::random_device rd;
    static thread_local std::mt19937 gen(rd());
    
    std::uniform_real_distribution<double> dist(heading_min, heading_max);
    return dist(gen);
}

double Bounds::sampleVelocity() const {
    static thread_local std::random_device rd;
    static thread_local std::mt19937 gen(rd());
    
    std::uniform_real_distribution<double> dist(velocity_min, velocity_max);
    return dist(gen);
}

// ============================================================================
// State Implementation
// ============================================================================

State::State()
    : position(Eigen::Vector2d::Zero()),
      heading(0.0),
      velocity(0.0),
      time(0.0) {
}

State::State(const Eigen::Vector2d& pos, double head, double vel, double t)
    : position(pos),
      heading(head),
      velocity(vel),
      time(t) {
    normalizeHeading();
}

State::State(double x, double y, double head, double vel, double t)
    : position(x, y),
      heading(head),
      velocity(vel),
      time(t) {
    normalizeHeading();
}

double State::distanceTo(const State& other) const {
    // Euclidean distance in position space
    return (position - other.position).norm();
}

double State::configurationDistanceTo(const State& other, double heading_weight) const {
    // Combined metric: position + weighted heading difference
    // This is useful for planning in SE(2) space where orientation matters
    double pos_dist = distanceTo(other);
    double heading_diff = std::abs(angleDifference(heading, other.heading));
    
    // Weighted combination
    return pos_dist + heading_weight * heading_diff;
}

State State::interpolate(const State& other, double t) const {
    // Clamp t to [0, 1]
    t = std::clamp(t, 0.0, 1.0);
    
    // Linear interpolation for position and velocity
    Eigen::Vector2d interp_pos = position + t * (other.position - position);
    double interp_vel = velocity + t * (other.velocity - velocity);
    double interp_time = time + t * (other.time - time);
    
    // Special handling for heading to account for wraparound
    double heading_diff = angleDifference(other.heading, heading);
    double interp_heading = normalizeAngle(heading + t * heading_diff);
    
    return State(interp_pos, interp_heading, interp_vel, interp_time);
}

void State::normalizeHeading() {
    heading = normalizeAngle(heading);
}

bool State::isValid(const Bounds& bounds) const {
    return bounds.isPositionValid(position) &&
           heading >= bounds.heading_min &&
           heading <= bounds.heading_max &&
           velocity >= bounds.velocity_min &&
           velocity <= bounds.velocity_max;
}

// ============================================================================
// Trajectory Implementation
// ============================================================================

Trajectory::Trajectory() : states_() {
}

Trajectory::Trajectory(const std::vector<State>& states) : states_(states) {
}

void Trajectory::addState(const State& state) {
    states_.push_back(state);
}

double Trajectory::getLength() const {
    if (states_.size() < 2) {
        return 0.0;
    }
    
    double length = 0.0;
    for (size_t i = 1; i < states_.size(); ++i) {
        length += states_[i].distanceTo(states_[i - 1]);
    }
    
    return length;
}

double Trajectory::getDuration() const {
    if (states_.empty()) {
        return 0.0;
    }
    
    return states_.back().time - states_.front().time;
}

double Trajectory::getCurvature() const {
    if (states_.size() < 2) {
        return 0.0;
    }
    
    // Sum of squared heading changes
    // This penalizes sharp turns and rewards smooth paths
    double curvature = 0.0;
    for (size_t i = 1; i < states_.size(); ++i) {
        double heading_change = angleDifference(states_[i].heading, states_[i - 1].heading);
        curvature += heading_change * heading_change;
    }
    
    return curvature;
}

double Trajectory::getCost(double curvature_weight) const {
    // Combined objective: minimize path length and curvature
    // This is the typical cost function for RRT* and other optimal planners
    return getLength() + curvature_weight * getCurvature();
}

Trajectory Trajectory::interpolateToTimeStep(double dt) const {
    if (states_.size() < 2 || dt <= 0.0) {
        return *this;
    }
    
    Trajectory interpolated;
    
    double total_duration = getDuration();
    if (total_duration <= 0.0) {
        return *this;
    }
    
    double current_time = states_.front().time;
    size_t segment_idx = 0;
    
    // Add first state
    interpolated.addState(states_.front());
    
    // Interpolate at fixed time steps
    while (current_time < states_.back().time) {
        current_time += dt;
        
        // Find which segment we're in
        while (segment_idx < states_.size() - 1 &&
               states_[segment_idx + 1].time < current_time) {
            segment_idx++;
        }
        
        if (segment_idx >= states_.size() - 1) {
            break;
        }
        
        // Interpolate within segment
        const State& s1 = states_[segment_idx];
        const State& s2 = states_[segment_idx + 1];
        
        double segment_duration = s2.time - s1.time;
        if (segment_duration > 0.0) {
            double t = (current_time - s1.time) / segment_duration;
            interpolated.addState(s1.interpolate(s2, t));
        }
    }
    
    // Add last state
    if (interpolated.states_.back().time < states_.back().time) {
        interpolated.addState(states_.back());
    }
    
    return interpolated;
}

Trajectory Trajectory::downsample(size_t factor) const {
    if (factor <= 1 || states_.empty()) {
        return *this;
    }
    
    Trajectory downsampled;
    
    // Always keep first state
    downsampled.addState(states_.front());
    
    // Keep every n-th state
    for (size_t i = factor; i < states_.size(); i += factor) {
        downsampled.addState(states_[i]);
    }
    
    // Always keep last state if not already included
    if ((states_.size() - 1) % factor != 0) {
        downsampled.addState(states_.back());
    }
    
    return downsampled;
}

Trajectory Trajectory::reverse() const {
    Trajectory reversed;
    
    for (auto it = states_.rbegin(); it != states_.rend(); ++it) {
        reversed.addState(*it);
    }
    
    return reversed;
}

} // namespace fastplanner
