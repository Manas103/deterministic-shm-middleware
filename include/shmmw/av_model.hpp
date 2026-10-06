#pragma once
#include <cstdint>
#include <vector>

#include "shmmw/scenario.hpp"

namespace shmmw::av {

// Pure functions behind the AV profile's blocks. Nothing in here knows about
// messages, rings, ROS 2 or time; each function is a deterministic map from
// its arguments to its result, which is what lets the same scenario produce
// the same bytes through any transport.

struct EgoState {
    double x = 0.0, y = 0.0, yaw = 0.0, v = 0.0;
};

// Kinematic bicycle model about the rear axle:
//   x' = v cos(yaw), y' = v sin(yaw), yaw' = v tan(steer) / L, v' = accel
// integrated with classic fourth-order Runge-Kutta at a fixed dt. Speed is
// clamped at zero after the step (a vehicle braking to a stop does not
// reverse), and steer is clamped to +/- max_steer before the step.
EgoState bicycle_rk4(const EgoState& s, double accel, double steer, double wheelbase,
                     double max_steer, double dt);

struct AgentState {
    double x = 0.0, y = 0.0, vx = 0.0, vy = 0.0;
};

// Scripted traffic agent: linear interpolation between waypoints, velocity
// equal to the slope of the active segment, holds (zero velocity) before the
// first and after the last waypoint.
AgentState agent_at(const std::vector<Waypoint>& wps, double t);

struct Observation {
    double range = 0.0;      // meters
    double bearing = 0.0;    // radians, relative to ego heading, wrapped to (-pi, pi]
    double range_rate = 0.0; // meters/second, negative when closing
    int id = 0;              // 1-based agent id, 0 when not detected
};

// Wraps an angle to (-pi, pi].
double wrap_pi(double a);

// Counter-based deterministic noise: a SplitMix64 hash of (seed, tick,
// stream) turned into an approximately standard-normal sample with the
// Irwin-Hall construction (sum of 12 uniforms minus 6). There is no RNG
// state anywhere, so the noise on tick k is the same no matter which process
// computes it, in what order, or how many times.
uint64_t splitmix64(uint64_t x);
double det_normal(uint64_t seed, uint64_t tick, uint64_t stream);

// Range/bearing/range-rate sensor on the ego looking at one agent. Noise is
// added to range and bearing from det_normal streams (2*id, 2*id+1). An
// agent beyond max_range is reported with id 0 (not detected).
Observation observe(const EgoState& ego, const AgentState& agent, int id, double sigma_range,
                    double sigma_bearing, double max_range, uint64_t seed, uint64_t tick);

// Of two observations, the detected one with the smaller range; ties go to
// the lower id. If neither is detected, a not-detected observation at
// max_range is returned.
Observation nearest(const Observation& a, const Observation& b, double max_range);

// Time to collision along the line of sight: range / closing speed when
// closing on a detected object, -1 otherwise (no threat).
double time_to_collision(const Observation& o);

} // namespace shmmw::av
