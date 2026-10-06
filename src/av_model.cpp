#include "shmmw/av_model.hpp"

#include <algorithm>
#include <cmath>

namespace shmmw::av {

namespace {
constexpr double kPi = 3.14159265358979323846;

struct Deriv {
    double dx, dy, dyaw, dv;
};

Deriv bicycle_deriv(const EgoState& s, double accel, double steer, double wheelbase) {
    return {s.v * std::cos(s.yaw), s.v * std::sin(s.yaw), s.v * std::tan(steer) / wheelbase, accel};
}

EgoState advance(const EgoState& s, const Deriv& d, double h) {
    // Clamp v >= 0 at every RK4 stage, not just the final output. Without
    // this, a braking step whose *final* v clamps to exactly 0 still feeds
    // a negative intermediate v (s.v + h/2*dv with dv < 0) into k2/k3's own
    // bicycle_deriv call, which leaks a small nonzero v*cos(yaw) into dx/dy
    // even though the step's reported v never goes negative. The vehicle
    // then drifts backward by a tiny amount on every step after it has
    // nominally stopped, one RK4 stage at a time.
    return {s.x + h * d.dx, s.y + h * d.dy, s.yaw + h * d.dyaw, std::max(0.0, s.v + h * d.dv)};
}
} // namespace

EgoState bicycle_rk4(const EgoState& s, double accel, double steer, double wheelbase,
                     double max_steer, double dt) {
    const double st = std::clamp(steer, -max_steer, max_steer);
    const Deriv k1 = bicycle_deriv(s, accel, st, wheelbase);
    const Deriv k2 = bicycle_deriv(advance(s, k1, dt / 2), accel, st, wheelbase);
    const Deriv k3 = bicycle_deriv(advance(s, k2, dt / 2), accel, st, wheelbase);
    const Deriv k4 = bicycle_deriv(advance(s, k3, dt), accel, st, wheelbase);
    EgoState n;
    n.x = s.x + dt / 6.0 * (k1.dx + 2.0 * k2.dx + 2.0 * k3.dx + k4.dx);
    n.y = s.y + dt / 6.0 * (k1.dy + 2.0 * k2.dy + 2.0 * k3.dy + k4.dy);
    n.yaw = wrap_pi(s.yaw + dt / 6.0 * (k1.dyaw + 2.0 * k2.dyaw + 2.0 * k3.dyaw + k4.dyaw));
    n.v = std::max(0.0, s.v + dt / 6.0 * (k1.dv + 2.0 * k2.dv + 2.0 * k3.dv + k4.dv));
    return n;
}

AgentState agent_at(const std::vector<Waypoint>& wps, double t) {
    AgentState a;
    if (wps.empty()) return a;
    if (t <= wps.front().t) { a.x = wps.front().x; a.y = wps.front().y; return a; }
    if (t >= wps.back().t) { a.x = wps.back().x; a.y = wps.back().y; return a; }
    for (size_t i = 0; i + 1 < wps.size(); ++i) {
        const Waypoint& p = wps[i];
        const Waypoint& q = wps[i + 1];
        if (t < q.t) {
            const double span = q.t - p.t;
            const double u = (t - p.t) / span;
            a.x = p.x + u * (q.x - p.x);
            a.y = p.y + u * (q.y - p.y);
            a.vx = (q.x - p.x) / span;
            a.vy = (q.y - p.y) / span;
            return a;
        }
    }
    a.x = wps.back().x; a.y = wps.back().y;
    return a;
}

double wrap_pi(double a) {
    // In range: return unchanged, so a yaw that never wraps never picks up
    // the rounding of an (a + pi) - pi round trip. Out of range: fmod-based,
    // a pure function of `a` with no loop count.
    if (a > -kPi && a <= kPi) return a;
    double r = std::fmod(a + kPi, 2.0 * kPi);
    if (r <= 0.0) r += 2.0 * kPi;
    return r - kPi;
}

uint64_t splitmix64(uint64_t x) {
    x += 0x9e3779b97f4a7c15ULL;
    x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ULL;
    x = (x ^ (x >> 27)) * 0x94d049bb133111ebULL;
    return x ^ (x >> 31);
}

double det_normal(uint64_t seed, uint64_t tick, uint64_t stream) {
    uint64_t state = splitmix64(seed ^ splitmix64(tick ^ splitmix64(stream)));
    double sum = 0.0;
    for (int i = 0; i < 12; ++i) {
        state = splitmix64(state);
        sum += static_cast<double>(state >> 11) * 0x1.0p-53; // uniform [0, 1)
    }
    return sum - 6.0;
}

Observation observe(const EgoState& ego, const AgentState& agent, int id, double sigma_range,
                    double sigma_bearing, double max_range, uint64_t seed, uint64_t tick) {
    const double dx = agent.x - ego.x;
    const double dy = agent.y - ego.y;
    const double range_true = std::hypot(dx, dy);
    const double ego_vx = ego.v * std::cos(ego.yaw);
    const double ego_vy = ego.v * std::sin(ego.yaw);
    Observation o;
    o.range_rate = range_true > 0.0 ? (dx * (agent.vx - ego_vx) + dy * (agent.vy - ego_vy)) / range_true : 0.0;
    const uint64_t s = static_cast<uint64_t>(id);
    o.range = std::max(0.0, range_true + sigma_range * det_normal(seed, tick, 2 * s));
    o.bearing = wrap_pi(std::atan2(dy, dx) - ego.yaw + sigma_bearing * det_normal(seed, tick, 2 * s + 1));
    o.id = (range_true <= max_range) ? id : 0;
    return o;
}

Observation nearest(const Observation& a, const Observation& b, double max_range) {
    const bool da = a.id > 0, db = b.id > 0;
    if (da && db) {
        if (a.range < b.range) return a;
        if (b.range < a.range) return b;
        return a.id <= b.id ? a : b;
    }
    if (da) return a;
    if (db) return b;
    Observation none;
    none.range = max_range;
    return none;
}

double time_to_collision(const Observation& o) {
    if (o.id <= 0 || !(o.range_rate < -1e-9)) return -1.0;
    return o.range / -o.range_rate;
}

} // namespace shmmw::av
