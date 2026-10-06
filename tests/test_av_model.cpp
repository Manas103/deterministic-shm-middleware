// Hand-rolled check suite for the AV profile's math (av_model.cpp). Each
// check compares against something that does not share code with the
// implementation: a closed-form trajectory, a finite difference, a
// geometric identity, or a statistical property of the noise construction.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "shmmw/av_model.hpp"

using namespace shmmw;
using namespace shmmw::av;

static int g_checks = 0;
#define CHECK(cond) do { \
    ++g_checks; \
    if (!(cond)) { std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); std::exit(1); } \
} while (0)
#define CHECK_NEAR(a, b, tol) do { \
    ++g_checks; \
    const double va_ = (a), vb_ = (b); \
    if (!(std::fabs(va_ - vb_) <= (tol))) { \
        std::fprintf(stderr, "FAIL %s:%d: %s = %.17g, want %.17g (tol %g)\n", __FILE__, __LINE__, #a, va_, vb_, (double)(tol)); \
        std::exit(1); \
    } \
} while (0)

constexpr double kPi = 3.14159265358979323846;

static bool same_bits(double a, double b) { return std::memcmp(&a, &b, sizeof(double)) == 0; }

// Straight-line constant acceleration: x(t) = v0 t + a t^2 / 2 exactly. RK4 is
// exact for polynomials of degree <= 4, so only rounding error remains.
void test_straight_line_matches_closed_form() {
    EgoState s{0, 0, 0, 10.0};
    const double dt = 0.01, accel = 1.0;
    for (int i = 0; i < 1000; ++i) {
        s = bicycle_rk4(s, accel, 0.0, 2.8, 0.5, dt);
        const double t = (i + 1) * dt;
        CHECK_NEAR(s.x, 10.0 * t + 0.5 * accel * t * t, 1e-9);
        CHECK(s.y == 0.0);   // sin(0) == 0 exactly, so no lateral drift at all
        CHECK(s.yaw == 0.0);
        CHECK_NEAR(s.v, 10.0 + accel * t, 1e-11);
    }
}

// Constant speed and steer: the rear axle traces a circle of radius
// R = L / tan(steer) centred at (0, R). Distance to the centre is the
// conserved quantity; RK4's O(dt^4) global error keeps it within 1e-6 m over
// 20 s (2000 steps, more than three full laps).
void test_constant_steer_traces_circle() {
    const double L = 2.8, steer = 0.2, v = 10.0, dt = 0.01;
    const double R = L / std::tan(steer);
    EgoState s{0, 0, 0, v};
    for (int i = 0; i < 2000; ++i) {
        s = bicycle_rk4(s, 0.0, steer, L, 0.5, dt);
        CHECK_NEAR(std::hypot(s.x - 0.0, s.y - R), R, 1e-6);
        CHECK(s.v == v);                       // zero accel never perturbs speed
        CHECK(s.yaw > -kPi && s.yaw <= kPi);   // wrapped every step
        const double t = (i + 1) * dt;
        CHECK_NEAR(std::cos(s.yaw), std::cos(v / R * t), 1e-9);
        CHECK_NEAR(std::sin(s.yaw), std::sin(v / R * t), 1e-9);
    }
}

// Braking to a stop: speed is clamped at zero and never goes negative, and
// once stopped the vehicle stays exactly where it is.
void test_brake_to_stop_never_reverses() {
    EgoState s{0, 0, 0, 5.0};
    bool stopped = false;
    double x_at_stop = 0.0;
    for (int i = 0; i < 300; ++i) {
        s = bicycle_rk4(s, -10.0, 0.1, 2.8, 0.5, 0.01);
        CHECK(s.v >= 0.0);
        if (stopped) CHECK(s.x == x_at_stop);
        if (!stopped && s.v == 0.0) { stopped = true; x_at_stop = s.x; }
    }
    CHECK(stopped);
}

void test_steer_is_clamped() {
    const EgoState s{1, 2, 0.3, 8.0};
    const EgoState a = bicycle_rk4(s, 0.5, 2.0, 2.8, 0.5, 0.01);
    const EgoState b = bicycle_rk4(s, 0.5, 0.5, 2.8, 0.5, 0.01);
    CHECK(same_bits(a.x, b.x) && same_bits(a.y, b.y) && same_bits(a.yaw, b.yaw) && same_bits(a.v, b.v));
    const EgoState c = bicycle_rk4(s, 0.5, -9.0, 2.8, 0.5, 0.01);
    const EgoState d = bicycle_rk4(s, 0.5, -0.5, 2.8, 0.5, 0.01);
    CHECK(same_bits(c.yaw, d.yaw));
}

void test_wrap_pi() {
    CHECK(wrap_pi(0.0) == 0.0);
    CHECK(wrap_pi(kPi) == kPi);
    CHECK_NEAR(wrap_pi(-kPi), kPi, 1e-15);
    CHECK_NEAR(wrap_pi(3 * kPi), kPi, 1e-14);
    CHECK_NEAR(wrap_pi(-1.5 * kPi), 0.5 * kPi, 1e-14);
    CHECK(wrap_pi(1.0) == 1.0); // in range: returned bit-exact
    for (int i = 0; i <= 20000; ++i) {
        const double a = -100.0 + i * 0.01;
        const double w = wrap_pi(a);
        CHECK(w > -kPi && w <= kPi);
        CHECK_NEAR(std::sin(w), std::sin(a), 1e-12);
        CHECK_NEAR(std::cos(w), std::cos(a), 1e-12);
    }
}

void test_agent_interpolation() {
    const std::vector<Waypoint> wps = {{1.0, 0.0, 0.0}, {3.0, 10.0, -4.0}, {4.0, 10.0, 6.0}};
    // Before the first waypoint and after the last: hold, zero velocity.
    AgentState a = agent_at(wps, 0.0);
    CHECK(a.x == 0.0 && a.y == 0.0 && a.vx == 0.0 && a.vy == 0.0);
    a = agent_at(wps, 9.0);
    CHECK(a.x == 10.0 && a.y == 6.0 && a.vx == 0.0 && a.vy == 0.0);
    // Exactly on interior waypoints: exact coordinates (u == 0 on the next segment).
    a = agent_at(wps, 3.0);
    CHECK(a.x == 10.0 && a.y == -4.0);
    CHECK(a.vx == 0.0 && a.vy == 10.0); // velocity of the segment starting there
    // Midpoint of the first segment and its slope.
    a = agent_at(wps, 2.0);
    CHECK_NEAR(a.x, 5.0, 1e-12);
    CHECK_NEAR(a.y, -2.0, 1e-12);
    CHECK(a.vx == 5.0 && a.vy == -2.0);
    // Position is continuous, and the reported velocity matches a central
    // finite difference of position everywhere away from the corners.
    for (int i = 1; i < 4000; ++i) {
        const double t = 0.5 + i * 0.001;
        const double h = 1e-6;
        if (std::fabs(t - 1.0) < 2e-6 || std::fabs(t - 3.0) < 2e-6 || std::fabs(t - 4.0) < 2e-6) continue;
        const AgentState p = agent_at(wps, t - h), q = agent_at(wps, t + h), m = agent_at(wps, t);
        CHECK_NEAR((q.x - p.x) / (2 * h), m.vx, 1e-5);
        CHECK_NEAR((q.y - p.y) / (2 * h), m.vy, 1e-5);
    }
    // A single waypoint holds forever.
    a = agent_at({{0.0, 3.0, 4.0}}, 100.0);
    CHECK(a.x == 3.0 && a.y == 4.0 && a.vx == 0.0);
}

void test_sensor_geometry_noise_free() {
    const EgoState ego{0, 0, 0, 10.0};
    Observation o = observe(ego, AgentState{50, 0, 0, 0}, 1, 0, 0, 120, 1, 0);
    CHECK(o.range == 50.0 && o.bearing == 0.0 && o.id == 1);
    CHECK_NEAR(o.range_rate, -10.0, 1e-12); // driving straight at a stationary object

    o = observe(ego, AgentState{0, 10, 0, 0}, 2, 0, 0, 120, 1, 0);
    CHECK_NEAR(o.bearing, kPi / 2, 1e-15); // directly left
    CHECK_NEAR(o.range_rate, 0.0, 1e-12);  // moving perpendicular to the line of sight

    // Heading rotates the bearing: ego facing +y sees an object at +x on its right.
    o = observe(EgoState{0, 0, kPi / 2, 0}, AgentState{7, 0, 0, 0}, 1, 0, 0, 120, 1, 0);
    CHECK_NEAR(o.bearing, -kPi / 2, 1e-15);

    // Beyond max range: still measured, but flagged not detected.
    o = observe(ego, AgentState{130, 0, 0, 0}, 3, 0, 0, 120, 1, 0);
    CHECK(o.id == 0);

    // Range rate equals the time derivative of the true range (finite
    // difference oracle) for an arbitrary relative motion.
    const EgoState e{1, -2, 0.4, 7.0};
    const AgentState ag{30, 12, -3, 1.5};
    const double h = 1e-6;
    auto true_range = [&](double dt) {
        const double ex = e.x + e.v * std::cos(e.yaw) * dt, ey = e.y + e.v * std::sin(e.yaw) * dt;
        return std::hypot(ag.x + ag.vx * dt - ex, ag.y + ag.vy * dt - ey);
    };
    o = observe(e, ag, 1, 0, 0, 120, 1, 0);
    CHECK_NEAR(o.range_rate, (true_range(h) - true_range(-h)) / (2 * h), 1e-6);
    CHECK_NEAR(o.range, true_range(0), 1e-12);
}

void test_deterministic_noise() {
    CHECK(same_bits(det_normal(42, 7, 3), det_normal(42, 7, 3)));
    CHECK(det_normal(42, 7, 3) != det_normal(42, 8, 3));
    CHECK(det_normal(42, 7, 3) != det_normal(42, 7, 4));
    CHECK(det_normal(42, 7, 3) != det_normal(43, 7, 3));
    // Irwin-Hall with 12 uniforms: mean 0, variance exactly 1, support [-6, 6].
    const int n = 200000;
    double sum = 0, sum2 = 0;
    for (int i = 0; i < n; ++i) {
        const double z = det_normal(20261002, static_cast<uint64_t>(i), 5);
        CHECK(z >= -6.0 && z <= 6.0);
        sum += z; sum2 += z * z;
    }
    const double mean = sum / n, var = sum2 / n - mean * mean;
    CHECK_NEAR(mean, 0.0, 0.01);
    CHECK_NEAR(var, 1.0, 0.02);

    // With noise on, the sensor is still a pure function of its arguments.
    const EgoState e{0, 0, 0, 10};
    const AgentState a{40, 3, 0, 0};
    const Observation o1 = observe(e, a, 2, 0.05, 0.002, 120, 99, 123);
    const Observation o2 = observe(e, a, 2, 0.05, 0.002, 120, 99, 123);
    CHECK(same_bits(o1.range, o2.range) && same_bits(o1.bearing, o2.bearing));
    const Observation o0 = observe(e, a, 2, 0, 0, 120, 99, 123);
    CHECK(o1.range != o0.range);
    CHECK(std::fabs(o1.range - o0.range) < 6 * 0.05 + 1e-12); // bounded by the Irwin-Hall support
}

void test_nearest_and_ttc() {
    Observation a; a.range = 30; a.id = 2; a.range_rate = -5;
    Observation b; b.range = 20; b.id = 3; b.range_rate = 1;
    Observation u; u.range = 5;  u.id = 0;
    CHECK(nearest(a, b, 120).id == 3);
    CHECK(nearest(b, a, 120).id == 3);     // argument order does not matter
    CHECK(nearest(a, u, 120).id == 2);     // undetected never wins, even when closer
    CHECK(nearest(u, u, 120).id == 0 && nearest(u, u, 120).range == 120);
    Observation tie = a; tie.id = 1;
    CHECK(nearest(a, tie, 120).id == 1 && nearest(tie, a, 120).id == 1);

    CHECK_NEAR(time_to_collision(a), 6.0, 1e-15);
    CHECK(time_to_collision(b) == -1.0);   // opening
    Observation c = a; c.id = 0;
    CHECK(time_to_collision(c) == -1.0);   // not detected
}

int main() {
    test_straight_line_matches_closed_form();
    test_constant_steer_traces_circle();
    test_brake_to_stop_never_reverses();
    test_steer_is_clamped();
    test_wrap_pi();
    test_agent_interpolation();
    test_sensor_geometry_noise_free();
    test_deterministic_noise();
    test_nearest_and_ttc();
    std::printf("check_av_model: %d checks passed\n", g_checks);
    return 0;
}
