#include "shmmw/scenario.hpp"

#include <charconv>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace shmmw {

namespace {

[[noreturn]] void fail(const std::string& origin, int line, const std::string& why) {
    throw std::runtime_error(origin + ":" + std::to_string(line) + ": " + why);
}

double num(const std::string& tok, const std::string& origin, int line) {
    double v = 0.0;
    auto res = std::from_chars(tok.data(), tok.data() + tok.size(), v);
    if (res.ec != std::errc() || res.ptr != tok.data() + tok.size() || !std::isfinite(v)) {
        fail(origin, line, "not a finite number: '" + tok + "'");
    }
    return v;
}

uint64_t u64(const std::string& tok, const std::string& origin, int line) {
    uint64_t v = 0;
    auto res = std::from_chars(tok.data(), tok.data() + tok.size(), v);
    if (res.ec != std::errc() || res.ptr != tok.data() + tok.size()) {
        fail(origin, line, "not an unsigned integer: '" + tok + "'");
    }
    return v;
}

struct RawCmd { double t, accel, steer; int line; };

} // namespace

int Scenario::agent_id(const std::string& agent_name) const {
    for (size_t i = 0; i < agents.size(); ++i) {
        if (agents[i].name == agent_name) return static_cast<int>(i) + 1;
    }
    return -1;
}

const EgoCommand& Scenario::command_at(uint64_t tick) const {
    static const EgoCommand kIdle{};
    const EgoCommand* active = &kIdle;
    for (const auto& c : commands) {
        if (c.start_tick <= tick) active = &c;
        else break;
    }
    return *active;
}

Scenario parse_scenario_text(const std::string& text, const std::string& origin) {
    Scenario s;
    std::vector<RawCmd> raw_cmds;
    bool have_dt = false, have_ego = false;
    std::istringstream in(text);
    std::string line;
    int lineno = 0;
    while (std::getline(in, line)) {
        ++lineno;
        const auto hash = line.find('#');
        if (hash != std::string::npos) line.erase(hash);
        std::istringstream ls(line);
        std::vector<std::string> tok;
        for (std::string t; ls >> t;) tok.push_back(t);
        if (tok.empty()) continue;
        const std::string& kw = tok[0];
        auto need = [&](size_t n) {
            if (tok.size() != n + 1) {
                fail(origin, lineno, "'" + kw + "' expects " + std::to_string(n) + " fields, got " +
                                         std::to_string(tok.size() - 1));
            }
        };
        if (kw == "name") { need(1); s.name = tok[1]; }
        else if (kw == "dt") {
            need(1); s.dt = num(tok[1], origin, lineno); have_dt = true;
            if (!(s.dt > 0.0)) fail(origin, lineno, "dt must be > 0");
        }
        else if (kw == "seed") { need(1); s.seed = u64(tok[1], origin, lineno); }
        else if (kw == "wheelbase") {
            need(1); s.wheelbase = num(tok[1], origin, lineno);
            if (!(s.wheelbase > 0.0)) fail(origin, lineno, "wheelbase must be > 0");
        }
        else if (kw == "max_steer") {
            need(1); s.max_steer = num(tok[1], origin, lineno);
            if (!(s.max_steer > 0.0 && s.max_steer < 1.5)) fail(origin, lineno, "max_steer must be in (0, 1.5)");
        }
        else if (kw == "ego_init") {
            need(4);
            s.ego_x = num(tok[1], origin, lineno); s.ego_y = num(tok[2], origin, lineno);
            s.ego_yaw = num(tok[3], origin, lineno); s.ego_v = num(tok[4], origin, lineno);
            if (s.ego_v < 0.0) fail(origin, lineno, "ego speed must be >= 0");
            have_ego = true;
        }
        else if (kw == "cmd") {
            need(3);
            raw_cmds.push_back({num(tok[1], origin, lineno), num(tok[2], origin, lineno),
                                num(tok[3], origin, lineno), lineno});
        }
        else if (kw == "agent") {
            need(4);
            Waypoint w{num(tok[2], origin, lineno), num(tok[3], origin, lineno), num(tok[4], origin, lineno)};
            AgentScript* a = nullptr;
            for (auto& existing : s.agents) if (existing.name == tok[1]) a = &existing;
            if (!a) { s.agents.push_back({tok[1], {}}); a = &s.agents.back(); }
            if (!a->waypoints.empty() && !(w.t > a->waypoints.back().t)) {
                fail(origin, lineno, "agent " + tok[1] + " waypoint times must be strictly increasing");
            }
            a->waypoints.push_back(w);
        }
        else if (kw == "sensor") {
            need(3);
            s.sigma_range = num(tok[1], origin, lineno);
            s.sigma_bearing = num(tok[2], origin, lineno);
            s.max_range = num(tok[3], origin, lineno);
            if (s.sigma_range < 0 || s.sigma_bearing < 0 || !(s.max_range > 0)) {
                fail(origin, lineno, "sensor sigmas must be >= 0 and max_range > 0");
            }
        }
        else fail(origin, lineno, "unknown directive '" + kw + "'");
    }
    if (!have_dt) fail(origin, lineno, "missing 'dt'");
    if (!have_ego) fail(origin, lineno, "missing 'ego_init'");

    // Command start times become integer ticks once, here, so "which command
    // is active at tick k" is an integer comparison and never depends on how
    // k*dt happens to round.
    for (const auto& rc : raw_cmds) {
        if (rc.t < 0.0) fail(origin, rc.line, "cmd start time must be >= 0");
        EgoCommand c;
        c.start_tick = static_cast<uint64_t>(std::llround(rc.t / s.dt));
        c.accel = rc.accel;
        c.steer = rc.steer;
        if (!s.commands.empty() && c.start_tick < s.commands.back().start_tick) {
            fail(origin, rc.line, "cmd start times must be non-decreasing");
        }
        s.commands.push_back(c);
    }
    return s;
}

Scenario load_scenario(const std::string& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("could not open scenario: " + path);
    std::ostringstream ss;
    ss << f.rdbuf();
    return parse_scenario_text(ss.str(), path);
}

} // namespace shmmw
