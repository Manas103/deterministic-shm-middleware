// Hand-rolled check suite for the scenario parser. The session digest of an
// AV run is a function of the scenario file, so the parser has to be strict
// (a typo must fail loudly, never load a half-scenario) and exact (the same
// text must always produce the same doubles).
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <string>

#include "shmmw/scenario.hpp"

using namespace shmmw;

static int g_checks = 0;
#define CHECK(cond) do { \
    ++g_checks; \
    if (!(cond)) { std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); std::exit(1); } \
} while (0)

static const char* kValid = R"(# comment line
name   t1          # trailing comment
dt     0.01
seed   18446744073709551615
wheelbase 2.9
max_steer 0.5
ego_init 1 -2 0.25 20
sensor 0.05 0.002 90
cmd 0.0 0.0 0.0
cmd 1.5 -2.5 0.0
cmd 1.5  1.0 0.1
cmd 3.004 0.5 -0.1
agent a1 0 40 0
agent a2 0 10 3.7
agent a1 6 160 0
)";

// Returns the exception message, or "" if parsing succeeded.
static std::string parse_error(const std::string& text) {
    try { parse_scenario_text(text, "mem"); } catch (const std::runtime_error& e) { return e.what(); }
    return "";
}

void test_valid_scenario() {
    const Scenario s = parse_scenario_text(kValid, "mem");
    CHECK(s.name == "t1");
    CHECK(s.dt == 0.01);
    CHECK(s.seed == 18446744073709551615ULL);
    CHECK(s.wheelbase == 2.9 && s.max_steer == 0.5);
    CHECK(s.ego_x == 1 && s.ego_y == -2 && s.ego_yaw == 0.25 && s.ego_v == 20);
    CHECK(s.sigma_range == 0.05 && s.sigma_bearing == 0.002 && s.max_range == 90);
    // Agents in order of first appearance; waypoints grouped by name.
    CHECK(s.agents.size() == 2);
    CHECK(s.agents[0].name == "a1" && s.agents[0].waypoints.size() == 2);
    CHECK(s.agents[1].name == "a2" && s.agents[1].waypoints.size() == 1);
    CHECK(s.agent_id("a1") == 1 && s.agent_id("a2") == 2 && s.agent_id("zz") == -1);
    CHECK(s.agents[0].waypoints[1].x == 160.0);
    // Command start times become integer ticks: 1.5/0.01 -> 150, 3.004/0.01 -> 300.
    CHECK(s.commands.size() == 4);
    CHECK(s.commands[1].start_tick == 150 && s.commands[3].start_tick == 300);
    CHECK(s.command_at(0).accel == 0.0);
    CHECK(s.command_at(149).accel == 0.0);
    // Two commands at the same tick: the later line wins.
    CHECK(s.command_at(150).accel == 1.0 && s.command_at(150).steer == 0.1);
    CHECK(s.command_at(299).accel == 1.0);
    CHECK(s.command_at(300).accel == 0.5);
    CHECK(s.command_at(1000000).steer == -0.1);
}

void test_rejects_bad_input() {
    const std::string base = "dt 0.01\nego_init 0 0 0 1\n";
    std::string e;
    e = parse_error(base + "bogus 1\n");
    CHECK(e.find("mem:3") != std::string::npos && e.find("unknown directive") != std::string::npos);
    e = parse_error(base + "cmd 1 2\n");
    CHECK(e.find("mem:3") != std::string::npos && e.find("expects 3") != std::string::npos);
    e = parse_error(base + "agent a1 0 1x 0\n");
    CHECK(e.find("not a finite number") != std::string::npos);
    e = parse_error(base + "agent a1 0 nan 0\n");
    CHECK(e.find("not a finite number") != std::string::npos);
    e = parse_error(base + "agent a1 1 0 0\nagent a1 1 5 0\n");
    CHECK(e.find("mem:4") != std::string::npos && e.find("strictly increasing") != std::string::npos);
    e = parse_error(base + "cmd 2 0 0\ncmd 1 0 0\n");
    CHECK(e.find("non-decreasing") != std::string::npos);
    CHECK(parse_error("ego_init 0 0 0 1\n").find("missing 'dt'") != std::string::npos);
    CHECK(parse_error("dt 0.01\n").find("missing 'ego_init'") != std::string::npos);
    CHECK(parse_error("dt 0\nego_init 0 0 0 1\n").find("dt must be > 0") != std::string::npos);
    CHECK(parse_error("dt 0.01\nego_init 0 0 0 -1\n").find(">= 0") != std::string::npos);
    CHECK(parse_error(base + "seed -3\n").find("unsigned") != std::string::npos);
    CHECK(parse_error(base + "sensor 0.1 0.1 0\n").find("max_range") != std::string::npos);
    CHECK(parse_error(base).empty());
}

void test_parse_is_exact_and_repeatable() {
    // from_chars is correctly rounded: the parsed value is the double nearest
    // the decimal text, the same one the compiler picks for the literal.
    const Scenario s = parse_scenario_text("dt 0.1\nego_init 0.30000000000000004 1e-3 -0 7\n", "mem");
    CHECK(s.dt == 0.1);
    CHECK(s.ego_x == 0.30000000000000004 && s.ego_y == 1e-3);
    CHECK(std::signbit(s.ego_yaw)); // "-0" parses to negative zero, not positive
    const Scenario a = parse_scenario_text(kValid, "mem"), b = parse_scenario_text(kValid, "mem");
    CHECK(a.agents[1].waypoints[0].y == b.agents[1].waypoints[0].y);
}

void test_shipped_scenarios_load() {
    const std::string dir = SHMMW_SOURCE_DIR "/scenarios/";
    for (const char* f : {"highway_cut_in.scn", "urban_crossing.scn"}) {
        const Scenario s = load_scenario(dir + f);
        CHECK(s.agents.size() == 3);
        CHECK(s.agent_id("a1") == 1 && s.agent_id("a2") == 2 && s.agent_id("a3") == 3);
        CHECK(s.dt == 0.01);
    }
    bool threw = false;
    try { load_scenario(dir + "does_not_exist.scn"); } catch (const std::runtime_error&) { threw = true; }
    CHECK(threw);
}

int main() {
    test_valid_scenario();
    test_rejects_bad_input();
    test_parse_is_exact_and_repeatable();
    test_shipped_scenarios_load();
    std::printf("check_scenario: %d checks passed\n", g_checks);
    return 0;
}
