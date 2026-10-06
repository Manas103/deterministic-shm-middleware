#pragma once
#include <cstdint>
#include <string>
#include <vector>

namespace shmmw {

// A driving scenario: everything the AV profile's blocks need that is not
// the graph topology. It is the only input to the AV graph besides the tick
// count, so the session digest of an AV run is a function of this file (and
// of the floating-point contract the binaries were built with, see README
// "Findings").
//
// Text format, one directive per line, `#` starts a comment, fields are
// whitespace separated, numbers are parsed with std::from_chars (locale
// independent and correctly rounded, so the same text always yields the same
// doubles on any machine):
//
//   name      <identifier>
//   dt        <seconds>                     fixed integration timestep, > 0
//   seed      <uint64>                      sensor-noise stream seed
//   wheelbase <meters>                      ego kinematic bicycle wheelbase
//   max_steer <radians>                     steering command clamp, optional
//   ego_init  <x> <y> <yaw_rad> <speed_mps>
//   cmd       <t_start_s> <accel_mps2> <steer_rad>   piecewise-constant ego command
//   agent     <name> <t_s> <x> <y>          one waypoint of a scripted traffic agent
//   sensor    <sigma_range_m> <sigma_bearing_rad> <max_range_m>
//
// `cmd` lines take effect at tick round(t_start/dt) and hold until the next
// one. `agent` lines for the same name must have strictly increasing times;
// the agent moves linearly between waypoints and holds its last waypoint.
struct EgoCommand {
    uint64_t start_tick = 0;
    double accel = 0.0;
    double steer = 0.0;
};

struct Waypoint {
    double t = 0.0;
    double x = 0.0;
    double y = 0.0;
};

struct AgentScript {
    std::string name;
    std::vector<Waypoint> waypoints;
};

struct Scenario {
    std::string name = "unnamed";
    double dt = 0.01;
    uint64_t seed = 0;
    double wheelbase = 2.8;
    double max_steer = 0.6;
    double ego_x = 0.0, ego_y = 0.0, ego_yaw = 0.0, ego_v = 0.0;
    std::vector<EgoCommand> commands; // sorted by start_tick
    std::vector<AgentScript> agents;  // in order of first appearance
    double sigma_range = 0.0;
    double sigma_bearing = 0.0;
    double max_range = 150.0;

    // 1-based agent id used on the wire (0 means "no detection"); -1 if absent.
    int agent_id(const std::string& agent_name) const;
    const EgoCommand& command_at(uint64_t tick) const;
};

// Throws std::runtime_error with "<path>:<line>: <reason>" on any malformed
// or semantically invalid line. A scenario is never partially loaded.
Scenario parse_scenario_text(const std::string& text, const std::string& origin = "<text>");
Scenario load_scenario(const std::string& path);

} // namespace shmmw
