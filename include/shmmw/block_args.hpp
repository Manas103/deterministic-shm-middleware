#pragma once
#include <cstdint>
#include <map>
#include <sstream>
#include <string>

#include "shmmw/block_engine.hpp"

namespace shmmw {

// Command line shared by both per-block executables (block_runner for the
// shared-memory transport, ros2_block_node for ROS 2). graph_launcher builds
// the same argument vector for either one.
struct BlockArgs {
    std::string config_path;
    std::string block_name;
    std::string run_id;
    std::string log_path;
    std::string scenario_path;
    uint32_t num_blocks = 0;                           // start barrier size; 0 disables it
    std::map<std::string, uint32_t> edge_cap_override; // shm only: edge name -> forced ring capacity
    EngineOptions engine;
};

inline BlockArgs parse_block_args(int argc, char** argv) {
    BlockArgs a;
    for (int i = 1; i < argc; ++i) {
        std::string k = argv[i];
        auto next = [&]() -> std::string { return (i + 1 < argc) ? argv[++i] : std::string(); };
        if (k == "--config") a.config_path = next();
        else if (k == "--block") a.block_name = next();
        else if (k == "--run-id") a.run_id = next();
        else if (k == "--ticks") a.engine.ticks = std::stoull(next());
        else if (k == "--rate-hz") a.engine.rate_hz = std::stod(next());
        else if (k == "--mode") a.engine.mode = next();
        else if (k == "--trace") a.engine.trace_path = next();
        else if (k == "--log") a.log_path = next();
        else if (k == "--merge-mode") a.engine.merge_mode = next();
        else if (k == "--num-blocks") a.num_blocks = static_cast<uint32_t>(std::stoul(next()));
        else if (k == "--scenario") a.scenario_path = next();
        else if (k == "--edge-cap-small") {
            // comma-separated edge names forced to an 8-slot ring instead of the default
            std::stringstream ss(next());
            std::string name;
            while (std::getline(ss, name, ',')) a.edge_cap_override[name] = 8;
        }
        else if (k == "--inject-delay-ns") a.engine.inject_delay_ns = std::stoll(next());
        else if (k == "--idle-timeout-ms") a.engine.idle_timeout_ns = std::stoll(next()) * 1'000'000LL;
    }
    return a;
}

} // namespace shmmw
