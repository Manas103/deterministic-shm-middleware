// graph_launcher: the only process that creates shared memory. It reads the
// static graph config, creates one stats segment per block and the start
// barrier, plus (shm transport only) one ring-buffer segment per edge, then
// forks and execs one block process per block, waits for all of them, and
// aggregates every block's message log into a single SHA-256 session digest
// printed as a JSON line on stdout.
//
// --transport shm   (default) execs block_runner; edges are shm rings.
// --transport ros2  execs ros2_block_node; edges are ROS 2 topics. The stats
//                   segments and start barrier are still shared memory (they
//                   are session control and live introspection, not the data
//                   path), so `monitor` works unchanged against either.
#include <sys/wait.h>
#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <new>
#include <sstream>
#include <string>
#include <vector>

#include "shmmw/graph_config.hpp"
#include "shmmw/ring_buffer.hpp"
#include "shmmw/session_log.hpp"
#include "shmmw/shm_names.hpp"
#include "shmmw/shm_segment.hpp"
#include "shmmw/stats.hpp"

using namespace shmmw;

namespace {

struct Options {
    std::string config_path;
    std::string run_id;
    uint64_t ticks = 500;
    double rate_hz = 200.0;
    std::string mode = "record";
    std::string trace_path;
    std::string log_dir = "/tmp";
    std::string merge_mode = "tick_keyed";
    std::string small_edges; // comma-separated, forwarded verbatim to every block_runner
    std::string block_runner_path = "./block_runner";
    std::string regress_block;    // name of the one block to slow down, for the CI latency-regression harness
    int64_t regress_delay_ns = 0; // how much, applied only to regress_block
    std::string transport = "shm";
    std::string scenario_path;
    std::string idle_timeout_ms;
};

Options parse(int argc, char** argv) {
    Options o;
    for (int i = 1; i < argc; ++i) {
        std::string k = argv[i];
        auto next = [&]() -> std::string { return (i + 1 < argc) ? argv[++i] : std::string(); };
        if (k == "--config") o.config_path = next();
        else if (k == "--run-id") o.run_id = next();
        else if (k == "--ticks") o.ticks = std::stoull(next());
        else if (k == "--rate-hz") o.rate_hz = std::stod(next());
        else if (k == "--mode") o.mode = next();
        else if (k == "--trace") o.trace_path = next();
        else if (k == "--log-dir") o.log_dir = next();
        else if (k == "--merge-mode") o.merge_mode = next();
        else if (k == "--small-edges") o.small_edges = next();
        else if (k == "--block-runner") o.block_runner_path = next();
        else if (k == "--regress-block") o.regress_block = next();
        else if (k == "--regress-delay-ns") o.regress_delay_ns = std::stoll(next());
        else if (k == "--transport") o.transport = next();
        else if (k == "--scenario") o.scenario_path = next();
        else if (k == "--idle-timeout-ms") o.idle_timeout_ms = next();
    }
    return o;
}

std::map<std::string, uint32_t> parse_small_edges(const std::string& s) {
    std::map<std::string, uint32_t> m;
    std::stringstream ss(s);
    std::string name;
    while (std::getline(ss, name, ',')) if (!name.empty()) m[name] = 8;
    return m;
}

std::string read_file(const std::string& path) {
    std::ifstream f(path);
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

} // namespace

int main(int argc, char** argv) {
    Options o = parse(argc, argv);
    if (o.transport != "shm" && o.transport != "ros2") {
        std::fprintf(stderr, "unknown --transport %s (shm|ros2)\n", o.transport.c_str());
        return 1;
    }
    GraphConfig g = load_graph_config(o.config_path);
    auto small = parse_small_edges(o.small_edges);

    // 1. Create every segment before any block process exists.
    std::vector<ShmSegment> edge_segs, stats_segs;
    if (o.transport == "shm") {
        for (auto& e : g.edges) {
            uint32_t cap = small.count(e.name) ? small[e.name] : kDefaultEdgeCapacity;
            ShmSegment seg = ShmSegment::create(shm_edge_name(o.run_id, e.name), EdgeRing::size_for(cap));
            EdgeRing::init(seg.data(), cap);
            edge_segs.push_back(std::move(seg));
        }
    }
    for (auto& b : g.blocks) {
        ShmSegment seg = ShmSegment::create(shm_stats_name(o.run_id, b.name), sizeof(BlockStats));
        new (seg.data()) BlockStats();
        stats_segs.push_back(std::move(seg));
    }
    ShmSegment barrier_seg = ShmSegment::create(shm_barrier_name(o.run_id), sizeof(StartBarrier));
    new (barrier_seg.data()) StartBarrier();

    // 2. Fork+exec one block process per block.
    std::vector<pid_t> pids;
    std::vector<std::string> log_paths;
    for (auto& b : g.blocks) {
        std::string log_path = o.log_dir + "/" + o.run_id + "_" + b.name + ".log";
        log_paths.push_back(log_path);

        pid_t pid = fork();
        if (pid < 0) { perror("fork"); return 1; }
        if (pid == 0) {
            std::vector<std::string> args = {
                o.block_runner_path,
                "--config", o.config_path,
                "--block", b.name,
                "--run-id", o.run_id,
                "--ticks", std::to_string(o.ticks),
                "--rate-hz", std::to_string(o.rate_hz),
                "--log", log_path,
                "--merge-mode", o.merge_mode,
                "--num-blocks", std::to_string(g.blocks.size()),
            };
            if (b.type == BlockType::Source) {
                args.push_back("--mode"); args.push_back(o.mode);
                args.push_back("--trace"); args.push_back(o.trace_path);
            }
            if (!o.scenario_path.empty()) { args.push_back("--scenario"); args.push_back(o.scenario_path); }
            if (!o.idle_timeout_ms.empty()) { args.push_back("--idle-timeout-ms"); args.push_back(o.idle_timeout_ms); }
            if (!o.small_edges.empty()) { args.push_back("--edge-cap-small"); args.push_back(o.small_edges); }
            if (!o.regress_block.empty() && b.name == o.regress_block) {
                args.push_back("--inject-delay-ns"); args.push_back(std::to_string(o.regress_delay_ns));
            }

            std::vector<char*> cargs;
            for (auto& s : args) cargs.push_back(const_cast<char*>(s.c_str()));
            cargs.push_back(nullptr);
            execv(o.block_runner_path.c_str(), cargs.data());
            perror("execv");
            _exit(127);
        }
        pids.push_back(pid);
    }

    // 3. Wait for every block process to finish.
    int worst_status = 0;
    for (pid_t pid : pids) {
        int status = 0;
        waitpid(pid, &status, 0);
        if (status != 0) worst_status = status;
    }

    // 4. Aggregate in fixed (config) order, so the digest does not depend on
    // which order the child processes happened to exit.
    std::vector<std::string> names, logs;
    for (size_t i = 0; i < g.blocks.size(); ++i) {
        names.push_back(g.blocks[i].name);
        logs.push_back(read_file(log_paths[i]));
    }
    const std::string digest = session_digest(names, logs);

    std::cout << "{\"run_id\":\"" << o.run_id << "\",\"transport\":\"" << o.transport
              << "\",\"session_digest\":\"" << digest
              << "\",\"exit_ok\":" << (worst_status == 0 ? "true" : "false") << "}\n";

    return worst_status == 0 ? 0 : 1;
}
