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

#include <algorithm>
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

    // Inter-host bridge (see block_args.hpp, mixed_transport.hpp): this
    // invocation only forks the blocks named in only_blocks (empty means
    // every block in the config, the single-host default), and never
    // creates a shm segment for bridge_edge, which crosses over UDP.
    std::string only_blocks;
    std::string bridge_edge;
    std::string bridge_role;
    std::string bridge_peer_ip;
    std::string bridge_peer_port;
    std::string bridge_listen_port;
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
        else if (k == "--only-blocks") o.only_blocks = next();
        else if (k == "--bridge-edge") o.bridge_edge = next();
        else if (k == "--bridge-role") o.bridge_role = next();
        else if (k == "--bridge-peer-ip") o.bridge_peer_ip = next();
        else if (k == "--bridge-peer-port") o.bridge_peer_port = next();
        else if (k == "--bridge-listen-port") o.bridge_listen_port = next();
    }
    return o;
}

std::vector<std::string> split_csv(const std::string& s) {
    std::vector<std::string> out;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, ',')) if (!item.empty()) out.push_back(item);
    return out;
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

    // This invocation's blocks: every block in the config (single-host,
    // the default), or only the named subset (--only-blocks, one host's
    // share of a graph split across a bridge edge).
    std::vector<BlockConfig> host_blocks = g.blocks;
    if (!o.only_blocks.empty()) {
        auto wanted = split_csv(o.only_blocks);
        host_blocks.clear();
        for (auto& b : g.blocks) {
            if (std::find(wanted.begin(), wanted.end(), b.name) != wanted.end()) host_blocks.push_back(b);
        }
    }

    // 1. Create every segment before any block process exists. The bridge
    // edge (if any) never gets a shm segment on either host: it crosses
    // over UDP instead (see block_runner.cpp, mixed_transport.hpp).
    std::vector<ShmSegment> edge_segs, stats_segs;
    if (o.transport == "shm") {
        for (auto& e : g.edges) {
            if (!o.bridge_edge.empty() && e.name == o.bridge_edge) continue;
            bool local = o.only_blocks.empty();
            if (!local) {
                for (auto& b : host_blocks) {
                    if (std::find(b.input_edges.begin(), b.input_edges.end(), e.name) != b.input_edges.end() ||
                        std::find(b.output_edges.begin(), b.output_edges.end(), e.name) != b.output_edges.end()) {
                        local = true;
                    }
                }
            }
            if (!local) continue; // belongs entirely to the other host's subset
            uint32_t cap = small.count(e.name) ? small[e.name] : kDefaultEdgeCapacity;
            ShmSegment seg = ShmSegment::create(shm_edge_name(o.run_id, e.name), EdgeRing::size_for(cap));
            EdgeRing::init(seg.data(), cap);
            edge_segs.push_back(std::move(seg));
        }
    }
    for (auto& b : host_blocks) {
        ShmSegment seg = ShmSegment::create(shm_stats_name(o.run_id, b.name), sizeof(BlockStats));
        new (seg.data()) BlockStats();
        stats_segs.push_back(std::move(seg));
    }
    ShmSegment barrier_seg = ShmSegment::create(shm_barrier_name(o.run_id), sizeof(StartBarrier));
    new (barrier_seg.data()) StartBarrier();

    // 2. Fork+exec one block process per block this host runs.
    std::vector<pid_t> pids;
    std::vector<std::string> log_paths;
    for (auto& b : host_blocks) {
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
                "--num-blocks", std::to_string(host_blocks.size()),
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
            if (!o.bridge_edge.empty()) {
                args.push_back("--bridge-edge"); args.push_back(o.bridge_edge);
                args.push_back("--bridge-role"); args.push_back(o.bridge_role);
                if (!o.bridge_peer_ip.empty()) { args.push_back("--bridge-peer-ip"); args.push_back(o.bridge_peer_ip); }
                if (!o.bridge_peer_port.empty()) { args.push_back("--bridge-peer-port"); args.push_back(o.bridge_peer_port); }
                if (!o.bridge_listen_port.empty()) { args.push_back("--bridge-listen-port"); args.push_back(o.bridge_listen_port); }
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
    // which order the child processes happened to exit. When --only-blocks
    // split the graph across a bridge, this is this host's partial digest
    // over its own blocks only; tools/combine_digest.cpp combines both
    // hosts' partial logs into the one digest a single-host run would have
    // produced directly.
    std::vector<std::string> names, logs;
    for (size_t i = 0; i < host_blocks.size(); ++i) {
        names.push_back(host_blocks[i].name);
        logs.push_back(read_file(log_paths[i]));
    }
    const std::string digest = session_digest(names, logs);

    std::cout << "{\"run_id\":\"" << o.run_id << "\",\"transport\":\"" << o.transport
              << "\",\"session_digest\":\"" << digest
              << "\",\"exit_ok\":" << (worst_status == 0 ? "true" : "false") << "}\n";

    return worst_status == 0 ? 0 : 1;
}
