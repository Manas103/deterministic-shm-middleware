// graph_launcher: the only process that creates shared memory. It reads the
// static graph config, creates one ring-buffer segment per edge and one
// stats segment per block, forks and execs one block_runner process per
// block, waits for all of them, then aggregates every block's message log
// into a single SHA-256 session digest and prints a JSON summary to stdout.
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
#include "shmmw/sha256.hpp"
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
    }
    return o;
}

std::string shm_edge_name(const std::string& run_id, const std::string& edge) {
    return "/shmmw_" + run_id + "_e_" + edge;
}
std::string shm_stats_name(const std::string& run_id, const std::string& block) {
    return "/shmmw_" + run_id + "_s_" + block;
}
std::string shm_barrier_name(const std::string& run_id) {
    return "/shmmw_" + run_id + "_barrier";
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
    GraphConfig g = load_graph_config(o.config_path);
    auto small = parse_small_edges(o.small_edges);

    // 1. Create every edge and stats segment before any block process exists.
    std::vector<ShmSegment> edge_segs, stats_segs;
    for (auto& e : g.edges) {
        uint32_t cap = small.count(e.name) ? small[e.name] : kDefaultEdgeCapacity;
        ShmSegment seg = ShmSegment::create(shm_edge_name(o.run_id, e.name), EdgeRing::size_for(cap));
        EdgeRing::init(seg.data(), cap);
        edge_segs.push_back(std::move(seg));
    }
    for (auto& b : g.blocks) {
        ShmSegment seg = ShmSegment::create(shm_stats_name(o.run_id, b.name), sizeof(BlockStats));
        new (seg.data()) BlockStats();
        stats_segs.push_back(std::move(seg));
    }
    ShmSegment barrier_seg = ShmSegment::create(shm_barrier_name(o.run_id), sizeof(StartBarrier));
    new (barrier_seg.data()) StartBarrier();

    // 2. Fork+exec one block_runner per block.
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

    // 4. Aggregate: hash each block's own log, then hash the concatenation
    // of those hex digests in a fixed (block id) order, so the final digest
    // does not depend on which order the child processes happened to exit.
    std::ostringstream combined;
    for (size_t i = 0; i < g.blocks.size(); ++i) {
        std::string content = read_file(log_paths[i]);
        combined << g.blocks[i].name << ':' << Sha256::hash_string(content) << ';';
    }
    std::string session_digest = Sha256::hash_string(combined.str());

    std::cout << "{\"run_id\":\"" << o.run_id << "\",\"session_digest\":\"" << session_digest
              << "\",\"exit_ok\":" << (worst_status == 0 ? "true" : "false") << "}\n";

    return worst_status == 0 ? 0 : 1;
}
