// Hand-rolled check suite for the transport-agnostic block engine, run with
// every block of a 12-block graph on its own thread, connected by the real
// EdgeRing/ShmTransport code over heap memory instead of POSIX shared
// memory. That puts the engine, the kernels and the rings under genuine
// concurrency inside one process, which is what lets ThreadSanitizer see the
// path that block_runner normally runs across process boundaries.
//
// Every concurrent run is diffed against the sequential reference oracle
// (reference.cpp): same digest, block by block, regardless of how the OS
// interleaved twelve threads. Two negative controls prove the comparison has
// teeth: a one-ulp scenario change and a lossy undersized ring must both
// change the digest.
#include <unistd.h>

#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "shmmw/block_engine.hpp"
#include "shmmw/graph_config.hpp"
#include "shmmw/kernels.hpp"
#include "shmmw/reference.hpp"
#include "shmmw/ring_buffer.hpp"
#include "shmmw/scenario.hpp"
#include "shmmw/session_log.hpp"
#include "shmmw/shm_transport.hpp"
#include "shmmw/stats.hpp"

using namespace shmmw;

static int g_checks = 0;
#define CHECK(cond) do { \
    ++g_checks; \
    if (!(cond)) { std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); std::exit(1); } \
} while (0)

static const std::string kSrc = SHMMW_SOURCE_DIR;

struct ThreadedRun {
    std::string digest;
    std::vector<std::string> logs;
    std::vector<int> status;
    std::vector<std::unique_ptr<BlockStats>> stats;
    std::map<std::string, uint64_t> edge_drops;
};

// One session: one thread per block, one heap EdgeRing per edge.
ThreadedRun run_threaded(const GraphConfig& g, const Scenario* scn, uint64_t ticks, const std::string& mode,
                         const std::string& trace, const std::map<std::string, uint32_t>& caps = {},
                         int64_t idle_timeout_ns = 2'000'000'000LL) {
    std::map<std::string, std::vector<uint64_t>> ring_mem; // uint64_t storage keeps the header 8-byte aligned
    for (const auto& e : g.edges) {
        const uint32_t cap = caps.count(e.name) ? caps.at(e.name) : kDefaultEdgeCapacity;
        ring_mem[e.name].assign(EdgeRing::size_for(cap) / 8 + 1, 0);
        EdgeRing::init(ring_mem[e.name].data(), cap);
    }
    const size_t n = g.blocks.size();
    ThreadedRun r;
    r.status.assign(n, -1);
    std::vector<std::ostringstream> logs(n), hops(n);
    std::vector<std::unique_ptr<Kernel>> kernels;
    std::vector<std::unique_ptr<ShmTransport>> transports;
    for (size_t i = 0; i < n; ++i) {
        const BlockConfig& b = g.blocks[i];
        std::vector<void*> in, out;
        for (const auto& e : b.input_edges) in.push_back(ring_mem[e].data());
        for (const auto& e : b.output_edges) out.push_back(ring_mem[e].data());
        transports.push_back(std::make_unique<ShmTransport>(in, out));
        kernels.push_back(make_kernel(b, scn));
        r.stats.push_back(std::make_unique<BlockStats>());
    }
    EngineOptions opt;
    opt.ticks = ticks;
    opt.rate_hz = 0; // unpaced
    opt.mode = mode;
    opt.trace_path = trace;
    opt.idle_timeout_ns = idle_timeout_ns;

    std::vector<std::thread> threads;
    for (size_t i = 0; i < n; ++i) {
        threads.emplace_back([&, i] {
            BlockIo io{&logs[i], &hops[i], r.stats[i].get()};
            r.status[i] = run_block(g, g.blocks[i], *kernels[i], *transports[i], io, opt);
        });
    }
    for (auto& t : threads) t.join();

    std::vector<std::string> names;
    for (size_t i = 0; i < n; ++i) { names.push_back(g.blocks[i].name); r.logs.push_back(logs[i].str()); }
    r.digest = session_digest(names, r.logs);
    for (const auto& e : g.edges) {
        EdgeRing ring(ring_mem[e.name].data());
        r.edge_drops[e.name] = ring.drop_count().load();
    }
    return r;
}

// Every block ran to completion and the per-block counters obey the flow
// identities of a lossless session.
void check_clean_conservation(const GraphConfig& g, const ThreadedRun& r, uint64_t ticks) {
    for (size_t i = 0; i < g.blocks.size(); ++i) {
        const BlockConfig& b = g.blocks[i];
        CHECK(r.status[i] == 0);
        CHECK(r.stats[i]->msgs_in.load() == ticks * b.input_edges.size());
        CHECK(r.stats[i]->msgs_out.load() == ticks * b.output_edges.size());
        CHECK(r.stats[i]->drops_total.load() == 0);
    }
    for (const auto& kv : r.edge_drops) CHECK(kv.second == 0);
}

void test_av_threaded_matches_reference(const std::string& scenario_file, uint64_t ticks, int repeats) {
    const GraphConfig g = load_graph_config(kSrc + "/configs/av_12block.cfg");
    const Scenario scn = load_scenario(kSrc + "/scenarios/" + scenario_file);
    const ReferenceResult ref = run_reference(g, &scn, ticks);
    for (int rep = 0; rep < repeats; ++rep) {
        const ThreadedRun r = run_threaded(g, &scn, ticks, "record", "");
        check_clean_conservation(g, r, ticks);
        for (size_t i = 0; i < g.blocks.size(); ++i) CHECK(r.logs[i] == ref.block_logs[i]); // block by block
        CHECK(r.digest == ref.session_digest);
    }
    std::printf("  %s: %d threaded sessions == reference %s\n", scenario_file.c_str(), repeats,
                ref.session_digest.substr(0, 16).c_str());
}

void test_generic_replay_matches_reference() {
    // The original sine graph in replay mode, from a synthetic trace that
    // includes values with long decimal expansions and a nonzero jitter column.
    const GraphConfig g = load_graph_config(kSrc + "/configs/graph_12block.cfg");
    char path[] = "/tmp/shmmw_test_trace_XXXXXX";
    const int fd = mkstemp(path);
    CHECK(fd >= 0);
    close(fd);
    {
        std::ofstream t(path);
        t.precision(17);
        for (int k = 0; k < 300; ++k) {
            t << k << ',' << std::sin(k * 0.01) * 100.0 << ',' << std::cos(k * 0.017) * 50.0 << ',' << k << ','
              << (k * 7919) % 50021 << '\n';
        }
    }
    const ReferenceResult ref = run_reference(g, nullptr, 300, path);
    for (int rep = 0; rep < 3; ++rep) {
        const ThreadedRun r = run_threaded(g, nullptr, 300, "replay", path);
        check_clean_conservation(g, r, 300);
        CHECK(r.digest == ref.session_digest);
    }
    std::remove(path);
    std::printf("  graph_12block replay: 3 threaded sessions == reference %s\n", ref.session_digest.substr(0, 16).c_str());
}

void test_digest_detects_one_ulp_scenario_change() {
    const GraphConfig g = load_graph_config(kSrc + "/configs/av_12block.cfg");
    Scenario scn = load_scenario(kSrc + "/scenarios/highway_cut_in.scn");
    const std::string base = run_reference(g, &scn, 400).session_digest;
    Waypoint& w = scn.agents[1].waypoints[1];
    w.x = std::nextafter(w.x, 1e9);
    const std::string nudged = run_reference(g, &scn, 400).session_digest;
    CHECK(base != nudged);
}

void test_lossy_edge_changes_digest_and_is_accounted() {
    // An 8-slot ring on the ego's input with an unpaced source: the source
    // pushes faster than the ego drains, so the ring drops. The digest must
    // differ from the reference, and every message the source sent on that
    // edge is either consumed by ego or counted as a drop, exactly once.
    const GraphConfig g = load_graph_config(kSrc + "/configs/av_12block.cfg");
    const Scenario scn = load_scenario(kSrc + "/scenarios/highway_cut_in.scn");
    const uint64_t ticks = 600;
    const std::string ref = run_reference(g, &scn, ticks).session_digest;
    // Run until a drop actually occurs; a fast enough consumer can in
    // principle keep up with an 8-slot ring for a whole session.
    for (int attempt = 0; attempt < 20; ++attempt) {
        const ThreadedRun r = run_threaded(g, &scn, ticks, "record", "", {{"scn_ego", 8}}, 200'000'000LL);
        const uint64_t drops = r.edge_drops.at("scn_ego");
        const uint64_t ego_in = r.stats[1]->msgs_in.load();
        CHECK(ego_in + drops == ticks); // conservation on the lossy edge
        if (drops == 0) continue;
        CHECK(r.digest != ref);
        CHECK(r.status[1] == 2);        // ego gave up waiting for ticks that never came
        std::printf("  lossy edge: %llu drops on scn_ego, digest differs from reference (attempt %d)\n",
                    (unsigned long long)drops, attempt + 1);
        return;
    }
    std::printf("  lossy edge: no drop in 20 attempts (consumer kept up); negative control inconclusive\n");
}

int main(int argc, char** argv) {
    const int repeats = argc > 1 ? std::atoi(argv[1]) : 5;
    test_av_threaded_matches_reference("highway_cut_in.scn", 600, repeats);
    test_av_threaded_matches_reference("urban_crossing.scn", 600, repeats);
    test_generic_replay_matches_reference();
    test_digest_detects_one_ulp_scenario_change();
    test_lossy_edge_changes_digest_and_is_accounted();
    std::printf("check_engine: %d checks passed\n", g_checks);
    return 0;
}
