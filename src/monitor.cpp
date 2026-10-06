// monitor: attaches to every block's stats segment for a running (or just
// finished) session, purely as a reader, and prints a live table. This is
// the "exported live" half of the backpressure/drop/latency claim: it is a
// separate process from every block, added after the fact, that needed zero
// changes to block_runner.cpp because the counters already live in shared
// memory rather than inside any one block's private state.
#include <unistd.h>

#include <cstdio>
#include <string>
#include <vector>

#include "shmmw/graph_config.hpp"
#include "shmmw/shm_names.hpp"
#include "shmmw/shm_segment.hpp"
#include "shmmw/stats.hpp"

using namespace shmmw;

int main(int argc, char** argv) {
    std::string config_path, run_id;
    int snapshots = 5;
    int interval_ms = 200;
    for (int i = 1; i < argc; ++i) {
        std::string k = argv[i];
        auto next = [&]() -> std::string { return (i + 1 < argc) ? argv[++i] : std::string(); };
        if (k == "--config") config_path = next();
        else if (k == "--run-id") run_id = next();
        else if (k == "--snapshots") snapshots = std::stoi(next());
        else if (k == "--interval-ms") interval_ms = std::stoi(next());
    }
    GraphConfig g = load_graph_config(config_path);

    std::vector<ShmSegment> segs;
    std::vector<BlockStats*> stats;
    for (auto& b : g.blocks) {
        ShmSegment seg = ShmSegment::attach(shm_stats_name(run_id, b.name), sizeof(BlockStats));
        BlockStats* st = reinterpret_cast<BlockStats*>(seg.data());
        segs.push_back(std::move(seg));
        stats.push_back(st);
    }

    for (int snap = 0; snap < snapshots; ++snap) {
        std::printf("--- snapshot %d/%d (run %s) ---\n", snap + 1, snapshots, run_id.c_str());
        std::printf("%-6s %10s %10s %8s %8s %12s %12s\n",
                     "block", "msgs_in", "msgs_out", "drops", "bpress", "last_hop_ns", "ewma_hop_ns");
        for (size_t i = 0; i < g.blocks.size(); ++i) {
            BlockStats* st = stats[i];
            std::printf("%-6s %10llu %10llu %8llu %8llu %12lld %12lld\n",
                        g.blocks[i].name.c_str(),
                        (unsigned long long)st->msgs_in.load(),
                        (unsigned long long)st->msgs_out.load(),
                        (unsigned long long)st->drops_total.load(),
                        (unsigned long long)st->backpressure_total.load(),
                        (long long)st->last_hop_latency_ns.load(),
                        (long long)st->ewma_hop_latency_ns.load());
        }
        if (snap + 1 < snapshots) usleep(interval_ms * 1000);
    }
    return 0;
}
