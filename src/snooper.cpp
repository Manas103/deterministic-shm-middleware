// snooper: attaches to one edge's live ring buffer read-only and prints what
// is in flight, without ever being a consumer of it. This is the piece the
// middleware did not have: `monitor` reads each block's *stats* segment
// (counters a block writes about itself); snooper reads the *data* ring
// itself, the thing carrying the actual messages between two blocks, from a
// third process that is neither the producer nor the real consumer.
//
// Two independent guarantees back "without touching producer state":
//   1. SnoopRing (snoop_ring.hpp) never calls try_pop or anything that
//      writes `tail`, so it never consumes a message the real consumer was
//      going to see.
//   2. The segment is attached with ShmSegment::attach_readonly, a PROT_READ
//      mapping, so even a bug in this file could not write into the ring;
//      the kernel enforces it. tests/test_snooper.cpp proves this by forking
//      a child that tries to write through the same mapping and asserting
//      it dies of SIGSEGV.
#include <unistd.h>

#include <cstdio>
#include <string>

#include "shmmw/graph_config.hpp"
#include "shmmw/ring_buffer.hpp"
#include "shmmw/shm_names.hpp"
#include "shmmw/shm_segment.hpp"
#include "shmmw/snoop_ring.hpp"

using namespace shmmw;

int main(int argc, char** argv) {
    std::string run_id, edge_name;
    uint32_t cap = kDefaultEdgeCapacity;
    int snapshots = 5;
    int interval_ms = 200;
    for (int i = 1; i < argc; ++i) {
        std::string k = argv[i];
        auto next = [&]() -> std::string { return (i + 1 < argc) ? argv[++i] : std::string(); };
        if (k == "--run-id") run_id = next();
        else if (k == "--edge") edge_name = next();
        else if (k == "--cap") cap = static_cast<uint32_t>(std::stoul(next()));
        else if (k == "--snapshots") snapshots = std::stoi(next());
        else if (k == "--interval-ms") interval_ms = std::stoi(next());
    }
    if (run_id.empty() || edge_name.empty()) {
        std::fprintf(stderr, "usage: snooper --run-id <id> --edge <name> [--cap N] [--snapshots N] [--interval-ms N]\n");
        return 1;
    }

    ShmSegment seg = ShmSegment::attach_readonly(shm_edge_name(run_id, edge_name), EdgeRing::size_for(cap));
    SnoopRing snoop(seg.data());

    for (int snap = 0; snap < snapshots; ++snap) {
        Message latest{};
        bool have_latest = snoop.peek_latest(&latest);
        std::printf("--- snoop %d/%d (edge %s, run %s) ---\n", snap + 1, snapshots, edge_name.c_str(), run_id.c_str());
        std::printf("head=%llu tail=%llu occupancy=%llu drops=%llu backpressure=%llu\n",
                    (unsigned long long)snoop.head(), (unsigned long long)snoop.tail(),
                    (unsigned long long)snoop.occupancy(), (unsigned long long)snoop.drop_count(),
                    (unsigned long long)snoop.backpressure_events());
        if (have_latest) {
            std::printf("latest: tick=%llu seq=%llu src_block_id=%u payload[0]=%g\n",
                        (unsigned long long)latest.tick, (unsigned long long)latest.seq,
                        latest.src_block_id, latest.payload[0]);
        } else {
            std::printf("latest: (ring empty)\n");
        }
        if (snap + 1 < snapshots) usleep(interval_ms * 1000);
    }
    return 0;
}
