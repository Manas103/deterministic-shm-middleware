#pragma once
#include <atomic>
#include <cstdint>

namespace shmmw {

// One of these lives in its own shared-memory segment per block, updated by
// the block process on every message and readable by any other process
// (chiefly the `monitor` tool) without going through the block at all. This
// is the "live" in "per-block latency, backpressure and drop counters
// exported live": a separate process can watch a block's health while the
// graph is running, without adding a single line of IPC code to the block
// logic itself, because the counters already live in shared memory.
struct BlockStats {
    std::atomic<uint64_t> msgs_in{0};
    std::atomic<uint64_t> msgs_out{0};
    std::atomic<uint64_t> drops_total{0};        // summed from all input edges
    std::atomic<uint64_t> backpressure_total{0}; // summed from all input edges
    std::atomic<int64_t> last_hop_latency_ns{0};
    std::atomic<int64_t> ewma_hop_latency_ns{0}; // exponential moving average, alpha = 1/8
    std::atomic<uint64_t> alive{0}; // heartbeat, incremented once per tick processed
};

// A start barrier shared by every block process in a session. fork()+exec()
// gives no guarantee about when each child finishes attaching its shared
// memory, and a source that starts emitting before every downstream block
// has attached will fill a 64-slot edge buffer and start dropping before
// anyone is listening. Every block increments ready_count once it has
// attached everything it needs, then spins until ready_count reaches the
// total block count, so tick 0 is never emitted until every consumer for it
// already exists. See README "Findings": replay's unpaced source made this
// race visible where the paced 200 Hz live source almost never did.
struct StartBarrier {
    std::atomic<uint32_t> ready_count{0};
};

inline void update_ewma(std::atomic<int64_t>& ewma, int64_t sample_ns) {
    int64_t prev = ewma.load(std::memory_order_relaxed);
    int64_t next = prev == 0 ? sample_ns : prev + (sample_ns - prev) / 8;
    ewma.store(next, std::memory_order_relaxed);
}

} // namespace shmmw
