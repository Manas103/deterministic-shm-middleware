#pragma once
#include <cstdint>
#include <cstring>

namespace shmmw {

// Fixed-layout typed message. Every edge in the graph carries exactly this
// struct; there is no dynamic schema, which is what "typed messages" means
// here: the compiler, not a runtime negotiation, fixes the wire format.
struct Message {
    uint64_t tick;         // logical tick index, source of all determinism
    uint64_t seq;          // monotonic sequence number assigned by the source
    uint32_t src_block_id; // numeric id of the block that produced this
    uint32_t edge_id;      // numeric id of the edge (src,dst) pair, for stats
    int64_t send_ts_ns;    // CLOCK_MONOTONIC timestamp at enqueue
    double payload[4];     // deterministic function of (tick, src_block_id)
};

static_assert(sizeof(Message) == 64, "Message layout changed; bump edge ABI version");

} // namespace shmmw
