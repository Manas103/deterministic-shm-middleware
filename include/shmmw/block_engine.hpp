#pragma once
#include <cstdint>
#include <ostream>
#include <string>

#include "shmmw/graph_config.hpp"
#include "shmmw/kernels.hpp"
#include "shmmw/stats.hpp"
#include "shmmw/transport.hpp"

namespace shmmw {

struct EngineOptions {
    uint64_t ticks = 0;
    double rate_hz = 200.0;              // source pacing in record mode; <= 0 means unpaced
    std::string mode = "record";         // record|replay, source blocks only
    std::string trace_path;              // source: written in record mode, read in replay mode
    std::string merge_mode = "tick_keyed"; // tick_keyed|arrival_order, merge blocks only
    int64_t inject_delay_ns = 0;         // CI latency-regression harness only
    int64_t idle_timeout_ns = 8'000'000'000LL; // give up after this long with no input
};

struct BlockIo {
    std::ostream* log = nullptr;     // hashed into the session digest (session_log.hpp)
    std::ostream* hop_log = nullptr; // wall-clock hop latency, never hashed
    BlockStats* stats = nullptr;     // live counters read by `monitor`
};

// Runs one block for one session: the loop that used to live in
// block_runner.cpp, now independent of how messages move. Returns 0 when
// the block processed every tick, 2 when it gave up on the idle timeout.
//
// Everything that reaches the digest (tick, seq, src_block_id, payload) is
// decided here and in the kernel; everything the transport decides
// (arrival interleaving across edges, wire format, timing) is kept out of it.
int run_block(const GraphConfig& g, const BlockConfig& bc, Kernel& kernel, ITransport& transport,
              const BlockIo& io, const EngineOptions& opt);

} // namespace shmmw
