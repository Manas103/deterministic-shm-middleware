#pragma once
#include <cstdint>
#include <string>
#include <vector>

#include "shmmw/graph_config.hpp"
#include "shmmw/scenario.hpp"

namespace shmmw {

struct ReferenceResult {
    std::vector<std::string> block_names;
    std::vector<std::string> block_logs;
    std::string session_digest;
};

// The deliberately slow, obviously correct oracle: one thread, no
// transport, no rings, no timestamps. For each tick it walks the blocks in
// config (topological) order, hands each kernel exactly the values its input
// edges carry for that tick, and writes the same log rows the real runtime
// writes. Any transport whose session digest differs from this one has
// changed *what* a block computed, not merely *when*.
//
// trace_path, when non-empty, supplies the source payloads (replay); when
// empty the source kernel is asked to produce them with zero jitter. Throws
// std::runtime_error if the graph is not topologically ordered.
ReferenceResult run_reference(const GraphConfig& g, const Scenario* scenario, uint64_t ticks,
                              const std::string& trace_path = "");

} // namespace shmmw
