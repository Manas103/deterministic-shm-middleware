#pragma once
#include <ostream>
#include <sstream>
#include <string>
#include <vector>

#include "shmmw/message.hpp"
#include "shmmw/sha256.hpp"

namespace shmmw {

// The one definition of what a session digest covers, shared by
// graph_launcher (both transports) and the sequential reference oracle.
//
// A log row is the logical content of a message: tick, seq, producing block
// id and the four payload doubles at 17 significant digits (enough to
// round-trip any double exactly). It deliberately excludes send_ts_ns
// (wall clock) and edge_id (a routing detail), and it is written from the
// in-memory Message, never from transport wire bytes, so a ROS 2 CDR frame
// and a shared-memory slot that carry the same values hash identically.
inline void write_log_row(std::ostream& log, const Message& m) {
    log << m.tick << ',' << m.seq << ',' << m.src_block_id << ',' << m.payload[0] << ','
        << m.payload[1] << ',' << m.payload[2] << ',' << m.payload[3] << '\n';
}

// Hash each block's log, then hash "name:hex;" for every block in config
// order, so the result is independent of which order processes exited.
inline std::string session_digest(const std::vector<std::string>& block_names,
                                  const std::vector<std::string>& block_logs) {
    std::ostringstream combined;
    for (size_t i = 0; i < block_names.size(); ++i) {
        combined << block_names[i] << ':' << Sha256::hash_string(block_logs[i]) << ';';
    }
    return Sha256::hash_string(combined.str());
}

} // namespace shmmw
