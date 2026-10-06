#pragma once
#include <string>

namespace shmmw {

// POSIX shared-memory segment names for one session. One definition, used
// by graph_launcher (creator), both block executables and monitor
// (attachers), so the names can never drift apart.
inline std::string shm_edge_name(const std::string& run_id, const std::string& edge) {
    return "/shmmw_" + run_id + "_e_" + edge;
}
inline std::string shm_stats_name(const std::string& run_id, const std::string& block) {
    return "/shmmw_" + run_id + "_s_" + block;
}
inline std::string shm_barrier_name(const std::string& run_id) { return "/shmmw_" + run_id + "_barrier"; }

} // namespace shmmw
