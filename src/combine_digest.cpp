// Combines two hosts' partial session digests (each produced by a
// --only-blocks graph_launcher invocation) into the one digest a
// single-host run of the same config would have produced: reads every
// block's log file in full-config order, picking it up from whichever
// host's log directory actually ran that block, and hashes with the exact
// same session_digest() graph_launcher itself uses. This is the only way
// to check "replay stays bit-exact across the bridge" against the base
// repo's own single-host digest for the identical config and trace.
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "shmmw/graph_config.hpp"
#include "shmmw/session_log.hpp"

using namespace shmmw;

namespace {

std::string read_file(const std::string& path) {
    std::ifstream f(path);
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

std::vector<std::string> split_csv(const std::string& s) {
    std::vector<std::string> out;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, ',')) if (!item.empty()) out.push_back(item);
    return out;
}

bool contains(const std::vector<std::string>& v, const std::string& s) {
    for (auto& x : v) if (x == s) return true;
    return false;
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 8) {
        std::fprintf(stderr,
                      "usage: combine_digest <config> <host_a_blocks_csv> <host_a_run_id> <host_a_log_dir> "
                      "<host_b_blocks_csv> <host_b_run_id> <host_b_log_dir>\n");
        return 1;
    }
    GraphConfig g = load_graph_config(argv[1]);
    std::vector<std::string> a_blocks = split_csv(argv[2]);
    std::string a_run_id = argv[3], a_log_dir = argv[4];
    std::vector<std::string> b_blocks = split_csv(argv[5]);
    std::string b_run_id = argv[6], b_log_dir = argv[7];

    std::vector<std::string> names, logs;
    for (auto& b : g.blocks) {
        names.push_back(b.name);
        if (contains(a_blocks, b.name)) {
            logs.push_back(read_file(a_log_dir + "/" + a_run_id + "_" + b.name + ".log"));
        } else if (contains(b_blocks, b.name)) {
            logs.push_back(read_file(b_log_dir + "/" + b_run_id + "_" + b.name + ".log"));
        } else {
            std::fprintf(stderr, "block %s not assigned to either host\n", b.name.c_str());
            return 1;
        }
    }
    std::printf("%s\n", session_digest(names, logs).c_str());
    return 0;
}
