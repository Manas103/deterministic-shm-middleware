// sim_reference: command-line front end to the sequential reference oracle
// (reference.cpp). Prints the session digest a correct run of the graph must
// produce, as one JSON line, so the digest tools can compare every live
// session, on either transport, against it.
//
//   sim_reference --config configs/av_12block.cfg --scenario scenarios/cut_in.scn --ticks 400
//   sim_reference --config configs/graph_12block.cfg --trace /tmp/trace.csv --ticks 200
//   add --dump-dir DIR to write each block's reference log for line-by-line diffing.
#include <cstdio>
#include <exception>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>

#include "shmmw/graph_config.hpp"
#include "shmmw/kernels.hpp"
#include "shmmw/reference.hpp"
#include "shmmw/scenario.hpp"

using namespace shmmw;

int main(int argc, char** argv) {
    std::string config, scenario_path, trace, dump_dir;
    uint64_t ticks = 0;
    for (int i = 1; i < argc; ++i) {
        std::string k = argv[i];
        auto next = [&]() -> std::string { return (i + 1 < argc) ? argv[++i] : std::string(); };
        if (k == "--config") config = next();
        else if (k == "--scenario") scenario_path = next();
        else if (k == "--trace") trace = next();
        else if (k == "--ticks") ticks = std::stoull(next());
        else if (k == "--dump-dir") dump_dir = next();
    }
    try {
        GraphConfig g = load_graph_config(config);
        std::unique_ptr<Scenario> scn;
        if (!scenario_path.empty()) scn = std::make_unique<Scenario>(load_scenario(scenario_path));
        ReferenceResult r = run_reference(g, scn.get(), ticks, trace);
        if (!dump_dir.empty()) {
            for (size_t i = 0; i < r.block_names.size(); ++i) {
                std::ofstream(dump_dir + "/reference_" + r.block_names[i] + ".log") << r.block_logs[i];
            }
        }
        std::cout << "{\"reference_digest\":\"" << r.session_digest << "\",\"ticks\":" << ticks
                  << ",\"blocks\":" << r.block_names.size() << "}\n";
    } catch (const std::exception& e) {
        std::fprintf(stderr, "sim_reference: %s\n", e.what());
        return 1;
    }
    return 0;
}
