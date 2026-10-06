#include "shmmw/reference.hpp"

#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>

#include "shmmw/kernels.hpp"
#include "shmmw/session_log.hpp"

namespace shmmw {

ReferenceResult run_reference(const GraphConfig& g, const Scenario* scenario, uint64_t ticks,
                              const std::string& trace_path) {
    std::string why;
    if (!is_topologically_ordered(g, &why)) throw std::runtime_error("reference oracle: " + why);

    const size_t n = g.blocks.size();
    std::vector<std::unique_ptr<Kernel>> kernels;
    std::vector<std::ostringstream> logs(n);
    std::vector<uint64_t> seq(n, 0);
    for (size_t i = 0; i < n; ++i) {
        kernels.push_back(make_kernel(g.blocks[i], scenario));
        logs[i].precision(17);
    }

    std::ifstream trace;
    if (!trace_path.empty()) {
        trace.open(trace_path);
        if (!trace) throw std::runtime_error("reference oracle: cannot open trace " + trace_path);
    }

    bool trace_ended = false;
    for (uint64_t tick = 0; tick < ticks && !trace_ended; ++tick) {
        std::map<std::string, Message> edge; // value each edge carries on this tick
        for (size_t i = 0; i < n && !trace_ended; ++i) {
            const BlockConfig& b = g.blocks[i];
            Message m{};
            switch (b.type) {
                case BlockType::Source: {
                    m.tick = tick;
                    if (trace.is_open()) {
                        std::string line, field;
                        if (!std::getline(trace, line)) { trace_ended = true; continue; }
                        std::stringstream ss(line);
                        std::getline(ss, field, ',');
                        for (int k = 0; k < 4; ++k) { std::getline(ss, field, ','); m.payload[k] = std::stod(field); }
                    } else {
                        kernels[i]->produce(tick, 0, m.payload);
                    }
                    break;
                }
                case BlockType::Transform:
                    m = edge.at(b.input_edges[0]);
                    kernels[i]->step(m.tick, m.payload);
                    break;
                case BlockType::Merge: {
                    const Message& a = edge.at(b.input_edges[0]);
                    const Message& c = edge.at(b.input_edges[1]);
                    m.tick = a.tick;
                    kernels[i]->combine(a.tick, a.payload, c.payload, m.payload);
                    break;
                }
                case BlockType::Sink:
                    m = edge.at(b.input_edges[0]);
                    write_log_row(logs[i], m); // a sink logs exactly what it received
                    continue;
            }
            m.seq = seq[i]++;
            m.src_block_id = static_cast<uint32_t>(b.id);
            write_log_row(logs[i], m);
            for (const auto& e : b.output_edges) edge[e] = m;
        }
    }

    ReferenceResult r;
    for (size_t i = 0; i < n; ++i) {
        r.block_names.push_back(g.blocks[i].name);
        r.block_logs.push_back(logs[i].str());
    }
    r.session_digest = session_digest(r.block_names, r.block_logs);
    return r;
}

} // namespace shmmw
