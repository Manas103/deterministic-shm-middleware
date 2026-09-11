// block_runner: the one executable every block in the graph runs as. Which
// deterministic compute function it executes is decided entirely by the
// block's `type` field in the static config, read once at startup; there is
// no code path that lets a running block change what it does.
#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "shmmw/clock.hpp"
#include "shmmw/graph_config.hpp"
#include "shmmw/ring_buffer.hpp"
#include "shmmw/shm_segment.hpp"
#include "shmmw/stats.hpp"

using namespace shmmw;

namespace {

struct Args {
    std::string config_path;
    std::string block_name;
    std::string run_id;
    uint64_t ticks = 0;
    double rate_hz = 200.0;
    std::string mode = "record"; // record|replay, only meaningful for a source block
    std::string trace_path;      // source: file to write (record) or read (replay)
    std::string log_path;        // every block: its own emitted/received message log
    std::string merge_mode = "tick_keyed"; // tick_keyed|arrival_order, merge blocks only
    uint32_t num_blocks = 0; // for the start barrier; 0 disables the barrier
    std::map<std::string, uint32_t> edge_cap_override; // edge name -> forced capacity, for the fault-injection experiment
    int64_t inject_delay_ns = 0; // artificial per-message processing delay, for the CI latency-regression harness only
};

Args parse_args(int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        std::string k = argv[i];
        auto next = [&]() -> std::string { return (i + 1 < argc) ? argv[++i] : std::string(); };
        if (k == "--config") a.config_path = next();
        else if (k == "--block") a.block_name = next();
        else if (k == "--run-id") a.run_id = next();
        else if (k == "--ticks") a.ticks = std::stoull(next());
        else if (k == "--rate-hz") a.rate_hz = std::stod(next());
        else if (k == "--mode") a.mode = next();
        else if (k == "--trace") a.trace_path = next();
        else if (k == "--log") a.log_path = next();
        else if (k == "--merge-mode") a.merge_mode = next();
        else if (k == "--num-blocks") a.num_blocks = static_cast<uint32_t>(std::stoul(next()));
        else if (k == "--edge-cap-small") {
            // comma-separated edge names forced to an 8-slot ring instead of 64
            std::stringstream ss(next());
            std::string name;
            while (std::getline(ss, name, ',')) a.edge_cap_override[name] = 8;
        }
        else if (k == "--inject-delay-ns") a.inject_delay_ns = std::stoll(next());
    }
    return a;
}

std::string shm_edge_name(const std::string& run_id, const std::string& edge) {
    return "/shmmw_" + run_id + "_e_" + edge;
}
std::string shm_stats_name(const std::string& run_id, const std::string& block) {
    return "/shmmw_" + run_id + "_s_" + block;
}
std::string shm_barrier_name(const std::string& run_id) {
    return "/shmmw_" + run_id + "_barrier";
}

// Deterministic per-block linear transform. Coefficients are a fixed
// function of block id, not random, so two runs of the same block always
// apply the same map.
void apply_transform(int block_id, Message* m) {
    const double scale = 1.0 + 0.01 * block_id;
    const double offset = 0.5 * block_id;
    for (double& v : m->payload) v = v * scale + offset;
}

// Deterministic combine for a merge block: elementwise sum. Order of the two
// arguments does not matter (addition commutes), which is what makes the
// tick-keyed pairing safe regardless of which input happened to arrive
// first.
Message combine(const Message& a, const Message& b, int block_id, uint64_t seq_out) {
    Message out;
    out.tick = a.tick;
    out.seq = seq_out;
    out.src_block_id = static_cast<uint32_t>(block_id);
    out.edge_id = 0; // filled by caller
    out.send_ts_ns = now_ns();
    for (int i = 0; i < 4; ++i) out.payload[i] = a.payload[i] + b.payload[i];
    return out;
}

struct EdgeHandle {
    ShmSegment seg;
    EdgeRing* ring = nullptr;
};

uint32_t capacity_for(const Args& a, const std::string& edge) {
    auto it = a.edge_cap_override.find(edge);
    return it != a.edge_cap_override.end() ? it->second : kDefaultEdgeCapacity;
}

EdgeHandle attach_edge(const Args& a, const std::string& edge) {
    EdgeHandle h;
    const uint32_t cap = capacity_for(a, edge);
    h.seg = ShmSegment::attach(shm_edge_name(a.run_id, edge), EdgeRing::size_for(cap));
    h.ring = new EdgeRing(h.seg.data());
    return h;
}

void log_row(std::ofstream& log, const Message& m) {
    log << m.tick << ',' << m.seq << ',' << m.src_block_id << ','
        << m.payload[0] << ',' << m.payload[1] << ',' << m.payload[2] << ',' << m.payload[3] << '\n';
}

// Hop latency goes to a *separate* file from the message log above, on
// purpose: the message log is what graph_launcher hashes into the session
// digest, and hop_ns is wall-clock elapsed time, which is never the same
// between a live record and a later replay of the same trace. An earlier
// version of this file logged hop_ns in the same row as the message log and
// it broke the 100-of-100 replay-determinism claim outright (every session
// mismatched, see README "Findings"); moving it to its own file, excluded
// from the digest, fixed it without losing the measurement.
void log_hop(std::ofstream& hop_log, uint64_t tick, int64_t hop_ns) {
    hop_log << tick << ',' << hop_ns << '\n';
}

// Artificial per-message processing delay for the CI latency-regression
// harness (tools/ci_latency_gate.py). Zero on every real run; only a seeded
// regression trial passes --inject-delay-ns on the one block under test.
void inject_delay(int64_t delay_ns) {
    if (delay_ns <= 0) return;
    const int64_t deadline = now_ns() + delay_ns;
    while (now_ns() < deadline) { /* busy-wait: a real slow block burns CPU, it does not sleep */ }
}

constexpr int64_t kIdleTimeoutNs = 8'000'000'000LL; // give up waiting for more input after 8s idle

} // namespace

int main(int argc, char** argv) {
    Args a = parse_args(argc, argv);
    GraphConfig g = load_graph_config(a.config_path);
    const BlockConfig* bc = g.find_block(a.block_name);
    if (!bc) { std::fprintf(stderr, "unknown block %s\n", a.block_name.c_str()); return 1; }

    ShmSegment stats_seg = ShmSegment::attach(shm_stats_name(a.run_id, a.block_name), sizeof(BlockStats));
    BlockStats* stats = reinterpret_cast<BlockStats*>(stats_seg.data());

    std::ofstream log(a.log_path, std::ios::trunc);
    std::ofstream hop_log(a.log_path + ".hop", std::ios::trunc); // excluded from the session digest, see log_hop()
    // Default ostream precision is 6 significant digits, which silently
    // rounds a double before it hits disk. That is invisible for most
    // debugging but fatal here: replay re-parses exactly what record wrote,
    // so a rounded trace value becomes a *different* number fed into the
    // same deterministic transform, and the two runs' digests diverge even
    // though every block behaved correctly. See README "Findings": this
    // was the actual first cause of a failed 100-session replay run.
    log.precision(17);

    std::vector<EdgeHandle> inputs, outputs;
    for (auto& e : bc->input_edges) inputs.push_back(attach_edge(a, e));
    for (auto& e : bc->output_edges) outputs.push_back(attach_edge(a, e));

    if (a.num_blocks > 0) {
        ShmSegment barrier_seg = ShmSegment::attach(shm_barrier_name(a.run_id), sizeof(StartBarrier));
        auto* barrier = reinterpret_cast<StartBarrier*>(barrier_seg.data());
        barrier->ready_count.fetch_add(1, std::memory_order_acq_rel);
        while (barrier->ready_count.load(std::memory_order_acquire) < a.num_blocks) {
            usleep(200);
        }
    }

    uint64_t seq_counter = 0;

    if (bc->type == BlockType::Source) {
        std::ofstream trace_out;
        std::ifstream trace_in;
        if (a.mode == "record") { trace_out.open(a.trace_path, std::ios::trunc); trace_out.precision(17); }
        else trace_in.open(a.trace_path);

        const int64_t period_ns = static_cast<int64_t>(1e9 / a.rate_hz);
        const int64_t t0 = now_ns();

        for (uint64_t tick = 0; tick < a.ticks; ++tick) {
            Message m{};
            m.tick = tick;
            m.seq = seq_counter++;
            m.src_block_id = static_cast<uint32_t>(bc->id);

            if (a.mode == "record") {
                const int64_t intended = t0 + tick * period_ns;
                sleep_until_ns(intended);
                const int64_t jitter_ns = now_ns() - intended;
                m.payload[0] = std::sin(tick * 0.01) * 100.0;
                m.payload[1] = std::cos(tick * 0.017) * 50.0;
                m.payload[2] = static_cast<double>(tick);
                m.payload[3] = static_cast<double>(jitter_ns);
                trace_out << tick << ',' << m.payload[0] << ',' << m.payload[1] << ','
                          << m.payload[2] << ',' << m.payload[3] << '\n';
            } else {
                std::string line;
                if (!std::getline(trace_in, line)) break; // trace shorter than requested ticks
                std::stringstream ss(line);
                std::string field;
                std::getline(ss, field, ','); // tick, already known
                std::getline(ss, field, ','); m.payload[0] = std::stod(field);
                std::getline(ss, field, ','); m.payload[1] = std::stod(field);
                std::getline(ss, field, ','); m.payload[2] = std::stod(field);
                std::getline(ss, field, ','); m.payload[3] = std::stod(field);
            }

            m.send_ts_ns = now_ns();
            log_row(log, m);
            for (size_t i = 0; i < outputs.size(); ++i) {
                Message om = m;
                om.edge_id = static_cast<uint32_t>(g.edge_id(bc->output_edges[i]));
                if (!outputs[i].ring->try_push(om)) {
                    stats->drops_total.fetch_add(1, std::memory_order_relaxed);
                }
            }
            stats->msgs_out.fetch_add(static_cast<uint64_t>(outputs.size()), std::memory_order_relaxed);
            stats->alive.fetch_add(1, std::memory_order_relaxed);
        }
        return 0;
    }

    if (bc->type == BlockType::Transform) {
        uint64_t processed = 0;
        int64_t last_progress = now_ns();
        while (processed < a.ticks) {
            Message m;
            if (!inputs[0].ring->try_pop(&m)) {
                if (now_ns() - last_progress > kIdleTimeoutNs) break;
                continue;
            }
            const int64_t hop_ns = now_ns() - m.send_ts_ns;
            stats->last_hop_latency_ns.store(hop_ns, std::memory_order_relaxed);
            update_ewma(stats->ewma_hop_latency_ns, hop_ns);
            stats->msgs_in.fetch_add(1, std::memory_order_relaxed);
            stats->drops_total.store(inputs[0].ring->drop_count().load(), std::memory_order_relaxed);
            stats->backpressure_total.store(inputs[0].ring->backpressure_events().load(), std::memory_order_relaxed);

            log_hop(hop_log, m.tick, hop_ns);
            apply_transform(bc->id, &m);
            inject_delay(a.inject_delay_ns);
            m.seq = seq_counter++;
            m.src_block_id = static_cast<uint32_t>(bc->id);
            m.send_ts_ns = now_ns();
            log_row(log, m);
            for (size_t i = 0; i < outputs.size(); ++i) {
                Message om = m;
                om.edge_id = static_cast<uint32_t>(g.edge_id(bc->output_edges[i]));
                if (!outputs[i].ring->try_push(om)) stats->drops_total.fetch_add(1, std::memory_order_relaxed);
            }
            stats->msgs_out.fetch_add(static_cast<uint64_t>(outputs.size()), std::memory_order_relaxed);
            stats->alive.fetch_add(1, std::memory_order_relaxed);
            ++processed;
            last_progress = now_ns();
        }
        return 0;
    }

    if (bc->type == BlockType::Merge) {
        uint64_t combined = 0;
        int64_t last_progress = now_ns();

        if (a.merge_mode == "arrival_order") {
            // The naive first implementation: pop whichever input has a
            // message and pair the two most recently popped values in
            // arrival order, trusting that both edges stay in lockstep. See
            // README "Findings": this desynchronizes permanently the first
            // time either edge drops a message, because nothing re-aligns
            // the two streams by tick again.
            bool have_a = false, have_b = false;
            Message pend_a, pend_b;
            while (combined < a.ticks) {
                bool progressed = false;
                if (!have_a && inputs[0].ring->try_pop(&pend_a)) { have_a = true; progressed = true; }
                if (!have_b && inputs[1].ring->try_pop(&pend_b)) { have_b = true; progressed = true; }
                if (have_a && have_b) {
                    const int64_t hop_ns = now_ns() - std::min(pend_a.send_ts_ns, pend_b.send_ts_ns);
                    stats->last_hop_latency_ns.store(hop_ns, std::memory_order_relaxed);
                    update_ewma(stats->ewma_hop_latency_ns, hop_ns);
                    log_hop(hop_log, pend_a.tick, hop_ns);
                    inject_delay(a.inject_delay_ns);
                    Message out = combine(pend_a, pend_b, bc->id, seq_counter++);
                    out.edge_id = static_cast<uint32_t>(g.edge_id(bc->output_edges[0]));
                    log_row(log, out);
                    if (!outputs[0].ring->try_push(out)) stats->drops_total.fetch_add(1, std::memory_order_relaxed);
                    stats->msgs_in.fetch_add(2, std::memory_order_relaxed);
                    stats->msgs_out.fetch_add(1, std::memory_order_relaxed);
                    stats->alive.fetch_add(1, std::memory_order_relaxed);
                    have_a = have_b = false;
                    ++combined;
                    last_progress = now_ns();
                    progressed = true;
                }
                if (!progressed) {
                    if (now_ns() - last_progress > kIdleTimeoutNs) break;
                }
            }
        } else {
            // Tick-keyed merge: buffer each side by its logical tick number
            // and only combine two messages that carry the same tick,
            // regardless of which order they arrived in or whether some
            // other tick was dropped in between. A drop just leaves that one
            // tick uncombined; it cannot desynchronize any later tick.
            std::map<uint64_t, Message> pending_a, pending_b;
            while (combined < a.ticks) {
                bool progressed = false;
                Message m;
                if (inputs[0].ring->try_pop(&m)) { pending_a[m.tick] = m; progressed = true; }
                if (inputs[1].ring->try_pop(&m)) { pending_b[m.tick] = m; progressed = true; }

                for (auto it = pending_a.begin(); it != pending_a.end();) {
                    auto match = pending_b.find(it->first);
                    if (match == pending_b.end()) { ++it; continue; }
                    const int64_t hop_ns = now_ns() - std::min(it->second.send_ts_ns, match->second.send_ts_ns);
                    stats->last_hop_latency_ns.store(hop_ns, std::memory_order_relaxed);
                    update_ewma(stats->ewma_hop_latency_ns, hop_ns);
                    log_hop(hop_log, it->first, hop_ns);
                    inject_delay(a.inject_delay_ns);
                    Message out = combine(it->second, match->second, bc->id, seq_counter++);
                    out.edge_id = static_cast<uint32_t>(g.edge_id(bc->output_edges[0]));
                    log_row(log, out);
                    if (!outputs[0].ring->try_push(out)) stats->drops_total.fetch_add(1, std::memory_order_relaxed);
                    stats->msgs_in.fetch_add(2, std::memory_order_relaxed);
                    stats->msgs_out.fetch_add(1, std::memory_order_relaxed);
                    stats->alive.fetch_add(1, std::memory_order_relaxed);
                    pending_b.erase(match);
                    it = pending_a.erase(it);
                    ++combined;
                    last_progress = now_ns();
                    progressed = true;
                }
                if (!progressed) {
                    if (now_ns() - last_progress > kIdleTimeoutNs) break;
                }
            }
        }
        return 0;
    }

    if (bc->type == BlockType::Sink) {
        uint64_t processed = 0;
        int64_t last_progress = now_ns();
        while (processed < a.ticks) {
            Message m;
            if (!inputs[0].ring->try_pop(&m)) {
                if (now_ns() - last_progress > kIdleTimeoutNs) break;
                continue;
            }
            const int64_t hop_ns = now_ns() - m.send_ts_ns;
            stats->last_hop_latency_ns.store(hop_ns, std::memory_order_relaxed);
            update_ewma(stats->ewma_hop_latency_ns, hop_ns);
            stats->msgs_in.fetch_add(1, std::memory_order_relaxed);
            stats->drops_total.store(inputs[0].ring->drop_count().load(), std::memory_order_relaxed);
            stats->backpressure_total.store(inputs[0].ring->backpressure_events().load(), std::memory_order_relaxed);
            log_hop(hop_log, m.tick, hop_ns);
            log_row(log, m);
            stats->alive.fetch_add(1, std::memory_order_relaxed);
            ++processed;
            last_progress = now_ns();
        }
        return 0;
    }

    return 1;
}
