#include "shmmw/block_engine.hpp"

#include <algorithm>
#include <fstream>
#include <map>
#include <sstream>
#include <vector>

#include "shmmw/clock.hpp"
#include "shmmw/session_log.hpp"

namespace shmmw {

namespace {

// Hop latency goes to a *separate* stream from the message log, on purpose:
// the message log is what graph_launcher hashes into the session digest, and
// hop_ns is wall-clock elapsed time, which is never the same between a live
// record and a later replay of the same trace. An earlier version logged
// hop_ns in the same row and it broke the 100-of-100 replay-determinism
// claim outright (every session mismatched, see README "Findings").
void log_hop(std::ostream& hop_log, uint64_t tick, int64_t hop_ns) { hop_log << tick << ',' << hop_ns << '\n'; }

// Artificial per-message processing delay for the CI latency-regression
// harness (tools/ci_latency_gate.py). Zero on every real run.
void inject_delay(int64_t delay_ns) {
    if (delay_ns <= 0) return;
    const int64_t deadline = now_ns() + delay_ns;
    while (now_ns() < deadline) { /* busy-wait: a real slow block burns CPU, it does not sleep */ }
}

void record_hop(BlockStats* st, std::ostream& hop_log, uint64_t tick, int64_t hop_ns) {
    st->last_hop_latency_ns.store(hop_ns, std::memory_order_relaxed);
    update_ewma(st->ewma_hop_latency_ns, hop_ns);
    log_hop(hop_log, tick, hop_ns);
}

} // namespace

int run_block(const GraphConfig& g, const BlockConfig& bc, Kernel& kernel, ITransport& tr,
              const BlockIo& io, const EngineOptions& opt) {
    std::ostream& log = *io.log;
    std::ostream& hop_log = *io.hop_log;
    BlockStats* stats = io.stats;
    // Default ostream precision is 6 significant digits, which silently
    // rounds a double before it hits the log; 17 round-trips any double.
    log.precision(17);

    std::vector<uint32_t> out_edge_ids;
    for (const auto& e : bc.output_edges) out_edge_ids.push_back(static_cast<uint32_t>(g.edge_id(e)));
    const uint32_t self_id = static_cast<uint32_t>(bc.id);
    uint64_t seq_counter = 0;

    auto emit = [&](Message& m) {
        m.send_ts_ns = now_ns();
        write_log_row(log, m);
        for (size_t i = 0; i < out_edge_ids.size(); ++i) {
            Message om = m;
            om.edge_id = out_edge_ids[i];
            if (!tr.send(i, om)) stats->drops_total.fetch_add(1, std::memory_order_relaxed);
        }
        stats->msgs_out.fetch_add(static_cast<uint64_t>(out_edge_ids.size()), std::memory_order_relaxed);
        stats->alive.fetch_add(1, std::memory_order_relaxed);
    };

    if (bc.type == BlockType::Source) {
        std::ofstream trace_out;
        std::ifstream trace_in;
        const bool record = opt.mode == "record";
        if (record && !opt.trace_path.empty()) { trace_out.open(opt.trace_path, std::ios::trunc); trace_out.precision(17); }
        if (!record) trace_in.open(opt.trace_path);

        const int64_t period_ns = opt.rate_hz > 0 ? static_cast<int64_t>(1e9 / opt.rate_hz) : 0;
        const int64_t t0 = now_ns();

        for (uint64_t tick = 0; tick < opt.ticks; ++tick) {
            Message m{};
            m.tick = tick;
            m.seq = seq_counter++;
            m.src_block_id = self_id;
            if (record) {
                const int64_t intended = t0 + static_cast<int64_t>(tick) * period_ns;
                if (period_ns > 0) sleep_until_ns(intended);
                const int64_t jitter_ns = period_ns > 0 ? now_ns() - intended : 0;
                kernel.produce(tick, jitter_ns, m.payload);
                if (trace_out.is_open()) {
                    trace_out << tick << ',' << m.payload[0] << ',' << m.payload[1] << ',' << m.payload[2]
                              << ',' << m.payload[3] << '\n';
                }
            } else {
                std::string line;
                if (!std::getline(trace_in, line)) break; // trace shorter than requested ticks
                std::stringstream ss(line);
                std::string field;
                std::getline(ss, field, ','); // tick, already known
                for (int i = 0; i < 4; ++i) { std::getline(ss, field, ','); m.payload[i] = std::stod(field); }
            }
            emit(m);
        }
        return 0;
    }

    if (bc.type == BlockType::Transform || bc.type == BlockType::Sink) {
        const bool sink = bc.type == BlockType::Sink;
        uint64_t processed = 0;
        int64_t last_progress = now_ns();
        while (processed < opt.ticks) {
            Message m;
            if (!tr.try_recv(0, &m)) {
                if (now_ns() - last_progress > opt.idle_timeout_ns) return 2;
                tr.idle();
                continue;
            }
            const int64_t hop_ns = now_ns() - m.send_ts_ns;
            stats->msgs_in.fetch_add(1, std::memory_order_relaxed);
            stats->drops_total.store(tr.input_drops(0), std::memory_order_relaxed);
            stats->backpressure_total.store(tr.input_backpressure(0), std::memory_order_relaxed);
            record_hop(stats, hop_log, m.tick, hop_ns);
            if (sink) {
                write_log_row(log, m);
                stats->alive.fetch_add(1, std::memory_order_relaxed);
            } else {
                kernel.step(m.tick, m.payload);
                inject_delay(opt.inject_delay_ns);
                m.seq = seq_counter++;
                m.src_block_id = self_id;
                emit(m);
            }
            ++processed;
            last_progress = now_ns();
        }
        return 0;
    }

    // Merge.
    uint64_t combined = 0;
    int64_t last_progress = now_ns();
    auto combine_and_emit = [&](const Message& a, const Message& b) {
        const int64_t hop_ns = now_ns() - std::min(a.send_ts_ns, b.send_ts_ns);
        record_hop(stats, hop_log, a.tick, hop_ns);
        inject_delay(opt.inject_delay_ns);
        Message out{};
        out.tick = a.tick;
        out.seq = seq_counter++;
        out.src_block_id = self_id;
        kernel.combine(a.tick, a.payload, b.payload, out.payload);
        stats->msgs_in.fetch_add(2, std::memory_order_relaxed);
        emit(out);
        ++combined;
        last_progress = now_ns();
    };

    if (opt.merge_mode == "arrival_order") {
        // The naive first implementation: pair the two most recently popped
        // values in arrival order, trusting both edges stay in lockstep. It
        // desynchronizes permanently the first time either edge drops a
        // message, because nothing re-aligns the two streams by tick again.
        bool have_a = false, have_b = false;
        Message pend_a, pend_b;
        while (combined < opt.ticks) {
            bool progressed = false;
            if (!have_a && tr.try_recv(0, &pend_a)) { have_a = true; progressed = true; }
            if (!have_b && tr.try_recv(1, &pend_b)) { have_b = true; progressed = true; }
            if (have_a && have_b) {
                combine_and_emit(pend_a, pend_b);
                have_a = have_b = false;
                progressed = true;
            }
            if (!progressed) {
                if (now_ns() - last_progress > opt.idle_timeout_ns) return 2;
                tr.idle();
            }
        }
        return 0;
    }

    // Tick-keyed merge: buffer each side by logical tick and only combine two
    // messages carrying the same tick, regardless of arrival order. Input 0
    // is always passed to the kernel first, so an order-sensitive kernel
    // (the AV sensor takes ego then agent) sees a fixed argument order.
    std::map<uint64_t, Message> pending_a, pending_b;
    while (combined < opt.ticks) {
        bool progressed = false;
        Message m;
        if (tr.try_recv(0, &m)) { pending_a[m.tick] = m; progressed = true; }
        if (tr.try_recv(1, &m)) { pending_b[m.tick] = m; progressed = true; }
        for (auto it = pending_a.begin(); it != pending_a.end();) {
            auto match = pending_b.find(it->first);
            if (match == pending_b.end()) { ++it; continue; }
            combine_and_emit(it->second, match->second);
            pending_b.erase(match);
            it = pending_a.erase(it);
            progressed = true;
        }
        if (!progressed) {
            if (now_ns() - last_progress > opt.idle_timeout_ns) return 2;
            tr.idle();
        }
    }
    return 0;
}

} // namespace shmmw
