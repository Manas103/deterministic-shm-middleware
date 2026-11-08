#pragma once
#include <cstdint>
#include <deque>
#include <vector>

namespace shmmw {

// Earliest-deadline-first send scheduling over a set of pending outbound
// items sharing one wire. Each item carries a deadline (an absolute
// CLOCK_MONOTONIC nanosecond timestamp); next_index() returns the index of
// whichever currently-pending item's deadline is soonest, so a control
// topic's tight deadline always wins over a bulk topic's loose one, no
// matter which arrived first. FIFO is not a mode of this class; it is
// "always send index 0, in arrival order", which bridge_qos_bench.cpp
// implements directly with a plain queue for the baseline comparison.
struct ScheduledItem {
    int64_t deadline_ns;
    uint64_t enqueue_seq; // tie-break: earlier arrival wins among equal deadlines
};

class EdfScheduler {
public:
    void push(int64_t deadline_ns) {
        items_.push_back(ScheduledItem{deadline_ns, next_seq_++});
    }

    bool empty() const { return items_.empty(); }
    size_t size() const { return items_.size(); }

    // Index (into the order items were push()ed, ignoring prior pop()s'
    // renumbering) of the pending item with the earliest deadline. Callers
    // pop_front-equivalent by calling pop_earliest() once they have sent
    // the corresponding payload.
    size_t earliest_position() const {
        size_t best = 0;
        for (size_t i = 1; i < items_.size(); ++i) {
            const auto& a = items_[i];
            const auto& b = items_[best];
            if (a.deadline_ns < b.deadline_ns ||
                (a.deadline_ns == b.deadline_ns && a.enqueue_seq < b.enqueue_seq)) {
                best = i;
            }
        }
        return best;
    }

    void erase(size_t position) { items_.erase(items_.begin() + static_cast<long>(position)); }

private:
    std::vector<ScheduledItem> items_;
    uint64_t next_seq_ = 0;
};

} // namespace shmmw
