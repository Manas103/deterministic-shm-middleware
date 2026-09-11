#pragma once
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <new>

#include "shmmw/message.hpp"

namespace shmmw {

// Single-producer single-consumer ring buffer of fixed-size Message slots,
// living entirely inside one shared-memory segment. head is only ever
// written by the producer, tail only ever written by the consumer; each
// side only reads the other's index. That single-writer-per-index rule is
// what makes this lock-free without a CAS loop: there is never more than
// one writer for any given memory location.
//
// Capacity is chosen when the segment is created (a power of two, so index
// masking replaces a modulo) and is stored as the first header field, so
// every attaching process reads it from shared memory rather than assuming
// a compile-time constant. That is what lets the fault-injection experiment
// in the README undersize exactly one edge without a second build.
//
// Policy on overflow: drop-newest. A slow consumer is far more common in
// this graph than a slow producer (every block downstream of a merge is
// consuming from up to two producers at 200 Hz), and dropping the newest
// message rather than overwriting an unread one keeps every message a
// consumer does see uncorrupted, at the cost of a gap in tick numbers that
// the consumer can detect and count.
struct EdgeRingHeader {
    uint32_t capacity;
    uint32_t _pad;
    std::atomic<uint64_t> head{0}; // next slot the producer will write
    std::atomic<uint64_t> tail{0}; // next slot the consumer will read
    std::atomic<uint64_t> drop_count{0};
    std::atomic<uint64_t> backpressure_events{0};
};

class EdgeRing {
public:
    static size_t size_for(uint32_t capacity) {
        return sizeof(EdgeRingHeader) + static_cast<size_t>(capacity) * sizeof(Message);
    }

    // Placement-initializes a fresh header at `mem` (called once, by the
    // owner, right after the segment is created).
    static void init(void* mem, uint32_t capacity) {
        auto* h = new (mem) EdgeRingHeader();
        h->capacity = capacity;
    }

    EdgeRing() = default;

    // Wraps an already-initialized segment (every other process, including
    // re-attaches, call this; it never re-initializes head/tail).
    explicit EdgeRing(void* mem) { attach(mem); }

    void attach(void* mem) {
        h_ = reinterpret_cast<EdgeRingHeader*>(mem);
        slots_ = reinterpret_cast<Message*>(reinterpret_cast<uint8_t*>(mem) + sizeof(EdgeRingHeader));
    }

    uint32_t capacity() const { return h_->capacity; }
    std::atomic<uint64_t>& drop_count() { return h_->drop_count; }
    std::atomic<uint64_t>& backpressure_events() { return h_->backpressure_events; }

    bool try_push(const Message& m) {
        const uint64_t h = h_->head.load(std::memory_order_relaxed);
        const uint64_t t = h_->tail.load(std::memory_order_acquire);
        const uint64_t occupancy = h - t;
        const uint32_t cap = h_->capacity;
        if (occupancy >= cap) {
            h_->drop_count.fetch_add(1, std::memory_order_relaxed);
            return false;
        }
        if (occupancy * 5 >= static_cast<uint64_t>(cap) * 4) {
            h_->backpressure_events.fetch_add(1, std::memory_order_relaxed);
        }
        slots_[h & (cap - 1)] = m;
        h_->head.store(h + 1, std::memory_order_release);
        return true;
    }

    bool try_pop(Message* out) {
        const uint64_t t = h_->tail.load(std::memory_order_relaxed);
        const uint64_t h = h_->head.load(std::memory_order_acquire);
        if (t == h) return false;
        *out = slots_[t & (h_->capacity - 1)];
        h_->tail.store(t + 1, std::memory_order_release);
        return true;
    }

    uint64_t occupancy() const {
        return h_->head.load(std::memory_order_relaxed) - h_->tail.load(std::memory_order_relaxed);
    }

private:
    EdgeRingHeader* h_;
    Message* slots_;
};

// Every edge in the shipped graph uses this depth. The first version of this
// project shipped with 64 slots (320 ms at 200 Hz) on the theory that this
// was already generous; see README "Findings" for the measurement that
// proved it wrong (this machine's real multi-process scheduling jitter,
// under the load of 12 forked block processes plus whatever else is running
// on a shared development machine, exceeded 320 ms often enough that only
// 4 of 10 replay sessions produced a matching digest). 8192 slots is
// 40.96 s of buffering at 200 Hz, comfortably past any stall actually
// observed. The fault-injection experiment overrides it per-edge to 8 slots
// (40 ms) to force real drops deliberately.
constexpr uint32_t kDefaultEdgeCapacity = 8192;

} // namespace shmmw
