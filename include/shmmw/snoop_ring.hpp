#pragma once
#include <atomic>
#include <cstdint>

#include "shmmw/message.hpp"
#include "shmmw/ring_buffer.hpp"

namespace shmmw {

// A non-intrusive view of an EdgeRing: every method is const and reads only,
// never advances `tail` (what a consumer does) and never touches `head`
// (what the producer does). Pairs with ShmSegment::attach_readonly so the
// "does not touch producer state" claim is two layers deep: this class has
// no write path at all, and even if it did, the underlying mapping is
// PROT_READ so the write would fault. See tests/test_snooper.cpp.
class SnoopRing {
public:
    SnoopRing() = default;
    explicit SnoopRing(const void* mem)
        : h_(reinterpret_cast<const EdgeRingHeader*>(mem)),
          slots_(reinterpret_cast<const Message*>(reinterpret_cast<const uint8_t*>(mem) + sizeof(EdgeRingHeader))) {}

    uint32_t capacity() const { return h_->capacity; }
    uint64_t head() const { return h_->head.load(std::memory_order_acquire); }
    uint64_t tail() const { return h_->tail.load(std::memory_order_acquire); }
    uint64_t occupancy() const { return head() - tail(); }
    uint64_t drop_count() const { return h_->drop_count.load(std::memory_order_relaxed); }
    uint64_t backpressure_events() const { return h_->backpressure_events.load(std::memory_order_relaxed); }

    // Reads the slot at absolute ring index `idx` (caller picks an index in
    // [tail(), head())). Snooping a slot the producer is mid-write to is
    // possible in principle (there is no seqlock here, unlike the market-data
    // publisher) but vanishingly unlikely at the polling rates `snooper`
    // uses; test_snooper.cpp's correctness check runs many iterations and
    // never caught a torn read, and the honest limitation is recorded in the
    // README rather than hidden behind an unused retry loop.
    Message read_slot(uint64_t idx) const { return slots_[idx & (capacity() - 1)]; }

    bool peek_latest(Message* out) const {
        const uint64_t h = head();
        const uint64_t t = tail();
        if (h == t) return false;
        *out = read_slot(h - 1);
        return true;
    }

private:
    const EdgeRingHeader* h_ = nullptr;
    const Message* slots_ = nullptr;
};

} // namespace shmmw
