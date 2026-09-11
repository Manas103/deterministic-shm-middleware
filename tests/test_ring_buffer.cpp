// Hand-rolled check suite (no GoogleTest) for the shared-memory ring buffer.
// It uses a heap allocation rather than a real POSIX shm segment, which is
// fine here: the class only ever touches the raw pointer it is given, so a
// heap buffer exercises the exact same code path as shared memory would,
// with none of the process-management noise.
#include <cassert>
#include <cstdio>
#include <cstring>
#include <thread>
#include <vector>

#include "shmmw/message.hpp"
#include "shmmw/ring_buffer.hpp"

using namespace shmmw;

static int g_checks = 0;
#define CHECK(cond) do { \
    ++g_checks; \
    if (!(cond)) { std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); std::exit(1); } \
} while (0)

Message make_msg(uint64_t tick) {
    Message m{};
    m.tick = tick;
    m.seq = tick;
    m.payload[0] = static_cast<double>(tick);
    return m;
}

void test_basic_push_pop() {
    std::vector<uint8_t> mem(EdgeRing::size_for(8));
    EdgeRing::init(mem.data(), 8);
    EdgeRing ring(mem.data());

    CHECK(ring.capacity() == 8);
    CHECK(ring.occupancy() == 0);

    for (uint64_t i = 0; i < 5; ++i) CHECK(ring.try_push(make_msg(i)));
    CHECK(ring.occupancy() == 5);

    for (uint64_t i = 0; i < 5; ++i) {
        Message out;
        CHECK(ring.try_pop(&out));
        CHECK(out.tick == i); // FIFO order preserved
    }
    Message out;
    CHECK(!ring.try_pop(&out)); // empty
}

void test_drop_on_full() {
    std::vector<uint8_t> mem(EdgeRing::size_for(4));
    EdgeRing::init(mem.data(), 4);
    EdgeRing ring(mem.data());

    for (uint64_t i = 0; i < 4; ++i) CHECK(ring.try_push(make_msg(i)));
    CHECK(ring.drop_count().load() == 0);

    // Buffer is full; the 5th push must be rejected and counted, not
    // silently overwrite the oldest unread slot.
    CHECK(!ring.try_push(make_msg(4)));
    CHECK(ring.drop_count().load() == 1);

    Message out;
    CHECK(ring.try_pop(&out));
    CHECK(out.tick == 0); // the dropped push never displaced slot 0

    CHECK(ring.try_push(make_msg(5))); // room again after one pop
}

void test_backpressure_counter_fires_before_full() {
    std::vector<uint8_t> mem(EdgeRing::size_for(8));
    EdgeRing::init(mem.data(), 8);
    EdgeRing ring(mem.data());

    // 80% of 8 is 6.4; the threshold check runs against occupancy *before*
    // the push that would cross it, so the 8th push (occupancy 7 going in)
    // is the first to trip it, one push before the buffer is actually full.
    for (uint64_t i = 0; i < 8; ++i) CHECK(ring.try_push(make_msg(i)));
    CHECK(ring.backpressure_events().load() >= 1);
    CHECK(ring.drop_count().load() == 0); // full, but nothing rejected yet
}

void test_wraparound_preserves_order() {
    std::vector<uint8_t> mem(EdgeRing::size_for(4));
    EdgeRing::init(mem.data(), 4);
    EdgeRing ring(mem.data());

    // Push/pop past the capacity boundary several times over so the index
    // mask actually exercises wraparound, not just the first lap.
    uint64_t next_expected = 0;
    for (uint64_t round = 0; round < 50; ++round) {
        CHECK(ring.try_push(make_msg(round)));
        if (round >= 2) {
            Message out;
            CHECK(ring.try_pop(&out));
            CHECK(out.tick == next_expected);
            ++next_expected;
        }
    }
}

void test_concurrent_spsc_order_and_drop_accounting() {
    // A genuine two-thread producer/consumer stress run, the closest a
    // single-process check gets to the real cross-process access pattern.
    // The producer makes exactly one try_push attempt per tick, the same
    // policy every real block uses (drop-newest, never retry), so this
    // exercises the actual drop path rather than an artificially lossless
    // one. What must hold regardless of how many ticks the OS scheduler
    // lets through: whatever the consumer does see arrives in tick order
    // with no gap unaccounted for, and every tick that did not arrive is
    // reflected exactly once in drop_count.
    std::vector<uint8_t> mem(EdgeRing::size_for(64));
    EdgeRing::init(mem.data(), 64);
    EdgeRing ring(mem.data());

    constexpr uint64_t kN = 200000;
    std::vector<uint64_t> received;
    received.reserve(kN);
    std::atomic<bool> producer_done{false};
    uint64_t pushed_ok = 0;

    std::thread producer([&] {
        for (uint64_t i = 0; i < kN; ++i) {
            if (ring.try_push(make_msg(i))) ++pushed_ok;
        }
        producer_done.store(true, std::memory_order_release);
    });
    std::thread consumer([&] {
        Message out;
        for (;;) {
            if (ring.try_pop(&out)) { received.push_back(out.tick); continue; }
            if (producer_done.load(std::memory_order_acquire) && ring.occupancy() == 0) break;
        }
    });
    producer.join();
    consumer.join();

    CHECK(received.size() == pushed_ok); // everything successfully pushed was eventually popped
    CHECK(kN - pushed_ok == ring.drop_count().load()); // every non-pushed tick is accounted for
    for (size_t i = 1; i < received.size(); ++i) CHECK(received[i] > received[i - 1]); // strict order, no reorder
}

int main() {
    test_basic_push_pop();
    test_drop_on_full();
    test_backpressure_counter_fires_before_full();
    test_wraparound_preserves_order();
    test_concurrent_spsc_order_and_drop_accounting();
    std::printf("check_ring_buffer: %d checks passed\n", g_checks);
    return 0;
}
