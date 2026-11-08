// Exercises UdpEdgeLink and MixedTransport over loopback (no network
// namespace needed for a unit test; tools/run_two_host_session.sh is what
// actually crosses the hosta/hostb veth pair). Confirms: a sender/receiver
// pair delivers every message, in order, round trip; MixedTransport routes
// its bridge index to the UDP link and every other index to a local
// EdgeRing untouched.
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <thread>
#include <vector>

#include "shmmw/clock.hpp"
#include "shmmw/mixed_transport.hpp"
#include "shmmw/ring_buffer.hpp"
#include "shmmw/udp_edge_link.hpp"

using namespace shmmw;

static int g_checks = 0;
#define CHECK(cond) do { \
    ++g_checks; \
    if (!(cond)) { std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); std::exit(1); } \
} while (0)

namespace {

Message make_msg(uint64_t tick, uint64_t seq) {
    Message m{};
    m.tick = tick;
    m.seq = seq;
    m.src_block_id = 7;
    m.payload[0] = (double)tick;
    return m;
}

} // namespace

int main() {
    // UdpEdgeLink: 2,000 messages sent back to back over loopback, all
    // delivered in order.
    {
        const int kPort = 58123;
        UdpEdgeLink receiver = UdpEdgeLink::make_receiver(kPort);
        UdpEdgeLink sender = UdpEdgeLink::make_sender("127.0.0.1", kPort);

        const int n = 2000;
        std::thread sender_thread([&]() {
            for (int i = 0; i < n; ++i) {
                while (!sender.send(make_msg((uint64_t)i, (uint64_t)i))) {}
            }
        });

        int received = 0;
        int64_t deadline = now_ns() + 5'000'000'000LL;
        while (received < n && now_ns() < deadline) {
            Message m;
            if (receiver.try_recv(&m)) {
                CHECK(m.tick == (uint64_t)received);
                CHECK(m.seq == (uint64_t)received);
                received++;
            }
        }
        sender_thread.join();
        CHECK(received == n);
        CHECK(receiver.drop_count() == 0);
    }

    // MixedTransport: input index 1 is the bridge (UDP loopback); input 0
    // and the single output are local heap-backed EdgeRings (the same
    // in-process pattern tests/test_engine.cpp already uses for the shm
    // transport).
    {
        const int kPort = 58124;
        alignas(8) static uint8_t in0_mem[256];
        alignas(8) static uint8_t out0_mem[256];
        EdgeRing::init(in0_mem, 8);
        EdgeRing::init(out0_mem, 8);

        auto receiver = std::make_unique<UdpEdgeLink>(UdpEdgeLink::make_receiver(kPort));
        UdpEdgeLink sender = UdpEdgeLink::make_sender("127.0.0.1", kPort);

        MixedTransport tr({in0_mem, nullptr}, {out0_mem}, /*bridge_input_index=*/1,
                           /*bridge_output_index=*/-1, std::move(receiver));

        // Local input (index 0) still works exactly like ShmTransport.
        {
            EdgeRing local_in(in0_mem);
            CHECK(local_in.try_push(make_msg(10, 10)));
        }
        Message out;
        CHECK(tr.try_recv(0, &out));
        CHECK(out.tick == 10);

        // Bridge input (index 1) is the UDP link. UdpEdgeLink's reorder
        // window is keyed on seq starting from 0 and rising by 1 per
        // message (true of every real sender in this repo, since
        // block_engine.cpp's seq_counter always starts at 0), so this
        // sends seq 0 rather than an arbitrary number.
        CHECK(sender.send(make_msg(20, 0)));
        bool got = false;
        int64_t deadline = now_ns() + 2'000'000'000LL;
        while (now_ns() < deadline) {
            if (tr.try_recv(1, &out)) { got = true; break; }
        }
        CHECK(got);
        CHECK(out.tick == 20);

        // Local output still works.
        CHECK(tr.send(0, make_msg(30, 30)));
        EdgeRing local_out(out0_mem);
        Message popped;
        CHECK(local_out.try_pop(&popped));
        CHECK(popped.tick == 30);
    }

    std::printf("%d checks passed\n", g_checks);
    return 0;
}
