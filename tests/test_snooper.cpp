// Hand-rolled check suite for the snooper's two guarantees: it sees what the
// producer wrote, and it cannot touch producer or consumer state. Unlike
// test_ring_buffer.cpp this uses a real POSIX shm segment, because the
// second guarantee (a write through a PROT_READ mapping faults) is a kernel
// behavior that a heap buffer cannot exercise.
#include <sys/wait.h>
#include <unistd.h>

#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "shmmw/message.hpp"
#include "shmmw/ring_buffer.hpp"
#include "shmmw/shm_segment.hpp"
#include "shmmw/snoop_ring.hpp"

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
    m.src_block_id = 7;
    m.payload[0] = static_cast<double>(tick);
    return m;
}

void test_snoop_sees_what_producer_wrote() {
    const std::string name = "/shmmw_test_snoop_" + std::to_string(getpid());
    const uint32_t cap = 16;
    ShmSegment owner = ShmSegment::create(name, EdgeRing::size_for(cap));
    EdgeRing::init(owner.data(), cap);
    EdgeRing producer(owner.data());

    ShmSegment ro = ShmSegment::attach_readonly(name, EdgeRing::size_for(cap));
    SnoopRing snoop(ro.data());

    CHECK(snoop.capacity() == cap);
    CHECK(snoop.occupancy() == 0);
    Message out;
    CHECK(!snoop.peek_latest(&out)); // empty ring, nothing to see yet

    for (uint64_t i = 0; i < 5; ++i) CHECK(producer.try_push(make_msg(i)));
    CHECK(snoop.occupancy() == 5); // snoop re-reads head/tail live, no refresh call needed
    CHECK(snoop.peek_latest(&out));
    CHECK(out.tick == 4);
    CHECK(out.src_block_id == 7);

    for (uint64_t i = 5; i < 20; ++i) producer.try_push(make_msg(i)); // overflow, forces drops
    CHECK(snoop.drop_count() > 0);
}

void test_snoop_never_advances_tail() {
    // The real test of "does not touch producer state": a snoop that read
    // the ring a thousand times must not have consumed a single message the
    // real consumer was entitled to.
    const std::string name = "/shmmw_test_snoop_tail_" + std::to_string(getpid());
    const uint32_t cap = 64;
    ShmSegment owner = ShmSegment::create(name, EdgeRing::size_for(cap));
    EdgeRing::init(owner.data(), cap);
    EdgeRing producer(owner.data());

    ShmSegment ro = ShmSegment::attach_readonly(name, EdgeRing::size_for(cap));
    SnoopRing snoop(ro.data());

    for (uint64_t i = 0; i < 40; ++i) CHECK(producer.try_push(make_msg(i)));

    for (int i = 0; i < 1000; ++i) {
        Message peeked;
        snoop.occupancy();
        snoop.peek_latest(&peeked);
        snoop.drop_count();
    }

    // A second, independent EdgeRing attach is the real consumer; it must
    // still pop all 40 messages in order, proving the 1000 snoop reads
    // consumed nothing.
    ShmSegment consumer_seg = ShmSegment::attach(name, EdgeRing::size_for(cap));
    EdgeRing consumer(consumer_seg.data());
    for (uint64_t i = 0; i < 40; ++i) {
        Message out;
        CHECK(consumer.try_pop(&out));
        CHECK(out.tick == i);
    }
    Message out;
    CHECK(!consumer.try_pop(&out)); // exactly 40, not fewer (nothing double-consumed by the snoop)
}

void test_readonly_mapping_faults_on_write() {
    // The second, independent layer: even if SnoopRing had a bug and tried
    // to write, the mapping itself cannot be written to. Proven by forking a
    // child that tries, and checking it dies of SIGSEGV rather than
    // succeeding or failing some other way.
    const std::string name = "/shmmw_test_snoop_fault_" + std::to_string(getpid());
    const uint32_t cap = 8;
    ShmSegment owner = ShmSegment::create(name, EdgeRing::size_for(cap));
    EdgeRing::init(owner.data(), cap);

    pid_t pid = fork();
    CHECK(pid >= 0);
    if (pid == 0) {
        ShmSegment ro = ShmSegment::attach_readonly(name, EdgeRing::size_for(cap));
        volatile uint8_t* p = reinterpret_cast<volatile uint8_t*>(ro.data());
        *p = 0xFF; // must fault; PROT_READ forbids this write
        std::_Exit(42); // only reached if the write did NOT fault, which is the failure case
    }
    int status = 0;
    CHECK(waitpid(pid, &status, 0) == pid);
    CHECK(WIFSIGNALED(status));
    CHECK(WTERMSIG(status) == SIGSEGV);
}

int main() {
    test_snoop_sees_what_producer_wrote();
    test_snoop_never_advances_tail();
    test_readonly_mapping_faults_on_write();
    std::printf("check_snooper: %d checks passed\n", g_checks);
    return 0;
}
