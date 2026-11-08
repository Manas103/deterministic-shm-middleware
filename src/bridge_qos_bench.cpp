// Two topics sharing one UDP link across the inter-host bridge: a 100Hz
// control topic (64-byte messages, a tight deadline) and a 2MB-per-frame
// LiDAR-sized topic sent back to back (a loose deadline), scheduled either
// FIFO (plain arrival order) or EDF (include/shmmw/edf_scheduler.hpp).
// Measures the control topic's end-to-end p50/p99 latency and the LiDAR
// stream's achieved throughput under each policy, over a real veth pair
// shaped to 1Gbps by tc (see tools/setup_netns_bridge.sh). This is a
// standalone measurement tool; it shares no code with block_engine.cpp
// because there is no block graph here, only two topics contending for one
// wire, which is the thing the resume's EDF claim is actually about.
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <string>
#include <vector>

#include "shmmw/clock.hpp"
#include "shmmw/edf_scheduler.hpp"

using namespace shmmw;

namespace {

constexpr int kControlType = 0;
constexpr int kLidarType = 1;
constexpr int kDoneType = 2;

constexpr size_t kChunkPayload = 1400; // bytes, under the default 1500 MTU
constexpr size_t kLidarFrameBytes = 2 * 1024 * 1024;
constexpr int64_t kControlPeriodNs = 10'000'000;      // 100Hz
constexpr int64_t kControlDeadlineNs = 5'000'000;      // EDF: control is urgent
constexpr int64_t kLidarDeadlineNs = 500'000'000;      // EDF: LiDAR is not

#pragma pack(push, 1)
struct WireHeader {
    uint8_t type;
    uint32_t chunk_index;
    uint32_t chunk_count;
    int64_t send_ts_ns;
};
#pragma pack(pop)

struct Options {
    std::string role;   // sender | receiver
    std::string policy; // fifo | edf
    std::string peer_ip;
    int port = 0;
    double duration_sec = 5.0;
};

Options parse(int argc, char** argv) {
    Options o;
    for (int i = 1; i < argc; ++i) {
        std::string k = argv[i];
        auto next = [&]() -> std::string { return (i + 1 < argc) ? argv[++i] : std::string(); };
        if (k == "--role") o.role = next();
        else if (k == "--policy") o.policy = next();
        else if (k == "--peer-ip") o.peer_ip = next();
        else if (k == "--port") o.port = std::atoi(next().c_str());
        else if (k == "--duration-sec") o.duration_sec = std::atof(next().c_str());
    }
    return o;
}

struct PendingChunk {
    std::vector<uint8_t> datagram;
    int64_t deadline_ns;
};

int run_sender(const Options& o) {
    int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in peer{};
    peer.sin_family = AF_INET;
    peer.sin_port = htons(static_cast<uint16_t>(o.port));
    ::inet_pton(AF_INET, o.peer_ip.c_str(), &peer.sin_addr);
    ::connect(fd, reinterpret_cast<sockaddr*>(&peer), sizeof(peer));

    const bool edf = o.policy == "edf";
    std::deque<PendingChunk> fifo_q;
    EdfScheduler edf_sched;
    std::deque<PendingChunk> edf_items; // parallel storage, indexed like edf_sched's positions

    auto make_control = [&]() {
        PendingChunk pc;
        pc.datagram.resize(sizeof(WireHeader));
        auto* h = reinterpret_cast<WireHeader*>(pc.datagram.data());
        h->type = kControlType;
        h->chunk_index = 0;
        h->chunk_count = 1;
        h->send_ts_ns = now_ns(); // set at send time below, this is a placeholder
        pc.deadline_ns = now_ns() + kControlDeadlineNs;
        return pc;
    };

    auto make_lidar_frame = [&](std::vector<PendingChunk>& out) {
        const uint32_t chunk_count = static_cast<uint32_t>((kLidarFrameBytes + kChunkPayload - 1) / kChunkPayload);
        for (uint32_t i = 0; i < chunk_count; ++i) {
            PendingChunk pc;
            pc.datagram.resize(sizeof(WireHeader) + kChunkPayload);
            auto* h = reinterpret_cast<WireHeader*>(pc.datagram.data());
            h->type = kLidarType;
            h->chunk_index = i;
            h->chunk_count = chunk_count;
            h->send_ts_ns = now_ns();
            pc.deadline_ns = now_ns() + kLidarDeadlineNs;
            out.push_back(std::move(pc));
        }
    };

    const int64_t t_start = now_ns();
    const int64_t t_end = t_start + static_cast<int64_t>(o.duration_sec * 1e9);
    int64_t next_control_ns = t_start;
    uint64_t control_sent = 0, lidar_chunks_sent = 0, lidar_frames_started = 0;
    size_t lidar_pending_in_queue = 0; // chunks of the current frame still unsent (enqueued already)

    while (now_ns() < t_end) {
        const int64_t t = now_ns();
        if (t >= next_control_ns) {
            PendingChunk c = make_control();
            if (edf) {
                edf_sched.push(c.deadline_ns);
                edf_items.push_back(std::move(c));
            } else {
                fifo_q.push_back(std::move(c));
            }
            next_control_ns += kControlPeriodNs;
            control_sent++;
        }
        if (lidar_pending_in_queue == 0) {
            // A whole ~1,500-chunk frame arrives at once, the way a real
            // sensor driver hands a full frame to the network stack in one
            // write, not one chunk trickling in per loop iteration. This
            // is what makes FIFO's contention real: a control message
            // enqueued while this frame is still in the queue genuinely
            // has to wait behind however much of the frame precedes it.
            // The next frame is not started until this one is fully sent
            // (lidar_pending_in_queue back to 0), so frames run back to
            // back rather than piling up unboundedly in memory.
            std::vector<PendingChunk> frame;
            make_lidar_frame(frame);
            for (auto& c : frame) {
                if (edf) {
                    edf_sched.push(c.deadline_ns);
                    edf_items.push_back(std::move(c));
                } else {
                    fifo_q.push_back(std::move(c));
                }
            }
            lidar_pending_in_queue = frame.size();
            lidar_frames_started++;
        }

        PendingChunk next;
        bool have_next = false;
        if (edf) {
            if (!edf_sched.empty()) {
                size_t pos = edf_sched.earliest_position();
                next = std::move(edf_items[pos]);
                edf_sched.erase(pos);
                edf_items.erase(edf_items.begin() + static_cast<long>(pos));
                have_next = true;
            }
        } else if (!fifo_q.empty()) {
            next = std::move(fifo_q.front());
            fifo_q.pop_front();
            have_next = true;
        }
        if (have_next) {
            auto* h = reinterpret_cast<WireHeader*>(next.datagram.data());
            h->send_ts_ns = now_ns(); // stamped at actual send time, not enqueue time
            ssize_t rc;
            do {
                rc = ::send(fd, next.datagram.data(), next.datagram.size(), 0);
            } while (rc < 0 && (errno == EAGAIN || errno == ENOBUFS));
            if (h->type == kLidarType) {
                lidar_chunks_sent++;
                lidar_pending_in_queue--;
            }
        }
    }

    WireHeader done{};
    done.type = kDoneType;
    ::send(fd, &done, sizeof(done), 0);
    ::close(fd);
    std::printf(
        "{\"role\":\"sender\",\"policy\":\"%s\",\"control_sent\":%llu,\"lidar_chunks_sent\":%llu,"
        "\"lidar_frames_started\":%llu}\n",
        o.policy.c_str(), (unsigned long long)control_sent, (unsigned long long)lidar_chunks_sent,
        (unsigned long long)lidar_frames_started);
    return 0;
}

int run_receiver(const Options& o) {
    int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    int one = 1;
    ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(static_cast<uint16_t>(o.port));
    ::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));

    std::vector<int64_t> control_latencies_ns;
    uint64_t lidar_bytes = 0;
    int64_t t_first = -1, t_last = -1;

    std::vector<uint8_t> buf(sizeof(WireHeader) + kChunkPayload + 64);
    for (;;) {
        ssize_t n = ::recv(fd, buf.data(), buf.size(), 0);
        if (n < static_cast<ssize_t>(sizeof(WireHeader))) continue;
        auto* h = reinterpret_cast<WireHeader*>(buf.data());
        const int64_t now = now_ns();
        if (h->type == kDoneType) break;
        if (t_first < 0) t_first = now;
        t_last = now;
        if (h->type == kControlType) {
            control_latencies_ns.push_back(now - h->send_ts_ns);
        } else if (h->type == kLidarType) {
            lidar_bytes += static_cast<uint64_t>(n) - sizeof(WireHeader);
        }
    }
    ::close(fd);

    std::sort(control_latencies_ns.begin(), control_latencies_ns.end());
    auto pct = [&](double p) -> int64_t {
        if (control_latencies_ns.empty()) return -1;
        size_t idx = static_cast<size_t>(p * (double)(control_latencies_ns.size() - 1));
        return control_latencies_ns[idx];
    };
    const double secs = t_first >= 0 ? (double)(t_last - t_first) / 1e9 : 0.0;
    const double mbps = secs > 0 ? (double)lidar_bytes * 8.0 / secs / 1e6 : 0.0;

    std::printf(
        "{\"role\":\"receiver\",\"control_count\":%zu,\"control_p50_ns\":%lld,\"control_p99_ns\":%lld,"
        "\"control_max_ns\":%lld,\"lidar_bytes\":%llu,\"wall_secs\":%.3f,\"lidar_mbps\":%.1f}\n",
        control_latencies_ns.size(), (long long)pct(0.50), (long long)pct(0.99),
        (long long)(control_latencies_ns.empty() ? -1 : control_latencies_ns.back()),
        (unsigned long long)lidar_bytes, secs, mbps);
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    Options o = parse(argc, argv);
    if (o.role == "sender") return run_sender(o);
    if (o.role == "receiver") return run_receiver(o);
    std::fprintf(stderr, "usage: bridge_qos_bench --role sender|receiver --policy fifo|edf --peer-ip <ip> "
                          "--port <port> [--duration-sec N]\n");
    return 1;
}
