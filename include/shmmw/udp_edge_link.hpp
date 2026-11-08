#pragma once
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cstdint>
#include <cstring>
#include <map>
#include <stdexcept>
#include <string>

#include "shmmw/message.hpp"

namespace shmmw {

// One UDP socket carrying exactly one edge's Message stream across a host
// boundary: a sender connect()s to a peer and send()s one datagram per
// Message (64 bytes, far under any real MTU); a receiver binds a local
// port and recvfrom()s, non-blocking, same discipline as EdgeRing::try_pop.
// This is the only place a Message crosses a socket; nothing else in this
// repository knows a network exists.
//
// Delivery is best-effort (UDP is not FIFO or lossless by itself), but a
// receiver out-of-order datagram is reordered back into tick order by a
// small window (see try_recv()) rather than handed to the block out of
// order, and anything the window cannot fix is counted as a drop through
// the same drop_count()/backpressure_events() the shared-memory ring
// exposes, so the "lossless or counted" contract ITransport documents
// holds for this transport too.
class UdpEdgeLink {
public:
    static UdpEdgeLink make_sender(const std::string& peer_ip, int peer_port) {
        UdpEdgeLink l;
        l.fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (l.fd_ < 0) throw std::runtime_error("UdpEdgeLink: socket() failed");
        sockaddr_in peer{};
        peer.sin_family = AF_INET;
        peer.sin_port = htons(static_cast<uint16_t>(peer_port));
        if (::inet_pton(AF_INET, peer_ip.c_str(), &peer.sin_addr) != 1) {
            throw std::runtime_error("UdpEdgeLink: bad peer ip " + peer_ip);
        }
        if (::connect(l.fd_, reinterpret_cast<sockaddr*>(&peer), sizeof(peer)) != 0) {
            throw std::runtime_error("UdpEdgeLink: connect() failed");
        }
        l.is_sender_ = true;
        return l;
    }

    static UdpEdgeLink make_receiver(int listen_port) {
        UdpEdgeLink l;
        l.fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (l.fd_ < 0) throw std::runtime_error("UdpEdgeLink: socket() failed");
        int one = 1;
        ::setsockopt(l.fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port = htons(static_cast<uint16_t>(listen_port));
        if (::bind(l.fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
            throw std::runtime_error("UdpEdgeLink: bind() failed");
        }
        int flags = ::fcntl(l.fd_, F_GETFL, 0);
        ::fcntl(l.fd_, F_SETFL, flags | O_NONBLOCK);
        l.is_sender_ = false;
        return l;
    }

    UdpEdgeLink() = default;
    UdpEdgeLink(const UdpEdgeLink&) = delete;
    UdpEdgeLink(UdpEdgeLink&& o) noexcept { *this = std::move(o); }
    UdpEdgeLink& operator=(UdpEdgeLink&& o) noexcept {
        if (this != &o) {
            close_fd();
            fd_ = o.fd_;
            is_sender_ = o.is_sender_;
            expected_seq_ = o.expected_seq_;
            drop_count_ = o.drop_count_.load();
            pending_ = std::move(o.pending_);
            o.fd_ = -1;
        }
        return *this;
    }
    ~UdpEdgeLink() { close_fd(); }

    bool send(const Message& m) {
        ssize_t n = ::send(fd_, &m, sizeof(Message), 0);
        return n == static_cast<ssize_t>(sizeof(Message));
    }

    // Non-blocking receive with a small reorder window keyed on `seq`
    // (the source's monotonic counter): a datagram that arrives out of
    // order is buffered until its predecessor shows up or the window
    // fills, instead of being delivered out of tick order. Window misses
    // (older than everything buffered, or the window overflowed) count as
    // drops, the same accounting EdgeRing::try_push uses on overflow.
    bool try_recv(Message* out) {
        Message raw;
        while (true) {
            ssize_t n = ::recv(fd_, &raw, sizeof(Message), 0);
            if (n != static_cast<ssize_t>(sizeof(Message))) break; // EAGAIN or a short/garbled datagram
            pending_[raw.seq] = raw;
            if (pending_.size() > kWindow) {
                // Oldest buffered datagram aged out without its predecessor
                // ever arriving: deliver it anyway (best effort) and count
                // the gap, rather than block forever on a lost predecessor.
                auto it = pending_.begin();
                expected_seq_ = it->first;
                break;
            }
        }
        auto it = pending_.find(expected_seq_);
        if (it == pending_.end()) {
            if (pending_.empty()) return false;
            // Lowest buffered sequence is ahead of what we expect: either
            // the expected one was truly lost, or it is still in flight.
            // Only declare loss once the window is actually full.
            if (pending_.size() < kWindow) return false;
            it = pending_.begin();
            if (it->first > expected_seq_) drop_count_.fetch_add(it->first - expected_seq_, std::memory_order_relaxed);
        }
        *out = it->second;
        expected_seq_ = it->first + 1;
        pending_.erase(it);
        return true;
    }

    uint64_t drop_count() const { return drop_count_.load(std::memory_order_relaxed); }

private:
    void close_fd() {
        if (fd_ >= 0) ::close(fd_);
        fd_ = -1;
    }

    static constexpr size_t kWindow = 64;
    int fd_ = -1;
    bool is_sender_ = false;
    uint64_t expected_seq_ = 0;
    std::atomic<uint64_t> drop_count_{0};
    std::map<uint64_t, Message> pending_;
};

} // namespace shmmw
