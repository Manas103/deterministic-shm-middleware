#pragma once
#include <cstddef>
#include <cstdint>

#include "shmmw/message.hpp"

namespace shmmw {

// How a block's messages move, and nothing else. The block engine
// (block_engine.hpp) owns tick bookkeeping, logging, hop-latency stamping
// and calls into the kernel; a transport only has to deliver Message values
// from a block's i-th configured input edge and to its o-th configured
// output edge, each edge in FIFO order.
//
// Two implementations ship:
//   ShmTransport  (shm_transport.hpp)  lock-free SPSC rings in POSIX shared
//                 memory (or plain heap memory, for the in-process threaded
//                 test harness).
//   Ros2Transport (ros2/shmmw_ros2/src/ros2_block_node.cpp)  one rclcpp
//                 publisher per output edge and one subscription per input
//                 edge, a custom shmmw_ros2/msg/BlockMessage on the wire.
//
// The contract the session digest depends on is only "each edge is FIFO and
// lossless for this session"; it is deliberately not "messages from
// different edges arrive in any particular interleaving", because no real
// transport promises that.
class ITransport {
public:
    virtual ~ITransport() = default;

    // Non-blocking receive from input edge `input`; false when nothing is
    // available right now.
    virtual bool try_recv(size_t input, Message* out) = 0;

    // Send on output edge `output`; false when the transport dropped it
    // (shm ring full). A dropped message is counted by the engine.
    virtual bool send(size_t output, const Message& m) = 0;

    // Called by the engine when a receive loop made no progress. The shm
    // transport returns immediately (busy polling is the latency design);
    // the ROS 2 transport blocks in its executor for up to a millisecond so
    // a waiting block is event driven instead of burning a core.
    virtual void idle() {}

    // Cumulative drop and backpressure counters observed on an input edge,
    // for the live stats segment. Transports without the concept return 0.
    virtual uint64_t input_drops(size_t) const { return 0; }
    virtual uint64_t input_backpressure(size_t) const { return 0; }
};

} // namespace shmmw
