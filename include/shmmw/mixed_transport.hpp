#pragma once
#include <memory>
#include <vector>

#include "shmmw/ring_buffer.hpp"
#include "shmmw/transport.hpp"
#include "shmmw/udp_edge_link.hpp"

namespace shmmw {

// ITransport over a mix of local shared-memory rings and (at most one) UDP
// bridge edge, so that a single block near a host boundary (a merge block
// whose other inputs stay local, a transform block whose one input is the
// bridge) does not need a special-cased engine; block_engine.cpp and
// kernels never know this transport exists, exactly as the ROS 2 and shm
// transports already do not know about each other.
class MixedTransport final : public ITransport {
public:
    MixedTransport(std::vector<void*> input_mems, std::vector<void*> output_mems, int bridge_input_index,
                    int bridge_output_index, std::unique_ptr<UdpEdgeLink> bridge_link)
        : bridge_input_index_(bridge_input_index),
          bridge_output_index_(bridge_output_index),
          bridge_link_(std::move(bridge_link)) {
        for (size_t i = 0; i < input_mems.size(); ++i) {
            if (static_cast<int>(i) == bridge_input_index_) {
                inputs_.emplace_back(); // placeholder, never attached or used
            } else {
                inputs_.emplace_back(input_mems[i]);
            }
        }
        for (size_t i = 0; i < output_mems.size(); ++i) {
            if (static_cast<int>(i) == bridge_output_index_) {
                outputs_.emplace_back();
            } else {
                outputs_.emplace_back(output_mems[i]);
            }
        }
    }

    bool try_recv(size_t input, Message* out) override {
        if (static_cast<int>(input) == bridge_input_index_) return bridge_link_->try_recv(out);
        return inputs_[input].try_pop(out);
    }

    bool send(size_t output, const Message& m) override {
        if (static_cast<int>(output) == bridge_output_index_) return bridge_link_->send(m);
        return outputs_[output].try_push(m);
    }

    uint64_t input_drops(size_t i) const override {
        if (static_cast<int>(i) == bridge_input_index_) return bridge_link_->drop_count();
        return inputs_[i].drop_count().load(std::memory_order_relaxed);
    }
    uint64_t input_backpressure(size_t i) const override {
        if (static_cast<int>(i) == bridge_input_index_) return 0; // UDP has no backpressure concept
        return inputs_[i].backpressure_events().load(std::memory_order_relaxed);
    }

private:
    mutable std::vector<EdgeRing> inputs_;
    std::vector<EdgeRing> outputs_;
    int bridge_input_index_;  // -1 if this block has no bridge input
    int bridge_output_index_; // -1 if this block has no bridge output
    std::unique_ptr<UdpEdgeLink> bridge_link_;
};

} // namespace shmmw
