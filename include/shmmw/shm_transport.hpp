#pragma once
#include <vector>

#include "shmmw/ring_buffer.hpp"
#include "shmmw/transport.hpp"

namespace shmmw {

// ITransport over EdgeRing. The rings' memory is owned by the caller: POSIX
// shared memory segments in block_runner, heap buffers in the threaded test
// harness (tests/test_engine.cpp). The class only ever touches the raw
// pointers, which is why the same code path is exercised by both.
class ShmTransport final : public ITransport {
public:
    ShmTransport(std::vector<void*> input_mems, std::vector<void*> output_mems) {
        for (void* m : input_mems) inputs_.emplace_back(m);
        for (void* m : output_mems) outputs_.emplace_back(m);
    }

    bool try_recv(size_t input, Message* out) override { return inputs_[input].try_pop(out); }
    bool send(size_t output, const Message& m) override { return outputs_[output].try_push(m); }
    uint64_t input_drops(size_t i) const override {
        return inputs_[i].drop_count().load(std::memory_order_relaxed);
    }
    uint64_t input_backpressure(size_t i) const override {
        return inputs_[i].backpressure_events().load(std::memory_order_relaxed);
    }

private:
    // EdgeRing's counter accessors hand out references into shared memory
    // and are non-const; reading a counter does not change the ring.
    mutable std::vector<EdgeRing> inputs_;
    std::vector<EdgeRing> outputs_;
};

} // namespace shmmw
