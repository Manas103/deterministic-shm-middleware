#pragma once
#include <atomic>
#include <cstdint>

#include "shmmw/perception/frame_gen.hpp"

namespace shmmw::perception {

// A single-slot POSIX-shared-memory mailbox between the perception process
// and the OpenGL viewer process: the smallest thing that can hand a host
// byte buffer across an OS process boundary without reusing the generic
// 64-byte shmmw::Message (an RGBA image does not fit in four doubles).
// Lockstep by design (the producer waits for the viewer to have consumed
// frame k before writing k+1), since the viewer demo's purpose is to prove
// every frame was actually read, not to sustain the 18,000-frame
// throughput benchmark (that one never leaves perception_bench).
struct FrameMailbox {
    std::atomic<uint64_t> write_seq{0};
    std::atomic<uint64_t> read_seq{0};
    uint64_t frame_idx = 0;
    uint64_t producer_checksum = 0; // simple additive checksum of image bytes
    uint8_t image[kImgW * kImgH * 4];
};

inline uint64_t additive_checksum(const uint8_t* data, size_t n) {
    uint64_t s = 0;
    for (size_t i = 0; i < n; ++i) s = s * 1099511628211ULL + data[i];
    return s;
}

} // namespace shmmw::perception
