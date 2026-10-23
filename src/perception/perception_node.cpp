// The producer side of the OpenGL-viewer demo: runs the GPU-resident
// pipeline for a small number of frames, writes each final image into a
// shared-memory mailbox, and waits (lockstep) for the viewer process to
// confirm it read that exact frame before moving to the next one.
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>
#include <vector>

#include "shmmw/perception/frame_gen.hpp"
#include "shmmw/perception/frame_mailbox.hpp"
#include "shmmw/perception/pipeline_api.hpp"
#include "shmmw/shm_segment.hpp"

using namespace shmmw::perception;

static const char* kSegmentName = "/shmmw_perception_mailbox";

int main(int argc, char** argv) {
    uint64_t seed = 777;
    int n_frames = 20;
    if (argc > 1) n_frames = atoi(argv[1]);
    if (argc > 2) seed = strtoull(argv[2], nullptr, 10);

    auto seg = shmmw::ShmSegment::create(kSegmentName, sizeof(FrameMailbox));
    auto* mailbox = new (seg.data()) FrameMailbox();

    GpuPipeline pipeline;
    std::vector<LidarPoint> points(kNumPoints);
    std::vector<uint8_t> camera_bg(kImgW * kImgH * 4);

    for (int f = 0; f < n_frames; ++f) {
        generate_frame(seed, f, points.data(), camera_bg.data());
        pipeline.run_frame(points.data(), camera_bg.data(), mailbox->image, nullptr, /*host_round_trip=*/false);
        mailbox->producer_checksum = additive_checksum(mailbox->image, sizeof(mailbox->image));
        mailbox->frame_idx = f;
        mailbox->write_seq.fetch_add(1, std::memory_order_release);

        int waited_ms = 0;
        while (mailbox->read_seq.load(std::memory_order_acquire) < mailbox->write_seq.load(std::memory_order_acquire)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            if (++waited_ms > 5000) {
                fprintf(stderr, "viewer did not consume frame %d within 5s, giving up\n", f);
                return 1;
            }
        }
        printf("frame %d produced, checksum=%llu, confirmed consumed by viewer\n",
               f, (unsigned long long)mailbox->producer_checksum);
    }
    printf("perception_node: all %d frames consumed\n", n_frames);
    return 0;
}
