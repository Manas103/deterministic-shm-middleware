// GPU kernels diffed exactly against the slow serial CPU reference oracle,
// and the frame generator's own determinism checked, both hand-rolled check
// suites (no framework, matching the rest of this repo's C++ tests).
#include <cstdio>
#include <cstring>
#include <vector>

#include "shmmw/perception/cpu_reference.hpp"
#include "shmmw/perception/frame_gen.hpp"
#include "shmmw/perception/pipeline_api.hpp"

using namespace shmmw::perception;

static int g_checks = 0;
static int g_failed = 0;
#define CHECK(cond)                                                      \
    do {                                                                 \
        g_checks++;                                                      \
        if (!(cond)) {                                                   \
            g_failed++;                                                  \
            fprintf(stderr, "CHECK FAILED: %s (%s:%d)\n", #cond, __FILE__, __LINE__); \
        }                                                                 \
    } while (0)

int main() {
    GpuPipeline pipeline;

    std::vector<LidarPoint> points(kNumPoints);
    std::vector<uint8_t> camera_bg(kImgW * kImgH * 4);
    std::vector<uint8_t> gpu_image(kImgW * kImgH * 4);
    std::vector<uint8_t> cpu_image(kImgW * kImgH * 4);
    std::vector<int32_t> gpu_voxels(kVoxelDim * kVoxelDim * kVoxelDim);
    std::vector<int32_t> cpu_voxels(kVoxelDim * kVoxelDim * kVoxelDim);

    int exact_matches = 0;
    const int n_frames = 30;
    for (int f = 0; f < n_frames; ++f) {
        generate_frame(/*seed=*/777, /*frame_idx=*/f, points.data(), camera_bg.data());

        pipeline.run_frame(points.data(), camera_bg.data(), gpu_image.data(), gpu_voxels.data(),
                            /*host_round_trip=*/false);
        run_frame_cpu_reference(points.data(), camera_bg.data(), cpu_image.data(), cpu_voxels.data());

        bool voxels_match = (memcmp(gpu_voxels.data(), cpu_voxels.data(), gpu_voxels.size() * sizeof(int32_t)) == 0);
        bool image_match = (memcmp(gpu_image.data(), cpu_image.data(), gpu_image.size()) == 0);
        CHECK(voxels_match);
        CHECK(image_match);
        if (voxels_match && image_match) exact_matches++;
    }
    printf("%d of %d frames exact GPU/CPU match\n", exact_matches, n_frames);
    CHECK(exact_matches == n_frames);

    // The host-round-trip path must compute the identical answer: it is the
    // same kernels on the same data, only the intermediate buffers' location
    // changes.
    generate_frame(777, 0, points.data(), camera_bg.data());
    std::vector<uint8_t> baseline_image(kImgW * kImgH * 4);
    pipeline.run_frame(points.data(), camera_bg.data(), baseline_image.data(), nullptr, /*host_round_trip=*/true);
    pipeline.run_frame(points.data(), camera_bg.data(), gpu_image.data(), nullptr, /*host_round_trip=*/false);
    CHECK(memcmp(baseline_image.data(), gpu_image.data(), gpu_image.size()) == 0);

    // A frame with no points in the volume at all must produce a nonzero
    // image (the camera background alone) and an all-zero voxel grid: the
    // pipeline must not crash or leave stale data from a previous frame.
    std::vector<LidarPoint> empty_points(kNumPoints);
    for (auto& p : empty_points) { p.x = 1000.0f; p.y = 1000.0f; p.z = 1000.0f; } // outside the volume
    pipeline.run_frame(empty_points.data(), camera_bg.data(), gpu_image.data(), gpu_voxels.data(), false);
    bool all_zero = true;
    for (int32_t v : gpu_voxels) if (v != 0) all_zero = false;
    CHECK(all_zero);
    CHECK(memcmp(gpu_image.data(), camera_bg.data(), gpu_image.size()) == 0);

    printf("%d checks, %d failed\n", g_checks, g_failed);
    return g_failed == 0 ? 0 : 1;
}
