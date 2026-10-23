// Runs N synthetic frames through the GPU-resident perception pipeline and
// reports: (a) p50 per-frame latency for the optimized (device-resident)
// path and the host-round-trip baseline, over the same frame count, and
// (b) with --digest, the SHA-256 session digest of every frame's final
// image and voxel grid, for the 100-session replay-determinism check
// (tools/run_perception_replay.py).
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include <chrono>

#include "shmmw/perception/frame_gen.hpp"
#include "shmmw/perception/pipeline_api.hpp"
#include "shmmw/sha256.hpp"

using namespace shmmw::perception;

static double percentile(std::vector<double>& v, double p) {
    std::sort(v.begin(), v.end());
    size_t idx = (size_t)(p / 100.0 * (v.size() - 1));
    return v[idx];
}

int main(int argc, char** argv) {
    uint64_t seed = 777;
    int n_frames = 1000;
    std::string mode = "latency"; // latency | digest
    bool host_round_trip = false;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--seed" && i + 1 < argc) seed = strtoull(argv[++i], nullptr, 10);
        else if (a == "--frames" && i + 1 < argc) n_frames = atoi(argv[++i]);
        else if (a == "--mode" && i + 1 < argc) mode = argv[++i];
        else if (a == "--host-round-trip") host_round_trip = true;
    }

    GpuPipeline pipeline;
    fprintf(stderr, "device: %s\n", pipeline.device_info().c_str());

    std::vector<LidarPoint> points(kNumPoints);
    std::vector<uint8_t> camera_bg(kImgW * kImgH * 4);
    std::vector<uint8_t> image(kImgW * kImgH * 4);
    std::vector<int32_t> voxels(kVoxelDim * kVoxelDim * kVoxelDim);

    // Warmup: excludes one-time CUDA context setup from the timed region.
    generate_frame(seed, 0, points.data(), camera_bg.data());
    pipeline.run_frame(points.data(), camera_bg.data(), image.data(), nullptr, host_round_trip);

    if (mode == "digest") {
        shmmw::Sha256 hasher;
        for (int f = 0; f < n_frames; ++f) {
            generate_frame(seed, f, points.data(), camera_bg.data());
            pipeline.run_frame(points.data(), camera_bg.data(), image.data(), voxels.data(), host_round_trip);
            hasher.update(image.data(), image.size());
            hasher.update(voxels.data(), voxels.size() * sizeof(int32_t));
        }
        printf("%s\n", hasher.hex_digest().c_str());
        return 0;
    }

    std::vector<double> latencies_ms;
    latencies_ms.reserve(n_frames);
    for (int f = 0; f < n_frames; ++f) {
        generate_frame(seed, f, points.data(), camera_bg.data());
        auto t0 = std::chrono::steady_clock::now();
        pipeline.run_frame(points.data(), camera_bg.data(), image.data(), nullptr, host_round_trip);
        auto t1 = std::chrono::steady_clock::now();
        latencies_ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
    }

    double p50 = percentile(latencies_ms, 50);
    double p99 = percentile(latencies_ms, 99);
    printf("mode=%s host_round_trip=%d frames=%d p50_ms=%.4f p99_ms=%.4f\n",
           mode.c_str(), (int)host_round_trip, n_frames, p50, p99);
    return 0;
}
