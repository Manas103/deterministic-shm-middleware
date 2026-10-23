#pragma once
#include <cstdint>
#include <memory>
#include <string>

#include "shmmw/perception/frame_gen.hpp"

// The CUDA/non-CUDA boundary for the perception pipeline: only kernels.cu
// (compiled by nvcc) includes <cuda_runtime.h>; every other translation
// unit, including the process main()s, sees only this header.
namespace shmmw::perception {

class GpuPipeline {
public:
    GpuPipeline();
    ~GpuPipeline();
    GpuPipeline(const GpuPipeline&) = delete;
    GpuPipeline& operator=(const GpuPipeline&) = delete;

    // Runs voxelize -> project -> composite for one already-generated
    // frame. `host_round_trip` selects which path: false keeps the voxel
    // grid and the per-pixel accumulator device-resident across the three
    // kernels (the optimized path); true bounces each of those two
    // intermediate buffers through a host copy and back between kernels,
    // simulating a design where every stage hands off through host memory.
    // `out_image_rgba` (kImgW*kImgH*4 bytes) and `out_voxel_grid`
    // (kVoxelDim^3 int32s, may be null) are host buffers this call fills.
    void run_frame(const LidarPoint* points, const uint8_t* camera_bg_rgba,
                   uint8_t* out_image_rgba, int32_t* out_voxel_grid,
                   bool host_round_trip);

    std::string device_info() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace shmmw::perception
