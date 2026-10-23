#pragma once
#include <cstdint>

#include "shmmw/perception/frame_gen.hpp"

namespace shmmw::perception {

// A deliberately slow, obviously-correct serial implementation of the same
// voxelize -> project -> composite pipeline the GPU kernels compute, used
// only to diff the GPU path exactly, never for a measured path.
void run_frame_cpu_reference(const LidarPoint* points, const uint8_t* camera_bg_rgba,
                              uint8_t* out_image_rgba, int32_t* out_voxel_grid);

} // namespace shmmw::perception
