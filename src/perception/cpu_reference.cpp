#include "shmmw/perception/cpu_reference.hpp"

#include <algorithm>
#include <cstring>
#include <vector>

namespace shmmw::perception {

void run_frame_cpu_reference(const LidarPoint* points, const uint8_t* camera_bg_rgba,
                              uint8_t* out_image_rgba, int32_t* out_voxel_grid) {
    // Heap, not a stack array: kVoxelDim^3 int32s can be several MB at the
    // sizes this benchmark uses, comfortably past a default 8MB stack once
    // the image accumulator and the caller's own frame buffers are added.
    std::vector<int32_t> voxel_grid(kVoxelDim * kVoxelDim * kVoxelDim, 0);

    float size[3] = {kVolumeMax[0] - kVolumeMin[0], kVolumeMax[1] - kVolumeMin[1], kVolumeMax[2] - kVolumeMin[2]};
    for (int i = 0; i < kNumPoints; ++i) {
        const LidarPoint& p = points[i];
        int vx = (int)(((p.x - kVolumeMin[0]) / size[0]) * kVoxelDim);
        int vy = (int)(((p.y - kVolumeMin[1]) / size[1]) * kVoxelDim);
        int vz = (int)(((p.z - kVolumeMin[2]) / size[2]) * kVoxelDim);
        if (vx < 0 || vx >= kVoxelDim || vy < 0 || vy >= kVoxelDim || vz < 0 || vz >= kVoxelDim) continue;
        voxel_grid[(vz * kVoxelDim + vy) * kVoxelDim + vx] += 1;
    }
    if (out_voxel_grid) {
        memcpy(out_voxel_grid, voxel_grid.data(), voxel_grid.size() * sizeof(int32_t));
    }

    std::vector<int32_t> accum(kImgW * kImgH, 0);
    for (int idx = 0; idx < kVoxelDim * kVoxelDim * kVoxelDim; ++idx) {
        int32_t count = voxel_grid[idx];
        if (count == 0) continue;
        int vx = idx % kVoxelDim;
        int vy = (idx / kVoxelDim) % kVoxelDim;
        int px = (vx * kImgW) / kVoxelDim;
        int py = (vy * kImgH) / kVoxelDim;
        accum[py * kImgW + px] += count;
    }

    for (int idx = 0; idx < kImgW * kImgH; ++idx) {
        int boost = accum[idx] * 40;
        boost = std::min(255, boost);
        out_image_rgba[idx * 4 + 0] = (uint8_t)std::min(255, (int)camera_bg_rgba[idx * 4 + 0] + boost);
        out_image_rgba[idx * 4 + 1] = camera_bg_rgba[idx * 4 + 1];
        out_image_rgba[idx * 4 + 2] = camera_bg_rgba[idx * 4 + 2];
        out_image_rgba[idx * 4 + 3] = 255;
    }
}

} // namespace shmmw::perception
