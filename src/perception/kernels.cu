#include <cuda_runtime.h>

#include <cstring>
#include <sstream>
#include <stdexcept>

#include "shmmw/perception/pipeline_api.hpp"

namespace {

#define CUDA_CHECK(expr)                                                     \
    do {                                                                     \
        cudaError_t _e = (expr);                                             \
        if (_e != cudaSuccess) {                                             \
            std::ostringstream _oss;                                         \
            _oss << "CUDA error at " << __FILE__ << ":" << __LINE__ << ": "  \
                 << cudaGetErrorString(_e);                                  \
            throw std::runtime_error(_oss.str());                           \
        }                                                                    \
    } while (0)

using shmmw::perception::kImgH;
using shmmw::perception::kImgW;
using shmmw::perception::kNumPoints;
using shmmw::perception::kVoxelDim;
using shmmw::perception::kVolumeMax;
using shmmw::perception::kVolumeMin;
using shmmw::perception::LidarPoint;

// One thread per LiDAR point: bins it into the voxel grid. atomicAdd on an
// int32 is a well-defined, order-independent sum, so this kernel gives the
// same voxel grid every time regardless of which thread lands on a shared
// voxel first or in what warp-scheduling order, which is what the replay
// digest depends on.
__global__ void voxelize_kernel(const LidarPoint* points, int n, int32_t* voxel_grid, int voxel_dim,
                                 float3 vmin, float3 vmax) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;
    LidarPoint p = points[i];
    float3 size = make_float3(vmax.x - vmin.x, vmax.y - vmin.y, vmax.z - vmin.z);
    int vx = (int)(((p.x - vmin.x) / size.x) * voxel_dim);
    int vy = (int)(((p.y - vmin.y) / size.y) * voxel_dim);
    int vz = (int)(((p.z - vmin.z) / size.z) * voxel_dim);
    if (vx < 0 || vx >= voxel_dim || vy < 0 || vy >= voxel_dim || vz < 0 || vz >= voxel_dim) return;
    int idx = (vz * voxel_dim + vy) * voxel_dim + vx;
    atomicAdd(&voxel_grid[idx], 1);
}

// One thread per voxel: a bird's-eye-view projection into the image plane,
// collapsing height (z) by summing every occupied voxel's count into its
// (x, y) pixel. atomicAdd keeps this deterministic for the same reason as
// above.
__global__ void project_kernel(const int32_t* voxel_grid, int voxel_dim, int32_t* accum, int img_w, int img_h) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    int total = voxel_dim * voxel_dim * voxel_dim;
    if (idx >= total) return;
    int32_t count = voxel_grid[idx];
    if (count == 0) return;
    int vx = idx % voxel_dim;
    int vy = (idx / voxel_dim) % voxel_dim;
    int px = (vx * img_w) / voxel_dim;
    int py = (vy * img_h) / voxel_dim;
    atomicAdd(&accum[py * img_w + px], count);
}

// One thread per pixel, pure read, no atomics needed: composites the
// LiDAR density map onto the synthetic camera background's red channel.
__global__ void composite_kernel(const uint8_t* camera_bg, const int32_t* accum, uint8_t* out, int img_w, int img_h) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    int total = img_w * img_h;
    if (idx >= total) return;
    int32_t a = accum[idx];
    int boost = a * 40;
    if (boost > 255) boost = 255;
    out[idx * 4 + 0] = (uint8_t)min(255, (int)camera_bg[idx * 4 + 0] + boost);
    out[idx * 4 + 1] = camera_bg[idx * 4 + 1];
    out[idx * 4 + 2] = camera_bg[idx * 4 + 2];
    out[idx * 4 + 3] = 255;
}

} // namespace

namespace shmmw::perception {

struct GpuPipeline::Impl {
    LidarPoint* d_points = nullptr;
    uint8_t* d_camera_bg = nullptr;
    int32_t* d_voxel_grid = nullptr;
    int32_t* d_accum = nullptr;
    uint8_t* d_image = nullptr;

    // Host-side staging buffers, used only in host_round_trip mode to
    // bounce the intermediate buffers off the host between kernels.
    int32_t* h_voxel_grid = nullptr;
    int32_t* h_accum = nullptr;

    Impl() {
        const int voxel_bytes = kVoxelDim * kVoxelDim * kVoxelDim * sizeof(int32_t);
        const int accum_bytes = kImgW * kImgH * sizeof(int32_t);
        const int image_bytes = kImgW * kImgH * 4;
        CUDA_CHECK(cudaMalloc(&d_points, kNumPoints * sizeof(LidarPoint)));
        CUDA_CHECK(cudaMalloc(&d_camera_bg, image_bytes));
        CUDA_CHECK(cudaMalloc(&d_voxel_grid, voxel_bytes));
        CUDA_CHECK(cudaMalloc(&d_accum, accum_bytes));
        CUDA_CHECK(cudaMalloc(&d_image, image_bytes));
        h_voxel_grid = (int32_t*)malloc(voxel_bytes);
        h_accum = (int32_t*)malloc(accum_bytes);
    }

    ~Impl() {
        cudaFree(d_points);
        cudaFree(d_camera_bg);
        cudaFree(d_voxel_grid);
        cudaFree(d_accum);
        cudaFree(d_image);
        free(h_voxel_grid);
        free(h_accum);
    }
};

GpuPipeline::GpuPipeline() : impl_(std::make_unique<Impl>()) {}
GpuPipeline::~GpuPipeline() = default;

void GpuPipeline::run_frame(const LidarPoint* points, const uint8_t* camera_bg_rgba,
                             uint8_t* out_image_rgba, int32_t* out_voxel_grid, bool host_round_trip) {
    const int voxel_bytes = kVoxelDim * kVoxelDim * kVoxelDim * sizeof(int32_t);
    const int accum_bytes = kImgW * kImgH * sizeof(int32_t);
    const int image_bytes = kImgW * kImgH * 4;

    CUDA_CHECK(cudaMemcpy(impl_->d_points, points, kNumPoints * sizeof(LidarPoint), cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(impl_->d_camera_bg, camera_bg_rgba, image_bytes, cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemset(impl_->d_voxel_grid, 0, voxel_bytes));
    CUDA_CHECK(cudaMemset(impl_->d_accum, 0, accum_bytes));

    float3 vmin = make_float3(kVolumeMin[0], kVolumeMin[1], kVolumeMin[2]);
    float3 vmax = make_float3(kVolumeMax[0], kVolumeMax[1], kVolumeMax[2]);

    int block = 128;
    int grid_pts = (kNumPoints + block - 1) / block;
    voxelize_kernel<<<grid_pts, block>>>(impl_->d_points, kNumPoints, impl_->d_voxel_grid, kVoxelDim, vmin, vmax);
    CUDA_CHECK(cudaGetLastError());

    if (host_round_trip) {
        // The "before" path: bounce the voxel grid through host memory
        // between the two kernels that actually need it on the device,
        // simulating a design where each stage hands its output to the
        // next through host-visible shared memory instead of staying
        // device-resident.
        CUDA_CHECK(cudaMemcpy(impl_->h_voxel_grid, impl_->d_voxel_grid, voxel_bytes, cudaMemcpyDeviceToHost));
        CUDA_CHECK(cudaMemcpy(impl_->d_voxel_grid, impl_->h_voxel_grid, voxel_bytes, cudaMemcpyHostToDevice));
    }

    int total_voxels = kVoxelDim * kVoxelDim * kVoxelDim;
    int grid_vox = (total_voxels + block - 1) / block;
    project_kernel<<<grid_vox, block>>>(impl_->d_voxel_grid, kVoxelDim, impl_->d_accum, kImgW, kImgH);
    CUDA_CHECK(cudaGetLastError());

    if (host_round_trip) {
        CUDA_CHECK(cudaMemcpy(impl_->h_accum, impl_->d_accum, accum_bytes, cudaMemcpyDeviceToHost));
        CUDA_CHECK(cudaMemcpy(impl_->d_accum, impl_->h_accum, accum_bytes, cudaMemcpyHostToDevice));
    }

    int total_pixels = kImgW * kImgH;
    int grid_px = (total_pixels + block - 1) / block;
    composite_kernel<<<grid_px, block>>>(impl_->d_camera_bg, impl_->d_accum, impl_->d_image, kImgW, kImgH);
    CUDA_CHECK(cudaGetLastError());

    CUDA_CHECK(cudaDeviceSynchronize());
    CUDA_CHECK(cudaMemcpy(out_image_rgba, impl_->d_image, image_bytes, cudaMemcpyDeviceToHost));
    if (out_voxel_grid) {
        CUDA_CHECK(cudaMemcpy(out_voxel_grid, impl_->d_voxel_grid, voxel_bytes, cudaMemcpyDeviceToHost));
    }
}

std::string GpuPipeline::device_info() const {
    int device = 0;
    if (cudaGetDevice(&device) != cudaSuccess) return "no CUDA device";
    cudaDeviceProp prop;
    if (cudaGetDeviceProperties(&prop, device) != cudaSuccess) return "no CUDA device";
    std::ostringstream oss;
    oss << prop.name << " (sm_" << prop.major << prop.minor << "), " << prop.multiProcessorCount << " SMs";
    return oss.str();
}

} // namespace shmmw::perception
