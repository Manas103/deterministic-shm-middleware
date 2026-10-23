#pragma once
#include <cstdint>
#include <vector>

namespace shmmw::perception {

constexpr int kNumPoints = 300000;   // synthetic LiDAR points per frame (one full Velodyne-scale scan)
constexpr int kVoxelDim = 128;       // voxel grid is kVoxelDim^3
constexpr int kImgW = 512;
constexpr int kImgH = 512;
constexpr float kVolumeMin[3] = {-16.0f, -16.0f, 0.0f};  // meters
constexpr float kVolumeMax[3] = {16.0f, 16.0f, 8.0f};
constexpr int kNumObjects = 3;

struct LidarPoint {
    float x, y, z;
};

// Deterministic counter-based hash -> uniform [0,1); no RNG state carried
// across calls, so frame k of a given (seed) depends only on (seed, k) and
// nothing about call order, which is the whole basis of the replay digest.
inline uint64_t splitmix64(uint64_t x) {
    x += 0x9E3779B97f4A7C15ULL;
    uint64_t z = x;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31);
}
inline float uniform01(uint64_t seed, uint64_t stream) {
    uint64_t h = splitmix64(seed ^ (stream * 0xD1B54A32D192ED03ULL));
    return (float)((h >> 11) * (1.0 / 9007199254740992.0));
}

// Fills `points` (kNumPoints) with a synthetic LiDAR scan: kNumObjects
// clusters of points on moving spherical "vehicles" inside the sensing
// volume, and `camera_bg` (kImgW*kImgH RGBA bytes) with a static synthetic
// background gradient standing in for a camera frame. Everything here is a
// pure function of (seed, frame_idx); there is no sensor hardware and no
// randomness beyond the seeded hash above.
void generate_frame(uint64_t seed, uint64_t frame_idx, LidarPoint* points, uint8_t* camera_bg_rgba);

} // namespace shmmw::perception
