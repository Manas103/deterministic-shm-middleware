#include "shmmw/perception/frame_gen.hpp"

#include <cmath>

namespace shmmw::perception {

void generate_frame(uint64_t seed, uint64_t frame_idx, LidarPoint* points, uint8_t* camera_bg_rgba) {
    // Object k's center orbits slowly inside the volume; phase and radius
    // are seeded so different seeds give different traffic, and frame_idx
    // alone advances time (no wall-clock dependence anywhere).
    float centers[kNumObjects][3];
    for (int k = 0; k < kNumObjects; ++k) {
        float phase = uniform01(seed, 10 + k) * 6.2831853f;
        float radius = 4.0f + 6.0f * uniform01(seed, 20 + k);
        float speed = 0.02f + 0.03f * uniform01(seed, 30 + k);
        float t = (float)frame_idx * speed + phase;
        centers[k][0] = radius * cosf(t);
        centers[k][1] = radius * sinf(t);
        centers[k][2] = 1.0f + 2.0f * uniform01(seed, 40 + k);
    }

    int points_per_object = kNumPoints / kNumObjects;
    int idx = 0;
    for (int k = 0; k < kNumObjects; ++k) {
        int n = (k == kNumObjects - 1) ? (kNumPoints - idx) : points_per_object;
        for (int i = 0; i < n; ++i) {
            uint64_t stream = (uint64_t)frame_idx * 1000003ULL + (uint64_t)idx;
            float dx = (uniform01(seed, stream * 3 + 0) - 0.5f) * 2.0f;
            float dy = (uniform01(seed, stream * 3 + 1) - 0.5f) * 2.0f;
            float dz = (uniform01(seed, stream * 3 + 2) - 0.5f) * 1.0f;
            points[idx].x = centers[k][0] + dx;
            points[idx].y = centers[k][1] + dy;
            points[idx].z = centers[k][2] + dz;
            idx++;
        }
    }

    for (int y = 0; y < kImgH; ++y) {
        for (int x = 0; x < kImgW; ++x) {
            uint8_t* p = camera_bg_rgba + (y * kImgW + x) * 4;
            p[0] = (uint8_t)(20 + (x * 60) / kImgW);   // R: horizontal gradient
            p[1] = (uint8_t)(30 + (y * 60) / kImgH);   // G: vertical gradient
            p[2] = 60;                                  // B: constant "sky" tint
            p[3] = 255;
        }
    }
}

} // namespace shmmw::perception
