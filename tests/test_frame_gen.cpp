#include <cstdio>
#include <cstring>
#include <vector>

#include "shmmw/perception/frame_gen.hpp"

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
    std::vector<LidarPoint> p1(kNumPoints), p2(kNumPoints);
    std::vector<uint8_t> bg1(kImgW * kImgH * 4), bg2(kImgW * kImgH * 4);

    generate_frame(42, 5, p1.data(), bg1.data());
    generate_frame(42, 5, p2.data(), bg2.data());
    CHECK(memcmp(p1.data(), p2.data(), p1.size() * sizeof(LidarPoint)) == 0);
    CHECK(memcmp(bg1.data(), bg2.data(), bg1.size()) == 0);

    generate_frame(42, 6, p2.data(), bg2.data());
    CHECK(memcmp(p1.data(), p2.data(), p1.size() * sizeof(LidarPoint)) != 0);

    generate_frame(43, 5, p2.data(), bg2.data());
    CHECK(memcmp(p1.data(), p2.data(), p1.size() * sizeof(LidarPoint)) != 0);

    // Every generated point must land inside the sensing volume the
    // voxelizer actually covers, or the pipeline would silently drop a
    // large fraction of every frame.
    generate_frame(1, 0, p1.data(), bg1.data());
    int inside = 0;
    for (const auto& p : p1) {
        if (p.x >= kVolumeMin[0] && p.x <= kVolumeMax[0] &&
            p.y >= kVolumeMin[1] && p.y <= kVolumeMax[1] &&
            p.z >= kVolumeMin[2] && p.z <= kVolumeMax[2]) {
            inside++;
        }
    }
    CHECK(inside == kNumPoints);

    printf("%d checks, %d failed\n", g_checks, g_failed);
    return g_failed == 0 ? 0 : 1;
}
