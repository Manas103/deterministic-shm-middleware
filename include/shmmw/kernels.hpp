#pragma once
#include <cstdint>
#include <memory>
#include <string>

#include "shmmw/graph_config.hpp"
#include "shmmw/scenario.hpp"

namespace shmmw {

// The compute a block performs, separated from how its messages move. A
// kernel only ever sees the logical tick and the four payload doubles; it
// never sees a ring, a ROS 2 topic, a timestamp or a sequence number. That
// separation is the whole reason the shared-memory and ROS 2 transports can
// be compared bit for bit: they run the identical kernel object code on the
// identical inputs, and only the delivery mechanism differs.
//
// A kernel may keep state across ticks (the ego integrator does). It is
// called exactly once per tick, in tick order, by whichever transport
// drives it.
class Kernel {
public:
    virtual ~Kernel() = default;
    // Source blocks. jitter_ns is the live pacing error of this tick (record
    // mode only); kernels that must be a pure function of the scenario
    // ignore it.
    virtual void produce(uint64_t tick, int64_t jitter_ns, double out[4]);
    // Transform blocks: rewrite the payload in place.
    virtual void step(uint64_t tick, double payload[4]);
    // Merge blocks: in0/in1 are the block's first and second configured
    // input edges, always in that order regardless of arrival order.
    virtual void combine(uint64_t tick, const double in0[4], const double in1[4], double out[4]);
};

// Builds the kernel named by the block's config entry (`bc.kernel`), or the
// generic kernel for its type when that is empty. AV kernels require a
// scenario; passing nullptr for one throws. Throws std::runtime_error for an
// unknown kernel name or a kernel used on the wrong block type.
//
// Kernel names:
//   (empty)          generic: sine/cosine source, per-block linear transform,
//                    elementwise-sum merge, pass-through sink
//   av_source        emits [t, accel_cmd, steer_cmd, tick] from the scenario
//   ego_bicycle      kinematic bicycle, RK4 at the scenario dt -> [x, y, yaw, v]
//   agent:<name>     scripted traffic agent at t+dt -> [x, y, vx, vy]
//   sensor:<name>    merge(ego, agent) -> [range, bearing, range_rate, id]
//   nearest          merge(obs, obs) -> the nearer detected observation
//   ttc              transform obs -> [range, range_rate, ttc_or_-1, id]
std::unique_ptr<Kernel> make_kernel(const BlockConfig& bc, const Scenario* scenario);

// True when any block in the graph uses a kernel that needs a scenario.
bool graph_needs_scenario(const GraphConfig& g);

} // namespace shmmw
