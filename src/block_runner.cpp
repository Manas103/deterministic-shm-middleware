// block_runner: the shared-memory transport's per-block executable. It
// attaches the block's edge rings and stats segment, waits at the start
// barrier, then hands the block to the transport-agnostic engine
// (block_engine.cpp). Which kernel the block runs is decided entirely by its
// static config entry; there is no code path that lets a running block
// change what it does. The ROS 2 transport's equivalent is
// ros2/shmmw_ros2/src/ros2_block_node.cpp; the two differ only in the
// ITransport they construct.
#include <unistd.h>

#include <cstdio>
#include <exception>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "shmmw/block_engine.hpp"
#include "shmmw/block_args.hpp"
#include "shmmw/graph_config.hpp"
#include "shmmw/kernels.hpp"
#include "shmmw/ring_buffer.hpp"
#include "shmmw/scenario.hpp"
#include "shmmw/shm_names.hpp"
#include "shmmw/shm_segment.hpp"
#include "shmmw/shm_transport.hpp"
#include "shmmw/stats.hpp"

using namespace shmmw;

int main(int argc, char** argv) {
    BlockArgs a = parse_block_args(argc, argv);
    GraphConfig g = load_graph_config(a.config_path);
    const BlockConfig* bc = g.find_block(a.block_name);
    if (!bc) { std::fprintf(stderr, "unknown block %s\n", a.block_name.c_str()); return 1; }

    std::unique_ptr<Scenario> scenario;
    std::unique_ptr<Kernel> kernel;
    try {
        if (!a.scenario_path.empty()) scenario = std::make_unique<Scenario>(load_scenario(a.scenario_path));
        kernel = make_kernel(*bc, scenario.get());
    } catch (const std::exception& e) {
        std::fprintf(stderr, "%s: %s\n", a.block_name.c_str(), e.what());
        return 1;
    }

    ShmSegment stats_seg = ShmSegment::attach(shm_stats_name(a.run_id, a.block_name), sizeof(BlockStats));
    auto* stats = reinterpret_cast<BlockStats*>(stats_seg.data());

    std::ofstream log(a.log_path, std::ios::trunc);
    std::ofstream hop_log(a.log_path + ".hop", std::ios::trunc); // excluded from the session digest

    std::vector<ShmSegment> segs;
    std::vector<void*> in_mems, out_mems;
    auto attach_edge = [&](const std::string& edge) {
        const uint32_t cap = a.edge_cap_override.count(edge) ? a.edge_cap_override.at(edge) : kDefaultEdgeCapacity;
        segs.push_back(ShmSegment::attach(shm_edge_name(a.run_id, edge), EdgeRing::size_for(cap)));
        return segs.back().data();
    };
    for (auto& e : bc->input_edges) in_mems.push_back(attach_edge(e));
    for (auto& e : bc->output_edges) out_mems.push_back(attach_edge(e));
    ShmTransport transport(in_mems, out_mems);

    if (a.num_blocks > 0) {
        ShmSegment barrier_seg = ShmSegment::attach(shm_barrier_name(a.run_id), sizeof(StartBarrier));
        auto* barrier = reinterpret_cast<StartBarrier*>(barrier_seg.data());
        barrier->ready_count.fetch_add(1, std::memory_order_acq_rel);
        while (barrier->ready_count.load(std::memory_order_acquire) < a.num_blocks) usleep(200);
    }

    BlockIo io{&log, &hop_log, stats};
    return run_block(g, *bc, *kernel, transport, io, a.engine);
}
