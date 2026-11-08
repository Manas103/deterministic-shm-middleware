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
#include "shmmw/mixed_transport.hpp"
#include "shmmw/ring_buffer.hpp"
#include "shmmw/scenario.hpp"
#include "shmmw/shm_names.hpp"
#include "shmmw/shm_segment.hpp"
#include "shmmw/shm_transport.hpp"
#include "shmmw/stats.hpp"
#include "shmmw/udp_edge_link.hpp"

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
    int bridge_input_index = -1, bridge_output_index = -1;
    auto attach_edge = [&](const std::string& edge) -> void* {
        if (!a.bridge_edge.empty() && edge == a.bridge_edge) return nullptr; // crosses the host boundary over UDP
        const uint32_t cap = a.edge_cap_override.count(edge) ? a.edge_cap_override.at(edge) : kDefaultEdgeCapacity;
        segs.push_back(ShmSegment::attach(shm_edge_name(a.run_id, edge), EdgeRing::size_for(cap)));
        return segs.back().data();
    };
    for (size_t i = 0; i < bc->input_edges.size(); ++i) {
        if (!a.bridge_edge.empty() && bc->input_edges[i] == a.bridge_edge) bridge_input_index = static_cast<int>(i);
        in_mems.push_back(attach_edge(bc->input_edges[i]));
    }
    for (size_t i = 0; i < bc->output_edges.size(); ++i) {
        if (!a.bridge_edge.empty() && bc->output_edges[i] == a.bridge_edge) bridge_output_index = static_cast<int>(i);
        out_mems.push_back(attach_edge(bc->output_edges[i]));
    }

    const bool this_block_has_bridge = bridge_input_index >= 0 || bridge_output_index >= 0;
    std::unique_ptr<ITransport> transport;
    if (!this_block_has_bridge) {
        transport = std::make_unique<ShmTransport>(in_mems, out_mems);
    } else {
        std::unique_ptr<UdpEdgeLink> link;
        if (a.bridge_role == "sender") {
            link = std::make_unique<UdpEdgeLink>(UdpEdgeLink::make_sender(a.bridge_peer_ip, a.bridge_peer_port));
        } else if (a.bridge_role == "receiver") {
            link = std::make_unique<UdpEdgeLink>(UdpEdgeLink::make_receiver(a.bridge_listen_port));
        } else {
            std::fprintf(stderr, "--bridge-edge given without --bridge-role sender|receiver\n");
            return 1;
        }
        transport = std::make_unique<MixedTransport>(in_mems, out_mems, bridge_input_index, bridge_output_index,
                                                       std::move(link));
    }

    if (a.num_blocks > 0) {
        ShmSegment barrier_seg = ShmSegment::attach(shm_barrier_name(a.run_id), sizeof(StartBarrier));
        auto* barrier = reinterpret_cast<StartBarrier*>(barrier_seg.data());
        barrier->ready_count.fetch_add(1, std::memory_order_acq_rel);
        while (barrier->ready_count.load(std::memory_order_acquire) < a.num_blocks) usleep(200);
    }

    BlockIo io{&log, &hop_log, stats};
    return run_block(g, *bc, *kernel, *transport, io, a.engine);
}
