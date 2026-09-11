#pragma once
#include <string>
#include <vector>

namespace shmmw {

enum class BlockType { Source, Transform, Merge, Sink };

struct BlockConfig {
    int id = -1;
    std::string name;
    BlockType type = BlockType::Transform;
    std::vector<std::string> input_edges;  // edge names, size 0/1/2
    std::vector<std::string> output_edges; // edge names, size 0/1
};

struct EdgeConfig {
    int id = -1;
    std::string name;
};

struct GraphConfig {
    std::vector<BlockConfig> blocks;
    std::vector<EdgeConfig> edges;

    int edge_id(const std::string& name) const {
        for (const auto& e : edges) if (e.name == name) return e.id;
        return -1;
    }
    const BlockConfig* find_block(const std::string& name) const {
        for (const auto& b : blocks) if (b.name == name) return &b;
        return nullptr;
    }
};

// Parses the static graph description read at process startup. This is the
// entirety of "statically configured block-and-port runtime": the graph
// topology is fixed text on disk before any process exists, never a runtime
// registration call, so there is no way to add a node while the system runs.
// Format, one block per non-comment line:
//   <id> <name> <source|transform|merge|sink> <in1,in2|-> <out1|->
GraphConfig load_graph_config(const std::string& path);

} // namespace shmmw
