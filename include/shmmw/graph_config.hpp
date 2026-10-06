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
    std::vector<std::string> output_edges; // edge names, size 0..N
    // Which compute kernel this block runs (see kernels.hpp). Empty means the
    // generic kernel for the block type (sine source, linear transform, sum
    // merge), which is what the original 12-block benchmark graph uses.
    std::string kernel;
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
//   <id> <name> <source|transform|merge|sink> <in1,in2|-> <out1,...|-> [kernel]
// The optional sixth column names a kernel from kernels.hpp (for example
// `ego_bicycle` or `sensor:a1`); omitted or `-` selects the generic kernel.
GraphConfig load_graph_config(const std::string& path);

// True when every block's inputs are produced by a block that appears
// earlier in the file. The sequential reference oracle relies on this, and
// both shipped configs satisfy it; a config that does not is rejected by the
// oracle rather than silently evaluated in the wrong order.
bool is_topologically_ordered(const GraphConfig& g, std::string* why = nullptr);

} // namespace shmmw
