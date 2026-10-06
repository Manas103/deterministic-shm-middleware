#include "shmmw/graph_config.hpp"

#include <fstream>
#include <iostream>
#include <set>
#include <sstream>

namespace shmmw {

namespace {
std::vector<std::string> split_csv(const std::string& s) {
    std::vector<std::string> out;
    if (s == "-") return out;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, ',')) out.push_back(item);
    return out;
}

BlockType parse_type(const std::string& s) {
    if (s == "source") return BlockType::Source;
    if (s == "transform") return BlockType::Transform;
    if (s == "merge") return BlockType::Merge;
    if (s == "sink") return BlockType::Sink;
    std::cerr << "unknown block type: " << s << "\n";
    std::exit(1);
}
} // namespace

GraphConfig load_graph_config(const std::string& path) {
    std::ifstream f(path);
    if (!f) {
        std::cerr << "could not open graph config: " << path << "\n";
        std::exit(1);
    }
    GraphConfig g;
    std::string line;
    int next_edge_id = 0;
    auto edge_id_for = [&](const std::string& name) -> int {
        int id = g.edge_id(name);
        if (id >= 0) return id;
        g.edges.push_back({next_edge_id, name});
        return next_edge_id++;
    };

    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::stringstream ss(line);
        std::string id_str, name, type_str, in_str, out_str, kernel_str;
        ss >> id_str >> name >> type_str >> in_str >> out_str >> kernel_str;
        if (name.empty()) continue;

        BlockConfig b;
        b.id = std::stoi(id_str);
        b.name = name;
        b.type = parse_type(type_str);
        for (auto& e : split_csv(in_str)) { edge_id_for(e); b.input_edges.push_back(e); }
        for (auto& e : split_csv(out_str)) { edge_id_for(e); b.output_edges.push_back(e); }
        b.kernel = (kernel_str == "-") ? std::string() : kernel_str;
        g.blocks.push_back(b);
    }
    return g;
}

bool is_topologically_ordered(const GraphConfig& g, std::string* why) {
    std::set<std::string> produced;
    for (const auto& b : g.blocks) {
        for (const auto& e : b.input_edges) {
            if (!produced.count(e)) {
                if (why) *why = "block " + b.name + " reads edge " + e + " before any earlier block writes it";
                return false;
            }
        }
        for (const auto& e : b.output_edges) produced.insert(e);
    }
    return true;
}

} // namespace shmmw
