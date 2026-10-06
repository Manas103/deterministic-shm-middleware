#include "shmmw/kernels.hpp"

#include <cmath>
#include <stdexcept>

#include "shmmw/av_model.hpp"

namespace shmmw {

void Kernel::produce(uint64_t, int64_t, double*) { throw std::logic_error("kernel has no produce()"); }
void Kernel::step(uint64_t, double*) { throw std::logic_error("kernel has no step()"); }
void Kernel::combine(uint64_t, const double*, const double*, double*) {
    throw std::logic_error("kernel has no combine()");
}

namespace {

// ---- Generic kernels: byte-for-byte the compute of the original 12-block
// benchmark graph, so its digests and latency numbers stay comparable. ----

class SineSource final : public Kernel {
public:
    void produce(uint64_t tick, int64_t jitter_ns, double out[4]) override {
        out[0] = std::sin(tick * 0.01) * 100.0;
        out[1] = std::cos(tick * 0.017) * 50.0;
        out[2] = static_cast<double>(tick);
        out[3] = static_cast<double>(jitter_ns);
    }
};

// Deterministic per-block linear transform. Coefficients are a fixed
// function of block id, not random, so two runs of the same block always
// apply the same map.
class LinearTransform final : public Kernel {
public:
    explicit LinearTransform(int block_id) : scale_(1.0 + 0.01 * block_id), offset_(0.5 * block_id) {}
    void step(uint64_t, double p[4]) override {
        for (int i = 0; i < 4; ++i) p[i] = p[i] * scale_ + offset_;
    }
private:
    double scale_, offset_;
};

class SumMerge final : public Kernel {
public:
    void combine(uint64_t, const double a[4], const double b[4], double out[4]) override {
        for (int i = 0; i < 4; ++i) out[i] = a[i] + b[i];
    }
};

class PassThrough final : public Kernel {
public:
    void step(uint64_t, double*) override {}
};

// ---- AV profile kernels. ----

class AvSource final : public Kernel {
public:
    explicit AvSource(const Scenario& s) : s_(s) {}
    void produce(uint64_t tick, int64_t, double out[4]) override {
        const EgoCommand& c = s_.command_at(tick);
        out[0] = static_cast<double>(tick) * s_.dt;
        out[1] = c.accel;
        out[2] = c.steer;
        out[3] = static_cast<double>(tick);
    }
private:
    const Scenario& s_;
};

// The one stateful kernel: it owns the ego state and advances it by exactly
// one fixed dt per tick, using the command carried in the message (not a
// fresh scenario lookup, so a replayed trace drives it identically).
class EgoBicycle final : public Kernel {
public:
    explicit EgoBicycle(const Scenario& s) : s_(s) {
        state_.x = s.ego_x; state_.y = s.ego_y; state_.yaw = s.ego_yaw; state_.v = s.ego_v;
    }
    void step(uint64_t, double p[4]) override {
        state_ = av::bicycle_rk4(state_, p[1], p[2], s_.wheelbase, s_.max_steer, s_.dt);
        p[0] = state_.x; p[1] = state_.y; p[2] = state_.yaw; p[3] = state_.v;
    }
private:
    const Scenario& s_;
    av::EgoState state_;
};

// Reports the agent at the end of the tick, t = (tick + 1) * dt, which is the
// same instant the ego state leaving EgoBicycle for this tick describes.
class ScriptedAgent final : public Kernel {
public:
    ScriptedAgent(const Scenario& s, int idx) : s_(s), idx_(idx) {}
    void step(uint64_t tick, double p[4]) override {
        const double t = static_cast<double>(tick + 1) * s_.dt;
        const av::AgentState a = av::agent_at(s_.agents[idx_].waypoints, t);
        p[0] = a.x; p[1] = a.y; p[2] = a.vx; p[3] = a.vy;
    }
private:
    const Scenario& s_;
    size_t idx_;
};

class RangeBearingSensor final : public Kernel {
public:
    RangeBearingSensor(const Scenario& s, int id) : s_(s), id_(id) {}
    void combine(uint64_t tick, const double ego[4], const double agent[4], double out[4]) override {
        const av::EgoState e{ego[0], ego[1], ego[2], ego[3]};
        const av::AgentState a{agent[0], agent[1], agent[2], agent[3]};
        const av::Observation o =
            av::observe(e, a, id_, s_.sigma_range, s_.sigma_bearing, s_.max_range, s_.seed, tick);
        out[0] = o.range; out[1] = o.bearing; out[2] = o.range_rate; out[3] = static_cast<double>(o.id);
    }
private:
    const Scenario& s_;
    int id_;
};

av::Observation as_obs(const double p[4]) {
    av::Observation o;
    o.range = p[0]; o.bearing = p[1]; o.range_rate = p[2]; o.id = static_cast<int>(p[3]);
    return o;
}

class NearestFusion final : public Kernel {
public:
    explicit NearestFusion(const Scenario& s) : s_(s) {}
    void combine(uint64_t, const double a[4], const double b[4], double out[4]) override {
        const av::Observation o = av::nearest(as_obs(a), as_obs(b), s_.max_range);
        out[0] = o.range; out[1] = o.bearing; out[2] = o.range_rate; out[3] = static_cast<double>(o.id);
    }
private:
    const Scenario& s_;
};

class TtcMonitor final : public Kernel {
public:
    void step(uint64_t, double p[4]) override {
        const av::Observation o = as_obs(p);
        p[0] = o.range;
        p[1] = o.range_rate;
        p[2] = av::time_to_collision(o);
        p[3] = static_cast<double>(o.id);
    }
};

bool is_av_kernel(const std::string& k) { return !k.empty(); }

void require_type(const BlockConfig& bc, BlockType t, const char* kernel) {
    if (bc.type != t) {
        throw std::runtime_error("block " + bc.name + ": kernel '" + kernel + "' used on the wrong block type");
    }
}

int agent_index_or_throw(const BlockConfig& bc, const Scenario& s, const std::string& agent) {
    const int id = s.agent_id(agent);
    if (id < 0) throw std::runtime_error("block " + bc.name + ": scenario has no agent '" + agent + "'");
    return id;
}

} // namespace

bool graph_needs_scenario(const GraphConfig& g) {
    for (const auto& b : g.blocks) if (is_av_kernel(b.kernel)) return true;
    return false;
}

std::unique_ptr<Kernel> make_kernel(const BlockConfig& bc, const Scenario* scenario) {
    const std::string& k = bc.kernel;
    if (k.empty()) {
        switch (bc.type) {
            case BlockType::Source: return std::make_unique<SineSource>();
            case BlockType::Transform: return std::make_unique<LinearTransform>(bc.id);
            case BlockType::Merge: return std::make_unique<SumMerge>();
            case BlockType::Sink: return std::make_unique<PassThrough>();
        }
    }
    if (!scenario) throw std::runtime_error("block " + bc.name + ": kernel '" + k + "' needs --scenario");
    const Scenario& s = *scenario;
    const auto colon = k.find(':');
    const std::string head = k.substr(0, colon);
    const std::string arg = colon == std::string::npos ? std::string() : k.substr(colon + 1);

    if (head == "av_source") { require_type(bc, BlockType::Source, "av_source"); return std::make_unique<AvSource>(s); }
    if (head == "ego_bicycle") { require_type(bc, BlockType::Transform, "ego_bicycle"); return std::make_unique<EgoBicycle>(s); }
    if (head == "agent") {
        require_type(bc, BlockType::Transform, "agent");
        return std::make_unique<ScriptedAgent>(s, agent_index_or_throw(bc, s, arg) - 1);
    }
    if (head == "sensor") {
        require_type(bc, BlockType::Merge, "sensor");
        return std::make_unique<RangeBearingSensor>(s, agent_index_or_throw(bc, s, arg));
    }
    if (head == "nearest") { require_type(bc, BlockType::Merge, "nearest"); return std::make_unique<NearestFusion>(s); }
    if (head == "ttc") { require_type(bc, BlockType::Transform, "ttc"); return std::make_unique<TtcMonitor>(); }
    throw std::runtime_error("block " + bc.name + ": unknown kernel '" + k + "'");
}

} // namespace shmmw
