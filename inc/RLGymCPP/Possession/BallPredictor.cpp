#include "BallPredictor.h"
#include <algorithm>
#include <cmath>
#include <mutex>
#include <stdexcept>

namespace {
// Must match #[repr(C)] RCBallState in RocketSimV3/rocketsim/src/c_api.rs.
struct RCBallState {
    float pos[3];
    float forward[3], right[3], up[3];
    float vel[3], angVel[3];
};
static_assert(sizeof(RCBallState) == 18 * sizeof(float));
bool ready = false;
extern "C" {
    bool rocketsim_init(bool silent);
    void* arena_create(int32_t gameMode);
    void arena_destroy(void* arena);
    void arena_set_ball_state(void* arena, const RCBallState* state);
    void arena_predict_ball(void* arena, uint32_t ticks, RCBallState* states);
}
}

namespace RLGC {
const BallSlice& BallTrajectory::GetSliceAtTime(float time) const {
    if (slices.empty()) throw std::runtime_error("Empty RocketSim v3 prediction");
    const int index = static_cast<int>(time / CommonValues::TICK_TIME);
    return slices[std::clamp(index, 0, static_cast<int>(slices.size()) - 1)];
}

bool BallPredictor::InitV3Backend(const std::filesystem::path& meshesParent) {
    static std::once_flag once;
    std::call_once(once, [&] {
        const auto previous = std::filesystem::current_path();
        std::filesystem::current_path(meshesParent);
        ready = rocketsim_init(true);
        std::filesystem::current_path(previous);
    });
    return ready;
}

BallPredictor::~BallPredictor() {
    if (arena) arena_destroy(arena);
}

void BallPredictor::Predict(BallTrajectory& trajectory, const Vec& pos,
                          const Vec& vel, const Vec& angVel, float horizon) {
    if (!ready) throw std::runtime_error("RocketSim v3 was not initialized");
    if (!std::isfinite(horizon) || horizon <= 0.f || horizon > 10.f)
        throw std::runtime_error("Invalid ball prediction horizon");
    if (!arena) arena = arena_create(0);
    if (!arena) throw std::runtime_error("RocketSim v3 arena creation failed");
    const int ticks = std::max(1, static_cast<int>(std::ceil(horizon / CommonValues::TICK_TIME)));
    const RCBallState initial{{pos.x, pos.y, pos.z}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1},
                              {vel.x, vel.y, vel.z}, {angVel.x, angVel.y, angVel.z}};
    arena_set_ball_state(arena, &initial);
    std::vector<RCBallState> states(ticks);
    arena_predict_ball(arena, ticks, states.data());
    trajectory.slices.clear();
    trajectory.slices.reserve(ticks + 1);
    trajectory.slices.push_back({pos, vel, angVel});
    for (const auto& state : states) {
        trajectory.slices.push_back({Vec(state.pos[0], state.pos[1], state.pos[2]),
                                     Vec(state.vel[0], state.vel[1], state.vel[2]),
                                     Vec(state.angVel[0], state.angVel[1], state.angVel[2])});
    }
    trajectory.validUntil = ticks * CommonValues::TICK_TIME;
}
}
