#pragma once

#include "RLGymCPP/CommonValues.h"
#include <filesystem>
#include <vector>

namespace RLGC {
struct BallSlice {
    Vec pos, vel, angVel;
};

struct BallTrajectory {
    std::vector<BallSlice> slices;
    float validUntil = 0.f;
    const BallSlice& GetSliceAtTime(float time) const;
};

// RocketSim v3 C API adapter, derived from FuryGGLBot's V3Ffi120 path.
class BallPredictor {
    void* arena = nullptr;
public:
    BallPredictor() = default;
    ~BallPredictor();
    BallPredictor(const BallPredictor&) = delete;
    BallPredictor& operator=(const BallPredictor&) = delete;
    static bool InitV3Backend(const std::filesystem::path& meshesParent);
    void Predict(BallTrajectory& trajectory, const Vec& pos, const Vec& vel,
                 const Vec& angVel, float horizon);
};
}
