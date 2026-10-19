#pragma once
#include "RLGymCPP/Framework.h"

namespace RLGC {
    inline Vec InvertVec(const Vec& vec, bool shouldInvert = true) {
        return shouldInvert ? Vec(-vec.x, -vec.y, vec.z) : vec;
    }
	PhysState InvertPhys(const PhysState& physState, bool shouldInvert = true);
	PhysState MirrorPhysX(const PhysState& physState, bool shouldMirror = true);
}
