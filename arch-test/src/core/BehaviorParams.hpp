#pragma once
#include "backend/Backend.hpp"

#include <cmath>
#include <variant>

// Byte layout MUST match physics.metal ClothParams / ClothGridParams
// (sent verbatim via setBytes). Field NAMES differ from the metal side
// (stretch vs kstretch) but the layout is identical — DECISIONS C22.
template <typename PR>
struct ClothBehaviorParams { PR stretch, shear, bend; PR thickness; };

template <typename PR>
struct FastGridClothBehaviorParams {
    uint32_t particleNum1D;
    PR stretchRestX, stretchRestY;
    PR shearRestA, shearRestB;
    PR bendRestX, bendRestY;
    PR kstretch, kshear, kbend;
    PR thickness;
};

template <typename PR>
struct FloatBehaviorParams {};

template <typename PR>
using BehaviorParams = std::variant<
    ClothBehaviorParams<PR>, FastGridClothBehaviorParams<PR>, FloatBehaviorParams<PR>>;

// Derive FastGridCloth's 6 directional rest lengths from live grid geometry.
template <typename PR>
inline void recomputeFastGridRest(const PR* x, Index numPoints,
                                  FastGridClothBehaviorParams<PR>& p) {
    uint32_t pn = p.particleNum1D;
    if (pn < 2 || x == nullptr) return;
    auto dist = [&](Index a, Index b) {
        PR dx = x[a*3+0]-x[b*3+0], dy = x[a*3+1]-x[b*3+1], dz = x[a*3+2]-x[b*3+2];
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    };
    auto set = [&](PR& dst, Index a, Index b) {
        if (a < numPoints && b < numPoints) { PR d = dist(a, b); if (d > PR(1e-9)) dst = d; }
    };
    const Index P = (Index)pn;
    set(p.stretchRestX, 0, 1);
    set(p.stretchRestY, 0, P);
    set(p.shearRestA,   0, P + 1);
    set(p.shearRestB,   1, P);
    set(p.bendRestX,    0, 2);
    set(p.bendRestY,    0, 2 * P);
}
