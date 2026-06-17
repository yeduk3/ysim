#pragma once
#include "backend/Backend.hpp"
#include "backend/VectorBase.hpp"

// LIVE simulation state — the blueprint's headline split out of Scene
// (DECISIONS C9). Positions/velocities/forces are here, NOT in Scene.
// Allocated contiguously across all meshes; per-mesh sub-views are derived
// from Scene::statesOffsets. Mass `m` is immutable after realization.
template <typename BE, typename PR>
struct SimState {
    VectorBase<BE, PR> x, xPrev, v, f, n, externalForces; // 3*numPoints each
    VectorBase<BE, PR> m;                                  // numPoints (immutable)
    Index numPoints = 0;

    void allocate(Index np) {
        numPoints = np;
        if (np == 0) return;
        x  = VectorBase<BE, PR>(np * 3, PR(0));
        xPrev = VectorBase<BE, PR>(np * 3, PR(0));
        v  = VectorBase<BE, PR>(np * 3, PR(0));
        f  = VectorBase<BE, PR>(np * 3, PR(0));
        n  = VectorBase<BE, PR>(np * 3, PR(0));
        externalForces = VectorBase<BE, PR>(np * 3, PR(0));
        m  = VectorBase<BE, PR>(np, PR(1));
    }
};
