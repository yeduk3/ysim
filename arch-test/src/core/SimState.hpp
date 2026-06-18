#pragma once
#include "backend/Backend.hpp"
#include "backend/VectorBase.hpp"
#include "core/Types.hpp"   // NarrowCollision (contact slot element type)

#include <vector>

// LIVE simulation state — the blueprint's headline split out of Scene
// (DECISIONS C9). Positions/velocities/forces are here, NOT in Scene.
// Allocated contiguously across all meshes; per-mesh sub-views are derived
// from Scene::statesOffsets. Mass `m` is immutable after realization.
template <typename BE, typename PR>
struct SimState {
    VectorBase<BE, PR> x, xPrev, v, f, n, externalForces; // 3*numPoints each
    VectorBase<BE, PR> m;                                  // numPoints (immutable)
    Index numPoints = 0;

    // --- GPU 2-pass integrator prerequisites (ported alongside the spring
    // kernels). Per the design these live in SimState so the ISystem virtual
    // signatures stay unchanged (memory rule: add new, don't widen).
    // fixedParticles: pin MASK (1 = movable, 0 = pinned). The integrate kernel
    //   multiplies the v/x update by this. No pins this pass -> all 1.
    VectorBase<BE, PR> fixedParticles;                    // numPoints, init 1
    // Empty contact set: narrow phase fills these later. Offsets all-zero ->
    //   begin==end per vertex -> recovery loop is a no-op (correct contract).
    VectorBase<BE, NarrowCollision> vertColFacets;        // 1 dummy (valid pool)
    VectorBase<BE, Index> vertColFacetsOffsets;           // numPoints+1, zero
    // statesOffsets mirror on the GPU (integrate reads obase=statesOffsets[oid]).
    VectorBase<BE, Index> statesOffsetsGPU;               // numObjects+1

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
        fixedParticles = VectorBase<BE, PR>(np, PR(1));          // all movable
        vertColFacets = VectorBase<BE, NarrowCollision>(1);      // valid-but-unread dummy
        vertColFacetsOffsets = VectorBase<BE, Index>(np + 1, Index(0));
    }

    // Sized by object count, not vertex count — built after Scene finalizes the
    // CPU statesOffsets vector. Copies it into a GPU-visible buffer.
    void realizeAux(const std::vector<Index>& cpuStatesOffsets) {
        statesOffsetsGPU = VectorBase<BE, Index>(cpuStatesOffsets.size());
        for (size_t i = 0; i < cpuStatesOffsets.size(); ++i)
            statesOffsetsGPU[i] = cpuStatesOffsets[i];
    }
};
