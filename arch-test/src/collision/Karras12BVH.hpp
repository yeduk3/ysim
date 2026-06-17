#pragma once
#include "collision/IBVH.hpp"

// DEFAULT IBVH concrete: Karras (2012) linear BVH over triangle primitives.
//
// ponytail: this pass wires the interface + per-part backend config; the
// GPU build pipeline body (fillMortons -> radixSort -> buildTree ->
// bottomUpBoxes -> queryPoints, src/main.cpp 5101-6934 + bvh.metal) is the
// headline next port. See BVH_VERSIONS.md §4 (per-part CPU/GPU matrix) and
// PORT_MAP.md (collision rows). Methods are no-ops until ported so the
// architecture compiles + the Runner loop exercises the seam.
template <typename BE, typename PR>
struct Karras12BVH : IBVH<BE, PR> {
    BVHPartConfig parts;
    explicit Karras12BVH(const BVHPartConfig& p) : parts(p) {}

    const char* name() const override { return "Karras12 LBVH"; }
    void build(Scene<BE, PR>&, SimState<BE, PR>&) override {}
    void refit(SimState<BE, PR>&) override {}
    void enlargeTrajectory(SimState<BE, PR>&, PR) override {}
    void detectCollisions(PR, bool) override {}
};
