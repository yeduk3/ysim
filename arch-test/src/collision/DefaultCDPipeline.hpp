#pragma once
#include "collision/ICDPipeline.hpp"
#include "collision/IBVH.hpp"
#include "collision/BVHFactory.hpp"

#include <memory>

// Composable concrete: an IBVH broad phase + (future) narrow phase.
// dcd = refit -> enlargeTrajectory -> detectCollisions -> narrow. ccd is a
// no-op: CCD is fused into the narrow swept test in the original engine
// (DECISIONS A9 / D-A1), not a separate pass.
//
// ponytail: narrow phase (BruteForce<METAL> narrow_pt_tri, src/main.cpp
// 7393-7681) is the next port; until then dcd only drives the broad seam.
template <typename BE, typename PR>
struct DefaultCDPipeline : ICDPipeline<BE, PR> {
    std::unique_ptr<IBVH<BE, PR>> broad;
    explicit DefaultCDPipeline(std::unique_ptr<IBVH<BE, PR>> b) : broad(std::move(b)) {}

    const char* name() const override { return "DefaultCDPipeline(broad+narrow)"; }
    void build(Scene<BE, PR>& scene, SimState<BE, PR>& state) override {
        broad->build(scene, state);
    }
    void dcd(Scene<BE, PR>&, SimState<BE, PR>& state, PR margin) override {
        broad->refit(state);
        broad->enlargeTrajectory(state, margin);
        broad->detectCollisions(margin, /*selfCollision=*/false);
        // narrow phase: TODO (see PORT_MAP.md narrow rows)
    }
    void ccd(Scene<BE, PR>&, SimState<BE, PR>&, PR) override {}  // fused into dcd
};

template <typename BE, typename PR>
inline std::unique_ptr<ICDPipeline<BE, PR>> makeDefaultCDPipeline(const BVHConfig& cfg = {}) {
    return std::make_unique<DefaultCDPipeline<BE, PR>>(BVHFactory<BE, PR>::make(cfg));
}
