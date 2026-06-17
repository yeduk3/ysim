#pragma once
#include "backend/MemoryPool.hpp"
#include "core/Scene.hpp"
#include "core/SimState.hpp"
#include "collision/ICDPipeline.hpp"
#include "collision/DefaultCDPipeline.hpp"
#include "system/ISystem.hpp"

#include <memory>
#include <utility>

// The hub. Generic — never subclassed (blueprint invariant c). Owns the
// static Scene, live SimState, and the swappable CD pipeline + integrator
// behind virtual interfaces (factory-injected). step() = dcd -> accumulate
// -> integration -> recoveryPenetration per substep, one GPU commit/frame.
template <typename BE, typename PR>
struct Simulator {
    Scene<BE, PR> scene;
    SimState<BE, PR> state;
    std::unique_ptr<ICDPipeline<BE, PR>> cdPipeline;
    std::unique_ptr<ISystem<BE, PR>> system;

    PR h = PR(1.0 / 60.0);
    int subSteps = 60;
    PR margin = PR(0.015);
    PR radius = PR(0.012);
    int targetFrames = 300;
    int frame = 0;

    // System is caller-owned and outlives nothing here; injected at ctor
    // (DECISIONS C17). CD pipeline defaults to DefaultCDPipeline(Karras12).
    explicit Simulator(std::unique_ptr<ISystem<BE, PR>> sys) : system(std::move(sys)) {}

    Index add(ObjectDesc d) { return scene.add(d); }

    void initialize() {
        GlobalAutoAllocator<BE>::reset();          // D-041 replay-stable rewind
        scene.realize(state);                       // seed SimState from static Scene
        if (!cdPipeline) cdPipeline = makeDefaultCDPipeline<BE, PR>();
        cdPipeline->build(scene, state);
    }

    // gravity*mass + wind -> externalForces (fixed verts m==0 -> 0). Filled
    // per-frame, reused across substeps (DECISIONS C13 / D-A3).
    void applyEnvironmentForces() {
        const auto& g = scene.environment.gravity;
        const auto& w = scene.environment.wind;
        for (Index i = 0; i < state.numPoints; ++i) {
            PR m = state.m[i];
            state.externalForces[i * 3 + 0] = m * PR(g.x) + PR(w.x);
            state.externalForces[i * 3 + 1] = m * PR(g.y) + PR(w.y);
            state.externalForces[i * 3 + 2] = m * PR(g.z) + PR(w.z);
        }
    }

    void step() {
        PR dt = h / PR(subSteps);
        for (int sub = 0; sub < subSteps; ++sub) {
            cdPipeline->dcd(scene, state, margin);
            for (Index i = 0; i < state.numPoints * 3; ++i) state.xPrev[i] = state.x[i];
            system->accumulate(scene, state);
            system->integration(state, dt);
            system->recoveryPenetration(state);
        }
        MetalGlobalContext::commitAndWait();  // one GPU sync/frame (no-op for CPU paths)
    }

    bool update() {
        if (frame >= targetFrames) return false;
        applyEnvironmentForces();
        step();
        ++frame;
        return true;
    }
};
