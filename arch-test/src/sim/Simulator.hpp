#pragma once
#include "backend/MemoryPool.hpp"
#include "core/Scene.hpp"
#include "core/SimState.hpp"
#include "core/LUT.hpp"
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

    // This sim's OWN memory (O2). VectorBase routes here while this sim is
    // active; kept alive for the sim's lifetime so its buffers stay valid even
    // after another sim becomes active. Lazy-growing (no up-front reservation).
    DynamicMemoryAllocator<BE> pool;

    LUT lut;   // runtime data bus: render-out handles + params (DECISIONS O1)

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

    // Expose render-out handles + params through the owned LUT. Called at the
    // END of initialize() (after realize() assigns SimState buffers and after
    // the pool reset), so a re-initialize re-points entries in place for
    // key-caching consumers (invariant a). Counters bound by ADDRESS so the
    // Runner reads live values, not snapshots.
    void publish() {
        lut.template bind<Precision>("pos", state.x.ptr);   // 3*numPoints
        lut.template bind<Precision>("vel", state.v.ptr);
        lut.template bind<Precision>("nrm", state.n.ptr);
        lut.template bind<Index>("numPoints", &state.numPoints);
        lut.template bind<int>("frame", &frame);
        lut.template bind<tinym::vec3>("gravity", &scene.environment.gravity);
        lut.template bind<tinym::vec3>("wind", &scene.environment.wind);
        lut.template bind<int>("subSteps", &subSteps, LUT::UpdatePolicy::RebuildPaused);
        lut.template bind<int>("targetFrames", &targetFrames);
    }

    void initialize() {
        GlobalAutoAllocator<BE>::setActive(&pool);  // route allocs to THIS sim's pool
        GlobalAutoAllocator<BE>::reset();           // D-041 replay-stable rewind of THIS pool
        scene.realize(state);                       // seed SimState from static Scene
        if (!cdPipeline) cdPipeline = makeDefaultCDPipeline<BE, PR>();
        cdPipeline->build(scene, state);
        publish();
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
        GlobalAutoAllocator<BE>::setActive(&pool);  // BVH dcd may allocate scratch (real port)
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
