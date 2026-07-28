#ifndef ENGINE_HPP
#define ENGINE_HPP

// The generic simulation engine: shared vocabulary, the concepts that constrain
// each interface, the mock components, and the scene-driven Simulator. Nothing
// here names a concrete scene -- the engine consumes whatever a scene declares.

#include <concepts>
#include <iostream>
#include <string>
#include <string_view>
#include <vector>

#include "CommonTypes.h"

// ---------------------------------------------------------------------------
// Scene vocabulary
// ---------------------------------------------------------------------------

enum class ObjectKind { Cloth, Rigid, Floor };

inline std::string_view kindLabel(ObjectKind k) {
    switch (k) {
        case ObjectKind::Cloth: return "Cloth";
        case ObjectKind::Rigid: return "Rigid";
        case ObjectKind::Floor: return "Floor";
    }
    return "?";
}

struct SceneObject {
    std::string name;
    ObjectKind  kind;
    UInt        vertexCount;
};

// ---------------------------------------------------------------------------
// Concepts -- one per interface the engine relies on
// ---------------------------------------------------------------------------

// A single broad- or narrow-phase collision stage.
template <class P, class SceneT>
concept CollisionPhase = requires(P& phase, SceneT& scene) {
    { phase.detect(scene) } -> std::same_as<void>;
};

// A collision-detection pipeline: discrete (dcd) + continuous (ccd) passes.
template <class CD, class SceneT>
concept CollisionDetection = requires(CD& cd, SceneT& scene) {
    { cd.dcd(scene) } -> std::same_as<void>;
    { cd.ccd(scene) } -> std::same_as<void>;
};

// A time-integration solver.
template <class S, class CDT, class SceneT>
concept Solver = requires(S& solver, CDT& cd, SceneT& scene, Real dt) {
    { solver.accumulate(cd, scene) } -> std::same_as<void>;
    { solver.integrate(scene, dt) }  -> std::same_as<void>;
};

// A runnable simulator.
template <class Sim>
concept Simulator = requires(Sim& sim, UInt frames) {
    { sim.step(UInt{}) } -> std::same_as<void>;
    { sim.run(frames) }  -> std::same_as<void>;
};

// A scene is the single source of configuration: it names the solver, the two
// collision phases (compile-time type aliases), the time-step, and the objects
// to create. Its dt/substeps/name/objects are *instance* accessors so the same
// interface serves both cpp scenes (fixed values) and JSON-loaded scenes
// (SceneRuntime, values filled at runtime).
template <class SceneT>
concept SceneConfig = requires(const SceneT& scene) {
    typename SceneT::SolverType;
    typename SceneT::BroadPhase;
    typename SceneT::NarrowPhase;
    { scene.name() }     -> std::convertible_to<std::string_view>;
    { scene.dt() }       -> std::convertible_to<Real>;
    { scene.substeps() } -> std::convertible_to<UInt>;
    { scene.build() }    -> std::same_as<std::vector<SceneObject>>;
};

// ---------------------------------------------------------------------------
// Mock solvers -- print what they do; no real physics
// ---------------------------------------------------------------------------

struct SolverExplicit {
    SolverExplicit() { std::cout << "    [solver] SolverExplicit created\n"; }

    template <class CDT, class SceneT>
    void accumulate(CDT&, const SceneT&) {
        std::cout << "      SolverExplicit.accumulate  (sum explicit forces)\n";
    }
    template <class SceneT>
    void integrate(SceneT&, Real dt) {
        std::cout << "      SolverExplicit.integrate   dt=" << dt << "  (x += v*dt)\n";
    }
};

struct SolverXPBD {
    SolverXPBD() { std::cout << "    [solver] SolverXPBD created\n"; }

    template <class CDT, class SceneT>
    void accumulate(CDT&, const SceneT&) {
        std::cout << "      SolverXPBD.accumulate      (gather constraints)\n";
    }
    template <class SceneT>
    void integrate(SceneT&, Real dt) {
        std::cout << "      SolverXPBD.integrate       dt=" << dt << "  (predict + relax)\n";
    }
};

// ---------------------------------------------------------------------------
// Mock collision phases -- broad and narrow
// ---------------------------------------------------------------------------

struct LBVH_Karras12 {
    LBVH_Karras12() { std::cout << "    [broad ] LBVH_Karras12 created\n"; }
    template <class SceneT>
    void detect(SceneT&) { std::cout << "        broad : LBVH_Karras12.detect   (Morton BVH sweep)\n"; }
};

struct SpatialHashing {
    SpatialHashing() { std::cout << "    [broad ] SpatialHashing created\n"; }
    template <class SceneT>
    void detect(SceneT&) { std::cout << "        broad : SpatialHashing.detect  (uniform grid buckets)\n"; }
};

struct ExhaustiveSearch {
    ExhaustiveSearch() { std::cout << "    [narrow] ExhaustiveSearch created\n"; }
    template <class SceneT>
    void detect(SceneT&) { std::cout << "        narrow: ExhaustiveSearch.detect (all point-triangle pairs)\n"; }
};

struct ProximityQuery {
    ProximityQuery() { std::cout << "    [narrow] ProximityQuery created\n"; }
    template <class SceneT>
    void detect(SceneT&) { std::cout << "        narrow: ProximityQuery.detect  (distance within margin)\n"; }
};

// A two-phase collision-detection pipeline built from a broad and narrow phase.
template <class BroadPhase, class NarrowPhase>
struct CollisionDetector {
    BroadPhase  broad;
    NarrowPhase narrow;

    template <class SceneT>
    void dcd(SceneT& scene) {
        std::cout << "      CollisionDetector.dcd (discrete)\n";
        broad.detect(scene);
        narrow.detect(scene);
    }
    template <class SceneT>
    void ccd(SceneT& scene) {
        std::cout << "      CollisionDetector.ccd (continuous)\n";
        broad.detect(scene);
        narrow.detect(scene);
    }
};

// ---------------------------------------------------------------------------
// Generic simulator -- fully typed by the scene it is instantiated with
// ---------------------------------------------------------------------------

template <SceneConfig SceneT>
struct SimulatorBasic {
    using SolverType  = typename SceneT::SolverType;
    using BroadPhase  = typename SceneT::BroadPhase;
    using NarrowPhase = typename SceneT::NarrowPhase;
    using CDType      = CollisionDetector<BroadPhase, NarrowPhase>;

    static_assert(CollisionPhase<BroadPhase, SceneT>,   "BroadPhase must satisfy CollisionPhase");
    static_assert(CollisionPhase<NarrowPhase, SceneT>,  "NarrowPhase must satisfy CollisionPhase");
    static_assert(CollisionDetection<CDType, SceneT>,   "CollisionDetector must satisfy CollisionDetection");
    static_assert(Solver<SolverType, CDType, SceneT>,   "SolverType must satisfy Solver");

    SceneT                   scene;
    SolverType               solver;
    CDType                   cd;
    std::vector<SceneObject> objects;

    SimulatorBasic() = default;
    explicit SimulatorBasic(SceneT s) : scene(std::move(s)) {}

    void setup() {
        std::cout << "== setup scene '" << scene.name() << "' ==\n";
        objects = scene.build();
        std::cout << "    objects (" << objects.size() << "):\n";
        for (const auto& o : objects) {
            std::cout << "      - " << o.name << "  [" << kindLabel(o.kind)
                      << ", " << o.vertexCount << " verts]\n";
        }
        std::cout << "    dt=" << scene.dt() << "  substeps=" << scene.substeps() << "\n";
        // A solver that owns real state (e.g. SolverPBD) gets the object list
        // here. The printing mocks define no setup(), so nothing is called.
        if constexpr (requires { solver.setup(objects); }) solver.setup(objects);
    }

    void step(UInt frameIdx) {
        const Real subDt = scene.dt() / static_cast<Real>(scene.substeps());
        std::cout << "-- frame " << frameIdx << " --\n";
        for (UInt s = 0; s < scene.substeps(); ++s) {
            std::cout << "    substep " << s << "\n";
            cd.dcd(scene);
            solver.accumulate(cd, scene);
            solver.integrate(scene, subDt);
            cd.ccd(scene);
        }
    }

    void run(UInt frames) {
        std::cout << "== run " << frames << " frame(s) ==\n";
        for (UInt f = 0; f < frames; ++f) step(f);
        std::cout << "== done ==\n";
    }
};

#endif // ENGINE_HPP
