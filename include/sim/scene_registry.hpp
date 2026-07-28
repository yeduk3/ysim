#pragma once
// Extracted from src/main.cpp (ARCHITECTURE.md §2.1 section map).
// Fragment header: included in order by main.cpp after the sim fragments;
// relies on that preamble (Simulator / SymplecticSystem / Scene / PlaneDirection /
// BehaviorType / tinym, using Precision) and is not independently compilable.
//
// Named-scene registry for the `--scene <name>`, `--demo-uniform`, and no-arg
// launch paths. Backend / Precision / System are fixed at the main() call site,
// so this is a plain runtime registry (name -> imperative setup), NOT the
// compile-time template dispatch used by the arch-test experiment. Each setup
// wraps a scene block that used to live inline in main(), so every scene has
// exactly one definition.

#include <string_view>
#include <vector>

namespace scene_registry {

template <typename BE, typename PR>
using SimOf = Simulator<BE, PR, SymplecticSystem<BE, PR>>;
template <typename BE, typename PR>
using SysOf = SymplecticSystem<BE, PR>;

// "default": checkerboard floor only (no cloth, no obstacle). The no-arg
// launch path calls this, so the default scene has a single definition.
template <typename BE, typename PR>
void setupDefault(SimOf<BE, PR>& simulator, SysOf<BE, PR>& /*system*/) {
    simulator.addGround(PlaneDirection::XZPlane, tinym::vec3(0, 0, 0), 50);
    Scene<BE, PR>::requestsGeneralMeshes.back().checkerboard = true;
}

// "demo_uniform": tessellated static floor (3 wide, 24x24 -> face ~0.12) + a
// cloth (1 wide, 20x20 -> face ~0.05). Uniform face size keeps every hgrid
// cell sparse — the ML-spatial-hash sweet spot. Mirrors the old --demo-uniform
// block; broad-phase activation runs post-initialize via postInitDemoUniform.
template <typename BE, typename PR>
void setupDemoUniform(SimOf<BE, PR>& simulator, SysOf<BE, PR>& /*system*/) {
    const PR kstretch = 1e5, kshear = 1e5, kbend = 2e5;
    const PR mass = 0.1, thickness = 0.01;
    simulator.addPlane(PlaneDirection::XZPlane, tinym::vec3(0, 0, 0), 24, 3.0,
                       0.1, BehaviorType::Float);
    simulator.addCloth(20, 1, tinym::vec3(0, 0.6, 0), kstretch, kshear, kbend,
                       thickness, mass);
    simulator.mlBroadPhase.floorExcludeDiag = 1e9f;   // exclude NOTHING
}

template <typename BE, typename PR>
void postInitDemoUniform(SimOf<BE, PR>& simulator) {
    // Multi-level spatial hash as the active broad phase, held over cdP=8
    // substeps (stable + ~20-26fps on this scene; BVH tunnels at this cadence).
    // Toggle off live via the Profiler checkbox / CD-period slider to A/B.
    simulator.useMultiLevelSH   = true;
    simulator.useSpatialHashing = false;
    simulator.cdSubstepPeriod    = 8;
    simulator.refitSubstepPeriod = 8;
    std::cout << "[Main] --demo-uniform: ML broad phase active, cdP=8\n";
}

// "pbd_cloth": the same floor + cloth geometry as demo_uniform, but solved by
// the CPU PBD sibling instead of the symplectic integrator. Same idiom as
// postInitDemoUniform flipping the broad phase — the registry can only touch
// runtime fields of the already-constructed Simulator, and `usePbd` is one.
template <typename BE, typename PR>
void setupPbdCloth(SimOf<BE, PR>& simulator, SysOf<BE, PR>& /*system*/) {
    const PR kstretch = 1e5, kshear = 1e5, kbend = 2e5;
    const PR mass = 0.1, thickness = 0.01;
    simulator.addPlane(PlaneDirection::XZPlane, tinym::vec3(0, 0, 0), 24, 3.0,
                       0.1, BehaviorType::Float);
    simulator.addCloth(20, 1, tinym::vec3(0, 0.6, 0), kstretch, kshear, kbend,
                       thickness, mass);
    simulator.mlBroadPhase.floorExcludeDiag = 1e9f;   // exclude NOTHING
}

template <typename BE, typename PR>
void postInitPbdCloth(SimOf<BE, PR>& simulator) {
    simulator.usePbd = true;
    // PBD projects contacts from the CONTACT SET, so it needs one fresh every
    // substep — a held broad phase would let the cloth drift through.
    simulator.cdSubstepPeriod    = 1;
    simulator.refitSubstepPeriod = 1;
    std::cout << "[Main] --scene pbd_cloth: CPU PBD solver active\n";
}

template <typename BE, typename PR>
struct Entry {
    const char* name;
    const char* description;
    void (*setup)(SimOf<BE, PR>&, SysOf<BE, PR>&);
    void (*postInit)(SimOf<BE, PR>&);   // nullptr when the scene needs none
};

// The registry itself. Add a scene by writing its setup above and one row here.
template <typename BE, typename PR>
inline const std::vector<Entry<BE, PR>>& registry() {
    static const std::vector<Entry<BE, PR>> r = {
        { "default", "checkerboard floor only (no-arg default)",
          &setupDefault<BE, PR>, nullptr },
        { "demo_uniform", "cloth on a tessellated floor; ML broad phase, cdP=8",
          &setupDemoUniform<BE, PR>, &postInitDemoUniform<BE, PR> },
        { "pbd_cloth", "demo_uniform geometry solved by the CPU PBD system",
          &setupPbdCloth<BE, PR>, &postInitPbdCloth<BE, PR> },
    };
    return r;
}

template <typename BE, typename PR>
const Entry<BE, PR>* find(std::string_view name) {
    for (const auto& e : registry<BE, PR>())
        if (name == e.name) return &e;
    return nullptr;
}

} // namespace scene_registry
