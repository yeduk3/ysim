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
    // PBD consumes self rows via the two-way contact path.
    simulator.enableSelfCollisions = true;
    std::cout << "[Main] --scene pbd_cloth: CPU PBD solver active\n";
}

// "pbd_cloth_ball": the pbd_cloth sheet pinned at its four corners, with a
// 0.5 m Rigid ball dropped from just above the center. Repro scene for the
// stale-analytic-collider bug (a falling Rigid's Sphere collider used to
// stay frozen at its spawn transform, so the ball fell through the cloth
// with zero contacts). Corners are pinned through the request's
// fixedVertices — the pack-surviving source of truth Scene::pack re-applies
// after the initializer regenerates the grid.
template <typename BE, typename PR>
void setupPbdClothBall(SimOf<BE, PR>& simulator, SysOf<BE, PR>& /*system*/) {
    const PR kstretch = 1e5, kshear = 1e5, kbend = 2e5;
    const PR mass = 0.1, thickness = 0.01;
    const Index n = 20;
    const PR size = 1, y = 0.6;
    simulator.addPlane(PlaneDirection::XZPlane, tinym::vec3(0, 0, 0), 24, 3.0,
                       0.1, BehaviorType::Float);                        // id 0
    simulator.addCloth(n, size, tinym::vec3(0, (float)y, 0), kstretch, kshear,
                       kbend, thickness, mass);                          // id 1
    {
        auto& req = Scene<BE, PR>::requestsGeneralMeshes.back();
        const float h = (float)(size / 2);
        const uint32_t vids[4] = { 0, (uint32_t)n - 1,
                                   (uint32_t)(n * (n - 1)),
                                   (uint32_t)(n * n - 1) };
        const tinym::vec3 pos[4] = { { -h, (float)y, -h }, { h, (float)y, -h },
                                     { -h, (float)y,  h }, { h, (float)y,  h } };
        for (int i = 0; i < 4; ++i)
            req.fixedVertices.push_back(FixedVertex{ vids[i], pos[i] });
    }
    // Rigid → Bullet drives the fall; the Sphere analytic collider must
    // follow the body (refreshAnalyticShapes reads the live centroid).
    simulator.addSphere(tinym::vec3(0, 1.0, 0), 16, 0.5, PR(0.1),
                        BehaviorType::Rigid);                            // id 2
    simulator.mlBroadPhase.floorExcludeDiag = 1e9f;   // exclude NOTHING
}

template <typename BE, typename PR>
void postInitPbdClothBall(SimOf<BE, PR>& simulator) {
    simulator.usePbd = true;
    simulator.cdSubstepPeriod    = 1;   // same rationale as postInitPbdCloth
    simulator.refitSubstepPeriod = 1;
    // PBD consumes self rows via the two-way contact path.
    simulator.enableSelfCollisions = true;
    std::cout << "[Main] --scene pbd_cloth_ball: CPU PBD solver, corner-pinned "
                 "cloth, Rigid ball drop\n";
}

// "pbd_cloth_ball_self": pbd_cloth_ball with the ball retagged CLOTH and
// ONLY its per-object 자기 충돌 (selfCollide) toggle set — the scene-wide
// enableSelfCollisions stays FALSE on purpose. This is the user's inspector
// flow expressed as code: retag a primitive to cloth, tick that one mesh's
// self-collision box, and self rows must appear. It exercises three things
// that used to be broken together:
//   * addGeneralMesh seeds colliderKind = Mesh for a cloth-tagged sphere
//     (an analytic collider on a deforming mesh is triple-dropped);
//   * RequestGeneralMesh::selfCollide survives Scene::pack onto the mesh;
//   * CpuSpatialHash::detectSelfCollisions honours the PER-OBJECT flag with
//     the global switch off (Simulator::useCpuShSelf, default on).
template <typename BE, typename PR>
void setupPbdClothBallSelf(SimOf<BE, PR>& simulator, SysOf<BE, PR>& /*system*/) {
    const PR kstretch = 1e5, kshear = 1e5, kbend = 2e5;
    const PR mass = 0.1, thickness = 0.01;
    const Index n = 20;
    const PR size = 1, y = 0.6;
    simulator.addPlane(PlaneDirection::XZPlane, tinym::vec3(0, 0, 0), 24, 3.0,
                       0.1, BehaviorType::Float);                        // id 0
    simulator.addCloth(n, size, tinym::vec3(0, (float)y, 0), kstretch, kshear,
                       kbend, thickness, mass);                          // id 1
    {
        auto& req = Scene<BE, PR>::requestsGeneralMeshes.back();
        const float h = (float)(size / 2);
        const uint32_t vids[4] = { 0, (uint32_t)n - 1,
                                   (uint32_t)(n * (n - 1)),
                                   (uint32_t)(n * n - 1) };
        const tinym::vec3 pos[4] = { { -h, (float)y, -h }, { h, (float)y, -h },
                                     { -h, (float)y,  h }, { h, (float)y,  h } };
        for (int i = 0; i < 4; ++i)
            req.fixedVertices.push_back(FixedVertex{ vids[i], pos[i] });
    }
    // The ball as CLOTH, not Rigid. addSphere hands addGeneralMesh a
    // FloatBehaviorParams regardless of the behavior tag, so the request's
    // params are replaced with real cloth params — otherwise thicknessOf /
    // springConstantsOf would read 0 and the sheet would have no contact
    // thickness and no stiffness.
    simulator.addSphere(tinym::vec3(0, 1.0, 0), 16, 0.5, PR(0.1),
                        BehaviorType::TriangularCloth);                  // id 2
    {
        auto& req = Scene<BE, PR>::requestsGeneralMeshes.back();
        req.behaviorParams = ClothBehaviorParams<PR>{ kstretch, kshear,
                                                      kbend, thickness };
        req.selfCollide = true;   // ← the ONLY self-collision switch here
    }
    simulator.mlBroadPhase.floorExcludeDiag = 1e9f;   // exclude NOTHING
}

template <typename BE, typename PR>
void postInitPbdClothBallSelf(SimOf<BE, PR>& simulator) {
    simulator.usePbd = true;
    simulator.cdSubstepPeriod    = 1;
    simulator.refitSubstepPeriod = 1;
    // DELIBERATELY off: the point of this scene is that the per-object
    // toggle alone must produce self rows.
    simulator.enableSelfCollisions = false;
    std::cout << "[Main] --scene pbd_cloth_ball_self: CPU PBD solver, "
                 "cloth-tagged ball, PER-OBJECT self-collision only "
                 "(global enableSelfCollisions off)\n";
}

// "pbd_cloth_xyfold": the user's self-collision toggle test bed. A sheet
// standing VERTICALLY (XY plane — rotated 90° about x vs the usual XZ sheet)
// released just above a Float floor, at the user's interactive settings
// (subSteps 3, PBD iterations 8). It buckles as it lands ("구부러짐").
//
// Purpose: the per-object 자기 충돌 checkbox on this mesh shows no visible
// difference, and this scene makes that reproducible. The request's
// selfCollide stays FALSE — the toggle IS the experimental variable, flipped
// in the inspector (or in the CPSH-8 A/B harness). Note the distinction
// under test: BUCKLING is not LAYER CONTACT. A sheet can fold into smooth
// curves whose opposing faces never come within thickness+radius, in which
// case zero self rows is the physically correct answer and the toggle
// legitimately does nothing.
template <typename BE, typename PR>
void setupPbdClothXYFold(SimOf<BE, PR>& simulator, SysOf<BE, PR>& system) {
    const PR kstretch = 1e5, kshear = 1e5, kbend = 2e5;
    const PR thickness = 0.01;
    simulator.addPlane(PlaneDirection::XZPlane, tinym::vec3(0, 0, 0), 24, 3.0,
                       0.1, BehaviorType::Float);                        // id 0
    // The vertical sheet. addPlane hands addGeneralMesh a Float behavior (and
    // FloatBehaviorParams), so the request is retagged to cloth here — the
    // same idiom setupPbdClothBallSelf uses for its ball. Without the params
    // swap thicknessOf / springConstantsOf would read 0.
    simulator.addPlane(PlaneDirection::XYPlane, tinym::vec3(0, 0.55, 0), 20,
                       1.0, 0.1, BehaviorType::Float);                   // id 1
    {
        auto& req = Scene<BE, PR>::requestsGeneralMeshes.back();
        req.behaviorType   = BehaviorType::TriangularCloth;
        req.behaviorParams = ClothBehaviorParams<PR>{ kstretch, kshear,
                                                      kbend, thickness };
        // selfCollide deliberately LEFT FALSE — see the header comment.
    }
    // The user's interactive settings. setup() owns these because subSteps is
    // System state, not Simulator state (postInit only sees the Simulator).
    system.subSteps = 3;
    system.subh     = system.h / PR(3);
    simulator.mlBroadPhase.floorExcludeDiag = 1e9f;   // exclude NOTHING
}

template <typename BE, typename PR>
void postInitPbdClothXYFold(SimOf<BE, PR>& simulator) {
    simulator.usePbd = true;
    simulator.cdSubstepPeriod    = 1;
    simulator.refitSubstepPeriod = 1;
    // Global self OFF: the per-object checkbox is the variable under test.
    simulator.enableSelfCollisions = false;
    simulator.pbd.iterations = 8;
    std::cout << "[Main] --scene pbd_cloth_xyfold: CPU PBD solver, VERTICAL "
                 "(XY) sheet, subSteps=3, PBD iterations=8, global self OFF "
                 "— flip the mesh's 자기 충돌 pill to A/B it\n";
}

// "pbd_cloth_stack": two PBD cloths, one corner-pinned sheet (id 1) with a
// smaller free sheet (id 2) dropped onto it, over a Float floor. The
// cross-cloth contacts are the two-way path's reason to exist: both meshes
// are live cloths, so every vertex-triangle row corrects BOTH sides
// (Müller 2007 eq 12/13) instead of pushing only the query vertex.
template <typename BE, typename PR>
void setupPbdClothStack(SimOf<BE, PR>& simulator, SysOf<BE, PR>& /*system*/) {
    const PR kstretch = 1e5, kshear = 1e5, kbend = 2e5;
    const PR mass = 0.1, thickness = 0.01;
    const Index n = 20;
    const PR size = 1, y = 0.6;
    simulator.addPlane(PlaneDirection::XZPlane, tinym::vec3(0, 0, 0), 24, 3.0,
                       0.1, BehaviorType::Float);                        // id 0
    simulator.addCloth(n, size, tinym::vec3(0, (float)y, 0), kstretch, kshear,
                       kbend, thickness, mass);                          // id 1
    {
        auto& req = Scene<BE, PR>::requestsGeneralMeshes.back();
        const float h = (float)(size / 2);
        const uint32_t vids[4] = { 0, (uint32_t)n - 1,
                                   (uint32_t)(n * (n - 1)),
                                   (uint32_t)(n * n - 1) };
        const tinym::vec3 pos[4] = { { -h, (float)y, -h }, { h, (float)y, -h },
                                     { -h, (float)y,  h }, { h, (float)y,  h } };
        for (int i = 0; i < 4; ++i)
            req.fixedVertices.push_back(FixedVertex{ vids[i], pos[i] });
    }
    // Free sheet, dropped onto the pinned one.
    simulator.addCloth(12, 0.5, tinym::vec3(0, 0.75, 0), kstretch, kshear,
                       kbend, thickness, mass);                          // id 2
    simulator.mlBroadPhase.floorExcludeDiag = 1e9f;   // exclude NOTHING
}

template <typename BE, typename PR>
void postInitPbdClothStack(SimOf<BE, PR>& simulator) {
    simulator.usePbd = true;
    simulator.cdSubstepPeriod    = 1;   // same rationale as postInitPbdCloth
    simulator.refitSubstepPeriod = 1;
    // PBD consumes self rows via the two-way contact path.
    simulator.enableSelfCollisions = true;
    std::cout << "[Main] --scene pbd_cloth_stack: CPU PBD solver, two stacked "
                 "cloths, two-way contacts\n";
}

// "pd_cloth": the pbd_cloth scene solved by the CPU Projective Dynamics
// sibling instead. Identical geometry on purpose — the two CPU solvers are
// only comparable if the scene is byte-identical, so `--scene pbd_cloth` vs
// `--scene pd_cloth` is a one-flag A/B.
template <typename BE, typename PR>
void setupPdCloth(SimOf<BE, PR>& simulator, SysOf<BE, PR>& /*system*/) {
    const PR kstretch = 1e5, kshear = 1e5, kbend = 2e5;
    const PR mass = 0.1, thickness = 0.01;
    simulator.addPlane(PlaneDirection::XZPlane, tinym::vec3(0, 0, 0), 24, 3.0,
                       0.1, BehaviorType::Float);
    simulator.addCloth(20, 1, tinym::vec3(0, 0.6, 0), kstretch, kshear, kbend,
                       thickness, mass);
    simulator.mlBroadPhase.floorExcludeDiag = 1e9f;   // exclude NOTHING
}

template <typename BE, typename PR>
void postInitPdCloth(SimOf<BE, PR>& simulator) {
    simulator.usePd = true;
    // PD projects contacts from the CONTACT SET once per substep (post-solve),
    // so it needs a fresh one every substep for the same reason PBD does — a
    // held broad phase would let the cloth drift through.
    simulator.cdSubstepPeriod    = 1;
    simulator.refitSubstepPeriod = 1;
    std::cout << "[Main] --scene pd_cloth: CPU Projective Dynamics solver active\n";
}

// ---- P2 analytic-collider scenes (collider_pipeline_rework.md §5-P2) -----
// One cloth dropped onto one analytic primitive, plus a "zoo" that drapes a
// single sheet over three primitives standing on a Plane floor. The
// primitives pick up their analytic colliderKind from their initializer
// automatically (Sphere/Cube/Cylinder → Sphere/Box/Cylinder); only the grid
// floor has to be retagged Plane, which postInit does — the same gesture the
// inspector dropdown performs (request + live mesh, so it survives a repack).

// Shared cloth knobs across the analytic scenes.
template <typename PR>
struct AnalyticSceneCloth {
    static constexpr PR kstretch = PR(1e5);
    static constexpr PR kshear   = PR(1e5);
    static constexpr PR kbend    = PR(2e5);
    static constexpr PR mass     = PR(0.1);
    static constexpr PR thick    = PR(0.01);
};

// Retag one mesh's collider, writing BOTH the request (survives Scene::pack)
// and the live mesh — the inspector's immediate-apply pattern.
template <typename BE, typename PR>
void setColliderKind(SimOf<BE, PR>& simulator, int meshId, ColliderKind kind) {
    if (auto* r = simulator.findRequest(meshId)) r->colliderKind = kind;
    if (auto* m = Scene<BE, PR>::findById(meshId)) m->colliderKind = kind;
}

// "analytic_sphere" / "_box" / "_cylinder": a 1 m sheet released 0.6 m above
// a size-1.2 primitive centred at the origin. Moderate drop, no fixed
// particles — the cloth drapes over the cap.
template <typename BE, typename PR>
void setupAnalyticSphere(SimOf<BE, PR>& simulator, SysOf<BE, PR>&) {
    using C = AnalyticSceneCloth<PR>;
    simulator.addSphere(tinym::vec3(0, 0, 0), 16, 1.2);              // id 0
    simulator.addCloth(24, 1.0, tinym::vec3(0, 1.2, 0),
                       C::kstretch, C::kshear, C::kbend, C::thick, C::mass);
}

template <typename BE, typename PR>
void setupAnalyticBox(SimOf<BE, PR>& simulator, SysOf<BE, PR>&) {
    using C = AnalyticSceneCloth<PR>;
    simulator.addCube(tinym::vec3(0, 0, 0), 4, 1.2);                 // id 0
    simulator.addCloth(24, 1.0, tinym::vec3(0, 1.2, 0),
                       C::kstretch, C::kshear, C::kbend, C::thick, C::mass);
}

template <typename BE, typename PR>
void setupAnalyticCylinder(SimOf<BE, PR>& simulator, SysOf<BE, PR>&) {
    using C = AnalyticSceneCloth<PR>;
    simulator.addCylinder(tinym::vec3(0, 0, 0), 24, 1.2);            // id 0
    simulator.addCloth(24, 1.0, tinym::vec3(0, 1.2, 0),
                       C::kstretch, C::kshear, C::kbend, C::thick, C::mass);
}

// "analytic_plane": a grid floor retagged Plane (infinite half-space, local
// +Y up) + a 1.5 m sheet released 1 m above it. The grid's own extent is
// irrelevant to collision after P2 — the broad phase never culls a Plane pair
// on the source grid's AABB any more.
template <typename BE, typename PR>
void setupAnalyticPlane(SimOf<BE, PR>& simulator, SysOf<BE, PR>&) {
    using C = AnalyticSceneCloth<PR>;
    simulator.addGround(PlaneDirection::XZPlane, tinym::vec3(0, 0, 0), 6.0);  // id 0
    simulator.addCloth(28, 1.5, tinym::vec3(0, 1.0, 0),
                       C::kstretch, C::kshear, C::kbend, C::thick, C::mass);
}

template <typename BE, typename PR>
void postInitAnalyticPlane(SimOf<BE, PR>& simulator) {
    setColliderKind<BE, PR>(simulator, 0, ColliderKind::Plane);
    std::cout << "[Main] --scene analytic_plane: mesh 0 retagged Plane collider\n";
}

// "analytic_zoo": one 4 m sheet draped over a sphere / box / cylinder standing
// side by side on a Plane floor. Exercises three analytic kinds + the
// half-space in the SAME broad phase, i.e. multiple markers per cloth per
// substep and the D1 index invariant across four collider slots.
template <typename BE, typename PR>
void setupAnalyticZoo(SimOf<BE, PR>& simulator, SysOf<BE, PR>&) {
    using C = AnalyticSceneCloth<PR>;
    simulator.addGround(PlaneDirection::XZPlane, tinym::vec3(0, 0, 0), 8.0);  // id 0
    simulator.addSphere  (tinym::vec3(-1.5, 0.5, 0), 16, 1.0);   // id 1
    simulator.addCube    (tinym::vec3( 0.0, 0.5, 0),  4, 1.0);   // id 2
    simulator.addCylinder(tinym::vec3( 1.5, 0.5, 0), 24, 1.0);   // id 3
    simulator.addCloth(48, 4.0, tinym::vec3(0, 1.7, 0),
                       C::kstretch, C::kshear, C::kbend, C::thick, C::mass);  // id 4
}

template <typename BE, typename PR>
void postInitAnalyticZoo(SimOf<BE, PR>& simulator) {
    setColliderKind<BE, PR>(simulator, 0, ColliderKind::Plane);
    std::cout << "[Main] --scene analytic_zoo: floor retagged Plane; "
                 "sphere/box/cylinder keep their initializer colliders\n";
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
        { "pbd_cloth_ball", "corner-pinned PBD cloth + a 0.5 m Rigid ball drop",
          &setupPbdClothBall<BE, PR>, &postInitPbdClothBall<BE, PR> },
        { "pbd_cloth_ball_self", "pbd_cloth_ball with a CLOTH-tagged ball; "
                                 "per-object 자기 충돌 only (global self off)",
          &setupPbdClothBallSelf<BE, PR>, &postInitPbdClothBallSelf<BE, PR> },
        { "pbd_cloth_xyfold", "vertical (XY) PBD sheet buckling onto the floor; "
                              "self-collision toggle test bed (subSteps 3, iters 8)",
          &setupPbdClothXYFold<BE, PR>, &postInitPbdClothXYFold<BE, PR> },
        { "pbd_cloth_stack", "two PBD cloths stacked; cross-cloth two-way contacts",
          &setupPbdClothStack<BE, PR>, &postInitPbdClothStack<BE, PR> },
        { "pd_cloth", "pbd_cloth geometry solved by the CPU PD (Liu 2013) system",
          &setupPdCloth<BE, PR>, &postInitPdCloth<BE, PR> },
        { "analytic_sphere", "cloth dropped on a Sphere analytic collider",
          &setupAnalyticSphere<BE, PR>, nullptr },
        { "analytic_box", "cloth dropped on a Box analytic collider",
          &setupAnalyticBox<BE, PR>, nullptr },
        { "analytic_cylinder", "cloth dropped on a Cylinder analytic collider",
          &setupAnalyticCylinder<BE, PR>, nullptr },
        { "analytic_plane", "cloth dropped on a Plane (infinite half-space) collider",
          &setupAnalyticPlane<BE, PR>, &postInitAnalyticPlane<BE, PR> },
        { "analytic_zoo", "one sheet draped over sphere+box+cylinder on a Plane floor",
          &setupAnalyticZoo<BE, PR>, &postInitAnalyticZoo<BE, PR> },
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
