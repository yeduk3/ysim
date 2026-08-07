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
    const PR kstretch = 1e5, kshear = 1e5, kbend = 5e4;
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

// "pbd_cloth_flag": a wide VERTICAL sheet pinned along its whole LEFT edge,
// flying in a steady cross-wind — the tearing showcase (PbdSystem milestone
// 1 + the phase-2 render propagation). Everything here exists to make a rip
// reachable without touching a single env var:
//   * pinned along the full left COLUMN, not four corners: a flag has one
//     anchored edge, and a full column is what concentrates the load into the
//     edges just right of the pins (where the rip starts);
//   * wind mostly along +Z, i.e. through the sheet's own plane normal, with a
//     lift (+Y) and a lateral (+X) component so it flutters instead of
//     bulging into one static shape;
//   * tearRatio pulled down to 1.30 from the 1.4 default, because the wind
//     alone must be able to break it — the point of the scene is that it
//     tears on its own, not after a GUI drag.
// The sheet is a SQUARE 20x20 grid stretched 2.5x in x by the initializer's
// per-axis scale (the same field the inspector's scale path writes), so the
// vertex count stays exactly pbd_cloth's 400. The cells are therefore 2.5:1
// rectangles; Scene::pack measures rest lengths from the TRANSFORMED geometry
// so nothing is pre-stressed, the horizontal edges are simply longer (and so
// tear later) than the vertical ones.
template <typename BE, typename PR>
void setupPbdClothFlag(SimOf<BE, PR>& simulator, SysOf<BE, PR>& system) {
    const PR kstretch = 1e5, kshear = 1e5, kbend = 5e4;
    const PR mass = 0.1, thickness = 0.01;
    const Index n = 20;
    const PR size = 1, y = 1.2;
    const float aspect = 2.5f;   // flag is 2.5 m x 1 m after the x scale
    simulator.addGround(PlaneDirection::XZPlane, tinym::vec3(0, 0, 0), 8.0);
    Scene<BE, PR>::requestsGeneralMeshes.back().checkerboard = true;  // id 0
    // Vertical (XY) sheet. addPlane hands addGeneralMesh a Float behavior, so
    // the request is retagged to cloth here — the same idiom
    // setupPbdClothXYFold uses. Without the params swap thicknessOf /
    // springConstantsOf would read 0.
    simulator.addPlane(PlaneDirection::XYPlane, tinym::vec3(0, (float)y, 0), n,
                       size, mass, BehaviorType::Float);              // id 1
    {
        auto& req = Scene<BE, PR>::requestsGeneralMeshes.back();
        req.behaviorType   = BehaviorType::TriangularCloth;
        req.behaviorParams = ClothBehaviorParams<PR>{ kstretch, kshear,
                                                      kbend, thickness };
        // Per-axis stretch. BOTH copies are written for the same reason
        // Simulator::scaleObject writes both: the initializer params drive the
        // geometry Scene::pack bakes, the request mirror is what the inspector
        // displays and what the next scale delta composes against.
        const tinym::vec3 s(aspect, 1.0f, 1.0f);
        req.scale = s;
        if (req.initializer) req.initializer->getParams()->scale = s;
        // Pin the ENTIRE left column (col 0 of every row). MeshGridInitializer
        // lays an XY grid out as vid = row*n + col with
        //   px = col*length - size/2   (x, then scaled by `aspect` about the
        //                               center, which is the scale pivot)
        //   py = -row*length + size/2  (y)
        // so col 0 is x = -size/2 and the pinned position must be given in the
        // SCALED frame or the pack would yank the sheet at t=0.
        const float length = (float)size / (float)(n - 1);
        const float half   = (float)size / 2.0f;
        for (Index row = 0; row < n; ++row) {
            const uint32_t vid = (uint32_t)(row * n);
            const tinym::vec3 p(-half * aspect,
                                (float)y + (half - (float)row * length),
                                0.0f);
            req.fixedVertices.push_back(FixedVertex{ vid, p });
        }
    }
    // Steady cross-wind. Scene-global (SceneEnvironment::wind) and applied as
    // a per-PARTICLE force with no mass scaling (see
    // Simulator::applyEnvironmentForces), so with the 0.1 per-particle mass
    // here 0.981 would exactly cancel gravity — this is ~5.4x a particle's
    // own weight, i.e. a gale, not a breeze. Only wind-susceptible (cloth)
    // behaviors see it, so the floor is unaffected.
    //
    // The magnitude is MEASURED, not guessed. PBD only sees overstretch in
    // the PREDICTION, which needs a real velocity DIFFERENCE across an edge —
    // a quasi-static flag has none no matter how hard it is pushed. Sweep at
    // subSteps=3 / ratio 1.30, tearCount after 150 frames:
    //   |w| 2.4 -> 0 (steady state, never tears even at ratio 1.10)
    //   |w| 4.0 -> 3        |w| 5.2 -> 27        |w| 6.4 -> 19
    // 5.2 is the first magnitude that opens a visible rip inside ~1.5 s.
    // Note the engine's wind is a constant body force with NO drag term, so a
    // fully detached scrap keeps accelerating and blows out of frame — that is
    // the force model, not the tear rule.
    Scene<BE, PR>::environment.wind = tinym::vec3(0.75f, 1.90f, 5.00f);
    // Same operating point as the other CPU-solver scenes: the substep budget
    // is what the tear pass measures its PREDICTED overstretch against, so it
    // is part of the scene definition, not a detail.
    system.subSteps = 3;
    system.subh     = system.h / PR(3);
    simulator.mlBroadPhase.floorExcludeDiag = 1e9f;   // exclude NOTHING
}

template <typename BE, typename PR>
void postInitPbdClothFlag(SimOf<BE, PR>& simulator) {
    simulator.usePbd = true;
    simulator.cdSubstepPeriod    = 1;   // same rationale as postInitPbdCloth
    simulator.refitSubstepPeriod = 1;
    simulator.enableSelfCollisions = true;
    simulator.pbd.iterations = 8;
    // The scene's reason to exist. Budget 3/step (not the 4 default): a
    // smaller budget makes the rip PROPAGATE along the strained row instead
    // of opening several holes at once.
    simulator.pbd.tearEnabled     = true;
    simulator.pbd.tearRatio       = PR(1.30);
    simulator.pbd.maxTearsPerStep = 3;
    std::cout << "[Main] --scene pbd_cloth_flag: CPU PBD solver, left-edge "
                 "pinned 2.5x1 m flag in a cross-wind, TEARING ON "
                 "(ratio 1.30, budget 3/step, subSteps=3)\n";
}

// "ls_cloth_hang": a corner-pinned sheet with NO collider, solved by the
// Baraff-Witkin implicit-Euler sibling. Deliberately collision-free — this is
// the scene that isolates the material model and the modified-PCG solve, so a
// wrong condition force shows up here instead of hiding behind contact
// response. The subStep budget is 3, not the symplectic default's 60: taking
// a large stable step is the ONLY reason this solver exists.
template <typename BE, typename PR>
void setupLsClothHang(SimOf<BE, PR>& simulator, SysOf<BE, PR>& system) {
    const PR kstretch = 1e5, kshear = 1e5, kbend = 2e5;
    const PR mass = 0.1, thickness = 0.01;
    const Index n = 20;
    const PR size = 1, y = 0.6;
    simulator.addCloth(n, size, tinym::vec3(0, (float)y, 0), kstretch, kshear,
                       kbend, thickness, mass);                          // id 0
    {
        // Two adjacent corners only, so the sheet hangs and sags instead of
        // staying a taut plane — sag is what the stretch/bend model is read
        // off, and a 4-corner pin would make gravity a nearly pure membrane
        // load.
        auto& req = Scene<BE, PR>::requestsGeneralMeshes.back();
        const float h = (float)(size / 2);
        const uint32_t vids[2] = { 0, (uint32_t)n - 1 };
        const tinym::vec3 pos[2] = { { -h, (float)y, -h }, { h, (float)y, -h } };
        for (int i = 0; i < 2; ++i)
            req.fixedVertices.push_back(FixedVertex{ vids[i], pos[i] });
    }
    system.subSteps = 3;
    system.subh     = system.h / PR(3);
}

template <typename BE, typename PR>
void postInitLsClothHang(SimOf<BE, PR>& simulator) {
    simulator.useLargeSteps = true;
    std::cout << "[Main] --scene ls_cloth_hang: CPU Baraff-Witkin implicit "
                 "solver, 2-corner pinned sheet, no collider, subSteps=3\n";
}

// "ls_cloth": the pbd_cloth/pd_cloth geometry (floor + free sheet) under the
// same solver, so the contact FILTER (BW98 §5.3) is exercised on identical
// geometry — a one-flag A/B against the other three solvers.
template <typename BE, typename PR>
void setupLsCloth(SimOf<BE, PR>& simulator, SysOf<BE, PR>& system) {
    setupPbdCloth<BE, PR>(simulator, system);
    system.subSteps = 3;
    system.subh     = system.h / PR(3);
}

template <typename BE, typename PR>
void postInitLsCloth(SimOf<BE, PR>& simulator) {
    simulator.useLargeSteps = true;
    // Same reason the PBD/PD scenes force it: the filter is rebuilt from the
    // CONTACT SET every substep, so a held broad phase would let the sheet
    // drift through between refreshes.
    simulator.cdSubstepPeriod    = 1;
    simulator.refitSubstepPeriod = 1;
    std::cout << "[Main] --scene ls_cloth: CPU Baraff-Witkin implicit solver, "
                 "floor contact filter ON, subSteps=3\n";
}

// The three scenes below are EXISTING scenes re-solved by LargeStepsSystem —
// geometry comes from the SAME setup function, so each is a one-flag A/B
// against its PBD/symplectic twin (the pd_cloth principle). Only one-sided
// contact scenes are cloned: the BW98 constraint filter is a per-vertex
// operator and cannot express a two-way cloth-cloth or self row at all, so
// `pbd_cloth_stack` (cross-cloth) and `pbd_cloth_xyfold` (self-collision test
// bed) have NO ls_ twin — cloning them would ship two scenes whose whole
// point this solver silently drops. Those stay PD's.

// "ls_cloth_ball": pbd_cloth_ball geometry — corner-pinned sheet, 0.5 m Rigid
// ball dropped through it. Exercises the filter against a MOVING analytic
// Sphere collider. Caveat this scene makes visible: LargeStepsSystem has no
// cloth→rigid writeback (no rigidDelta), so the ball is not supported by the
// sheet — it drives the cloth and keeps falling.
template <typename BE, typename PR>
void setupLsClothBall(SimOf<BE, PR>& simulator, SysOf<BE, PR>& system) {
    setupPbdClothBall<BE, PR>(simulator, system);
    system.subSteps = 3;
    system.subh     = system.h / PR(3);
}

template <typename BE, typename PR>
void postInitLsClothBall(SimOf<BE, PR>& simulator) {
    simulator.useLargeSteps = true;
    simulator.cdSubstepPeriod    = 1;   // same rationale as postInitLsCloth
    simulator.refitSubstepPeriod = 1;
    std::cout << "[Main] --scene ls_cloth_ball: CPU Baraff-Witkin implicit "
                 "solver, corner-pinned cloth, Rigid ball drop "
                 "(no cloth→rigid coupling yet — the ball is not held up)\n";
}

// "ls_cloth_flag": pbd_cloth_flag geometry — left-column pinned 2.5x1 m sheet
// in the same gale. TEARING IS OFF and cannot be turned on here: tearing is a
// PbdSystem pass, so this scene is the flag's dynamics only. What it shows
// that the hanging scene cannot: a large wind load carried by the implicit
// solve without the substep budget the explicit path needs.
template <typename BE, typename PR>
void setupLsClothFlag(SimOf<BE, PR>& simulator, SysOf<BE, PR>& system) {
    setupPbdClothFlag<BE, PR>(simulator, system);
    system.subSteps = 3;
    system.subh     = system.h / PR(3);
}

template <typename BE, typename PR>
void postInitLsClothFlag(SimOf<BE, PR>& simulator) {
    simulator.useLargeSteps = true;
    simulator.cdSubstepPeriod    = 1;
    simulator.refitSubstepPeriod = 1;
    std::cout << "[Main] --scene ls_cloth_flag: CPU Baraff-Witkin implicit "
                 "solver, left-edge pinned 2.5x1 m flag in a cross-wind, "
                 "NO tearing (PBD-only pass), subSteps=3\n";
}

// ls_analytic_sphere lives further down, next to the analytic scenes it
// clones — setupAnalyticSphere is declared after this point.

// "pd_cloth": the pbd_cloth scene solved by the CPU Projective Dynamics
// sibling instead. Identical geometry on purpose — the two CPU solvers are
// only comparable if the scene is byte-identical, so `--scene pbd_cloth` vs
// `--scene pd_cloth` is a one-flag A/B.
template <typename BE, typename PR>
void setupPdCloth(SimOf<BE, PR>& simulator, SysOf<BE, PR>& system) {
    const PR kstretch = 1e5, kshear = 1e5, kbend = 2e5;
    const PR mass = 0.1, thickness = 0.01;
    simulator.addPlane(PlaneDirection::XZPlane, tinym::vec3(0, 0, 0), 24, 3.0,
                       0.1, BehaviorType::Float);
    simulator.addCloth(20, 1, tinym::vec3(0, 0.6, 0), kstretch, kshear, kbend,
                       thickness, mass);
    // PD is an implicit solve: the global step is unconditionally stable and
    // the momentum term keeps stiffness iteration-independent, so it does NOT
    // want the 60-substep budget the symplectic default carries. 3 substeps x
    // 10 iterations is the paper's operating point (Bouaziz 2014 §7.3).
    system.subSteps = 3;
    system.subh     = system.h / PR(3);
    simulator.mlBroadPhase.floorExcludeDiag = 1e9f;   // exclude NOTHING
}

template <typename BE, typename PR>
void postInitPdCloth(SimOf<BE, PR>& simulator) {
    simulator.usePd = true;
    // Contacts are resolved per substep from the narrow-phase rows (in-energy
    // unilateral constraints), so the broad phase must be fresh every substep
    // for the same reason PBD's is — a held set would let the cloth drift
    // through.
    simulator.cdSubstepPeriod    = 1;
    simulator.refitSubstepPeriod = 1;
    std::cout << "[Main] --scene pd_cloth: CPU Projective Dynamics solver, "
                 "subSteps=3, PD iterations=10\n";
}

// "pd_cloth_10k": pd_cloth at a solver-relevant vertex count. The 20x20 sheet
// pd_cloth carries (441 verts) sits BELOW PdSystem::kParallelMinVerts, so it
// exercises the serial schedule and says nothing about how the local/global
// steps scale. 100x100 = 10201 verts puts every sweep on the parallel
// schedule and makes the per-element costs (strain projection, back
// substitution) the thing being measured instead of dispatch overhead.
// Geometry and material are otherwise byte-identical to pd_cloth.
template <typename BE, typename PR>
void setupPdCloth10k(SimOf<BE, PR>& simulator, SysOf<BE, PR>& system) {
    const PR kstretch = 1e5, kshear = 1e5, kbend = 2e5;
    const PR mass = 0.1, thickness = 0.01;
    simulator.addPlane(PlaneDirection::XZPlane, tinym::vec3(0, 0, 0), 24, 3.0,
                       0.1, BehaviorType::Float);
    simulator.addCloth(100, 1, tinym::vec3(0, 0.6, 0), kstretch, kshear, kbend,
                       thickness, mass);
    system.subSteps = 3;
    system.subh     = system.h / PR(3);
    simulator.mlBroadPhase.floorExcludeDiag = 1e9f;   // exclude NOTHING
}

template <typename BE, typename PR>
void postInitPdCloth10k(SimOf<BE, PR>& simulator) {
    simulator.usePd = true;
    simulator.cdSubstepPeriod    = 1;
    simulator.refitSubstepPeriod = 1;
    std::cout << "[Main] --scene pd_cloth_10k: CPU Projective Dynamics solver, "
                 "100x100 sheet (10201 verts), subSteps=3, PD iterations=10\n";
}

// "pd_cloth_ball" / "pd_cloth_stack" / "pd_cloth_xyfold": the three PBD
// contact scenes solved by PD instead — geometry comes from the SAME setup
// function (byte-identical A/B, the pd_cloth principle), the wrapper only
// moves the System to PD's operating point (3 substeps; the implicit global
// solve is what buys the large stable substep). What each one showcases in
// the PD contact machinery:
//   ball   — one-sided in-energy planes against a falling Rigid's analytic
//            Sphere, incl. the rigidDelta cloth→body coupling;
//   stack  — cross-cloth two-way rows, each side's share landing in its own
//            prefactored system (PD-5's scene, interactively);
//   xyfold — the self-collision toggle test bed: per-object 자기 충돌 pill
//            feeds self rows through the same two-way path (PD-6's physics).
template <typename BE, typename PR>
void setupPdClothBall(SimOf<BE, PR>& simulator, SysOf<BE, PR>& system) {
    setupPbdClothBall<BE, PR>(simulator, system);
    system.subSteps = 3;
    system.subh     = system.h / PR(3);
}

template <typename BE, typename PR>
void postInitPdClothBall(SimOf<BE, PR>& simulator) {
    simulator.usePd = true;
    simulator.cdSubstepPeriod    = 1;   // same rationale as postInitPdCloth
    simulator.refitSubstepPeriod = 1;
    simulator.enableSelfCollisions = true;
    std::cout << "[Main] --scene pd_cloth_ball: CPU PD solver, corner-pinned "
                 "cloth, Rigid ball drop, subSteps=3\n";
}

template <typename BE, typename PR>
void setupPdClothStack(SimOf<BE, PR>& simulator, SysOf<BE, PR>& system) {
    setupPbdClothStack<BE, PR>(simulator, system);
    system.subSteps = 3;
    system.subh     = system.h / PR(3);
}

template <typename BE, typename PR>
void postInitPdClothStack(SimOf<BE, PR>& simulator) {
    simulator.usePd = true;
    simulator.cdSubstepPeriod    = 1;
    simulator.refitSubstepPeriod = 1;
    simulator.enableSelfCollisions = true;
    std::cout << "[Main] --scene pd_cloth_stack: CPU PD solver, two stacked "
                 "cloths, two-way in-energy contacts, subSteps=3\n";
}

template <typename BE, typename PR>
void setupPdClothXYFold(SimOf<BE, PR>& simulator, SysOf<BE, PR>& system) {
    // Already sets subSteps = 3 itself — the PBD scene runs at the same
    // interactive settings, so the wrapper adds nothing.
    setupPbdClothXYFold<BE, PR>(simulator, system);
}

template <typename BE, typename PR>
void postInitPdClothXYFold(SimOf<BE, PR>& simulator) {
    simulator.usePd = true;
    simulator.cdSubstepPeriod    = 1;
    simulator.refitSubstepPeriod = 1;
    // Global self OFF, exactly like the PBD scene: the per-object 자기 충돌
    // pill is the experimental variable.
    simulator.enableSelfCollisions = false;
    std::cout << "[Main] --scene pd_cloth_xyfold: CPU PD solver, vertical "
                 "(XY) sheet, subSteps=3, global self OFF — flip the mesh's "
                 "자기 충돌 pill to A/B it\n";
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

// "ls_analytic_sphere": analytic_sphere geometry — a 24x24 sheet released on a
// static 1.2 m Sphere collider, solved by LargeStepsSystem. The purest read of
// the contact filter: one analytic target, no rigid body, no pins, so a drape
// that slides off or a sheet that sticks is the filter's doing and nothing
// else's.
//
// Deliberately the SPHERE scene and not analytic_zoo: the zoo's sheet is
// 48x48 (6912 DOF) and this solver assembles + CG-solves on the CPU per
// substep, so the zoo is a perf experiment, not a correctness scene.
template <typename BE, typename PR>
void setupLsAnalyticSphere(SimOf<BE, PR>& simulator, SysOf<BE, PR>& system) {
    setupAnalyticSphere<BE, PR>(simulator, system);
    system.subSteps = 3;
    system.subh     = system.h / PR(3);
}

template <typename BE, typename PR>
void postInitLsAnalyticSphere(SimOf<BE, PR>& simulator) {
    simulator.useLargeSteps = true;
    simulator.cdSubstepPeriod    = 1;
    simulator.refitSubstepPeriod = 1;
    std::cout << "[Main] --scene ls_analytic_sphere: CPU Baraff-Witkin "
                 "implicit solver, sheet draped on a Sphere collider, "
                 "subSteps=3\n";
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
        { "pbd_cloth_flag", "left-edge pinned 2.5x1 m PBD flag in a cross-wind; "
                            "TEARING enabled (ratio 1.30, budget 3/step)",
          &setupPbdClothFlag<BE, PR>, &postInitPbdClothFlag<BE, PR> },
        { "pd_cloth", "pbd_cloth geometry solved by the CPU PD (Liu 2013) system",
          &setupPdCloth<BE, PR>, &postInitPdCloth<BE, PR> },
        { "pd_cloth_10k", "pd_cloth geometry at 100x100 (10201 verts); the PD "
                          "scene above kParallelMinVerts, for solver scaling",
          &setupPdCloth10k<BE, PR>, &postInitPdCloth10k<BE, PR> },
        { "pd_cloth_ball", "pbd_cloth_ball geometry under PD; one-sided "
                           "in-energy contacts + rigid coupling",
          &setupPdClothBall<BE, PR>, &postInitPdClothBall<BE, PR> },
        { "pd_cloth_stack", "pbd_cloth_stack geometry under PD; cross-cloth "
                            "two-way in-energy contacts",
          &setupPdClothStack<BE, PR>, &postInitPdClothStack<BE, PR> },
        { "pd_cloth_xyfold", "pbd_cloth_xyfold geometry under PD; self-collision "
                             "toggle test bed (subSteps 3, PD iters 10)",
          &setupPdClothXYFold<BE, PR>, &postInitPdClothXYFold<BE, PR> },
        { "ls_cloth_hang", "2-corner pinned sheet, no collider, solved by the "
                           "CPU Baraff-Witkin (1998) implicit system",
          &setupLsClothHang<BE, PR>, &postInitLsClothHang<BE, PR> },
        { "ls_cloth", "pbd_cloth geometry under the Baraff-Witkin implicit "
                      "system; contact filter on the floor",
          &setupLsCloth<BE, PR>, &postInitLsCloth<BE, PR> },
        { "ls_cloth_ball", "pbd_cloth_ball geometry under the implicit system; "
                           "moving Sphere collider (no cloth→rigid coupling)",
          &setupLsClothBall<BE, PR>, &postInitLsClothBall<BE, PR> },
        { "ls_cloth_flag", "pbd_cloth_flag geometry under the implicit system; "
                           "cross-wind load, NO tearing",
          &setupLsClothFlag<BE, PR>, &postInitLsClothFlag<BE, PR> },
        { "ls_analytic_sphere", "analytic_sphere geometry under the implicit "
                                "system; pure one-sided contact filter",
          &setupLsAnalyticSphere<BE, PR>, &postInitLsAnalyticSphere<BE, PR> },
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
