# Architecture

> Owner: **Planner**. Generators read this to know boundaries; Estimators read this to detect violations.
> Updated: 2026-07-22 (main.cpp split into `include/sim/` fragment headers + tests/benches removed; supersedes the 2026-07-12 v2 rewrite).
> Companion docs: [VISION.md](VISION.md) (why), [COLLISION_PIPELINE.md](COLLISION_PIPELINE.md) (collision detail), [ROADMAP.md](ROADMAP.md) (when).

## 1. System purpose

ysim is a macOS GPU simulation engine and **personal research platform** for cloth dynamics, collision detection, and (kinematic) character motion. The design goal is *replaceable simulation stages*: solver, broad phase, narrow phase, and response should each be swappable so algorithms from papers can be implemented and compared quantitatively. (This reframes the v1 "authoring tool" purpose — see VISION.md §1.)

Dual-GPU strategy: **OpenGL renders, Metal compute simulates.** ImGui sits on the OpenGL backend. Single-threaded on the CPU side; all parallelism is GPU-internal.

## 2. Current reality (as-built, 2026-07)

The engine was split out of `src/main.cpp` on 2026-07-22: `src/main.cpp` is now a ~2,970-line **app shell** (preamble includes, the ordered `#include "sim/*"` list, `backendName<>`, `SimulatorBuilder`, `int main` → CLI dispatch + GLFW/ImGui main loop). The sim classes live in `include/sim/` **fragment headers** — cut region-by-region along the boundaries below, `#include`d in order by `main.cpp` after its preamble, so the TU is byte-equivalent to the old monolith. They rely on that preamble (`using Index`, std/GL/Metal includes) and on earlier fragments; **not independently compilable by design** (a later slice makes them standalone `sim/core/…` modules per §4). All embedded tests/benches were removed (§6.4); the validity self-test moved to `test/self_test_inline.hpp`.

### 2.1 include/sim/ fragment map

Ordered as `main.cpp` includes them (each was a contiguous slice of the old main.cpp):

| Header (`include/sim/`) | ~Lines | Contents |
|---|---|---|
| `core_types.hpp` | 657 | Backend tags (`CPU`/`CUDA`/`METAL`), `MemoryBlock`, `ByteMemoryPool`, `MetalGlobalContext`, `MetalKernelContext`, `VectorBase`, `Matrix`, `SparseMatrix` |
| `mesh_state.hpp` | 519 | `DebugLineGL`/`DebugPointGL`; `BehaviorType`, `ExternalForces`, `MeshState<BE,PR>`, `MeshAdjacency`, initializer base |
| `initializers.hpp` | 607 | `MeshGridInitializer`, `MeshFileInitializer`, `AssimpMeshFileInitializer`, sphere/cylinder/cube |
| `collision_pod.hpp` | 458 | selection/preset PODs, `NarrowCollision`, `Constraints`, cloth params, `TriangularClothBehavior<METAL>`, `FastGridClothBehavior<METAL>` |
| `material_quat.hpp` | 713 | `Material`, `Quat` math family, `MeshKinematicInitializer` (motion-system host) |
| `scene.hpp` | 896 | `GeneralMesh<BE,PR>`, `Ray`/`RayHit`, `SceneEnvironment`, `Scene<BE,PR>` |
| `spatial_hash.hpp` | 1,501 | `RadixSorter<METAL>`, `SpatialHashing<METAL>`, `MultiLevelSpatialHashing<METAL>` |
| `bvh.hpp` | 2,993 | `BVH<…,LINEAR,PRIMITIVE>` per-mesh LBVH; `BVH<…,SCENE,OBJECT>` TLAS; `BroadPhase` |
| `bruteforce.hpp` | 469 | `BruteForce<CPU/METAL>` narrow phase, `CollisionPipeline` |
| `simulator.hpp` | 4,145 | **`Simulator<BE,PR,System>`**: add* factories, `update`, transforms, rigid wiring, save/load, ray pick, motion driver |
| `explicit_system.hpp` | 446 | `ExplicitSystem<CPU/METAL>` (integration + force dispatch) |

`test/self_test_inline.hpp` (~6,140 lines, 40+ BDD validity blocks) is `#include`d last by `main.cpp` and reached via `--self-test`.

### 2.2 What is already cleanly extracted

`include/sim/*.hpp` (the fragment headers above — mechanical extraction, TU-equivalent, pre-§4). Fully decoupled: `include/scene_format.hpp` (headless, versioned JSON), `include/sim_config.hpp` (`RunConfig`), `include/MeshInspectorWindow.hpp` + `src/mesh_inspector_gui.cpp` (callback-struct decoupling — the model to imitate), rigid backends (`include/{Null,Euler,Bullet}RigidPhysicsBackend.hpp`, `include/RigidPhysicsTypes.hpp`), the motion stack (`include/{bvh_motion,motion_clip,motion_graph,motion_verb,kinematic_body,mesh_cluster}.hpp`), `include/FrameProfiler.hpp`, `include/SceneActionLog.hpp`, `include/primitive_geometry.hpp`.

### 2.3 Behaviors

`BehaviorType`: `TriangularCloth`, `FastGridCloth`, `Elastic` (reserved), `Rigid`, `Float`, `Fluid` (reserved), `Generator` (reserved), **`Kinematic`** (added ~2026-06; hosts the motion-graph/blend-space/verb system — previously undocumented). Cloth behaviors dispatch inside `ExplicitSystem::update`; Rigid and Kinematic are driven at the `Simulator::update` layer on the CPU.

Adding a behavior today touches: the enum, a params struct, a behavior kernel + `.metal` kernel, `ExplicitSystem`/`Simulator` dispatch, `scene_format` name tables, and the inspector combo. Not localized — a known cost until the System extraction (§4).

### 2.4 Rigid (as-built vs design)

`docs/design/rigid_physics_backend.md` specified a `RigidBackend` template param selected via `std::variant`. As built, `Simulator` hard-codes `BulletRigidPhysicsBackend rigid_`. The duck-typed 12-method contract is real and all three backends satisfy it; swapping is a one-line type edit. Rigid→mesh sync is **translation-only** (no rotation propagation) — an open gap. Quat marshalling: ysim `{w,x,y,z}` ↔ Bullet `(x,y,z,w)`.

### 2.5 Backend abstraction: reality check

`CUDA` is a tag with zero implementations. `CPU` is a type-system reservation (D-012 resolved testing via headless Metal self-test instead). The entire engine is monomorphized to `<METAL, float>`. The templates buy a no-vtable hot path and a future seam, at the cost of compile time and cognitive load. **Policy: keep the tags, stop pretending — no new code needs a CPU mirror** (see VISION.md §3).

## 3. Simulation pipeline (per frame)

Full detail with kernel names and file:line anchors in [COLLISION_PIPELINE.md](COLLISION_PIPELINE.md). Summary:

1. Every 10th frame: LBVH topology rebuild (`broadPhase.build`).
2. `applyEnvironmentForces` (gravity·mass + wind → per-vertex buffer).
3. Rigid: Bullet `step` once per frame, snap mesh to body centroid.
4. Kinematic: advance motion clock, `writePose` (one-way coupling; xPrev keeps the previous pose for the swept narrow phase).
5. Substep loop (~50–60×): refit(+swept enlarge) → broad detect → narrow (`narrow_pt_tri`) → per-vertex CSR bucketing → `snapshotXPrev` → `System::update` (Pass 1 forces, Pass 2 integrate **with collision response fused in**).
6. Frame boundary: `commitAndWait` → `syncPreviewFromState` → render.

Solver: explicit force evaluation + semi-implicit (symplectic) Euler. No implicit solve, no constraint solver — response is a velocity filter + position projection inside the integrate kernels.

## 4. Target module structure (extraction plan)

Strangler pattern: extract from main.cpp module by module; never a big-bang rewrite (`arch-test/` is frozen as a reference/lessons repository — its `PORT_MAP.md` de-risks step 1 below). Target dependency graph:

```
app (main loop, CLI dispatch)
 ├── editor/        ImGui panels (inspector pattern: callback structs, no engine types)
 ├── render/        OpenGL, shaders, id-buffer picking   — never sees Metal
 └── sim/
      ├── core/     backend tags, MemoryPool, VectorBase, Metal contexts, MeshState, Scene
      ├── collision/ BroadPhase variants + NarrowPhase + response params
      ├── system/   ISystem seam: ExplicitSystem | XPBDSystem | ImplicitSystem
      ├── rigid/    RigidPhysicsBackend contract (exists)
      └── motion/   mograph/kinematic (exists, feature-frozen)
persist/  scene_format, sim_config (exists, headless)
```

Extraction order and the *reason* each is worth it:

1. **core/** — mechanical, low-risk (arch-test already ported it once). Unblocks everything else.
2. **system/** — the first seam with a real second implementation waiting: `XPBDSystem` (ROADMAP M2). Interface sketch: `struct System { void snapshotXPrev(Scene&); void update(Scene&, SubstepCtx); }` — duck-typed like the rigid contract, selected per-`RunConfig` (`engine.system: Explicit|XPBD`), monomorphized via `std::variant` at the Simulator level. Per-substep granularity ⇒ dispatch cost irrelevant.
3. **collision/** — broad-phase selection is already runtime bools (`useSpatialHashing`, `useMultiLevelSH`, …); formalize as an enum-dispatched `BroadPhaseKind` and pull the classes out.
4. **Simulator split** — last: separate scene mutation (editor commands) from stepping (pipeline orchestration).

**Dispatch policy** (settles the template-vs-virtual tension between production and arch-test): inside kernels and per-vertex loops — templates/enum switch only, as today. At stage boundaries invoked ≤ once per substep — `std::variant`/`visit` or even virtual is fine; measurability is zero, replaceability is the point.

## 5. Boundaries (Estimator-enforced invariants)

Carried over from v1, still binding, with updates:

- **5.1 Backend boundary** — renderer, GUI, scene I/O never depend on Metal types. (Unenforceable inside one TU; becomes checkable as modules extract. Extracted headers already comply.)
- **5.2 No virtual dispatch on the per-vertex hot path.** Stage-boundary dispatch is exempt (§4).
- **5.3 No ad-hoc allocations on the hot path** — everything through `ByteMemoryPool`/`MemoryBlock`.
- **5.4 `BehaviorType` values are on-disk format** — append only, never reorder. (Now includes `Kinematic`.)
- **5.5 macOS/Metal-only platform surface.** Gate via backend tag, not `#ifdef`.
- **5.6 Single-machine determinism only** (BDD-102).
- **5.7 (new) GPU wedge safety** — any new kernel with a loop needs an iteration cap and index-range guard; any kernel writing positions goes through `sanitizeIntegrateOutput`-style non-finite handling (fast-math makes `isnan` unreliable; check exponent bits).
- **5.8 (new) xPrev discipline** — slot 10 (`xPrev`) is the swept narrow phase's contract: it must hold start-of-previous-substep positions for sim meshes and previous-frame pose for Kinematic meshes. Any new integrator/system must call `snapshotXPrev` at the same point in the substep or CCD silently degrades.

## 6. Known debts (acknowledged, scheduled or frozen)

1. main.cpp monolith → **split into `include/sim/` fragment headers (2026-07-22)**; main.cpp is now a ~2,970-line app shell. Still one TU (fragments are ordered `#include`s, not standalone modules) — the §4 standalone-module extraction (ROADMAP M1/M2) is the remaining step.
2. Docs drift: DECISIONS.md stops at D-042 (2026-05-14); the June–July motion/BVH-variant/analytic arc has no decision records. Policy: resume decision records from today **forward**; backfill only when a module is touched.
3. `TEST_MATRIX.md` cites stale main.cpp line numbers (now doubly stale — the self-test lives in `test/self_test_inline.hpp`).
4. Removed 2026-07-22: all embedded CLI bench harnesses (perf, ~15 runners + their `main` dispatch) and the `src/temp.cpp`/`test_temp.cpp`/`test/test_sap_topphase.cpp` scratch files. Kept as validity infra: `test/self_test_inline.hpp` (BDD self-test) and the doctest suite (`test/{scene_io,sim_config,motion_blend,primitive}_test.cpp`). Still unwired: edge LBVH kernels (no edge-edge narrow phase yet — intentional next step, COLLISION_PIPELINE §6).
5. `MeshAnimationWriter`/Alembic design doc unimplemented — **deliberately dropped** (VISION §3), doc kept for the record.
6. Rigid rotation propagation missing (translation-only vertex sync).

## 7. Open structural questions

- **Q-E — XPBD and the collision contract.** XPBD wants contacts as *constraints* solved in its iteration loop, not a velocity filter in the integrator. Does `NarrowCollision` + per-vertex CSR survive as the interface, with each System free to consume contacts its own way? (Working assumption: yes — detection is System-agnostic, response is System-owned.) Forced by ROADMAP M2.
- **Q-F — Where does friction state live?** Coulomb friction needs per-contact tangential info and possibly warm-starting across substeps. Buffer ownership (CollisionPipeline vs System) to be decided with the first friction implementation (ROADMAP M3).
- **Q-G — Kinematic skinning.** `kinematic::BodyProxy` writes joint spheres+bones, not a skinned character mesh. Is a proper LBS skin needed for the flagship demo, or do proxy capsules collide well enough? Forced by ROADMAP M4.
- Q-B (asset paths in saved scenes) and Q-C (export cadence) from v1: Q-B still open but low-pressure; Q-C dropped with Alembic.
