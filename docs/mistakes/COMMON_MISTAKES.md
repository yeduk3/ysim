# Common Mistakes (Active)

> Owner: **Generator** appends; **Planner** and **Estimator** read.
>
> Active list of recurring failure modes. Criteria for what belongs here is in `docs/roles/GENERATOR.md` — do not pad this file with one-off bugs.

## Format

```
## CM-NNN — <short title>

- Where: <files / functions / boundary affected>
- Low-level cause: <the immediate bug pattern>
- High-level cause: <the underlying reason this keeps happening>
- Fix direction: <what to do instead, or what check to add>
- First seen: <ISO date>   Last seen: <ISO date>
```

When an entry has not recurred for a while and the underlying cause is gone (architecture changed, a check was added, a refactor removed the foot-gun), graduate it to `OLD_MISTAKES.md`.

## Entries

## CM-001 — `::Material` shadowed by `scene_format::Material` inside `Simulator`

- Where: `src/main.cpp` — anywhere a `Simulator<BE,PR>` member references `Material` after `#include "scene_format.hpp"` brings `scene_format::Material` into a using-context.
- Low-level cause: both names resolve through ADL/qualified lookup; the compiler flags `Material` as ambiguous, not always at the first occurrence — the build can fail in a member function that *uses* `Material` long after the conflicting include.
- High-level cause: the persistence slice deliberately keeps `scene_format::*` POD types parallel to the C++-side runtime structs, but did not rename either side. The two will keep colliding as more fields are added.
- Fix direction: inside `Simulator` (and any future code that mixes both), qualify the C++-side struct as `::Material`. The same pattern likely applies to `Source`, `Transform`, `Object`, `Environment` if they ever grow runtime equivalents — check before adding a new collision.
- First seen: 2026-05-06   Last seen: 2026-05-06

## CM-002 — `Scene::pack()` double-frees initializers when re-run after an existing pack

- Where: `src/main.cpp:Scene<BE,PR>::pack` and any caller that may run pack twice (e.g. `Simulator::loadScene`, the new `Simulator::addSphere/addCube` followed by `simulator.initialize()`).
- Low-level cause: `meshes[i].initializer` is the **same pointer** stored in `requestsGeneralMeshes[i].initializer` (it's copied, not moved, by `addGeneralMesh` then `meshes.emplace_back`). When `pack()` runs a second time, `meshes.clear()` fires `~GeneralMesh()` which `delete`s the pointer; the rebuild loop then dereferences `requestsGeneralMeshes[i].initializer->getParams()` — freed memory → segfault.
- High-level cause: ownership of the initializer is split implicitly between two containers without an explicit owner-of-record. The first pack happened to "work" because `meshes` started empty; subsequent packs do not.
- Fix direction: treat `requestsGeneralMeshes` as the canonical owner. Before any `meshes.clear()`, null out each `meshes[i].initializer` so `~GeneralMesh` becomes a no-op delete. When `requestsGeneralMeshes` itself is wiped (only `loadScene` does this today), explicitly `delete r.initializer` first, then `clear()`. Search this file for `meshes.clear()` before adding a new one — every site needs both halves.
- First seen: 2026-05-06   Last seen: 2026-05-06

## CM-003 — `BVH::build` skips re-allocation when primitive count changes

- Where: `src/main.cpp:BVH<BE, PR, BVHMODE::LINEAR, PRIMITIVE>::build` (the `build(int oid, …)` overload, ~line 3384). Hits both the per-mesh tri-LBVH and the scene-level tree (which is itself a `LINEAR/EDGE` BVH wrapping object-AABB pairs).
- Low-level cause: `if (!tree.ptr) memoryAllocation();` skips allocation whenever the buffer pointer is non-null. On a re-build with a different primitive count, the existing `tree`/`treeParent`/`mortons`/radix arrays are sized for the **old** count; `radixSortGPU` and `buildTreeGPU` then dispatch threads that write past the allocation → memory corruption / segfault. The per-mesh trees mostly avoid this because their owning vector (`objTrees`) is reassigned when `scene.numMeshes` changes, which fresh-constructs each tree, but the scene-level inner `tree` persists across `BVH<SCENE, OBJECT>::build()` calls and is the canonical victim.
- High-level cause: caching pattern conflates "already allocated" with "already correctly sized". For BVHs sized to `2*N - 1` nodes, the size *is* part of the validity check.
- Fix direction: gate `memoryAllocation()` on both `!tree.ptr` and a size mismatch, e.g. `Index expectedNumNodes = numPrimitives > 0 ? 2*numPrimitives - 1 : 0; if (!tree.ptr || tree.size != expectedNumNodes) memoryAllocation();`. Same pattern likely applies to the SCENE-level container (`positions`, `indices`) — check before adding any new BVH variant that supports growth.
- First seen: 2026-05-07   Last seen: 2026-05-07

## CM-004 — Cloth force kernels hardcoded gravity and ignored the bound `externalForces` buffer

- Where: `src/metal/physics.metal::compute_cloth_grid_forces_fast` and `compute_tri_spring_forces`. The C++ side (`TriangularClothBehavior::setBuffer`, `FastGridClothBehavior::setBuffer` in `src/main.cpp` ~lines 1525, 1582) was already binding `mesh.externalForces.externalForces` at slot 7.
- Low-level cause: `compute_cloth_grid_forces_fast` did not declare buffer 7 at all. `compute_tri_spring_forces` declared it but never read it in the body. Both kernels computed gravity as `float3(0, simParams.G * m[id], 0)` from a hardcoded `SimParams::G` (`-9.8` in the METAL specialization). So the GUI Environment panel and the persistence-side `Scene::environment` had no connection to actual motion — gravity was a compile-time constant.
- High-level cause: a "the buffer is bound, so the kernel must consume it" assumption written into the persistence and env-forces slice plans, without verifying the kernel signatures matched the C++ binding plan.
- Fix direction: read `externalForces[id]` as the per-particle environmental force inside both kernels and drop the hardcoded gravity term. Air resistance (`vel * kair`) stays as an internal drag. Whenever a new C++-side `setBuffer` call adds an index, grep the corresponding kernel signature for that `[[buffer(N)]]` and a body reference — do not assume binding and consumption are in sync.
- First seen: 2026-05-07   Last seen: 2026-05-07

## CM-005 — Cloth tunnels through static ground despite broad/narrow contacts firing — **fixed by D-013, eligible for `OLD_MISTAKES.md` after one regression-free slice**

- Where: `src/main.cpp::Simulator::update`'s broad/narrow-phase substep loop (~line 4450 onward), `BVH<METAL, PR, SCENE, OBJECT>::detectCollisions` / `enlargeTrajectory` (~line 3915 / 3882), narrow phase `narrow_pt_tri` (`src/metal/bruteforce.metal`), and the cloth integrators `integrate_cloth` / `integrate_cloth_grid` (`src/metal/physics.metal`).
- Low-level cause (originally): `Scene::packedCollisionData.numBroadCollisions[0]` stayed `0` for every substep across a 60-frame cloth drop. Root cause: the cloth is a flat XZ plane (~zero Y-thickness AABB) moving ~0.021 m per substep at terminal velocity; without trajectory expansion, the cloth's AABB and the ground's AABB only overlap for an instant the broad-phase check rarely catches. **Partially fixed** in the BDD-007 slice by uncommenting `collisionPipeline.broadPhase.enlargeTrajectory(system.subh)` immediately after `refit()` — the swept AABB now overlaps the ground for one substep during the cloth's transit.
- Remaining issue: the cloth still tunnels (`minY` reaches ~-4.66 below the ground at -1) because the contact response only fires for *one* substep, and only for the subset of cloth particles whose distance to a ground triangle is within `radius + thickness = 0.022`. After that single substep, the cloth's particles end up split — some pushed back to `y ≈ -0.99`, others tunneled to `y ≈ -1.005`, the spring forces between them are small, and the broad phase no longer detects intersection because the cloth's AABB is now entirely below the ground's AABB.
- High-level cause: the narrow-phase distance check is a *snapshot* test, not a swept (continuous) collision test. Combined with thin moving meshes and one-substep-only contact windows, particles can pass through static surfaces between snapshots.
- Fix direction (next slice — cloth-CCD slice): replace the snapshot point-vs-triangle distance check in `narrow_pt_tri` with a swept-segment-vs-triangle check (cloth particle's previous position to current position vs the static triangle). Alternatively, increase `Simulator::radius` (currently `0.012`) so the narrow check accepts a wider band — quick fix but introduces more false positives. The proper fix is CCD; this slice's `enlargeTrajectory` change is a prerequisite (without it, even CCD cannot fire because the broad phase rejects the pair).
- First seen: 2026-05-07 (BDD-007 self-test Block 6).   Last seen: 2026-05-07 (3-of-4 clauses now PASS post-`enlargeTrajectory` fix; tunneling clause still FAILs).
- **2026-05-07 follow-up (D-013):** Tunneling clause now PASS. Closed by replacing snapshot point-vs-triangle distance check in `narrow_pt_tri` with swept-segment CCD using `xPrev` (start-of-prior-substep position) at buffer slot 10 + emitting **signed** distance so the integrator's `(thickness - distance) * n` push grows correctly for tunneled particles. Harness substep count bumped from 4 to 8 to keep one-substep-lag residual penetration below the BDD-007 strict tolerance. Graduate to `OLD_MISTAKES.md` after one slice without regression.
