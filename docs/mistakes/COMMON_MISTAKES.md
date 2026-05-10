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

_Note: CM-005 (cloth tunnels through static ground) graduated to `docs/mistakes/OLD_MISTAKES.md` on 2026-05-08 — the structural cause (snapshot narrow-phase) was replaced by swept-segment CCD in D-013, so the failure mode cannot recur in the same form._

_Note: CM-006 (slow-touch band drained vy off non-penetrating particles via the integrator's unconditional vn-zero) graduated to `docs/mistakes/OLD_MISTAKES.md` on 2026-05-09 — D-016 moved the vn-zero block inside the position-push's `(distance < thickness)` gate so detection and response gates are now symmetric. The failure mode cannot recur in the same form._

_Note: CM-007 (`rand()`-based jiggle in `MeshGridInitializer` breaks BDD-102 single-machine determinism) graduated to `docs/mistakes/OLD_MISTAKES.md` on 2026-05-09 — D-018 replaced `rand()` with a per-mesh seeded `std::mt19937`, so the global-RNG-state-leak failure mode cannot recur in the same form._

## CM-010 — BVH consumer treating leaf AABB-tmin as triangle-tmin; rotated geometry exposes the approximation

- Where: `src/main.cpp::BVH<METAL, PR, BVHMODE::LINEAR, BVHPRIMITIVE::TRIANGLE>::queryClickRay`'s leaf branch (~line 3867, fixed by D-024). Pattern likely recurs in any future BVH walk that wants "user-clickable" or "user-targetable" intersection semantics: shadow-ray tests, mouse hover hit-tests, pickup interactions, raycast queries.
- Low-level cause: the BVH's interior-node AABB filter (`if(! node.aabb.intersect(ray, hit)) return;`) returns `tmin/tmax` based on the AABB's extent, not the underlying triangle's. For axis-aligned geometry the AABB tightly bounds its triangle so AABB-tmin ≈ triangle-tmin; the consumer's smallest-tmin walk gets the right answer. For rotated geometry the AABB grows beyond the triangle (along the diagonal), and the AABB-tmin can be much smaller than the triangle's actual ray crossing — the leaf wins the consumer's smallest-tmin race even when the ray never crosses the triangle.
- High-level cause: writing the AABB's intersection result as if it were a primitive intersection conflates two different geometric questions. AABB-vs-ray asks "could the primitive plausibly be hit?" (a broad-phase reject filter); primitive-vs-ray asks "is the primitive actually hit, and where?" (the narrow-phase answer). They produce the same numeric value for axis-aligned primitives but diverge for rotated ones. The conflation stays invisible until the user can rotate things.
- Fix direction: every BVH consumer that needs a "user-facing" intersection answer must do real primitive-vs-ray at the leaf, not just rely on the AABB filter's `hit.tmin`. For triangles this is Möller–Trumbore (D-024 / D-022 family). For other primitives (line segments, spheres in future variants), use the corresponding closed-form test. The interior-node AABB filter stays — it's still a cheap reject that prunes most of the tree before the expensive primitive test. Future BVH variants must keep the two roles separate: AABB filter for traversal, primitive test for the answer.
- First seen: 2026-05-10   Last seen: 2026-05-10

## CM-009 — BVH walk that mixes leaf "write hit" with "recurse children" treats `childB` (primitive id) as a node index

- Where: `src/main.cpp::BVH<METAL, PR, BVHMODE::LINEAR, PRIMITIVE>::queryClickRay(const Ray&, const BVHNode&)` (~line 3815, fixed by D-020). Pattern likely recurs in any future BVH walk that follows the same shape: interior nodes encode `childA, childB` as node indices; leaves encode `childA == -1, childB == primitive_id`. If the walk writes the hit and then **falls through** to a generic "recurse if children are positive" block, the leaf's `childB` (a primitive id) gets dereferenced as `tree[childB]` — a random sibling node — and recursion explodes.
- Low-level cause: the leaf branch lacks an explicit `return` after writing the hit. Both `if (childA > 0)` and `if (childB > 0)` are evaluated next; for a leaf, `childA == -1 < 0` so the first is false, but `childB == primitive_id` is positive, so `tree[primitive_id]` recurses into an unrelated node.
- High-level cause: the BVH node struct uses a sentinel encoding (`childA < 0` marks leaf) but the walk treats post-write fall-through as the "no special case" path. Two semantics share one storage layout; the walk has to disambiguate.
- Fix direction: every BVH walk that writes a leaf hit must `return` from the leaf branch — leaves do not have node-index children. Concretely: `if (node.childA < 0) { ... write hit ...; return; }`. Equivalently, restructure as `if (interior) recurse; else leaf-action;`. Any future BVH variant (per-mesh narrow tree, scene-level tree, click-ray walk, ray-cast walk, debug-AABB walk) that copies this loop shape needs the same `return` discipline. Buffer caps (e.g., `approxColsPerRay = 4096`) only mask the symptom — they don't bound spurious recursion in trees with unboundedly-many "valid-looking" childB indices to follow.
- First seen: 2026-05-10   Last seen: 2026-05-10

## CM-008 — `BroadPhase::build` skips Float-tagged per-mesh BVH rebuild, so harness scene-swaps reuse stale tree data

- Where: `src/main.cpp::BVH<METAL, PR, BVHMODE::SCENE, BVHPRIMITIVE::OBJECT>::build` (~line 3907: `if(objTrees[i].tree.ptr && objTrees[i].objBehavior == BehaviorType::Float) continue;`). Triggered when consecutive `runSelfTest` blocks use `resetScene()` + new mesh creation but `numMeshes` matches the previous block's count, so `objTrees.size() != scene.numMeshes` is false and the per-slot rebuild loop runs against stale Float entries.
- Low-level cause: the build loop's continue is a production optimization — Float meshes don't change shape across `Simulator::initialize()` calls, so rebuilding their per-mesh BVH every init is wasted work. But the skip checks `objBehavior == Float` against the **previous** tree's stored value, not the new mesh's. After `resetScene()` + new addCube×2, the new meshes' behavior happens to be Float; the previous tree slot is also Float; the skip fires and the old AABBs persist. Click-ray queries then hit nothing (or hit the wrong object).
- High-level cause: the optimization assumes "same slot index" implies "same mesh"; that's true for production's incremental scene edits (add/remove individual meshes) but false for the harness's pattern of `resetScene` + full scene rebuild between blocks.
- Fix direction: the harness fix is `sim.collisionPipeline.broadPhase.objTrees.clear();` before `sim.initialize()`. This forces the size-mismatch branch in `broadPhase.build` (line 3900: `if(objTrees.size() != scene.numMeshes) objTrees = std::vector<TRI_LBVH>(scene.numMeshes);`), which reallocates with null `tree.ptr` and triggers a fresh rebuild of every slot. Production-side, a fix would be to either (a) gate the skip on `objTrees[i].objBehavior == scene.meshes[i].behaviorType` AND a content hash comparison, or (b) require an explicit `broadPhase.invalidate()` call before `Simulator::initialize()` after a scene mutation. v1 doesn't currently expose any "swap entire scene at the same numMeshes count" flow outside the harness, so the production cost is theoretical; the harness workaround is sufficient until that flow ships.
- First seen: 2026-05-10   Last seen: 2026-05-10
