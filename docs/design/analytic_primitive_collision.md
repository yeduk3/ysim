# Slice (c) — Analytic primitive collision (skip BVH/SH for sphere/cube/cylinder)

Status: design. Depends on slice (a)+(b) (commit `f075a1e`), which made
`ShapeType` accurate and GPU-visible. This doc specifies the collision
fast path that finally *consumes* that classification.

## 1. Goal & scope

When a collision pair involves an analytic primitive (Sphere / Cube /
Cylinder), do **not** route it through the triangle-soup pipeline
(BVH/SH broad phase → `narrow_pt_tri`). Instead test the *other*
object's vertices against the primitive's closed-form surface and emit
the same `NarrowCollision` rows the integrator already consumes.

In scope:
- **cloth/deformable point → primitive surface** (the high-value case:
  a sheet draping a sphere, a cube on cloth).
- **primitive vertex → primitive surface** can be skipped entirely
  (see §6 — primitives are rigid, Bullet owns rigid-vs-rigid).

Out of scope (unchanged): mesh↔mesh (File/Grid), self-collision,
reference-point constraints, the Bullet rigid backend.

## 2. Why this is safe to stage

The integrator response is fully decoupled from how a contact was
produced. `integrate_cloth*` (physics.metal) reads only
`NarrowCollision.collisionNormalAndDistance` (a world normal + signed
distance) grouped per query vertex via `vertColFacets` /
`vertColFacetsOffsets` (main.cpp:2389-2390). Any narrow stage that
emits well-formed `NarrowCollision` rows is indistinguishable to the
response. So the analytic path is a drop-in alternative producer, not
a response change.

`shapePair` is already carried end-to-end (BroadCollision →
NarrowCollision; bvh.metal / spatialhashing.metal / bruteforce.metal)
but never read for branching. It is the ready-made gate.

## 3. Data the GPU needs (new per-mesh buffer)

The analytic surfaces are defined by the initializer params + the live
transform. Today only the `ShapeType` enum reaches the GPU
(`meshShapes`, slot 10). We must also upload the surface parameters.

New CPU struct (mirror in a `.metal` header-style include or duplicated
struct, matching the existing `BroadCollision`/`NarrowCollision`
convention):

```cpp
struct alignas(16) AnalyticShape {
    tinym::vec4 center_radius;   // xyz = world center, w = radius (sphere/cyl)
    tinym::vec4 halfext_height;  // xyz = OBB half-extents (cube), w = cyl half-height
    tinym::vec4 axis_shape;      // xyz = cylinder axis (unit, world), w = (float)ShapeType
    // OBB orientation for cube / cylinder: rows of the world→local rotation.
    tinym::vec4 rot0, rot1, rot2;
};
```

Source of truth (CPU): `GeneralMesh::transformPosition`,
`GeneralMesh::scale`, `GeneralMesh::rotationQuat`, and the initializer
`params.size` (sphere/cylinder radius; cube half-extent — see the
param structs in main.cpp ~1454/1514/1574). Radius/extent must be
scaled by `GeneralMesh::scale` (per-axis; non-uniform scale on a sphere
makes it an ellipsoid — v1 may assume uniform scale and document the
limitation, or treat as OBB-bounded).

Build it next to `rebuildMeshKinds()` (main.cpp ~3005, where
`meshShapes` is filled) into a new
`VectorBase<METAL, AnalyticShape> meshAnalytic;` and bind it at the
next free Metal slot for **both** broad-phase kernels and the narrow
kernel. Buffer slots: append after the current highest used slot in
each kernel (sh_broadPhase currently uses 0..13 — add as 14; rebind
`SHBroadParams`/counters only if a renumber is unavoidable, otherwise
append). Rebuild on the same trigger as `meshShapes` (size change) AND
every pack, because center/rotation/scale change on translate/rotate/
scale/reset/load — unlike `meshShapes`, this buffer is **not**
size-invariant; refill its contents every substep from the live mesh
transforms (cheap: numMeshes entries).

## 4. Where the branch goes

Two viable insertion points; recommendation: **B (narrow-replace)** for
v1, then **A (broad-skip)** as the perf follow-up.

### Option A — broad-phase skip (max perf, more invasive)
In `refit`/`detectCollisions`, exclude primitive meshes from the
BVH/SH structures entirely (don't insert their triangles; don't query
their points if rigid). A separate dispatch generates
primitive-involved `BroadCollision` rows directly (cloth point → each
nearby primitive, using the primitive AABB only). Removes the BVH
build cost for primitive geometry — the stated goal — but touches the
hot broad-phase build path.

### Option B — narrow-phase replace (lower risk, v1)
Leave broad phase as is (it already produces point→triangle candidate
rows with `shapePair`). Add a kernel `narrow_pt_analytic` that, for
broad rows where `shapePair.target` is a primitive, ignores the
triangle and tests the query point against the analytic surface,
emitting `NarrowCollision`. `narrow_pt_tri` early-returns on those same
rows (one extra branch on `bc.shapePair`). Net: correct analytic
contacts immediately; BVH still built (perf win deferred to A).

v1 = B. It is independently shippable and de-risks the math/response
before the broad-phase surgery.

## 5. Analytic contact math (per query point `p`, world space)

Emit a `NarrowCollision` iff signed surface distance `d < radius +
thickness` (same gate `narrow_pt_tri` uses, bruteforce.metal ~122).
`collisionNormalAndDistance = float4(n, d)`, `n` = outward unit surface
normal at the closest point, `d` = signed distance (negative =
penetrating), matching the integrator's `pos += (thickness - d) * n`.

- **Sphere**: `c,r`. `dir = p - c; len = length(dir);
  n = dir/len; d = len - r`.
- **Cube (OBB)**: transform `p` into box-local via `rot`/`center`;
  `q = abs(plocal) - halfext`. Outside: `d = length(max(q,0))`,
  normal = local axis of max component rotated back to world. Inside
  (all q<0): `d = max(q.x,q.y,q.z)` (negative), normal = that axis.
- **Cylinder (finite, axis `a`, half-height `h`, radius `r`)**:
  decompose `p-c` into axial `t = dot(.,a)` and radial `rad`. Caps vs
  side via the standard finite-cylinder closest-point; sign by
  inside/outside test. v1 may approximate end caps as flat disks.

Keep the math in one `.metal` include so the (future) broad-skip path
and the narrow-replace path share it.

## 6. Pair-direction rules

`objPair`/`shapePair` are ordered {query, target}. Rules:

| query | target | action |
|---|---|---|
| deformable point | primitive | analytic test, emit (the main case) |
| primitive vertex | deformable tri | keep mesh path (cloth tri is not analytic) |
| primitive | primitive | **drop** — rigid↔rigid is Bullet's; Metal pipeline need not respond |
| mesh/grid | mesh/grid | unchanged |

"deformable" = behavior ∈ {TriangularCloth, FastGridCloth} (i.e.
`behaviorPair`, already on the row). Float/Rigid primitives don't
self-collide (`enableSelfCollisions=false` default; objPair gate at
spatialhashing.metal ~423), so primitive↔primitive Metal rows are
already rare; the explicit drop is just belt-and-suspenders.

## 7. Phasing

1. **(c-0) plumb data** — `AnalyticShape` struct, `meshAnalytic`
   buffer, fill from live transforms each substep, bind to narrow
   kernel. No behavior change yet (buffer unused). Build + tests green.
2. **(c-1) narrow-replace (Option B)** — `narrow_pt_analytic` kernel +
   `bc.shapePair` gate in `narrow_pt_tri`. Sphere first (simplest,
   exact), then cube (OBB), then cylinder. Validate: cloth draped on a
   sphere rests at radius+thickness, no tunneling at high `subSteps`,
   matches the old triangle-soup result within thickness.
3. **(c-2) broad-skip (Option A)** — exclude primitive triangles from
   BVH/SH build; direct primitive-involved broad emission. This is
   where the actual BVH-cost win lands. Gate behind a runtime toggle
   (parallel to `useSpatialHashing`) for A/B perf comparison and quick
   rollback.
4. **(c-3) cleanup** — once (c-2) is trusted, primitives stop being
   inserted into adjacency/BVH at all; measure broad_refit/broad_detect
   profiler scopes (FrameProfiler) before/after.

## 8. Risks / open questions

- **Non-uniform scale**: a scaled sphere is an ellipsoid; a scaled
  cube/cylinder is still an OBB. v1: handle cube/cyl exactly via OBB;
  sphere under non-uniform scale → either ellipsoid math or fall back
  to mesh path (decide in c-1; document).
- **CCD / tunneling**: `narrow_pt_tri` uses `xPrev` (slot 10) for
  swept contacts (D-013). The analytic test should likewise use the
  swept segment `p_prev→p` against the surface (ray-vs-sphere/OBB)
  when `subSteps` is low, else fast cloth tunnels through a small
  sphere. Reuse `system.subh` enlarge logic conceptually.
- **Buffer slot pressure**: confirm a free slot in each kernel; prefer
  append over renumber to avoid touching unrelated bindings.
- **Cloth-as-target**: when the primitive is the *query* and cloth the
  *target*, we still need cloth's triangles — that direction stays on
  the mesh path (table §6). Ensure both broad directions are still
  emitted for cloth↔primitive so the cloth-point↔sphere direction
  exists; it does today (6 rows per pair, both directions).
- **Plane**: `ShapeType::Plane` is still unassigned. A grid used as
  static ground is a strong analytic-plane candidate but only when
  Float/static (not cloth) — a natural (c-4), out of scope here.

## 9. Acceptance

- Cloth on sphere/cube/cylinder rests at the analytic surface
  (±thickness), visually identical to today, with no triangle-soup
  artifacts at low tessellation.
- `broad_refit`/`broad_detect` profiler time drops measurably for
  scenes dominated by primitive geometry after (c-2).
- `ysim_tests` green; add a primitive-rest regression to
  `primitive_test.cpp`.

---

# 10. Refinement — chosen architecture (user-directed) + review

This supersedes the §4 "Option B first" recommendation. The chosen
architecture is **broad-skip + per-particle analytic narrow**:

- Primitives are **excluded from broad phase**. `BVH<SCENE,OBJECT>`
  (main.cpp:4933 — confirmed two-level: per-object triangle LBVH
  `objTrees[i]` + scene `tree` over their root AABBs) skips
  `objTrees[i].build/refit/enlargeTrajectory` when
  `meshes[i].shapeType` is a primitive.
- Narrow phase gets a **dedicated analytic kernel**: 1 GPU thread per
  cloth particle, looping the (small) primitive array, applying the
  closed-form test from §5. primitive↔primitive is a documented
  no-op hook (Bullet owns rigid↔rigid for now).
- A pack-time-built primitive array `{shapeType, position, rotation,
  scale, objId, ...}` feeds the analytic kernel (no triangle geometry,
  no BVH).

## 10.1 Decisions made

- **D1 — broad-skip mechanics (keep the slot, empty the AABB).** Do
  NOT compact `objTrees`/`positions`/`indices`. The invariant "array
  index == objPair == statesOffsets subscript" is relied on across the
  pipeline (main.cpp:2238/2322/2468/5077). Instead: for a primitive
  `i`, skip `objTrees[i].build/refit`, and write a degenerate
  (inverted) AABB into `positions[i*6..]` so the scene `tree` never
  yields it as a broad pair. In `detectCollisions` (main.cpp:5055),
  `continue` past primitive `q` (query) and primitive `t` (target).
  Minimal, index-stable, reversible.

- **D2 — per-particle narrow needs NO sort.** The existing
  `narrowAndSortByVertices` (main.cpp:5190) exists because the
  triangle path emits unordered contacts that must be regrouped per
  query vertex into `vertColFacets`/`vertColFacetsOffsets`
  (main.cpp:2389-2390) for the integrator. The analytic kernel is
  *already* one-thread-per-particle, so thread `v` writes its own
  contact(s) directly into its `vertColFacetsOffsets[v]` region — the
  atomic-append + sort is skipped entirely for analytic contacts. This
  is a real simplification, not just a port. (Open: bounded contacts
  per particle — cap at K, e.g. 4, sufficient for convex primitives.)

- **D3 — refresh cadence: pack-time layout, per-substep contents.**
  Authoring edits (inspector translate/rotate/scale) already trigger a
  re-pack (dirty), so size/allocation at pack is correct. But the
  array CONTENTS must be refilled every substep from the live mesh
  transform, because: (a) the future rigid unification has Bullet
  moving primitives every substep; (b) even now an animated/teleported
  primitive must be current. Cost is O(numPrimitives) memcpy of a few
  vec4 — negligible. "Build once at pack" is correct only for the
  *buffer*, not its values. Update §3 accordingly.

- **D4 — array fields (rigid-ready).** Per primitive:
  `shapeType` (u32), `objId` (u32, == array index for the
  index-invariant), `center` (vec3, world), `rotationQuat` (vec4),
  `scale` (vec3), `halfExtents/radius` (vec3+f, scale-applied from
  initializer `params.size`), `behaviorType` (u32 — gate
  cloth-vs-primitive vs future primitive-vs-primitive),
  `prevCenter`+`prevRotation` (for swept CCD / future Bullet motion),
  `flags` (u32: collidable/enabled bit). This layout is the single
  contract that a later Bullet-integration slice fills from rigid body
  state instead of from initializer+inspector transform.

## 10.2 Open questions to resolve before c-1

- **Q1 — picking.** `BVH<SCENE,OBJECT>::queryClickRay` (main.cpp:5105)
  ray-casts `objTrees`. If object selection relies on this, skipping
  primitive `objTrees[i]` breaks picking primitives. MUST verify the
  selection path (the GL id-buffer pass vs this BVH ray). If picking
  uses the id buffer (likely — see the idpoint/id passes), no issue;
  if it uses `queryClickRay`, either keep a minimal AABB-only entry
  for primitives or pick them analytically. **Action: confirm before
  c-2 lands.**

- **Q2 — CCD / tunneling.** §8 stands and is now load-bearing: with
  primitives out of the swept-AABB BVH, fast cloth vs a small/thin
  primitive must use the swept segment `xPrev→x` (xPrev maintained
  per substep) against the analytic surface (ray-vs-sphere is cheap;
  ray-vs-OBB standard). Decide v1 = swept for sphere at minimum;
  static-sample is unacceptable for the very small-substep scenes the
  사용자 sim-env panel now lets users pick.

- **Q3 — non-uniform scale.** Cube/cylinder stay exact as OBB under
  any scale. Sphere under non-uniform scale is an ellipsoid: v1
  options — (i) ellipsoid distance (no closed-form exact distance;
  iterative), (ii) treat as OBB-bounded capsule/box (approx), or
  (iii) clamp to uniform scale on spheres in the inspector. Recommend
  (iii) for v1 simplicity unless ellipsoid draping is required.

- **Q4 — cloth-as-target direction.** Today both broad directions are
  emitted (cloth-pt→prim-tri AND prim-pt→cloth-tri). The analytic
  model only needs cloth-pt→prim-surface. prim-pt→cloth-tri is
  dropped (a rigid primitive does not need cloth to push it in the
  Metal pipeline — Bullet does, later). Confirm no current scene
  depends on a Float primitive being pushed by cloth via the Metal
  path (it does not — Float is integrator-inert). OK to drop.

- **Q5 — render vs collision split.** Primitives keep their tessellated
  render mesh; only collision goes analytic. Tessellation slider stays
  meaningful for visuals. No change to MeshGL / preview.

## 10.3 Revised phasing

- **c-0** primitive array struct + buffer + per-substep refill
  (D3/D4), bound to a new analytic-narrow kernel slot. Inert.
- **c-1** analytic narrow kernel (sphere, swept) writing directly to
  per-particle `vertColFacets` slots (D2); `narrow_pt_tri` unchanged.
  Primitives still in broad (double-covered) → compare analytic vs
  triangle-soup contact for the same scene as a correctness oracle.
- **c-2** broad-skip (D1) behind a runtime toggle; flip to analytic-
  only; measure `broad_refit`/`broad_detect`.
- **c-3** cube (OBB) then cylinder; resolve Q3.
- **c-4** (later) `ShapeType::Plane` for static grids; Bullet→array
  unification (fills D4 from rigid state); primitive↔primitive hook.

---

# 11. Resolutions & disambiguation

## 11.1 Q1–Q5 resolved (final)

- **Q1 picking — not a concern.** Selection is the off-screen id
  buffer pass, not `queryClickRay`. Skipping primitive `objTrees[i]`
  does not affect picking. `queryClickRay` may be left intact/dead for
  primitives.
- **Q2 CCD — DCD only for v1.** Primitives are static in the target
  scenes; discrete sampling of the current position `x` against the
  surface is sufficient. **No swept `xPrev→x` test in v1.** §8/Q2 and
  §5's "swept" notes are deferred to c-4 (moving/rigid primitives).
  The analytic kernel does not bind `xPrev`.
- **Q3 sphere — forced uniform scale.** Spheres clamp to uniform
  scale (enforce at the inspector scale widget: writing one axis sets
  all three for a Sphere). Analytic sphere radius = `params.size *
  scale` (scalar). No ellipsoid path.
- **Q4 — confirmed.** Only `cloth-pt → primitive-surface` is produced.
  The `primitive-pt → cloth-tri` direction is dropped (Float/Rigid
  primitive is integrator-inert in the Metal path; Bullet owns rigid
  push later).
- **Q5 — confirmed.** Primitives keep their tessellated render mesh;
  only collision is analytic. No MeshGL/preview/tessellation change.

## 11.2 Ambiguities found in review — decided

- **A1 — dispatch domain & cloth identification.** The analytic
  kernel dispatches over **all packed vertices** (`numPoints`,
  matching the existing packed layout), and each thread early-returns
  unless its owning mesh is deformable (TriangularCloth /
  FastGridCloth). Thread→mesh resolution: add a per-vertex `vertObj`
  (u32, owning mesh index) array built in `Scene::pack` alongside
  `statesOffsets` — O(numPoints) once per pack, removes an in-kernel
  binary search over `statesOffsets`. Behavior is read via the
  existing `meshBehaviors` (slot 9).

- **A2 — analytic contacts use a SEPARATE fixed-stride buffer, not
  `vertColFacets`.** Allocate `analyticContacts`
  (`numPoints * K` NarrowCollision, K=4 — D2's cap) +
  `analyticContactCount` (`numPoints` u32). Thread for vertex `v`
  writes its ≤K contacts at `v*K …`. The integrators
  (`integrate_cloth`, `integrate_cloth_grid` in physics.metal) get
  this buffer bound and run a second, fixed-stride contact loop
  (identical response math: `vn`-zero + `pos += (thickness-d)*n`)
  after the existing `vertColFacets` loop. Rationale: full decoupling
  from the triangle path's prefix-sum (`narrowAndSortByVertices`),
  no atomics, and clean c-1 coexistence (see A6). Supersedes D2's
  "write into vertColFacetsOffsets[v]" phrasing.

- **A3 — both cloth integrators must consume it.** `integrate_cloth`
  (TriangularCloth) and `integrate_cloth_grid` (FastGridCloth) both
  get the analytic buffer + the extra loop. Float/Rigid meshes are
  not integrated by these and are the *primitives* (targets), so no
  change there.

- **A4 — D4 correction: primitive array is COMPACT, not
  objPair-indexed.** The "objId == array index" clause in D4 was
  wrong — it conflated two arrays. The primitive array has
  `count = numPrimitives` (subset of meshes), each entry carrying its
  mesh `objId` as a *field*. The "index == objPair == statesOffsets
  subscript" invariant (D1) is a *separate* concern of the broad-phase
  arrays only and is untouched. `objId` is used solely to fill
  `NarrowCollision.objPair` for debug/consistency.

- **A5 — NarrowCollision metadata for analytic rows.** Fill
  `objPair = {clothObj, primitiveObj}`,
  `behaviorPair = {clothBehavior, primitiveBehavior}`,
  `shapePair = {Mesh, primitiveShape}`. The integrator response reads
  only `collisionNormalAndDistance`, so these are debug/forward-compat
  only — but fill them so the analytic rows are inspectable with the
  same tooling as triangle rows.

- **A6 — c-1 must NOT double-feed the integrator.** If primitives stay
  in broad during c-1, `narrow_pt_tri` would emit cloth-vs-primitive
  triangle contacts while the analytic kernel emits analytic ones →
  double push. Resolution: from **c-1**, gate `narrow_pt_tri` to
  early-return on rows whose target is a primitive (the `bc.shapePair`
  check). So only analytic feeds the response from day one; the
  triangle path's *broad build* for primitives is still wasted work
  until **c-2** removes it. The "oracle" comparison in c-1 is an
  offline/logged diff (dump both contact sets for one scene), never
  two live responses at once.

- **A7 — thickness/radius parity.** The analytic kernel takes the same
  `NarrowParams` (`radius`, per-cloth `thickness`) as `narrow_pt_tri`.
  Emit gate: `d < radius + thickness`. Emit signed `d`; the integrator
  already does `pos += (thickness - d) * n`. Rest distance therefore
  matches the triangle path exactly — no separate tuning.

- **A8 — scope of the broad skip (explicit).** Skip is **strictly
  `shapeType ∈ {Sphere, Cube, Cylinder}`**, never by behavior. A
  static ground (grid, `ShapeType::Mesh`) and imported OBJ colliders
  stay fully in broad/narrow — cloth-vs-ground is unchanged. `Plane`
  classification of static grids is the separate c-4.

- **A9 — primitive-as-Rigid motion (v1 boundary).** D3 refills the
  array from `mesh.transformPosition`/`rotationQuat`/`scale` (authoring
  truth). For a Rigid primitive Bullet drives motion, but per Q2 v1
  assumes static primitives, so reading authoring transform is
  acceptable for v1. Reconciling Bullet-driven per-substep motion into
  the array is explicitly c-4 (Bullet→array unification), not v1.

- **A10 — many-primitive cost.** Per-particle × all-primitives is
  O(P_cloth · N_prim) with no broad cull. Acceptable for the expected
  handful of primitives. If N_prim grows, an optional per-thread
  primitive-AABB reject before the exact test is a localized
  optimization (no architecture change). Not in v1.

## 11.3 Still-open (small, decide at implementation)

- **K cap value.** K=4 contacts/particle assumed. Convex primitive
  surfaces give ≤1 nearest contact normally; K=4 is slack for
  edge/corner cases of cube/cylinder. Confirm with the cube OBB math
  in c-3; raise only if corners under-report.
- **`vertObj` array placement.** Add to `PackedMeshData`
  (main.cpp:2362) next to `statesOffsets`, or as a sibling
  `VectorBase`. Trivial; decide when wiring c-0.
- **Inspector sphere-scale clamp UX.** Forcing uniform scale on
  spheres: clamp silently, or show all three axes locked? Cosmetic;
  decide with the UI change in c-3/Q3.

---

# 12. c-1 as built — deviation from A2 (recorded)

c-1 shipped. It **deviates from A2** and the deviation is intentional:

- **A2 said**: separate fixed-stride `analyticContacts[numPoints*K]`
  buffer + a second contact loop added to BOTH cloth integrators.
- **Built instead**: the analytic kernel (`narrow_pt_analytic`,
  bruteforce.metal) atomic-appends `NarrowCollision` rows into the
  **same shared `narrowCollisions`/`numNarrowCollisions`** the
  triangle path uses. The existing CPU sort in
  `narrowAndSortByVertices` folds them into `vertColFacets` and the
  **unchanged** `integrate_cloth` / `integrate_cloth_grid` consume
  them transparently.
- **Why**: strictly less invasive — zero edits to the two integrator
  kernels, no new integrator buffer slots, no second response loop to
  keep in lockstep, no K cap. Same observable result (the integrator
  math is identical regardless of contact source). The A2 rationale
  (avoid the prefix-sum / atomics) did not outweigh the cost of
  touching both hot integrators; the sort already exists and is
  proven.
- **Consequence**: D2's "no-sort per-particle write" and A2's separate
  buffer are **retired**. K-cap open item is moot. `analyticContacts`
  buffer is not created.

Other as-built notes:

- **Dispatch is per-cloth-mesh**, not the all-packed-verts + `vertObj`
  scheme of A1. CPU already knows which meshes are cloth (mirrors the
  integrator dispatch loop), so no GPU behavior buffer and no
  `vertObj` lookup is needed. **`PackedMeshData.vertObj` (c-0) is now
  unused** — kept inert/reserved (harmless; a future single-dispatch
  optimization or c-2 broad path may use it). Not removed to avoid
  churn.
- **Gate is Sphere-specific, not "any primitive".** `narrow_pt_tri`
  early-returns only when `shapePair.{x,y} == Sphere`. Cube/Cylinder
  pairs still flow through the triangle soup unchanged → **no
  regression** for cube/cylinder cloth collision. c-3 extends both the
  analytic kernel and this gate to Cube/Cylinder together.
- **DCD only** (Q2): tests current position; no `xPrev`/swept. The
  kernel does not bind xPrev.
- **Coexistence (A6) holds**: because the gate removes sphere pairs
  from `narrow_pt_tri`, only the analytic kernel feeds sphere contacts
  to the integrator — no double push, even though c-1 still builds
  primitives into broad (that wasted broad work is removed in c-2).
- **Two commit+wait per substep** (tri, then analytic) for cross-
  dispatch coherence of the shared atomic counter/array. Accepted for
  c-1 (correctness over perf; perf is c-2). Single-encoder merge is a
  possible later optimization.
- **Overflow**: shared with the pre-existing `narrow_pt_tri` pattern —
  kernels guard `outIdx >= maxNumCollisions` before writing, but the
  atomic counter can over-count. Practically safe (per-vertex sphere
  contacts ≪ `numPoints * approxColsPerPoints`); hardening the sort
  clamp is deferred (pre-existing, not c-1 scope).

**c-1 acceptance**: build green; `ysim_tests` 14/14, `ysim_primitive_tests`
9/9 (no regression — non-sphere paths untouched). Sphere-vs-cloth
behavior itself needs a **runtime visual check** (cloth draped on a
sphere rests at the surface, no pass-through) — not covered by the
headless suites; a primitive-rest regression is c-3 acceptance.
