# Plan — BDD-017 Ray-Pick Mechanization (`feat/bdd-017-ray-pick`)

> Owner: **Planner** writes; **Generator** executes; **Estimator** judges.
> Updated: 2026-05-09

## Course note: previous slice's verdict

Estimator turn 14 returned **NOTE** (clean — no BLOCK, no WARNING,
no NOTE). BDD-010 + BDD-004 pass. Nothing to fold.

## Goal

Promote `BDD-017` from `pending` to `pass` in
`docs/TEST_MATRIX.md` by adding **Block 14 in `runSelfTest`** that
mechanizes BDD-017 verbatim against `docs/TESTS.md#BDD-017`. The
existing ray-pick implementation in `main.cpp`'s mouse callback
(uses `Scene::rayTracedData.clickRayCollisions` populated by
`BroadPhase::queryClickRay`) is reachable directly from the harness
— no API extraction needed.

When this slice ships:
- 29/29 self-test PASS (was 27/27; +2 BDD-017 lines for the
  non-overlapping clause + overlapping-front-most clause).
- `docs/TEST_MATRIX.md` row `BDD-017` flips `pending → pass`.
- The Notes line's "GUI input simulation" caveat is finally
  resolved — the harness drives `queryClickRay` with a synthesized
  world-space ray, so no actual mouse / GLFW / ImGui surface is
  needed.

## Scope

`docs/TESTS.md#BDD-017` (lines 161–167) is the binding spec:

> **Given** a scene with two objects whose screen-space projections
> do not overlap
> **When**  the user clicks on one object's screen position
> **Then**  that object becomes the selected object; the inspector
> displays its parameters.
>
> *Notes: also test the overlapping case — the front-most object
> (smallest ray `t`) wins.*

Two clauses, both mechanized in Block 14:

- **(a) Non-overlapping clause.** Two cubes at distinct world-space
  positions whose screen-space projections (under any reasonable
  camera) do not overlap. Synthesize a world-space ray that passes
  through cube₁'s volume but misses cube₂'s. Call
  `queryClickRay(ray)` + walk `clickRayCollisions[]` for the
  smallest `tmin`. Assert the closest hit's `obj` id matches
  cube₁'s id. Repeat the symmetric test for cube₂.

- **(b) Overlapping clause (front-most wins).** Two cubes
  positioned along the same line of sight from a chosen ray
  origin. Cast the ray; the BVH returns hits on both. The harness
  selects the smallest `tmin` (mirroring production's mouse-callback
  loop at `src/main.cpp:6588-6596`). Assert the front-most cube's
  id wins.

- **Spec substitution (mild).** "Click" semantics — there's no
  GLFW/ImGui in the harness. The substitution is "synthesize a
  world-space ray that mirrors what production constructs from a
  click" — the production callback (~line 6577) builds a ray from
  cursor pos via camera unprojection and calls
  `queryClickRay(ray)`. Block 14 skips the unprojection step and
  builds the ray directly. The BDD's load-bearing claim ("ray
  hits the clicked object's id") is fully satisfied by the BVH
  query path; the unprojection is harness-skippable plumbing.
  Pass label notes the substitution.

- **No API extraction.** `BroadPhase<METAL, PR, BVHMODE::SCENE,
  BVHPRIMITIVE::OBJECT>::queryClickRay(const Ray&)` is already a
  public method on `simulator.collisionPipeline.broadPhase`. The
  harness calls it directly. Production's mouse callback also
  reads `Scene<BE,PR>::rayTracedData.clickRayCollisions[i].obj`
  and `.tmin`; the harness reads the same fields.

- **`docs/TEST_MATRIX.md` row `BDD-017`** — promote `pending →
  pass`. Test address: `src/main.cpp::runSelfTest::BDD-017 (Block
  14)`.

- **No new D-NNN.** Pure mechanization slice; no architectural
  decision.

## Non-goals (this slice)

- **GUI / ImGui / GLFW input simulation.** The harness has no
  window. Block 14 calls `queryClickRay` directly with a
  world-space ray. Production-side ray construction (camera
  unprojection from cursor) is not exercised; that's a render-
  loop concern outside `runSelfTest`.

- **`Simulator::selectedObj` mutation as part of the assertion.**
  In production, the click callback writes
  `simulator->selectedObj = closestObj`. The harness asserts on the
  *closest object id* directly (via `clickRayCollisions[i].obj` +
  smallest `tmin` walk) without touching `selectedObj`. That's
  cleaner and matches the BDD's load-bearing semantic ("ray hits
  the clicked object's id").

- **Inspector update.** The BDD's "Then" line says "the inspector
  displays its parameters." Block 18 (BDD-018, pending) is the
  proper home for inspector-update mechanization. BDD-017 is
  only about the ray-vs-scene id resolution. The pass label
  scopes to that explicitly.

- **Cloth meshes as ray targets.** Cubes (`Float`-tagged) are the
  cleanest target — predictable AABB, stable across substeps. A
  ray-pick on cloth would have to deal with the cloth's animated
  geometry; out of scope.

- **Negative ray (no hit) case.** Not in BDD-017's wording. Skip.

- **Ray constructed by camera unprojection.** No camera in the
  harness. Block 14 constructs `Ray{origin, dir}` directly in
  world space.

- **Resolving `Q1`, `Q2`, `Q4`, `Q5`, `Q6`, `Q7`.**

## Todo

Ordered. Generator executes top-to-bottom.

1. **Branch hygiene.** Already on `feat/bdd-017-ray-pick` (off
   `main` at `ea6cc48`). No new branch. Commit prefix: `add:` (new
   test coverage).

2. **Re-read `docs/TESTS.md#BDD-017`** (lines 161–167) and the
   Notes line. Block 14 is authored from this verbatim, including
   the pass-label wording.

3. **Read the production click callback** at
   `src/main.cpp:6577-6597` so Block 14's structure mirrors
   production's read pattern (reset
   `numClickRayCollisions[0] = 0`, call `queryClickRay`, walk the
   array for smallest `tmin`).

4. **Author Block 14 in `src/main.cpp::runSelfTest`** after Block
   13 (just before the `if (failures == 0) { ... }` summary).
   Concrete shape:

   ```cpp
   // ---- Block 14: BDD-017 — Ray-pick selects nearest hit object. -------
   // TESTS.md#BDD-017 wording (verbatim, *not* the matrix-row label):
   //   Given a scene with two objects whose screen-space projections do
   //         not overlap
   //   When  the user clicks on one object's screen position
   //   Then  that object becomes the selected object; the inspector
   //         displays its parameters.
   //   Notes: also test the overlapping case — the front-most object
   //          (smallest ray t) wins.
   //
   // Substitution: harness has no GLFW/ImGui, so "click on screen
   // position" is mechanized as a world-space Ray fed directly to
   // BroadPhase::queryClickRay (mirroring the production path at
   // src/main.cpp:6577 minus the camera unprojection). The BDD's
   // load-bearing claim — that the ray hits the clicked object's id —
   // is fully exercised by the BVH query + smallest-tmin walk.
   // "Inspector displays its parameters" is a separate concern (BDD-018).
   {
       auto pickClosest = [&]() -> Index {
           auto& rt = Scene<Backend, Precision>::rayTracedData;
           Index n = rt.numClickRayCollisions[0];
           if (n == 0) return -1;
           Index closest = rt.clickRayCollisions[0].obj;
           float tmin = rt.clickRayCollisions[0].tmin;
           for (Index i = 1; i < n; ++i) {
               if (rt.clickRayCollisions[i].tmin < tmin) {
                   tmin = rt.clickRayCollisions[i].tmin;
                   closest = rt.clickRayCollisions[i].obj;
               }
           }
           return closest;
       };

       // --- Clause (a): non-overlapping screen projections. -----------
       // cubeA at x=-1.5, cubeB at x=+1.5; both at y=0, z=0; size=0.5.
       // Their AABBs in world space are disjoint along x. Two rays cast
       // along +x from far-left x: ray_A's z-line passes through cubeA's
       // bbox only; ray_B's z-line passes through cubeB's bbox only.
       resetScene();
       sim.addCube(tinym::vec3(-1.5f, 0.0f, 0.0f), /*tess=*/2,
                   /*size=*/0.5f, /*mass=*/0.1f);
       sim.addCube(tinym::vec3( 1.5f, 0.0f, 0.0f), /*tess=*/2,
                   /*size=*/0.5f, /*mass=*/0.1f);
       sim.initialize();
       const Index cubeAId = 0;
       const Index cubeBId = 1;

       // Ray A: through cubeA only.
       Ray rayA;
       rayA.origin = tinym::vec3(-1.5f, 0.0f,  10.0f);
       rayA.dir    = tinym::vec3( 0.0f, 0.0f, -1.0f);
       Scene<Backend, Precision>::rayTracedData.numClickRayCollisions[0] = 0;
       sim.collisionPipeline.broadPhase.queryClickRay(rayA);
       Index pickedA = pickClosest();
       if (pickedA != cubeAId) {
           fail("BDD-017 / ray hits the clicked object's id (non-overlapping)",
                "expected cubeA id=" + std::to_string(cubeAId) +
                ", got " + std::to_string(pickedA));
       } else {
           // Ray B: through cubeB only — symmetric.
           Ray rayB;
           rayB.origin = tinym::vec3( 1.5f, 0.0f,  10.0f);
           rayB.dir    = tinym::vec3( 0.0f, 0.0f, -1.0f);
           Scene<Backend, Precision>::rayTracedData.numClickRayCollisions[0] = 0;
           sim.collisionPipeline.broadPhase.queryClickRay(rayB);
           Index pickedB = pickClosest();
           if (pickedB != cubeBId) {
               fail("BDD-017 / ray hits the clicked object's id (non-overlapping)",
                    "expected cubeB id=" + std::to_string(cubeBId) +
                    ", got " + std::to_string(pickedB));
           } else {
               pass("BDD-017 / ray hits the clicked object's id (non-overlapping)");
           }
       }

       // --- Clause (b): overlapping case, front-most (smallest tmin). -
       // Two cubes along the same line of sight: cubeFront at z=+2,
       // cubeBack at z=-2 (further from origin). Both share the x=0 line.
       // Ray cast from z=+10 toward -z direction passes through cubeFront
       // first, then cubeBack. The smallest-tmin walk must pick cubeFront.
       resetScene();
       sim.addCube(tinym::vec3(0.0f, 0.0f,  2.0f), /*tess=*/2,
                   /*size=*/0.5f, /*mass=*/0.1f);
       sim.addCube(tinym::vec3(0.0f, 0.0f, -2.0f), /*tess=*/2,
                   /*size=*/0.5f, /*mass=*/0.1f);
       sim.initialize();
       const Index cubeFrontId = 0; // closer to ray origin
       const Index cubeBackId  = 1; // farther

       Ray rayDeep;
       rayDeep.origin = tinym::vec3(0.0f, 0.0f, 10.0f);
       rayDeep.dir    = tinym::vec3(0.0f, 0.0f, -1.0f);
       Scene<Backend, Precision>::rayTracedData.numClickRayCollisions[0] = 0;
       sim.collisionPipeline.broadPhase.queryClickRay(rayDeep);
       Index numHits = Scene<Backend, Precision>::rayTracedData.numClickRayCollisions[0];
       Index pickedDeep = pickClosest();
       if (numHits < 2) {
           fail("BDD-017 / overlapping case: front-most object (smallest ray t) wins",
                "expected ≥2 hits along through-line, got " +
                std::to_string(numHits));
       } else if (pickedDeep != cubeFrontId) {
           fail("BDD-017 / overlapping case: front-most object (smallest ray t) wins",
                "expected cubeFront id=" + std::to_string(cubeFrontId) +
                ", got " + std::to_string(pickedDeep));
       } else {
           pass("BDD-017 / overlapping case: front-most object (smallest ray t) wins");
       }
   }
   ```

5. **Run `./scripts/verify-light.sh`.** Doctest binaries should
   stay 159/159 + 1120/1120.

6. **Run `--self-test` 5+ times.** Expect **29/29 PASS**
   consistently. Ray-pick results should be deterministic (BVH
   build is deterministic per D-018; ray-vs-AABB intersection is
   pure float math).

7. **Optional bug-probe.** For confidence: temporarily flip the
   ray dir for clause (a), or swap cube positions for clause (b);
   confirm the assertion FAILs. Restore.

8. **Promote `BDD-017` matrix row.** `docs/TEST_MATRIX.md:31`:
   - Status: `pending → pass`.
   - Test address: `src/main.cpp::runSelfTest::BDD-017 (Block 14)
     — non-overlapping (two cubes at distinct x; rays through each
     pick the right id) + overlapping (two cubes on same line of
     sight; smallest-tmin wins) clauses. World-space Ray
     synthesized directly; camera unprojection is harness-
     skippable.`

9. **Update CURRENT_WORK / RESUME.** Four-line max as work
   proceeds. RESUME near end of turn.

10. **Stop and hand off to the Estimator.** No new BDDs, no API
    refactoring, no spec edits, no other matrix rows.

## Course corrections

- **Spec-vs-label discipline.** Block 14's pass labels are
  authored from the BDD's "Then" clause + Notes line verbatim.
  The matrix-row label "Ray-pick selects nearest hit object" is
  the compressed summary; the actual clauses break into
  (non-overlapping) + (overlapping front-most-wins).

- **Rays must be normalized.** `BVHNode::aabb.intersect(ray, hit)`
  may rely on `ray.dir` being unit-length for `tmin`/`tmax`
  semantics to be in world units. Block 14 uses unit-direction
  rays (e.g., `(0, 0, -1)`). Don't pass un-normalized directions.

- **Reset `numClickRayCollisions[0]` before each query.**
  Production does this at line 6579; the harness must do the same
  or hits accumulate across queries.

- **Mesh ids start at 0 and increment with each `addCube` /
  `addGround` / `addCloth`.** `cubeAId = 0`, `cubeBId = 1` after
  two `addCube` calls following `resetScene`. If a future change
  to `addGeneralMesh`'s id-assignment logic reorders this, the
  test surface needs updating; for now this is stable per D-015's
  invariant (`numMeshes`-pre-call read).

- **Ray-vs-AABB hits the BVH leaves.** The BVH built by
  `BroadPhase<SCENE, OBJECT>::build(scene)` represents per-mesh
  AABBs as leaves. `queryClickRay` walks down the tree and writes
  hit pairs to `clickRayCollisions`. Cube AABBs are tight enough
  (size 0.5 means ±0.25 in each axis) that the rays in clause (a)
  cleanly miss the non-target cube — the +1.5/-1.5 offset along x
  is 6× the cube extent, no false-positive overlap.

- **No D-NNN required.** The slice doesn't introduce new
  architectural patterns — Block 14 just calls existing public
  methods.

## What to read before writing code

- `docs/TESTS.md#BDD-017` (lines 161–167) — binding "Then" clause
  + Notes line. Verbatim source for Block 14.
- `src/main.cpp:6577-6597` — production click-ray callback. The
  harness mirrors its read pattern.
- `src/main.cpp::Ray` (~line 1643) and `RayHit` (~line 1649) —
  struct definitions.
- `src/main.cpp::Scene::RayTracedData` (~line 1747) — the buffer
  the BVH writes hits into. Per-thread? No — single-threaded
  populated by BVH walk in `queryClickRay`. Reset
  `numClickRayCollisions[0] = 0` before each call.
- `src/main.cpp::BVH::queryClickRay` (~line 3815) — the BVH walk
  that populates `clickRayCollisions`.
- `src/main.cpp::runSelfTest` Block 13 (~line 6232+) — recent
  block template; mirror the `resetScene` + `addCube` x2 setup
  pattern.
- `.agent/ESTIMATION.md` — turn 14 NOTE-clean verdict.
