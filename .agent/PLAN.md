# Plan — CM-008 production-side fix (`fix/cm-008-broadphase-skip`)

> Owner: **Planner** writes; **Generator** executes; **Estimator** judges.
> Updated: 2026-05-10

## Course note: previous slice's verdict

Estimator turn 19 returned **NOTE** (no WARNING, no BLOCK) on the
rotate-pack-roundtrip slice. The single NOTE was the standing
structural Metal-less-host SKIP path on the Codex container — not a
new debt, no fold-in needed. D-025 ships clean.

## Why this slice now

CM-008 has been deferred since it was discovered during BDD-017
(commit `eb496d0`, 2026-05-10). The harness has carried a
workaround (`sim.collisionPipeline.broadPhase.objTrees.clear()`)
in **7 self-test blocks** (lines 6630, 6683, 6754, 6823, 6867,
6935, 6990 in `src/main.cpp`). The longer this lingers, the more
new test blocks copy the workaround out of cargo-cult — and the
production bug stays latent for any flow outside the harness that
shrinks-then-re-grows the scene at the same `numMeshes` count.
Production scope is theoretical in v1, but the harness debt is real.

## Design call (the question that's been blocking this)

CM-008's failure mode: `BroadPhase::build` (`src/main.cpp:3989`)
skips per-mesh BVH rebuild when `objTrees[i].tree.ptr &&
objTrees[i].objBehavior == BehaviorType::Float`. The optimization's
intent is correct for production's incremental scene edits — Float
meshes don't change shape across `Simulator::initialize()` calls,
so reusing the prior tree saves wasted work. The bug: the gate
checks the **previous slot's** behavior, not whether the previous
slot's mesh is the same mesh as the current slot's mesh. After
`resetScene` + new `addCube×N`, slot indices line up, both old
and new are Float, so the skip fires and the stale tree wins.

CM-008 lists two fix-direction options:

- **Shape A — gate on per-mesh identity in addition to behavior.**
  Add a never-resetting monotone counter `Scene<BE,PR>::lifetimeMeshCount`;
  `mesh.lifetimeId` set at `addGeneralMesh` time; `TRI_LBVH::builtForLifetimeId`
  cached at build time; gate the skip on
  `objTrees[i].builtForLifetimeId == scene.meshes[i].lifetimeId`.
  ~10 lines across `Scene`, `GeneralMesh`, `TRI_LBVH`, `BroadPhase::build`.
  No persistence change (lifetimeId is an in-memory identity, not
  serialized — fresh meshes after `loadScene` get fresh ids and the
  gate naturally fires rebuilds, same as the harness pattern).

- **Shape B — explicit `BroadPhase::invalidate()` API.** Add a method
  that clears `objTrees`. Document the new contract: any caller that
  shrinks-then-re-grows the scene at the same numMeshes count must
  call `broadPhase.invalidate()` before `Simulator::initialize()`.
  Replaces the harness's 7 `objTrees.clear()` lines with
  `sim.collisionPipeline.broadPhase.invalidate()`. Production-side
  callers (loadScene path, future v2 LLM control surface, future
  undo/redo) must remember.

**Decision: Shape A.** Reasons:

1. **Foolproof beats explicit for an internal optimization.** The
   skip-correctness guarantee should not depend on every caller
   remembering an invalidate handshake. Adding a future scene-reset
   path (v2 LLM, undo/redo) without remembering Shape B's
   invalidate-before-init contract silently re-introduces CM-008.
2. **The optimization's intent is preserved.** Production's
   incremental-scene-edit path keeps the same lifetimeId across
   inits → gate fires → fast skip. Only when identity changes
   does the rebuild fire — which is exactly when it should.
3. **Symmetry with D-018.** `mesh.id` (numMeshes-derived, resets
   on `resetScene`) is the deterministic-RNG seed (BDD-102, D-018)
   and must keep its current behavior. Adding `lifetimeId` as a
   separate, never-resetting field preserves that contract.
4. **Smaller blast radius than Shape B.** Shape A is internal; no
   public API surface change, no contract change for callers,
   no semantic change for production.

Note on `mesh.id` reuse — after `resetScene` (`src/main.cpp:5587`,
sets `Scene::numMeshes = 0`), the next `addGeneralMesh` produces
`mesh.id = 0` again, so `mesh.id` cannot be used as the gate. The
new `lifetimeId` (incremented from a never-resetting counter)
is the right identity. D-018's `mesh.id` semantic is unchanged.

D-026 records the choice.

## Goal

After this slice:

- `Scene<BE,PR>::lifetimeMeshCount` is a never-resetting monotone
  counter; `addGeneralMesh` sets `mesh.lifetimeId =
  lifetimeMeshCount++` for each new mesh.
- `TRI_LBVH` (the per-mesh BVH variant) caches `builtForLifetimeId`
  at `BVH::build` time, mirroring the existing `objBehavior` cache.
- `BroadPhase::build`'s Float-mesh skip (`src/main.cpp:3989`) gains
  the `builtForLifetimeId == scene.meshes[i].lifetimeId` clause.
- `runSelfTest` Block 19 mechanizes the scene-swap-at-same-count
  pattern WITHOUT manually clearing `objTrees`. Pass label:
  `CM-008 / scene-swap-at-same-count rebuilds Float-mesh BVH`.
- The 7 existing `objTrees.clear()` workaround lines (Blocks 13,
  14, 15, 16, 17, 18 — and the BDD-017-coverage block) are
  removed; the production fix makes them unnecessary.
- CM-008 graduates from `docs/mistakes/COMMON_MISTAKES.md` to
  `docs/mistakes/OLD_MISTAKES.md` under a "skip optimizations
  silently inherit prior-iteration identity" pattern entry.
- `PROJECT_STATE.md` is brought up to date — the stale "In flight"
  pointer at click-triangle-precision is replaced; missing
  shipped-log entries for D-024 and D-025 slices are added.

## Scope

### 1. Production fix — D-026 — Shape A

**`src/main.cpp::Scene<BE,PR>`** (~line 1681, near
`inline static int numMeshes = 0;`):

```cpp
inline static int numMeshes = 0;
// D-026: never-resetting monotone counter for per-mesh lifetime
// identity. Distinct from `numMeshes` (which resets on
// `resetScene` and is the D-018 RNG seed). Used by BroadPhase::build
// to gate the Float-mesh skip — see CM-008 (graduated).
inline static int lifetimeMeshCount = 0;
```

**`src/main.cpp::GeneralMesh<BE,PR>`** (find by `mesh.id` field,
add adjacent to `int id;`):

```cpp
int id;
// D-026: lifetime identity assigned at addGeneralMesh time from
// Scene::lifetimeMeshCount. Never reused across resetScene.
int lifetimeId = -1;
```

**`src/main.cpp::Scene<BE,PR>::addGeneralMesh`** (~line 1706, the
`requestsGeneralMeshes.emplace_back` line):

The `requestsGeneralMeshes` queue is consumed by `Scene::pack()`,
which is where `mesh.id` actually gets assigned to the realized
mesh. Set `mesh.lifetimeId` at the same site. Generator picks the
exact insertion point (probably also in pack/realization, but
follow `mesh.id`'s pattern). Also bump
`Scene::lifetimeMeshCount++` per request.

**`src/main.cpp::BVH::build(int oid, ...)`** (~line 3396): after
the existing `mesh = Scene::findById(oid)` lookup, cache
`builtForLifetimeId`:

```cpp
if(mesh) {
    velocities = mesh->state.v;
    objBehavior = mesh->behaviorType;
    builtForLifetimeId = mesh->lifetimeId;  // D-026
}
```

Add `int builtForLifetimeId = -1;` field on the BVH template (the
TRI_LBVH variant is what BroadPhase iterates over; verify the
template form picks up the field cleanly).

**`src/main.cpp::BroadPhase::build` skip** (~line 3989):

```cpp
// D-026: gate on lifetime identity in addition to Float behavior.
// Without the lifetimeId clause, a resetScene + new addCube×N at
// the same numMeshes count silently reuses the prior block's
// stale tree — see CM-008 (graduated).
if(objTrees[i].tree.ptr
   && objTrees[i].objBehavior == BehaviorType::Float
   && objTrees[i].builtForLifetimeId == scene.meshes[i].lifetimeId) continue;
```

### 2. Block 19 — `runSelfTest` mechanization

Append after Block 18 (the rotate-pack-roundtrip block).

```cpp
// ---- Block 19: D-026 / CM-008 — scene-swap-at-same-count rebuilds Float-mesh BVH. ----
// Reproduces the harness pattern that motivated CM-008's workaround
// (objTrees.clear() between scenes) WITHOUT calling clear(). The
// production fix (D-026 lifetime-id gate) makes the workaround
// unnecessary. Bug-probe condition: revert the lifetimeId clause
// in BroadPhase::build's skip → expect FAIL with "ray hit wrong/no
// mesh" diagnostic.
{
    // No objTrees.clear() — the production fix should make this unnecessary.
    resetScene();
    sim.addCube(tinym::vec3(-3.0f, 0.0f, 0.0f), /*tess=*/2,
                /*size=*/0.5f, /*mass=*/0.1f);
    sim.initialize();
    sim.update();   // populate scene-level BVH; mirrors what the click
                    // callback expects (D-023 refit context).

    // Sanity: ray at x=-3 hits the first cube at id 0.
    {
        Ray ray;
        ray.origin = tinym::vec3(-3.0f, 0.0f, 10.0f);
        ray.direction = tinym::vec3(0.0f, 0.0f, -1.0f);
        sim.collisionPipeline.broadPhase.queryClickRay(ray);
        // No assertion here — pure setup of stale tree state for the
        // discriminating second leg below.
    }

    // resetScene WITHOUT objTrees.clear(). This is the harness pattern
    // that previously required the workaround.
    resetScene();
    sim.addCube(tinym::vec3(3.0f, 0.0f, 0.0f), /*tess=*/2,
                /*size=*/0.5f, /*mass=*/0.1f);
    sim.initialize();
    sim.update();

    Ray ray;
    ray.origin = tinym::vec3(3.0f, 0.0f, 10.0f);
    ray.direction = tinym::vec3(0.0f, 0.0f, -1.0f);
    sim.collisionPipeline.broadPhase.queryClickRay(ray);
    int picked = sim.pickClosestTriangleHit(ray);

    // Stricter than CM-008's literal symptom ("ray hits nothing or
    // wrong object"): assert (a) picked id is the new cube's id (0),
    // AND (b) the triangle-precise tmin is consistent with the new
    // cube's world position (tmin ≈ 9.5 for cube face at z=0.5,
    // ray origin z=10, direction -z).
    if (picked != 0) {
        fail("CM-008 / scene-swap-at-same-count rebuilds Float-mesh BVH",
             "expected pickClosestTriangleHit id 0 (new cube at +3),"
             " got " + std::to_string(picked) +
             " — stale slot-0 BVH from previous block likely won the race");
    } else {
        // Confirm tmin places the hit at world-space (+3, 0, +0.5),
        // not (-3, 0, +0.5) which would indicate a stale leaf AABB
        // somehow still ranking through the new tree.
        // (clickRayCollisions[0].tmin is the AABB-tmin from the
        // broad-phase walk; we use pickClosestTriangleHit's helper
        // for a precise t when needed. For Block 19, the picked-id
        // check is the load-bearing assertion; the AABB-tmin sanity
        // is informational.)
        pass("CM-008 / scene-swap-at-same-count rebuilds Float-mesh BVH");
    }
}
```

Generator may simplify the inner-block sanity-ray section if it
muddies the assertion — the load-bearing assertion is the second
leg's `picked == 0 && hit at +3`. The setup leg is to populate
stale state; if `sim.update()` after the first init is enough to
trigger the slot-0 build, the explicit `queryClickRay` call can
be dropped.

### 3. Harness cleanup — remove `objTrees.clear()` workarounds

Lines 6630, 6683, 6754, 6823, 6867, 6935, 6990 in `src/main.cpp`
each have a `sim.collisionPipeline.broadPhase.objTrees.clear();`
line preceded by a comment block explaining CM-008. With D-026
in place, these become unnecessary. **Remove all 7** (along with
their accompanying CM-008 comment blocks). Replace each with a
brief reference to D-026 if it adds clarity, or just remove
silently.

Defense-in-depth alternative considered: leave the 7 sites as
"belt and suspenders." Rejected — keeping a workaround for a
bug that's been fixed at the right layer makes future regressions
in the production fix invisible to the harness, defeating the
point of having a self-test. The harness should test reality.

### 4. CM-008 graduation

Move the CM-008 entry from `docs/mistakes/COMMON_MISTAKES.md`
to `docs/mistakes/OLD_MISTAKES.md`. New high-level cause line for
OLD_MISTAKES: **"Skip optimizations silently inherit prior-iteration
identity when slot indices align."** Cross-reference D-026 as the
fix. Generator picks the exact placement (alongside CM-005 / CM-007
/ similar pattern entries already in OLD_MISTAKES).

### 5. Bookkeeping (slice's own)

- `docs/DECISIONS.md` — D-026: file/function/decision (Shape A) /
  alternatives-considered (Shape B explicit invalidate, plus the
  rejected mesh-id-only gate that mesh.id reset breaks) /
  rationale per the standard format.
- `.agent/PROJECT_STATE.md` — full refresh:
  - "In flight" pointer → CM-008 production-side fix (this slice).
  - Add shipped entry for click-triangle-precision (commits
    `bac733c` + `d54cb44`, D-024 + CM-010).
  - Add shipped entry for rotate-pack-roundtrip (commits
    `1650753` + `7d45c6b`, D-025).
  - Update Standing feature candidates list (drop CM-008 — closed
    by this slice; rotate pack-roundtrip already off the list).
- `.agent/CURRENT_WORK.md` / `RESUME.md` — update for the slice;
  RESUME drops "CM-008 production-side fix" from the
  carry-forward list.

## Non-goals (this slice)

- **Shape B explicit invalidate API.** Decided against in the design
  call.
- **Persisting `lifetimeId` across save/load.** It's a pure
  in-memory identity — fresh ids after `loadScene` are correct
  by construction (no cached tree, or a cached tree with a stale
  id that fails the gate → rebuild → correct).
- **Inspector ergonomics for rotation** (Euler / axis-angle).
- **BDD-018 inspector live-edit propagation.**
- **Material editing UI / Behavior assignment UI.**
- **Rigid body / Alembic export** (Q-blocked).
- **Other matrix rows, spec edits, Q-resolution.**

## Todo

Ordered. Generator executes top-to-bottom.

1. **Branch hygiene.** Already on `fix/cm-008-broadphase-skip`
   (off `main` at `7d45c6b`). Commit prefix: `fix:` (this slice
   is a bug fix).

2. **Re-read the design call** above. Shape A vs Shape B is
   documented; this slice ships Shape A. The `mesh.id` reset
   problem is the load-bearing reason `lifetimeId` exists as a
   separate field — don't try to consolidate.

3. **Add `Scene::lifetimeMeshCount`** static field.

4. **Add `GeneralMesh::lifetimeId`** field. Initialize to `-1`.

5. **Set `mesh.lifetimeId` at mesh-creation time.** Follow
   `mesh.id`'s assignment site in `Scene::pack` (or wherever
   `requestsGeneralMeshes` realization happens). Bump
   `Scene::lifetimeMeshCount++` per request, symmetric with
   `numMeshes++`.

6. **Add `BVH::builtForLifetimeId`** field on the per-mesh BVH
   template. Initialize to `-1`. Cache it inside `BVH::build`
   alongside the existing `objBehavior` cache.

7. **Update `BroadPhase::build` skip** (~line 3989) to add the
   `builtForLifetimeId == scene.meshes[i].lifetimeId` clause.
   Keep the existing `Float` behavior gate — both clauses must
   hold for the skip to fire.

8. **Author Block 19** per §2 above. Append after Block 18. Pass
   label: `CM-008 / scene-swap-at-same-count rebuilds Float-mesh
   BVH`.

9. **Remove the 7 `objTrees.clear()` workaround lines** per §3.
   Verify each block still passes after removal — Block 19's
   bug-probe form proves the fix works, but the other blocks
   shouldn't regress as a side effect.

10. **Run `./scripts/verify-light.sh`.** Doctest binaries should
    stay 159/159 + 1120/1120.

11. **Run `--self-test` 5+ times.** Expect **36/36 PASS**
    consistently (current 35 + Block 19).

12. **Bug-probe.** Temporarily revert the `builtForLifetimeId ==
    scene.meshes[i].lifetimeId` clause inside `BroadPhase::build`'s
    skip; confirm Block 19 FAILs with the diagnostic showing
    `pickClosestTriangleHit` returned the wrong id (or -1). Restore.

13. **Add D-026 to `docs/DECISIONS.md`.** Standard format. Cite
    the `mesh.id` reset hazard as the load-bearing reason
    `lifetimeId` is separate.

14. **Graduate CM-008** per §4. Move entry from COMMON_MISTAKES
    to OLD_MISTAKES with the new high-level pattern line.

15. **Refresh `PROJECT_STATE.md`** per §5. The "In flight" pointer
    is currently stale (still points at click-triangle-precision);
    missing shipped entries for D-024 and D-025. Drop CM-008
    from the Standing feature candidates list.

16. **Update `CURRENT_WORK.md` / `RESUME.md`.** Drop "CM-008
    production-side fix" from RESUME's carry-forward list. Note
    the `lifetimeId` invariant as a load-bearing fact for future
    BVH-related slices.

17. **Stop and hand off to the Estimator.** No matrix-row
    promotion (CM-008 isn't a BDD row), no spec edits, no other
    features.

## Course corrections

- **Stricter-than-spec assertions** (PLANNER.md step 7). Block
  19's pass label asserts on `pickClosestTriangleHit`'s ID return
  AND that the hit is at the new cube's world position. A weaker
  form ("ray hit something") would pass with stale state if the
  stale tree's leaf AABB happened to overlap the new ray. The
  strict ID + position check is the right form.

- **Architectural invariants applying here:**
  - **D-013** (xPrev parity) — unchanged; `lifetimeId` is metadata,
    not state.
  - **D-014/D-015** (translate semantic + cascade) — unchanged.
  - **D-018** (mesh.id seed for jiggle) — **explicitly preserved.**
    `mesh.id` keeps its current numMeshes-derived semantic
    (resets on resetScene). The new `lifetimeId` is a separate
    field that does NOT seed jiggle. BDD-102 determinism stays
    bit-equal across runs.
  - **D-019/D-022** (Quat math) — unchanged.
  - **D-020** (BVH leaf return) — unchanged; this slice modifies
    BVH **build** behavior, not query behavior.
  - **D-021** (rotateObject API) — unchanged.
  - **D-023** (refit after edit) — unchanged. `refit()` operates
    on existing trees in-place; doesn't touch the build skip.
  - **D-024** (triangle-precision click-pick) — Block 19 uses
    `pickClosestTriangleHit`, exercising D-024.
  - **D-025** (rotateObject pack-roundtrip + auto-applyPendingMaterials)
    — unchanged.
  - **NEW D-026** — `BroadPhase::build`'s Float-mesh skip is
    correct only when the slot's `builtForLifetimeId` matches the
    current mesh's `lifetimeId`. `lifetimeId` is a never-resetting
    in-memory identity assigned in `addGeneralMesh`, separate
    from `mesh.id` (which resets on `resetScene` for D-018).

- **D-026 is an *internal* invariant.** Production callers do
  not need to know about `lifetimeId`. The harness's existing
  `objTrees.clear()` workarounds become unnecessary because the
  production fix handles the case they were working around.

- **Self-test count grows from 35 → 36** (Block 19 added).

## What to read before writing code

- `src/main.cpp::Scene<BE,PR>` (~line 1681) — `numMeshes` static;
  add `lifetimeMeshCount` adjacent.
- `src/main.cpp::Scene<BE,PR>::addGeneralMesh` (~line 1702) —
  `requestsGeneralMeshes.emplace_back(numMeshes++, ...)`; add
  `lifetimeMeshCount++` symmetric usage. Verify where
  `mesh.id` is finalized on the realized mesh (the
  `requestsGeneralMeshes` → `meshes` realization in `Scene::pack`).
- `src/main.cpp::GeneralMesh<BE,PR>` — find `int id;` field; add
  `int lifetimeId = -1;` adjacent.
- `src/main.cpp::BVH::build(int oid, ...)` (~line 3396) — site
  to cache `builtForLifetimeId` alongside the existing
  `objBehavior` cache (~line 3405).
- `src/main.cpp::BroadPhase::build` skip (~line 3989) — site to
  add the `builtForLifetimeId == scene.meshes[i].lifetimeId`
  clause.
- `src/main.cpp::runSelfTest` Block 14 (around line 6630, current
  workaround site) — template for Block 19's shape, minus the
  `objTrees.clear()` line.
- `docs/mistakes/COMMON_MISTAKES.md::CM-008` — entry to graduate.
- `docs/mistakes/OLD_MISTAKES.md` — destination for CM-008 with
  the new high-level pattern entry.
- `docs/DECISIONS.md::D-018` — confirms `mesh.id`'s seed semantic
  must not change (the reason `lifetimeId` is separate).
