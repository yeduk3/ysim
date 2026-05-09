# Plan — BDD-010 Collision Detection Mechanization (`feat/bdd-010-collision-detected`)

> Owner: **Planner** writes; **Generator** executes; **Estimator** judges.
> Updated: 2026-05-09

## Course note: previous slice's verdict

Estimator turn 13 returned **WARNING** (no BLOCK). One WARNING + one
NOTE:

- **WARNING (folded into this slice as todo 4):** Block 12's
  round-trip assertion re-normalizes `r2_round_trip` after the
  multiply, but never checks `r1_post_load` itself for unit norm —
  load-side norm regression could be silently absorbed by the
  re-normalize. `docs/TESTS.md#BDD-004`'s Notes line says "the
  stored quaternion remains unit-norm after each composition," so
  the post-load value is on the contract path. ~3 line fix.
- **NOTE (informational, no action):** component-wise orientation
  compare would false-fail on a sign-equivalent `q` vs `-q` flip.
  The persistence path doesn't canonicalize signs today, so this
  can't fire. If a future slice ever does, switch to an
  orientation-equivalence check. Not folded in.

Per PLANNER.md procedure step 2 ("folding small WARNINGs into the
next slice is fine when both items are small"), this slice's
primary work (BDD-010) is itself small and the BDD-004 fold-in is
~3 lines — comfortably within the small-WARNING budget.

## Goal

Promote `BDD-010` from `pending` to `pass` in `docs/TEST_MATRIX.md`
by adding **Block 13 in `runSelfTest`** that mechanizes BDD-010's
two clauses (positive AABB-overlap + negative non-overlap) verbatim
against `docs/TESTS.md#BDD-010`. Plus close the BDD-004 turn-13
WARNING with a 3-line assertion in Block 12.

When this slice ships:
- 27/27 self-test PASS (was 25/25; Block 13 adds 2 PASS lines for
  the positive + negative clauses).
- `docs/TEST_MATRIX.md` row `BDD-010` flips `pending → pass`.
- Block 12's BDD-004 round-trip gains a unit-norm assertion on
  `r1_post_load` before the re-multiply.
- Estimator turn-13 WARNING closed.

## Scope

### 1. Block 13 (BDD-010) — verbatim from `docs/TESTS.md#BDD-010`

```
Given two simulated meshes positioned so that their AABBs overlap
      on the next step
When  the broad-phase + narrow-phase pipeline runs once
Then  the resulting constraint set contains at least one contact
      pair (A, B) between the two objects.

Notes: also assert that the same scene with non-overlapping AABBs
       produces an empty constraint set (negative case).
       Self-collision is parked (PRD §4); the test scene must not
       trigger self-collision.
```

Two scenes, one update each:

- **Positive scene.** Cloth (TriangularCloth) at center
  `(0, -0.99, 0)` — particles sit just above the ground plane at
  `y = -1`, AABBs overlap at t=0. `addGround(XZPlane, (0, -1, 0),
  size1D=2.0)` is the second mesh. After
  `sim.initialize() + sim.update()` (one frame, 8 substeps), assert:
  - `cumulativeNarrowCollisions > 0` (the broad+narrow pipeline
    fired contacts at least once across the substep loop).
  - At least one entry in
    `Scene<Backend, Precision>::packedCollisionData.narrowCollisions[0..numNarrowCollisions[0])`
    has `objPair.query != objPair.target` (the contact is between
    two distinct mesh ids — the (A, B) pair).
  - **Pass label:** `BDD-010 / overlapping AABBs produce a contact
    pair between two distinct objects`.

- **Negative scene.** Cloth at center `(0, 10.0, 0)` — particles
  10m above ground; AABBs do not overlap. `addGround` same as
  above. After one `sim.update()`, assert
  `cumulativeNarrowCollisions == 0`.
  - **Pass label:** `BDD-010 / non-overlapping AABBs produce empty
    constraint set`.

- **Self-collision avoidance.** `Simulator::enableSelfCollisions
  defaults to false`, so the broad phase already filters
  same-object pairs out of the `narrowCollisions` output. The
  BDD's Notes line "test scene must not trigger self-collision"
  is satisfied by the default flag + the assertion only counting
  `query != target` pairs. Don't touch `enableSelfCollisions` in
  Block 13.

- **Reset between positive and negative.** `resetScene()` +
  `cumulativeNarrowCollisions = 0` between the two; same pattern
  Block 6 (BDD-007) uses for fresh scene state.

### 2. BDD-004 fold-in (Estimator turn-13 WARNING)

In Block 12 (`src/main.cpp::runSelfTest` ~line 6280 area), add a
unit-norm assertion on `r1_post_load` **before** the re-multiply:

```cpp
::Quat r1_post_load = meshAfterLoad->rotationQuat;
float n1_post_load = quatNorm(r1_post_load);
if (std::abs(n1_post_load - 1.0f) >= normTol) {
    fail("BDD-004 / quaternion composition round-trip",
         "r1_post_load is not unit-norm; |r1_post_load| = " +
         std::to_string(n1_post_load));
} else {
    // ... existing composition + comparison ...
}
```

Wrap the existing composition logic inside the `else` branch (or
short-circuit on fail). Pass label wording stays unchanged.

### 3. Bookkeeping

- `docs/TEST_MATRIX.md` row `BDD-010` — promote `pending → pass`.
  Test address: `src/main.cpp::runSelfTest::BDD-010 (Block 13) —
  positive (cloth-on-ground touching) + negative (cloth far above
  ground) clauses.`
- No new `D-NNN` entry — Block 13 is mechanization-only; no new
  architectural decision.
- `.agent/CURRENT_WORK.md` / `RESUME.md` — update for the slice.

## Non-goals (this slice)

- **Cloth-on-cloth (self) collision.** Parked at PRD §4. The test
  scene uses `enableSelfCollisions = false` (the default).

- **Multi-frame collision dynamics.** BDD-010 says "the broad-
  phase + narrow-phase pipeline runs once". One `sim.update()` is
  the contract; longer-horizon collision behavior is BDD-007's job.

- **Cloth-vs-cube or cloth-vs-sphere collision.** v1's narrow
  phase fires on triangle pairs; cloth-vs-ground (both grids) is
  the canonical pair. Cube/sphere collision targets are out of
  scope until rigid-body lands (FR-008, blocked on Q4).

- **Tolerance loosening on positive case.** `cumulativeNarrowCollisions
  > 0` is the strict assertion; the integrator's response is
  irrelevant — Block 13 only verifies the **detection** pipeline.

- **Estimator turn-13 NOTE on q vs -q sign equivalence.** The
  persistence path doesn't canonicalize signs today. Recorded in
  ESTIMATION.md NOTE; if a future slice introduces sign
  canonicalization, switch the orientation compare to an
  equivalence check.

- **Resolving `Q1`, `Q2`, `Q4`, `Q5`, `Q6`, `Q7`.**

## Todo

Ordered. Generator executes top-to-bottom.

1. **Branch hygiene.** Already on `feat/bdd-010-collision-detected`
   (off `main` at `089279b`). No new branch. Commit prefix: `add:`
   (new test coverage; small fold-in WARNING fix piggybacks).

2. **Re-read `docs/TESTS.md#BDD-010`** (lines 97–103) and the
   Notes line. Block 13 is authored from this verbatim, including
   the pass-label wording.

3. **Author Block 13 in `src/main.cpp::runSelfTest`** after Block
   12 (just before the `if (failures == 0) { ... }` summary).
   Concrete shape:

   ```cpp
   // ---- Block 13: BDD-010 — Collision detected between simulated objects.
   // TESTS.md#BDD-010 wording (verbatim, *not* the matrix-row label):
   //   Given two simulated meshes positioned so that their AABBs overlap
   //         on the next step
   //   When  the broad-phase + narrow-phase pipeline runs once
   //   Then  the resulting constraint set contains at least one contact
   //         pair (A, B) between the two objects.
   //   Notes: also assert that the same scene with non-overlapping AABBs
   //          produces an empty constraint set (negative case). Self-
   //          collision is parked (PRD §4); the test scene must not
   //          trigger self-collision.
   //
   // The test exercises the broad + narrow pipeline directly: one
   // sim.update() per scene, then read packedCollisionData. Self-
   // collision is filtered by the default enableSelfCollisions = false;
   // the (A, B) assertion only counts contacts with
   // objPair.query != objPair.target.
   {
       // --- Positive: cloth touching ground, AABBs overlap. ---
       resetScene();
       sim.addCloth(/*particleNum1D=*/4, /*size1D=*/0.5,
                    tinym::vec3(0.0f, -0.99f, 0.0f),
                    /*kstretch=*/1e3, /*kshear=*/1e3, /*kbend=*/1e3,
                    /*thickness=*/0.01, /*mass=*/0.1);
       sim.addGround(PlaneDirection::XZPlane, tinym::vec3(0, -1, 0),
                     /*size1D=*/2.0f);
       sim.initialize();
       Scene<Backend, Precision>::packedCollisionData.cumulativeNarrowCollisions = 0;

       sim.update();
       size_t cumPos =
           Scene<Backend, Precision>::packedCollisionData.cumulativeNarrowCollisions;
       auto& packedColPos =
           Scene<Backend, Precision>::packedCollisionData;

       bool foundDistinctPair = false;
       size_t lastSubstep = packedColPos.numNarrowCollisions[0];
       for (size_t i = 0; i < lastSubstep; ++i) {
           auto& nc = packedColPos.narrowCollisions[i];
           if (nc.objPair.query != nc.objPair.target) {
               foundDistinctPair = true;
               break;
           }
       }
       if (cumPos == 0) {
           fail("BDD-010 / overlapping AABBs produce a contact pair between two distinct objects",
                "cumulativeNarrowCollisions == 0 — pipeline didn't fire");
       } else if (!foundDistinctPair) {
           fail("BDD-010 / overlapping AABBs produce a contact pair between two distinct objects",
                "no narrow contact has objPair.query != objPair.target "
                "(cumNarrow=" + std::to_string(cumPos) + ", lastSubstep=" +
                std::to_string(lastSubstep) + ")");
       } else {
           pass("BDD-010 / overlapping AABBs produce a contact pair between two distinct objects");
       }

       // --- Negative: cloth far above ground, AABBs disjoint. ---
       resetScene();
       sim.addCloth(/*particleNum1D=*/4, /*size1D=*/0.5,
                    tinym::vec3(0.0f, 10.0f, 0.0f),
                    /*kstretch=*/1e3, /*kshear=*/1e3, /*kbend=*/1e3,
                    /*thickness=*/0.01, /*mass=*/0.1);
       sim.addGround(PlaneDirection::XZPlane, tinym::vec3(0, -1, 0),
                     /*size1D=*/2.0f);
       sim.initialize();
       Scene<Backend, Precision>::packedCollisionData.cumulativeNarrowCollisions = 0;

       sim.update();
       size_t cumNeg =
           Scene<Backend, Precision>::packedCollisionData.cumulativeNarrowCollisions;
       if (cumNeg != 0) {
           fail("BDD-010 / non-overlapping AABBs produce empty constraint set",
                "cumulativeNarrowCollisions == " + std::to_string(cumNeg) +
                " on a scene with disjoint AABBs");
       } else {
           pass("BDD-010 / non-overlapping AABBs produce empty constraint set");
       }
   }
   ```

4. **Fold-in: BDD-004 unit-norm assertion on `r1_post_load`**
   (Estimator turn-13 WARNING). In Block 12, immediately after
   `::Quat r1_post_load = meshAfterLoad->rotationQuat;`, add:

   ```cpp
   float n1_post_load = quatNorm(r1_post_load);
   if (std::abs(n1_post_load - 1.0f) >= normTol) {
       fail("BDD-004 / quaternion composition round-trip",
            "r1_post_load is not unit-norm; |r1_post_load| = " +
            std::to_string(n1_post_load));
   } else {
       // existing composition + comparison ...
   }
   ```

   Wrap the existing comparison logic in the `else` branch (or use
   an early-return-style guard — Generator's call). Pass label
   stays unchanged so the matrix-row test address keeps matching.

5. **Run `./scripts/verify-light.sh`.** Doctest binaries should
   stay 159/159 + 1120/1120.

6. **Run `--self-test` 5+ times.** Expect **27/27 PASS**
   consistently. The positive case relies on cloth-on-ground
   contact firing in the first frame; if it FAILs intermittently,
   the cloth's initial position may need to drop slightly closer
   (e.g. y=-0.995 instead of -0.99). The negative case is robust
   — if it ever FAILs, that's a real broad-phase bug worth
   surfacing.

7. **Optional bug-probe.** For confidence: temporarily move the
   negative scene's cloth to y=-0.99 (matches positive); confirm
   the negative assertion FAILs with `cumulativeNarrowCollisions ==
   N > 0`. Restore. Same discipline as prior slices.

8. **Promote `BDD-010` matrix row.** `docs/TEST_MATRIX.md:24`:
   - Status: `pending → pass`.
   - Test address: `src/main.cpp::runSelfTest::BDD-010 (Block 13)
     — positive (cloth-on-ground at touching distance, asserts at
     least one (A, B) narrow contact between distinct objects) +
     negative (cloth 10m above ground, asserts empty constraint
     set) clauses. Self-collision filtered by default
     enableSelfCollisions = false.`

9. **Update CURRENT_WORK / RESUME.** Four-line max as work
   proceeds; write RESUME near end of turn. Note explicitly that
   the Estimator turn-13 WARNING is closed.

10. **Stop and hand off to the Estimator.** No new BDDs, no
    `Simulator::rotateObject`, no kernel changes, no other matrix
    rows, no spec edits.

## Course corrections

- **Spec-vs-label discipline.** Block 13's pass labels are
  authored from the BDD's "Then" clause + Notes line verbatim. The
  matrix-row label "Collision detected between simulated objects"
  is too compressed to drive the assertion off.

- **Cumulative vs last-substep.** `numNarrowCollisions[0]` resets
  per substep; `narrowCollisions[i]` records also reset (via
  `resetNarrow()`). `cumulativeNarrowCollisions` accumulates
  across substeps. Block 13 uses `cumulativeNarrowCollisions` as
  the "ever fired" signal and the last-substep `narrowCollisions`
  records (still readable post-`update()`) for the (A, B)
  distinct-mesh assertion. For positive cloth-on-ground, contact
  persists through the last substep, so this works.

- **Don't run more than one update.** BDD-010 says "pipeline runs
  once". A multi-frame pump would conflate detection correctness
  with simulation evolution. Single update is the contract.

- **`enableSelfCollisions = false` is the default.** Block 13
  must not flip it on. The Notes line about self-collision is
  satisfied by the default flag + the `query != target` filter
  in the assertion.

- **Cumulative counter must be reset between the two scenes.**
  `resetScene()` doesn't clear it; the harness explicitly sets
  `packedCollisionData.cumulativeNarrowCollisions = 0` before
  each `sim.update()`. Block 6 (BDD-007) uses the same pattern.

- **Existing tolerance constants**: `quatTol = 1e-5f` and
  `normTol = 1e-5f` already live in Block 12. The fold-in reuses
  `normTol` — no new tolerance value needed.

## What to read before writing code

- `docs/TESTS.md#BDD-010` (lines 97–103) — binding "Then" clause
  + Notes line. Verbatim source for Block 13's pass labels.
- `src/main.cpp::runSelfTest` Block 6 (BDD-007, ~line 5530) —
  template for resetting `cumulativeNarrowCollisions` and
  asserting against it.
- `src/main.cpp::runSelfTest` Block 12 (~line 6213) — the existing
  BDD-004 block where the fold-in goes.
- `src/main.cpp::struct NarrowCollision` (~line 1352) — confirms
  `objPair.query` and `objPair.target` are accessible as expected.
- `src/main.cpp::Scene::PackedCollisionData` (~line 1679) — has
  `narrowCollisions`, `numNarrowCollisions`, and
  `cumulativeNarrowCollisions` members.
- `src/main.cpp::Simulator::enableSelfCollisions` (~line 4290) —
  default false; Block 13 leaves it alone.
- `.agent/ESTIMATION.md` — current turn-13 WARNING text being
  folded in todo 4.
