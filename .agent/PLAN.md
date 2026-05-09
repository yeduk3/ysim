# Plan — BDD-017 Coverage Fix-turn (`fix/bdd-017-coverage`)

> Owner: **Planner** writes; **Generator** executes; **Estimator** judges.
> Updated: 2026-05-10

## Course note: previous slice's verdict

Estimator turn 15 returned **WARNING** (no BLOCK) on the BDD-017
slice — two coverage gaps, both small, both directly related to
Block 14:

- **Gap 1 (`src/main.cpp:6532-6536`):** The overlapping clause
  asserts `numHits >= 2` after `queryClickRay`, but
  `queryClickRay` writes per-triangle hits — a single cube can
  alone generate ≥2 hits (entry + exit triangle, or any pair of
  intersected triangles), satisfying the count without proving
  the back cube ever participated. An overlapping-object
  regression where the BVH stops finding the back cube would
  pass this assertion silently.

- **Gap 2 (`src/main.cpp:6434-6440`):** Block 14 reads
  `clickRayCollisions[]` and computes the smallest `tmin` via the
  local `pickClosest` lambda. Production at line 6718 (current
  numbering — was 6596 in the previous slice) writes that result
  into `simulator->selectedObj`. The harness never assigns
  `selectedObj`, so a callback-side regression that breaks the
  assignment but leaves the BVH intact would be invisible to
  Block 14.

Per PLANNER.md procedure step 2, both items are ~3–5 line fixes
and the slice's primary work IS this fold-in — it's a fix-turn-
style slice on a fresh branch (the parent slice was already merged
on `main`).

## Goal

Close Estimator turn-15's two WARNINGs by extending Block 14 in
`src/main.cpp::runSelfTest`. Pass-label wording stays unchanged so
`docs/TEST_MATRIX.md` row `BDD-017` keeps its existing test
address. After this slice:

- 29/29 self-test PASS unchanged (no new pass labels — just
  tighter assertions inside existing labels).
- Both turn-15 WARNINGs closed.
- BDD-017 coverage is now structurally complete: BVH query +
  smallest-tmin walk + `selectedObj` assignment.

## Scope

### 1. Overlapping case: assert both cubes participated

In Block 14's clause (b), after `queryClickRay(rayDeep)` populates
`clickRayCollisions[]`, walk the array once and verify that BOTH
`cubeFrontId` AND `cubeBackId` appear in some `nc.obj`. The
existing `numHits >= 2` gate is keep-able as a coarse check, but
the load-bearing assertion becomes the both-cubes-present check.

```cpp
bool sawFront = false;
bool sawBack  = false;
for (Index i = 0; i < numHits; ++i) {
    Index hitObj = Scene<Backend, Precision>::rayTracedData
        .clickRayCollisions[i].obj;
    if (hitObj == cubeFrontId) sawFront = true;
    if (hitObj == cubeBackId)  sawBack  = true;
}
```

The existing front-most-wins clause (`pickedDeep == cubeFrontId`)
stays exactly as is — the new check augments it. The clause now
fails-with-diagnostic when:
- `numHits < 2` (existing — coarse), OR
- `!sawFront || !sawBack` (new — proves both participated), OR
- `pickedDeep != cubeFrontId` (existing — front-most-wins).

### 2. selectedObj assignment path

In Block 14's both clauses, after computing the closest id via
`pickClosest()`, write it into `sim.selectedObj` and assert from
there — mirroring what production does at line ~6718:

```cpp
// Production callback writes selectedObj on the same closest-tmin
// walk; mirror that to exercise the assignment path. Asserting
// against sim.selectedObj catches a callback-side regression that
// drops the assignment but leaves the BVH intact.
sim.selectedObj = static_cast<int>(pickedX);
// ... assertion uses sim.selectedObj ...
```

For both clauses, replace the `pickedX != expectedId` check with
`sim.selectedObj != static_cast<int>(expectedId)`. This is ~1 line
of substitution per assertion. The diagnostic message updates to
read from `sim.selectedObj` for grep-ability.

`Simulator::selectedObj` is `int`; the cast from `Index`
(`uint32_t`) is fine for small ids. The "no hit" case where
`pickClosest` returns `kNoHit` (UINT32_MAX) cast to `int` is
implementation-defined but produces some negative number — that
won't match any positive expected id, so the existing fail path
still triggers. The cast still goes through `int` because
production writes `simulator->selectedObj = closestObj` where
closestObj is `Index` and selectedObj is `int` (line ~6720 in
`mouseButtonCallback`).

### 3. Bookkeeping

- **`docs/TEST_MATRIX.md`:** unchanged. Pass labels stay; the
  BDD-017 test-address line already mentions the smallest-tmin
  walk; no rewording needed.
- **No new D-NNN.** No architectural decision.
- **No CM update.** CM-008 stays in active list — its production-
  side fix is still deferred. The harness still uses
  `objTrees.clear()` workaround.
- **`.agent/CURRENT_WORK.md` / `RESUME.md`** — update for the
  fix-turn slice.

## Non-goals (this slice)

- **CM-008 production-side fix.** Deferred. The harness workaround
  stays.
- **`Simulator::rotateObject` UI side (FR-004).** Independent
  larger slice.
- **BDD-018 inspector live-edit propagation.** Separate concern.
- **Any other matrix row.**
- **Spec edits.**
- **Resolving any of `Q1`, `Q2`, `Q4`, `Q5`, `Q6`, `Q7`.**

## Todo

Ordered. Generator executes top-to-bottom.

1. **Branch hygiene.** Already on `fix/bdd-017-coverage` (off
   `main` at `3e5f6a4`). No new branch. Commit prefix: `fix:`
   (closing prior-slice WARNINGs).

2. **Re-read** `docs/TESTS.md#BDD-017` (lines 161–167) and the
   Notes line. Both fold-ins must continue to satisfy the BDD's
   "Then" wording: pass labels are unchanged; the additional
   assertions narrow the false-pass surface within the same
   contract.

3. **Extend clause (b) with the both-cubes-present walk.** In
   Block 14 (`src/main.cpp` ~line 6532), restructure the gate
   chain to:

   ```cpp
   bool sawFront = false, sawBack = false;
   for (Index i = 0; i < numHits; ++i) {
       Index hitObj = Scene<Backend, Precision>::rayTracedData
           .clickRayCollisions[i].obj;
       if (hitObj == cubeFrontId) sawFront = true;
       if (hitObj == cubeBackId)  sawBack  = true;
   }

   sim.selectedObj = static_cast<int>(pickedDeep);

   if (numHits < 2) {
       fail("BDD-017 / overlapping case: front-most object (smallest ray t) wins",
            "expected >=2 hits along through-line, got " +
            std::to_string(numHits));
   } else if (!sawFront || !sawBack) {
       fail("BDD-017 / overlapping case: front-most object (smallest ray t) wins",
            "ray hits did not include both cubes (sawFront=" +
            std::to_string(sawFront) + ", sawBack=" +
            std::to_string(sawBack) + ")");
   } else if (sim.selectedObj != static_cast<int>(cubeFrontId)) {
       fail("BDD-017 / overlapping case: front-most object (smallest ray t) wins",
            "expected sim.selectedObj=" + std::to_string(cubeFrontId) +
            " (cubeFront), got " + std::to_string(sim.selectedObj));
   } else {
       pass("BDD-017 / overlapping case: front-most object (smallest ray t) wins");
   }
   ```

4. **Extend clause (a) with the selectedObj assignment.** Apply
   the same `sim.selectedObj = static_cast<int>(pickedX);` pattern
   for both ray A and ray B sub-checks; assertions read from
   `sim.selectedObj` instead of the raw `pickedA` / `pickedB`
   locals. Diagnostic messages update to `expected
   sim.selectedObj=…`.

5. **Restore `sim.selectedObj` between clauses if needed.** The
   harness's other blocks (Block 13 etc.) don't touch
   `selectedObj`, so a residual non-(-1) value at the end of Block
   14 is harmless. Optional: explicitly reset to `-1` at the start
   of Block 14 for cleanliness.

6. **Run `./scripts/verify-light.sh`.** Doctest binaries should
   stay 159/159 + 1120/1120.

7. **Run `--self-test` 5+ times.** Expect **29/29 PASS**
   consistently. Same number — only the assertion shapes inside
   existing pass labels change.

8. **Optional bug-probe.** For confidence: temporarily skip the
   `sim.selectedObj = …` assignment in clause (b); confirm the
   selectedObj assertion FAILs (sim.selectedObj likely stays -1
   from the harness default or whatever Block 14's start leaves
   it at — see todo 5). Restore. Same discipline as prior fix-
   turns.

9. **Update CURRENT_WORK / RESUME.** Four-line max as work
   proceeds. Note explicitly that Estimator turn-15's two
   WARNINGs are closed.

10. **Stop and hand off to the Estimator.** No other work, no
    new BDDs, no spec edits.

## Course corrections

- **Pass labels are unchanged.** The BDD-017 matrix row's
  test-address line ("…non-overlapping (two cubes at distinct x)
  + overlapping (two cubes on same line of sight; smallest-tmin
  wins via the same walk production uses at
  `src/main.cpp:6588-6596`)") still describes the slice
  accurately. The matrix row needs no rewording.

- **`numHits >= 2` stays as a coarse pre-condition gate.** Don't
  drop it — it's cheap and emits a useful diagnostic on the
  pathological "BVH found nothing" failure mode. The both-cubes-
  present check is the load-bearing assertion that closes the
  WARNING.

- **`Index` is `uint32_t`; `selectedObj` is `int`.** The cast
  `static_cast<int>(pickedDeep)` reproduces production's
  implicit conversion at `simulator->selectedObj = closestObj;`
  exactly. For pickedDeep = UINT32_MAX (kNoHit sentinel), the
  cast produces -1, which happens to match the "nothing
  selected" sentinel in `Simulator::selectedObj = -1` — that's
  semantically harmless for these tests since we never expect
  no-hit on a hit-expected case.

- **Both-cubes check uses raw `clickRayCollisions[i].obj`, not
  `pickClosest`.** The existing `pickClosest` lambda only returns
  the single closest id; we need the full hit list to verify
  both cubes are present. A simple linear scan over `0..numHits`
  is sufficient.

## What to read before writing code

- `docs/TESTS.md#BDD-017` (lines 161–167) — binding "Then"
  clause + Notes line. Both pass-label texts stay verbatim.
- `src/main.cpp::runSelfTest` Block 14 (current numbering ~line
  6427–6544) — the block to extend.
- `src/main.cpp` ~line 6718 (mouseButtonCallback) —
  production's `simulator->selectedObj = closestObj;` write.
  Block 14 mirrors this pattern.
- `src/main.cpp::Simulator` — confirm `selectedObj` is `int`
  (~line 4350: `int selectedObj = -1;`).
- `.agent/ESTIMATION.md` — current turn-15 WARNING text being
  closed.
