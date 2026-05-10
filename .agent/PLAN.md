# Plan — Rotate pack-roundtrip (`feat/rotate-pack-roundtrip`)

> Owner: **Planner** writes; **Generator** executes; **Estimator** judges.
> Updated: 2026-05-10

## Course note: previous slice's verdict

Estimator turn 18 returned **WARNING** (no BLOCK) on the
click-triangle-precision slice. Single item: `docs/TEST_MATRIX.md`'s
BDD-017 row + `.agent/PLAN.md` don't cross-reference Block 17.
Tiny (~1 line) — folded into this slice as the bookkeeping todo.

## Escape-pattern note

The rotate pack-roundtrip gap has been **deferred 5 slices in a
row** in RESUME's "next candidates" carry-forward (translate-pack /
profiler-gate / cloth-thickness / BDD-102 / BDD-004 / BDD-010 /
BDD-017 / refit-after-edit / click-triangle-precision). Per
PLANNER.md procedure step 6 (escape-pattern detection), this
slice closes the gap before the deferral itself becomes the
problem.

## Design call (the question that's been blocking this)

D-014 / D-015 closed translate's pack-roundtrip via an **initializer
write-back**: `translateObject` writes the new center into
`mesh->initializer->params.center`, and pack-time reads it back
when constructing fresh state.x. Symmetric with how
`transformPosition` is seeded.

For rotate, the analogous shapes are:

- **Shape A — initializer carries rotation.** Add `Quat rotation`
  to each initializer's params; each initializer's `initialize()`
  applies the rotation to state.x as a post-step around
  `params.center`; `rotateObject` writes back to `params.rotation`
  via the same dynamic_cast cascade. Pack-time also seeds
  `mesh.rotationQuat = params.rotation`. Mirrors D-015 exactly.
  **Cost:** modify 4 initializer classes + dynamic_cast cascade
  in rotateObject. Larger surface area than D-015 (which only
  touched the cascade — initializer.center was already a
  build-time input).

- **Shape B — re-apply via pendingRotations + auto applyPendingMaterials.**
  `rotateObject` writes `pendingRotations[id] = newAbs` (in
  addition to its existing state.x mutation). `Simulator::initialize()`
  auto-calls `applyPendingMaterials()` at the end. The
  `applyPendingMaterials` body is updated: instead of just setting
  `mesh.rotationQuat = pendingRotations[id]`, it calls
  `rotateObject(mesh.id, pendingRotations[id])`. The rotateObject
  call rotates fresh state.x by the saved quat around the
  (already-correctly-seeded) `transformPosition` pivot, sets
  `mesh.rotationQuat`, and triggers `broadPhase.refit()`.
  **Cost:** ~5 lines: one auto-call in `Simulator::initialize`,
  a `rotateObject` swap inside `applyPendingMaterials`, an
  additional `pendingRotations[id] = newAbs;` write inside
  `rotateObject`. **No initializer changes.**

**Decision: Shape B.** Reasons:

1. **Smaller surface area.** Shape A modifies 4 initializer
   classes' `initialize()` plus the cascade; Shape B is a 5-line
   contract change to `Simulator::initialize` + `applyPendingMaterials`
   + `rotateObject`.
2. **Re-uses the existing `pendingRotations` mechanism.** The
   side-table was already designed for `loadScene`'s flow — making
   it serve `rotateObject` too is consistent.
3. **`applyPendingMaterials` becomes the canonical "post-pack
   restoration" path** for both load-time AND edit-time
   rotations. Its name slightly under-sells what it does, but
   the contract is unified.
4. **`Simulator::initialize` auto-calling `applyPendingMaterials`**
   is a contract change — but the existing explicit call sites
   (after `loadScene` in main.cpp and runSelfTest) become
   no-ops on the second call (the map is cleared after the
   auto-call). Backwards-compatible.

D-025 records the choice.

## Goal

After this slice:

- `Simulator::rotateObject(id, q)` writes both `state.x` (current
  behavior) AND `pendingRotations[id] = q` (new).
- `Simulator::initialize()` auto-calls `applyPendingMaterials()` at
  the very end (after `broadPhase.build` and friends).
- `applyPendingMaterials()`'s rotation branch calls
  `rotateObject(m.id, rit->second)` instead of just setting
  `mesh.rotationQuat`. This rotates fresh state.x by the saved
  quat AND updates rotationQuat AND triggers BVH refit.
- A `rotateObject` then `addCube + sim.initialize()` sequence
  preserves the rotation on the originally-rotated cube. Click-
  pick remains correct on the rotated cube.
- Block 18 in `runSelfTest` mechanizes the round-trip explicitly.
  Pass label mirrors BDD-003 clause (d) shape:
  `FR-004 / rotateObject survives Scene::pack rebuild`.
- Estimator turn-18 WARNING closed via a 1-line cross-reference
  in `docs/TEST_MATRIX.md` BDD-017 row's test address.

## Scope

### 1. Production fix — D-025 — Shape B

**`src/main.cpp::Simulator::rotateObject`** (~line 4545): after
`mesh->rotationQuat = newAbs;` and `broadPhase.refit();`, add:

```cpp
pendingRotations[meshId] = newAbs;
```

**`src/main.cpp::Simulator::initialize`** (~line 4582 area, end of
function): add `applyPendingMaterials();` as the final line.
Document that auto-calling is the new contract.

**`src/main.cpp::Simulator::applyPendingMaterials`** (~line 5242):
update the rotation branch from:

```cpp
auto rit = pendingRotations.find(m.id);
if (rit != pendingRotations.end()) m.rotationQuat = rit->second;
```

to:

```cpp
auto rit = pendingRotations.find(m.id);
if (rit != pendingRotations.end()) {
    // D-025: re-apply the rotation to fresh state.x via rotateObject,
    // not just by setting rotationQuat. After Scene::pack rebuilds
    // state.x from the initializer's geometry, the rotation effect
    // is gone; rotateObject's pivot-aware rotation writes it back.
    rotateObject(static_cast<int>(m.id), rit->second);
}
```

The `rotateObject` call inside `applyPendingMaterials` will
itself try to write `pendingRotations[id] = newAbs` (via D-025's
own write-back). That's fine — the map gets re-populated with
the same value, then cleared at the end of the loop. Net effect:
idempotent.

**Side note: `m.rotationQuat` after pack is identity** (default).
`rotateObject` computes `delta = newAbs * conjugate(identity) =
newAbs`. Applies delta to fresh state.x → state.x rotated.
Sets `m.rotationQuat = newAbs`. ✓

### 2. Block 18 — `runSelfTest` mechanization

Append after Block 17 (the click-triangle-precision block).

```cpp
// ---- Block 18: D-025 — rotateObject survives Scene::pack rebuild. ----
// Mirrors BDD-003's clause (d) shape (translate-survives-re-pack)
// for rotation. Re-pack rebuilds state.x from the initializer's
// geometry, which loses rotateObject's effect on state.x. D-025
// closes the gap by writing pendingRotations[id] inside rotateObject
// AND auto-calling applyPendingMaterials() from Simulator::initialize().
{
    sim.collisionPipeline.broadPhase.objTrees.clear();
    resetScene();
    sim.addCube(tinym::vec3(0.0f, 0.0f, 0.0f), /*tess=*/2,
                /*size=*/0.5f, /*mass=*/0.1f);
    sim.initialize();
    const int rotateRoundTripId = 0;

    auto* m0 = Scene<Backend, Precision>::findById(rotateRoundTripId);
    if (!m0) {
        fail("FR-004 / rotateObject survives Scene::pack rebuild",
             "cube id=" + std::to_string(rotateRoundTripId) + " not found pre-rotate");
    } else {
        // 90deg-Z rotation; witness vertex (0.25, -0.25, -0.25) → (0.25, 0.25, -0.25).
        constexpr float kPi18 = 3.14159265358979323846f;
        float halfA = (kPi18 / 2.0f) * 0.5f;
        ::Quat newAbs = ::Quat{std::cos(halfA), 0.0f, 0.0f, std::sin(halfA)};
        sim.rotateObject(rotateRoundTripId, newAbs);

        double rx = m0->state.x.ptr[0];
        double ry = m0->state.x.ptr[1];
        double rz = m0->state.x.ptr[2];

        // Force a re-pack: add another mesh and re-initialize.
        sim.addCube(tinym::vec3(5.0f, 0.0f, 0.0f), /*tess=*/2,
                    /*size=*/0.5f, /*mass=*/0.1f);
        sim.initialize();  // pack + applyPendingMaterials (auto via D-025)

        auto* m0_after = Scene<Backend, Precision>::findById(rotateRoundTripId);
        if (!m0_after) {
            fail("FR-004 / rotateObject survives Scene::pack rebuild",
                 "cube id=" + std::to_string(rotateRoundTripId) + " disappeared after re-pack");
        } else {
            const double posTol = 1e-5;
            double rrx = m0_after->state.x.ptr[0];
            double rry = m0_after->state.x.ptr[1];
            double rrz = m0_after->state.x.ptr[2];
            if (std::abs(rrx - rx) > posTol ||
                std::abs(rry - ry) > posTol ||
                std::abs(rrz - rz) > posTol) {
                fail("FR-004 / rotateObject survives Scene::pack rebuild",
                     "post-repack state.x[0] drifted from rotated value: "
                     "expected (" + std::to_string(rx) + ", " +
                     std::to_string(ry) + ", " + std::to_string(rz) + ") "
                     "got (" + std::to_string(rrx) + ", " +
                     std::to_string(rry) + ", " + std::to_string(rrz) + ")");
            } else {
                pass("FR-004 / rotateObject survives Scene::pack rebuild");
            }
        }
    }
}
```

### 3. Bookkeeping fold-in (Estimator turn-18 WARNING)

`docs/TEST_MATRIX.md` BDD-017 row's test address line — append
"+ Block 17 (D-024 triangle-precision sister mechanization)" after
the existing Block 14 reference. ~1 line edit.

### 4. Bookkeeping (slice's own)

- `docs/DECISIONS.md` — D-025: the rotateObject pack-roundtrip
  decision. File / function / decision (Shape B over Shape A) /
  alternatives-considered / rationale per the standard format.
- `.agent/CURRENT_WORK.md` / `RESUME.md` — update for the
  slice. RESUME finally drops the "rotate pack-roundtrip" item
  from the carry-forward list. Promote next candidates: CM-008
  production-side fix (theoretical), inspector ergonomics for
  rotation, BDD-018, BDD-005, BDD-006, BDD-008/013 (Q-blocked).

## Non-goals (this slice)

- **Initializer refactor** (Shape A). Shape B is sufficient and
  smaller.
- **`applyPendingMaterials` rename or signature change.** The
  function's name slightly under-sells what it now does, but
  the contract is unified across load-time + edit-time. Renaming
  would touch every call site for cosmetic gain. Keep.
- **Performance optimization** of rotation re-apply for many
  meshes. v1's small N keeps this trivial.
- **CM-008 production-side fix** — still deferred (theoretical
  concern in v1).
- **Other matrix rows, spec edits.**
- **Resolving `Q1`, `Q2`, `Q4`, `Q5`, `Q6`, `Q7`.**

## Todo

Ordered. Generator executes top-to-bottom.

1. **Branch hygiene.** Already on `feat/rotate-pack-roundtrip`
   (off `main` at `d54cb44`). Commit prefix: `add:` (this slice
   adds the contract change + new pendingRotations write +
   Block 18). Or `fix:` is also defensible — Generator picks.

2. **Re-read the design call** above. The Shape A vs Shape B
   trade-off is documented; this slice ships Shape B. Don't
   second-guess unless implementation reveals a blocking
   subtlety.

3. **Update `Simulator::rotateObject`** to write
   `pendingRotations[meshId] = newAbs` after the existing
   state.x mutation + refit.

4. **Update `Simulator::applyPendingMaterials`** to call
   `rotateObject(m.id, rit->second)` instead of just setting
   `m.rotationQuat`. Comment cites D-025.

5. **Update `Simulator::initialize`** to call
   `applyPendingMaterials()` at the end. Document the new
   contract: any caller that previously relied on the explicit
   post-`loadScene` call sees no change (the map is empty after
   the auto-call). Comment cites D-025.

6. **Author Block 18** per §2 above. Append after Block 17.
   Pass label: `FR-004 / rotateObject survives Scene::pack rebuild`.

7. **`docs/TEST_MATRIX.md` BDD-017 row** — append the Block 17
   cross-reference per §3.

8. **Run `./scripts/verify-light.sh`.** Doctest binaries should
   stay 159/159 + 1120/1120.

9. **Run `--self-test` 5+ times.** Expect **35/35 PASS**
   consistently.

10. **Bug-probe.** Temporarily skip the
    `pendingRotations[meshId] = newAbs;` write inside
    `rotateObject` (or skip the auto-`applyPendingMaterials`
    call inside `Simulator::initialize`); confirm Block 18
    FAILs with the diagnostic showing post-repack state.x has
    drifted from the rotated value. Restore. Same discipline
    as recent slices.

11. **Add D-025 to `docs/DECISIONS.md`.** Standard format.

12. **Update CURRENT_WORK / RESUME.** Drop "rotate pack-roundtrip"
    from RESUME's carry-forward list. Note the `applyPendingMaterials`
    auto-call contract change as a load-bearing fact for future
    callers.

13. **Stop and hand off to the Estimator.** No matrix-row
    promotion, no spec edits, no other features.

## Course corrections

- **Stricter-than-spec assertions** (PLANNER.md step 7). Block
  18's pass label tests state.x exact equality (within FP
  tolerance). A weaker assertion that just checks `mesh.rotationQuat`
  would miss the bug — the rotate effect is on state.x, not
  the field. The strict form is the right call.

- **Architectural invariants applying here:**
  - **D-013** (xPrev parity) — preserved; rotateObject mirrors
    state.x mutation to state.xPrev.
  - **D-014/D-015** (translate semantic + cascade) — unchanged;
    translate's pack-roundtrip already works via initializer
    write-back. No conflict with Shape B for rotate.
  - **D-018** (mesh.id seed) — unchanged.
  - **D-019/D-022** (Quat math) — used by rotateObject;
    unchanged.
  - **D-020** (BVH leaf return) — unchanged.
  - **D-021** (rotateObject API + pivot semantic) — extended.
  - **D-023** (refit after edit) — preserved; rotateObject
    still calls refit().
  - **D-024** (triangle-precision click-pick) — unchanged.
  - **NEW D-025** — `Simulator::initialize()` auto-calls
    `applyPendingMaterials()`; `applyPendingMaterials()` now
    re-applies rotation via `rotateObject` (not just rotationQuat
    set). pendingRotations is the canonical post-pack rotation
    side-table for both load-time (loadScene populates) AND
    edit-time (rotateObject populates).

- **`pendingMaterials` path is unchanged.** Materials are CPU-
  side state that doesn't depend on state.x; just setting
  `m.material = pendingMaterials[id]` is correct. Only the
  rotation branch needs the rotateObject upgrade.

- **`Simulator::initialize` is called by all create flows.**
  addCube, addSphere, addCloth, importMesh — they all call
  `initialize()` after `addGeneralMesh`. So pendingRotations
  gets re-applied automatically on every scene mutation.

## What to read before writing code

- `src/main.cpp::Simulator::rotateObject` (~line 4545) — site
  to add `pendingRotations[id] = newAbs`.
- `src/main.cpp::Simulator::initialize` (~line 4582) — site
  to add the auto-`applyPendingMaterials()` call at the end.
- `src/main.cpp::Simulator::applyPendingMaterials` (~line 5242)
  — site to swap `m.rotationQuat = ...` for `rotateObject(...)`.
- `src/main.cpp::runSelfTest` Block 9 (BDD-003 clause d, "translate
  survives Scene::pack rebuild") — template for Block 18's
  shape.
- `docs/DECISIONS.md::D-014, D-015` — translate pack-roundtrip
  precedent; D-025 is the rotate analog with a different shape
  (side-table re-apply vs initializer write-back).
- `.agent/RESUME.md` — the carry-forward list that's about to
  drop "rotate pack-roundtrip".
