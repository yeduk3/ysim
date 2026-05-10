# Resume — Rotate pack-roundtrip Slice (D-025 lands; 5-slice deferral chain ended)

## Must remember

- **Branch:** `feat/rotate-pack-roundtrip` (off `main` at `d54cb44`).
- **D-025 is Shape B — the side-table re-apply path, not a Shape A initializer rewrite.** Three production-side touchpoints in `src/main.cpp`:
  1. `Simulator::rotateObject` — after the existing state.x mutation + `broadPhase.refit()`, append `pendingRotations[meshId] = newAbs;`.
  2. `Simulator::initialize` — appends `applyPendingMaterials();` at the very end.
  3. `Simulator::applyPendingMaterials` — rotation branch now snapshots `pendingRotations` into a local `std::vector<std::pair<int,::Quat>>` *before* the loop, then calls `rotateObject(id, savedQuat)` for each entry. Snapshot is load-bearing because rotateObject re-populates the map during its call (would mutate the container under the iterator otherwise).
- **`applyPendingMaterials` is now auto-called by `initialize()`.** This is the contract change. Existing explicit callers (after `loadScene` in `main.cpp` and `runSelfTest`) are still correct — they become no-ops because the maps are cleared by `initialize()` already.
- **Future state.x write paths** must take one of two routes: (a) D-015 translate pattern — write back to the initializer's persistent param so re-pack reproduces it; (b) D-025 rotate pattern — write to a Simulator-side side-table, auto-applied via `applyPendingMaterials` in `initialize`. Both share the `broadPhase.refit()` call (D-023) so click-pick reflects the new pose immediately.
- **Block 18 mechanizes the round-trip.** 90deg-Z rotate cube at origin → addCube (forces re-pack) → assert state.x[0] still reflects the rotation within 1e-5. Pass label: `FR-004 / rotateObject survives Scene::pack rebuild`.
- **Bug-probe verified.** Skipping the `pendingRotations[meshId] = newAbs` write makes Block 18 FAIL with `expected (0.25, 0.25, -0.25) got (0.25, -0.25, -0.25)` (un-rotated witness). Restored.
- **Estimator turn-18 WARNING fold-in.** `docs/TEST_MATRIX.md` BDD-017 row's test address gained a Block 17 cross-reference for the triangle-precision sister mechanization.

## Last decisions + why

- **D-025 — Shape B (side-table + auto-applyPendingMaterials) over Shape A (initializer carries rotation).** Both work; Shape B is 5 lines vs Shape A's 4-class refactor. Rejected the "make applyPendingMaterials only set rotationQuat" variant because that conceptually duplicates rotateObject's pivot-aware logic. Calling rotateObject directly from applyPendingMaterials reuses the existing pivot-aware rotation + xPrev mirror + refit triggering, all in one path. Auto-calling applyPendingMaterials from initialize is the seam that makes loadScene and rotateObject share one canonical post-pack restoration code path.

## Next step you were about to take

Slice complete. Next concrete step is the **Estimator's** turn — `./scripts/verify.sh` should exit 0 with **35/35** self-test PASS lines (Block 18 added; previous count was 34). Expected verdict: NOTE level.

After this lands, planner-tracked candidates per `PROJECT_STATE.md` (rotate pack-roundtrip is **off this list now**):

- **CM-008 production-side fix** — `BroadPhase::build`'s Float-mesh skip robust against scene-swap-at-same-count cases. Theoretical concern in v1.
- **Inspector ergonomics for rotation** — Euler / axis-angle input affordances per FR-004 Notes.
- **BDD-018 inspector live-edit propagation** — implementation exists; mechanization needs ImGui-side simulation or callable abstraction.
- **Material editing UI (FR-005 / BDD-005)** — needs PBR preview shader. Large.
- **Behavior assignment UI (FR-006 / BDD-006)** — Q2 still open.
- **Rigid body slice (FR-008 / BDD-008)** — blocked on Q4. When this lands: D-014/D-015 + D-018 + D-019/D-022 + D-020 + D-021 + D-023 + D-024 + D-025 all apply.
- **Alembic export slice (FR-013 / BDD-013)** — blocked on Q5 + Q6.

See `.agent/PLAN.md` and `.agent/CURRENT_WORK.md` for full plan and progress.
