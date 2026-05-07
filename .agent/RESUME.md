# Resume — Cloth-CCD Slice (CM-005 closed)

## Must remember

- **Branch:** `feat/cloth-ccd` (off `main`, after `feat/import-mesh-ui` was fast-forwarded into `main`).
- **D-013 is load-bearing.** `narrow_pt_tri` does swept-segment CCD using `xPrev` (slot 10). The kernel writes a **signed** distance — *do not* re-introduce the `if (l < 0) { n = -n; l = -l; }` flip that the prior version had. The integrator's `(thickness - distance) * n` push relies on the negative sign for tunneled particles; abs'ing it pushes the wrong way.
- **`xPrev` is snapshotted between narrow and integrate** (in `Simulator::update`'s substep loop). NOT before narrow runs — that would make `xPrev == state.x` and degenerate the segment. NOT after integrate — that would make `xPrev` reflect the post-integrate position, which is what `state.x` already holds.
- **`enlargeTrajectory(system.subh)` is still load-bearing** (from cloth-drape slice). Without it, broad phase doesn't even feed pairs into narrow, so CCD never gets a chance.
- **Harness `subSteps = 8`** (was 4). At `subSteps = 4`, the one-substep-lag residual gravity-per-substep penetration is 0.176mm — *just* over the BDD-007 strict tolerance. With 8 substeps the lag shrinks. If a future slice cuts substep count for performance, BDD-007's tunneling clause may regress; bump tolerance OR keep 8.
- **CM-005 is fixed but still listed in `docs/mistakes/COMMON_MISTAKES.md`** as eligible for graduation to `OLD_MISTAKES.md` after one regression-free slice. The next planner that surveys mistakes should move it.

## Last decisions + why

- **D-013** — Snapshot → swept-segment CCD in `narrow_pt_tri`. Closes CM-005, makes BDD-007 PASS.
- **Estimator turn-4 BDD-002 follow-ups folded in this slice** — modal default path correction + AABB/facet-count assertion in Block 7. Both items were tiny; a separate housekeeping slice would have been more process overhead than the work itself.

## Next step you were about to take

Slice complete. Next concrete step is the **Estimator's** turn — `./scripts/verify.sh` exits 0 cleanly (first time since cloth-drape). Expected: NOTE-level verdict.

After this lands, the next planner-tracked candidates per `PROJECT_STATE.md`:

- **CM-005 graduation** — move to `OLD_MISTAKES.md` after one regression-free slice. Trivial cleanup.
- **BDD-102 Determinism mechanization** — extend the harness with two-runs-bit-identical assertion against a saved-scene baseline.
- **Material editing UI (FR-005 / BDD-005)** — needs PBR preview shader.
- **Behavior assignment UI (FR-006 / BDD-006)** — in-place behavior switching reallocates per-mesh state.
- **BDD-003 Translate object** — needs runtime-editable transform.
- **Rigid body slice (FR-008 / BDD-008)** — blocked on Q4.
- **Alembic export slice (FR-013 / BDD-013)** — blocked on Q5 + Q6.

See `.agent/PLAN.md` and `.agent/CURRENT_WORK.md` for full plan and progress.
