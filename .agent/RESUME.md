# Resume — Cloth-Drape (BDD-007) Slice (3/4 clauses passing; tunneling parked)

## Must remember

- **Branch:** `feat/cloth-drape` (off `main`, after `feat/sim-self-test` was fast-forwarded into `main`).
- **`enlargeTrajectory(system.subh)` is now live in `Simulator::update`** (uncommented from CM-005 partial fix). Inflates per-mesh swept AABBs by velocity × subh between `refit()` and `detectCollisions(...)`. Without it, fast-moving thin meshes (cloth = flat XZ plane) never overlap a static surface's AABB long enough for the broad phase to fire. **Do not re-comment.** If a future slice removes the line, the cloth-vs-ground broad phase silently breaks again — same failure mode as CM-005's original symptom.
- **`Scene::packedCollisionData.cumulativeNarrowCollisions` (size_t)** accumulates narrow-contact counts across substeps inside `narrowAndSortByVertices`. Reset by the harness before each test block; never reset by the engine. Necessary because `resetNarrow()` zeroes the per-substep counter at the start of every detect cycle, which would otherwise hide contacts that fire mid-frame.
- **Block 6 in `src/main.cpp::runSelfTest`** mechanizes BDD-007's four "Then" clauses verbatim from `docs/TESTS.md`. Three pass; one fails (tunneling). The fail is *not* a harness bug — it's a real CCD gap. Don't relax the assertion to make `verify.sh` green.
- **CM-005 is updated** to reflect the partial fix. The remaining gap is the snapshot-vs-swept narrow-phase check; that's the next slice's job.
- **Spec substitution call still in force**: `TESTS.md#BDD-007` says "static rigid sphere"; v1 has no rigid pipeline (Q4 blocked). Harness uses the existing Float-tagged ground plane. Spec intent — "cloth drapes onto a static surface" — is satisfied by either; the rigid-sphere variant returns when the rigid slice ships.

## Last decisions + why

- No new `DECISIONS.md` entries this slice. The `enlargeTrajectory` change is a bug fix (CM-005), not a structural decision; the `cumulativeNarrowCollisions` counter is harness infrastructure under D-012.

## Next step you were about to take

Slice complete with partial coverage. The next concrete step is the **Estimator's** turn — `verify.sh` exits non-zero (1 self-test FAIL), Estimator likely BLOCKs to loop the planner.

Next planner-tracked milestone: **cloth-CCD slice**. Localization in CM-005:
- The narrow-phase kernel `narrow_pt_tri` (`src/metal/bruteforce.metal`) does a snapshot point-vs-triangle distance check. Replace with a swept-segment check: for cloth particle's old position `p0` and current position `p1`, test whether segment `[p0, p1]` crosses the triangle plane and lands inside the triangle. Register contact at the crossing point with full-depth pushback.
- Quick alternative for the planner to consider: bump `Simulator::radius` from `0.012` to `0.05` (or wider) so the snapshot accepts a wider band. Coarse — introduces false positives — but might pass BDD-007 tunneling clause without writing CCD.
- Whichever path the planner picks, the BDD-007 Block 6 in this slice should turn the fourth clause green without code changes.

After CCD: BDD-002 import UI; BDD-102 determinism mechanization; material/behavior/rigid/Alembic each blocked on its respective spec answer or shader work.

See `.agent/PLAN.md` and `.agent/CURRENT_WORK.md` for full plan and progress.
