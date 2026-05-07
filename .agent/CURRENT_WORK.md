# Current Work — Cloth-Drape (BDD-007) Slice (feat/cloth-drape)

- File in flight: none — slice complete with **partial coverage**; ready for Estimator. Three of four BDD-007 clauses PASS; the tunneling clause still FAILs because it requires continuous collision detection (CCD), a structural change that is its own slice.
- How far:
  1. Block 6 added to `src/main.cpp::runSelfTest`, mechanizing the four "Then" clauses from `docs/TESTS.md#BDD-007` verbatim.
  2. **Real fix landed** — `collisionPipeline.broadPhase.enlargeTrajectory(system.subh)` was previously commented out in `Simulator::update`'s substep loop. Without it, fast-moving thin cloth's AABB never overlapped the static ground's AABB long enough for the broad phase to fire. Uncommenting + passing `system.subh` (per-substep dt) inflates the cloth's swept AABB and now the broad/narrow phase detect the cloth-on-ground crossing.
  3. **Cumulative narrow-collision counter** — `Scene::packedCollisionData.cumulativeNarrowCollisions` (size_t, accumulates across substeps). Necessary because the per-substep counter resets in `resetNarrow()`; the harness samples per-frame and would miss contacts that fire mid-frame and reset by the next substep. The harness's BDD-007 contact assertion now reads this cumulative value.
- What's tested:
  - 8 self-test blocks pass cleanly: CM-002, CM-003, BDD-009, BDD-011, BDD-012, three of four BDD-007 clauses, BDD-015 numMeshes round-trip, BDD-012 env round-trip, BDD-015 sim-step-after-load.
  - 1 self-test FAIL: `BDD-007 / no cloth vertex tunnels through ground` — cloth `minY` reaches ~-4.66 below the ground at -1 because the snapshot narrow-phase point-vs-triangle distance test only registers contacts during one substep of the cloth's transit; the single-substep response isn't enough to halt the cloth's downward velocity for all particles.
  - Doctest binaries (`ysim_tests`, `ysim_primitive_tests`) unchanged.
  - `TEST_MATRIX.md` `BDD-007` row stays `warning` (3/4 clauses pass).
- What's next: Estimator review. Expect a continuing BLOCK on `verify.sh` exit-non-zero, but with stronger evidence that the slice ships a real, narrowly-scoped fix (`enlargeTrajectory`) and surfaces the remaining gap (CCD) cleanly. The next planner-tracked milestone is the **cloth-CCD slice** — replace the snapshot point-vs-triangle narrow check with a swept-segment-vs-triangle check; CM-005 has the localization. With CCD wired, BDD-007's tunneling clause should pass without further harness changes.
