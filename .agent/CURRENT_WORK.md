# Current Work — BDD-010 Collision Detection Slice (`feat/bdd-010-collision-detected`)

- File in flight: none — slice complete; ready for Estimator. **27/27 self-test PASS** deterministic across 5 consecutive runs. Doctest 159/159 + 1120/1120 green.
- How far: all 10 PLAN todos done.
  - **Block 13** mechanizes BDD-010's two clauses verbatim:
    - Positive: cloth (4×4 grid, mass=0.1) co-located at `y = -1` with the ground plane → AABBs overlap at t=0 → after one `sim.update()`, assert `cumulativeNarrowCollisions > 0` AND last-substep `narrowCollisions` has at least one entry with `objPair.query != objPair.target` (the (A, B) distinct-mesh contact pair).
    - Negative: cloth at `y = 10` above ground → AABBs disjoint → after one `sim.update()`, assert `cumulativeNarrowCollisions == 0`.
  - **Cloth-position discovery during build:** initial plan placed cloth at `y = -0.99` (touching ground from above). At t=0 with zero velocity, the broad phase couldn't fire — both AABBs are essentially flat planes at distinct y values, and `enlargeTrajectory(system.subh)` only inflates by velocity·subh which is initially zero. Moved to `y = -1.0` so cloth particles land within the ground's flat AABB at t=0 (modulo D-018 jiggle ~1e-4); broad fires immediately. Comment in Block 13 explains why.
  - **BDD-004 fold-in (Estimator turn-13 WARNING):** Block 12 now asserts `quatNorm(r1_post_load) ≈ 1` *before* the re-multiply. If load-side norm regression occurs, the failure surfaces as `r1_post_load is not unit-norm; |r1_post_load| = ...` — without this gate, `quatNormalize(dq2 * r1_post_load)` would silently absorb it.
  - **Bug-probe verified:** moved the negative case's cloth y to `-1.0` (overlapping); negative clause FAILed with `cumulativeNarrowCollisions == 92 on a scene with disjoint AABBs`. Restored; flips back to PASS.
- What's tested:
  - **27/27 self-test PASS** on macOS Apple Silicon, deterministic across 5 runs.
  - Doctest binaries unchanged.
  - `docs/TEST_MATRIX.md` row `BDD-010` promoted `pending → pass`.
- Non-goals respected: no cloth-on-cloth, no multi-frame collision, no cloth-vs-cube/sphere targets, no `enableSelfCollisions` flag flip, no spec edits, no new D-NNN (mechanization-only slice).
- What's next: Estimator review. Expect verdict at NOTE level — Block 13 verbatim from spec, two clauses both bug-probe-verified, BDD-004 fold-in closes the prior turn's WARNING.
