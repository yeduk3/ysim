# Current Work — BVH bottom-up GPU combine, fix-turn (`feat/bvh-bottomup-gpu`)

- File in flight: none — fix-turn complete. **42/42 self-test PASS** deterministic across 5 runs. Doctest 159/159 + 1120/1120 green.
- How far: all 12 PLAN todos done.
  - **N=1 guard added** to `BVH::bottomUpBoxesGPU` (`src/main.cpp`). Widened `if (numPrimitives == 0) return;` to `<= 1`. Comment explains why N=1 also short-circuits (leaf-is-root, no interior nodes to combine, treeParent[0] uninitialized).
  - **Block 22 added** to `runSelfTest`. Writes a single-triangle `.obj` to `/tmp/bdd_d029_n1.obj`, imports via `sim.importMesh`, runs `sim.initialize + sim.update` to exercise BVH::build + BVH::refit with N=1, asserts:
    1. No crash.
    2. `tree[0]` AABB matches the triangle bounds within tolerance.
    3. `queryClickRay` finds the single triangle (walks the BVH correctly when tree[0] is leaf-root, exercising D-024 leaf-recognition under N=1).
    Pass label: `D-029 fix / N=1 BVH safely bypasses bottom-up combine`.
  - **TEST_MATRIX BDD-010 row prose updated** (NOTE fold-in). Drops the OLD `objPair.query != objPair.target` shape, replaces with cumNarrow-based description + CM-011 cross-ref. Status stays `pass`.
  - **D-029 entry in `docs/DECISIONS.md`** appended fix-turn paragraph: documents the N=1 guard widening + the Apple-Silicon-mask of the bug-probe.
- What's tested:
  - 42/42 self-test PASS on macOS Apple Silicon, deterministic 5×.
  - Block 22 (new) + Block 21 + Block 13 + FR-003/004 (refit-after-edit) all green.
  - Doctest 159/159 + 1120/1120 SUCCESS via `verify-light.sh`.
  - **Bug-probe is Apple-Silicon-masked (documented honestly).** Probe (a) — revert the N=1 guard — silently passed all 5 runs because `treeParent[0]` reads as 0 from the zero-init shared buffer, the kernel's first-arrival path exits with `old == 0u`, and tree[0] is preserved. Probe (b) — guard removed AND force `treeParent[0] = 999999` (deliberate OOB) — STILL passed because Metal's shared storage absorbs OOB atomic reads (return 0) and OOB writes (silently dropped or landed in adjacent unmapped memory). The UB the Estimator flagged is real (spec-incorrect, would crash on non-Apple memory models), just invisible to this harness on Apple Silicon. The production guard is the spec-correct fix regardless.
- Non-goals respected: no GPU `enlargeTrajectory`, no `buildTree_*` restructuring, no new D-NNN, no CM-012 (the trap pattern is captured directly in D-029's fix-turn paragraph; not generic enough to warrant a separate CM).
- What's next: Estimator review (Codex). Expected verdict: NOTE-clean (BLOCK closed; turn-23 NOTE folded into TEST_MATRIX; bug-probe Apple-Silicon-mask documented in D-029 + here).
