# Current Work — Translate-Pack + BDD-019 Slice (`fix/translate-pack-and-bdd019`)

- File in flight: none — slice complete; ready for Estimator. **`./src/ysim --self-test` 23/23 PASS**, `./scripts/verify-light.sh` clean.
- How far: all 10 PLAN todos done.
  - **D-015** — `Simulator::translateObject` write-back to initializer. Same `dynamic_cast` cascade pattern as `Scene::pack` pack-time seed and `Simulator::toSnapshot` realized-mesh override. Three sites must move together when a fifth initializer subtype ships.
  - **Block 9 clause (d)** — `BDD-003 / translate survives Scene::pack rebuild`. Calls `sim.initialize()` after the existing three "Then" assertions and re-checks `transformPosition` + per-axis `state.x` mean against the (1, 2, 3) target. Bug-probe (commenting out the write-back cascade) confirmed the assertion FAILs without the fix; restored and PASSes after.
  - **Block 10 (BDD-019)** — three new PASS lines authored verbatim from `docs/TESTS.md#BDD-019`:
    - `BDD-019 / per-section timings updated each frame` — fresh `FrameProfiler`, beginFrame → sim.update → endFrame, assert latest snapshot has at least one non-zero `section_ms`.
    - `BDD-019 / CSV written under profiles containing history` — `exportCsv("/tmp/ysim_profiler_test.csv")` returns true, header contains `frame_sequence`/`frame_ms`/`broad_collisions`/`narrow_collisions`, at least one data row exists. Path substituted to `/tmp` for harness hygiene; substitution noted in block comment per spec-substitution rule.
    - `BDD-019 / history collection pauses when sim pauses` — set `sim.pause = true`, deliberately skip beginFrame/endFrame (matching production gating at `main.cpp:6180-6182`), assert `history.frames().size()` did not grow.
- What's tested:
  - **23/23 self-test PASS** on macOS Apple Silicon, deterministic.
  - Doctest binaries unchanged (159 + 1120 assertions, both green).
  - `docs/TEST_MATRIX.md` row `BDD-019` promoted from `pending` to `pass`. Row `BDD-003` test-address line extended to mention the new round-trip clause + D-015.
- Non-goals respected: CM-006 vn-zero gate stays parked; no profiler GUI changes; no CSV format mutations.
- What's next: Estimator review. Expect verdict at NOTE level — both turn-7 WARNINGs are closed, bug-probe was used to verify the new assertion catches the bug pre-fix.
