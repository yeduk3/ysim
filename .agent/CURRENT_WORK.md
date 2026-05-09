# Current Work — Profiler Pause-Gate Slice (`fix/profiler-pause-gate`)

- File in flight: none — slice complete; ready for Estimator. **23/23 self-test PASS**, `./scripts/verify-light.sh` clean.
- How far: all 9 PLAN todos done.
  - **D-017** — new `profiler::ProfilerFrameGate` RAII guard in `include/FrameProfiler.hpp` (~30 lines including the doc comment). Constructor takes `(profiler, collect, sequence, wall_time)`; calls `beginFrame` only when `collect == true`. `close()` is symmetric, only calls `endFrame` if it was opened. Destructor calls `close()` as a safety net. Copy/assign deleted (the guard owns the begin/end pairing).
  - **Production swap** — render loop's `if (collectProfileFrame) { beginFrame }` / `if (collectProfileFrame) { endFrame }` pair replaced with a single `ProfilerFrameGate frameGate(...)` declaration + an explicit `frameGate.close()` before the window-title read. The 290+ lines of body in between are unchanged; nested `if (collectProfileFrame) { profiler->scoped(...) }` guards stay as-is (out of slice scope).
  - **Block 10 clause (c) refactored** — instead of "intentionally skip beginFrame/endFrame", the harness now constructs `ProfilerFrameGate(harnessProfiler, !sim.pause, ...)` with `sim.pause = true` and asserts no snapshot is pushed. The harness drives the EXACT predicate production uses (`!sim.pause`). Pass label wording unchanged.
  - **Bug-probe verified** — flipped the harness's `collect` argument to hardcoded `true` and confirmed clause (c) FAILs with `ProfilerFrameGate(collect=false) still pushed a snapshot`. Restored; flips back to PASS. The assertion catches what it claims to catch.
- Non-goals respected: no lambda-callable helper, no scoped-section refactor, no spec-touching work, no CSV format changes.
- What's tested:
  - **23/23 self-test PASS** on macOS Apple Silicon.
  - Doctest binaries unchanged (159 + 1120 assertions, both green).
  - Pass label `BDD-019 / history collection pauses when sim pauses` is unchanged so `docs/TEST_MATRIX.md` does not need updating.
- What's next: Estimator review. Expect verdict at NOTE level — Estimator turn-8 WARNING closed via shared gate object + bug-probe-verified harness assertion.
