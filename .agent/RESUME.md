# Resume — Profiler Pause-Gate Slice

## Must remember

- **Branch:** `fix/profiler-pause-gate` (off `main` at `ea5e973`).
- **D-017 is load-bearing.** `ProfilerFrameGate` (in `FrameProfiler.hpp`) is the **single** owner of the `beginFrame` / `endFrame` pairing. Construction with `collect == true` calls `beginFrame`; `close()` (or destructor) calls the matching `endFrame` exactly once. Initial `closed_ = !collect` guarantees `endFrame` is never called without a paired `beginFrame`. Anyone adding a new code path that wants to drive the profiler's per-frame collection must construct a `ProfilerFrameGate`, not call `beginFrame` / `endFrame` directly.
- **Production calls `frameGate.close()` explicitly** before the window-title read at the end of the render lambda. Destructor-only would change ordering — the title reads `frameProfiler.history().latestFrame()` and needs `endFrame` to have run. Don't move `close()` later or remove it.
- **Block 10 clause (c) drives the same gate predicate** production uses (`!sim.pause`). If a future refactor changes the predicate (e.g., adds "also pause when window unfocused"), the harness will still drive the new predicate as long as it constructs the gate with `!sim.pause`. Bug-probe-verified.
- **Pass label wording unchanged** — `BDD-019 / history collection pauses when sim pauses`. Matrix-row test address still correct.
- **Nested scoped() guards stay** — the `if (collectProfileFrame) { auto scope = frameProfiler.scoped(...); ... }` blocks for `physics_total`, `render_total`, `imgui_draw` etc. are unchanged. They're out of this slice's scope and correct as-is.

## Last decisions + why

- **D-017** — RAII guard wraps the begin/end pair. Rejected: free lambda-helper (render-loop body spans 290 lines and includes ordering constraints with the post-endFrame window-title read); free predicate function (too thin — doesn't enforce begin/end pairing); leave the proxy (Estimator turn-8 explicitly called it out as a real risk). The RAII guard owns the pairing and is testable in isolation.

## Next step you were about to take

Slice complete. Next concrete step is the **Estimator's** turn — `./scripts/verify.sh` should exit 0 cleanly with **23/23** self-test PASS lines. Expected verdict: NOTE level.

After this lands, planner-tracked candidates per `PROJECT_STATE.md`:

- **BDD-102 Determinism mechanization** — extend the harness with two-runs-bit-identical assertion against a saved-scene baseline. Smallest slice. Newly tractable now that BDD-007 is stable and the profiler gate is testable end-to-end.
- **Material editing UI (FR-005 / BDD-005)** — needs PBR preview shader.
- **Behavior assignment UI (FR-006 / BDD-006)** — in-place behavior switching reallocates per-mesh state.
- **Rigid body slice (FR-008 / BDD-008)** — blocked on Q4. When this lands it adds a fifth initializer subtype and triggers D-015's three-site cascade update.
- **Alembic export slice (FR-013 / BDD-013)** — blocked on Q5 + Q6.

See `.agent/PLAN.md` and `.agent/CURRENT_WORK.md` for full plan and progress.
