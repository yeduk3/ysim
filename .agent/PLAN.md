# Plan — Profiler Pause-Gate Helper (`fix/profiler-pause-gate`)

> Owner: **Planner** writes; **Generator** executes; **Estimator** judges.
> Updated: 2026-05-09

## Course note: previous slice's verdict

Estimator turn 9 on `fix/cloth-thickness-band` (now merged at
`ea5e973`) returned **NOTE** — clean verdict, no items. CM-006 is
graduated, D-016 is recorded, BDD-007 is stable. This slice picks up
the only outstanding item: Estimator turn 8's still-open WARNING from
two slices ago — Block 10's pause-gate check is proxy-level and does
not exercise the actual gate predicate the render loop uses.

## Goal

Close Estimator turn-8 WARNING on `BDD-019 / history collection
pauses when sim pauses`. Today Block 10 clause (c) asserts that
**not** calling `beginFrame` / `endFrame` keeps `history.frames()`
size stable — which is trivially true and proves nothing about the
gate predicate. After this slice:

- `FrameProfiler.hpp` exposes a small RAII guard
  `ProfilerFrameGate(FrameProfiler&, bool collect, uint64_t seq,
  double t)` that wraps the `if (!paused) { beginFrame(); ... }` /
  `if (!paused) { endFrame(); }` pair as a single object.
  Constructor calls `beginFrame` only if `collect == true`;
  destructor (or explicit `close()`) calls `endFrame` symmetrically.
  Both production and the harness call this guard with the same
  predicate `!simulator.pause`.
- Production render loop (`src/main.cpp:6330-6333` and
  `6620-6622`) uses the guard instead of the inline `if
  (collectProfileFrame) { beginFrame/endFrame }` blocks. The
  begin/end behavior is preserved exactly; the `collectProfileFrame`
  bool stays as the variable that gates `physics_total` /
  `render_total` / `imgui_draw` scoped sections (those stay
  untouched — they're not the WARNING's target).
- Block 10 clause (c) drives `ProfilerFrameGate(harnessProfiler,
  !sim.pause, ...)` directly with `sim.pause = true`, calls
  `sim.update()` inside the guard's lifetime, and asserts
  `history.frames().size()` is unchanged. The assertion now exercises
  the **same gate predicate** production uses; a future regression
  in the predicate (e.g., flipping the condition) breaks the harness
  immediately rather than waiting for human GUI testing.

When this slice ships:
- 23/23 self-test PASS unchanged. Block 10's three pass labels
  unchanged in wording; only the mechanism behind clause (c)
  changes.
- `verify.sh` exits 0 cleanly.
- Estimator turn-8 WARNING closed.

## Scope

- **`include/FrameProfiler.hpp` — new `ProfilerFrameGate` class.**
  Tiny RAII guard, ~15 lines. Constructor signature:
  `ProfilerFrameGate(FrameProfiler& profiler, bool collect, uint64_t
  sequence, double wall_time_seconds)`. If `collect`, call
  `profiler.beginFrame(sequence, wall_time_seconds)`; otherwise
  no-op. Provide an explicit `close()` method that calls
  `profiler.endFrame()` (only if `collect && !closed`) so production
  can call it before the window-title read at
  `src/main.cpp:6624`. Destructor calls `close()` for safety. No
  copy/move (move is allowed if simple to implement; copy is
  forbidden).

- **Production render loop (`src/main.cpp:6328-6624` area).** Replace:
  ```cpp
  bool collectProfileFrame = !simulator.pause;
  if (collectProfileFrame) {
      frameProfiler.beginFrame(simulator.frame, currentTime);
  }
  // ... body using collectProfileFrame for nested if-guards ...
  if (collectProfileFrame) {
      frameProfiler.endFrame();
  }
  ```
  with:
  ```cpp
  bool collectProfileFrame = !simulator.pause;
  profiler::ProfilerFrameGate frameGate(frameProfiler, collectProfileFrame,
                                         simulator.frame, currentTime);
  // ... body unchanged (still uses collectProfileFrame for nested guards) ...
  frameGate.close();
  ```
  The `collectProfileFrame` bool stays — it's used by the body's
  nested scoped() guards (lines 6502, 6510, 6591). Don't touch
  those; they're correct as-is.

- **Block 10 clause (c) in `src/main.cpp::runSelfTest`.** Replace
  the "intentionally skip begin/end" mechanism with a
  `ProfilerFrameGate` constructed with the same predicate
  production uses:

  ```cpp
  size_t framesBefore = hist.frames().size();
  sim.pause = true;
  {
      profiler::ProfilerFrameGate pausedGate(harnessProfiler,
                                              !sim.pause,
                                              99, 99.0);
      // sim.update() inside the guard's lifetime — guard's collect
      // is false because !sim.pause is false, so beginFrame/endFrame
      // never run. This drives the EXACT predicate production uses.
      sim.update();
  }  // pausedGate destructor; close() is a no-op when collect=false.
  size_t framesAfter = hist.frames().size();
  if (framesAfter != framesBefore) {
      fail("BDD-019 / history collection pauses when sim pauses",
           "ProfilerFrameGate(collect=false) still pushed a snapshot");
  } else {
      pass("BDD-019 / history collection pauses when sim pauses");
  }
  ```

  The pass label is unchanged in wording so the matrix-row test
  address stays correct without an update.

## Non-goals (this slice)

- **Helper that takes a lambda body** (the user's "e.g." form). The
  render loop's body spans 300+ lines of mixed
  profiler/non-profiler work and a destructor-callable lambda would
  bloat the diff. The RAII guard is the smallest closure of the
  WARNING with no scope creep.

- **Scoped-section refactor.** `physics_total`, `render_total`,
  `imgui_draw`, etc. stay where they are; their `if
  (collectProfileFrame) { ... profiler->scoped(...) ... }` shape
  is correct and outside the WARNING's scope.

- **Per-substep collision-count plumbing changes.** The
  `addCollisionCounts` API is independent of begin/end gating;
  unchanged.

- **`FrameProfiler.endFrame()` semantic change.** The function
  already no-ops when `frame_open == false`; that property is what
  makes the guard's `close()` safe to call from both an explicit
  call site and the destructor. Don't touch.

- **CSV format / new test rows.** No spec-touching work.

- **`Simulator::pause` becomes a `bool` getter.** It already is; no
  change.

- **Resolving any of `Q1`, `Q2`, `Q4`, `Q5`, `Q6`, `Q7`.**

## Todo

Ordered. Generator executes top-to-bottom.

1. **Branch hygiene.** Already on `fix/profiler-pause-gate` (off
   `main` at `ea5e973`). No new branch. Commit prefix: `fix:`.

2. **Re-read Estimator turn-8 WARNING text** in
   `.agent/ESTIMATION.md` (last cycle's content; current file is
   turn 9). For reference, the wording was: "[Block 10's pause
   check] only proves that skipping beginFrame() / endFrame()
   leaves history unchanged. It does not exercise the actual
   paused-frame branch in the render loop at src/main.cpp:6180-6182
   / 6470-6472." (Line numbers from before the cloth-thickness slice
   — current locations are 6330-6333 and 6620-6622.)

3. **Add `ProfilerFrameGate` to `include/FrameProfiler.hpp`.**
   Place it inside `namespace profiler` near the bottom of the file,
   after `FrameProfiler` is fully defined. Shape (Generator may
   tweak idiomatically):

   ```cpp
   class ProfilerFrameGate {
   public:
       ProfilerFrameGate(FrameProfiler& profiler, bool collect,
                          uint64_t sequence, double wall_time_seconds)
           : profiler_(profiler), collect_(collect), closed_(!collect) {
           if (collect_) profiler_.beginFrame(sequence, wall_time_seconds);
       }
       ProfilerFrameGate(const ProfilerFrameGate&) = delete;
       ProfilerFrameGate& operator=(const ProfilerFrameGate&) = delete;
       ~ProfilerFrameGate() { close(); }
       void close() {
           if (closed_) return;
           profiler_.endFrame();
           closed_ = true;
       }
       bool collecting() const { return collect_; }
   private:
       FrameProfiler& profiler_;
       bool collect_;
       bool closed_;
   };
   ```

   The `closed_ = !collect` initial value means
   `close()` is a no-op when the gate isn't collecting. That keeps
   `endFrame()` paired with `beginFrame()` exactly — never called
   without it.

4. **Update production render loop.** In `src/main.cpp` ~line
   6330-6333 (begin) and ~6620-6622 (end), replace the inline
   `if (collectProfileFrame) { begin/endFrame }` blocks with a
   single guard variable + explicit `close()` call:

   ```cpp
   // ~line 6330
   bool collectProfileFrame = !simulator.pause;
   profiler::ProfilerFrameGate frameGate(frameProfiler,
                                          collectProfileFrame,
                                          simulator.frame, currentTime);
   // ... 290 lines of render-loop body unchanged ...
   // ~line 6620
   frameGate.close();
   ```

   The `collectProfileFrame` bool stays declared and stays consumed
   by the nested `if (collectProfileFrame) { auto scope =
   frameProfiler.scoped(...); ... }` blocks that intercept
   `physics_total`, `render_total`, `imgui_draw`. Those blocks are
   correct and out-of-scope.

5. **Update Block 10 clause (c) in `runSelfTest`.** Replace the
   "Intentionally skip beginFrame/endFrame here" mechanism with the
   guard-driven path described in the Scope section above. Keep the
   pass label wording identical so the matrix-row test address
   does not need changing.

6. **Run `./scripts/verify-light.sh`.** Then build `ysim` and run
   `--self-test`. Expect **23/23 PASS unchanged**. The slice's
   value is mechanism-quality, not assertion count.

7. **Optional bug-probe.** For confidence, temporarily flip the
   guard's predicate to `collect = true` (or pass `!sim.pause` as
   `false` in the harness — flipped). Confirm Block 10 clause (c)
   FAILs (a snapshot would be pushed under "paused" sim). Restore
   and confirm PASS. Mirrors the bug-probe discipline used in the
   translate-pack slice.

8. **Update DECISIONS / CURRENT_WORK / RESUME.** New numbered
   entry in `docs/DECISIONS.md`:
   > **D-NNN — `ProfilerFrameGate` is the shared profiler-pause
   > gate consumed by both production and the test harness.**
   > Production's `if (!simulator.pause) { beginFrame; ...;
   > endFrame; }` pattern is bundled into a tiny RAII guard so
   > production and the harness drive the **same predicate**.
   > Block 10 clause (c) now exercises the gate object directly
   > under `sim.pause = true` and asserts no snapshot is pushed.

   Update `CURRENT_WORK.md` and `RESUME.md` per role doc step 6/7.

9. **Stop and hand off to the Estimator.** No other refactor, no
   format changes, no spec-touching work.

## Course corrections

- **The render loop's body is large; don't touch it.** Only the
  begin/end pair (4 lines total) gets refactored. The 290 lines of
  body in between stay verbatim. Any temptation to "while we're
  here, also clean up the nested scoped() guards" is scope creep
  and is forbidden.

- **`endFrame()` must run before the window-title read at line
  6624.** That's why the guard exposes an explicit `close()` method
  rather than relying solely on the destructor. The destructor is
  there as a safety net; production calls `close()` explicitly.

- **No copy of the guard.** It owns the begin/end pairing; copying
  would call `endFrame()` twice. Mark copy/assign deleted. Move is
  allowed but the production usage doesn't need it; default-deleting
  is fine.

- **Block 10 clause (c)'s pass-label wording stays identical.**
  `BDD-019 / history collection pauses when sim pauses`. The matrix
  row's test-address line cites this label. Changing the wording
  forces a matrix update; not worth it for cosmetic improvement.

- **The "actual paused-frame branch" in production is now the
  guard.** The Estimator's WARNING called out that the harness
  doesn't exercise the production code path. After this slice, the
  guard IS the production code path; the harness drives it. The
  exact GUI-side `if (collectProfileFrame) { ... }` blocks for
  `physics_total` / `render_total` / `imgui_draw` are NOT in this
  slice's scope — they're separate, smaller predicates that gate
  scoped() sections, not the begin/end pair. The Estimator's
  complaint was specifically about the begin/end pair; that's what
  this slice addresses.

## What to read before writing code

- `include/FrameProfiler.hpp` (entire file, ~250 lines) — the place
  where `ProfilerFrameGate` lives. Particularly note how
  `beginFrame` / `endFrame` interact with `frame_open_` so the
  guard's no-op semantics are correct.
- `src/main.cpp:6325-6624` — the render loop body that wraps
  begin/end. The guard substitutes for the begin/end pair only; the
  body is unchanged.
- `src/main.cpp::runSelfTest` Block 10 clause (c) (~line 6086) —
  the assertion to refactor.
- `.agent/ESTIMATION.md` — current turn-9 verdict (NOTE, clean).
  Turn-8 WARNING text is no longer in the file but its essence is
  recapped above and in PROJECT_STATE's "Recent scope changes".
- `docs/TESTS.md#BDD-019` (lines 177–183) — the BDD this slice
  verifies. The pass-label wording must continue to match the spec
  text verbatim; do not change it.
