# PLAN — Loadshader skip-safety + HiddenGLContext cleanup (`fix/loadshader-skip-safety`)

> Owner: **Planner** writes; **Generator** executes; **Estimator** judges.
> Updated: 2026-05-13

## Course note: previous slice's verdict

Estimator turn 28 returned **WARNING** on the bdd-018-inspector-live-edit
slice. The WARNING was the standing parked BDD-018 behavior-tag gap
(blocked behind BDD-006/Q2); NOT a new regression and NOT actionable
this slice. The NOTE was the duplicated `MeshInspectorTarget` wiring
between production (`src/main.cpp:8349-8387`) and Block 26 — a future
cleanup candidate when source-file split lands.

This slice closes the older **turn-27 WARNING + NOTE** that the
BDD-018 slice deliberately did NOT fold, per user direction:

- WARNING: `Program::loadShader` failure path calls `exit(1)` (via
  `printLog()`), defeating Block 25's documented `programID == 0`
  SKIP semantic.
- NOTE: `HiddenGLContext` destructor doesn't call `glfwTerminate()`
  on early-init failure paths.

## Goal

Make Block 25's documented SKIP-on-shader-load-failure semantic
*actually work* by refactoring the `Program::loadShader` failure path
to return-on-error with `programID == 0`, AND harden
`HiddenGLContext`'s constructor failure paths so the destructor cleans
up symmetrically. Add a mechanizing assertion (new Block 27) that calls
`Program::loadShader` with explicitly bogus filenames and asserts
`programID == 0` — without this assertion the loud-FAIL bug-probe is
the only signal, and a future regression that re-introduces
`exit(1)` would only surface as a self-test process abort (not as a
clean FAIL diagnostic).

Closes Estimator turn-27 WARNING + NOTE. No new BDD/FR. New `D-034`
(loader failure contract) + new `CM-012` (utility-helper-calls-exit
trap pattern).

## Scope

**Design call: Shape B** from the slice brief — `printLog()` and
`linkShader()` both stop calling `exit(1)`, producing a uniform "return
on failure with `programID = 0`" contract. Shape A (in-place return
only inside `loadShader`) doesn't work because `loadShader` doesn't
own the exit point — `printLog()` (called by `linkShader()`, called by
each `loadShader` variant) is where `exit(1)` actually lives. A
syntactic Shape A patch would require duplicating cleanup logic into
each `loadShader` variant; Shape B fixes it at the source.

**Production behavior implication.** Currently when a production-side
`loadShader` call hits a compile or link error, the process aborts
loudly. After Shape B, the process keeps running with `programID = 0`;
production callers that don't check will silently render with no
program bound (`glUseProgram(0)`). This is a behavior change worth
documenting in D-034 + the Course corrections section below. The
production main path's first shader load is at the bottom of
`runSelfTest`'s sibling code path (`main()` at `src/main.cpp:8127+`);
the Generator should verify by reading whether existing callers check
`programID` and, if NOT, add a defensive `if (!programID) { print +
exit(1); }` at the production call site only. The harness keeps the
clean SKIP semantic; production keeps the loud-failure semantic but
moves the abort decision to the caller.

**NEW symbols this slice adds**:

- `bool HiddenGLContext::glfwInitialized = false` — new struct field
  in `include/HiddenGLContext.hpp`. Set to `true` after the
  constructor's `glfwInit()` returns truthy. The destructor consults
  this flag to decide whether to call `glfwTerminate()`.
- Block 27 in `runSelfTest` (new lines after Block 26's closing
  brace, before the failure-counter summary). Mechanizes D-034 by
  calling `Program::loadShader` with explicitly bogus filenames and
  asserting `programID == 0`.
- `docs/DECISIONS.md` D-034 entry.
- `docs/mistakes/COMMON_MISTAKES.md` CM-012 entry.

**MODIFIED symbols this slice changes in place** (the user's brief
used "refactor" + "add ... to ... destructor" — modification language,
so in-place edit is allowed per the parallel-symbol-rule exception):

- `Program::printLog()` in `include/program.hpp` — remove the
  `exit(1)` line. After this slice, `printLog` only prints; the
  process lives.
- `Program::linkShader()` in `include/program.hpp` — after
  `printLog()` on link-status failure, call `cleanUp()` (which zeros
  `programID` and all shader IDs) before the existing `return`.
- `Program::loadShader(const char*, const char*, const char*)`
  (the 3-arg geom variant Block 25 uses) and the other 3 overloads
  — after each `loadShaderOf` call, if `programID == 0` (compile
  failure already triggered an internal cleanUp), `return`
  immediately. This prevents subsequent `loadShaderOf` calls and
  `linkShader()` from running against a zero programID, which is
  wasteful and noise-generating. ~6 line change across the 4
  overloads (one early-return per).
- `HiddenGLContext::~HiddenGLContext()` in
  `include/HiddenGLContext.hpp` — symmetric cleanup: destroy window
  if non-null (existing behavior), AND call `glfwTerminate()` if
  `glfwInitialized` (new behavior, gated by the new field).
- The header comment block at the top of `HiddenGLContext.hpp` is
  updated to reflect the new contract: glfwTerminate IS now called on
  destruction. The original "avoids invalidating a future YGLWindow"
  rationale is rewritten to note that `--self-test` mode and
  default-main mode never coexist; the cleanup is correctness, not
  a regression risk.

**PRESERVED symbols** (this slice MUST NOT modify any of these):

- `YGLWindow` (`include/YGLWindow.hpp`) — production main window;
  this slice touches GLFW init/term semantics only in the hidden
  context.
- `Framebuffer` (`include/framebuffer.hpp`) — unchanged.
- `MeshGL` — unchanged.
- `mesh_inspector::MeshInspectorTarget` — unchanged (just shipped in
  the prior slice; no follow-up needed).
- `runSelfTest` Blocks 1–26 — Block 27 is appended.
- `Simulator::setMaterial / translateObject / rotateObject` — unchanged.
- All shader source files (`shader.vert`, `shader.geom`, `shader.frag`)
  — unchanged.

**No spec substitution.** This slice doesn't mechanize any BDD; it
fixes test-harness skip-safety + records a new architectural
invariant (D-034) about the loader's failure contract.

## Non-goals

- **NO production-side audit of every `loadShader` call** to add
  failure checks. If the Generator finds just the one main()
  call site, add the defensive `if (!programID) abort` there.
  Anything beyond a single call site is out of scope — that's a
  separate refactor slice if it ever becomes needed.
- **NO change to `loadText` or `shaderCompileCheck`** — their
  existing return-on-failure shape is already correct.
- **NO change to `cleanUp` or the destructor** of `Program` — those
  already handle non-zero `programID` correctly.
- **NO new BDD/FR.** Test-harness skip-safety isn't user-facing
  acceptance; D-034 + CM-012 record the contract.
- **NO new harness gotcha entries** in `docs/CONVENTIONS.md` or
  the stable-harness-gotchas list in `GENERATOR.md` — Block 27's
  SKIP-on-no-GL pattern is already established (Block 25 wrote the
  template).
- **NO mechanization of the `HiddenGLContext` destructor cleanup**
  — testing that `glfwTerminate` was called requires instrumentation
  the Estimator can't run; commit-time visual verification of the
  destructor body is the gate. Mention this trade-off in CM-012's
  fix direction or as a NOTE the Estimator can pick up.
- **NO folding of any other open WARNING.** Estimator turn 28's
  WARNING (BDD-018 behavior-tag) is BDD-006/Q2-blocked and out of
  scope. Turn 28's NOTE (duplicated `MeshInspectorTarget` wiring)
  is queued for the source-file split slice when that ships.

## Todo

1. **Branch hygiene.** Already on `fix/loadshader-skip-safety` (off
   `main` at `1d8b6d9`). Commit prefix: `fix:` (closes
   Estimator-flagged WARNING + NOTE, not a new feature).
2. **Modify `include/program.hpp`**:
    1. **`printLog()`**: remove the `exit(1);` line at the end of
       the function. The function now only prints the program
       info-log.
    2. **`linkShader()`**: after the `printLog();` call on link
       failure, call `cleanUp();` before the existing `return;`.
       (`cleanUp` zeros `programID` and all shader IDs, so the
       caller's `if (!programID)` check fires.)
    3. **Each `loadShader` overload (4 total)**: after each
       `loadShaderOf(...)` call, check `if (!programID) return;`
       and return early. This prevents wasteful subsequent calls
       once a compile failure has nuked the program. The early
       return is ~1 line per call site; 4 overloads × N shader
       stages each. Generator may write a one-line macro/helper or
       just inline the check.
3. **Modify `include/HiddenGLContext.hpp`**:
    1. Add `bool glfwInitialized = false;` field after the existing
       `bool ok = false;` field.
    2. In the constructor, set `glfwInitialized = true;` right
       after `if (!glfwInit()) return;` (i.e., on the success branch
       of glfwInit).
    3. In the destructor, after the existing
       `if (window) glfwDestroyWindow(window);`, add
       `if (glfwInitialized) glfwTerminate();`. Symmetric cleanup.
    4. Update the header comment block to reflect the new contract:
       glfwTerminate IS now called on destruction; `--self-test`
       mode and default-main mode are mutually exclusive at runtime
       so the original "avoids invalidating future YGLWindow"
       worry doesn't apply.
4. **(Defensive — only if Generator finds it's a single call site)
   Production `loadShader` callers in `src/main.cpp`**: after each
   `loadShader` call, add a defensive `if (!program.programID) {
   std::cerr << "shader load failed\n"; std::exit(1); }`. This
   preserves production's loud-failure behavior. Skip this todo
   if there are >1 call sites (out of scope; record as a CM-012
   follow-up).
5. **Author Block 27 in `src/main.cpp::runSelfTest`** at the end
   of the block sequence, before the failure-counter summary (look
   for `BDD-018 / rotate inspector edit propagates live` as
   anchor — Block 27 goes right after Block 26's closing brace).
   Block 27 structure:
    1. Bring up `HiddenGLContext glctx(64, 64);` (small context;
       no actual rendering needed for the loader check).
    2. SKIP if `!glctx.ok`.
    3. `Program p;`
    4. `p.loadShader("__nonexistent_vert__.vert",
       "__nonexistent_geom__.geom", "__nonexistent_frag__.frag");`
       — the explicit-3-arg variant that Block 25 uses, so the
       fix is exercised on the same overload.
    5. Assert `p.programID == 0`. PASS label:
       `D-034 / Program::loadShader returns programID=0 on
       missing shader files`.
    6. If `p.programID != 0`, FAIL with the actual value (probably
       indicates the loader still calls exit(1) somewhere or a new
       regression introduced exit-on-failure).
6. **Bug-probe each fix** (per GENERATOR.md bug-probe discipline):
    1. **Loader fix probe**: temporarily revert the `printLog`
       change (re-add `exit(1)`). Build. Run `--self-test`. Block
       27 should still produce the right output BUT the process
       should abort INSIDE Block 27's `p.loadShader` call (before
       the assertion fires), so the self-test reports < expected
       PASS count + the harness terminates abnormally. This
       confirms the fix is load-bearing. Restore.
    2. **HiddenGLContext probe**: Generator may inspect the
       destructor body visually + commit review. A runtime probe
       would require forcing `glfwInit` to fail mid-test, which
       isn't easy on macOS. Skip the runtime probe; the visual
       review + the new `glfwInitialized` field is the audit
       surface. Document in CURRENT_WORK.md that runtime probe was
       intentionally skipped.
7. **Build + verify deterministic.** `cmake --build build` then
   `./src/ysim --self-test` from `build/` 5 times in a row; expect
   `49/49 PASS` every time (48 prior + 1 new Block 27 PASS). If
   any run differs, STOP and hand back.
8. **`verify-light.sh` cross-check.** Run from project root. Expect
   doctest `159/159 SUCCESS` + `1120/1120 SUCCESS` unchanged (no
   test/ files edited).
9. **Append `docs/DECISIONS.md` D-034 entry**: title
   "Program::loadShader failure path returns silently with
   programID=0; printLog/linkShader stop calling exit(1)." Body
   covers: file/function pointers (program.hpp), rationale (harness
   SKIP semantics), invariant (callers MUST check `programID`
   before using the program; production callers in main.cpp got
   defensive `if (!programID) exit(1)` per todo 4 OR were verified
   to already check), reference to Estimator turn 27's WARNING that
   triggered this slice. Also covers the HiddenGLContext destructor
   cleanup: symmetric construction/destruction via
   `glfwInitialized` field.
10. **Append `docs/mistakes/COMMON_MISTAKES.md` CM-012 entry**:
    title "Utility helper calls `exit(1)` on failure path, defeating
    harness SKIP semantics." File/function pointer (program.hpp's
    `printLog`/`linkShader` chain). Low-level cause: a print-the-error
    helper unilaterally exited the process. High-level cause: utility
    helpers should print + return / set failure flag; the caller
    decides whether the failure is fatal. Fix direction: at every
    utility-helper boundary, audit whether the helper makes process-
    lifetime decisions or merely communicates state; the latter is
    almost always correct.
11. **TEST_MATRIX update**: NO row changes (this slice doesn't touch
    any BDD). Optional: add a short tracking note that BDD-005's
    Block 25 SKIP path is now honest. Skip if it bloats the row.
12. **Update `.agent/CURRENT_WORK.md`** with: file in flight (none —
    slice complete), how far (loader + HiddenGLContext fixes
    + Block 27 + D-034 + CM-012), what's tested (49/49 PASS
    deterministic + loader bug-probe verified), what's next
    (Estimator review).
13. **Update `.agent/RESUME.md`** with: must-remember (the
    loader contract change, the production behavior implication, the
    `glfwInitialized` field, the bug-probe shape), last decisions +
    why (D-034 picks Shape B; CM-012 trap pattern), next step
    (Estimator review).

## Course corrections

- **Loader behavior change is production-affecting.** Before this
  slice, `Program::loadShader` failures abort the process. After,
  failures leave `programID == 0` and the process continues. Any
  production caller that doesn't check `programID` will silently
  render with no program bound (most OpenGL drivers emit a
  GL_INVALID_OPERATION on the first draw call). Todo 4 adds a
  defensive check at the production call site. If the Generator
  finds >1 call site, treat that as a build-time discovery (hand
  back to Planner) — multi-site audit is its own slice.
- **`linkShader()`'s post-success path calls `glUseProgram(programID)`**
  unconditionally; this is unchanged by the slice. After the fix,
  if linkShader fails, programID becomes 0, and the post-failure
  `glUseProgram` call (which doesn't happen — the failure branch
  returns before that line) is moot.
- **The `glfwInit/glfwTerminate` ref-counting question**: GLFW's
  `glfwInit` is NOT ref-counted; calling it twice in a row is
  idempotent and `glfwTerminate` undoes ALL of it regardless of
  call count. So if a future caller creates two `HiddenGLContext`s
  back-to-back, the FIRST destructor's `glfwTerminate` invalidates
  the SECOND context's window. This is a real risk for nested or
  parallel use. Mitigation for v1: only one `HiddenGLContext` is
  ever alive at a time (Block 25 + Block 27 sequence them
  back-to-back, never concurrent). If future code creates them
  concurrently, the `glfwInitialized` flag needs a process-global
  counter, not a per-instance bool. Document this constraint in
  the header comment.
- **D-014 / D-021 / D-027 / D-032 unchanged.** This slice doesn't
  touch any of the prior simulation or render decisions.
- **`feedback_make_means_add_new` rule.** The user's slice brief
  uses "refactor" (loader) + "add ... to ... destructor"
  (HiddenGLContext). "Refactor" is explicit modification language;
  "add to destructor" reads as in-place edit of an existing method
  (modifying the existing destructor's body), not creation of a
  new class. The slice respects the rule in its softer form: new
  symbols are `glfwInitialized` (field), Block 27 (test), D-034
  (decision), CM-012 (mistake); modified symbols are
  `printLog`/`linkShader`/each `loadShader` overload/`~HiddenGLContext`.
  No widening of `Program`'s or `HiddenGLContext`'s public API
  beyond the new `glfwInitialized` member.

Expected matrix delta: none (no BDD row touched).
Expected self-test count: 48 → 49 (Block 27 adds 1 PASS label).
Expected verify.sh: exits 0 on macOS dev host. On the Estimator's
Linux container the Metal SKIP path returns 0 before Block 27 reaches;
doctest binaries pass unchanged.
Expected Estimator verdict: NOTE or WARNING. Possible items:
(i) production behavior change (loud-abort → silent programID=0) may
warrant a CM-012 note even with the defensive Todo 4 check, in case
future callers forget; (ii) the `glfwInit` non-ref-count constraint
(Course corrections bullet 3) is a documented-not-enforced limitation
that a future concurrent-context slice would hit; (iii) Block 27
doesn't verify the HiddenGLContext cleanup path runtime-wise — that's
a NOTE about audit limitations.
