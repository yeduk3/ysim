# PLAN — Role-doc maintenance pass (`chore/role-doc-maintenance-pass`)

> Owner: **Planner** writes; **Generator** executes; **Estimator** judges.
> Updated: 2026-05-13

## Course note: previous slice's verdict

Estimator turn 30 (re-issued, post-BLOCK-fix) returned **NOTE** on the
inspector rotation ergonomics slice (D-035). Sole NOTE: parallel
implementations of conversion math in `src/main.cpp` and
`include/MeshInspectorWindow.hpp` should stay mirrored until source-
file split lands. **This NOTE is folded into the role-doc pass** as a
documented standing constraint in PLANNER.md (rather than a code
change in this slice — the lockstep is already in place; the doc
needs to record it as the long-lived discipline going forward).

## Goal

Fold discipline learnings from slices 17–30 (turns 17–30 of the
Estimator log; approximately the last 14 cycle iterations) into the
three role documents at `docs/roles/{PLANNER,GENERATOR,ESTIMATOR}.md`.
Each role doc absorbs the lessons that fall in its read/write surface
+ procedure scope; the same lesson may appear in 2–3 docs with
role-specific framing (e.g., "probe coverage edges" is a planning
checklist for PLANNER, an authoring discipline for GENERATOR, and a
verification heuristic for ESTIMATOR).

Pure docs slice. No code/test changes. Commit prefix `chore:`. No new
D-NNN, no new CM-NNN, no new BDD/FR, no TEST_MATRIX changes. The
previous role-doc maintenance pass was commit `4217619` after turn 16
(2026-05-10) — 14 cycles ago — so the pacing aligns with the
established "every ~12–15 cycles" rhythm.

## Scope

**Five discipline lessons to fold** (each enumerated below with its
role-specific framing):

### Lesson 1 — PROBE-COVERAGE-EDGES (turn-30 bite)

**Source:** D-035 turn-30 BLOCK. Block 28's three positive-w
forward-built quaternion probes missed the antipodal-identity case
`q = (-1, 0, 0, 0)` where `quatToAxisAngle` computed
`s = sqrt(1 - qw²) = 0` and produced `axis = q.x/s = NaN`. The
Estimator caught the gap by reading the diff (probe array's inputs
were all in one regime); the harness's clean 51/51 PASS hid the
divide-by-zero entirely.

- **PLANNER.md** absorbs as a new **procedure step** ("Domain-edge
  probe enumeration for math-layer slices"). When planning any slice
  that adds a new conversion / extraction / inverse function,
  enumerate the input domain's edges in PLAN.md's Course corrections:
  antipodes (`q` vs `-q` for quaternions, `θ` vs `2π - θ` for
  angles), singularities (gimbal lock, division by zero), degenerate
  cases (zero-norm axis, identity input, ±boundary clamps for
  `acos`/`asin`/`sqrt`), zero-magnitude inputs. Pass these to the
  Generator as required probe coverage in the Todo list.
- **GENERATOR.md** absorbs in **bug-probe discipline** as an
  "edge-case probe enumeration" sub-section. When authoring a math
  function's harness probes, sense-check whether the probe array
  spans the function's natural domain or sits in one regime. If the
  PLAN specifies 3 inputs and they're all positive-w / all
  origin-centered / all axis-aligned, ask whether the other regimes
  are exercised. Add edge probes proactively when the function's
  math has natural boundaries (antipodes, singularities,
  degeneracies).
- **ESTIMATOR.md** absorbs in **"What to flag, what to skip"** as a
  flag-able item: "missing-edge probe coverage." When the
  Generator's probe array exercises only one regime of a function's
  domain (positive-w-only, zero-only, single-axis-only, etc.), flag
  the gap as WARNING (or BLOCK if the missed edge produces NaN /
  process-corruption in the existing code path). The "stricter-
  than-spec assertions" heuristic that's already in role doc has
  the same shape — this is its domain-coverage sibling.

### Lesson 2 — PARALLEL-IMPL-LOCKSTEP (turn-29 NOTE, turn-30 re-NOTE)

**Source:** D-035 introduced parallel copies of the same conversion
math in `src/main.cpp` (Quat-based) and `include/MeshInspectorWindow.hpp`
(float-array, so the inspector TU doesn't need the bare `Quat`
struct). Turn-29 NOTE flagged the duplication as a standing
constraint; turn-30 BLOCK fix-turn had to honor it mid-fix (apply the
antipodal canonicalization to BOTH files in the same commit).

- **PLANNER.md** absorbs as a new **"Standing constraints"
  subsection** under (or beside) the existing **Spec-substitution
  log**. This subsection lists items that don't substitute spec
  wording but travel across slices as documented limitations the
  Estimator carries forward without re-flagging. Initial entries:
  - **PARALLEL-IMPL-LOCKSTEP** (D-035, 2026-05-13): conversion math
    duplicated in `src/main.cpp` + `include/MeshInspectorWindow.hpp`.
    Any change to one MUST mirror to the other in the same commit.
    Resolved when source-file split lands.
  - **GLFWINIT-NON-REF-COUNTED** (D-034 HiddenGLContext header
    note, 2026-05-13): GLFW's `glfwInit()` is not ref-counted, so
    concurrent `HiddenGLContext` instances would clash. v1 has no
    concurrent use; future concurrent users would need a process-
    global init counter.
  - **DUPLICATED-INSPECTOR-WIRING** (turn-28 NOTE, 2026-05-13):
    production `buildSelectedMeshTarget` lambda at
    `src/main.cpp:8349-8387` shape-duplicated in Block 26's harness
    setup. Bundles with source-file split.
  - **BDD-018-BEHAVIOR-TAG-PARKED** (turn-28 WARNING, 2026-05-13):
    BDD-018 matrix row marked `pass` but "if the behavior changed,
    the next sim step dispatches through the new behavior" clause
    is parked under BDD-006 / Q2 (in-place behavior switching). The
    Estimator should not re-flag this every turn.
  - **BDD-102-vs-ALEMBIC-BYTES** (long-standing, since the BDD-102
    determinism slice, 2026-05-09): state.x snapshot stands in for
    Alembic bytes while FR-013 is blocked on Q5/Q6.
- **GENERATOR.md** absorbs in **stable harness gotchas** as a new
  entry: "Parallel implementation lockstep." When a math conversion
  (or any shared utility) has parallel copies in two TUs, any
  change to one MUST mirror to the other in the same commit. Grep
  both files before declaring done. Bitten on D-035 turn-30 fix-
  turn — the antipodal canonicalization had to land in both files
  simultaneously, and the Generator needed to verify.
- **ESTIMATOR.md** absorbs in **"Standing structural WARNINGs"**
  (already exists) by referencing PLANNER.md's new Standing
  constraints subsection as the canonical list. Estimator pulls
  from there; doesn't maintain its own parallel list.

### Lesson 3 — BLOCK-FIX-TURN cadence (turn-30 invocation)

**Source:** Turn 30 BLOCKed the D-035 slice; the user invoked the
BLOCK fix-turn manually (`/planner` → `/generator` → `/codex:rescue`,
stay on branch, `fix:` prefix). The `/slice` skill explicitly does
not handle this case (it halts in State D). The cadence has been in
GENERATOR.md ("Bug-fix turns in response to an Estimator BLOCK stay
on the same slice branch") and the `/slice` skill's State D handler,
but PLANNER.md doesn't have a procedural anchor for the planner's
side of a BLOCK fix-turn.

- **PLANNER.md** absorbs as a new **procedure step** ("Planning a
  BLOCK fix-turn"). When ESTIMATION.md's verdict is BLOCK and the
  user invokes `/planner` for the fix:
  - PLAN.md's Goal section names the specific BLOCK item being
    closed (verbatim or near-verbatim from ESTIMATION.md).
  - Scope is narrow: just the fix + regression-protection probes.
    No scope expansion beyond what closes the BLOCK.
  - No new D-NNN if the BLOCK is enforcement of an existing
    decision; add an addendum to the existing entry instead.
  - Commit prefix is `fix:` (per GENERATOR.md, fix-turn stays on
    the same branch).
- **GENERATOR.md** already covers "Bug-fix turns in response to an
  Estimator BLOCK stay on the same slice branch (commit prefix
  `fix:` rather than `add:`)." Augment with: "When the BLOCK is a
  missed edge case (e.g., D-035 turn-30 antipode), the bug-probe
  for the fix should be 'temporarily revert the fix, observe the
  same BLOCK signature' — proving the fix is what closes the BLOCK
  (not some other code-path change)."
- **ESTIMATOR.md** already covers BLOCK semantics. No new content
  needed; the existing "Standing parked failures vs new
  regressions" section handles the case where a BLOCK fix-turn
  closes a specific item.

### Lesson 4 — UTILITY-HELPER-EXIT-IS-CALLERS-DECISION (D-034 / CM-012)

**Source:** Turn-27 WARNING on `Program::loadShader` calling
`exit(1)` from inside `printLog()`, defeating the harness's
documented `programID == 0` SKIP semantic. D-034 + CM-012 captured
the lesson at the decision/mistake layers, but PLANNER doesn't have
a procedural prompt to design around it when planning new utility
helpers.

- **PLANNER.md** absorbs as a procedural note in **"Output
  discipline"** or near it: "When planning a slice that adds /
  refactors utility helpers (loaders, parsers, validators, etc.),
  ask whether the helper makes process-lifetime decisions
  (`exit`, `abort`, `terminate`). If so, the signature should make
  the decision visible (`[[noreturn]] void crashOnFailure(...)` or
  `bool tryLink(...)` returning failure). Implicit `exit(1)` inside
  an innocuously-named helper is a CM-012 trap; PLAN should call
  out the contract explicitly." Link to CM-012.
- **GENERATOR.md** absorbs in **build-time discoveries** as a
  reference: when a build-time discovery surfaces that a helper
  exits unilaterally, treat it as the CM-012 trap and either
  refactor (small, in-scope, ≤ ~30 lines for printLog/linkShader/
  load-overloads as in D-034) or hand back to Planner. Cite the
  fix-on-the-way vs hand-back decision tree.
- **ESTIMATOR.md** — no new content; the existing "stricter-than-
  spec assertions" and "SKIP-as-suppression is BLOCK-worthy"
  sections cover the symptom.

### Lesson 5 — STANDING-FOLD-IN candidates list (general housekeeping)

**Source:** Multiple turns have left small NOTE/WARNING items that
travel across slices without being foldable (turn-28 duplicated
inspector wiring, turn-29 glfwInit non-ref-counting, turn-30
parallel-impl lockstep). PROJECT_STATE.md already tracks shipped
slices and standing candidates; PLANNER's spec-substitution log
section is the closest existing home but it's specifically about
substitutions.

- **PLANNER.md** — already covered by Lesson 2's Standing
  constraints subsection. The new subsection is the canonical
  home; PROJECT_STATE.md's "Recent scope changes" continues to
  reference it.

### Other small additions

- **PLANNER.md** procedure step 7 ("Author assertions stricter than
  the BDD's literal wording when it costs almost nothing") gains a
  short example reference to D-035 turn-30: "Block 28's antipodal-
  equivalent comparison (accepts `a == b` OR `a == -b`) is wider
  than positive-w-only and catches the negative-w probe regression
  the original comparison would have missed."
- **GENERATOR.md** "stable harness gotchas" gains a short entry
  pointing at the file-array vs Quat-struct math duplication
  pattern: when a math conversion needs to be visible from two
  TUs that can't share the storage type (e.g., `Quat` is bare in
  main.cpp), the pattern is parallel implementations in a shared
  header with float-array signatures, lockstep-maintained until
  source-file split.

## Non-goals

- **NO code changes.** This is a docs-only slice. `src/`, `test/`,
  `include/`, all unchanged. Self-test count stays 51 → 51.
- **NO `.claude/skills/slice/SKILL.md` modification.** The
  orchestrator skill is meta; modifying it would change the cycle's
  behavior. Out of scope.
- **NO changes to `docs/specs/*` / `docs/ARCHITECTURE.md` /
  `docs/DECISIONS.md` / `docs/mistakes/*`.** Lessons land in role
  docs only; the source-of-truth references (D-034, D-035, CM-012)
  are already correct and unchanged.
- **NO TEST_MATRIX changes** — this slice doesn't touch any BDD.
- **NO new D-NNN, no new CM-NNN, no new BDD/FR.**
- **NO retroactive editing of past PLAN.md / CURRENT_WORK.md /
  RESUME.md / ESTIMATION.md content.** Those are slice-local
  records; the lessons distill into role docs, not edits to
  history.
- **NO Estimator-side write to `docs/roles/ESTIMATOR.md`.** Per
  ESTIMATOR.md's own write set, the Estimator cannot modify role
  docs. The Generator authors all three role-doc updates in this
  meta-slice (write set effectively extended to `docs/roles/*.md`
  for this slice only — documented in this PLAN's Course
  corrections).

## Todo

1. **Branch hygiene.** Already on `chore/role-doc-maintenance-pass`
   (off `main` at `282780e`). Commit prefix: `chore:` (docs-only
   maintenance pass).
2. **Generator writes `docs/roles/PLANNER.md` additions.**
   - New procedure step (number it 9 or 10 — pick the next
     unused; check existing list): **"Domain-edge probe
     enumeration for math-layer slices."** ~10–15 lines. Reference
     D-035 turn-30 as the canonical example.
   - New procedure step: **"Planning a BLOCK fix-turn."** ~8–10
     lines. Reference D-035 turn-30 fix-turn cadence.
   - Procedure step 7 ("stricter-than-spec assertions") gains a
     1–2 line reference to D-035 turn-30's antipodal-equivalent
     comparison as an example.
   - **Spec-substitution log section** gains a new sibling
     subsection: **"Standing constraints"** with 5 initial
     entries (PARALLEL-IMPL-LOCKSTEP, GLFWINIT-NON-REF-COUNTED,
     DUPLICATED-INSPECTOR-WIRING, BDD-018-BEHAVIOR-TAG-PARKED,
     BDD-102-vs-ALEMBIC-BYTES). Each entry: source slice/decision,
     constraint description, resolution trigger.
   - **Output discipline** gains a CM-012 procedural note about
     utility-helper exit semantics. ~5 lines.
3. **Generator writes `docs/roles/GENERATOR.md` additions.**
   - **Bug-probe discipline** gains an **edge-case probe
     enumeration** sub-section. ~10 lines. Reference D-035 turn-30.
   - **Stable harness gotchas** gains a new entry: **"Parallel
     implementation lockstep."** ~5 lines. Reference D-035 +
     PLANNER's Standing constraints subsection.
   - **Build-time discoveries** gains a CM-012 reference. ~3 lines.
   - **Bug-fix turns** entry (already present) gains a 2–3 line
     addendum about the "revert-the-fix, observe-the-same-signature"
     bug-probe pattern for BLOCK fix-turns.
4. **Generator writes `docs/roles/ESTIMATOR.md` additions.**
   - **"What to flag, what to skip"** gains a new flag-able item:
     **"Missing-edge probe coverage."** ~5 lines. Reference D-035
     turn-30.
   - **Procedure step 4** ("Check the test matrix") gains a 2–3
     line note about probe-domain-coverage for math-layer slices.
   - **Standing structural WARNINGs** section gains a 2–3 line
     pointer to PLANNER's new Standing constraints subsection as
     the canonical list (Estimator pulls from there; doesn't
     maintain its own parallel list).
5. **Generator updates `.agent/PROJECT_STATE.md`** with a "Recent
   scope changes" entry for the 2026-05-13 role-doc maintenance
   pass (2–3 line summary citing the 5 lessons folded + the
   commit-prefix `chore:`).
6. **Verify no code/test changes.** `git diff --stat src/ test/
   include/` should show empty diff. If any file under those paths
   is touched, STOP and hand back.
7. **`./scripts/verify-light.sh`** as cheap sanity insurance —
   docs-only changes shouldn't affect the build or doctests, but
   running once confirms nothing slipped. Expect doctest 159/159 +
   1120/1120 SUCCESS unchanged.
8. **Self-test optional.** `./src/ysim --self-test` should still
   produce 51/51 PASS (no source changes). One run is enough; no
   need for 5x determinism since nothing executable changed.
9. **No bug-probe step.** The "test" for a docs-only slice is the
   Estimator's diff-reading pass. The Generator should write each
   addition so that re-reading it 6 months later, a fresh planner /
   generator / estimator session can apply the discipline without
   needing to dig into the historical slice context.
10. **Generator updates `.agent/CURRENT_WORK.md`** with: file in
    flight (none — slice complete), how far (all 5 lessons folded
    across 3 role docs + PROJECT_STATE entry), what's tested (no
    code changes; verify-light.sh green; visual review of role-doc
    diffs is the gate), what's next (Estimator review of diffs).
11. **Generator updates `.agent/RESUME.md`** with: must-remember
    (the 5 lesson labels + the Standing constraints subsection
    location in PLANNER.md + the special-case write-set extension
    for this meta-slice), last decisions + why (no new D-NNN; the
    lessons distill prior decisions into procedural guidance), next
    step (Estimator review).

## Course corrections

- **Write-set extension for this meta-slice.** Per the established
  convention (commit `4217619`, the prior role-doc maintenance
  pass), the Generator's write set is effectively extended to
  `docs/roles/{PLANNER,GENERATOR,ESTIMATOR}.md` for this slice
  only. The Estimator cannot write to its own role doc (per
  ESTIMATOR.md's strict write set), so the Generator authors all
  three docs based on this PLAN. The Estimator reviews the diff
  for sense, citing whether each addition matches the lesson it
  claims to capture.
- **`feedback_make_means_add_new` rule.** The user's slice brief
  used "fold discipline learnings" (modification verb) — in-place
  edits to existing role docs are correct. Each role doc gains
  new sections / sub-sections / entries; existing content stays
  unchanged except for the small procedure-step-7 inline example
  in PLANNER.md.
- **Pacing.** This is the second role-doc maintenance pass
  (previous: commit `4217619` after turn 16, 2026-05-10). Pacing is
  ~14 cycles per pass. Future passes can wait for similar
  accumulation (~10–15 cycles of discipline learning).
- **Standing constraints list canonicalizes.** Future planners /
  generators / estimators should reference PLANNER.md's Standing
  constraints subsection as the authoritative list of long-lived
  documented limitations. The Estimator's "Standing structural
  WARNINGs" section in ESTIMATOR.md points at it; the Generator's
  "stable harness gotchas" entry on parallel-impl lockstep points
  at it; PROJECT_STATE.md's "Recent scope changes" can reference
  individual entries by their label (PARALLEL-IMPL-LOCKSTEP, etc.)
  going forward.
- **Pace check on lessons.** 5 lessons across 14 cycles is ~1
  lesson per 3 cycles — healthy. If a future maintenance pass
  surfaces fewer than 3 lessons or more than 10, that's a signal
  to either delay (too few = no accumulated wisdom) or batch the
  current state into a smaller intermediate doc (too many = too
  much to fold cleanly).

Expected matrix delta: none.
Expected self-test count: 51 → 51 (no code changes).
Expected verify.sh: exits 0 unchanged.
Expected Estimator verdict: NOTE (docs-only; sense-check pass).
Possible NOTE items: (i) the Standing constraints subsection
might want a separate canonical home (its own `docs/standing-constraints.md`
file?) — defer until the list grows beyond ~10 entries; (ii) the
"edge-case probe enumeration" sub-section in GENERATOR.md could
benefit from a worked example beyond D-035 — defer to next
maintenance pass when more examples accumulate.
