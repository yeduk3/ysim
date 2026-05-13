# Resume — Role-doc maintenance pass (slices 17–30 discipline learnings)

## Must remember

- **Branch:** `chore/role-doc-maintenance-pass` (off `main` at `282780e`). Commit prefix: `chore:`.
- **5 lessons folded across 3 role docs + PROJECT_STATE entry. No code/test changes.** Special-case write-set extension: Generator authored all 3 role docs (same convention as commit `4217619`, the prior maintenance pass).
- **PLANNER.md** gained: procedure step 9 (Domain-edge probe enumeration), procedure step 10 (Planning a BLOCK fix-turn), procedure step 7 augmentation (Block 28 antipodal-equivalent example), new "Standing constraints" subsection (5 entries: PARALLEL-IMPL-LOCKSTEP, GLFWINIT-NON-REF-COUNTED, DUPLICATED-INSPECTOR-WIRING, BDD-018-BEHAVIOR-TAG-PARKED, BDD-102-vs-ALEMBIC-BYTES), Output discipline CM-012 note.
- **GENERATOR.md** gained: bug-probe → edge-case probe enumeration sub-section, stable harness gotchas → parallel-impl-lockstep entry, build-time discoveries → CM-012 trap entry, new "Bug-fix turns in response to an Estimator BLOCK" sub-section (revert-the-fix bug-probe pattern).
- **ESTIMATOR.md** gained: "What to flag" → "Missing-edge probe coverage" item, procedure step 4 math-layer addendum, "Standing structural WARNINGs" pointer to PLANNER's Standing constraints subsection (canonicalizes — Estimator pulls from PLANNER's list rather than maintaining a parallel).
- **PROJECT_STATE.md** gained: "Recent scope changes" entry for 2026-05-13 role-doc maintenance pass.
- **No bug-probe step** for docs-only slice. Lessons themselves were forensically derived from real incidents (D-035 turn-30, D-034 turn-27 + turn-29, turn-28 NOTE, BDD-102 long-standing).
- **Pacing**: 14 cycles since the last maintenance pass (`4217619` after turn 16). Future passes expected every ~10–15 cycles when 3–10 new discipline lessons have accumulated.

## Last decisions + why

- **PARALLEL-IMPL-LOCKSTEP, etc. canonicalized into PLANNER's "Standing constraints" subsection** rather than scattering across multiple role docs. Estimator's "Standing structural WARNINGs" section points at it; Generator's parallel-impl gotcha entry points at it; PROJECT_STATE can reference entries by label. Single canonical home prevents drift.
- **No `docs/standing-constraints.md` separate file yet.** 5 entries fit comfortably inside PLANNER.md. If the list grows to ~10+ entries, a future maintenance pass can extract it.
- **No `.claude/skills/slice/SKILL.md` edits.** The orchestrator skill is meta — modifying it changes cycle behavior, out of scope for a role-doc pass.
- **No new D-NNN / CM-NNN.** Pure procedural distillation of prior decisions/incidents. The source-of-truth references (D-034 / D-035 / CM-012) stay unchanged.

## Next step you were about to take

Slice complete. Next concrete step is the **Estimator's** turn (Codex). `./scripts/verify.sh` should exit 0 with **51/51** self-test PASS on macOS dev host (single run; no determinism stress needed). On the Estimator's Linux container the Metal SKIP path returns 0 unchanged. Expected verdict: NOTE (docs-only sense-check). Possible items:

- (i) "Standing constraints" subsection might warrant its own canonical home file when it grows; current 5 entries fit comfortably.
- (ii) GENERATOR's "Edge-case probe enumeration" could use a second worked example — defer to next pass.
- (iii) The pacing note ("~14 cycles per maintenance pass") is informational; may surface as a NOTE about scope-of-pass meta-comment.
- (iv) The 5 lessons reference real D-034 / D-035 / CM-012 incidents; any future doc-prune that drops those references would orphan the role-doc text — current setup is correct.

After this lands NOTE-level, planner-tracked candidates per `PROJECT_STATE.md`:

- **Behavior assignment UI (FR-006 / BDD-006)** — Q2 still open.
- **Rigid body (FR-008)** — Q4 blocked.
- **Alembic export (FR-013)** — Q5+Q6 blocked.
- **Source-file split slice** — still user-deferred; retires PARALLEL-IMPL-LOCKSTEP + DUPLICATED-INSPECTOR-WIRING standing constraints when it ships.
- **Strict-D-029-column bench slice** — conditional.

See `.agent/PLAN.md` and `.agent/CURRENT_WORK.md` for full plan and progress.
