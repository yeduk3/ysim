# Resume — BDD-006 BLOCK fix-turn (D-036 turn-32 addendum)

## Must remember

- **Branch / worktree**: working in `.claude/worktrees/fixturn-bdd-006` (branch `fixturn-bdd-006`, branched off `037d83f` with slice WIP patched in). The primary checkout's `feat/bdd-006-behavior-assignment-ui` still carries the pre-fix-turn slice WIP uncommitted; merge requires unstaging the primary's WIP first. Commit prefix `fix:` per BLOCK-fix-turn cadence; the close-out lands as ONE `fix:` commit containing both the original D-036 slice content + the turn-32 fix-turn changes (D-035 turn-30 precedent — commit `1f21335 fix: ...`).
- **BLOCK closure (option b — load-side accept Rigid)**: `scene_format::isReservedBehavior` narrowed `{Rigid, Elastic, Fluid, Generator}` → `{Elastic, Fluid, Generator}`; `isKnownBehavior` widened to include `"Rigid"`; new `else if (o.behavior.type == "Rigid")` branch in `Simulator::loadScene` sets `btype = Rigid + bparams = FloatBehaviorParams<PR>{}` (FloatBehaviorParams placeholder satisfies the variant; Rigid is tag-set only per D-036 invariant). Symmetric with `Simulator::changeBehavior` (already accepts Rigid). Round-trip preserved; forward-compat with B-3 (which widens dispatch, not persistence).
- **WARNING closure (cache-sync in changeBehavior)**: `syncBroadPhaseCaches(BehaviorType bt)` lambda inside `changeBehavior` writes `objTrees[idx].objBehavior` AND `shBroadPhase.meshBehaviors[idx]` directly when caches are allocated (gated on `.ptr` / size). Called at the tail of each accept case. D-026's `builtForLifetimeId` invariant is preserved — lifetime tracking is unchanged.
- **`test/scene_io_test.cpp` BDD-016 swap**: example changed Rigid → Elastic so the test's intent (reserved-not-shipped rejection on load) is preserved. The new error-message find is `"Elastic"`.
- **Block 29 grew 4 → 6 clauses**. Clause 5 = Rigid round-trip; Clause 6 = changeBehavior cache-sync. Self-test count 55 → 57.
- **Clause 6 pre-state asymmetry**: pre-state asserts `objTrees[0].objBehavior == Float` (D-026's skip-rebuild gate re-populates on `!= Float || lifetimeId mismatch`) but does NOT assert `meshBehaviors[0] == Float` because `rebuildMeshKinds()` short-circuits on size-match, leaking residuals from prior clauses (e.g., Clause 3 leaves `meshBehaviors[0] = FastGridCloth = 1`). The post-state check is what closes the WARNING — bug-probe (c) confirms it's load-bearing.
- **All 4 bug-probes verified, load-bearing.** (a) scene_format reserved-list revert → Clause 5 FAILs with turn-32 BLOCK signature. (b) loadScene Rigid-branch revert → Clause 5 `loadOk=1 postTagOk=0`. (c) cache-sync revert → Clause 6 `postObjBehaviorOk=0 postMeshBehaviorsOk=0`. (d) BDD-016 test-update revert → doctest FAILs (3 assertions). All restored.
- **No new D-NNN, no new CM-NNN.** D-036 turn-32 fix-turn addendum captures both invariants. BDD-006-RIGID-DISPATCH-PARKED standing constraint narrows (persistence done; only integrator dispatch parked until B-3).
- **Manual GUI gate post-fix-turn**: cube → Rigid → save → load → confirm Rigid tag preserved; Float→Cloth switch reacts in the same frame (no one-frame collision-filter lag).

## Last decisions + why

- **Option (b) over (a) over (c) for BLOCK**: load-side accept is symmetric with runtime, preserves the user's behavior choice across save/load (no silent data loss), forward-compatible with B-3.
- **Cache-sync via in-method lambda over a helper method**: keeps the sync localized to changeBehavior so future readers see the invariant alongside the mutation. The 4× DRY via lambda beats inline duplication.
- **Clause 6 drops meshBehaviors pre-check**: stale-cache leak across `resetScene` boundaries is a pre-existing condition unrelated to this fix-turn; would be scope expansion to fix `rebuildMeshKinds`. Post-check alone closes the WARNING and bug-probe (c) keeps the assertion load-bearing.
- **D-036 addendum over new D-NNN**: this fix-turn enforces existing D-036 invariants (persistence symmetry; runtime cache consistency) that the original entry under-specified. New D-NNN reserved for new architectural patterns.

## Next step you were about to take

Fix-turn complete. Next concrete step: **user's manual GUI test** (cube → Rigid → save → load → confirm tag preserved) then the **Estimator's turn 33** (Codex). `./scripts/verify.sh` should exit 0 with **57/57** self-test PASS on macOS. Expected verdict: NOTE or WARNING. Possible items:

- (i) D-036 turn-32 fix-turn addendum prose is long — could trim for CHANGELOG-style. NOTE-able.
- (ii) Clause 6's pre-state asymmetry (meshBehaviors deliberately not asserted) — informational. The source comment explains.
- (iii) BDD-006-RIGID-DISPATCH-PARKED narrowing — first appearance of the narrowed text.

After this fix-turn lands + Estimator approves + `/slice` close-out merges:

- **C-1 (FlatBuffers mesh-cache writer)** — next per agreed slice order.
- **B-1 (Rigid physics backend contract + Null impl)** — after C-1.
- **B-2 (Bullet impl)** — after B-1.
- **B-3 (Wire Rigid behavior tag into Bullet)** — retires BDD-006-RIGID-DISPATCH-PARKED + BDD-018-BEHAVIOR-TAG-PARKED.

See `.agent/PLAN.md` and `.agent/CURRENT_WORK.md` for full plan and progress.
