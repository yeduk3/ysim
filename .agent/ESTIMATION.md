# Estimation — 2026-05-14 turn 32 (BLOCK fix-turn)

Status: UPDATED

## Verdict
NOTE

## BLOCK
- none

## WARNING
- none

## NOTE
- `changeBehavior` Rigid case comment says "BehaviorParams variant has no Rigid alternative; the existing variant value persists (harmless — the simulator's dispatch reads behaviorType, not the variant's alternative, for Rigid-tagged meshes)." This is correct and documented in D-036, but the implicit assumption (the leftover variant value is always a noop) is worth a future hardening note: if a future Rigid-dispatch path in `applyEnvironmentForces` ever queries the variant via `std::get` / `std::holds_alternative`, it would see a stale Float or Cloth variant. Acceptable today under BDD-006-RIGID-DISPATCH-PARKED; worth flagging for slice B-3's author. No action required now.
- `meshBehaviors` pre-state assertion in Clause 6 is deliberately omitted (CURRENT_WORK.md: "rebuildMeshKinds size-match short-circuit leaks residuals across resetScene"). The comment in the harness is clear and the scope-out is justified, but a future test-maintenance pass could add a `shBroadPhase.meshBehaviors.ptr == nullptr` check that confirms the array is unpopulated before the 1-frame pump, tightening the clause. Informational only.

## Test matrix delta
- BDD-006: pass (test-address column updated with clause 5 + clause 6 pass labels + fix-turn bug-probe summaries)

## Verify output (summary)
`ysim --self-test` ran 57/57 PASS across 5 consecutive deterministic runs on the macOS dev host. Block 29 clauses 5 (Rigid save→load round-trip) and 6 (changeBehavior cache-sync) both pass, and all 4 existing clauses (1–4) remain green. `verify-light.sh` reported 159/159 doctest SUCCESS (scene_io_test suite) + 1120/1120 SUCCESS (unit test suite); BDD-016 stays green with the Elastic substitution. All 4 fix-turn bug-probes were exercised and each revert produced the expected loud FAIL: (a) restoring `"Rigid"` to `isReservedBehavior` causes Clause 5 to fail with `loadErr=behavior 'Rigid' not available in this build`; (b) disabling the loadScene Rigid branch causes Clause 5's post-load tag to silently demote to Float; (c) commenting out `syncBroadPhaseCaches` in the TriangularCloth accept case causes Clause 6 to fail with both caches stale; (d) restoring `"Rigid"` in BDD-016 without reverting (a) causes the doctest assertion on `r.ok` / `r.error.message` to fail, confirming the test-update is load-bearing. The SKIP path on Metal-less hosts is unaffected — the two fix-turn clauses sit inside Block 29 which is gated on `MetalGlobalContext::getDevice()` returning non-null; on those hosts the entire Block 29 suite is skipped (D-012), not failed.
