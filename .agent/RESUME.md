# Resume — Refit-After-Edit Fix-turn (Estimator turn-17 WARNING closed via D-023)

## Must remember

- **Branch:** `fix/refit-after-edit` (off `main` at `9c8da75`).
- **D-023 is the canonical invariant.** Direct `state.x` mutations outside the integrator MUST call `collisionPipeline.broadPhase.refit()` so the click-pick BVH reflects the new pose immediately on a paused sim. Currently `translateObject` and `rotateObject` are the only such mutators; future write paths (e.g., a "set absolute position" API, future rigid-body teleport) must follow the same pattern.
- **`refit()` is the canonical invalidator.** Don't reach for `tree.build()` directly or fiddle with `objTrees`. `BroadPhase::refit()` at `src/main.cpp:3961` covers both per-mesh refit (line 3962-3963) AND SCENE-level rebuild via `tree.build(-1, positions, indices)` (line 3979) in one call.
- **Block 16's 45°-Z rotation in clause (b) is deliberate.** A 90°-Z rotation leaves an axis-aligned cube's AABB at ±0.25 (rotational symmetry), which would make the assertion fail to distinguish "refit happened" from "refit didn't happen". 45° grows AABB by √2 (to ±0.354), which is observable. Witness ray at x=0.30 is between the two — outside pre-rotate, inside post-rotate.
- **Block 16 stricter-than-spec assertions.** Clause (a) checks BOTH new-pose-hit AND old-pose-miss; without the both-checks form, a refit that copies state.x but leaves stale leaf AABBs would still pass. Per PLANNER.md step 7's discipline.
- **Bug-probe-verified during build:** commenting out the `refit()` call in `translateObject` makes clause (a) FAIL with the exact diagnostic the assertion claims; restoring flips back to PASS.
- **Inspector ergonomics for rotation** still uses raw `InputFloat4("Quat (w,x,y,z)")`. Euler/axis-angle UX deferred (FR-004 Notes territory, future slice).

## Last decisions + why

- **D-023 — refit-after-direct-state.x-mutation.** Closes Estimator turn-17 WARNING. Rejected: requiring GUI to call `sim.update()` after edits (heavy + breaks paused-sim semantics); refit-on-click read-side (asymmetric, leaves visualization stale); dirty-flag + lazy refit (state machine adds a third concern for no gain at v1's mesh counts). The write-side refit is the symmetric extension to D-014 / D-015 / D-021's edit-path patterns.

## Next step you were about to take

Slice complete. Next concrete step is the **Estimator's** turn — `./scripts/verify.sh` should exit 0 with **33/33** self-test PASS lines. Expected verdict: NOTE level.

After this lands, planner-tracked candidates per `PROJECT_STATE.md`:

- **Rotate pack-roundtrip slice (FR-004 follow-up).** The rotate analog of D-015's translate write-back. Planner design call about whether `Simulator::initialize()` should auto-call `applyPendingMaterials()` (currently only `loadScene` flow does). RESUME has been carrying this for 2 slices now — promotion candidate next.
- **CM-008 production-side fix** — `BroadPhase::build`'s Float-mesh skip robust against scene-swap-at-same-count cases. Theoretical concern in v1.
- **Inspector ergonomics for rotation** — Euler / axis-angle input affordances per FR-004 Notes.
- **BDD-018 inspector live-edit propagation** — implementation exists; mechanization needs ImGui-side simulation or callable abstraction.
- **Material editing UI (FR-005 / BDD-005)** — needs PBR preview shader. Large.
- **Behavior assignment UI (FR-006 / BDD-006)** — Q2 still open.
- **Rigid body slice (FR-008 / BDD-008)** — blocked on Q4. When this lands: D-014/D-015 + D-018 + D-019/D-022 + D-020 + D-021 + D-023 all apply.
- **Alembic export slice (FR-013 / BDD-013)** — blocked on Q5 + Q6.

See `.agent/PLAN.md` and `.agent/CURRENT_WORK.md` for full plan and progress.
