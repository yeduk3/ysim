# Resume — BDD-017 Coverage Fix-turn (closes turn-15 WARNINGs + surprises a production fix)

## Must remember

- **Branch:** `fix/bdd-017-coverage` (off `main` at `3e5f6a4`).
- **D-020 — `BVH::queryClickRay` leaf branch must `return`.** Without the return, the leaf falls through to `if(node.childB > 0) queryClickRay(ray, tree[node.childB])` and treats the primitive id as a node index. Recursion explodes within the per-mesh tree, fills the 4096-entry `clickRayCollisions` buffer, starves later objTrees in the SCENE-level outer loop. Production wasn't visibly broken because smallest-tmin walk happened to pick the right id even among spurious entries; only multi-object click queries (e.g., harness's both-cubes-on-line-of-sight) exposed it.
- **CM-009 — BVH walk leaf-vs-interior semantic mix.** Any future BVH variant that copies the queryClickRay loop shape (write-then-fall-through-to-recurse) inherits the same bug. The discipline: every leaf branch must `return;` explicitly, OR restructure as `if (interior) recurse; else leaf-action;`. Buffer caps only mask the symptom.
- **Block 14 both-cubes assertion was the right test.** It made the bug surface as a deterministic harness FAIL (`sawFront=1, sawBack=0`). Production's smallest-tmin walk would have stayed correct indefinitely without the assertion. This is exactly the value of mechanizing BDDs with stricter assertions than the BDD's literal wording requires.
- **`sim.selectedObj` is now wired into Block 14's assertions.** Both clauses write `pickClosest()` result into `sim.selectedObj` and assert from there. Catches a callback-side regression that drops the assignment but leaves the BVH intact. Mirrors production at `src/main.cpp:6718`.
- **`numHits >= 2` stays as a coarse pre-condition gate.** Don't drop it — it emits a useful diagnostic on the pathological "BVH found nothing" case.
- **Pass labels unchanged** so matrix-row test address still reads correctly.

## Last decisions + why

- **D-020** — `return;` from leaf in `BVH::queryClickRay`. Rejected: aggressive buffer-overflow gating at the SCENE-level outer loop (papers over the symptom; doesn't fix the unbounded recursion). Rejected: bumping `approxColsPerRay` from 4096 (the explosion has no fixed termination — bigger buffer just delays the cap hit). The minimal fix is the explicit `return`.

## Next step you were about to take

Slice complete. Next concrete step is the **Estimator's** turn — `./scripts/verify.sh` should exit 0 with **29/29** self-test PASS lines. Expected verdict: NOTE level (turn-15 WARNINGs closed; D-020 production fix is in the diff and is bug-probe-verified by Block 14's both-cubes assertion).

After this lands, planner-tracked candidates per `PROJECT_STATE.md`:

- **CM-008 production-side fix** — `BroadPhase::build`'s Float-mesh skip robust against scene-swap-at-same-count cases. Theoretical concern in v1; defer unless a real flow surfaces.
- **`Simulator::rotateObject(meshId, deltaQuat)` + inspector wiring (FR-004 UI side)** — pairs with D-014's translateObject + D-019's canonical Quat math. Has open design questions (pivot point, cloth-in-flight, persistence side-table coordination with `applyPendingMaterials`).
- **BDD-018 inspector live-edit propagation** — implementation exists; mechanization needs ImGui-side simulation or a callable abstraction.
- **Material editing UI (FR-005 / BDD-005)** — needs PBR preview shader. Large.
- **Behavior assignment UI (FR-006 / BDD-006)** — Q2 still open.
- **Rigid body slice (FR-008 / BDD-008)** — blocked on Q4. When this lands: D-015 three-site cascade + D-018 seed-from-mesh-id + D-019 Quat math + D-020 BVH leaf-return invariant all apply.
- **Alembic export slice (FR-013 / BDD-013)** — blocked on Q5 + Q6.

See `.agent/PLAN.md` and `.agent/CURRENT_WORK.md` for full plan and progress.
