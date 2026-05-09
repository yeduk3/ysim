# Resume — BDD-017 Ray-Pick Slice (BDD-017 promoted to pass)

## Must remember

- **Branch:** `feat/bdd-017-ray-pick` (off `main` at `ea6cc48`).
- **CM-008 — `objTrees.clear()` before harness scene-swaps that keep numMeshes constant.** `BroadPhase::build` (~line 3907) skips rebuild for `objTrees[i].tree.ptr && objBehavior == Float`. Production-correct (Float meshes don't change), but harness's `resetScene` + new-mesh pattern with same numMeshes count reuses stale slots. Block 14's two clauses both call `sim.collisionPipeline.broadPhase.objTrees.clear();` before `sim.initialize()` to force the size-mismatch realloc branch. Future Block authors that create distinct Float scenes back-to-back must do the same. Production-side fix is theoretical until a "scene swap at same count" flow ships outside the harness.
- **`sim.update()` after `sim.initialize()` is required for click-ray.** Without it, BVH leaf AABBs aren't refit, `queryClickRay` returns 0 hits. Production calls `update()` every frame before the click callback fires, so the issue doesn't surface there.
- **Block 14's pickClosest mirrors production at `src/main.cpp:6588-6596`.** Production walks `clickRayCollisions[0..numClickRayCollisions[0])` for smallest `tmin` and writes the result to `simulator->selectedObj`. The harness reads the closest id directly (no `selectedObj` mutation) — that's the load-bearing semantic the BDD claims, while inspector display is BDD-018's concern.
- **Spec-substitution: world-space ray instead of camera-unprojected ray.** Harness has no GLFW/ImGui; the BDD's "click on screen position" is mechanized as `Ray{origin, dir}` constructed directly in world space. Pass label notes the substitution.
- **Mesh ids are stable per addGeneralMesh call order.** First addCube → id 0, second addCube → id 1. D-015's `numMeshes`-pre-call invariant is the canonical source.
- **No new D-NNN this slice.** Pure mechanization.

## Last decisions + why

- (No new architectural decisions this slice — pure test mechanization. CM-008 records the BVH rebuild-skip trap for future maintainers.)

## Next step you were about to take

Slice complete. Next concrete step is the **Estimator's** turn — `./scripts/verify.sh` should exit 0 with **29/29** self-test PASS lines. Expected verdict: NOTE level.

After this lands, planner-tracked candidates per `PROJECT_STATE.md`:

- **CM-008 production-side fix** — make `BroadPhase::build`'s skip condition robust against scene-swap-at-same-count cases (e.g., gate skip on objTree's stored `mesh.id` or content hash matching the new mesh, or expose an explicit `invalidate()` API). Theoretical concern in v1 since users don't normally swap scenes outside loadScene. Defer.
- **`Simulator::rotateObject(meshId, deltaQuat)` + inspector wiring (FR-004 UI side)** — pairs with D-014's translateObject + D-019's canonical Quat math. Has open design questions (pivot point, cloth-in-flight, persistence side-table coordination with `applyPendingMaterials`).
- **BDD-018 inspector live-edit propagation** — implementation exists; mechanization needs ImGui-side simulation or a callable abstraction.
- **Material editing UI (FR-005 / BDD-005)** — needs PBR preview shader. Large.
- **Behavior assignment UI (FR-006 / BDD-006)** — Q2 still open.
- **Rigid body slice (FR-008 / BDD-008)** — blocked on Q4. When this lands: D-015 three-site cascade + D-018 seed-from-mesh-id + D-019 Quat math all apply.
- **Alembic export slice (FR-013 / BDD-013)** — blocked on Q5 + Q6. Closes BDD-102's standing "state.x vs Alembic-bytes substitution" WARNING.

See `.agent/PLAN.md` and `.agent/CURRENT_WORK.md` for full plan and progress.
