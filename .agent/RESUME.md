# Resume — BDD-010 Collision Detection Slice (BDD-010 promoted to pass)

## Must remember

- **Branch:** `feat/bdd-010-collision-detected` (off `main` at `089279b`).
- **Block 13 cloth at `y = -1.0` (co-located with ground), NOT `y = -0.99`.** At t=0 with zero velocity, the broad phase's `enlargeTrajectory(system.subh)` inflation is zero; both meshes' AABBs are essentially flat planes that don't overlap unless the cloth's particles land within the ground's flat AABB. Co-locating at the same y solves it. If a future slice changes the broad-phase strategy (e.g., adds non-zero default AABB padding), this position can move back up — but Block 13 currently relies on co-location.
- **`cumulativeNarrowCollisions` reset is explicit.** `resetScene()` does NOT clear it (it's a static member of `PackedCollisionData`). Block 13 sets it to 0 before each `sim.update()`. Same pattern Block 6 (BDD-007) uses.
- **(A, B) distinct-pair assertion uses `objPair.query != objPair.target`.** This is also the structural safety net against self-collision contributing to the count, even with `enableSelfCollisions = false` already filtering at the broad phase.
- **Block 12 BDD-004 fold-in:** `r1_post_load` unit-norm is now asserted **before** `quatNormalize(dq2 * r1_post_load)`. The re-normalize can absorb load-side drift; the explicit pre-multiply check catches it. Estimator turn-13 WARNING closed.
- **No new D-NNN this slice.** Block 13 is mechanization-only; no architectural decision was made.

## Last decisions + why

- (No new decisions this slice — pure test mechanization + small WARNING fold-in.)

## Next step you were about to take

Slice complete. Next concrete step is the **Estimator's** turn — `./scripts/verify.sh` should exit 0 with **27/27** self-test PASS lines. Expected verdict: NOTE level.

After this lands, planner-tracked candidates per `PROJECT_STATE.md`:

- **`Simulator::rotateObject(meshId, deltaQuat)` + inspector wiring (FR-004 UI side)** — pairs with D-014's translateObject. Has open design questions (pivot point, cloth-in-flight semantics) that BDD-004 didn't need to answer because the spec is purely data-layer.
- **BDD-017 Ray-pick** — implementation exists; mechanization needs ray-pick logic extracted into a callable function. Medium-sized.
- **Material editing UI (FR-005 / BDD-005)** — needs PBR preview shader. Large.
- **Behavior assignment UI (FR-006 / BDD-006)** — Q2 still open (cloth UX surface).
- **Rigid body slice (FR-008 / BDD-008)** — blocked on Q4. When this lands it adds a fifth initializer subtype; D-015's three-site cascade applies AND D-018's seed-from-mesh-id invariant applies. D-019's canonical Quat math is also load-bearing for any rotation a rigid body needs.
- **Alembic export slice (FR-013 / BDD-013)** — blocked on Q5 + Q6. When this lands, BDD-102 mechanization can extend to compare Alembic bytes too (closes the standing turn-12 WARNING).

See `.agent/PLAN.md` and `.agent/CURRENT_WORK.md` for full plan and progress.
