# Resume — Environment Forces Slice

## Must remember

- **Branch:** `feat/environment-forces` (off `main`, after `feat/primitive-creation` was fast-forwarded into `main`).
- **`mesh.externalForces.externalForces` is the canonical force input** for cloth kernels (`compute_tri_spring_forces`, `compute_cloth_grid_forces_fast`). Before this slice it was a placeholder buffer that nobody allocated; now it's a slice of `packedMeshData.externalForces`, allocated and zeroed during `Scene::pack()` and overwritten each frame by `Simulator::applyEnvironmentForces` (which runs once per outer frame, **before** the substep loop). If a future slice wants to add a *different* per-mesh force (e.g. user-applied push-pull), don't compete with this fill — write into `mesh.state.f` directly, or extend `applyEnvironmentForces` so the buffer remains the sole source of truth.
- **Float is force-exempt by construction**, not by integration cancellation. `applyEnvironmentForces` zeroes the buffer for `BehaviorType::Float`. `BDD-009`'s strict-equality clause leans on this — do not introduce a path that adds gravity then subtracts it for Float; the buffer stays exact-zero.
- **Wind is force-per-particle (no mass scaling)**, gravity is force-per-particle scaled by mass. This matches FR-011 (gravity vector applied to all non-Float) and FR-012 (wind force vector for wind-susceptible behaviors). PRD §3.3 hints at a v2 reformulation of wind as an air-velocity field; the schema is forward-compatible.
- **Sim-step correctness is parked behind Q-D.** This slice's `BDD-009/011/012` rows are `warning` — data layer round-trips, but "object accumulates velocity in +x" needs a Metal-backed harness. Same convention as `BDD-015`. PROJECT_STATE explicitly tells the Estimator to expect WARNING, not BLOCK.

## Last decisions + why

No new `DECISIONS.md` entries this slice. The change is the consumer side of D-006 (which already established `Scene::environment` as the in-memory source of truth) — no non-obvious tradeoff to record.

## Next step you were about to take

Slice complete. The next concrete step is the **Estimator's** turn — running `./scripts/verify.sh` and judging whether to merge or send back. After that, per `PROJECT_STATE.md` "Next milestone", the priority is the **test-harness slice** (Metal-backed sim) — concrete motivation now stacked: persistence app-level WARNING, CM-002, CM-003, and now BDD-009/011/012 sim-step clauses. Closing it forces Q-D resolution (CPU backend reference vs. headless-Metal harness) and a `MeshGL` refactor out of `mesh.initialize()`.

See `.agent/PLAN.md` and `.agent/CURRENT_WORK.md` for full plan and progress.
