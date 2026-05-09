# Resume — BDD-004 Quaternion Slice (BDD-004 promoted to pass)

## Must remember

- **Branch:** `feat/bdd-004-rotation` (off `main` at `992b658`).
- **D-019 is canonical Quat math.** Three free functions next to `struct Quat` (~line 1554): `operator*` (Hamilton product), `quatNorm`, `quatNormalize`. Convention: `a * b = apply b first, then a`. Future rotation consumers (FR-004 inspector wiring, FR-008 rigid body, eventual renderer-side rotation) **must use these** rather than reimplementing — drift between consumers is the failure mode this avoids.
- **`pendingRotations` → `applyPendingMaterials` is load-side.** Despite the function name, `applyPendingMaterials()` also writes `pendingRotations[meshId]` into `mesh.rotationQuat`. Block 12's saveScene/loadScene flow calls it after `sim.initialize()`. Forgetting this would make the round-trip read identity instead of the saved rotation.
- **`Quat` aggregate-init shape stays intact.** The struct is `{w, x, y, z}` POD; on-disk schema relies on this order. Adding member fields (e.g., a precomputed norm cache) would break the aggregate. If a future consumer needs caching, do it externally.
- **Bug-probe-verified:** `meshAfterLoad->rotationQuat.w += 0.01f` after reload makes Block 12 FAIL with the orientation-drift diagnostic.
- **Tolerance 1e-5 for both orientation and norm.** Three unit-quaternion compositions accumulate at most ~3 ULPs of float drift; 1e-5 is comfortably above that floor.

## Last decisions + why

- **D-019** — canonical Quat math via free functions. Rejected: member functions (mixes data + math), reverse Hamilton convention (would surprise every consumer), defer-until-needed (BDD-004 needs it now; future consumers would re-implement and drift), GLM dependency (overkill for 30 lines). The convention is documented inline next to the operator so `R₂ * R₁` reads correctly without re-reading DECISIONS.

## Next step you were about to take

Slice complete. Next concrete step is the **Estimator's** turn — `./scripts/verify.sh` should exit 0 with **25/25** self-test PASS lines. Expected verdict: NOTE level.

After this lands, planner-tracked candidates per `PROJECT_STATE.md`:

- **`Simulator::rotateObject(meshId, deltaQuat)` + inspector wiring (FR-004 UI side)** — pairs with D-014's translateObject. Has open design questions (pivot point, cloth-in-flight semantics) that BDD-004 didn't need to answer because the spec is purely data-layer.
- **BDD-010 Collision detected between simulated objects** — small Block 13 mechanization, two clauses (positive/negative AABB-overlap cases).
- **BDD-017 Ray-pick** — implementation exists; mechanization needs ray-pick logic extracted into a callable function.
- **Material editing UI (FR-005 / BDD-005)** — needs PBR preview shader.
- **Behavior assignment UI (FR-006 / BDD-006)** — Q2 still open (cloth UX surface).
- **Rigid body slice (FR-008 / BDD-008)** — blocked on Q4. When this lands it adds a fifth initializer subtype; D-015's three-site cascade applies AND D-018's seed-from-mesh-id invariant applies.
- **Alembic export slice (FR-013 / BDD-013)** — blocked on Q5 + Q6. When this lands, BDD-102 mechanization can extend to compare Alembic bytes too.

See `.agent/PLAN.md` and `.agent/CURRENT_WORK.md` for full plan and progress.
