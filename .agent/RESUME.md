# Resume — FR-004 UI Rotate-Object Slice (D-021 + D-022 land; pack-roundtrip deferred)

## Must remember

- **Branch:** `feat/rotate-object` (off `main` at `4217619`).
- **D-021 — `Simulator::rotateObject(meshId, newQuat)` is absolute-form.** Mirrors `translateObject(newPos)`. Internally computes `delta = newAbs * conjugate(currentQuat)`, applies to state.x AND state.xPrev around `mesh.transformPosition` pivot, writes `mesh.rotationQuat = newAbs`. `state.v` unchanged. Future rotation API extensions (delta-form helper, rotateAroundCustomPivot, etc.) should layer on top of this.
- **D-022 — `quatConjugate` and `rotateVector(q, v)` join D-019's family.** Both inline free functions; both assume q is unit norm; both reuse D-019's `operator*` so the Hamilton convention (`a * b = apply b first, then a`) stays consistent. Future rotation consumers (FR-008 rigid body, eventual renderer-side rotation) should use these rather than re-inlining the math.
- **Pack-roundtrip is the open follow-up.** Re-pack rebuilds state.x from the initializer's geometry — rotateObject's effect on state.x is lost on next `Simulator::initialize()`. The clean fix mirrors D-015's translate write-back pattern: `rotateObject` writes `pendingRotations[mesh.id] = newAbs`, AND `applyPendingMaterials()` is called automatically from `Simulator::initialize()` (contract change). The Planner needs to decide whether the auto-call is the right shape — that's the design call. Until then, a `addCube` + `simulator.initialize()` sequence after a rotate silently drops the rotate effect.
- **Inspector raw-quat input is minimal.** FR-004 Notes line says "UI may expose Euler/axis-angle as input affordances" — that's a future slice. Current UI is `InputFloat4("Quat (w,x,y,z)")`. The user has to know quaternion math (or paste a value).
- **Block 15 cube witness vertex** lives at `(0.25, -0.25, -0.25)` after `addCube(center=(0,0,0), tess=2, size=0.5)`. Under 90° Z rotation: `(x, y, z) → (-y, x, z) = (0.25, 0.25, -0.25)`. If the cube generator changes (different witness vertex), Block 15's expected coordinates need to change too — the `posTol = 1e-5` is tight.
- **`state.v` is NOT rotated.** Cloth-mid-flight rotation would physically need v-rotation too; deferred. Block 15 only tests Float so this gap doesn't affect the assertion.

## Last decisions + why

- **D-021 absolute-form (not delta-form)** — symmetry with translateObject. Inspector naturally has the absolute current value + user edit; passing the delta would force the inspector to compute it. Rejected per-mesh model matrix in renderer (renderer rework, same reason D-014 baked into state.x). Rejected initializer write-back in this slice (needs Planner design call about applyPendingMaterials auto-call).
- **D-022 free functions, not Quat members** — D-019's existing decision. Keeps Quat as POD aggregate matching on-disk schema; math lives next to the struct.

## Next step you were about to take

Slice complete. Next concrete step is the **Estimator's** turn — `./scripts/verify.sh` should exit 0 with **31/31** self-test PASS lines. Expected verdict: NOTE level.

After this lands, planner-tracked candidates per `PROJECT_STATE.md`:

- **Rotate pack-roundtrip slice (FR-004 follow-up).** Mirror D-015's translate write-back pattern. The Planner design call: should `Simulator::initialize()` auto-call `applyPendingMaterials()`? Today it's only called explicitly from `loadScene` flows. Auto-calling would change the contract for existing call sites; not auto-calling means the user has to remember (or every call site duplicates the call). Pick.
- **CM-008 production-side fix** — `BroadPhase::build`'s Float-mesh skip robust against scene-swap-at-same-count. Theoretical concern in v1.
- **Inspector ergonomics for rotation** — Euler / axis-angle input affordances per FR-004 Notes.
- **BDD-018 inspector live-edit propagation** — implementation exists; mechanization needs ImGui-side simulation or callable abstraction.
- **Material editing UI (FR-005 / BDD-005)** — needs PBR preview shader. Large.
- **Behavior assignment UI (FR-006 / BDD-006)** — Q2 still open.
- **Rigid body slice (FR-008 / BDD-008)** — blocked on Q4. When this lands: D-015 three-site cascade + D-018 seed + D-019/D-022 Quat math + D-020 BVH leaf-return + D-021 rotate semantics all apply.
- **Alembic export slice (FR-013 / BDD-013)** — blocked on Q5 + Q6.

See `.agent/PLAN.md` and `.agent/CURRENT_WORK.md` for full plan and progress.
