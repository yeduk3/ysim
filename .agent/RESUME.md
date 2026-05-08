# Resume — Translate-Object Slice (BDD-003 closed)

## Must remember

- **Branch:** `feat/translate-object` (off `main` at `a85aba8`, the doc-maintenance pass).
- **D-014 is load-bearing.** `GeneralMesh::transformPosition` is the world-space center mirror; pack-time seeds it from the initializer's `center`/`offset` via a `dynamic_cast` cascade over the four initializer subtypes. `Simulator::translateObject(meshId, newPos)` is the **only** mutator and moves `state.x` and `state.xPrev` by `newPos - mesh.transformPosition`. `state.v` is unchanged.
- **`xPrev` parity with `x` is non-negotiable.** D-013's swept-CCD reads `xPrev` as the start-of-substep position. If `state.x` moves but `state.xPrev` doesn't, the next narrow phase sees a meters-long swept segment and emits spurious tunneling contacts. Any future code path that mutates `state.x` outside the integrator must mutate `state.xPrev` by the same delta.
- **`scene.dirty` is intentionally NOT set by `translateObject`.** Setting it would force a re-pack on next `Simulator::initialize()`, and pack reseeds `transformPosition` from the initializer's `center` — which would silently undo the translate. Translates are runtime mutations; topology is unchanged.
- **`MeshRenderState` needs no invalidation hook on translate.** D-011's renderer reads through `mesh.state.x.ptr` each frame in `Simulator::uploadMeshes`. Mutating `state.x` in place means the next upload picks up the new positions automatically.
- **`loadScene` does NOT need a side-table for `transformPosition`** (unlike D-007's `pendingRotations`). `o.transform.position` is fed into the new initializer's `center`/`offset`, and pack-time then reads that back into `transformPosition`. The round-trip closes implicitly.
- **CM-006 deferred WARNING.** `BruteForce::narrow` takes a `thickness` parameter now, but the call site passes `PR(0)`. The cloth-CCD turn-6 WARNING (`radius + thickness` slow-touch band) still applies — the integrator's unconditional vn-zero (`physics.metal:202`) needs to be gated behind `(distance < thickness)` first. Out of this slice's scope; recorded as CM-006.
- **Off-slice cleanup applied.** Reverted an uncommitted `refit() → enlargeTrajectory()` swap that was in `src/main.cpp::Simulator::update`'s working tree at session start (not in any commit). It was hiding a BDD-007 regression independent of my changes.

## Last decisions + why

- **D-014** — `transformPosition` on `GeneralMesh` + `translateObject` mutates `state.x` (and `state.xPrev`) in place. Rejected: per-mesh model matrix in the renderer (much larger refactor; v1 reads positions directly from `state.x` everywhere). Rejected: mutating the initializer's `center`/`offset` (initializers are deserialization records; mutating them mid-sim conflicts with re-pack flows). Rejected: resetting `state.v` on translate (contradicts BDD-003's "next simulation step uses the new position", which implies physics carries forward).

## Next step you were about to take

Slice complete. Next concrete step is the **Estimator's** turn — `./scripts/verify.sh` should exit 0 cleanly with 19/19 self-test PASS lines. Expected verdict: NOTE-level.

After this lands, planner-tracked candidates per `PROJECT_STATE.md`:

- **CM-006 follow-up slice** — gate `physics.metal::integrate_cloth*` vn-zero behind `(distance < thickness)`, then wire `nparams.thickness` to `simulator.margin` (or per-mesh cloth thickness). Closes the cloth-CCD turn-6 WARNING properly.
- **BDD-102 Determinism mechanization** — extend the harness with two-runs-bit-identical assertion against a saved-scene baseline.
- **Material editing UI (FR-005 / BDD-005)** — needs PBR preview shader.
- **Behavior assignment UI (FR-006 / BDD-006)** — in-place behavior switching reallocates per-mesh state.
- **BDD-019 Profiler test row** — already implemented in the GUI; add a Block in `runSelfTest` that exercises `FrameProfiler` and asserts CSV export contents.
- **Rigid body slice (FR-008 / BDD-008)** — blocked on Q4.
- **Alembic export slice (FR-013 / BDD-013)** — blocked on Q5 + Q6.

See `.agent/PLAN.md` and `.agent/CURRENT_WORK.md` for full plan and progress.
