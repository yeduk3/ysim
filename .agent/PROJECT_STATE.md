# Project State

> Owner: **Planner**. Compact rolling summary so the next planning turn does not have to re-read every spec.
> Updated: 2026-05-07

## What ysim is (one paragraph)

A macOS-only simulation engine targeting cloth and rigid-body shots, sitting between Houdini (too complex) and Unreal (too real-time-biased). v1 ships an end-to-end author → simulate → save → reload → export-to-Alembic flow, with the scene format and behavior API kept structured enough to support a v2 LLM control surface. Dual-GPU architecture (OpenGL render + Metal compute), template-parameterized backends (`CPU`/`METAL`), no virtual dispatch on the hot path.

## Specs in place

- `docs/specs/PRD.md` — authored 2026-05-06. Seven sections (Problem, Outcome, Scope, Non-goals, Constraints, Success metrics, Open questions).
- `docs/specs/FRD.md` — 19 functional requirements, 1:1 with v1 capabilities + no-regression items.
- `docs/specs/BDD.md` — 19 paired user-stories + 3 cross-cutting (BDD-101 round-trip, BDD-102 determinism, BDD-103 backend-boundary).
- `docs/TESTS.md` — 22 Given/When/Then scenarios.
- `docs/TEST_MATRIX.md` — 22 rows, all `pending`.

## What's already implemented (per PRD §6 metric #3 — no-regression set)

- Metal-based collision detection: per-object and scene-wide LBVH (spatial hashing parked due to perf).
- Metal-based explicit Euler integration for cloth.
- Click-to-select objects via ray-cast; live color/behavior edits.
- `FrameProfiler` with named timing sections + GUI window + CSV export.
- Both `TriangularCloth` and `FastGridCloth` cloth variants exist; `Float` and `Rigid` behaviors are defined in the enum but Rigid is not yet wired through.

## Open questions blocking future plan slices

Carried forward from `PRD.md`. Each lists which slice it blocks.

- **Q1 — OpenPBR subset.** Resolved in practice by **D-005**: v1 ships base color, metallic, roughness, specular weight, emission color; persistence round-trips all five. Material UI slice (FR-005 / BDD-005) can proceed; PBR preview shader is the remaining open work.
- **Q2 — Cloth UX surface (one option vs two).** Still open. Persistence and primitive slices treat `TriangularCloth` and `FastGridCloth` as two distinct behavior tags (D-005-ish convention); a future Behavior UI slice (FR-006 / BDD-006) decides whether to merge them in the user-facing dropdown.
- **Q3 — Scene file format.** **Resolved (D-001): JSON**, vendored `nlohmann/json` under `include/nlohmann/`.
- **Q4 — Rigid backend default (Bullet vs Jolt).** Still open. Blocks the rigid-body slice (FR-008 / BDD-008).
- **Q5 — Alembic schema specifics.** Still open. Blocks the export slice (FR-013 / BDD-013).
- **Q6 — Export FPS / time-step decoupling rule.** Still open. Blocks the export slice (FR-013 / BDD-013).
- **Q7 — Save-file forward migration policy.** Does not block v1 (only v1→v2 transition).
- **Q-D (from `docs/ARCHITECTURE.md §5`) — Test backend ownership.** **Resolved (D-012): headless Metal harness via `ysim --self-test`**, with SKIP path on hosts without Metal so the Estimator's Linux container can still run `verify.sh`.

## Decisions (Planner-resolved, awaiting human confirmation)

- **Scene file format = JSON.** Reason: structured (each field named), human-diffable (PRD §5.2 forward-compat constraint), easy LLM target for v2, low integration cost. Alternative (binary) was rejected because it works against the LLM-addressability constraint. Hybrid (JSON + sidecar binary) was rejected as premature optimization for v1 — scene files don't carry heavy data (mesh imports are referenced by path).
- **New primitives default to `Float` behavior.** Reason: prevents surprise on creation (a newly-spawned cube doesn't immediately fall through the floor before the user can assign anything to it).
- **Imported meshes are referenced by file path, not embedded.** Reason: smaller scene files, predictable diff. Cost: moving an asset breaks load. Acceptable for v1; revisit if user reports breakage.

## Next milestone

**In flight: BDD-003 Translate-object slice on `feat/translate-object`** (off `main` at `a85aba8`, the doc-maintenance pass). Plan in `.agent/PLAN.md`. Adds `GeneralMesh::transformPosition` (D-007's `rotationQuat` is the precedent), `Simulator::translateObject(id, vec3)` that mutates both `state.x` and `state.xPrev` by the delta (xPrev parity is load-bearing per D-013), inspector `InputFloat3("Position")` row, `toSnapshot`/`loadScene` round-trip via the new field, Block 9 in `runSelfTest` mechanizing BDD-003's three "Then" clauses verbatim, and folds in cloth-CCD turn-6 WARNING (`nparams.thickness = 0` → `simulator.margin`) as a small housekeeping todo.

Cloth-CCD slice (`feat/cloth-ccd`) was merged into `main` on 2026-05-08 (commit `95a710f`); closed `BDD-007` and `CM-005`. `verify.sh` exits 0 cleanly with 16/16 self-test PASS lines. The standing-BLOCK era is over. Estimator's turn-6 verdict was WARNING (one item, the `thickness = 0` hardcode) — folded into the translate-object slice per the small-WARNING bundling rule (PLANNER.md procedure step 2).

Standing feature candidates after this slice, ordered by what closes the most v1 acceptance:

- **BDD-102 Determinism mechanization** — extend `runSelfTest` with a two-runs-bit-identical assertion against a saved-scene baseline. Smallest slice; uses the harness D-012 already established. Newly tractable now that BDD-007 is stable (no random-tunneling drift to confuse a determinism test).
- **Material editing UI (FR-005 / BDD-005)** — needs a PBR preview shader to satisfy "preview render reflects the lower roughness". Q1 settled by D-005, so the data-layer side is ready; the renderer-side work is the slice's bulk.
- **Behavior assignment UI (FR-006 / BDD-006)** — in-place behavior switching reallocates per-mesh state; non-trivial. Q2 still open (one cloth UX option vs two).
- **BDD-019 Profiler test row** — already implemented in the GUI; add a doctest binding (or a Block in `runSelfTest`) that exercises `FrameProfiler` and asserts CSV export contents. Closes a `pending` matrix row trivially.
- **BDD-017 / BDD-018 Ray-pick + live-edit propagation** — both already implemented; harder to mechanize without GUI input simulation, so probably skipped until v1 ships.
- **Rigid body slice (FR-008 / BDD-008)** — blocked on Q4 (Bullet vs Jolt default).
- **Alembic export slice (FR-013 / BDD-013)** — blocked on Q5 + Q6.

`BDD-101` (the v1 spine: author → simulate → save/reload → export) is structurally complete except for the export step; once Alembic ships, BDD-101's acceptance can be mechanized.

## Recent scope changes

- **2026-05-06, persistence slice.** D-007 supersedes D-004: rotation moved from "JSON-only" to a real `GeneralMesh::rotationQuat` field after the Estimator BLOCKed the round-trip gap. No spec change — the design doc already required round-trip; the in-memory mirror was the missing piece. (See `docs/DECISIONS.md` D-004 / D-007.)
- **2026-05-06, persistence slice.** Design doc `docs/design/scene_format.md` updated to declare `"grid"` as the v1-shipped primitive shape with `"sphere"`/`"cube"` reserved-not-shipped, mirroring D-003. This was a doc/code reconciliation, not a scope change — v1 only ever had a grid initializer.
- **Pending in primitive-creation slice (this plan).** D-003 will be *amended* (not superseded) when `"sphere"` and `"cube"` ship as real initializers. The reserved-not-shipped *pattern* survives unchanged; only the membership of the reserved set narrows.
- **2026-05-06 / 07, primitive-creation slice.** D-010 amended D-003 (sphere/cube shipped). Two runtime bugs surfaced during manual GUI testing and were fixed in `2000aea` and `979f037`: CM-002 (`Scene::pack()` double-frees initializers when re-run because `meshes[i]` and `requestsGeneralMeshes[i]` share the pointer) and CM-003 (`BVH::build` skips re-allocation when primitive count changes, only checking `!tree.ptr`). Both fall in the gap the persistence slice's parked WARNING called out — the unit-test net cannot reach `Simulator::initialize` because of the GLFW/GL coupling in `mesh.initialize()`. This is the concrete motivation for promoting the test-harness slice to the next-after-environment-forces slot.
- **2026-05-07, env-forces slice (turn-2).** Estimator gave WARNING on the original commit (`4047af0`); user's manual GUI test caught CM-004 — both cloth force kernels (`compute_cloth_grid_forces_fast` and `compute_tri_spring_forces`) hardcoded gravity from `SimParams::G` and ignored the bound `externalForces` buffer. PLAN.md's "no Metal kernel changes" non-goal was based on a wrong assumption that the kernels consumed the buffer the C++ side bound; they did not. Patched in `e3a3154` (kernels now read `externalForces[id]` and drop the hardcoded gravity term). Pattern is now well-established: three successive slices ended with manual-test-catches-runtime-bug-in-Metal-path. The next planner-tracked milestone is the render-state decoupling slice, which is the structural prerequisite for actually testing these paths.
- **2026-05-07, render-state decoupling slice.** D-011 — `MeshGL<CPU>` lifted to `include/MeshGL.hpp`; `MeshRenderState` (in `include/MeshRenderState.hpp`) owns per-mesh GL state keyed by `mesh.id`; `GeneralMesh::meshGL` field removed. `Simulator::initialize` no longer touches GL. Estimator: NOTE-level. Strengthens `ARCHITECTURE §2.2/§2.3` boundary. Unblocks the harness slice (next milestone).
- **2026-05-07, harness slice (this plan).** Q-D resolved by Planner: headless Metal harness, **not** CPU backend reference. The CPU backend stays as type-system reservation only for v1. Resolution recorded in `docs/DECISIONS.md` D-012 when the harness slice landed.
- **2026-05-07, cloth-drape (BDD-007) slice.** Three of four BDD-007 clauses pass via Block 6 in `runSelfTest`. Real fix: uncommented `collisionPipeline.broadPhase.enlargeTrajectory(system.subh)` (CM-005 partial fix). New `cumulativeNarrowCollisions` counter in `Scene::packedCollisionData` so the harness can track contacts that fire and reset within a single frame. Tunneling clause still FAILs — parked under CM-005 for a future cloth-CCD slice (replace snapshot narrow-phase distance check with swept-segment-vs-triangle). Matrix row stays `warning`.
- **2026-05-07, BDD-002 import-mesh-ui slice.** `Simulator::importMesh` added with file-existence guard before `addFloatMesh` (without it, `MeshFileInitializer` silently queues a zero-vertex mesh on missing path — `ObjData::loadObject` is graceful on open-fail). `File > Import Mesh…` modal wired. Block 7 in `runSelfTest` mechanizes BDD-002's two "Then" clauses (happy: numMeshes++ + Float + path round-trip via `toSnapshot`; error: missing file → numMeshes unchanged). Block 8 (BDD-015) now resets the scene first because Block 7's imported mesh has a path that doesn't resolve relative to `/tmp/`. Estimator turn 4 verdict: WARNING (commit allowed) — two follow-ups bundled into the next (cloth-CCD) slice: tighten Block 7's happy-path assertion (AABB / facet-count); fix modal default path from `assets/Human.obj` to `src/assets/Human.obj` for the build-dir launch context.
- **2026-05-08, cloth-CCD slice (`feat/cloth-ccd` → `main`, commit `95a710f`).** D-013 — `narrow_pt_tri` rewritten as swept-segment CCD using new `xPrev` buffer (slot 10); writes signed distance (was abs'd before, which negated the integrator's push for tunneled particles). Per-substep `state.x → state.xPrev` snapshot in `Simulator::update`. Harness `subSteps = 4 → 8` for one-substep-lag headroom. Folded BDD-002 estimator turn-4 follow-ups (modal default + AABB/facet assertion). 16/16 self-test PASS; `verify.sh` exits 0 cleanly. CM-005 graduated to `OLD_MISTAKES.md` because the snapshot path was structurally removed. BDD-007 matrix row promoted `warning → pass`.
- **2026-05-08, doc maintenance pass.** `docs/CONVENTIONS.md` planned-status pointers refreshed (test/ + scripts/ exist; D-002 confirms doctest; include/ inventory updated). `docs/ARCHITECTURE.md §5` Q-D marked resolved by D-012. `docs/mistakes/COMMON_MISTAKES.md` CM-005 graduated; `docs/mistakes/OLD_MISTAKES.md` gained the "snapshot-only collision tests miss fast-moving thin geometry" pattern entry. PROJECT_STATE open-questions list synced with decisions (D-001/Q3, D-005/Q1, D-012/Q-D resolved). Role docs (`GENERATOR.md`, `PLANNER.md`, `ESTIMATOR.md`) codified learnings: spec-vs-label trap, Metal-kernel boundary check, fix-turns stay on slice branch (Generator); small-WARNING folding rule, escape-pattern detection, spec-substitution discipline (Planner); SKIP-vs-FAIL on Metal-less hosts, parked-failure callout, TESTS.md-not-labels (Estimator).
- **2026-05-07, translate-object slice planned (this plan).** `feat/translate-object` branched off `main` at `a85aba8`. BDD-003 closes via `GeneralMesh::transformPosition` field + `Simulator::translateObject` mutator + inspector `InputFloat3("Position")` + Block 9 in `runSelfTest`. Folded cloth-CCD turn-6 WARNING (`nparams.thickness = 0; // temp.` → `simulator.margin`) per small-WARNING bundling rule. Renderer mutates `state.x` directly rather than introducing a model-matrix refactor — that's a Non-goal explicitly carved out and recorded as the slice's load-bearing decision (new D-NNN). `xPrev` parity with `x` is load-bearing for D-013 swept-CCD; the slice mutates both by the same delta on translate.
- **2026-05-07, harness slice fix turn.** Estimator BLOCKed: `BDD-009`/`011`/`012` self-test bodies pattern-matched the matrix labels rather than mechanising the spec wording (no wind in BDD-009; no runtime-without-restart pivot in BDD-011; no actual wind application in BDD-012). Plus Metal-less host (Codex container) made `verify.sh` exit non-zero before any assertion ran. Fix plan: rewrite the three blocks against `docs/TESTS.md` "Then" clauses verbatim (full state.x/state.v strict equality for Float; gravity-runtime-pivot without re-init for BDD-011; wind-drives-cloth-+x for BDD-012); flip the null-device path from FAIL to SKIP so the Estimator's host is supported. Pattern lesson: harness assertions must be authored from `docs/TESTS.md`, not from the compressed matrix-row labels.
- **2026-05-07, harness slice second turn (commit `a0b5fca`).** All eight assertion blocks pass on macOS Apple Silicon; Estimator's Linux container takes the SKIP path and `verify.sh` exits 0 cleanly. Slice merged into main (FF-merge after commit). Test matrix rows BDD-009/011/012/015 now `pass` with addresses pointing at the rewritten block names.

## What the Estimator should know

- The backend-boundary invariant (`BDD-103`) applies to *every* slice. Any persistence-slice change that touches `src/shader/` or simulation kernels is suspect.
- "Determinism, scoped" (`BDD-102`) is a *single-machine* promise. Don't escalate cross-machine drift to BLOCK in v1.
- Reserved-but-not-shipped behaviors (`Elastic`, `Fluid`, `Generator`) keep their enum identifiers — reordering them silently corrupts saved scenes.
- `GeneralMesh::rotationQuat` exists as of D-007 but no consumer reads it yet. A future render/sim slice that *adds* a consumer is implementing FR-004, not violating BDD-103 — it's reading a field that's already there.
- The persistence slice's WARNING about app-level coverage (`Simulator::saveScene/loadScene` unexercised end-to-end) is parked until the test-harness slice resolves Q-D. The matrix lists BDD-015 as `pass` because the JSON-layer round-trip is the testable subset today; calling it `pass` is honest under "what we can verify with the harness we have", not under "every claim in TESTS.md#BDD-015 is mechanically checked".
- For the **environment forces slice** the same partial-pass convention applies: gravity/wind round-trip and Environment-panel wiring are unit-testable, but the "object accumulates velocity in +x" clause from BDD-011 (and the strict-equality clause from BDD-009) require sim-step execution and are parked behind the test-harness slice. Expect a `WARNING` row, not a `BLOCK` — this is the documented gap, not new debt.
