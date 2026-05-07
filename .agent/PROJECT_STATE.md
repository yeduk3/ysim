# Project State

> Owner: **Planner**. Compact rolling summary so the next planning turn does not have to re-read every spec.
> Updated: 2026-05-06

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

- **Q1 — OpenPBR subset.** Blocks the material-editing slice (FR-005 / BDD-005). Proposed minimum: base color, metallic, roughness, specular weight, emission color. Others are added later.
- **Q2 — Cloth UX surface (one option vs two).** Blocks the behavior-assignment UI slice (FR-006 / BDD-006). Current planner default: two options.
- **Q3 — Scene file format.** Was blocking the persistence slice. **Provisionally resolved by the Planner: JSON** (see "Decisions" below). Awaiting human confirmation; if overridden, the persistence slice plan changes accordingly.
- **Q4 — Rigid backend default (Bullet vs Jolt).** Blocks the rigid-body slice (FR-008 / BDD-008). Affects which backend the round-trip acceptance test uses.
- **Q5 — Alembic schema specifics.** Blocks the export slice (FR-013 / BDD-013).
- **Q6 — Export FPS / time-step decoupling rule.** Blocks the export slice (FR-013 / BDD-013).
- **Q7 — Save-file forward migration policy.** Does not block v1 (only v1→v2 transition).

## Decisions (Planner-resolved, awaiting human confirmation)

- **Scene file format = JSON.** Reason: structured (each field named), human-diffable (PRD §5.2 forward-compat constraint), easy LLM target for v2, low integration cost. Alternative (binary) was rejected because it works against the LLM-addressability constraint. Hybrid (JSON + sidecar binary) was rejected as premature optimization for v1 — scene files don't carry heavy data (mesh imports are referenced by path).
- **New primitives default to `Float` behavior.** Reason: prevents surprise on creation (a newly-spawned cube doesn't immediately fall through the floor before the user can assign anything to it).
- **Imported meshes are referenced by file path, not embedded.** Reason: smaller scene files, predictable diff. Cost: moving an asset breaks load. Acceptable for v1; revisit if user reports breakage.

## Next milestone

**Headless self-test harness slice** — add a `--self-test` mode to the `ysim` binary that mechanically verifies the simulator paths the unit-test net cannot reach. Closes the parked sim-step clauses on `BDD-009`/`011`/`012`/`015`. See `.agent/PLAN.md` for the concrete todo list.

Why this slice now:
- Render-state decoupling (D-011) just removed the structural blocker — `Simulator::initialize` no longer needs a live GL context. The harness slice was the explicit *next* milestone for the past four planning turns.
- Pattern of the past four slices: `Generator declares done → manual user test catches Metal-side bug → patch → re-review`. CM-002, CM-003, CM-004, and the persistence-slice WARNING all live in the same untested path. After this slice the failure mode should shift to "verify.sh catches it locally before review".
- **Q-D resolved (planner-level): headless Metal harness, not CPU backend reference.** v1 is macOS-only; the CPU backend stays as type-system reservation only. CPU sim implementation is multi-slice work that does not pay back inside v1's scope.
- Scope deliberately narrow: `--self-test` lives inside `src/main.cpp` next to `int main()`. **No** extraction of types into a static library / header-set in this slice — that's a multi-slice refactor of its own.

Standing feature candidates (all deferred one more slice):

- **BDD-007 cloth drapes onto rigid surface** — newly tractable with gravity wired (`e3a3154`). The self-test landing this slice does NOT cover collision response end-to-end; that's BDD-007's slice.
- **BDD-002 Import .obj mesh via UI** — small slice; underlying `addClothFile`/`addFloatMesh` paths already work.
- **Material editing UI (FR-005 / BDD-005)** — needs a PBR preview shader to satisfy "preview render reflects the lower roughness".
- **Behavior assignment UI (FR-006 / BDD-006)** — in-place behavior switching reallocates per-mesh state.
- **Rigid body slice (FR-008 / BDD-008)** — blocked on Q4.
- **Alembic export slice (FR-013 / BDD-013)** — blocked on Q5 + Q6.
- **Determinism mechanization (BDD-102)** — newly tractable now that the harness exists. Promote two-runs-bit-identical from manual to mechanical when the next harness expansion happens.

## Recent scope changes

- **2026-05-06, persistence slice.** D-007 supersedes D-004: rotation moved from "JSON-only" to a real `GeneralMesh::rotationQuat` field after the Estimator BLOCKed the round-trip gap. No spec change — the design doc already required round-trip; the in-memory mirror was the missing piece. (See `docs/DECISIONS.md` D-004 / D-007.)
- **2026-05-06, persistence slice.** Design doc `docs/design/scene_format.md` updated to declare `"grid"` as the v1-shipped primitive shape with `"sphere"`/`"cube"` reserved-not-shipped, mirroring D-003. This was a doc/code reconciliation, not a scope change — v1 only ever had a grid initializer.
- **Pending in primitive-creation slice (this plan).** D-003 will be *amended* (not superseded) when `"sphere"` and `"cube"` ship as real initializers. The reserved-not-shipped *pattern* survives unchanged; only the membership of the reserved set narrows.
- **2026-05-06 / 07, primitive-creation slice.** D-010 amended D-003 (sphere/cube shipped). Two runtime bugs surfaced during manual GUI testing and were fixed in `2000aea` and `979f037`: CM-002 (`Scene::pack()` double-frees initializers when re-run because `meshes[i]` and `requestsGeneralMeshes[i]` share the pointer) and CM-003 (`BVH::build` skips re-allocation when primitive count changes, only checking `!tree.ptr`). Both fall in the gap the persistence slice's parked WARNING called out — the unit-test net cannot reach `Simulator::initialize` because of the GLFW/GL coupling in `mesh.initialize()`. This is the concrete motivation for promoting the test-harness slice to the next-after-environment-forces slot.
- **2026-05-07, env-forces slice (turn-2).** Estimator gave WARNING on the original commit (`4047af0`); user's manual GUI test caught CM-004 — both cloth force kernels (`compute_cloth_grid_forces_fast` and `compute_tri_spring_forces`) hardcoded gravity from `SimParams::G` and ignored the bound `externalForces` buffer. PLAN.md's "no Metal kernel changes" non-goal was based on a wrong assumption that the kernels consumed the buffer the C++ side bound; they did not. Patched in `e3a3154` (kernels now read `externalForces[id]` and drop the hardcoded gravity term). Pattern is now well-established: three successive slices ended with manual-test-catches-runtime-bug-in-Metal-path. The next planner-tracked milestone is the render-state decoupling slice, which is the structural prerequisite for actually testing these paths.
- **2026-05-07, render-state decoupling slice.** D-011 — `MeshGL<CPU>` lifted to `include/MeshGL.hpp`; `MeshRenderState` (in `include/MeshRenderState.hpp`) owns per-mesh GL state keyed by `mesh.id`; `GeneralMesh::meshGL` field removed. `Simulator::initialize` no longer touches GL. Estimator: NOTE-level. Strengthens `ARCHITECTURE §2.2/§2.3` boundary. Unblocks the harness slice (next milestone).
- **2026-05-07, harness slice (this plan).** Q-D resolved by Planner: headless Metal harness, **not** CPU backend reference. The CPU backend stays as type-system reservation only for v1. Resolution will be recorded in `docs/DECISIONS.md` by the Generator when the harness slice ships.

## What the Estimator should know

- The backend-boundary invariant (`BDD-103`) applies to *every* slice. Any persistence-slice change that touches `src/shader/` or simulation kernels is suspect.
- "Determinism, scoped" (`BDD-102`) is a *single-machine* promise. Don't escalate cross-machine drift to BLOCK in v1.
- Reserved-but-not-shipped behaviors (`Elastic`, `Fluid`, `Generator`) keep their enum identifiers — reordering them silently corrupts saved scenes.
- `GeneralMesh::rotationQuat` exists as of D-007 but no consumer reads it yet. A future render/sim slice that *adds* a consumer is implementing FR-004, not violating BDD-103 — it's reading a field that's already there.
- The persistence slice's WARNING about app-level coverage (`Simulator::saveScene/loadScene` unexercised end-to-end) is parked until the test-harness slice resolves Q-D. The matrix lists BDD-015 as `pass` because the JSON-layer round-trip is the testable subset today; calling it `pass` is honest under "what we can verify with the harness we have", not under "every claim in TESTS.md#BDD-015 is mechanically checked".
- For the **environment forces slice** the same partial-pass convention applies: gravity/wind round-trip and Environment-panel wiring are unit-testable, but the "object accumulates velocity in +x" clause from BDD-011 (and the strict-equality clause from BDD-009) require sim-step execution and are parked behind the test-harness slice. Expect a `WARNING` row, not a `BLOCK` — this is the documented gap, not new debt.
