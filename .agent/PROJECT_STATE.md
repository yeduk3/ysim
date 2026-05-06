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

**Primitive creation slice** — implement `BDD-001` (create primitive object). Ships `Create > Sphere` and `Create > Cube` so a fresh ysim scene can be built from primitives via the GUI. See `.agent/PLAN.md` for the concrete todo list.

Why this next:
- First link of `BDD-101` (the v1 round-trip spine). Without primitives, every other slice is exercised on imported `.obj` meshes only.
- Spec is unambiguous and self-contained — no PRD open question blocks it.
- Testable from the CPU-side data layer alone (sphere/cube generators are pure math). No Metal harness required for `BDD-001` acceptance.
- Promotes `"sphere"`/`"cube"` from reserved-but-not-shipped (D-003) to shipping shape names.

After this slice, the open candidates remain:

- **Material editing UI slice (FR-005 / BDD-005)** — needs a PBR preview shader to satisfy "preview render reflects the lower roughness", so it ships paired with renderer work.
- **Behavior assignment UI slice (FR-006 / BDD-006)** — switching behavior in-place reallocates per-mesh state; non-trivial.
- **Rigid body slice (FR-008 / BDD-008)** — blocked on Q4.
- **Alembic export slice (FR-013 / BDD-013)** — blocked on Q5 + Q6.
- **Test-harness slice (Metal-backed sim)** — closes the parked persistence WARNING; forces Q-D.

## Recent scope changes

- **2026-05-06, persistence slice.** D-007 supersedes D-004: rotation moved from "JSON-only" to a real `GeneralMesh::rotationQuat` field after the Estimator BLOCKed the round-trip gap. No spec change — the design doc already required round-trip; the in-memory mirror was the missing piece. (See `docs/DECISIONS.md` D-004 / D-007.)
- **2026-05-06, persistence slice.** Design doc `docs/design/scene_format.md` updated to declare `"grid"` as the v1-shipped primitive shape with `"sphere"`/`"cube"` reserved-not-shipped, mirroring D-003. This was a doc/code reconciliation, not a scope change — v1 only ever had a grid initializer.
- **Pending in primitive-creation slice (this plan).** D-003 will be *amended* (not superseded) when `"sphere"` and `"cube"` ship as real initializers. The reserved-not-shipped *pattern* survives unchanged; only the membership of the reserved set narrows.

## What the Estimator should know

- The backend-boundary invariant (`BDD-103`) applies to *every* slice. Any persistence-slice change that touches `src/shader/` or simulation kernels is suspect.
- "Determinism, scoped" (`BDD-102`) is a *single-machine* promise. Don't escalate cross-machine drift to BLOCK in v1.
- Reserved-but-not-shipped behaviors (`Elastic`, `Fluid`, `Generator`) keep their enum identifiers — reordering them silently corrupts saved scenes.
- `GeneralMesh::rotationQuat` exists as of D-007 but no consumer reads it yet. A future render/sim slice that *adds* a consumer is implementing FR-004, not violating BDD-103 — it's reading a field that's already there.
- The persistence slice's WARNING about app-level coverage (`Simulator::saveScene/loadScene` unexercised end-to-end) is parked until the test-harness slice resolves Q-D. The matrix lists BDD-015 as `pass` because the JSON-layer round-trip is the testable subset today; calling it `pass` is honest under "what we can verify with the harness we have", not under "every claim in TESTS.md#BDD-015 is mechanically checked".
