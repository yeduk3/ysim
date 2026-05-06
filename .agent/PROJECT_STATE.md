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

**Persistence slice** — implement `BDD-014` (save), `BDD-015` (load), `BDD-016` (version reject). See `.agent/PLAN.md` for the concrete todo list.

Why this first:
- Every other v1 capability eventually depends on it (Alembic export reads from a scene; live editing benefits from save-as-checkpoint; v2 LLM control needs a persistence target).
- Purely additive against current code — no risk to the implemented core (cloth, collision, profiler).
- Resolvable now: the only blocking open question (Q3) has a defensible default.

## Recent scope changes

None. (This is the first plan slice.)

## What the Estimator should know

- The backend-boundary invariant (`BDD-103`) applies to *every* slice. Any persistence-slice change that touches `src/shader/` or simulation kernels is suspect.
- "Determinism, scoped" (`BDD-102`) is a *single-machine* promise. Don't escalate cross-machine drift to BLOCK in v1.
- Reserved-but-not-shipped behaviors (`Elastic`, `Fluid`, `Generator`) keep their enum identifiers — reordering them silently corrupts saved scenes.
