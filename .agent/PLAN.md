# Plan — Primitive Creation Slice

> Owner: **Planner** writes; **Generator** executes; **Estimator** judges.
> Updated: 2026-05-06

## Goal

Ship `Create > Sphere` and `Create > Cube` so a fresh ysim scene can be built from primitives via the GUI. This is the first link in `BDD-101` (the v1 round-trip spine) and promotes `"sphere"`/`"cube"` from reserved-but-not-shipped (D-003) to shipping shape names.

When the slice is done: the user opens ysim on an empty scene, picks `Create > Sphere`, sees a sphere appear in the viewport at the origin with default material and `Float` behavior; saves the scene to `.ysim.json`; loads it back and the sphere is in the same place.

## Scope

- `BDD-001` — create primitive object (FR-001).
- New CPU-side geometry generators: `MeshSphereInitializer` and `MeshCubeInitializer`, parallel to the existing `MeshGridInitializer`. Both produce `state.x` + `adjacency.facets`/`edges` in the same shape so the existing collision pipeline picks them up unchanged.
- ImGui menu wiring under the existing `Create` (or new) menu — minimum: shape, size, tessellation, optional initial position. Same pattern as the persistence slice's text-input modals.
- Loader / `scene_format.hpp` updates: `"sphere"` and `"cube"` join `"grid"` as known primitive shapes. The reserved-not-shipped rejection path stays for any future names that aren't yet shipping.
- Design-doc amendment: `docs/design/scene_format.md` is updated so the schema reflects what's actually shipping. D-003 stays as historical record; a new D-NNN entry documents the promotion.
- Tests (`test/scene_io_test.cpp` + a new `primitive_test.cpp` if cleaner): structural verification that sphere / cube objects round-trip through the JSON layer with the new shape names, and that the CPU-side initializers produce a mesh with the expected vertex/facet counts and a topology that survives the existing adjacency build.

Behaviors covered: see `docs/specs/BDD.md#BDD-001`. Tests: `docs/TESTS.md#BDD-001` is already authored. The matrix row `BDD-001` is `pending` and the Generator fills the test address.

## Non-goals (this slice)

- **Visual fidelity beyond what the existing renderer already does.** The shader stays diffuse-only; PBR preview belongs to a future renderer slice with the material-editing UI (`BDD-005`). New primitives render as flat-shaded white.
- **Per-primitive transform gizmo.** Position is specified in the create-modal as `[x, y, z]` text inputs; dragging in the viewport is `BDD-003`'s job.
- **Behavior assignment UI** (`BDD-006`). New primitives default to `BehaviorType::Float` per the existing planner-resolved decision in `PROJECT_STATE.md`.
- **Other primitive shapes** (cylinder, capsule, plane-as-primitive, etc.). Out of v1 scope (PRD §4 implicitly — only sphere and cube are listed).
- **Touching the Metal kernels, simulation pipeline, or shaders.** This slice is geometry generation + GUI + loader; if any change lands under `src/metal/` or `src/shader/`, the Estimator should flag a backend-boundary violation (`BDD-103`).
- **Resolving any of PRD Q1, Q2, Q4, Q5, Q6.**

## Todo

Ordered. The Generator should execute top-to-bottom.

1. **Author tests first (`docs/TESTS.md#BDD-001` already exists).** Add doctest cases that exercise:
   - `MeshSphereInitializer` produces a mesh with `params.numPoints == expected lat/lon count`, `params.numFacets == 2 * (lat-1) * lon`, every vertex is at distance `≈ size/2` from `params.center` (within FP tolerance), and the adjacency build succeeds (no crash, no zero-area triangles).
   - `MeshCubeInitializer` produces 6 faces × tessellation² quads → `numFacets = 12 * tessellation²`, all vertices on a face have one coordinate equal to `±size/2`.
   - The JSON encode/decode for `{ "type": "primitive", "shape": "sphere", ... }` and `{ "shape": "cube", ... }` round-trips correctly through `scene_format::SceneSnapshot`. Reserved-not-shipped rejection still fires for any *other* unknown shape (regression guard).
   - Fill `docs/TEST_MATRIX.md` row for `BDD-001` with the test addresses.
2. **Implement `MeshSphereInitializer<BE, PR>`** in `src/main.cpp` next to `MeshGridInitializer` (≈ line 1141). UV-sphere parameterization: `tessellation` is the longitude segment count; latitude segments default to `tessellation / 2`. Pole-vertex collapsing is acceptable for v1 (the existing adjacency build handles degenerate edges by inspection — verify with the test). Reuse `MeshAdjacencyInitializer<BE, PR>::initialize` for adjacency.
3. **Implement `MeshCubeInitializer<BE, PR>`** similarly. Six face quads, each subdivided into `tessellation × tessellation` cells (mirroring how `MeshGridInitializer` subdivides one plane). Edges along face boundaries are shared.
4. **Extend `scene_format::PrimitiveSource` and the loader.** `"sphere"` and `"cube"` join `"grid"` in `isKnownShape`. The encoder writes `shape: "sphere"` for `MeshSphereInitializer` and `shape: "cube"` for `MeshCubeInitializer` in `Simulator::toSnapshot`. The loader's primitive branch dispatches to the right initializer based on `shape`. Grid-specific keys (`direction`, `jiggle`) become optional / ignored when `shape != "grid"` — keep them in the JSON for grid only.
5. **Wire ImGui menu.** Add `Create > Sphere…` and `Create > Cube…` items beside `File > Save Scene…` / `Load Scene…`. Each opens a modal asking for size, tessellation, and position; on confirm, calls a new `Simulator::addPrimitive(shape, size, tessellation, position)` helper that wraps `scene.addGeneralMesh` with the right initializer + `BehaviorType::Float` + `FloatBehaviorParams`. Re-use the `simulator.initialize()` re-run pattern from the load handler so the new mesh appears immediately.
6. **Update `docs/design/scene_format.md`.** The "Primitive shape — v1 reality" subsection currently lists `"grid"` as the only shipping shape and `"sphere"`/`"cube"` as reserved. Promote `"sphere"` and `"cube"` to shipping; keep the reserved-not-shipped rule for any future names. The grid-specific keys section is unchanged.
7. **Append a new `D-010` entry to `docs/DECISIONS.md`** documenting the promotion: which initializer ships, what the tessellation parameter means for each shape (lat/lon vs face-cells), and that this amends D-003 without superseding it (the reserved-not-shipped *pattern* still holds for names that aren't yet shipping).
8. **Stop and hand off to the Estimator.** Do not pull in material UI, behavior UI, or transform gizmo even if they look small — they have their own slices.

## Course corrections

- **Persistence + verify-and-warnings slices both shipped** (commits `7cfc491` and `9455861`). The Estimator's last verdict on `verify-and-warnings` was NOTE — no carry-over follow-ups.
- **D-003 amendment, not supersession.** The reserved-but-not-shipped *pattern* is correct. Only the specific membership of the reserved set narrows when a primitive ships. D-010 (when authored) should make this explicit so a future planner reading D-003 doesn't assume `"sphere"`/`"cube"` are still reserved.
- **The persistence slice's parked WARNING** (no app-level `Simulator::saveScene/loadScene` integration test) is **still parked** — this slice does not address it. Q-D in `docs/ARCHITECTURE.md §5` is the gating decision; surface that as the question to resolve when the parked WARNING becomes urgent (likely when Alembic export needs deterministic round-trip verification).

## What to read before writing code

- `docs/specs/FRD.md` FR-001 — functional contract.
- `docs/specs/BDD.md` BDD-001 — user story.
- `docs/TESTS.md#BDD-001` — acceptance scenario; both sphere and cube must be covered.
- `docs/specs/BDD.md#BDD-103` — the backend-boundary invariant; this slice must stay above the Metal line.
- `docs/design/scene_format.md` — the binding schema; the "Primitive shape" subsection is what gets amended.
- `docs/DECISIONS.md` D-003 — the reserved-not-shipped decision this slice partially undoes.
- `src/main.cpp` around `MeshGridInitializer` (≈ line 1141) and the `Simulator::addCloth/addGround/addClothFile/addFloatMesh` helpers (≈ line 4243) — these are the templates for the new initializers and the new `addPrimitive` helper.
- `include/scene_format.hpp:isKnownShape` and `sourceFromJson` — where the loader's known-shape list lives.
- `test/scene_io_test.cpp` — the harness pattern; `BDD-001` tests can live here or in a new `primitive_test.cpp` (Generator's call).
