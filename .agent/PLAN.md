# Plan — Persistence Slice

> Owner: **Planner** writes; **Generator** executes; **Estimator** judges.
> Updated: 2026-05-06

## Goal

Add scene save / load / version-reject to ysim, so that every later v1 slice can rely on a durable, structured, human-diffable representation of a scene.

Concretely: when this slice ships, a user can build a small scene, save it to a `.ysim.json` file, quit the app, relaunch, load the file, and resume from an identical state. An unrecognized format version produces an explicit error rather than silent corruption.

## Scope

- `BDD-014` — save scene to disk (FR-014).
- `BDD-015` — load scene reproduces saved state (FR-015).
- `BDD-016` — reject incompatible scene file version (FR-016).
- The on-disk schema for scenes (the resolution of PRD Q3 — JSON).
- Wiring into the existing ImGui menu surface (`File > Save Scene`, `File > Load Scene`).

Behaviors covered: see `docs/specs/BDD.md#BDD-014`, `#BDD-015`, `#BDD-016`. Tests: see `docs/TESTS.md` blocks of the same ids; matrix rows already exist in `docs/TEST_MATRIX.md`.

## Non-goals (this slice)

- Saving simulation results / per-frame caches. That's the **export slice** (`BDD-013`), separate.
- Embedding mesh data in the scene file. v1 references imported meshes by path (decision in `PROJECT_STATE.md`).
- Format migration. v1 has exactly one format version; v1→v2 migration is a future slice. The version field exists *only* so v1 can refuse to load anything else.
- Material export through the scene file's renderer side — material parameters are just data; no rendering changes here.
- Touching the simulation kernels, the Metal shaders, or the renderer. If this slice modifies anything under `src/metal/` or `src/shader/`, the Estimator should flag a backend-boundary violation (`BDD-103`).
- Resolving any of PRD Q1, Q2, Q4, Q5, Q6.

## Todo

Ordered. The Generator should execute top-to-bottom. Each item is concrete enough that no re-planning is needed.

1. **Read the schema.** Schema is fully specified in `docs/design/scene_format.md` — top-level shape, per-object shape, transform/material/behavior schemas, error-behavior table, save semantics. Implement to match it exactly; do not improvise. If a field is ambiguous, stop and ask the Planner — do not silently choose.
2. **Add the JSON library.** Drop `nlohmann/json.hpp` (single-header) into `include/` alongside `stb_image.h`. Record the choice in `docs/DECISIONS.md` (this is your first entry — number it `001` and cite `BDD-014`/`BDD-015`/`BDD-016`).
3. **Implement save (`BDD-014`).** Add a `saveScene(path)` entry point that walks the in-memory scene (objects, transforms, materials, behavior tags + params, global forces) and writes the JSON described in step 1. The function lives next to the scene-management code in `src/main.cpp` for now (consistent with the project's "scene logic in main.cpp" convention per `CLAUDE.md`).
4. **Implement load (`BDD-015`).** Add a `loadScene(path)` entry point that parses the JSON, validates `format_version == 1` (else: error per `BDD-016`), and reconstructs the scene state. Use the existing `addGeneralMesh`, primitive-creation, and import paths rather than building a parallel construction pipeline — this enforces that loading produces the same in-memory shape as authoring.
5. **Implement version-reject (`BDD-016`).** When `format_version` is missing, unknown, or not `1`, the loader returns an error result without mutating the current scene. The error message names the version it found vs. the version it expected.
6. **Wire into ImGui.** Add `File > Save Scene…` and `File > Load Scene…` items. Use a native file dialog if one is already present in the project; otherwise a minimal text-input modal is acceptable for v1 (file dialog polish is a follow-up).
7. **Tests.** Implement the three scenarios from `docs/TESTS.md`:
   - `BDD-014` — author a populated scene, save, assert file exists with expected structural keys + `format_version: 1`.
   - `BDD-015` — save a populated scene, fresh-load it, assert in-memory state field-by-field equals pre-save state; run one simulation step and assert output equals the pre-save first-step output.
   - `BDD-016` — write a file with `format_version: 999`, attempt to load, assert load fails with an error naming the version mismatch and the in-memory scene is unchanged. Repeat with the version field omitted.
   Fill the corresponding rows in `docs/TEST_MATRIX.md` with the test addresses and update status to `pass`/`fail`.
8. **Stop and hand off to the Estimator** before scoping any further capability into this slice. Material UI, behavior UI, gravity/wind sliders, etc., are *separate* slices — do not pull them in even if they look small.

## Course corrections

(None yet — this is slice #1.)

## What to read before writing code

- `docs/design/scene_format.md` — **the schema is binding**, not a sketch.
- `docs/ARCHITECTURE.md` §4 — boundaries you must not cross (especially §4.1 backend boundary, §4.4 enum identifiers reserved).
- `docs/CONVENTIONS.md` — file naming, type/function casing, commit message style.
- `docs/specs/FRD.md` FR-014, FR-015, FR-016 — the functional contract.
- `docs/specs/BDD.md` BDD-014, BDD-015, BDD-016 (slice work) and **BDD-103** (the invariant the Estimator will check — keep the diff narrow).
- `docs/TESTS.md` blocks for those four BDDs — these are the acceptance scenarios you must turn into doctest cases.
- `src/main.cpp` around the `Scene<METAL, PR>` and `GeneralMesh<BE, PR>` definitions (≈ lines 1530–1650), the `BehaviorType` enum (line 794), and the behavior parameter structs (line 1382 onward) — these define the in-memory shape you are serializing.
