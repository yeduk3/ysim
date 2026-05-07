# Plan — Import Mesh UI (BDD-002) Slice

> Owner: **Planner** writes; **Generator** executes; **Estimator** judges.
> Updated: 2026-05-07

## Goal

Wire a `File > Import Mesh…` menu item that lets the user load a `.obj` from disk into the running scene, defaulting the new object to `BehaviorType::Float` (per the planner-resolved decision in `PROJECT_STATE.md`). After this slice ships:

- `BDD-002` row in `docs/TEST_MATRIX.md` flips from `pending` to `pass`.
- The user can pick a `.obj` from the existing `assets/` folder via the modal and see it appear in the scene immediately, render with the default white material, and persist through `Save Scene` / `Load Scene`.
- `Block 7` in `runSelfTest` exercises the underlying `Simulator::addFloatMesh(...)` path with a deterministic asset and asserts the loaded mesh produces a positive vertex count + valid AABB after `Simulator::initialize()`.
- `BDD-002`'s `Then` clause that "an invalid or unreadable file produces a clear error and does **not** mutate the scene" is mechanized by an additional self-test assertion that calls `addFloatMesh` with a missing path and confirms the scene's `numMeshes` is unchanged.

## Scope

- `BDD-002` — Import external mesh (FR-002).
- ImGui `File > Import Mesh…` modal — single text-input for path, behavior dropdown stub (defaulted to Float; rest greyed out — behavior-switching is BDD-006's slice). Optional `scale` input. Calls `Simulator::addFloatMesh(prefix, fileName, offset=0, scale, mass=0.1)` then `simulator.initialize()` then `simulator.applyPendingMaterials()` to mirror the Load handler's lifecycle.
- `Block 7` in `runSelfTest` (`src/main.cpp`) for BDD-002's two acceptance scenarios:
  - Happy path: `Simulator::addFloatMesh("assets", "Human.obj", ...)` then init; assert the new mesh exists, has `numPoints > 0`, behavior == `Float`, and source-encoded path round-trips through `toSnapshot()`.
  - Error path: `Simulator::addFloatMesh("assets", "this_file_does_not_exist.obj", ...)`; assert it does **not** crash and the scene state remains internally consistent (or — depending on `ObjReader` behavior — that `numMeshes` is unchanged from before the call). Note: the current `ObjData::loadObject` path may crash hard on missing files; if so, the slice introduces a path-existence check before queuing the request.
- Update `docs/TEST_MATRIX.md` row `BDD-002` from `pending` to `pass` with the test address pointing at Block 7.

## Non-goals (this slice)

- **Native file dialog.** The persistence slice's text-input modal is the established v1 pattern; same shape works here. File-dialog polish is a follow-up across the whole `File >` menu.
- **Multi-format support.** v1 supports `.obj` only (FR-002 + design-doc note). Other extensions hit the same reserved-but-not-shipped error pattern the loader already implements (D-008 area).
- **Behavior selection at import time.** The modal shows a dropdown for visual completeness but it's locked to Float. In-place behavior switching is BDD-006's slice.
- **Material editing on the imported mesh.** Default white baseColor (BDD-005's slice).
- **Resolving CM-005 (cloth-drape tunneling).** Stays parked under the cloth-CCD slice. The harness's failing BDD-007 tunneling clause continues to fail; matrix `warning` row stays.
- **Touching kernels, persistence schema, or render state.** Pure GUI + Simulator-helper wiring.
- **Resolving any of PRD Q1, Q2, Q4, Q5, Q6, Q7.**

## Todo

Ordered. Generator executes top-to-bottom.

1. **Read the spec wording.** `docs/TESTS.md#BDD-002` is binding. The two "Then" clauses are: (a) scene contains a new object whose geometry matches the file, with default material and Float behavior, and the persisted state records the import path so a later save/reload reproduces the same source; (b) an invalid or unreadable file produces a clear error and does not mutate the scene (no partial-add). Author Block 7 from these clauses verbatim — same discipline that fixed the BDD-009/011/012 BLOCK two slices ago.
2. **Inventory the existing import path.** `Simulator::addClothFile` / `addFloatMesh` (`src/main.cpp` ~line 4297 onward) and `MeshFileInitializer` (~line 1238). Confirm:
   - The constructor calls `data.loadObject(prefix, fileName)`. What happens when the file doesn't exist? If `loadObject` aborts or asserts, the slice must add a guard before queuing. If it returns gracefully with `nVertices == 0`, the slice can dispatch the error path naturally.
   - The pre-existing `Simulator::addFloatMesh("src/assets", "Human.obj", ...)` call in `main()` works in production — that's the happy-path baseline.
3. **Wire the ImGui modal.** Add a `File > Import Mesh…` menu item beside `File > Save Scene…` / `Load Scene…` (`src/main.cpp` ~line 5495 area, alongside the `primitiveModal` lambda). Modal fields: `Path` (`InputText`), `Scale` (`InputFloat`, default `1.0`). On `Import`:
   - Compute `prefix` and `fileName` by splitting `Path` at the last `/` (same pattern `loadScene` already uses).
   - Call `simulator.addFloatMesh(prefix, fileName, tinym::vec3(0), scale)`.
   - Call `simulator.initialize()` then `simulator.applyPendingMaterials()`.
   - Report status into `sceneIOStatus` ("imported: path" / "import failed: …").
4. **Error guard.** Before constructing `MeshFileInitializer`, verify the file exists via `std::ifstream(prefix + "/" + fileName).good()` (or equivalent). If missing, write `"import failed: file not found: <path>"` into `sceneIOStatus` and **do not** call `addGeneralMesh` — the scene must be unmodified. The Estimator will check the diff for this guard explicitly per BDD-002's "no partial-add" clause.
5. **Block 7 in `runSelfTest`.** Two assertion blocks:
   - `BDD-002 / .obj import via addFloatMesh appears in scene`: capture `numMeshes` before, call `simulator.addFloatMesh("assets", "Human.obj", tinym::vec3(0), 0.04)`, `simulator.initialize()`, assert `numMeshes` increased by 1, the new mesh's id is valid, `state.x.size > 0` (positive vertex count), behaviorType is `Float`. Optionally serialize via `simulator.toSnapshot()` and assert the new object's `source.kind == Source::Kind::Import` and `source.import.path` ends with `Human.obj` (round-trip clause).
   - `BDD-002 / missing import path leaves scene unchanged`: capture `numMeshes` before, call the same path with a non-existent filename `"missing_obj.obj"`. Assert `numMeshes` is unchanged. (If the existing loadObject crashes the binary outright, this assertion will surface as a `verify.sh` BLOCK that the slice must address by adding the step-4 guard before the call.)
6. **Update `docs/TEST_MATRIX.md`.** Promote `BDD-002` row from `pending` to `pass` with the test address pointing at the new Block 7 names. Use the same pass-string-grep convention as BDD-009/011/012/015.
7. **Run `./scripts/verify.sh` locally.** Build clean, doctest binaries pass, self-test prints the new BDD-002 PASS lines. The pre-existing BDD-007 tunneling FAIL stays — that's CM-005 territory, not this slice.
8. **Refresh `CURRENT_WORK.md` and `RESUME.md`.** Note that BDD-002 closes; the `File > Import Mesh…` menu is now first-class user-facing UI; the path-existence guard at step 4 is load-bearing for BDD-002's "no partial-add" clause.
9. **Stop and hand off to the Estimator.** Do not pile on material/behavior/transform UI. Each is its own slice.

## Course corrections

- **Cloth-drape (BDD-007) slice:** 3 of 4 clauses pass; tunneling clause is parked under CM-005 for a future cloth-CCD slice. Matrix row stays `warning`. The harness's existing FAIL line is *expected* during this slice; do not attempt to "fix" it here.
- **Spec-vs-label discipline still applies.** Block 7 must be authored from `docs/TESTS.md#BDD-002`'s "Then" clauses verbatim, not from the matrix-row label. The persistence-and-env-forces BLOCK from earlier turns was caused by reading from labels.
- **Estimator's host has no Metal.** The harness's SKIP path is the correct behavior for the `addFloatMesh` Block 7 too — if no Metal device, skip. Already in place from D-012.

## What to read before writing code

- `docs/TESTS.md#BDD-002` — binding "Then" clauses.
- `docs/specs/FRD.md#FR-002` — functional contract; v1 is `.obj` only.
- `docs/specs/BDD.md#BDD-002` — user intent; "import a .obj mesh and have it become a normal scene object".
- `src/main.cpp::Simulator::addFloatMesh` (~line 4297), `MeshFileInitializer` (~line 1238), and `ObjData::loadObject` (`include/objreader.hpp`) — to verify the missing-file behavior before writing the guard.
- `src/main.cpp` around the `File >` menu setup (~line 5495 area, in `main()`'s render lambda) — model the new modal on the existing `primitiveModal` / Save/Load patterns.
- `src/main.cpp::Simulator::loadScene` — the path-split + `addGeneralMesh` + `simulator.initialize()` + `applyPendingMaterials()` lifecycle pattern to mirror.
- `src/main.cpp::Simulator::toSnapshot` — confirms imports already serialize as `Source::Kind::Import` with the path round-tripped.
- `.agent/RESUME.md` — `enlargeTrajectory` line is load-bearing (do not re-comment); `cumulativeNarrowCollisions` infrastructure carries forward.
