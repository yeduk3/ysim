# Decisions

> Owner: **Generator** (appends). Read by everyone. Numbered, append-only — never reorder.

A decision belongs here when a future reader could not derive *why* from the code alone. If the rationale is "obvious from the API docs", it does not belong here.

## Format

```
## D-NNN — <short title>

- File / function: <path>:<symbol>
- Decision: <what was chosen>
- Alternatives considered: <briefly>
- Rationale: <why this one — constraints, tradeoffs, prior incident>
- Date: <ISO date>
```

If a later decision supersedes an earlier one, add the new entry and reference the old one (`Supersedes D-007`). Do not edit or delete the old entry.

## Entries

## D-001 — Vendor `nlohmann/json` as the scene file format library

- File / function: `include/nlohmann/`, `include/scene_format.hpp`, `src/main.cpp:saveScene`/`loadScene`
- Decision: vendor `nlohmann/json` (v3.12.0) into `include/nlohmann/` rather than depending on a system / brew install. JSON is the on-disk format for `.ysim.json` (`BDD-014`/`BDD-015`/`BDD-016`).
- Alternatives considered: a hand-rolled JSON writer (less code dependency, but reinvents validation and number-precision handling that the design doc explicitly leans on); CBOR / a binary format (the design doc explicitly rejects this for v1 because the file is small and human-diffable matters); brew-only dependency (breaks the project's "drop a single-header into `include/` like `stb_image.h`" pattern recorded in `docs/CONVENTIONS.md`).
- Rationale: matches the binding choice in `docs/design/scene_format.md` and the precedent of `include/stb_image.h`. Vendoring keeps the repo build-from-clone reproducible and avoids a brew prerequisite for contributors. Cost is ~1 MB of headers under `include/nlohmann/`, which is acceptable for v1.
- Date: 2026-05-06

## D-002 — Doctest as v1 test framework, vendored as a single header

- File / function: `include/doctest.h`, `test/scene_io_test.cpp`, `test/CMakeLists.txt`
- Decision: use **doctest** (vendored at `include/doctest.h`) as the v1 test framework. The persistence slice creates the first `test/` directory and wires a separate CMake test executable.
- Alternatives considered: GoogleTest (heavier dependency, requires CMake `FetchContent` / submodule); Catch2 (similar weight to doctest but slower to compile); writing tests inline in the main binary (couples test build to the GLFW/Metal/OpenGL toolchain, which is what the persistence-slice tests are explicitly trying to avoid).
- Rationale: doctest is single-header, fast to compile, and matches the project's "drop a single-header into `include/`" pattern (`stb_image.h`, `nlohmann/json`). A separate test executable lets persistence tests run without needing a Metal device or a GL context — important because the persistence layer is supposed to be backend-independent (`docs/ARCHITECTURE.md §4.1`).
- Date: 2026-05-06

## D-003 — `"grid"` is the v1 primitive shape; `"sphere"`/`"cube"` reserved-but-not-shipped

- File / function: `include/scene_format.hpp:Source`, `src/main.cpp:loadScene`
- Decision: the v1 schema's primitive `shape` field accepts `"grid"` (the only primitive-construction path the engine actually has — `MeshGridInitializer`) and treats `"sphere"`, `"cube"` as known-but-unsupported tags that the loader rejects with a clear "shape X not available in this build" error. This matches the same "reserved-but-not-shipped" pattern the design uses for `BehaviorType::Rigid`/`Elastic`/`Fluid`/`Generator`.
- Alternatives considered: refuse to ship persistence until a sphere primitive exists (couples the persistence slice to a future authoring slice — exactly what the slice plan tries to avoid); silently translate `"sphere"` to a placeholder grid (silent corruption, the failure mode the version-reject behavior `BDD-016` exists to prevent).
- Rationale: the design doc lists `"sphere"|"cube"` because the *PRD* commits to those primitives. v1 does not yet have them — `BDD-001` is a separate, future slice. To keep the persistence schema additive ("loaders ignore unknown top-level keys to support additive evolution"), v1 declares `"grid"` as the shape it actually supports today, and the reserved tags fail loud rather than silent.
- Date: 2026-05-06

## D-004 — Per-object transform lives in the initializer, not on `GeneralMesh`

- File / function: `src/main.cpp:GeneralMesh`, `MeshGridInitializerParams::center`, `MeshFileInitializerParams::offset`, `saveScene`/`loadScene`
- Decision: the v1 scene format's `transform.position` is read on save from the initializer's `center` (grid) / `offset` (file), and on load passed back into the initializer the same way. `transform.rotation` round-trips as a unit quaternion stored in the JSON, but is **not** plumbed into a `GeneralMesh` field — there is no rotation field today, and the persistence slice does not introduce one. Rotation defaults to identity `[1,0,0,0]` on save when no rotation is authored, and is read+renormalized on load but not yet applied.
- Alternatives considered: add a `Transform { vec3 position; vec4 rotation; }` to `GeneralMesh` and update every consumer (rendering, sim) to read it (out of scope for the persistence slice — would touch the renderer / hot path, violating `BDD-103`); store `transform` only in the JSON layer with no in-memory mirror (loses round-trip determinism the moment a rotation gizmo is wired up).
- Rationale: the existing in-memory representation already carries position inside the initializer struct — `center` for grids, `offset` for file imports — so reading from there is canonical, not invented. Rotation has no consumer in v1, but the design doc binds the on-disk schema to include it; the slice persists the field so a future rotation slice (`FR-004`) only has to wire the existing JSON value into a new `GeneralMesh` field, without breaking saved files.
- Date: 2026-05-06

## D-005 — `Material` struct extended to the OpenPBR v1 subset

- File / function: `src/main.cpp:Material`
- Decision: extend `Material` from `{ vec3 baseColor }` to the full OpenPBR v1 subset declared in `docs/design/scene_format.md` (`baseColor`, `metallic`, `roughness`, `specularWeight`, `emissionColor`). The renderer continues to read only `baseColor` in v1; the additional fields exist so save/load round-trip the values without losing them.
- Alternatives considered: keep `Material` as `baseColor`-only and store the OpenPBR fields in a parallel sidecar map keyed by mesh id (forces every future material consumer to look in two places); fold the OpenPBR fields into the JSON-only layer (loses round-trip the moment a material UI lands).
- Rationale: matches the design's "all keys required in v1" rule. The fields are POD floats — adding them costs nothing on the hot path. Out-of-range values clamp (per design's error-behavior table) rather than reject; `clampMaterial` lives next to the struct so the clamp is in one place.
- Date: 2026-05-06

## D-006 — Global `SceneEnvironment` (gravity, wind) introduced as a static singleton in `Scene`

- File / function: `src/main.cpp:Scene<BE,PR>::environment`
- Decision: introduce a `SceneEnvironment { tinym::vec3 gravity; tinym::vec3 wind; }` as a static member of `Scene<BE, PR>`, defaulting to `gravity = (0, -9.81, 0)` and `wind = (0, 0, 0)`. The persistence slice reads from / writes to this value; no simulation kernel consumes it yet (`FR-011`/`FR-012` are future slices).
- Alternatives considered: thread environment forces through every per-mesh `ExternalForces<BE,PR>` (already exists per-mesh, but is not currently populated by anything global — would commit the persistence slice to wiring up gravity-application, which is `FR-011`'s job); store gravity/wind in `main.cpp`'s outer scope as plain globals (loses the `Scene<BE,PR>` static-singleton convention used for `numMeshes`, `meshes`, `dirty`, `packedMeshData`, etc.).
- Rationale: the schema requires `environment.gravity` and `environment.wind` at the top level; we need a place for them to live in memory so save/load round-trip. Putting them as static members of `Scene` mirrors the existing `inline static` convention in this file and keeps "global scene state" in one container. The defaults match the schema defaults so an existing scene with no environment authored saves the canonical defaults instead of zeros.
- Date: 2026-05-06

## D-007 — Per-object rotation lives on `GeneralMesh` as `rotationQuat` (supersedes D-004)

- File / function: `src/main.cpp:GeneralMesh::rotationQuat`, `src/main.cpp:Simulator::toSnapshot/loadScene/applyPendingMaterials`
- Decision: add a `Quat { float w, x, y, z }` field `rotationQuat` to `GeneralMesh`, defaulting to identity. `toSnapshot` reads it; `loadScene` stashes the loaded quaternion into a `pendingRotations` side-table keyed by id and `applyPendingMaterials` paints it onto realized meshes (alongside material). Renderer/sim still ignore the field — adding the storage does not introduce a new consumer and so does not violate `BDD-103`.
- Supersedes: **D-004** (which kept rotation in the JSON layer only). The Estimator flagged the prior approach as a `BDD-014`/`BDD-015` round-trip violation: `toSnapshot` was always emitting identity, and `loadScene` had nowhere to put the parsed quaternion. The persistence slice fundamentally needs an in-memory mirror; deferring it to a future "rotation slice" loses the round-trip property.
- Alternatives considered: a separate parallel array keyed by mesh id (already covered by `pendingRotations` for the load path, but for the *save* path a side table doesn't survive `Scene::pack()` cleanly — putting the field on the mesh removes the bookkeeping); reusing `tinym::vec4` (its component order is `(x, y, z, w)`, opposite of the schema's `[w, x, y, z]` — easy off-by-one trap).
- Rationale: matches the design doc's binding "transform.rotation = [w, x, y, z]" requirement and FR-014/FR-015's obligation that rotation round-trips. Cost is one `Quat` per mesh — 16 bytes. Future rotation slices wire `rotationQuat` into rendering / sim without touching the persistence layer.
- Date: 2026-05-06

## D-008 — Import paths resolve relative to the scene file's directory

- File / function: `include/scene_format.hpp:sceneDir/resolveImportPath`, `src/main.cpp:Simulator::loadScene`
- Decision: on load, the loader computes the directory portion of the scene-file path and joins it with each `objects[i].source.import.path` whenever that path is relative. Absolute paths pass through unchanged. The resolution function lives in the header so it is exercised by `test/scene_io_test.cpp` without any GUI/Metal harness.
- Alternatives considered: pass the full path to `MeshFileInitializerParams.fileName` and let `objreader` figure it out (the reader currently expects `prefix + '/' + fileName`, so the join has to happen somewhere); resolve relative to the application's `cwd` (breaks portability the moment the user `cd`s elsewhere — the exact failure mode the design doc cites).
- Rationale: matches `docs/design/scene_format.md`'s binding rule "interpreted relative to the scene file's directory". Estimator flagged the prior implementation as a spec violation. Helper signature `resolveImportPath(dir, importPath)` is the smallest API that lets the test cover the four cases (rel + abs × dir-with-slash + dir-without-slash).
- Date: 2026-05-06

## D-009 — Material clamping surfaces a `LoadWarnings` channel on the snapshot

- File / function: `include/scene_format.hpp:LoadWarnings`, `clampInPlace`, `materialFromJson`, `SceneSnapshot::warnings`
- Decision: `clampInPlace(Material&)` returns `bool` indicating whether any field was clamped; `materialFromJson` records a warning into a `LoadWarnings* warnings` out-parameter when clamping triggers. `SceneSnapshot` carries a `LoadWarnings warnings` member so callers (`Simulator::loadScene`, the test harness) can either log to stderr or assert against the messages. Clamping does **not** fail the load.
- Alternatives considered: print directly to stderr from inside `clampInPlace` (forces a `stdio` dependency on the persistence layer and makes it untestable without redirection); upgrade clamp violations to `LoadError` (overkill — the design doc says "warning, not reject" for material values).
- Rationale: matches the design doc's "the loader logs a warning and clamps" rule. A separate channel keeps soft warnings out of the binary `Result.ok` flag, so callers can choose to surface them or not. Test harness asserts on `warnings.empty()` to verify the design's contract.
- Date: 2026-05-06
