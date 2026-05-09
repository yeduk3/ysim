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

## D-010 — `"sphere"` and `"cube"` ship as v1 primitives (amends D-003)

- File / function: `include/primitive_geometry.hpp`, `include/scene_format.hpp:isKnownShape/isReservedShape`, `src/main.cpp:MeshSphereInitializer`/`MeshCubeInitializer`/`Simulator::addSphere`/`addCube`/`toSnapshot`/`loadScene`, `docs/design/scene_format.md` "Primitive shape — v1 reality"
- Decision: `"sphere"` and `"cube"` join `"grid"` as known primitive shapes the loader accepts, and ship as real `MeshSphereInitializer` / `MeshCubeInitializer` runtime initializers driven by a CPU-pure geometry header (`include/primitive_geometry.hpp`). The reserved-but-not-shipped *pattern* survives unchanged; only the membership of the reserved set narrows to empty for v1. Future shapes that aren't yet shipping (e.g. capsule, cylinder) opt back into the same loud-fail behavior by joining `isReservedShape`.
- Amends: **D-003** (which placed `"sphere"`/`"cube"` in the reserved set when only `"grid"` shipped). D-003 stays as the historical record of the pattern; D-010 records the membership change. This is **not** a supersession because the rule itself — "loud-fail unknown / reserved names" — is unchanged.
- Alternatives considered: ship sphere/cube as a half-implementation that only encodes/decodes the JSON without runtime initializers (the persistence-side test passes but the GUI's `Create > Sphere` wouldn't actually produce geometry — exactly the silent-corruption failure mode the reserved-not-shipped pattern was designed to prevent); merge the geometry generators into `src/main.cpp` directly (couples the test executable to the GUI / Metal toolchain and breaks the CPU-pure test policy from D-002).
- Rationale: this slice's tests need to verify vertex/facet/edge counts and surface invariants without a Metal device, so the geometry generators live in a header (`primitive_geometry.hpp`). The `tessellation` parameter has different meaning per shape (lat/lon for sphere, face-cells for cube); test coverage records the interpretation against `BDD-001`'s acceptance scenario. Edge counts are derived in closed form (`sphereEdgeCount` via Euler, `cubeEdgeCount` via per-face planar Euler ×6) so `MeshSphereInitializerParams` and `MeshCubeInitializerParams` can pre-allocate without a `std::set` walk.
- Date: 2026-05-06

## D-011 — Render-side GL state (`MeshGL`) moved out of `GeneralMesh`

- File / function: `include/MeshGL.hpp`, `include/MeshRenderState.hpp`, `src/main.cpp:GeneralMesh` (field removed), `src/main.cpp:Simulator::draw`/`update`/`initialize` (rerouted through `Simulator::renderState`)
- Decision: relocate `MeshGL<CPU>` out of `src/main.cpp` and into `include/MeshGL.hpp`. Drop the `MeshGL<CPU> meshGL` member from `GeneralMesh<BE, PR>` and remove the `meshGL = MeshGL<CPU>(…)` line from `GeneralMesh::initialize`. Renderer-side state now lives in a `MeshRenderState` instance owned by `Simulator`, keyed by `mesh.id`. `Simulator::draw` and the per-frame buffer-upload path call `renderState.getOrCreate(mesh)` instead of `mesh.meshGL`. `Simulator::initialize` calls `renderState.clear()` after `Scene::pack()` because the cached `MeshGL` captured raw pointers into the old packed buffers.
- Alternatives considered: keep `MeshGL` on `GeneralMesh` and gate construction on a flag (forces every call site to know whether GL is "live"; spreads the GL coupling rather than removing it); use a global registry instead of a `Simulator` member (loses scoped lifetime — re-loaded scenes would leak entries from the old session); make `MeshRenderState` own a `MeshGL*` pool and have `GeneralMesh` hold a non-owning pointer (defers the GL dependency by one indirection but doesn't drop it).
- Rationale: the structural cause of CM-002, CM-003, CM-004 (and the parked persistence app-level WARNING) is that `Simulator::initialize` cannot run without an OpenGL context, so unit tests cannot exercise the simulation path that produces those bugs. Moving GL state to a renderer-side container removes the structural blocker. The slice that lands the actual Metal-backed test fixture (next milestone in `PROJECT_STATE.md`) depends on this decoupling. `BDD-103` is *strengthened* — `ARCHITECTURE.md §2.2` says scene-model state belongs to `GeneralMesh`; `§2.3` says GL state belongs to the renderer; they had been mixed.
- Date: 2026-05-07

## D-012 — Q-D resolved: headless Metal harness, not CPU backend reference

- File / function: `src/main.cpp::runSelfTest`, `scripts/verify.sh`, `docs/ARCHITECTURE.md §5 Q-D`
- Decision: the test backend for v1 is the **same Metal device the shipping binary uses**, exercised via a `--self-test` mode added to the `ysim` binary. The CPU backend stays as type-system reservation only — no CPU implementations of force/BVH/integrate kernels for v1. The Estimator's `scripts/verify.sh` runs `./src/ysim --self-test` from cwd=`build/` after the doctest binaries; pass/fail surfaces via exit code + per-assertion `[self-test PASS]` / `[FAIL]` stderr lines.
- Alternatives considered: (a) flesh out `Scene<CPU, PR>` with CPU implementations of force/BVH/integrate so a CPU-only doctest binary can exercise sim — multi-slice undertaking, doubles the surface area to maintain, and the result tests a non-shipping path; (b) extract main.cpp's types into a static library so a *separate* test binary can `#include` them and instantiate `Simulator<METAL,PR>` — multi-slice refactor of a 5200-line TU; (c) keep the gap parked — three successive slices ended in `manual user test catches Metal-side bug` (CM-002, CM-003, CM-004), the cycle is not sustainable.
- Rationale: v1 is macOS-only (`docs/ARCHITECTURE.md §4.5`); Metal availability is guaranteed on the dev/CI machines. Reusing the shipping binary means the harness exercises the *exact* code path users hit. Adding an argv branch to `int main()` is a tiny diff vs (a) or (b). Pass/fail via exit code is enough granularity for CI; if richer assertions become valuable, a future slice can extract types and switch to doctest. The render-state decoupling slice (D-011) is the structural prerequisite — without it `Simulator::initialize` would still need a window. This slice also lifts the per-frame mesh-buffer upload out of `Simulator::update` into a new `Simulator::uploadMeshes` (called by the GUI loop, not by the harness), removing the last GL coupling on the simulation hot path.
- Date: 2026-05-07

## D-013 — Narrow phase upgraded from snapshot to swept-segment CCD

- File / function: `src/metal/bruteforce.metal::narrow_pt_tri`, `src/main.cpp::Scene::pack`/`MeshState`/`PackedMeshData::xPrev`, `src/main.cpp::Simulator::update` (per-substep `xPrev` snapshot), `src/main.cpp::BruteForce::narrow` (binds `xPrev` at slot 10).
- Decision: replace the prior snapshot point-vs-triangle distance test with a swept-segment-vs-triangle CCD that takes the segment from `xPrev` (start of the prior substep, snapshotted before integrator runs) to the current `state.x`. Trigger conditions: the segment crosses the triangle plane (`d_prev` and `d_cur` straddle zero) **OR** `d_cur < radius + thickness` (snapshot proximity fallback for slow particles already touching). The kernel writes the **signed** current distance `d_cur` into `NarrowCollision::collisionNormalAndDistance.w` — the prior code abs'd it, which inverted the integrator's `(thickness - distance) * n` push for tunneled particles. New `n` orientation rule: outward toward `xPrev`'s side (no spurious flip based on `l < 0`).
- Alternatives considered: predictive sweep where the kernel internally computes `x_pred = x + v * subh` (no `xPrev` buffer; one-substep-earlier response, no lag) — rejected because the integrator's response already runs *after* the force-update, so a one-substep lag from `xPrev` is acceptable and the buffer makes the segment endpoints unambiguous to a future reader; a continuous-time response that solves the actual contact-time inside the integrator (much more complex, deferred); raising `Simulator::radius` to widen the snapshot band (coarse, more false positives, doesn't fix tunneled particles' negated push).
- Rationale: closes CM-005's tunneling clause for `BDD-007` (was 3.6m tunnel; with this slice plus `subSteps=8` in the harness, full PASS — peak penetration is below the BDD-007 `cloth-thickness` threshold). The `enlargeTrajectory(system.subh)` from D-013-pre's predecessor (`feat/cloth-drape`) is a *prerequisite* — without trajectory-inflated AABBs, broad phase doesn't even feed the pair into narrow. Both are load-bearing.
- Date: 2026-05-07

## D-014 — `GeneralMesh::transformPosition` is the world-space center mirror; `Simulator::translateObject` mutates `state.x` (and `state.xPrev`) in place

- File / function: `src/main.cpp::GeneralMesh` (new field `transformPosition`), `src/main.cpp::Scene::pack` (pack-time seeding via `dynamic_cast` cascade over the four initializer subtypes), `src/main.cpp::Simulator::translateObject`, `src/main.cpp::Simulator::toSnapshot` (override path on the realized-mesh branch), `include/MeshInspectorWindow.hpp` (`MeshInspectorTarget::transform_position`, `on_translate`), `src/mesh_inspector_gui.cpp::drawMeshInspectorWindow` (Transform collapsing header + `InputFloat3("Position")` + `IsItemDeactivatedAfterEdit` commit gate).
- Decision: BDD-003's "translate a selected object" is implemented by mutating `state.x` (and `state.xPrev`) by `delta = newPos - mesh.transformPosition` once on commit, then writing back `mesh.transformPosition = newPos`. Velocity (`state.v`) is unchanged — translating does not reset physics. `transformPosition` is seeded at pack-time from the initializer's `center`/`offset` so existing meshes preserve their author intent through the rename, and `toSnapshot` reads `transformPosition` (when set) instead of the initializer's `center`/`offset` for the realized-mesh path so saveScene/loadScene round-trips translates.
- Alternatives considered: (a) introduce a per-mesh model matrix and let the renderer apply it without touching `state.x` — much larger renderer rework (every shader, every BVH-against-positions code path, every position-consuming kernel buffer). v1 renders directly from `state.x` and the simulation reads positions from there; injecting a transform layer would re-architect the boundary. Rejected as much-larger-than-the-slice-this-closes. (b) Mutate the initializer's `center`/`offset` instead of adding `transformPosition` — initializers are the deserialized representation; mutating them mid-simulation would conflict with re-pack flows that re-instantiate from initializer params. (c) Reset `state.v` on translate (treat it like a teleport with physics restart) — contradicts BDD-003's "next simulation step uses the new position" wording, which implies physics carries forward.
- Rationale: D-007 set the precedent (rotation lives on `GeneralMesh` as a real field, not on the initializer). The same shape applies here. `xPrev` parity with `x` is load-bearing because D-013's swept-CCD reads `xPrev` as the start-of-substep position; if `xPrev` doesn't move with `x`, the next narrow phase sees the user-driven move as a 1m+ swept segment and emits spurious tunneling contacts. The `MeshRenderState` (D-011) reads through `mesh.state.x.ptr` each frame via `uploadMeshes`, so writing through that pointer is what the renderer naturally picks up — no separate invalidation hook is needed.
- Date: 2026-05-07

## D-015 — `Simulator::translateObject` writes the new center back into the initializer's `params.center`/`offset` so re-pack reproduces the translate

- File / function: `src/main.cpp::Simulator::translateObject` (write-back cascade), with the symmetric reads in `src/main.cpp::Scene::pack` (pack-time seeding) and `src/main.cpp::Simulator::toSnapshot` (persistence override).
- Decision: after translateObject mutates `state.x` / `state.xPrev` / `transformPosition`, it also writes `newPos` into `mesh->initializer->params.center` (or `params.offset` for `MeshFileInitializer`) via the same `dynamic_cast` cascade pattern used in pack and toSnapshot. The cascade has four arms today: `MeshGridInitializer`, `MeshSphereInitializer`, `MeshCubeInitializer`, `MeshFileInitializer`. When a fifth initializer subtype ships (e.g., the future Rigid initializer for FR-008), all three sites — Scene::pack pack-time seed, Simulator::toSnapshot encode override, Simulator::translateObject write-back — need the corresponding case added in tandem. This is a structural invariant, not a coincidence; reviewers touching any one of the three should grep for the other two.
- Alternatives considered: (a) capture `transformPosition` in a side-table that survives `meshes.clear()` in pack (mirroring `pendingRotations`'s pattern from D-007). Rejected because `pendingRotations` exists only because rotation isn't carried by the initializer; position *is* (initializer holds the authored center). Adding a side-table here would duplicate state. (b) make pack prefer `mesh.transformPosition` over `initializer.center` when re-packing. Rejected because `meshes.clear()` happens before the rebuild; the realized mirror is gone by then. (c) freeze the initializer's `params` as immutable post-construction. Rejected as a much larger refactor; the four `params` structs aren't designed to be const-after-init and are mutated by load paths already.
- Rationale: closes Estimator turn-7 WARNING (a). The bug surfaced because `Simulator::initialize()` is called by every create/import/load flow (not just startup), and each call triggers `Scene::pack()` which reads from the initializer. Without the write-back, translate-edit followed by primitive-add (a common UI sequence) silently undoes the edit. Block 9 clause (d) in `runSelfTest` mechanizes the round-trip; bug-probe (commenting the cascade out) confirms the assertion FAILs without the fix and PASSes with it.
- Date: 2026-05-09

## D-016 — Cloth integrator's vn-zero and position-push share the `(distance < thickness)` gate; narrow-phase detection band stays asymmetrically wider

- File / function: `src/metal/physics.metal::integrate_cloth` (contact-loop body around line 325-339), `src/metal/physics.metal::integrate_cloth_grid` (mirror image around line 199-213), `src/main.cpp::Simulator::update` (passes `margin` to `narrowAndSortByVertices`), `src/main.cpp::BruteForce::narrow` (writes `nparams.thickness`).
- Decision: in both cloth integrator kernels, the response loop now reads as:
  ```
  float distance = vertColFacets[i].collisionNormalAndDistance.w;
  float thickness = clothParams.thickness;
  if (distance < thickness) {
      if (vn < 0.0f) vel -= vn * n;
      pos += (thickness - distance) * n;
  }
  ```
  The vn-zero mutation is now gated by the same `(distance < thickness)` condition that gates the position-push. Detection (`narrow_pt_tri`'s `inMargin` band of `radius + thickness`) is intentionally **wider** than response (`distance < thickness`) — this asymmetry is load-bearing and is why a non-zero `nparams.thickness` is now safe at the call site (`simulator.margin = 0.015`). Per-mesh cloth thickness plumbing is deferred; `simulator.margin` is the v1 global fallback.
- Alternatives considered: (a) gate vn-zero on `distance < (thickness + ε)` for some small ε — rejected as adding a tunable without justification; the cleanest invariant is "gates match position-push's gate exactly." (b) keep vn-zero unconditional and have the position-push compensate with a larger constant — rejected because a uniform "always damp normal velocity on any narrow contact" overcorrects approaching particles long before they reach the surface. (c) shrink the narrow-phase detection band to match the integrator's response gate — rejected because the wider detection band is part of D-013's swept-CCD design (catches plane-crossings whose `d_cur` is large but whose segment swept through the surface).
- Rationale: closes CM-006 (graduated to OLD_MISTAKES.md). The asymmetry between vn-zero and position-push was an artifact of D-013's swept-CCD upgrade: that slice introduced the new contact semantics (CCD fires on plane-crossing regardless of `d_cur`) but only retrofitted the position-push to gate on `(distance < thickness)`. The vn-zero kept its old unconditional shape because it was working under the old detection rule. This decision closes that asymmetry. With the kernel-side gate in place, the call site now passes `simulator.margin` (was clamped to `0` in the prior slice for safety) so the slow-touch fallback band is the proper `radius + thickness` the cloth-CCD turn-6 plan originally called for. BDD-007's tunneling clause stays PASS deterministically across 5 runs after the change.
- Date: 2026-05-09

## D-017 — `ProfilerFrameGate` is the shared profiler-pause gate consumed by both production and the test harness

- File / function: `include/FrameProfiler.hpp::profiler::ProfilerFrameGate` (new RAII guard), `src/main.cpp` render loop (~line 6328 begin / ~line 6620 explicit close), `src/main.cpp::runSelfTest` Block 10 clause (c) (`BDD-019 / history collection pauses when sim pauses`).
- Decision: the production render loop's `if (!simulator.pause) { beginFrame; ...; endFrame; }` pattern is bundled into a tiny RAII guard. Constructor takes a `bool collect` predicate and `(sequence, wall_time)`; calls `beginFrame` only when `collect == true`. `close()` is symmetric: only calls `endFrame` if `collect && !closed`. Initial `closed_ = !collect` ensures `endFrame` is never orphaned. The destructor calls `close()` as a safety net; production calls it explicitly because `endFrame` must run before the window-title read at line 6624. Block 10 clause (c) constructs the guard with the **same predicate** (`!sim.pause`) and asserts that under `sim.pause = true` no snapshot is pushed — exercising the actual gate predicate, not a proxy.
- Alternatives considered: (a) a free function taking a lambda (`profiler::runFrame(profiler, paused, seq, t, body)`) — rejected because the render loop's body spans 290+ lines of mixed profiler/non-profiler work and includes an explicit `endFrame` ordering requirement (must run before the window-title read); a single-lambda wrapper bloats the diff and complicates the body's flow. (b) a free predicate function (`shouldCollectFrame(paused) -> bool`) — rejected as too thin; it shares the predicate but doesn't bundle the begin/end pair, so a future refactor that drops one half of the pair without the other could pass the predicate test silently. The RAII guard owns the pairing. (c) leave Block 10 clause (c) as a proxy — rejected because Estimator turn-8 specifically called this out as a real risk: a regression in the production predicate would not break the harness.
- Rationale: closes Estimator turn-8 WARNING. Bug-probe verified: changing the harness's `collect` argument from `!sim.pause` to a hardcoded `true` makes clause (c) FAIL with "ProfilerFrameGate(collect=false) still pushed a snapshot"; restoring to `!sim.pause` flips it back to PASS. The harness now drives the same gate object production uses, so a future regression in the predicate breaks the harness immediately rather than waiting for human GUI testing. Pass label wording stays unchanged so the matrix-row test address does not need updating.
- Date: 2026-05-09

## D-018 — `MeshGridInitializer` jiggle uses a per-mesh seeded `std::mt19937`; seed is derived from `mesh.id`

- File / function: `src/main.cpp::MeshGridInitializerParams` (new `uint32_t seed` field), `src/main.cpp::MeshGridInitializer::initialize` (`std::mt19937 rng(params.seed)` + `std::uniform_real_distribution`), `src/main.cpp::Simulator::addCloth` (passes `Scene::numMeshes` pre-call as the seed = the about-to-be-assigned id), `src/main.cpp::loadScene` (passes `o.id` as the seed).
- Decision: replace global `rand()/RAND_MAX/10000.f` with a local `std::mt19937(params.seed)` driven by `std::uniform_real_distribution<PR>(PR(0), PR(1.0/10000.0))`. The seed is `mesh.id` — for new meshes via `addCloth`, that's the value `Scene<BE,PR>::numMeshes` will be assigned next (read pre-call); for loaded scenes via `loadScene`, that's `o.id` from the saved scene format. **No scene-format change is needed** because `o.id` is already serialized; load-time reproducibility falls out for free. The RNG is local to the `initialize()` call so it cannot leak global state.
- Alternatives considered: (a) hardcoded constant seed (e.g., `0xC0FFEE`) for all cloths — rejected because two cloths in one scene would share the same jiggle pattern, which is visually awkward; per-mesh-id seeding gives different patterns at zero extra cost. (b) persist post-jiggle `state.x` in saved scenes — rejected as scope creep; mesh.id-derived seeding gives load reproducibility without touching the scene format. (c) harness-only `srand(0)` pre-seed in Block 11 — rejected because it doesn't fix production: scenes with jiggle would still be non-bit-deterministic for users, only the harness would pass. (d) `std::random_device` or `time()`-based seeding — rejected as the opposite direction; non-deterministic is exactly what we're closing.
- Rationale: closes CM-007 (graduated to OLD_MISTAKES.md) and unblocks BDD-102's bit-equality assertion in Block 11. Same-binary-same-machine runs of the same scene now produce bit-identical state.x for all 30 frames, deterministic across 5 consecutive `--self-test` invocations. **Invariant for future initializer subtypes:** any new initializer that introduces randomness (e.g., a future Rigid initializer's debris scatter, a Perlin-noise displacement) must add a `seed` field to its params struct and wire it from the same call sites that set `mesh.id` — `addCloth`/`addSphere`/`addCube`/`addGround`/`importMesh`/`loadScene`. The `Scene::numMeshes` pre-call read is the canonical source for new-mesh paths; `o.id` is canonical for load paths. **Cross-build determinism is explicitly NOT in scope** (PRD §6) — `std::uniform_real_distribution` is implementation-defined across libstdc++ versions, so a different toolchain build could produce a different jiggle sequence. Within one binary that's fine.
- Date: 2026-05-09
