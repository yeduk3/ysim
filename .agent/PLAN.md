# Plan — Environment Forces Slice

> Owner: **Planner** writes; **Generator** executes; **Estimator** judges.
> Updated: 2026-05-07

## Goal

Wire the existing `Scene<BE,PR>::environment` (gravity + wind, D-006) into the simulation step so the cloth simulator actually responds to forces. After this slice ships:

- A `TriangularCloth` or `FastGridCloth` mesh under non-zero gravity falls (mean y-position decreases over time).
- A `Float`-tagged mesh stays exactly where it was placed regardless of gravity / wind (`BDD-009`'s strict-equality clause).
- The user can edit gravity and wind from a new `Environment` panel; the change takes effect on the next sim step without restart (matches `FR-018`'s live-edit propagation expectation).
- Saved scenes round-trip the gravity / wind values (already wired by D-006; this slice doesn't re-touch persistence).

## Scope

- `BDD-009` — Float behavior is force-exempt (FR-009).
- `BDD-011` — Set gravity (FR-011).
- `BDD-012` — Set wind (FR-012).
- ImGui `Environment` panel exposing gravity (`vec3`) and wind (`vec3`) text inputs / sliders. Default values are the schema defaults (`gravity = (0, -9.81, 0)`, `wind = (0, 0, 0)`).
- Per-frame application of gravity to **non-`Float`** meshes' `externalForces.externalForces` buffer; wind to wind-susceptible behaviors (`TriangularCloth`, `FastGridCloth`). The existing kernels already consume the per-mesh `externalForces` buffer (`src/main.cpp:TriangularClothBehavior::setBuffer` line ~1437, `FastGridClothBehavior::setBuffer` line ~1494) — this slice fills it.

Behaviors covered: see `docs/specs/BDD.md#BDD-009`, `#BDD-011`, `#BDD-012`. Tests / acceptance: `docs/TESTS.md` blocks of the same ids; matrix rows already exist in `docs/TEST_MATRIX.md` as `pending`.

## Non-goals (this slice)

- **Wind reformulation as an air velocity field.** PRD §3.3 calls this out as a v2 concern; v1 ships wind as a force vector. The schema is forward-compatible (the v2 slice can extend `Environment` additively).
- **Adding a new `BehaviorType` consumer.** Existing kernels (`compute_tri_spring_forces`, `compute_cloth_grid_forces_fast`) and the `Float` path already read the per-mesh external-forces buffer — populating it is the slice's job, not adding a new dispatch.
- **Closing BDD-007 (cloth drapes onto rigid surface).** That's its own slice — needs collision response wired into the cloth path; this slice only delivers the gravity input that BDD-007 then *uses*.
- **Touching the Metal kernels themselves.** No `.metal` file changes. The slice changes only how the C++-side sets up the existing buffer.
- **Closing the parked persistence app-level test WARNING.** That requires a Metal-backed test harness (Q-D); deliberately deferred to the next slice. Force-application correctness will be marked "WARNING — manual verification" in the matrix until that harness exists.
- **Resolving any of PRD Q1, Q2, Q4, Q5, Q6.**
- **Any UI for behavior switching (BDD-006), per-object transforms (BDD-003), or material editing (BDD-005).** Separate slices.

## Todo

Ordered. Generator executes top-to-bottom.

1. **Read the wiring points first.** The existing kernels consume `mesh.externalForces.externalForces` as a flat `numPoints * 3` float buffer — see `src/main.cpp:TriangularClothBehavior::setBuffer` (~1437) and `FastGridClothBehavior::setBuffer` (~1494). The `ExternalForces<BE,PR>` struct is defined ~line 813. The buffer is currently allocated nowhere; this slice owns its allocation + per-frame fill.
2. **Allocate `mesh.externalForces.externalForces` during `Scene::pack()`.** Matches the existing pattern for `state.x/v/f/m/n` — slice into a packed `packedExternalForces` buffer that lives next to `packedMeshData`. Filled with zeros initially.
3. **Fill the buffer per simulation step.** Add a force-fill stage in `Simulator::update` *before* the per-substep behavior dispatch:
   - For each mesh `m`: if `m.behaviorType == Float`, leave `externalForces` zero — this is what makes `BDD-009` strict (no rounding error from an applied-then-cancelled force).
   - Else: write `(g.x * mass, g.y * mass, g.z * mass)` per particle from `Scene::environment.gravity` and `state.m`. For wind-susceptible behaviors (cloth), add `Scene::environment.wind` (no mass scaling — wind is a force per particle, not per kilogram).
   - Implementation lives **on the C++ side** writing through `state.f.ptr` / `externalForces.externalForces.ptr` (Apple Silicon unified memory). No new Metal kernel.
4. **Add an `Environment` ImGui panel.** Place beside the existing Scene I/O panel in the main render loop. Three-component `InputFloat3` for gravity, three-component `InputFloat3` for wind. Default values from `Scene::environment`. Edits write back into `Scene::environment` immediately — the per-step fill picks them up next frame (`FR-018` live-edit semantics, no pause/resume needed).
5. **Tests (data-layer only — sim correctness needs Metal harness, see Non-goals).**
   - `BDD-009` data-layer test: build a `SceneSnapshot` with gravity = (0, -9.81, 0), assert the existing scene_format round-trip is unaffected (regression guard for D-006).
   - `BDD-011` / `BDD-012` UI-state tests: add a small unit test in `test/scene_io_test.cpp` that constructs a `SceneSnapshot` with non-default gravity / wind, round-trips through `toJson`/`fromJson`, asserts the values survive bit-identical (a re-statement of D-006 with explicit values to guard against regression).
   - `BDD-009` strict-equality clause is **not** unit-testable today (needs sim step). Mark `BDD-009` row `pass` for the data-layer half with a note; the simulation half goes to a `WARNING` once the Estimator runs.
   - Fill `BDD-011`, `BDD-012` rows in `docs/TEST_MATRIX.md` analogously.
6. **Stop and hand off to the Estimator.** Do **not** scope cloth-drape acceptance (`BDD-007`), per-object transform UI, behavior switching, or material UI into this slice. Each is a separate planner-tracked candidate.

## Course corrections

- **Primitive-creation slice (turn-1 → manual testing → fixes):** Estimator gave NOTE-level on the original commit; user then exercised `Create > Sphere/Cube` manually and surfaced two runtime bugs that the unit-test net could not catch (`CM-002`: shared initializer ownership across re-pack; `CM-003`: BVH buffer not re-sized on `numPrimitives` change). Both fixed in commits `2000aea` and `979f037`. The pattern — runtime bugs slipping past CPU-only tests — is a real signal. Captured in `PROJECT_STATE.md` as a planner-tracked motivation for the **test-harness slice** (next milestone after this one). Do not roll the harness into this slice; that would expand scope past one BDD-cluster.
- **`Scene::environment` already exists** (D-006). This slice is the *consumer*; persistence already round-trips it. Confirm in step 5 that the existing scene_format tests still pass without any header changes.

## What to read before writing code

- `docs/specs/FRD.md` FR-009, FR-011, FR-012 — functional contracts.
- `docs/specs/BDD.md` BDD-009, BDD-011, BDD-012, and **BDD-103** (the boundary invariant the Estimator will check — kernel files must remain untouched).
- `docs/TESTS.md` blocks for those four BDDs.
- `src/main.cpp` around `ExternalForces<BE,PR>` (~line 813), the cloth kernel setBuffer paths (~lines 1437 and 1494), `Scene::pack()` (~line 1804) for the allocation pattern, and `Simulator::update()` (~line 4475) for where the force-fill stage goes.
- `docs/DECISIONS.md` D-006 — confirms the in-memory shape and defaults the slice consumes.
- `docs/CONVENTIONS.md` — file naming, type/function casing, commit message style.
