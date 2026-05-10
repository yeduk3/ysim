# Test Matrix

> Index that connects BDD scenarios to actual test functions and their pass/fail status.
>
> - **Planner** adds rows when authoring scenarios (test address blank, status `pending`).
> - **Generator** fills in the test address and updates status when running.
> - **Estimator** verifies status against the latest `./scripts/verify.sh` run.

Use this as the entry point — `docs/TESTS.md` will get long, so find scenarios here first.

## Matrix

| Behavior ID | Scenario                                          | Test address | Status  |
| ----------- | ------------------------------------------------- | ------------ | ------- |
| BDD-001     | Create a sphere primitive                         | `test/primitive_test.cpp::BDD-001: sphere primitive — vertex/facet counts match closed-form`, `…sphere primitive — every vertex lies on the sphere surface`, `…sphere primitive — facets reference no out-of-bounds indices`, `…sphere primitive — no degenerate triangle (zero area)`, `…cube primitive — vertex/facet counts match closed-form`, `…cube primitive — every vertex sits on a face plane`, `…cube primitive — center translation moves every vertex`, `…scene_format accepts sphere and cube as primitive shapes`, `…scene_format still rejects unknown primitive shapes` | pass    |
| BDD-002     | Import a `.obj` mesh                              | `src/main.cpp::runSelfTest` Block 7 — `BDD-002 / .obj import via importMesh appears in scene` (numMeshes++ + Float behavior + state.x.size > 0); `BDD-002 / import path round-trips through toSnapshot` (persisted state records the import path); `BDD-002 / missing import path leaves scene unchanged` (no partial-add when file is missing). Plus `Simulator::importMesh` path-existence guard in `src/main.cpp`. | pass    |
| BDD-003     | Translate a selected object                       | `src/main.cpp::runSelfTest` Block 9 — three "Then" clauses + a fourth round-trip clause PASS (`BDD-003 / object's center is (1, 2, 3)`, `…/ next simulation step uses the new position`, `…/ rendering reflects the new position on the next frame`, `…/ translate survives Scene::pack rebuild`). Closed by D-014 (`GeneralMesh::transformPosition` + `Simulator::translateObject`) and D-015 (translateObject writes back to initializer center/offset so re-pack reproduces the translate). | pass    |
| BDD-004     | Rotate with quaternion canonical storage          | `src/main.cpp::runSelfTest` Block 12 — quaternion composition round-trip across saveScene/loadScene boundary; orientation matches the in-memory composition within 1e-5 and unit-norm holds at each step. Closed by D-019 (Hamilton-product + normalize/norm helpers on the bare `Quat` struct). | pass    |
| BDD-005     | Edit OpenPBR material parameter                   |              | pending |
| BDD-006     | Assign behavior type to an object                 |              | pending |
| BDD-007     | Cloth drapes onto a rigid surface                 | `src/main.cpp::runSelfTest` Block 6 — all four clauses PASS (`BDD-007 / cloth meanY decreases over time`, `BDD-007 / contact constraints fire on broad/narrow phase`, `BDD-007 / no cloth vertex tunnels through ground`, `BDD-007 / total energy stays bounded`). Closed by D-013 (snapshot → swept-segment CCD in `narrow_pt_tri`) on top of `enlargeTrajectory(system.subh)` from the cloth-drape slice. | pass    |
| BDD-008     | Rigid body falls and rests                        |              | pending |
| BDD-009     | Float behavior ignores gravity and wind           | `src/main.cpp::runSelfTest` (`BDD-009 / Float exact x and v under non-zero gravity and wind` block — bitwise compare on full state.x and state.v under non-zero gravity AND wind) + `Simulator::applyEnvironmentForces` Float branch | pass    |
| BDD-010     | Collision detected between simulated objects      | `src/main.cpp::runSelfTest` Block 13 — positive (cloth co-located with ground at y=-1, asserts at least one (A, B) narrow contact between distinct objects via `objPair.query != objPair.target`) + negative (cloth at y=10 above ground, asserts `cumulativeNarrowCollisions == 0`) clauses. Self-collision filtered by default `enableSelfCollisions = false`. | pass    |
| BDD-011     | Change gravity at runtime                         | `src/main.cpp::runSelfTest` (`BDD-011 / runtime gravity pivot grows cloth +x velocity` block — gravity flips from `(0,-9.81,0)` to `(9.81,0,0)` *without* `simulator.initialize()` between pumps; cloth mean vx grows above tolerance) + `test/scene_io_test.cpp::BDD-011/012: non-default gravity and wind round-trip bit-stable` | pass    |
| BDD-012     | Apply wind force                                  | `src/main.cpp::runSelfTest` (`BDD-012 / wind (5,0,0) drives cloth +x velocity` block — gravity zero, wind set to `(5,0,0)` from rest, cloth mean vx grows above tolerance; plus `BDD-012 / env round-trip bit-stable through Simulator` for save/load) + `test/scene_io_test.cpp::BDD-011/012: non-default gravity and wind round-trip bit-stable` | pass    |
| BDD-013     | Export simulation to Alembic                      |              | pending |
| BDD-014     | Save scene to disk                                | `test/scene_io_test.cpp::BDD-014: save populated scene to disk` | pass    |
| BDD-015     | Load scene reproduces saved state                 | `src/main.cpp::runSelfTest` (`BDD-015 / numMeshes round-trip`, `…sim step after load is stable`) + `test/scene_io_test.cpp::BDD-015: load reproduces saved state field-by-field` | pass    |
| BDD-016     | Reject incompatible scene file version            | `test/scene_io_test.cpp::BDD-016: reject scene file with unsupported format_version`, `…missing format_version`, `…reserved-but-not-shipped behavior type`, `…unsupported import extension` | pass    |
| BDD-017     | Ray-pick selects nearest hit object               | `src/main.cpp::runSelfTest` Block 14 — non-overlapping (two cubes at distinct world-space x; rays through each pick the right id) + overlapping (two cubes on same line of sight; smallest-tmin wins via the same walk production uses at `src/main.cpp:6588-6596`). World-space `Ray` synthesized directly; camera unprojection is harness-skippable. Per-mesh BVH `objTrees` cleared between scenes to bypass the build-time skip for Float meshes (see CM-008). Block 17 (D-024 sister mechanization) verifies triangle-precision ranking when a rotated mesh's leaf AABB envelops the scene. | pass    |
| BDD-018     | Inspector edits propagate live                    |              | pending |
| BDD-019     | Frame profiler shows and exports timings          | `src/main.cpp::runSelfTest` Block 10 — three clauses PASS (`BDD-019 / per-section timings updated each frame`, `…/ CSV written under profiles containing history`, `…/ history collection pauses when sim pauses`). The CSV path uses `/tmp/` rather than `profiles/` for harness hygiene; spec-substitution noted in the block comment. | pass    |
| BDD-101     | End-to-end round-trip to Alembic into Unreal      |              | pending |
| BDD-102     | Single-machine determinism                        | `src/main.cpp::runSelfTest` Block 11 — per-frame bit-identical positions across two runs of `buildSyntheticScene` (30 frames, positions only, strict memcmp). Closed by D-018 (per-mesh seeded `std::mt19937` in `MeshGridInitializer`). Substitution noted: state.x stands in for Alembic outputs while FR-013 blocked. | pass    |
| BDD-103     | Backend-boundary invariant holds                  |              | pending |

## Status legend

- `pending` — scenario exists, no test code yet
- `pass` — test exists and last run passed
- `fail` — test exists and last run failed (Estimator must classify as WARNING/BLOCK)
- `skipped` — test exists but is intentionally skipped; reason in `DECISIONS.md`
- `warning` — partial coverage: data-layer half of the BDD has tests and passes, but a sim-step / GPU clause is parked behind the test-harness slice. See `PROJECT_STATE.md` "What the Estimator should know" for the convention.
