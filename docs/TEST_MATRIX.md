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
| BDD-002     | Import a `.obj` mesh                              |              | pending |
| BDD-003     | Translate a selected object                       |              | pending |
| BDD-004     | Rotate with quaternion canonical storage          |              | pending |
| BDD-005     | Edit OpenPBR material parameter                   |              | pending |
| BDD-006     | Assign behavior type to an object                 |              | pending |
| BDD-007     | Cloth drapes onto a rigid surface                 |              | pending |
| BDD-008     | Rigid body falls and rests                        |              | pending |
| BDD-009     | Float behavior ignores gravity and wind           | `src/main.cpp::Simulator::applyEnvironmentForces` (Float branch zeroes externalForces; data-layer covered by code review) | warning |
| BDD-010     | Collision detected between simulated objects      |              | pending |
| BDD-011     | Change gravity at runtime                         | `test/scene_io_test.cpp::BDD-011/012: non-default gravity and wind round-trip bit-stable`, `…missing environment falls back to schema defaults` | warning |
| BDD-012     | Apply wind force                                  | `test/scene_io_test.cpp::BDD-011/012: non-default gravity and wind round-trip bit-stable`, `…missing environment falls back to schema defaults` | warning |
| BDD-013     | Export simulation to Alembic                      |              | pending |
| BDD-014     | Save scene to disk                                | `test/scene_io_test.cpp::BDD-014: save populated scene to disk` | pass    |
| BDD-015     | Load scene reproduces saved state                 | `test/scene_io_test.cpp::BDD-015: load reproduces saved state field-by-field` | pass    |
| BDD-016     | Reject incompatible scene file version            | `test/scene_io_test.cpp::BDD-016: reject scene file with unsupported format_version`, `…missing format_version`, `…reserved-but-not-shipped behavior type`, `…unsupported import extension` | pass    |
| BDD-017     | Ray-pick selects nearest hit object               |              | pending |
| BDD-018     | Inspector edits propagate live                    |              | pending |
| BDD-019     | Frame profiler shows and exports timings          |              | pending |
| BDD-101     | End-to-end round-trip to Alembic into Unreal      |              | pending |
| BDD-102     | Single-machine determinism                        |              | pending |
| BDD-103     | Backend-boundary invariant holds                  |              | pending |

## Status legend

- `pending` — scenario exists, no test code yet
- `pass` — test exists and last run passed
- `fail` — test exists and last run failed (Estimator must classify as WARNING/BLOCK)
- `skipped` — test exists but is intentionally skipped; reason in `DECISIONS.md`
- `warning` — partial coverage: data-layer half of the BDD has tests and passes, but a sim-step / GPU clause is parked behind the test-harness slice. See `PROJECT_STATE.md` "What the Estimator should know" for the convention.
