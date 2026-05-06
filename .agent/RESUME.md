# Resume — Persistence Slice

## Must remember

- **Branch:** `feat/persistence` (off `main`). Do not write feature code on `main`.
- **Schema is binding** (`docs/design/scene_format.md`). After the Estimator pass, the doc now matches the implementation: `"grid"` is the v1 primitive shape; `"sphere"`/`"cube"` are reserved-but-not-shipped.
- **Rotation round-trips through `GeneralMesh::rotationQuat`** (D-007). Quaternion order is `[w, x, y, z]` — this is the schema's order, **not** `tinym::vec4`'s `(x, y, z, w)` order. Use the `Quat` struct in `src/main.cpp`.
- **Import paths resolve via `scene_format::resolveImportPath(sceneDir, importPath)`** (D-008). Relative paths are anchored to the scene file's directory; absolute paths pass through.
- **Material clamping is non-fatal** and surfaces via `SceneSnapshot::warnings` (D-009). A `LoadWarnings` is the channel; do not upgrade clamps to errors.
- **Material name collision** (CM-001): inside `Simulator`, qualify the C++-side struct as `::Material`. Same goes for `::Quat` after the rotation field landed.
- **Reserved-but-not-shipped pattern.** Both `BehaviorType::Rigid|Elastic|Fluid|Generator` and the schema's `"sphere"|"cube"` shapes are accepted as *known* names by the loader and rejected with a "not available in this build" error. No silent fallback.
- **Persistence layer is backend-independent.** `include/scene_format.hpp` + `test/scene_io_test.cpp` build without a Metal device. Keep it that way (`docs/ARCHITECTURE.md §4.1`, `BDD-103`).

## Last decisions + why

- **D-001** — Vendor `nlohmann/json` under `include/nlohmann/`.
- **D-002** — Doctest as v1 test framework, vendored at `include/doctest.h`.
- **D-003** — `"grid"` is v1 primitive shape; `"sphere"`/`"cube"` reserved-not-shipped.
- **D-004** — *(Superseded by D-007.)* Initially kept rotation in JSON only.
- **D-005** — `Material` extended to OpenPBR v1 subset.
- **D-006** — `Scene::environment` (gravity, wind) static singleton.
- **D-007** — Per-object rotation lives on `GeneralMesh` as `rotationQuat`. Supersedes D-004 after Estimator flagged a round-trip violation.
- **D-008** — Import paths resolve relative to the scene file's directory.
- **D-009** — Material clamping surfaces via `LoadWarnings` channel (non-fatal).

## Next step you were about to take

Slice complete pending Estimator re-review. The next concrete step is the **Estimator's** turn. The one remaining WARNING (no app-level `Simulator::saveScene/loadScene` test) is genuinely blocked on a Metal-capable test harness — that's a planning question, not a Generator one. If a future planner wants to close it, the cheapest path is a smoke test that boots the simulator without rendering, calls `saveScene` → `loadScene`, and asserts the in-memory `Scene<METAL,Precision>::meshes` lengths and `environment` round-trip. That requires `MetalGlobalContext` to come up successfully in CI; today the test executable bypasses it entirely.

See `.agent/PLAN.md` and `.agent/CURRENT_WORK.md` for full plan and progress.
