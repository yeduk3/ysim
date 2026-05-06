# Resume — Persistence Slice

## Must remember

- **Branch:** `feat/persistence` (off `main`). Do not write feature code on `main`.
- **Schema is binding.** `docs/design/scene_format.md` defines the on-disk shape. Any field rename, additive key, or behavior tag belongs there first.
- **Material name collision.** `::Material` in `src/main.cpp` and `scene_format::Material` in the new header overlap. Inside `Simulator`, fully qualify the C++-side struct as `::Material`. The compiler will not always flag the ambiguity until the second occurrence — see `docs/mistakes/COMMON_MISTAKES.md` (CM-001).
- **Reserved-but-not-shipped pattern.** Both `BehaviorType::Rigid|Elastic|Fluid|Generator` and the schema's `"sphere"|"cube"` shapes are accepted as *known* names by the loader and rejected with a "not available in this build" error. Do not add a silent fallback.
- **Transform.position is read from the initializer.** `MeshGridInitializerParams::center` (grid) and `MeshFileInitializerParams::offset` (file). There is no rotation field on `GeneralMesh` yet — rotation round-trips through the JSON only (D-004).
- **Persistence layer is backend-independent.** `include/scene_format.hpp` does not depend on Metal, GLFW, or tinym. `test/scene_io_test.cpp` builds without a Metal device. Keep it that way (`docs/ARCHITECTURE.md §4.1`, `BDD-103`).

## Last decisions + why

- **D-001** — Vendor `nlohmann/json` under `include/nlohmann/`. Matches the `stb_image.h` precedent + `docs/CONVENTIONS.md`.
- **D-002** — Doctest as v1 test framework, single-header at `include/doctest.h`. Separate test executable so it doesn't pull GLFW/Metal.
- **D-003** — `"grid"` is the v1 primitive shape; `"sphere"`/`"cube"` reserved-but-not-shipped. Avoids coupling the persistence slice to the future BDD-001 authoring slice.
- **D-004** — Per-object transform lives in the initializer, not on `GeneralMesh`. Rotation persists through JSON only (no consumer in v1, but the design requires the field).
- **D-005** — `Material` extended to the OpenPBR v1 subset. Only `baseColor` is rendered today; the rest exist so save/load round-trip the values.
- **D-006** — Global `SceneEnvironment` (gravity, wind) added as `inline static` on `Scene<BE,PR>`. Defaults match the schema defaults.

## Next step you were about to take

The slice is complete. The next concrete step is the **Estimator's** turn — running `verify.sh` (does not yet exist; the slice only added `scripts/verify-light.sh` for the Generator gate) and judging whether to merge or send back. If the Estimator wants a stricter Generator-side gate, the obvious follow-up is to extend `scripts/verify-light.sh` to compile (but not run) the `ysim` binary as well — currently it builds the test binary only because that's what was required to land the slice without a Metal device.

See `.agent/PLAN.md` and `.agent/CURRENT_WORK.md` for full plan and progress.
