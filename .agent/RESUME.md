# Resume — Render-State Decoupling Slice

## Must remember

- **Branch:** `feat/render-state-decoupling` (off `main`, after `feat/environment-forces` was fast-forwarded into `main`).
- **`MeshGL<CPU>` is now in `include/MeshGL.hpp`** (was inline in `src/main.cpp`). The CPU specialization references `struct CPU;` via forward-declaration; the actual `struct CPU : Backend {}` definition still lives in `src/main.cpp`. If a future header needs `MeshGL<CPU>`, it must include `MeshGL.hpp` *and* be included from a TU that has the `CPU` definition in scope (i.e., `src/main.cpp`).
- **`MeshRenderState` is keyed by `mesh.id`** and the cached `MeshGL` captures raw pointers into the *current* `packedMeshData`. `Scene::pack()` reallocates those buffers, so `Simulator::initialize` calls `renderState.clear()` immediately after pack. Any future code that re-packs without going through `Simulator::initialize` must clear the render state too — otherwise stale pointers + wrong sizes get rendered (this is the same trap CM-003 hit on a different cache).
- **GL handles are still leaked** when `MeshRenderState::clear()` runs. `MeshGL` has no destructor, same as before this slice. v1 is a one-shot process (no scene reload churn beyond manual user actions); revisit if the leak becomes measurable.
- **The dead CPU `ExplicitSystem<CPU,PR>::update` is now a stub.** It was uninstantiated even before the slice (`Backend = METAL` everywhere); the stub keeps it that way without referencing the removed `mesh.meshGL` field.
- **Visual regression is the user's gate** for this slice. `ysim_tests`/`ysim_primitive_tests` cover data-layer; the renderer's pixel output is not yet mechanically tested.

## Last decisions + why

- **D-011** — Render-side GL state moved out of `GeneralMesh` into `MeshRenderState`. Structural prerequisite for the test-harness slice; strengthens the `ARCHITECTURE §2.2/§2.3` boundary.

## Next step you were about to take

Slice complete. The next concrete step is the **user's manual visual-regression check** — run `./build/src/ysim`, confirm identical render to before. Then **Estimator's** turn — `./scripts/verify.sh` (now passing) and slice-vs-plan reconciliation. After that lands, the **test-harness slice** (Metal-backed sim fixture) is finally unblocked: with `Simulator::initialize` no longer needing GL, a `test/sim_round_trip_test.cpp` binary can bring up `Scene<METAL,Precision>`, run `addCloth/addSphere/addCube`, save/load, run a sim step, and assert. That slice closes the parked sim-step clauses on BDD-009/011/012/015.

See `.agent/PLAN.md` and `.agent/CURRENT_WORK.md` for full plan and progress.
