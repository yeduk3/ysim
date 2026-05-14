# PLAN — D-042 R-3: Scene::pack memcpys from PreviewState — `feat/d-042-r-3-pack-memcpy-preview`

> Owner: **Planner** writes; **Generator** executes; **Estimator** judges.
> Updated: 2026-05-14

## Course note: previous turn's verdict

Estimator turn 37 (D-042 R-2 — MeshGL ↔ PreviewState binding) returned **WARNING** with 0 BLOCK + 1 WARNING + 1 NOTE. WARNING flagged stale `previewBindings` at scene-reset/loadScene boundaries (nextMeshId resets to 0 but renderState doesn't clear). NOTE: `MeshRenderState::clear()` clears only materialized state, not bindings. Both fold into R-3 — adding `clearPreviewBindings()` + wiring it into the load/reset paths.

R-2 merged to `main` via commits `cf5555f add:` + `97d35f7 chore:`. Self-test 67 → 68 PASS deterministic on macOS (Linux Metal SKIP unchanged).

## Goal

**Make `Scene::pack` populate the packed vertex/facet/normal sub-views by `memcpy` from the per-request `PreviewState`** instead of relying solely on the legacy `mesh.initialize()` regen-from-initializer path. This is R-3 of the 7-slice D-042 refactor and the first slice where preview is **load-bearing** for the simulator's runtime data (R-1 + R-2 added preview + binding but `Scene::pack` still re-ran the initializer to fill packed buffers). Post-R-3: `mesh.initialize()` still runs (preserves the adjacency-derivation path: vertexAdjFacets, vertexAdjFacetsOffsets, edges, etc.), but the vertex/facet/normal data is overwritten by `memcpy` from `req.preview.x.data() / facets.data() / n.data()`. Future R-4 mutates preview directly for translateObject/rotateObject/setMaterial; R-5 mirrors the pattern in reverse (packed → preview sync at end of `Simulator::update`).

Also folds the R-2 Estimator WARNING by adding `MeshRenderState::clearPreviewBindings()` + calling it from `Simulator::loadScene`. (The harness `resetScene` lambdas don't need it — no GL context — but the production load path is real.)

Block 38 verifies the memcpy is load-bearing by mutating `preview.x[1]` to a sentinel value (99.0f) BEFORE `sim.initialize()`, then asserting the post-pack `meshes[0].state.x[1] == 99.0f`. Without the memcpy block, `state.x[1]` reflects the initializer's regenerated geometry (-0.1f for a tess=2 cube at origin), so the sentinel doesn't survive. Self-test count 68 → 69.

## Scope

**Design call (1) — Memcpy AFTER `mesh.initialize()`, not as a replacement.** `GeneralMesh::initialize()` calls `initializer->initialize(state, adjacency)` which builds:
- `state.x` (vertex positions) — REPLICATED by `preview.x` in R-1.
- `adjacency.facets` — REPLICATED by `preview.facets` in R-1.
- `state.n` (normals) — REPLICATED by `preview.n` in R-1.
- `adjacency.edges`, `vertexAdjFacets`, `vertexAdjFacetsOffsets`, `vertexAdjEdges`, `vertexAdjEdgesOffsets` — **NOT** in preview. Derived from the initializer's regenerated geometry. Required by `Scene::pack`'s post-init loop at lines ~2160-2185 (the cross-mesh offset stitching).

The cleanest R-3 keeps `mesh.initialize()` running (so adjacency derivation works unchanged) and adds memcpy after it. The memcpy overrides the initializer's regenerated x/n/facets with preview's data. For v1 the two are byte-equivalent (populatePreview mirrors the initializer's geometry algorithm exactly), so the override is a no-op overlay UNLESS preview is mutated between addX and `sim.initialize()` — which is exactly what Block 38 exercises to prove the memcpy is load-bearing.

**Design call (2) — Memcpy guarded on size match.** Preview's vertex/facet/normal sizes are populated by `populatePreview` at addX time. The packed sub-view sizes are derived from `initializer->getParams()->numPoints/numFacets`. For v1 these match exactly. To defend against an initializer subtype that defines populatePreview as a default no-op (the base `GeneralMeshInitializer::populatePreview` returns immediately), the memcpy block guards on size match before writing. On size mismatch (future no-preview initializer), the memcpy is silently skipped and the initializer's regen stands — the parallel-symbol invariant. For v1 all 4 subtypes have populatePreview, so every memcpy fires.

**Design call (3) — Memcpy placement inside Scene::pack's per-mesh loop.** Insert AFTER `meshes[i].initialize()` (line ~2151) and BEFORE the existing `std::memcpy(state.xPrev.ptr, state.x.ptr, ...)` (line ~2156, so xPrev mirrors the post-preview x, not the regenerated x). The vertex adjacency stitching at lines ~2160-2165 reads the mesh's locally-computed adjacency tables — already correct from initialize. memcpy block sits cleanly between these.

**Design call (4) — `MeshRenderState::clearPreviewBindings()` API.** New public method (R-2 WARNING fold-in):
```cpp
void clearPreviewBindings() { previewBindings.clear(); }
```
Wired into `Simulator::loadScene` AFTER `pendingRotations.clear();` at ~line 6025. Harness `resetScene` lambdas (at lines 6557, 6658) are NOT updated — those lambdas don't have `sim` capture and the harness has no GL context, so stale `previewBindings` are inert. Acceptable for R-3 scope.

**Design call (5) — Block 38 shape.** Verify memcpy is load-bearing via sentinel injection:
1. `resetScene()`; `sim.addCube(tinym::vec3(0, 0, 0), 2, 0.2f, 1.0f)`.
2. Capture `req = requestsGeneralMeshes[0]`; assert preview has ≥ 9 floats in x/n + 9 facet indices.
3. Mutate `reqs[0].preview.x[1] = 99.0f` (sentinel — distinct from cube's natural y=-0.1).
4. `sim.initialize()` — triggers `Scene::pack` → `mesh.initialize()` writes initializer's -0.1 → memcpy from preview overrides to 99.0f.
5. Assert `meshes[0].state.x[1] == 99.0f` (sentinel survived). Without memcpy: `state.x[1] == -0.1f` (assertion FAILs).
6. Assert `state.x.size / 3 == preview.numPoints()` (count integrity).

Pass label: `D-042 R-3 / Scene::pack memcpys preview x into packed sub-view (sentinel survives initializer regen)`.

**Design call (6) — Block 38 placement: INSIDE the Metal-gated section, AFTER Block 37.** Block 38 uses `sim.initialize()` which requires Metal device + buffer allocation. Linux container SKIPs along with Blocks 1-29 + 32-37. Same pattern as the prior preview/binding blocks. Acceptable per D-012.

**NEW symbols this slice adds**:
- `MeshRenderState::clearPreviewBindings()` method.
- Block 38 in `runSelfTest` (1 new pass clause).
- `docs/DECISIONS.md` — D-042 R-3 entry.

**MODIFIED symbols in place**:
- `include/MeshRenderState.hpp` — adds `clearPreviewBindings`.
- `src/main.cpp` — Scene::pack gains the post-initialize memcpy block (Design calls 1+2+3). `Simulator::loadScene` calls `renderState.clearPreviewBindings()`.
- `.agent/PROJECT_STATE.md` — next-milestone updates.

**PRESERVED symbols** (parallel-symbol invariant):
- `GeneralMeshInitializer::initialize` and all 4 overrides — UNCHANGED. The initializer regen path stays callable; R-3's memcpy overlays its output.
- `GeneralMesh::initialize` — UNCHANGED. Still calls `initializer->initialize(state, adjacency)` for adjacency derivation.
- `Scene::pack`'s sub-view allocation pattern — UNCHANGED. Same offsets, same `VectorBase` constructions.
- `PreviewState<PR>` — UNCHANGED.
- `MeshRenderState::registerPreviewBinding` / `previewBinding` / `removeById` / `getOrCreate` / `clear` / `has` — UNCHANGED.
- `Simulator::loadScene` other-than-the-one-new-call — UNCHANGED.
- `resetScene` lambdas in `runSelfTest` / `runRefitBench` — UNCHANGED (no GL context → stale bindings inert).
- All Metal kernels, BVH, narrow-phase — UNCHANGED.
- Block 1-37 — UNCHANGED.

## Non-goals

- **NO replacement of `mesh.initialize()` with adjacency-only path.** That's a future cleanup (R-7 candidate); R-3 keeps initialize() running for adjacency, overrides x/n/facets only.
- **NO mutation of `translateObject` / `rotateObject` / `setMaterial` to write into PreviewState.** That's R-4.
- **NO packed → preview resync at end of `Simulator::update`.** That's R-5.
- **NO self-test migration to read via Scene::meshes preview alias.** That's R-6.
- **NO retirement of the legacy `getOrCreate` packed-sub-view fallback.** That's R-7.
- **NO update to harness `resetScene` lambdas.** Acceptable per Design call (4); they have no GL context.
- **NO new BDD/FRD/CM.**
- **NO C-* FlatBuffers work.**

## Spec substitution

None this turn. R-3 is infrastructure work on the D-042 architectural-refactor path.

## Standing constraints

- **RIGID-BACKEND-PORTABILITY** (D-037) — UNCHANGED.
- **PARALLEL-IMPL-LOCKSTEP** (D-035 + D-038) — UNCHANGED.
- **BDD-102-vs-ALEMBIC-BYTES** — UNCHANGED.
- **DUPLICATED-INSPECTOR-WIRING** — UNCHANGED.
- **GLFWINIT-NON-REF-COUNTED** — UNCHANGED.

## Todo

1. **Branch hygiene.** Working in worktree `.claude/worktrees/r3-pack-memcpy` on branch `feat/d-042-r-3-pack-memcpy-preview` (branched off main HEAD `97d35f7`). Submodules already initialized. Commit prefix `add:`.

2. **`include/MeshRenderState.hpp` — add `clearPreviewBindings()`** alongside the existing `clear()` method.

3. **`src/main.cpp` — Scene::pack memcpy block** inserted AFTER `meshes[i].initialize()` and BEFORE the existing `xPrev` mirror memcpy. Body uses size guard + 3 conditional memcpys (x, n, facets) per Design call (2).

4. **`src/main.cpp` — `Simulator::loadScene` calls `renderState.clearPreviewBindings()`** after `pendingRotations.clear();`.

5. **New Block 38** in `runSelfTest` — inserted INSIDE the Metal-gated section AFTER Block 37. Sentinel injection (preview.x[1] = 99.0f) verifies memcpy survives initializer regen.

6. **Build + verify deterministic.** `cmake --build build && cd build && for i in 1 2 3 4 5; do ./src/ysim --self-test; done` — expect **69/69 PASS** each time.

7. **`verify-light.sh` cross-check.** Expect doctest 159/159 + 1120/1120 SUCCESS unchanged.

8. **Bug-probes** (each FAIL after revert; restore after):
   - **(a) Comment out the entire memcpy block in Scene::pack**: Block 38 FAILs (`packedHasSentinel=0`).
   - **(b) Skip only the `state.x` memcpy (keep `state.n` + `facets`)**: Block 38 FAILs (same — `packedHasSentinel=0`). Confirms the x branch is load-bearing.
   - **(c) `clearPreviewBindings` from `loadScene`**: harder to verify load-bearingness in self-test. Document the gap; the WARNING fold-in is verified by code-review only.

9. **Append D-042 R-3 to `docs/DECISIONS.md`**.

10. **Update `.agent/PROJECT_STATE.md`**: next-milestone updates R-3 → R-4.

11. **Update `.agent/CURRENT_WORK.md` + `.agent/RESUME.md`** — Generator-tier task.

## Course corrections

- **`feedback_make_means_add_new` rule**: user's brief uses "rewrite Scene::pack" — explicit rewrite verb, in-place modification of the existing pack body is correct. The legacy initialize() regen path stays callable (size-guarded fallback).
- **`project_flatbuffers_caching_skipped`**: stays in force.
- **D-026 lifetimeId invariant**: applies — `Scene::pack` sets `meshes[i].lifetimeId = req.lifetimeId` BEFORE the memcpy block.
- **D-041 nextMeshId monotone**: applies — preview's id-keyed binding is unaffected; memcpy operates on `requestsGeneralMeshes[i]` by index, not id.
- **D-021 transformPosition cascade**: applies — Scene::pack seeds `meshes[i].transformPosition` from the initializer's center/offset BEFORE memcpy.
- **CM-012 utility-helper-exit trap**: applies — `clearPreviewBindings()` does not call `exit()` / `abort()`.
- **PROBE-COVERAGE-EDGES per PLANNER §9**: not a math-layer slice. Edges considered: empty preview, size mismatch, multiple meshes, sentinel distinct from natural values.

## Expected metrics

- Self-test count: **68 → 69**.
- 5-run deterministic PASS on macOS dev host.
- `verify-light.sh`: doctest 159/159 + 1120/1120 SUCCESS unchanged.
- Linux container: Block 38 SKIPs.
- Estimator verdict next turn: **NOTE** if implementation clean. Possible NOTE items: memcpy is no-op overlay for v1; initialize() still does redundant work; clearPreviewBindings not in harness lambdas.
