# PLAN — D-042 R-2: MeshGL bound to PreviewState — `feat/d-042-r-2-meshgl-preview-binding`

> Owner: **Planner** writes; **Generator** executes; **Estimator** judges.
> Updated: 2026-05-14

## Course note: previous turn's verdict

Estimator turn 36 (D-042 R-1 — PreviewState infrastructure slice) returned **NOTE** with 0 BLOCK + 0 WARNING + 2 NOTEs. NOTEs flagged (i) the `populatePreview` overrides duplicate geometry math from initializer paths (will collapse in later R-* slices when the original path retires) and (ii) `MeshFileInitializer::populatePreview` re-reads the .obj from disk (acceptable for v1 since `MeshFile::load` is cached at file scope). Both items are deferred to R-7 cleanup.

R-1 merged to `main` via commit `08f3ef5`. Self-test count 65 → 66 PASS deterministic on macOS AND Linux.

## Goal

**Wire `MeshGL<CPU>` to point at `PreviewState`'s stable per-mesh buffers instead of the volatile packed sub-views.** R-1 (08f3ef5) added `PreviewState<PR>` heap-owned per-mesh vertex/facet/normal data populated by the initializer at `addGeneralMesh` time. R-2 turns those preview pointers into the **canonical binding source for `MeshGL`** by adding a new `MeshRenderState::registerPreviewBinding(id, preview)` API that each `Simulator::addX` wrapper calls right after `scene.addGeneralMesh`. The lazy `getOrCreate(mesh)` lookup (called from `uploadMeshes` + `draw` in the live GUI loop) is extended to **prefer the preview binding** over the packed sub-view at MeshGL construction time. `Simulator::initialize`'s `renderState.clear()` band-aid is retired — preview binding persists across Scene::pack rebuilds because the underlying `std::vector` heap is stable. New Block 37 verifies the binding exists immediately after `addX` (BEFORE `simulator.initialize()` runs) by checking `MeshRenderState::previewBinding(id)` returns pointers byte-identical to `request.preview.xPtr()/facetsPtr()/nPtr()`. Self-test count 66 → 67.

## Scope

**Design call (1) — Decouple "binding registration" (CPU-only, no GL) from "MeshGL materialization" (needs GL context).** The brief says "eagerly create MeshGL at addGeneralMesh time". But `MeshGL<CPU>::MeshGL(...)` calls `glGenVertexArrays` / `glGenBuffers` / `glBufferData` — all requiring an active GL context. The harness runs `Simulator::addCube` from `runSelfTest` WITHOUT a GL context (only Block 25 + the D-042 R-1's Block 36 brought up `HiddenGLContext` for the few GL-touching probes). Adding eager `MeshGL` ctor calls in every `addX` would crash the entire self-test on Linux + on macOS without --visible-window.

Resolution: split the API. `MeshRenderState::registerPreviewBinding(int id, PR* xPtr, size_t numVerts, uint32_t* facetPtr, size_t numFacets, PR* normalPtr)` stores a small `PreviewBinding` POD in a per-id map. **No GL calls.** Then `getOrCreate(mesh)` (already GL-context-gated by virtue of being called from `uploadMeshes`/`draw`) checks `previewBindings_.find(mesh.id)` first: if present, construct the MeshGL using the **preview pointers + counts** instead of `mesh.state.x.ptr` / `mesh.adjacency.facets.ptr` / `mesh.state.n.ptr`. Pull the binding entry out of the pending map after first materialize (the MeshGL is now stored in `state`).

This preserves Block 37's "pre-init MeshGL exists" intent at the **binding** layer (which is the load-bearing invariant) without forcing a GL context into the self-test harness. The "eagerly created" language in the brief shifts from "MeshGL ctor fires" → "preview binding is registered and is the source of truth for the next MeshGL materialization, whether that materialization happens at first draw or at any other GL-context-active moment".

**Design call (2) — Retire `renderState.clear()` in `Simulator::initialize` at `src/main.cpp:5403`.** The pre-R-2 clear() existed because `MeshGL.vertexPtr` captured the prior pack's `packedMeshData` sub-view; Scene::pack reallocated; the captured pointer became dangling; the next `updateBuffer` would deref the dangling pointer in `computeNormal` (Eigen::Map over `vertexPtr`). Clear() dropped the stale entries; the next draw lazy-rebuilt from the new packed sub-view.

After R-2 the bound pointer is `preview.xPtr()` (heap, stable across Scene::pack), so clear() is unnecessary for pointer hygiene. The `uploadMeshes` per-frame call still re-points `MeshGL.vertexPtr` to `mesh.state.x.ptr` (packed) via `updateBuffer` — that part is unchanged; the *initial* binding moves to preview, the per-frame upload still drives from packed.

Method `MeshRenderState::clear()` itself stays as a public API (might be useful in future tests / forced-rebuild paths); only the **call site at `Simulator::initialize:5403` retires**.

**Design call (3) — `MeshRenderState` API surface.** Add 3 NEW methods, all CPU-only (no GL):

```cpp
// CPU-only. Records a preview binding for `id`. Idempotent — overwrites
// if the same id is registered again.
template <typename PR>
void registerPreviewBinding(int id, PR* xPtr, size_t numVerts,
                            uint32_t* facetPtr, size_t numFacets,
                            PR* normalPtr);

// CPU-only. Returns optional pointer-tuple if a preview binding exists
// for `id`; nullptr otherwise. Block 37 uses this to assert pre-init.
struct PreviewBinding {
    void* xPtr; size_t numVerts;
    uint32_t* facetPtr; size_t numFacets;
    void* normalPtr;
};
const PreviewBinding* previewBinding(int id) const;

// CPU-only. Drops both the pending preview binding AND any materialized
// MeshGL for `id`. Called from Simulator::removeMesh to avoid orphan
// entries after id-not-reusing remove/add cycles (D-041 turn-2).
void removeById(int id);
```

MODIFY `getOrCreate(mesh)` to check `previewBindings_.find(mesh.id)` first; if present, construct MeshGL with those pointers + counts (NOT `mesh.state.x.ptr` etc.); then erase the pending entry so subsequent calls reuse the cached MeshGL.

**Design call (4) — Block 37 placement: INSIDE the Metal-gated section, AFTER Block 36.** Block 37 calls `sim.addCube`, which depends on Metal-allocated MemoryBlock; Linux container SKIPs the entire Metal-gated section. Same pattern as Block 36 (R-1's verification). Acceptable per D-012.

**Design call (5) — Block 37 shape (single pass clause)**:
1. `resetScene()`; `sim.addCube(tinym::vec3(0.0f, 1.0f, 0.0f), 2, 0.2f, 1.0f)` — cube tess=2 → 6 faces × 9 verts/face = 54 preview verts, 2×2×6 = 48 facets.
2. Assert `Scene<>::requestsGeneralMeshes.size() == 1`. Capture `req = Scene<>::requestsGeneralMeshes[0]`.
3. Assert `sim.renderState.previewBinding(req.id) != nullptr` (binding registered eagerly).
4. Assert `binding->xPtr == (void*)req.preview.xPtr()` (same heap-owned pointer).
5. Assert `binding->numVerts == req.preview.numPoints()` and > 0.
6. Assert `binding->facetPtr == req.preview.facetsPtr()` and `binding->numFacets == req.preview.numFacets()` and > 0.
7. Assert `binding->normalPtr == (void*)req.preview.nPtr()`.
8. **Do NOT call `sim.initialize()`** in this block — that's the whole point of pre-init binding.
9. Pass label: `D-042 R-2 / addX registers PreviewBinding immediately (pre-initialize) with stable preview pointers`.

**Design call (6) — Where `registerPreviewBinding` gets called.** 7 `Simulator::addX` wrappers + 1 `loadScene` site:
- `Simulator::addClothFile` (line 4992)
- `Simulator::addFloatMesh` (line 5000)
- `Simulator::addClothGridFast` (line 5024)
- `Simulator::addCloth` (line 5050)
- `Simulator::addSphere` (line 5068)
- `Simulator::addCube` (line 5076)
- `Simulator::addGround` (line 5084)
- `Simulator::loadScene` site at line 6085

Each adds the same 1-block call after `scene.addGeneralMesh(...)`:
```cpp
auto& req = Scene<BE, PR>::requestsGeneralMeshes.back();
renderState.registerPreviewBinding(req.id,
    req.preview.xPtr(), req.preview.numPoints(),
    req.preview.facetsPtr(), req.preview.numFacets(),
    req.preview.nPtr());
```

**Design call (7) — `Simulator::removeMesh` calls `renderState.removeById(id)`.** D-041 turn-2 made `nextMeshId` monotone so removed ids never collide with new ones, but the previewBinding/materialized MeshGL for the removed mesh stays orphaned in the map (small memory leak per remove). Add `renderState.removeById(removedId)` before the request is erased. Free correctness improvement; Estimator may flag as a small NOTE that R-2 added removal-side cleanup as a bonus. Acceptable.

**Design call (8) — `getOrCreate(mesh)` semantics.** Old behavior: if `state.find(mesh.id)` not found, construct MeshGL from `mesh.state.x.ptr` (packed sub-view) + `mesh.adjacency.facets.ptr` + `mesh.state.n.ptr`. New behavior: same outer flow, but the "construct" step has 2 sub-paths:
```cpp
template <typename Mesh>
MeshGL<CPU>& getOrCreate(Mesh& mesh) {
    auto it = state.find(mesh.id);
    if (it != state.end()) return it->second;
    // R-2: prefer preview binding if registered.
    auto pit = previewBindings_.find(mesh.id);
    if (pit != previewBindings_.end()) {
        auto& pb = pit->second;
        it = state.emplace(std::piecewise_construct,
            std::forward_as_tuple(mesh.id),
            std::forward_as_tuple(
                pb.numVerts, (float*)pb.xPtr,
                pb.numFacets, pb.facetPtr,
                (float*)pb.normalPtr
            )).first;
        previewBindings_.erase(pit);
        return it->second;
    }
    // Legacy fallback: construct from packed sub-views.
    it = state.emplace(std::piecewise_construct,
        std::forward_as_tuple(mesh.id),
        std::forward_as_tuple(
            mesh.state.x.size / 3, mesh.state.x.ptr,
            mesh.adjacency.facets.size / 3, mesh.adjacency.facets.ptr,
            mesh.state.n.ptr
        )).first;
    return it->second;
}
```

**Design call (9) — PreviewBinding holds `void*` for `xPtr`/`normalPtr` to avoid template parameter on `MeshRenderState`.** The class is non-templated and shared by all Simulator instantiations (only `Simulator<METAL, float>` exists in v1, but the binding API should not lock in PR). `registerPreviewBinding<PR>(...)` is a function template that casts `PR*` to `void*` internally. `getOrCreate` casts back to `float*` (v1 PR is always `float`). Acceptable simplification per existing pattern (`MeshGL<CPU>` hardcodes `float*` in the ctor signature).

**NEW symbols this slice adds**:
- `MeshRenderState::PreviewBinding` struct (POD).
- `MeshRenderState::previewBindings_` private map.
- `MeshRenderState::registerPreviewBinding<PR>(...)` method (function template).
- `MeshRenderState::previewBinding(int id) const` accessor.
- `MeshRenderState::removeById(int id)` method.
- Block 37 in `runSelfTest` (1 new pass clause).
- `docs/DECISIONS.md` — D-042 R-2 entry.

**MODIFIED symbols in place**:
- `include/MeshRenderState.hpp` — adds the 3 new methods + map + struct.
- `MeshRenderState::getOrCreate` — modified to check `previewBindings_` first.
- `src/main.cpp` — 8 `addX`/`loadScene` sites gain 1-block `registerPreviewBinding` call; `Simulator::removeMesh` gains `renderState.removeById(removedId)` call; `Simulator::initialize` line 5403 retires `renderState.clear()` call (DELETE the line, leave a R-2 explanatory comment).
- `.agent/PROJECT_STATE.md` — next-milestone updates.

**PRESERVED symbols** (parallel-symbol invariant):
- `MeshGL<CPU>` — UNCHANGED. Same ctor signature, same fields.
- `MeshGL<CPU>::updateBuffer` — UNCHANGED. Per-frame re-pointing to packed buffer still works.
- `MeshRenderState::clear()` method body — UNCHANGED (deprecated call site only).
- `MeshRenderState::has(int id)` — UNCHANGED.
- `PreviewState<PR>` — UNCHANGED (R-1's surface).
- `Scene::addGeneralMesh` — UNCHANGED. Still returns void; the wrappers do the binding-register step.
- `RequestGeneralMesh::preview` — UNCHANGED.
- All initializer `populatePreview` overrides — UNCHANGED.
- Block 1-36 — UNCHANGED.
- All Metal kernels, BVH, narrow-phase — UNCHANGED.
- `Simulator::uploadMeshes` / `Simulator::draw` — UNCHANGED. Both still iterate `scene.meshes` and call `getOrCreate`; the binding lookup is transparent.

## Non-goals

- **NO rewrite of `Scene::pack` to memcpy preview → packed.** That's R-3.
- **NO mutation of `translateObject` / `rotateObject` / `setMaterial` to write into PreviewState.** That's R-4.
- **NO packed → preview resync at end of `Simulator::update`.** That's R-5.
- **NO self-test migration to read via Scene::meshes preview alias.** That's R-6.
- **NO renderer iteration over `requestsGeneralMeshes` (vs `scene.meshes`)** — pre-init *render visibility* is enabled at the binding layer but the renderer still iterates `scene.meshes` this slice. End-to-end pre-init render is a separate slice on the R-3+ path.
- **NO eager `MeshGL` ctor at addX time.** GL context not guaranteed at addX (harness). See Design call (1).
- **NO new BDD/FRD/CM.**
- **NO C-* FlatBuffers work.**

## Spec substitution

None this turn. R-2 is infrastructure work on the D-042 architectural-refactor path; the user's R-* sequence is itself the spec (not a BDD-derived behavior).

## Standing constraints

- **RIGID-BACKEND-PORTABILITY** (D-037) — UNCHANGED. R-2 doesn't touch the rigid contract.
- **PARALLEL-IMPL-LOCKSTEP** (D-035 + D-038) — UNCHANGED. No conversion math touched.
- **BDD-102-vs-ALEMBIC-BYTES** — UNCHANGED.
- **DUPLICATED-INSPECTOR-WIRING** — UNCHANGED.
- **GLFWINIT-NON-REF-COUNTED** — UNCHANGED.

## Todo

1. **Branch hygiene.** Working in worktree `.claude/worktrees/r2-meshgl-preview` on branch `feat/d-042-r-2-meshgl-preview-binding` (branched off main HEAD `08f3ef5`). Commit prefix `add:`.

2. **`include/MeshRenderState.hpp` — add `PreviewBinding` struct + private map + 3 new methods** per Design call (3) sketch.

3. **`include/MeshRenderState.hpp` — modify `getOrCreate` to prefer preview binding** per Design call (8) sketch. Legacy `mesh.state.x.ptr` fallback retained.

4. **`src/main.cpp` — wire `registerPreviewBinding` into 8 sites.** After each `scene.addGeneralMesh(...)` call (lines 4993, 5001, 5027, 5054, 5069, 5077, 5085, 6085), insert the binding-register block from Design call (6).

5. **`src/main.cpp` — `Simulator::removeMesh` gains `renderState.removeById(removedId)` call** before the request is erased. Locate the existing removeMesh body and insert the call after the id-of-mesh-to-remove is computed but before the requests vector's erase. Generator: find the existing removeMesh implementation by grepping `removeMesh(` / `removeMeshById` / `int idToRemove`.

6. **`src/main.cpp:5403` — retire `renderState.clear()` call in `Simulator::initialize`.** DELETE the line. Leave the surrounding comment block intact with an addendum: `// D-042 R-2: clear() call retired — MeshGL bound to PreviewState heap pointers (stable across Scene::pack reallocations).`

7. **New Block 37** in `runSelfTest` — inserted INSIDE the Metal-gated section, AFTER Block 36 (D-042 R-1) closes. Body sketch (Generator finalizes exact lines):
   ```cpp
   // ---- Block 37: D-042 R-2 — addX registers PreviewBinding eagerly. ----
   // R-1 populated request.preview. R-2 wires renderState to bind to those
   // preview pointers at addX time. Block 37 verifies the binding exists
   // BEFORE simulator.initialize() runs, proving MeshGL can materialize from
   // preview pointers (not packed sub-views) once a GL context comes online.
   {
       resetScene();
       sim.addCube(tinym::vec3(0.0f, 1.0f, 0.0f), 2, 0.2f, 1.0f);
       size_t req_count = Scene<Backend, Precision>::requestsGeneralMeshes.size();
       bool oneReq = (req_count == 1);
       auto& req = Scene<Backend, Precision>::requestsGeneralMeshes[0];
       const auto* binding = sim.renderState.previewBinding(req.id);
       bool bindingExists = (binding != nullptr);
       bool xMatch     = bindingExists && (binding->xPtr == (void*)req.preview.xPtr());
       bool numVMatch  = bindingExists && (binding->numVerts == req.preview.numPoints()) && (binding->numVerts > 0);
       bool facetPMatch = bindingExists && (binding->facetPtr == req.preview.facetsPtr());
       bool facetCMatch = bindingExists && (binding->numFacets == req.preview.numFacets()) && (binding->numFacets > 0);
       bool nMatch     = bindingExists && (binding->normalPtr == (void*)req.preview.nPtr());

       if (oneReq && bindingExists && xMatch && numVMatch && facetPMatch && facetCMatch && nMatch) {
           pass("D-042 R-2 / addX registers PreviewBinding immediately (pre-initialize) with stable preview pointers");
       } else {
           fail("D-042 R-2 / addX registers PreviewBinding immediately (pre-initialize) with stable preview pointers",
                std::string("oneReq=") + std::to_string((int)oneReq)
                + " bindingExists=" + std::to_string((int)bindingExists)
                + " xMatch="     + std::to_string((int)xMatch)
                + " numVMatch="  + std::to_string((int)numVMatch)
                + " facetPMatch=" + std::to_string((int)facetPMatch)
                + " facetCMatch=" + std::to_string((int)facetCMatch)
                + " nMatch="     + std::to_string((int)nMatch));
       }
   }
   ```
   **Note**: Block 37 needs access to `Simulator::renderState`. Currently `private` (declared at line 4886 as `MeshRenderState renderState;` with default access). Generator: either promote to `public` (minimal blast radius — consistent with `getOrCreate`/`clear` already being public methods) OR add `const MeshRenderState& getRenderState() const` getter. Pick the smallest change.

8. **Build + verify deterministic.** `cmake --build build && cd build && for i in 1 2 3 4 5; do ./src/ysim --self-test; done` — expect **67/67 PASS** each time.

9. **`verify-light.sh` cross-check.** Expect doctest 159/159 + 1120/1120 SUCCESS unchanged.

10. **Bug-probes** (each FAIL after revert; restore after):
    - **(a) Remove the `registerPreviewBinding` call from `Simulator::addCube`**: Block 37 FAILs with `bindingExists=0`. Restore.
    - **(b) Make `registerPreviewBinding` store wrong `xPtr` (e.g., `b.xPtr = nullptr`)**: Block 37 FAILs with `xMatch=0`. Restore.
    - **(c) `MeshRenderState::getOrCreate` legacy-fallback load-bearingness**: harder to verify inside self-test (the legacy path runs only when no preview binding is registered — every R-2 addX registers, so the legacy path is dead in production self-test). Document this gap explicitly in the bug-probe note as expected; the legacy path is a safety fallback for hand-built MeshGL paths (Block 25's HiddenGLContext FBO test directly constructs `MeshGL<CPU> cubeMesh(...)` not via `getOrCreate`, so even there it's not exercised).

11. **Append D-042 R-2 to `docs/DECISIONS.md`** (Generator authors). Sketch:
    > **D-042 R-2 (2026-05-14)** — `MeshGL<CPU>` binding source migrates from packed sub-views to `PreviewState` heap pointers via a new CPU-only `MeshRenderState::registerPreviewBinding(id, xPtr, numVerts, facetPtr, numFacets, normalPtr)` API. Each `Simulator::addX` wrapper + `loadScene` calls `registerPreviewBinding` with `request.preview.*Ptr()` right after `scene.addGeneralMesh`. `MeshRenderState::getOrCreate(mesh)` checks the per-id `previewBindings_` map BEFORE falling back to `mesh.state.x.ptr` (packed); if a binding exists, MeshGL is constructed with the preview pointers + counts, the pending binding entry is consumed, and the cached MeshGL persists across re-init. `Simulator::initialize`'s `renderState.clear()` call (the band-aid for Scene::pack pointer invalidation) RETIRES — the preview heap pointers are stable across packs, so clear() is no longer needed for pointer hygiene. `Simulator::removeMesh` gains `renderState.removeById(removedId)` to avoid orphan binding/MeshGL entries (memory hygiene; D-041 turn-2 nextMeshId already prevents id collisions). Block 37 verifies the binding registration is eager + correct pre-initialize (pure CPU; no GL context needed). PreviewState pointer stability is the load-bearing invariant — std::vector heap-owned, NOT pool-backed, so D-041 pool reset cannot invalidate. Future R-* slices: R-3 rewrites Scene::pack to memcpy preview → packed; R-4 routes translate/rotate/setMaterial through preview; R-5 adds packed→preview resync at update end; R-6 migrates self-test reads via Scene::meshes preview alias; R-7 retires the `renderState.clear()` public API along with other dead code. Self-test count 66 → 67.

12. **Update `.agent/PROJECT_STATE.md`** (Planner-tier task during this planning pass):
    - **Next milestone**: R-2 in flight; R-3+ outlined.
    - **Recent scope changes**: append 2026-05-14 D-042 R-2 entry.

13. **Update `.agent/CURRENT_WORK.md` + `.agent/RESUME.md`** — Generator-tier task.

## Course corrections

- **`feedback_make_means_add_new` rule**: user's brief uses "wire MeshGL to point at PreviewState" — integration verb, but the parallel-symbol invariant is preserved by *not* removing the legacy packed-sub-view fallback in `getOrCreate`. The new code path is preferred when a binding is registered; the old path stays as the implicit fallback. Block 37 verifies the NEW path; existing Blocks 1-36 implicitly verify the legacy path is intact (they don't touch GL but they don't break the binding chain).
- **`project_flatbuffers_caching_skipped`**: stays in force.
- **D-026 lifetimeId invariant**: applies — `registerPreviewBinding` runs BEFORE `Scene::pack`; the preview is the source of truth at registration time, no lifetimeId dependency.
- **D-041 nextMeshId monotone**: applies — binding keyed on `request.id` which never collides with removed ids.
- **CM-012 utility-helper-exit trap**: applies — none of the new methods (`registerPreviewBinding` / `previewBinding` / `removeById`) call `exit()` / `abort()`. All return silently on missing entries.
- **PROBE-COVERAGE-EDGES per PLANNER §9**: not a math-layer slice. Edges considered:
  - Empty scene + binding query → `previewBinding(0)` returns nullptr (covered by `find(id) == end() → nullptr` in `previewBinding`).
  - removeById on non-existent id → `erase` returns 0 silently (std::unordered_map contract).
  - Re-register same id (e.g., scene reload) → overwrites map entry (last-write-wins).
  - PR* type erased through void* round-trip → only safe because v1 PR is always float; Generator confirms by reading MeshGL ctor's float* signature.

## Expected metrics

- Self-test count: **66 → 67** (Block 37 gains 1 pass clause).
- 5-run deterministic PASS on macOS dev host.
- `verify-light.sh`: doctest **159/159 + 1120/1120 SUCCESS** unchanged.
- Linux container: Block 37 SKIPs along with Blocks 1-29 + 32-36 (placed inside Metal-gated section because it uses `Simulator::addCube` → Metal MemoryBlock). Blocks 30 + 31 still run on Linux.
- Expected matrix delta: none (R-2 is infrastructure; no BDD/FRD row touched).
- Expected DECISIONS.md delta: D-042 R-2 entry added.
- Expected PLANNER.md delta: none (no new standing constraint).
- Expected PROJECT_STATE.md delta: next-milestone updates with R-2 → R-3 progression.
- Estimator verdict next turn: **NOTE** if implementation clean. Possible NOTE items:
  - (i) `void*` round-trip in `PreviewBinding` is a small type-safety footgun; acceptable for v1's single PR=float.
  - (ii) `getRenderState()` accessor OR `public:` promotion needed for Block 37 — taste call.
  - (iii) The legacy packed-sub-view fallback in `getOrCreate` is now dead code in production (every `addX` registers a binding); could be deleted in a R-7 cleanup. R-2 keeps it for safety + parallel-symbol invariant.
  - (iv) `removeById` adds a removal hygiene fix that's adjacent to R-2's primary scope — could be flagged as silent scope expansion, but it's a 2-line safety net for D-041 turn-2's monotone-id guarantee.
  - **WARNING** would land if: Block 37 FAILs OR existing Blocks 1-36 regress (PASS count < 66 baseline retained) OR `renderState.clear()` retirement breaks a Block 26+ dirty/pool path that Generator missed during the build.
  - **BLOCK** if Block 37 FAILs on macOS OR pre-existing PASS count regresses OR Scene::pack assumes `renderState.clear()` was called (none observed in current code; Generator confirms by grepping `renderState.clear`).
