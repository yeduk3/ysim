# Resume — D-042 R-2 — MeshGL ↔ PreviewState binding

## Must remember

- **User's R-* sequence is the spec.** R-1 (08f3ef5) shipped PreviewState; R-2 ships the MeshGL-binding path; R-3..R-7 are: pack memcpy, edit-side preview mutation, post-update packed→preview sync, self-test migration, cleanup. Each slice is a parallel-symbol layer; existing path must keep working.
- **`MeshGL::updateBuffer` re-points `vertexPtr` every frame.** R-2's preview pointer matters only at MATERIALIZATION (the first `getOrCreate(mesh)` inside a GL context); subsequent `uploadMeshes` calls re-point to `mesh.state.x.ptr` (packed) via `updateBuffer`. End-to-end pre-init render visibility needs R-3+ (renderer iterating over previews instead of `scene.meshes`).
- **Latent hygiene gap (Generator-discovered)**: `resetScene` clears `Scene::requestsGeneralMeshes` (destroys preview vectors) but does NOT clear `renderState.previewBindings`. Stale entries point at freed memory. Bug-probe (a) surfaced this — the new addCube without re-register read a STALE binding from a prior addCube. Harmless under R-2 (no GL-context `getOrCreate` call between resetScene and the next addX), but worth folding into R-7 cleanup OR a small fix-turn (add `renderState.previewBindings.clear()` to `resetScene` lambda + production resetScene paths).
- **`Simulator` is `struct` (default public).** Block 37 reaches `sim.renderState.previewBinding(...)` directly. No `friend` / getter needed.
- **`MeshRenderState` is non-templated.** `void*` round-trip in PreviewBinding keeps it backend-agnostic. v1's single `PR = float` instantiation means the `(float*)pb.xPtr` cast in `getOrCreate` is exact.

## Last decisions + why

- **D-042 R-2 entry in DECISIONS.md** — captures the parallel-symbol invariant (legacy packed-sub-view fallback retained), the 8 wire sites, the `removeById` hygiene fix, the `renderState.clear()` retirement rationale, and the bug-probe results.
- **No new D-NNN beyond R-2 itself.** No BDD/FRD touched. No CM-NNN.
- **`registerPreviewBindingForLastRequest()` helper** rather than inline 8 copies of the same 5-line block. Compromise between PLAN's "inline" wording and DRY.

## Next step you were about to take

Hand back to `/slice` orchestrator. Codex Estimator runs next. After R-2 merges, the next slice is **R-3**: rewrite `Scene::pack` so the packed buffers are memcpy'd FROM PreviewState (instead of `mesh.initialize()` regenerating them from the initializer against pool-backed `MeshState`). R-3 will close the divergence between preview and packed at pack time; R-5 closes it after each `Simulator::update`.

See `PLAN.md` and `CURRENT_WORK.md` for slice scope + status.
