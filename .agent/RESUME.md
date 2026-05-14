# Resume — D-042 R-3 — Scene::pack memcpys from PreviewState

## Must remember

- **R-3 made preview LOAD-BEARING for vertex data at pack time.** Pre-R-3 memcpy was a no-op overlay (initializer's regen byte-equaled preview); R-3's Block 38 sentinel test proves the memcpy now overrides the initializer's regen.
- **translateObject DUAL-WRITES state.x AND preview.x.** Without this, R-3's memcpy would revert translate's effects (5 self-test failures bitten on first build). R-4 will make preview the primary; R-3 dual-write is the transitional bridge.
- **rotateObject does NOT need preview dual-write.** Rotation persists through pack via D-025 pendingRotations + applyPendingMaterials (re-applied AFTER pack to the post-memcpy state.x). Verified by Block 38 + BDD-018 rotate clause both passing.
- **mesh.initialize() still runs for adjacency.** PreviewState doesn't include `vertexAdjFacets / vertexAdjEdges` + offsets. R-7 cleanup could refactor to skip initialize()'s data work and run adjacency-only.
- **Latent NOTE**: harness `resetScene` lambdas don't call `clearPreviewBindings` (no GL ctx → stale bindings inert). Document only; no fix needed for v1.

## Last decisions + why

- **D-042 R-3 entry in DECISIONS.md** — captures the size-guarded memcpy contract, translate dual-write rationale, clearPreviewBindings wire site, Block 38's sentinel mechanic, and the bug-probe verification.
- **No new D-NNN beyond R-3 itself.** No BDD/FRD touched. No CM-NNN.
- **Scope expansion**: `translateObject` preview dual-write — minimal R-4 precursor (6 lines) to keep BDD-003/018 green. Documented inline + in DECISIONS.

## Next step you were about to take

Hand back to `/slice` orchestrator. Codex Estimator runs next. After R-3 merges, the next slice is **R-4**: route `translateObject` / `rotateObject` / `setMaterial` THROUGH preview as the primary mutation target (state.x derived from preview at pack/sync time). Will finish what R-3's translate dual-write started.

See `PLAN.md` and `CURRENT_WORK.md` for slice scope + status.
