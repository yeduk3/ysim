# Resume — D-042 R-5 — packed→preview resync at end of update

## Must remember

- **R-5 resync is at the VERY END of Simulator::update.** After dirty-rebuild, applyEnvironmentForces, rigid Δpos, substep loop, AND `frame++`. Earlier placement would either miss substep state or run on stale buffers.
- **Size-guarded memcpy.** If preview's numPoints != state's vertex count, silently skip — defensive for future no-preview initializer paths.
- **preview.n is intentionally NOT recomputed in the resync.** Current renderer uses MeshGL's normals refreshed via updateBuffer. R-7 cleanup can decide whether to recompute preview.n or retire the field.
- **Block 40 cloth scenario falls hard without ground.** post_y reaches ~-0.98 over 30 frames; assertion threshold 0.20 is well-conservative.
- **Bug-probe (a) demonstrates load-bearingness through three assertion clauses:** `fellOk` (qualitative drop), `movedOk` (preview vs initial), `resyncOk` (preview vs state byte-equal). Disabling the resync fails all three.

## Last decisions + why

- **D-042 R-5 entry in DECISIONS.md** — captures the resync placement, size guard, preview.n staleness decision, Block 40 assertion shape, and bug-probe verification.
- **No new D-NNN beyond R-5 itself.**
- **No scope expansion** this slice. R-5 is purely additive.

## Next step you were about to take

Hand back to `/slice` orchestrator. Codex Estimator runs next. After R-5 merges, the final pre-cleanup slice is **R-6**: migrate self-test reads so blocks that currently access `Scene::meshes[i].state.x` can equivalently read `Scene::requestsGeneralMeshes[i].preview.x` after `sim.update()`. R-6 also establishes the `Scene::meshes` alias semantics promised in the original D-042 design (Scene::meshes as a view over PreviewState).

See `PLAN.md` and `CURRENT_WORK.md` for slice scope + status.
