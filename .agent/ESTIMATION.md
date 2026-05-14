# Estimation — 2026-05-14 turn-39

Status: UPDATED

## Verdict
NOTE

## BLOCK
- None.

## WARNING
- None.

## NOTE
- `src/main.cpp:6090` — the prior scene-boundary cache cleanup BLOCK is resolved: `loadScene` now clears both the materialized MeshGL cache and the pending preview bindings, so a same-process reload cannot reuse stale `MeshGL` pointers after `nextMeshId` resets.
- `src/main.cpp:5186` — `translateObject` still dual-writes `preview.x` as a bridge to R-4; keep `state.x` and `preview.x` in lockstep when touching that mutation path.
- `src/main.cpp:10171` — Block 38 proves the `x` memcpy is load-bearing; the `n` / `facets` copies remain covered by the same pack path but are not separately bug-probed.

## Test matrix delta
- BDD-003: pass
- BDD-018: pass

## Verify output (summary)
`./scripts/verify.sh` exited 0. CMake configured and built successfully, doctest passed (`159/159` and `1120/1120` assertions), and `./src/ysim --self-test` hit the expected Metal-device SKIP on this host (`[self-test SKIP] metal-device: MTL::CreateSystemDefaultDevice() returned null`), so the Metal-gated self-test block did not run here. Code review of the diff shows the reload path now clears both render-state caches, which resolves the previously BLOCKed scene-boundary cleanup issue.
