# Estimation — 2026-05-14 turn-37

Status: UPDATED

## Verdict
WARNING

## BLOCK
- None

## WARNING
- `src/main.cpp:6007` / `include/MeshRenderState.hpp:75` — `loadScene` and the reset helpers reset `Scene::nextMeshId` to 0, but the new R-2 render cache is never cleared there. Because `getOrCreate` returns an existing `MeshGL` before consulting a fresh preview binding, a scene reload/reset can reuse stale VAO/VBO sizes and pointers from the prior scene. The `renderState.clear()` retirement is safe for pack churn, but not for scene-boundary churn.

## NOTE
- `include/MeshRenderState.hpp:114` — `clear()` still only clears the cached `MeshGL` map and does not touch `previewBindings`. That is fine now that the call site is retired, but any future forced-rebuild path should not assume it is a full reset.

## Test matrix delta
- none

## Verify output (summary)
`./scripts/verify.sh` exited 0. The repo configured and built successfully, with only upstream Bullet/CMake deprecation warnings in the log. Both doctest binaries passed (`159/159` and `1120/1120` assertions). On this host `./src/ysim --self-test` correctly emitted `[self-test SKIP] metal-device: MTL::CreateSystemDefaultDevice() returned null (non-macOS host or container without Metal)`, so the Metal-gated self-test block did not run here, but no failing tests were reported.
