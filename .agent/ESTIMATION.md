# Estimation — 2026-05-06 turn-2
Status: UPDATED

## Verdict
WARNING

## BLOCK
- None

## WARNING
- Runtime persistence is only indirectly verified: [scripts/verify.sh](/Users/gyu/codes/ysim/scripts/verify.sh) is absent, so I could only run [scripts/verify-light.sh](/Users/gyu/codes/ysim/scripts/verify-light.sh#L1); [test/scene_io_test.cpp](/Users/gyu/codes/ysim/test/scene_io_test.cpp#L146) still checks BDD-015 via JSON byte equality instead of an actual `Simulator::saveScene/loadScene` plus simulation-step round-trip, so the app-level boundary in [src/main.cpp](/Users/gyu/codes/ysim/src/main.cpp#L4687) remains unexercised.

## NOTE
- [src/main.cpp](/Users/gyu/codes/ysim/src/main.cpp#L5350) loads the scene and immediately calls `initialize()` plus `applyPendingMaterials()`, which keeps the UI path coherent; if load warnings should reach users, the new `SceneSnapshot::warnings` channel still needs to be threaded into `sceneIOStatus` or logging.

## Test matrix delta
- BDD-014: pass
- BDD-015: pass
- BDD-016: pass

## Verify output (summary)
`./scripts/verify.sh` is not present in the repo, so the exact role-required gate could not be run. I ran `./scripts/verify-light.sh` instead; it built `ysim_tests` successfully and passed 9 doctest cases / 142 assertions. The current slice now resolves the earlier primitive-shape, rotation round-trip, import-path, and material-clamping mismatches, but the remaining risk is that the simulator save/load wrapper is only indirectly covered by the JSON-layer tests.
