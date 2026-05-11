# Estimation — 2026-05-12 turn 25

Status: UPDATED

## Verdict
WARNING

## BLOCK
- none

## WARNING
- Hybrid default depth is unmeasured: [src/main.cpp](/Users/gyu/codes/ysim/src/main.cpp):3181 - `bottomUpHybridDepth = 2` is now the default path for build/refit, but the slice explicitly treats 2 as a starting point and does not benchmark or tune it. That can silently pick the wrong D for real scenes, so performance risk remains.

## NOTE
- none

## Test matrix delta
- none

## Verify output (summary)
`./scripts/verify.sh` exited 0. The build completed, both doctest binaries passed (159/159 and 1120/1120 assertions), and the main executable reported `[self-test SKIP] metal-device: MTL::CreateSystemDefaultDevice() returned null`, which is the expected non-Metal-host skip rather than a failure.
