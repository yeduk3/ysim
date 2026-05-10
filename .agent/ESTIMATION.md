# Estimation — 2026-05-10 turn 17

Status: UPDATED

## Verdict
WARNING

## BLOCK
- None

## WARNING
- /Users/gyu/codes/ysim/src/main.cpp:4496, /Users/gyu/codes/ysim/src/main.cpp:4545, /Users/gyu/codes/ysim/src/main.cpp:4635, /Users/gyu/codes/ysim/src/main.cpp:4697, /Users/gyu/codes/ysim/src/main.cpp:6545, /Users/gyu/codes/ysim/src/main.cpp:6856, /Users/gyu/codes/ysim/docs/specs/FRD.md:192 - translateObject and rotateObject move the live mesh, but click selection still walks the BVH that is only refit inside sim.update(). On a paused sim, or on the first click before the next update tick, the picker can still select from the old pose instead of the visible translated/rotated mesh. The current BDD-017 coverage only exercises queryClickRay after a refit, so this path is untested.

## NOTE
- None

## Test matrix delta
- BDD-003: pass
- BDD-004: pass
- BDD-017: pass

## Verify output (summary)
./scripts/verify.sh rebuilt successfully; both doctest binaries passed (159/159 and 1120/1120 assertions), and the Metal-backed self-test took the expected SKIP path on this host: `[self-test SKIP] metal-device: MTL::CreateSystemDefaultDevice() returned null (non-macOS host or container without Metal)`. The script exited 0 with no build/test failures.
