# Estimation — 2026-05-09 turn 8

Status: UPDATED

## Verdict
WARNING

## BLOCK
- none

## WARNING
- `BDD-019 / history collection pauses when sim pauses`: [src/main.cpp](/Users/gyu/codes/ysim/src/main.cpp):6082-6093 only proves that skipping `beginFrame()` / `endFrame()` leaves history unchanged. It does not exercise the actual paused-frame branch in the render loop at [src/main.cpp](/Users/gyu/codes/ysim/src/main.cpp):6331-6622, so a regression in the real pause gate could slip through.

## NOTE
- [src/main.cpp](/Users/gyu/codes/ysim/src/main.cpp):6056-6079 deliberately writes the profiler CSV to `/tmp/ysim_profiler_test.csv` instead of `profiles/` for harness hygiene. If this block ever becomes a real UI-level integration test, switch the path back to the spec literal so the substitution stops living only in comments.

## Test matrix delta
- BDD-003: pass
- BDD-019: pass

## Verify output (summary)
`./scripts/verify.sh` completed successfully. CMake rebuilt `ysim`, `ysim_tests`, and `ysim_primitive_tests`; both doctest binaries passed (11/11 and 9/9 test cases, 159 and 1120 assertions). The `ysim --self-test` harness exited 0 with the expected `[self-test SKIP] metal-device ...` line because this host does not expose a Metal device, so the Metal-backed self-test blocks were not executed here.
