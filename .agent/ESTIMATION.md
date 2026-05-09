# Estimation — 2026-05-08 turn 7

Status: UPDATED

## Verdict
WARNING

## BLOCK
- none

## WARNING
- `src/main.cpp:4432-4448` and `src/main.cpp:1773-1789` - `translateObject()` updates only the realized-mesh state. Any later `Scene::pack()` triggered by create/import/load flows will rebuild from the stale initializer data and silently drop a prior translate.
- `include/FrameProfiler.hpp:24-140`, `src/main.cpp:4603-4610`, and `scripts/analyze_profile.py:1-245` - profiler collision-count export/analysis tooling was added outside the BDD-003 plan, while `docs/TEST_MATRIX.md:33` still leaves BDD-019 pending. That is scope creep plus missing coverage for the new profiler behavior.

## NOTE
- `src/main.cpp:4070` and `docs/mistakes/COMMON_MISTAKES.md:57-71` - CM-006 remains parked on purpose; the folded `nparams.thickness` warning was deferred instead of being resolved, because plumbing a non-zero thickness reintroduced the BDD-007 regression.

## Test matrix delta
- BDD-003: pass
- BDD-019: pending

## Verify output (summary)
`./scripts/verify.sh` completed successfully. CMake configured and built `ysim`, `ysim_tests`, and `ysim_primitive_tests`; both doctest binaries passed (11/11 and 9/9 test cases, 159 and 1120 assertions); the `ysim --self-test` harness exited 0 with the expected `[self-test SKIP] metal-device ...` line because this host does not provide Metal.
