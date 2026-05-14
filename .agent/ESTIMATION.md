# Estimation — 2026-05-14 turn 40

Status: UPDATED

## Verdict
WARNING

## BLOCK
- None.

## WARNING
- Scope drift vs plan: `src/main.cpp:5299` removes `pendingRotations[meshId] = newAbs;` even though `.agent/PLAN.md:109` said the D-025 pendingRotations re-apply flow would stay intact this slice. The code appears to need the change to avoid double-rotation, but this is no longer the narrow "preview write only" slice the plan described.

## NOTE
- `src/main.cpp:5236` still says the pack-roundtrip write-back is deferred to a later slice. That prose is now stale after the R-4 preview write-back landed.
- `src/main.cpp:6246` still describes `rotateObject()` as writing into `pendingRotations`. The runtime path is fine, but the comment now reflects the pre-R-4 implementation.

## Test matrix delta
- none: no behavior-id rows changed; the slice is infrastructure-only and the existing BDD-018 coverage stays intact.

## Verify output (summary)
`./scripts/verify.sh` completed successfully. CMake configured and built the tree, with only third-party Bullet warnings. `ysim_tests` passed 159/159 assertions and `ysim_primitive_tests` passed 1120/1120 assertions. The self-test gate reported the Null/Euler backend checks as PASS and then hit the expected Metal-less SKIP on this host (`MTL::CreateSystemDefaultDevice()` returned null), so the new Metal-gated Block 39 was not exercised here.
