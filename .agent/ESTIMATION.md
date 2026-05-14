# Estimation — 2026-05-14 43

Status: UPDATED

## Verdict
WARNING

## BLOCK
- none

## WARNING
- [src/main.cpp:10540](/Users/gyu/codes/ysim/.claude/worktrees/r7-cleanup/src/main.cpp#L10540) — Block 42 now uses a corrupt-and-restore sentinel probe. That proves `recomputeNormals()` runs, but it does not actually exercise the deformation witness described in [.agent/PLAN.md:36](/Users/gyu/codes/ysim/.claude/worktrees/r7-cleanup/.agent/PLAN.md#L36), so the test is narrower than the slice description and could miss a regression in the geometry-driven path.

## NOTE
- [src/main.cpp:5822](/Users/gyu/codes/ysim/.claude/worktrees/r7-cleanup/src/main.cpp#L5822) and [src/main.cpp:10545](/Users/gyu/codes/ysim/.claude/worktrees/r7-cleanup/src/main.cpp#L10545) still carry pre-fix narrative that no longer matches the final implementation. The surrounding comments should be tightened so they describe the recompute path and the final sentinel-based Block 42, not the abandoned draft.
- [src/main.cpp:5842](/Users/gyu/codes/ysim/.claude/worktrees/r7-cleanup/src/main.cpp#L5842) makes the preview-normal rebuild O(F+V) per mesh per frame. That is fine for the current v1 scene sizes, but it is the first place I would revisit if poly counts grow.

## Test matrix delta
- none

## Verify output (summary)
`./scripts/verify.sh` rebuilt the project successfully and both doctest binaries passed (`159/159` and `1120/1120`). `./src/ysim --self-test` passed all `73/73` checks, with the expected `[self-test SKIP] metal-device` on this host. The only non-fatal build noise was the existing Bullet `memcpy`/`memset` warnings from third_party code.
