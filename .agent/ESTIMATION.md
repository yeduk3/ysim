# Estimation — 2026-05-07 turn 2
Status: UPDATED

## Verdict
NOTE

## BLOCK
- None.

## WARNING
- None.

## NOTE
- `src/main.cpp:5396-5428` writes the self-test scene to a fixed `/tmp/ysim_selftest.ysim.json` and only removes it on the success path; a unique temp path or cleanup guard would avoid stale files if save/load exits early.

## Test matrix delta
- BDD-009: pass
- BDD-011: pass
- BDD-012: pass
- BDD-015: pass

## Verify output (summary)
`./scripts/verify.sh` configured and built `ysim`, `ysim_tests`, and `ysim_primitive_tests` successfully. Both doctest binaries passed (11/11 and 9/9 cases). `./src/ysim --self-test` exited 0 with `[self-test SKIP] metal-device: MTL::CreateSystemDefaultDevice() returned null (non-macOS host or container without Metal)`, so the Metal-backed assertion branches were not exercised in this container.
