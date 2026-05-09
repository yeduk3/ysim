# Estimation — 2026-05-09 turn 9

Status: UPDATED

## Verdict
NOTE

## BLOCK
- none

## WARNING
- none

## NOTE
- none

## Test matrix delta
- BDD-007: pass

## Verify output (summary)
`./scripts/verify.sh` exited 0. CMake rebuilt `ysim`, `ysim_tests`, and `ysim_primitive_tests`; both doctest binaries passed (159/159 and 1120/1120 assertions). The `ysim --self-test` stage hit the expected Metal-unavailable SKIP path on this host, so the runtime cloth clause was not re-executed locally; the separate macOS runs recorded in `CURRENT_WORK.md` report 23/23 self-test PASS across 5 runs. The implemented changes match the plan: both cloth kernels gate vn-zero behind `(distance < thickness)`, `Simulator::update` now passes `margin`, and the CM-006 / D-016 doc updates are in place.
