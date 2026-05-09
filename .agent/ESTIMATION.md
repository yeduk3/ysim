# Estimation — 2026-05-09 turn 10

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
- BDD-019: pass

## Verify output (summary)
`./scripts/verify.sh` exited 0. CMake rebuilt `ysim`, `ysim_tests`, and `ysim_primitive_tests`; both doctest binaries passed (159/159 and 1120/1120 assertions). The `ysim --self-test` stage hit the expected Metal-unavailable SKIP path on this host, which is non-failure per the role. The diff closes the BDD-019 proxy check by introducing `profiler::ProfilerFrameGate` in `include/FrameProfiler.hpp` and using the same `!sim.pause` predicate in both the render loop and `runSelfTest`.
