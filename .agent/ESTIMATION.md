# Estimation — 2026-05-14 turn-36

Status: UPDATED

## Verdict
WARNING

## BLOCK
- none

## WARNING
- `docs/TESTS.md:83-85` / `src/main.cpp:9545-9562` — Block 33 treats the "rest" clause as a position-delta proxy (`|Δy| < 0.001`) after a fixed 240-frame run, but the BDD wording requires the box to advance until kinetic energy stays below threshold for a sustained window and for angular velocity to have decayed. The new `pass` label is plausible, but the assertion is weaker than the spec it claims to close.

## NOTE
- `src/main.cpp:5231-5266` — the rigid delta-loop is translation-only, so Bullet rotation is not propagated back into `state.x`. That is acceptable for the current cube-only slice, but asymmetric rigid shapes will need a follow-up rotation path.

## Test matrix delta
- BDD-008: pass

## Verify output (summary)
`./scripts/verify.sh` completed successfully on this host. CMake configured and built the vendored Bullet subproject plus `ysim`; both doctest binaries passed (`159/159` and `1120/1120` assertions). The self-test ran the D-037 Null clauses and D-038 Euler clauses, then hit the expected `[self-test SKIP] metal-device` on this non-Metal host before Blocks 32 and 33, so the new Bullet-backed end-to-end assertions were not exercised in this local run.
