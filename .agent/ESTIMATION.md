# Estimation — 2026-05-07 turn 6
Status: UPDATED

## Verdict
WARNING

## BLOCK
- none

## WARNING
- [src/main.cpp:4049](/Users/gyu/codes/ysim/src/main.cpp#L4049), [src/metal/bruteforce.metal:121](/Users/gyu/codes/ysim/src/metal/bruteforce.metal#L121) - the new CCD path still hardcodes `nparams.thickness = 0`, so the slow-touch fallback is radius-only instead of the `radius + cloth-thickness` band the slice plan called for. That narrows the acceptance window and leaves the result more dependent on the `subSteps = 8` bump.

## NOTE
- [src/main.cpp:5485-5552](/Users/gyu/codes/ysim/src/main.cpp#L5485-L5552) - Block 6 still uses the Float ground plane as the rigid stand-in, while `docs/TESTS.md#BDD-007` names a sphere. The plan documents this substitution because v1 has no Rigid backend, so it is acceptable as a surrogate but not a literal match to the BDD wording.

## Test matrix delta
- BDD-002: pass
- BDD-007: pass

## Verify output (summary)
`./scripts/verify.sh` rebuilt `ysim`, `ysim_tests`, and `ysim_primitive_tests` successfully. Both doctest binaries passed (11/11 and 9/9 cases). The headless `--self-test` step exited 0 but skipped on this host because `MTL::CreateSystemDefaultDevice()` returned null, so the Metal-backed regression path was not exercised in this environment.
