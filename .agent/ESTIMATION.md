# Estimation — 2026-05-12 turn 24

Status: UPDATED

## Verdict
WARNING

## BLOCK
- none

## WARNING
- `src/main.cpp:7374` and `docs/DECISIONS.md:264` - Block 22 proves the N=1 happy path, but the negative probe is Apple-Silicon-masked; removing the guard and forcing `treeParent[0]` did not fail on this host, so the canary does not currently catch the regression it documents.

## NOTE
- none

## Test matrix delta
- BDD-010: pass

## Verify output (summary)
`./scripts/verify.sh` completed successfully in this container: the build succeeded, both doctest binaries passed, and the self-test took the expected Metal SKIP path (`[self-test SKIP] metal-device: ...`) because this host does not expose a Metal device. I did not see a spec mismatch in the D-029 guard, the new Block 22 mechanization, or the BDD-010 matrix wording update.
