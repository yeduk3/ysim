# Estimation — 2026-05-10 turn 14

Status: UPDATED

## Verdict
WARNING

## BLOCK
- None

## WARNING
- `src/main.cpp:6532-6536` — the overlapping-case gate only checks `numHits >= 2`, but `queryClickRay()` returns per-triangle hits. A single cube can satisfy that count, so this clause does not prove the back cube participated; an overlapping-object regression could still pass.
- `src/main.cpp:6434-6440` — the slice validates the BVH query / nearest-hit logic, but it never exercises the production click callback’s `selectedObj = closestObj` assignment at `src/main.cpp:6718`. The BDD wording names the selected object, so a callback-side regression would be invisible here.

## NOTE
- None

## Test matrix delta
- `BDD-017`: pass

## Verify output (summary)
`./scripts/verify.sh` completed successfully: the build finished, both doctest binaries passed (`11/11` and `9/9` test cases), and the Metal-backed self-test reported the expected `[self-test SKIP] metal-device` line on this non-Metal host.
