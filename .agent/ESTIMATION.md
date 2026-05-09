# Estimation — 2026-05-10 turn 15

Status: UPDATED

## Verdict
NOTE

## BLOCK
- None

## WARNING
- None

## NOTE
- `src/main.cpp:3819-3833`, `docs/DECISIONS.md:186-192`, `docs/mistakes/COMMON_MISTAKES.md:61-67` — the slice also picked up a production BVH leaf-return fix (D-020 / CM-009) after the stricter overlapping assertion exposed it. The fix is sound and documented, but it is a scope expansion beyond the original two turn-15 fold-ins.

## Test matrix delta
- `BDD-017`: pass

## Verify output (summary)
`./scripts/verify.sh` rebuilt successfully; both doctest binaries passed (`159/159` and `1120/1120` assertions), and `ysim --self-test` took the expected `[self-test SKIP] metal-device: MTL::CreateSystemDefaultDevice() returned null (non-macOS host or container without Metal)` path on this host, so the script exited 0 without running the Metal-backed assertions.
