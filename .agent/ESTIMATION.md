# Estimation — 2026-05-12 turn 26

Status: UPDATED

## Verdict
WARNING

## BLOCK
(none)

## WARNING
- `docs/DECISIONS.md:281`, `.agent/CURRENT_WORK.md:21`, `.agent/PROJECT_STATE.md:47`, `profiles/experiment/bvh-refit-2026-05-12/refit_bench.csv:112` — the written takeaway says Hybrid D=1/D=2 never beat FullCPU, but the committed CSV contradicts that at 499,849 vertices: the HybridD2 sweep is faster than the matching FullCPU sweep. The downstream tuning recommendation is still plausible, but this specific conclusion is wrong and needs to be rewritten before the follow-up slice uses it.

## NOTE
- `profiles/experiment/bvh-refit-2026-05-12/README.ko.md:110` — the artifact section still describes `refit_bench.csv` as a header-only placeholder, but the committed CSV already contains the populated 160-row sweep. Align the prose with the shipped artifact.

## Test matrix delta
(none)

## Verify output (summary)
`./scripts/verify.sh` configured and built successfully; both doctest suites passed (`159/159` and `1120/1120`). On this host, `src/ysim --self-test` took the expected Metal-less skip path (`MTL::CreateSystemDefaultDevice() returned null`), so the new bench smoke path was not re-exercised here. The committed benchmark artifacts are present: `refit_bench.csv` has 160 data rows and the two PNG charts are included.
