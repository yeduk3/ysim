# Current Work — D-042 R-6 — preview ≡ state.x byte-equal invariant pinned by Block 41

> Owner: **Generator**.
> Updated: 2026-05-14

## File in flight

None — slice complete pending Estimator review.

## How far

- `src/main.cpp` — new Block 41 inside the Metal-gated section after Block 40. addCloth + 5 sim.update() iterations; per-frame `memcmp(state.x.ptr, preview.x.data(), bytes) == 0` assertion; tracks frameMismatch + eqAllFrames counters.
- `docs/DECISIONS.md` — D-042 R-6 entry appended (R-3+R-4+R-5 invariant chain documented).
- No production code changes — pure test-only slice.

## What's tested

- `./src/ysim --self-test` — **72/72 PASS deterministic across 5 macOS runs**.
- `./scripts/verify-light.sh` — doctest 159/159 + 1120/1120 SUCCESS, unchanged.
- Bug-probe (a): corrupting the R-5 resync (`preview.x[0] = state.x[0] + 0.01f`) → Block 41 FAILs + BDD-003 + BDD-018 (×2) + BDD-006 also FAIL because corruption propagates to state.x via next pack memcpy. Confirms the byte-equality invariant is load-bearing across the entire suite. Restored.
- No `BUG-PROBE` markers remain.

## What's next

Hand back to `/slice` orchestrator for Codex Estimator pass. Final D-042 slice (R-7) is cleanup: stale comments referring to legacy regen path as canonical, retire dead getOrCreate fallback if uniformly preview-bound, decide on preview.n stale-ness.
