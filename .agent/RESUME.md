# Resume — D-042 R-6 — Block 41 pins preview ≡ state.x byte-equal invariant

## Must remember

- **Block 41 is a long-lived guard.** It will catch any future slice that breaks the R-3+R-4+R-5 round-trip (resync drop, edit dual-write break, pack memcpy regression).
- **memcmp is bit-strict.** R-5's `std::memcpy` is bit-identical so memcmp == 0 is the expected outcome. If a future floating-point reordering shifts bits, Block 41 fires; that's the correct signal.
- **Bug-probe (a) breaks 5 distinct tests.** Confirms the byte-equality invariant isn't just a Block 41 curiosity — it's load-bearing for translate/rotate/changeBehavior pack-survival across the whole suite. Restored.
- **R-7 (final) candidates**: stale comment cleanup (legacy regen language), preview.n stale-ness decision (recompute vs retire field), getOrCreate fallback retirement (now dead code given universal preview-binding registration).

## Last decisions + why

- **D-042 R-6 entry in DECISIONS.md** — captures the invariant chain (R-3→R-4→R-5), Block 41's memcmp shape, and the bug-probe's cross-suite impact.
- **No new D-NNN beyond R-6 itself.**
- **No scope expansion** — pure test-only slice.

## Next step you were about to take

Hand back to `/slice` orchestrator. Codex Estimator runs next. After R-6 merges, the final D-042 slice (R-7) handles cleanup: stale comments, decision on preview.n recompute vs retire, optional retirement of `getOrCreate`'s legacy packed-sub-view fallback now that R-2 registerPreviewBinding fires for every addX.

See `PLAN.md` and `CURRENT_WORK.md` for slice scope + status.
