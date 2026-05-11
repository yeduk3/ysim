# Resume — BVH refit benchmarking harness (D-031; `--bench-bvh-refit` CLI flag; FrameProfiler-based timing)

## Must remember

- **Branch:** `feat/bvh-refit-bench` (off `main` at `febc19b`).
- **D-031 closes Estimator turn 25's WARNING** ("default `bottomUpHybridDepth=3` unmeasured") by producing actual wall-clock data. The slice ships the HARNESS (CLI flag + chart pipeline + Korean README), not the tuned default — choosing the new default is a follow-up "Tune-default-bottomUpHybridDepth" slice candidate in PROJECT_STATE.
- **Measured finding (surprising):** FullGPU wins decisively at 100k (~1.8×) and 500k (~2.0×); Hybrid D=1 / D=2 do NOT beat FullCPU at any measured size. D-030's hybrid-sweet-spot hypothesis is NOT supported by Apple Silicon Metal 3.2 measurement.
- **`runRefitBench` is a NEW free function**, sibling of `runSelfTest` in `src/main.cpp`. Both will move out together once the source-file split slice lands. Per user note in the slice brief ("이런 테스트들은 나중에 main 안에 있는 클래스들이 파일로 쪼개지면 테스트도 모두 분리해낼 계획임."), this is explicitly deferred.
- **Method labels → depth knob:**
  | Label      | `bottomUpHybridDepth` | What it actually runs                                  |
  |------------|-----------------------|--------------------------------------------------------|
  | FullCPU    | 0                     | `bottomUpCombine()` (pure CPU, GPU dispatch skipped)   |
  | HybridD1   | 1                     | `bottomUpBoxesPartialGPU(1)` + `bottomUpCombineWithSkip` |
  | HybridD2   | 2                     | `bottomUpBoxesPartialGPU(2)` + `bottomUpCombineWithSkip` |
  | FullGPU    | 30                    | `bottomUpBoxesPartialGPU(30)` (kernel walks to root)   |
- **"FullGPU" is NOT strict D-029.** It routes through `bottomUpBoxesPartialGPU(30)`, which has 1 extra register-read + 1 extra compare per kernel-loop iteration vs `bottomUpBoxesGPU(sceneBox)`. Documented as sub-noise vs the atomic + 2 seq_cst fences in the same iteration. A future "Strict-D-029-column" follow-up slice can add a fifth method if the question matters.
- **Timing source = FrameProfiler's `broad_refit` scope.** Already instrumented at `src/main.cpp:4910` (SH path) and `:4927` (BVH path). `runRefitBench` wires its own local `FrameProfiler`, reads `history().sectionIndex("broad_refit")` + `latestFrame()` after `endFrame()`. **DO NOT rename the `broad_refit` scope** — the bench writes `-1` silently if the section name lookup fails (the chart.py skips negative rows). Keep the name stable.
- **Block 24 smoke is mechanism-only, NOT method-identity.** Bug-probe (a) — disable CSV row write — FAILs loudly. Bug-probe (b) — wrong depth mapping — PASSes silently because the smoke doesn't validate which method ran. The full-bench output is the user's gate for method-identity correctness.
- **The bench mutates `bottomUpHybridDepth` per run on the per-mesh `objTrees[i]` AND `broadPhase.tree` (scene-level).** Scene-level is trivial for cloth-only (1 leaf) but set for consistency / future multi-mesh scenes.
- **`[Pool] Tried to allocate more than the capacity` warnings appear during 500k bench runs.** Non-fatal; bench still produces correct row counts. Worth investigating in a follow-up but not load-bearing for this slice's claims.

## Last decisions + why

- **D-031 — parallel-symbol bench harness with runtime-knob method selection.**
  - All 4 methods route through D-030's existing `bottomUpHybrid` runtime knob (`bottomUpHybridDepth`), no new dispatch enum in production code.
  - FrameProfiler's existing `broad_refit` scope is the timing source — no new instrumentation hook needed.
  - CSV is the canonical output schema; matplotlib chart is generated post-run by the shipped `chart.py`.
  - Block 24 = mechanism smoke only (CSV header + 1 row + parseable refit_ms ≥ 0); does not assert perf correctness because perf is noisy + non-deterministic.
  - "FullGPU" approximates strict D-029 via `bottomUpBoxesPartialGPU(30)` — depth-counter overhead is sub-noise; strict comparison is a follow-up candidate.

## Next step you were about to take

Slice complete. Next concrete step is the **Estimator's** turn (Codex) — `./scripts/verify.sh` should exit 0 with **44/44** self-test PASS lines. Expected verdict: NOTE or WARNING. Possible items:

- (i) `[Pool] Tried to allocate more than the capacity` warning lines during 500k bench — informational, not a slice break, but the Estimator may flag for follow-up.
- (ii) The "FullGPU = bottomUpBoxesPartialGPU(30)" approximation is documented in both D-031 rationale and `README.ko.md`'s caveats, but is a measurement-honesty deviation from a strict D-029 column. Estimator may NOTE it.
- (iii) D-031's rationale paragraph includes a measured-result-driven recommendation ("raise production default toward FullGPU") that arguably belongs in a follow-up slice plan, not in DECISIONS.md. Informational; could be NOTE-level.
- (iv) Smoke test is mechanism-only and Bug-probe (b) shows it can't catch method-identity bugs — documented honestly; standing acceptable structural limitation per PLAN's course corrections.

After this lands, planner-tracked candidates per `PROJECT_STATE.md`:

- **Tune-default-bottomUpHybridDepth slice** — direct follow-up; pick the production default from the chart this slice produced. ~1-line change.
- **Source-file split slice** — user-deferred per slice brief.
- **Strict-D-029-column bench slice (follow-up to D-031)** — only if measurement-vs-noise becomes a question.
- **FBO-based render harness slice** — for BDD-005's render-side clause.
- **Inspector ergonomics for rotation** — Euler / axis-angle input.
- **BDD-018 inspector live-edit propagation** — needs ImGui-side simulation.
- **Behavior assignment UI (FR-006 / BDD-006)** — Q2 open.
- **Rigid body (FR-008)** — Q4 blocked.
- **Alembic export (FR-013)** — Q5+Q6 blocked.
- **Role-doc maintenance pass.**

See `.agent/PLAN.md` and `.agent/CURRENT_WORK.md` for full plan and progress.
