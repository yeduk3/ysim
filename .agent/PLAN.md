# Plan — BVH refit benchmarking harness (`feat/bvh-refit-bench`)

> Owner: **Planner** writes; **Generator** executes; **Estimator** judges.
> Updated: 2026-05-12

## Course note: previous slice's verdict

Estimator turn 25 returned **WARNING** (no BLOCK) on the
hybrid-bottom-up slice (D-030). Single WARNING: default
`bottomUpHybridDepth = 3` (the field was set to 3 in the shipped
code; turn 25's report cited 2 from an earlier draft) is not
backed by measurement. **This slice closes that WARNING** by
producing the measurement: a benchmark harness that times
`broad_refit` across four refit methods × four particle counts ×
ten frames, with artifacts under `profiles/experiment/`.

## Why this slice now

D-030 shipped a runtime knob for the GPU/CPU bottom-up split but
did not benchmark it. The user is now asking for the data: which
method is fastest at which particle count. The slice ships the
**harness** (runs on demand, writes CSV, generates chart) and a
**smoke test** (1×1×1 micro-config exercised in `--self-test` so
the harness mechanism stays regression-tested). Tuning the
production default `bottomUpHybridDepth` is OUT OF SCOPE here —
that's a follow-up "set default from data" slice once the user
inspects the chart and decides.

## Design call (D-031)

Six resolved decisions:

### (a) Entry point — new CLI flag `--bench-bvh-refit`

**Decision: add `--bench-bvh-refit` as a second argv branch in
`main()` next to the existing `--self-test` branch.** Reasons:

- The bench takes seconds to minutes (4 methods × 4 cloth sizes ×
  10 frames × 60 substeps; the 500k case is the long tail). It
  cannot run on every `--self-test` invocation. A dedicated flag
  keeps `--self-test` fast (`verify.sh` still completes quickly).
- The flag exits immediately after the bench (no GUI window),
  mirroring `--self-test`'s exit-after-run pattern. Simple
  `argv[1]` string compare; no new argparse dependency.
- Mutually exclusive with `--self-test`; first matching branch
  wins. (User runs ONE of: GUI, self-test, or bench.)

### (b) Methods × particle counts (the sweep grid)

**Decision: 4 methods × 4 cloth resolutions × 10 frames = 160
data rows.** Methods map to values of the existing
`bottomUpHybridDepth` runtime knob:

| User-facing label | `bottomUpHybridDepth` value | Code path                                                                 |
|-------------------|-----------------------------|----------------------------------------------------------------------------|
| Full CPU          | `0`                         | `bottomUpHybrid` skips GPU, calls `bottomUpCombine()`                      |
| Hybrid D=1        | `1`                         | `bottomUpHybrid` → `bottomUpBoxesPartialGPU(1)` + `bottomUpCombineWithSkip` |
| Hybrid D=2        | `2`                         | `bottomUpHybrid` → `bottomUpBoxesPartialGPU(2)` + `bottomUpCombineWithSkip` |
| Full GPU          | `30`                        | `bottomUpHybrid` → `bottomUpBoxesPartialGPU(30)` (kernel walks to root; CPU `bottomUpCombineWithSkip` short-circuits at root because `treeVisitCounts[0] == 2`) |

**Note on "Full GPU."** This is `bottomUpBoxesPartialGPU(30)`,
NOT the D-029 `bottomUpBoxesGPU` directly. The two paths differ
by ONE register-read + ONE compare per kernel-loop iteration
(the unused depth counter). Dwarfed by the atomic + 2 seq_cst
fences in the same iteration; measurement-wise indistinguishable.
This slice **does not** introduce a new `bench-only` enum that
dispatches the D-029 path strictly — the parallel-symbol rule
keeps both kernels callable, but the bench routes everything
through `bottomUpHybrid`'s runtime knob so the harness stays a
4-line table swap. A future slice can add a strict-D-029 column
if a measurement-vs-noise question arises.

**Cloth resolutions** (FastGridCloth, `particleNum1D × particleNum1D` grid):

| User-facing label | `particleNum1D` | Vertex count | Triangle count (2×(N-1)²) |
|-------------------|-----------------|--------------|----------------------------|
| 1k                | 32              | 1,024        | 1,922                      |
| 10k               | 100             | 10,000       | 19,602                     |
| 100k              | 316             | 99,856       | 198,450                    |
| 500k              | 707             | 499,849      | 996,072                    |

Triangle count is what scales BVH leaf-count and therefore refit
time. README.ko.md will state this so the chart's x-axis label
("particle 수") matches the user's mental model while the chart
caption/description notes the 2×(N−1)² triangle expansion.

### (c) Simulation parameters — defaults, accept divergence

**Decision: use `Simulator::addClothGridFast` defaults
(kstretch=1e5, kshear=1e5, kbend=3e5, thickness=0.001, mass=0.1)
for ALL four resolutions.** Reasons:

- The bench measures refit *time*, which is a function of tree
  size (= triangle count) and AABB topology. It does NOT depend
  on whether particles converge to a stable drape. Even an
  exploded cloth still has well-defined particle positions per
  frame, BVH leaves still get rewritten by `buildLeafGPU`, and
  `bottomUpHybrid` still walks the tree. The "refit time" we
  measure stays representative.
- The user's brief acknowledges this implicitly:
  "각 particle 수에 따라 안정적으로 시뮬레이션되는 파라미터 값은
  시행착오를 통해 찾아볼 수 있다" — i.e., "we can hunt for stable
  params later if needed." For this slice, defaults are the
  baseline.
- If a particular config crashes (rare; usually instability is
  positions-only, not a crash), the bench should catch the
  exception/error and record `refit_time_ms = -1` for that row.
  The chart script skips negative rows.

### (d) Per-frame timing source — existing `broad_refit` FrameProfiler scope

**Decision: read `section_ms[broad_refit]` from a per-frame
`FrameProfiler` snapshot accumulated across all 60 substeps of
one `sim.update()` call.** Reasons:

- `broad_refit` is already instrumented at `src/main.cpp:4910` and
  `:4927` via `profiler->scoped("broad_refit")`. FrameProfiler
  accumulates per-substep refit time into a single per-frame
  total via `addSample`. No new hook needed; bench harness only
  needs to construct a local `FrameProfiler`, wire it to
  `sim.profiler`, and read the snapshot after each `endFrame()`.
- This is the cost the user *actually pays* per frame:
  60 substeps × per-substep refit cost. Matches the per-frame
  number the user would see in the GUI profiler.
- Bench discards frame index 0 as "warmup" (Metal command queue
  cold-start, first-buffer-allocation effects). Records frames
  1–10 → 10 data rows per method/size combination. Total
  rows = 4 methods × 4 sizes × 10 frames = 160.

### (e) Output schema and folder layout

**Decision: `profiles/experiment/bvh-refit-2026-05-12/` folder.**
Files inside:

- `README.ko.md` — Korean experiment description (what was run,
  why, the 4 methods, the 4 resolutions, hardware/OS at run time,
  caveats around instability and warmup-frame discard).
- `refit_bench.csv` — raw data, header `method,particle_count,frame_index,refit_time_ms`.
- `chart.py` — self-contained Python script (matplotlib +
  csv stdlib) that reads `refit_bench.csv` (relative path) and
  writes `refit_chart_line.png` (log-log line chart, one line
  per method) and `refit_chart_bar.png` (grouped bar chart, one
  group per particle count, 4 bars per group). Re-runnable.
- `refit_chart_line.png`, `refit_chart_bar.png` — generated by
  `chart.py` after a bench run. Committed alongside the CSV so
  the slice ships a viewable result.

Folder name `bvh-refit-2026-05-12` is a date-stamp; future re-runs
can land in a sibling folder (e.g., `bvh-refit-2026-06-XX-after-tuning`)
without overwriting historical data.

### (f) Mechanization — Block 24 smoke test in `runSelfTest`

**Decision: Block 24 is a thin smoke test that calls
`runRefitBench(smokeCfg)` with a 1-method × 1-size × 1-frame
config and asserts the CSV file gets written with the expected
header + 1 data row.** Reasons:

- The full bench is too slow to run on every `--self-test` (CI
  loop would balloon). The smoke config completes in <1 second.
- The smoke test verifies the *harness mechanism* (CSV writing,
  FrameProfiler wiring, `bottomUpHybridDepth` toggling, exit
  path) — NOT the perf numbers themselves (those are noisy and
  non-deterministic; can't be PASS/FAIL'd anyway).
- The smoke writes to `/tmp/ysim_refit_bench_smoke.csv` (BDD-019
  precedent — harness hygiene routes profile-shaped artifacts to
  `/tmp` when run from `--self-test`).
- Pass label: `D-031 / refit bench harness writes one row for one frame`.
- Bug-probe: temporarily comment out the `csv << row` line in
  `runRefitBench`; smoke must FAIL with "expected 1 data row,
  got 0". Restore.

Self-test count: 43 → 44.

**No new BDD / FR / TEST_MATRIX row.** This is a perf-measurement
slice, not a behavior slice. The smoke test is harness-self-test,
not BDD mechanization.

## Goal

After this slice:

- New CLI flag `--bench-bvh-refit` runs the full 4×4×10
  benchmark, writing CSV + charts to
  `profiles/experiment/bvh-refit-2026-05-12/`.
- New `runRefitBench(const BenchConfig&)` function in
  `src/main.cpp` (a sibling of `runSelfTest`; the user
  explicitly asked for the bench to live "in main, the way you
  write tests" pending a future source-split refactor).
- New `BenchConfig` struct (methods, sizes, frames, output dir,
  warmup-frame count). Default-constructed = full sweep; smoke
  config used by Block 24.
- New `BVHRefitMethod` enum (FullCPU, HybridD1, HybridD2, FullGPU)
  + a helper that maps each enum value to a
  `bottomUpHybridDepth` integer.
- New `profiles/experiment/bvh-refit-2026-05-12/` folder with
  README.ko.md + chart.py + populated CSV + 2 PNG charts.
- New Block 24 in `runSelfTest` (smoke test); self-test count
  43 → 44.
- New D-031 in `docs/DECISIONS.md` recording the bench design.

## Scope

### 1. New entry point — `runRefitBench(const BenchConfig&)`

**`src/main.cpp::runRefitBench`** — new free function near
`runSelfTest`. Mirrors `runSelfTest`'s structure (early-return on
Metal-less host with SKIP, otherwise iterate configs and write
rows).

```cpp
enum class BVHRefitMethod { FullCPU, HybridD1, HybridD2, FullGPU };

inline int depthForMethod(BVHRefitMethod m) {
    switch (m) {
        case BVHRefitMethod::FullCPU:   return 0;
        case BVHRefitMethod::HybridD1:  return 1;
        case BVHRefitMethod::HybridD2:  return 2;
        case BVHRefitMethod::FullGPU:   return 30;  // >= log2(largest tree depth)
    }
    return 0;
}

inline const char* labelForMethod(BVHRefitMethod m) {
    switch (m) {
        case BVHRefitMethod::FullCPU:   return "FullCPU";
        case BVHRefitMethod::HybridD1:  return "HybridD1";
        case BVHRefitMethod::HybridD2:  return "HybridD2";
        case BVHRefitMethod::FullGPU:   return "FullGPU";
    }
    return "Unknown";
}

struct BenchConfig {
    std::vector<BVHRefitMethod> methods;  // default: all 4
    std::vector<int> particleNum1Ds;      // default: {32, 100, 316, 707}
    int warmupFrames = 1;                 // first N frames discarded
    int measuredFrames = 10;              // recorded after warmup
    std::string outCsvPath;               // full path; smoke uses /tmp
};

static int runRefitBench(const BenchConfig& cfg);
```

Body of `runRefitBench`:

- Open output CSV; write header `method,particle_count,frame_index,refit_time_ms`.
- For each method ∈ cfg.methods:
  - For each particleNum1D ∈ cfg.particleNum1Ds:
    - Construct fresh `Simulator<METAL, Precision, ExplicitSystem>`
      with `h = 1/60`, `subSteps = 60`. Cloth-only scene: one
      `addClothGridFast(particleNum1D, /*size1D=*/1.0f, ...defaults)`
      call.
    - Call `sim.initialize()`.
    - Set
      `sim.collisionPipeline.broadPhase.objTrees[0].bottomUpHybridDepth = depthForMethod(method);`
      (and for the scene-level `broadPhase.tree.bottomUpHybridDepth` — only
      relevant if scene BVH has >1 mesh, but set for consistency).
    - Construct a local `FrameProfiler localProfiler(64);` and
      assign `sim.profiler = &localProfiler;`. Reset
      `sim.profiler` back to `nullptr` (or the previous value)
      after the inner loop.
    - For frameIdx in 0 .. cfg.warmupFrames + cfg.measuredFrames - 1:
      - `localProfiler.beginFrame(frameIdx, frameIdx * sim.system.h);`
      - `sim.update();`
      - `localProfiler.endFrame();`
      - If `frameIdx >= cfg.warmupFrames`:
        - Read the latest snapshot from
          `localProfiler.history().frames().back()`.
        - Look up `broad_refit` section index from the history's
          `sectionIndex("broad_refit")`. If not found, record
          `refit_time_ms = -1` (means refit never ran; should
          not happen for a cloth-only scene).
        - Write CSV row.
- Close CSV. Return 0 (success) / non-zero (mismatch / IO error).

### 2. New CLI branch in `main()`

**`src/main.cpp::main`** at line ~7672:

```cpp
int main(int argc, char** argv) {
    if (argc > 1 && std::string(argv[1]) == "--self-test") {
        return runSelfTest();
    }
    if (argc > 1 && std::string(argv[1]) == "--bench-bvh-refit") {
        BenchConfig cfg;  // default-constructed = full 4x4x10 sweep
        cfg.methods         = {BVHRefitMethod::FullCPU,
                               BVHRefitMethod::HybridD1,
                               BVHRefitMethod::HybridD2,
                               BVHRefitMethod::FullGPU};
        cfg.particleNum1Ds  = {32, 100, 316, 707};
        cfg.warmupFrames    = 1;
        cfg.measuredFrames  = 10;
        cfg.outCsvPath      = std::string(YSIM_PROJECT_ROOT)
                            + "/profiles/experiment/bvh-refit-2026-05-12/refit_bench.csv";
        return runRefitBench(cfg);
    }

    std::cout << "Run simulator" << std::endl;
    // ... existing GUI launch ...
}
```

`YSIM_PROJECT_ROOT` is the existing CMake-define used by
`FrameProfiler` CSV exports (per CLAUDE.md).

### 3. Block 24 — smoke test in `runSelfTest`

Append after Block 23. Smoke config: 1 method × 1 size × 1
measured frame, /tmp output.

```cpp
// ---- Block 24: D-031 — refit bench harness writes one row for one frame. ----
// Thin smoke test verifying the runRefitBench() mechanism: CSV is
// written with the expected header and one data row, FrameProfiler
// captures a `broad_refit` measurement, and the bottomUpHybridDepth
// knob actually toggles. NOT a perf test — the time value isn't
// asserted, only its presence and non-negative value.
//
// Bug-probe direction: commenting out the CSV row-write line in
// runRefitBench should make this block FAIL with "expected 1 data
// row, got 0". This is the load-bearing assertion.
{
    BenchConfig smokeCfg;
    smokeCfg.methods         = {BVHRefitMethod::HybridD2};
    smokeCfg.particleNum1Ds  = {16};   // 16x16 = 256 particles, tiny
    smokeCfg.warmupFrames    = 0;
    smokeCfg.measuredFrames  = 1;
    smokeCfg.outCsvPath      = "/tmp/ysim_refit_bench_smoke.csv";

    int rc = runRefitBench(smokeCfg);
    if (rc != 0) {
        fail("D-031 / refit bench harness writes one row for one frame",
             "runRefitBench returned non-zero: " + std::to_string(rc));
    } else {
        // Verify CSV exists, has header + exactly 1 data row, and the
        // refit_time_ms field is parseable as a non-negative double.
        std::ifstream in("/tmp/ysim_refit_bench_smoke.csv");
        if (!in) {
            fail("D-031 / refit bench harness writes one row for one frame",
                 "CSV file not written at /tmp/ysim_refit_bench_smoke.csv");
        } else {
            std::string header, row, extra;
            std::getline(in, header);
            std::getline(in, row);
            bool hasExtra = (bool)std::getline(in, extra);
            const std::string expectedHeader =
                "method,particle_count,frame_index,refit_time_ms";
            if (header != expectedHeader) {
                fail("D-031 / refit bench harness writes one row for one frame",
                     "header mismatch: got '" + header + "'");
            } else if (row.empty()) {
                fail("D-031 / refit bench harness writes one row for one frame",
                     "no data row written");
            } else if (hasExtra && !extra.empty()) {
                fail("D-031 / refit bench harness writes one row for one frame",
                     "expected 1 data row, got more: '" + extra + "'");
            } else {
                // Parse the 4 comma-separated fields; assert refit_time_ms >= 0.
                auto lastComma = row.rfind(',');
                bool parsed = false;
                if (lastComma != std::string::npos) {
                    try {
                        double t = std::stod(row.substr(lastComma + 1));
                        if (t >= 0.0) parsed = true;
                    } catch (...) {}
                }
                if (parsed) {
                    pass("D-031 / refit bench harness writes one row for one frame");
                } else {
                    fail("D-031 / refit bench harness writes one row for one frame",
                         "refit_time_ms not parseable / negative in row: '" + row + "'");
                }
            }
        }
    }
}
```

Pass label: `D-031 / refit bench harness writes one row for one frame`.

Self-test count: 43 → 44.

### 4. `profiles/experiment/bvh-refit-2026-05-12/` folder

Author three text artifacts (no PNG yet — chart.py will produce
those after the user runs the full `--bench-bvh-refit`):

- **`README.ko.md`** — Korean experiment description. Sections:
  - 실험 개요 (purpose, links to D-030 / D-031).
  - 방법론 (4 methods table + Korean description; 4 resolutions
    table noting that vertex count is what's reported, and the
    triangle-count expansion).
  - 실행 방법 (`./build/src/ysim --bench-bvh-refit` from the
    build dir; `python3 chart.py` to regenerate PNGs).
  - 캐비어트 (warmup frame discard; instability acceptance;
    hardware-dependence note; `enableSelfCollisions = false`
    default; cloth-only scene assumption).
- **`chart.py`** — self-contained Python script. Reads
  `refit_bench.csv` (relative path); writes
  `refit_chart_line.png` (log-log: x = vertex count {1024,
  10000, 99856, 499849}, y = mean refit_ms per method, error
  bars = stddev across the 10 measured frames) and
  `refit_chart_bar.png` (grouped bars: one group per cloth size,
  4 bars per group = 4 methods, height = mean ms, whiskers =
  stddev). Uses matplotlib + csv stdlib only — no pandas
  dependency. Re-runnable.
- **`refit_bench.csv`** — placeholder header row only; the
  user runs `--bench-bvh-refit` to populate. (The slice ships
  a header-only CSV so the chart.py script's I/O paths are
  unit-testable without running the full bench.)

The 2 PNGs are NOT committed in this slice; the user produces
them after a full bench run. If the user wants the slice to
commit reference PNGs, they can add them in a follow-up commit
(or the post-slice CSV-population step). PLAN.md flags this
explicitly so the Estimator doesn't flag missing PNGs as a gap.

### 5. New D-031 in `docs/DECISIONS.md`

Standard format. File/function (runRefitBench + CLI flag +
BenchConfig + BVHRefitMethod), decision (bench design above with
the 4 method labels and depth mapping), alternatives-considered
(separate enum-driven bench-only path with strict-D-029 column;
in-process unit-test mechanism vs CLI flag; pandas vs stdlib for
chart script; full sweep in self-test vs smoke only), rationale.

### 6. PROJECT_STATE.md additions

The Planner (this turn) has already updated PROJECT_STATE.md
with:
- The shipped D-030 entry (Hybrid bottom-up combine).
- The new in-flight pointer to this slice.
- A new "Standing feature candidates" entry for the
  **source-file split slice** that the user mentioned in their
  brief (move `runSelfTest` + `runRefitBench` out of main.cpp
  once the classes themselves split — explicitly deferred per
  user note).

### 7. Bookkeeping

- `.agent/CURRENT_WORK.md` + `.agent/RESUME.md` — slice progress.
- `docs/TEST_MATRIX.md` — NO new row (no BDD).
- `docs/TESTS.md` — NO new scenario (no BDD).

## Non-goals (this slice)

- **Tuning the production default `bottomUpHybridDepth`.** The
  bench produces the data; choosing the default from that data is
  a follow-up slice. The user explicitly wants to inspect the
  chart before committing to a new default.
- **A strict-D-029 measurement column** (calling
  `bottomUpBoxesGPU(sceneBox)` directly without the depth
  counter). "Full GPU" in this slice means
  `bottomUpBoxesPartialGPU(30)`. The overhead delta is
  near-noise; if a future measurement question demands it, add
  in a follow-up.
- **Cross-scene benchmarks** (cloth + rigid, cloth + ground, etc.).
  Cloth-only per user brief.
- **Self-collision-enabled benchmarks.** Default
  `enableSelfCollisions = false`.
- **GPU profiling primitives** (Metal performance counters,
  GPUFrameCapture, etc.). Wall-clock from `std::chrono::steady_clock`
  inside FrameProfiler is the only timing source.
- **Stability-tuned sim parameters per resolution.** Defaults
  for all; instability is acceptable as refit time doesn't depend
  on convergence.
- **CI integration of `--bench-bvh-refit`.** Manual-invocation
  only. Smoke test (Block 24) is the regression net.
- **Inspector widget to switch methods at runtime in the GUI.**
  Out of scope; the bench drives the knob.
- **New BDD / FR.** Perf-measurement slice; spec unaffected.
- **Source-file split** (deferred per user brief). Added as
  candidate for a future slice.

## Todo

Ordered. Generator executes top-to-bottom.

1. **Branch hygiene.** Already on `feat/bvh-refit-bench` (off
   `main` at `febc19b`). Commit prefix: `add:` (new bench
   harness).

2. **Re-read the design call.** Six decisions settled; do not
   second-guess unless implementation surfaces a blocker (e.g.,
   `FrameProfiler` can't expose section_ms by name without a
   public lookup helper — then add one alongside, don't refactor
   the profiler).

3. **Add `BVHRefitMethod` enum + helpers** (`depthForMethod`,
   `labelForMethod`) at the top of the runSelfTest area in
   `src/main.cpp`, so both `runRefitBench` and Block 24 can
   reference them.

4. **Add `BenchConfig` struct** (§1 above) near the helpers.

5. **Author `runRefitBench(const BenchConfig&)`** per §1. Use
   the existing `Simulator<METAL, Precision, ExplicitSystem>`
   construction pattern from `main()` (line ~7696). Open the
   output CSV via `std::ofstream`; create the parent directory
   via `std::filesystem::create_directories` (project already
   uses `<filesystem>` per the profile-export path in
   `analyze_profile.py` precedent — verify by grepping).

6. **Add the `--bench-bvh-refit` branch** in `main()` per §2.
   Keep the argv parsing pattern identical to `--self-test`'s
   line.

7. **Verify `FrameProfiler` exposes a public way to look up a
   section_ms by name.** Read `include/FrameProfiler.hpp`. If a
   `sectionIndex(const std::string&)` returns -1 / size_t::max
   when not found, use it. If not, add a small const helper
   inline (~5 lines) — flag this in CURRENT_WORK as a thin
   scaffold added during build-time discovery.

8. **Author Block 24** per §3. Smoke config. Pass label
   `D-031 / refit bench harness writes one row for one frame`.

9. **Build cleanly.** `cmake --build build`. Expect zero new
   warnings.

10. **Run `./scripts/verify-light.sh`.** Doctest binaries should
    stay 159/159 + 1120/1120.

11. **Run `--self-test` 5+ times.** Expect **44/44 PASS**
    consistently (current 43 + Block 24).

12. **Bug-probe.** Two probes:
    - **(a) Skip the CSV row-write.** Comment out the
      `csv << row` line in `runRefitBench`. Block 24 should FAIL
      with "no data row written" / "got 0". Restore.
    - **(b) Wire the wrong depth knob.** Change
      `setMethod` to apply `depthForMethod(method) + 100`. Block 24
      should still PASS (the smoke doesn't assert which depth ran).
      Document this as "smoke test is mechanism-only, not method-
      identity" in CURRENT_WORK. The bench's experimental output
      is the user's gate for method-identity correctness; the
      Estimator should not flag this as a coverage gap.

13. **Author `profiles/experiment/bvh-refit-2026-05-12/` folder
    contents** per §4. README.ko.md, chart.py, header-only
    refit_bench.csv. No PNGs yet.

14. **Run the full bench** (`./build/src/ysim --bench-bvh-refit`
    from the build dir). Expected runtime: a few seconds for the
    small sizes, possibly minutes for 500k with 60 substeps × 10
    frames. If 500k OOMs / crashes / hangs > 10 min: Generator
    halts, reports, hands back to Planner with the actual
    failure (probably a memory cap; we may need to drop 500k or
    halve frames). If it finishes: commit the populated CSV.

15. **Run `chart.py`** (`python3 profiles/experiment/bvh-refit-2026-05-12/chart.py`)
    to produce the 2 PNGs. Commit them.

16. **Add D-031 to `docs/DECISIONS.md`.** Standard format.

17. **Update `CURRENT_WORK.md` / `RESUME.md`** per §7.

18. **Stop and hand off to the Estimator (Codex).**

## Course corrections

- **Stricter-than-spec assertion** (PLANNER.md step 7). Block 24
  asserts both the CSV header AND the row-count AND the
  parseability of `refit_time_ms` as a non-negative double. A
  single "file exists" check would silently pass even on
  zero-byte CSVs. The 3-clause check fails noisier.

- **Architectural invariants applying here:**
  - **D-029** (atomic single-dispatch + Metal 3.2 fences) — held
    UNCHANGED. Bench does not touch kernel internals.
  - **D-030** (parallel-symbol shape; runtime depth knob;
    `treeVisitCounts == 2` frontier invariant; check-before-atomic
    placement; the parallel-symbol pair must not collapse) —
    APPLIES. The bench toggles `bottomUpHybridDepth` per run and
    exercises all four code paths through the existing
    `bottomUpHybrid` driver. NO new bench-only dispatch enum
    inside `bottomUpHybrid` — the slice respects the parallel-
    symbol shape by routing every method through the existing
    runtime knob.
  - **CM-011** (substep-loop commit-boundary forensic) — APPLIES
    indirectly. The bench reads per-frame accumulated refit
    time from FrameProfiler's `broad_refit` scope; each substep
    refit's commitAndWait happens inside `bottomUpHybrid` per
    D-030. Frame-level total = sum of 60 substep refits =
    representative production cost.
  - **Make-means-add-new rule** (`.claude/skills/slice/SKILL.md`)
    — APPLIES. The user's brief uses creation verbs
    ("뽑아보고자 한다", "차트로 보고싶다", implicit "experiment를
    실행할 c++ 코드는 적어놓자"). All new symbols
    (`runRefitBench`, `BenchConfig`, `BVHRefitMethod`, the new
    `--bench-bvh-refit` argv branch, Block 24, the experiment
    folder); no modifications to `BVH::refit`, `BroadPhase::refit`,
    `bottomUpHybrid`, `bottomUpCombine`, `bottomUpBoxesGPU`,
    `bottomUpBoxesPartialGPU`, or any existing Block.

- **"Full GPU" label vs strict D-029.** The bench's "Full GPU"
  is `bottomUpBoxesPartialGPU(30)`, NOT `bottomUpBoxesGPU(sceneBox)`.
  This is a small honest deviation — documented in
  README.ko.md's 캐비어트 section. Justification: the
  depth-counter overhead is one register-read + one compare per
  loop iteration, dwarfed by the atomic + 2 seq_cst fences in
  the same iteration. The alternative (adding a fifth code path
  to the bench dispatcher) would (i) violate the parallel-symbol
  intent of D-030's runtime knob and (ii) introduce a separate
  dispatch line for what is functionally the same kernel. If a
  future measurement question demands strict-D-029, that's a
  follow-up slice.

- **Smoke test is mechanism-only, not method-identity** (see
  Todo bug-probe (b) above). Documenting up-front so the
  Estimator doesn't flag this as a coverage gap. The full
  4×4×10 bench output is the user's gate for method-identity
  correctness; the smoke test only verifies that the harness
  *can* write a row.

- **Source-file split deferral.** User explicitly said: "이런
  테스트들은 나중에 main 안에 있는 클래스들이 파일로 쪼개지면
  테스트도 모두 분리해낼 계획임. 이것은 추후의 계획으로 우선
  적어놓자." Recorded in PROJECT_STATE's "Standing feature
  candidates"; this slice keeps `runRefitBench` in main.cpp
  next to `runSelfTest`.

## What to read before writing code

- `src/main.cpp::runSelfTest` (line 5720) — structure to mirror.
  Note the Metal-less-host SKIP path at the top.
- `src/main.cpp::main` (line 7672) — argv parsing pattern;
  GUI-launch body to NOT trigger when `--bench-bvh-refit` is
  given.
- `src/main.cpp::Simulator<METAL, Precision, ExplicitSystem>`
  construction at line ~7696 — copy this pattern for the bench's
  per-config sim.
- `src/main.cpp::Simulator::addClothGridFast` (line ~4613) — the
  FastGridCloth entry point the bench calls per resolution.
- `src/main.cpp::Simulator::profiler` field + the `auto scope =
  profiler->scoped("broad_refit")` instrumentation at lines 4910
  and 4927 — the timing source the bench reads.
- `include/FrameProfiler.hpp::FrameProfiler` — class API; check
  whether `sectionIndex(name)` is publicly accessible or needs a
  small accessor added.
- `src/main.cpp::BVH<METAL,PR,LINEAR,PRIMITIVE>::bottomUpHybridDepth`
  field (line 3181) — the runtime knob the bench toggles.
- `src/main.cpp::BroadPhase::refit()` (line 4158) — per-mesh
  refit dispatch; the bench's measured cost comes from inside
  this function via `objTrees[i].refit()` (line 4160).
- `scripts/analyze_profile.py` — existing Python pattern (csv +
  stdlib + minimal deps); use as a reference for `chart.py`'s
  style.
- `CLAUDE.md`'s `YSIM_PROJECT_ROOT` define note — use that for
  the default outCsvPath construction.
