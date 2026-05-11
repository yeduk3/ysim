# Resume — Hybrid BVH bottom-up combine (D-030; runtime depth knob; build + refit)

## Must remember

- **Branch:** `feat/bvh-bottomup-hybrid` (off `main` at `0ff03a4`).
- **D-030 is parallel-symbol, not modify-in-place**: D-029's `bottomUpBoxes` kernel + `bottomUpBoxesGPU` method stay UNCHANGED. A new `bottomUpBoxesPartial` kernel + `bottomUpBoxesPartialGPU` method live alongside them with the `maxDepth` cutoff. The hybrid driver `bottomUpHybrid` picks between them. This shape is mandated by the user's make-means-add-new rule (see `.claude/skills/slice/SKILL.md` "Interpreting the user's slice request" and the `feedback_make_means_add_new` memory entry) — future modifications must NOT collapse the parallel pair back into one symbol.
- **Three modes via `bottomUpHybrid`:**
  - `maxDepth == 0` → pure CPU `bottomUpCombine()`.
  - `1 <= maxDepth < log2(N)` → hybrid: GPU first `maxDepth` levels via `bottomUpBoxesPartialGPU`, CPU finishes via `bottomUpCombineWithSkip()`.
  - `maxDepth >= log2(N)` → still routes through `bottomUpBoxesPartialGPU` (kernel walks to root because the depth check never fires); functionally equivalent to D-029's `bottomUpBoxesGPU`. The original `bottomUpBoxesGPU` is still callable directly for callers that want pure-GPU walk-to-root without the redundant depth counter.
  Default `bottomUpHybridDepth = 2` (starting point, not measurement-validated).
- **`treeVisitCounts == 2` is the GPU-completed-frontier marker.** Load-bearing invariant: depends on the `bottomUpBoxesPartial` kernel's CHECK-BEFORE-ATOMIC placement. If a future edit moves the depth check after the atomic, the marker becomes ambiguous (could be 1 from a depth-cutoff thread) and `bottomUpCombineWithSkip` produces wrong results. Don't reorder without thinking.
- **`bottomUpHybrid` is the canonical entry point** for BVH bottom-up combine; callers (`build` and `refit`) MUST go through it rather than calling `bottomUpBoxesGPU` or `bottomUpBoxesPartialGPU` directly. `bottomUpHybrid` handles N=1, pure-CPU, partial-GPU+CPU, and pure-GPU modes uniformly AND commits the encoder on all paths.
- **`MetalGlobalContext::commitAndWait` is now null-encoder-safe.** One-line guard at the top (`if (!computeCommandEncoder) return;`). This was load-bearing for `bottomUpHybrid`'s N<=1 / maxDepth=0 branches that need to flush prior dispatches but may have no encoder of their own.
- **Bug-probe (a) — off-by-one in depth check — passes silently.** Because the CPU completion's frontier-skip correctly handles "GPU did more work than expected", a wrong `>` vs `>=` makes GPU combine 1 extra level but the final tree result is identical. The D-sweep doesn't discriminate. Off-by-one is a perf-only bug, not a correctness bug, and the slice doesn't claim perf bounds. Documented in D-030's "alternatives considered" + CURRENT_WORK.
- **Bug-probe (b) — skip-logic inversion — caught loudly.** `!= 2u` instead of `== 2u` makes CPU return at root, leaving tree[0] uninitialized. FR-003 + FR-004 + Block 21 (D-029) all FAIL together. The frontier-skip semantic IS load-bearing for correctness.

## Last decisions + why

- **D-030 — runtime depth knob + check-before-atomic + treeVisitCounts==2 frontier.**
  - Runtime over compile-time: experiment slice needs runtime A/B without rebuild.
  - Check-before-atomic over check-after: keeps `treeVisitCounts` unambiguous at the frontier (always 0 or 2, never 1).
  - `treeVisitCounts == 2` over per-node flag: no new state; reuses existing scratch buffer.
  - Default D=2: starting point; future slice can measure and tune (or add auto-heuristic).
- **`commitAndWait` null-encoder guard** added at the MetalGlobalContext layer. Caller-side guards would be more verbose; making the API forgiving is the cleaner contract change.
- **bottomUpHybrid commits on all paths** (not caller-side). N<=1 + maxDepth=0 branches still need to flush prior dispatches (buildLeafGPU, buildTreeGPU, etc.) so CPU reads of tree.ptr are up-to-date.

## Next step you were about to take

Slice complete after parallel-symbol restructure. Next concrete step is to **re-run the Estimator** (Codex) on the restructured diff — `./scripts/verify.sh` should exit 0 with **43/43** self-test PASS lines (Block 23 added; previous was 42), and the Estimator should see that `bottomUpBoxes`/`bottomUpBoxesGPU` are back to their D-029 single-arg form with the partial-depth path living as the new `bottomUpBoxesPartial`/`bottomUpBoxesPartialGPU` parallel pair. Expected verdict: NOTE or WARNING. Possible items: (i) probe (a)'s silent pass + the D-sweep's lack of perf discrimination; (ii) default `bottomUpHybridDepth = 2` being unsupported by measurement; (iii) maintenance hazard from the two parallel kernels sharing most of their body (acceptable trade for keeping D-029 callable).

After this lands, planner-tracked candidates per `PROJECT_STATE.md`:

- **GPU port of `enlargeTrajectory`** — same per-substep-loop pattern (now well-trodden via D-029 + D-030).
- **BVH bottom-up perf benchmark slice** — add a `--bench-bottomup` flag, build a large mesh, time build() across D values. Closes the question D-030 leaves open.
- **FBO-based render harness slice** — for BDD-005's render-side clause.
- **Inspector ergonomics for rotation** — Euler / axis-angle input.
- **BDD-018 inspector live-edit propagation** — needs ImGui-side simulation.
- **Behavior assignment UI (FR-006 / BDD-006)** — Q2 open.
- **Rigid body (FR-008)** — Q4 blocked.
- **Alembic export (FR-013)** — Q5+Q6 blocked.
- **Role-doc maintenance pass**.

See `.agent/PLAN.md` and `.agent/CURRENT_WORK.md` for full plan and progress.
