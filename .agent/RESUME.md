# Resume — BVH bottom-up GPU combine (D-029 + fix-turn; FULL scope shipped)

## Must remember

- **Branch:** `feat/bvh-bottomup-gpu` (off `main` at `eb1413e`). Two phases on this branch:
  1. Original Generator turn: build + refit GPU port (+ CM-011 forensic resolved).
  2. Fix-turn (this turn, Estimator turn 23 BLOCK absorbed): N=1 guard in `bottomUpBoxesGPU`.
  Commit prefix `fix:` (BLOCK fix-turns stay on slice branch per GENERATOR.md).
- **D-029 ships build AND refit on GPU**, with the N=1 guard. `bottomUpBoxesGPU` short-circuits when `numPrimitives <= 1`. For N=1, the tree's single node IS the leaf-root (no interior nodes to combine), and `buildTree_*` doesn't write `treeParent[0]` for that case — dispatching the kernel would read uninitialized parent and produce UB. `buildLeafGPU` still runs for N=1 (correctly writes the single leaf at tree[0]).
- **`enlargeTrajectory` stays CPU.** Separate slice candidate; same per-substep-loop GPU port pattern as refit. CM-011 forensic confirmed the OLD "2-substep BVH lag" was an artifact, so future GPU enlargeTrajectory port is unblocked.
- **Bug-probe is Apple-Silicon-masked**, documented in D-029 + CURRENT_WORK. The N=1 UB is real (spec-incorrect, would manifest on weaker memory models / non-zero-init buffers / strict OOB-trap GPUs), just invisible to this harness on Apple Silicon. The production guard is the spec-correct fix regardless.
- **Block 22 is the N=1 path canary.** Single-triangle .obj imported from `/tmp/bdd_d029_n1.obj`, sim.initialize + sim.update, assert tree[0] AABB + queryClickRay hit. Pass label `D-029 fix / N=1 BVH safely bypasses bottom-up combine`.
- **Metal 3.2 seq_cst fences in `bottomUpBoxes`.** MSL atomics are relaxed-only; fences carry ordering. Kernel is spec-correct, not TSO-dependent. Two fence sites bracket the publication boundary.
- **CM-011 reframed as forensic.** OLD CPU refit's 2-substep BVH lag was a deferred-commit artifact, not a contract. NEW GPU refit has the standard 1-substep lag. BDD-010's OLD `lastSubstep`-iteration assertion was satisfied only by the artifact; the simplified `cumulativeNarrowCollisions > 0` form (with `enableSelfCollisions = false` default) honestly satisfies the spec.

## Last decisions + why

- **D-029 fix-turn — N=1 guard at the C++ layer** (`bottomUpBoxesGPU`) over kernel-internal guard or sentinel-init. Smallest surface (3 lines), truer semantic ("N=1 has no interior nodes to combine"), symmetric with the existing N=0 guard.
- **Block 22 stricter than minimum.** Three layered assertions: no-crash + tree[0] AABB + queryClickRay hit. Catches different fail modes (UB-crash, silent garbage write, BVH structural corruption). PLANNER.md step 7 stricter-when-cheap.
- **No new D-NNN; D-029 rationale gets a fix-turn paragraph.** The fix is a guard refinement on the existing decision's pre-condition, not a new architectural call.
- **No CM-012.** The trap pattern (kernel A's early-return leaves parallel array uninitialized; kernel B consumes assuming full coverage) is captured directly in D-029's fix-turn paragraph; not generic enough for its own CM yet.

## Next step you were about to take

Fix-turn complete. Next concrete step is the **Estimator's** turn — `./scripts/verify.sh` should exit 0 with **42/42** self-test PASS lines (Block 22 added; previous was 41). Expected verdict: **NOTE-clean** (BLOCK closed; turn-23 NOTE folded into TEST_MATRIX; bug-probe Apple-Silicon-mask documented).

After this lands, planner-tracked candidates per `PROJECT_STATE.md`:

- **GPU port of `enlargeTrajectory`** — same per-substep-loop pattern, now unblocked since CM-011 was resolved as forensic.
- **FBO-based render harness slice** — for BDD-005's render-side clause.
- **Inspector ergonomics for rotation** — Euler / axis-angle input.
- **BDD-018 inspector live-edit propagation** — needs ImGui-side simulation.
- **Behavior assignment UI (FR-006 / BDD-006)** — Q2 open.
- **Rigid body (FR-008)** — Q4 blocked.
- **Alembic export (FR-013)** — Q5+Q6 blocked.
- **Role-doc maintenance pass**.

See `.agent/PLAN.md` and `.agent/CURRENT_WORK.md` for full plan and progress.
