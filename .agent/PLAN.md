# Plan — Hybrid BVH bottom-up combine (`feat/bvh-bottomup-hybrid`)

> Owner: **Planner** writes; **Generator** executes; **Estimator** judges.
> Updated: 2026-05-12

## Course note: previous slice's verdict

Estimator turn 24 returned **WARNING** (no BLOCK) on the BVH
bottom-up GPU combine fix-turn (D-029). Single WARNING item:
Block 22's negative bug-probe is Apple-Silicon-masked (Metal shared
storage absorbs OOB atomic reads + silently drops OOB writes at
the test's assertion granularity). The production N=1 guard is the
spec-correct fix regardless; documented in D-029's fix-turn
paragraph. Nothing to fold into this slice — the WARNING is a
documented standing observation, not new debt.

## Why this slice now

D-029 (GPU bottom-up on the entire log(N)-depth walk via atomic-
fetch-add halving) is the load-bearing precursor. The user now
wants a **performance experiment**: do the GPU only does the
first few levels (wide, parallelism-rich), let CPU do the small
top of the tree (narrow, serial-friendly). The expectation is
that GPU's atomic-fence overhead per level at the narrow top
levels exceeds CPU's serial cost, so a hybrid split could outrun
pure-GPU at typical mesh sizes. This slice does NOT claim a
speedup number; it ships the hybrid path correctly and exposes
a runtime knob so the user can benchmark by toggling.

## Design call

Five resolved decisions:

### (a) Depth parameterization — runtime knob, default D=2

**Decision: runtime field `int bottomUpHybridDepth = 2;` on
`BVH<METAL,PR,LINEAR,PRIMITIVE>`.** Reasons:

- Runtime > compile-time for an experiment slice. User can toggle
  to measure; can later expose via inspector if desired.
- Default 2 is a reasonable starting point — gives GPU two
  levels' worth of parallelism (one halving per level), leaves
  the top `numNodes/4` interior nodes to CPU. For N=16
  primitives → 15 interior nodes → 4-ish CPU combines; for
  N=10000 → ~2500 CPU combines (still trivial vs the kernel
  launch overhead saved).
- Per-BVH field (not global) so per-mesh BVHs and the
  scene-level BVH (`BVH<METAL,PR,SCENE,OBJECT>`) can be tuned
  independently if measurement reveals different sweet spots.

### (b) GPU kernel adaptation — depth check before atomic, in the existing kernel

**Decision: extend `bottomUpBoxes` in-place with a `constant uint&
maxDepth` uniform at buffer(7).** Each thread tracks a local
`int depth = 0` and checks `if (depth >= maxDepth) return;`
**before** the atomic_fetch_add at each iteration. Reasons:

- Single kernel, no duplication.
- Branch cost: 1 register read + 1 compare per iteration, dwarfed
  by the atomic + 2 fences in the same iteration.
- Pre-atomic check means threads that would have raced into a
  level above the cutoff don't even touch `treeVisitCounts` at
  that level — so `treeVisitCounts[above-cutoff]` stays at 0
  post-dispatch, which is the CPU's frontier signal (see (d)).
- Pure-GPU mode (maxDepth = D-029 behavior): pass
  `maxDepth = UINT32_MAX` (or just a large constant); kernel
  walks all the way to root and exits via `if (child == 0) return`.

The kernel loop after the change:

```cpp
while (true) {
    int parent = treeParent[child];
    if ((uint)depth >= maxDepth) return;   // hybrid cutoff (NEW)

    uint old = atomic_fetch_add_explicit(
        &treeVisitCounts[parent], 1u, memory_order_relaxed);
    if (old == 0u) return;

    atomic_thread_fence(mem_flags::mem_device, memory_order_seq_cst,
                        thread_scope_device);

    int childA = tree[parent].childA;
    int childB = tree[parent].childB;
    /* ... combine ... */
    tree[parent].min = packed_float3(min(minA, minB));
    tree[parent].max = packed_float3(max(maxA, maxB));

    atomic_thread_fence(mem_flags::mem_device, memory_order_seq_cst,
                        thread_scope_device);
    depth++;                               // (NEW)
    child = parent;
    if (child == 0) return;
}
```

### (c) CPU completion strategy — top-down recursion from root with frontier-skip

**Decision: new `bottomUpCombineWithSkip()` helper.** Recursive
walk from `tree[0]` (root); at each interior node, if the GPU
already wrote it (frontier marker — see (d)), skip the subtree.
Otherwise recurse into children, then combine into this node's
AABB. The existing `bottomUpCombine()` (full-tree recursive)
stays as the Block 21 reference; the new variant adds 4 lines
of skip-logic.

```cpp
void bottomUpCombineWithSkip() {
    auto walk = [&](auto&& self, BVHNode& node, int nodeId) -> void {
        if (node.childA < 0) return;                       // leaf
        if (treeVisitCounts[nodeId] == 2u) return;         // GPU done
        self(self, tree[node.childA], node.childA);
        self(self, tree[node.childB], node.childB);
        node.aabb.min = tree[node.childA].aabb.min;
        node.aabb.max = tree[node.childA].aabb.max;
        node.aabb.combine(tree[node.childB].aabb);
    };
    walk(walk, tree[0], 0);
}
```

### (d) Frontier identification — `treeVisitCounts[node] == 2`

**Decision: reuse `treeVisitCounts` post-dispatch as the frontier
signal.** Nodes the GPU fully combined have `treeVisitCounts == 2`
(both children's threads arrived at this node). Nodes above the
cutoff have `treeVisitCounts == 0` (no thread reached). Nodes
that received exactly one arrival (the lone-survivor path from
one side of a subtree where the sibling exited via depth cutoff
... actually impossible: depth check is BEFORE the atomic, so
threads either arrive at a level (and increment visitCounts) or
exit BEFORE touching that level's atomic. Net: visitCounts at
post-cutoff levels is **either 2 (combined) or 0 (untouched)**;
never 1.

So `visitCounts == 2` is an unambiguous "GPU completed this
subtree" marker. No separate frontier list / no per-node flag
needed. Buffer reuse, ~1 extra atomic op per combined node
(amortized into the existing atomic).

CPU walk reads `treeVisitCounts[nodeId]` from shared storage —
after `commitAndWait`, values are CPU-visible. ✓

### (e) Mechanization — sweep D over {0, 1, 2, INF}, assert all produce same tree[]

**Decision: Block 23.** Build a tess=2 cube (same scene as
Block 21). Capture CPU-reference tree via plain
`bottomUpCombine()`. Then for each test D in `{0, 1, 2, 1000}`:

1. Re-dispatch `buildLeafGPU()` + `commitAndWait()` to reset
   leaves (so each iteration starts from the same leaf state).
2. Zero `treeVisitCounts` (via the existing kernel + commit).
3. Call `bottomUpHybrid(D)`.
4. Snapshot tree (raw flat array of min/max).
5. Diff against CPU reference at every interior node.

`D=0` exercises the pure-CPU path (GPU dispatch skipped); `D=1`
and `D=2` exercise the hybrid frontier; `D=1000` exercises the
pure-GPU path (kernel walks to root). All four should produce
bit-equal interior AABBs.

PLANNER.md step 7 stricter form: D-sweep catches off-by-one
frontier-marking errors (e.g., depth check after the atomic
instead of before; depth counter incremented at wrong site).
Single-D would miss those.

Pass label: `D-030 / hybrid bottom-up matches CPU reference for D ∈ {0,1,2,1000}`.

Self-test count: 42 → 43.

## Goal

After this slice:

- `BVH<METAL,PR,LINEAR,PRIMITIVE>` gains
  `int bottomUpHybridDepth = 2;` field +
  `void bottomUpHybrid(int maxDepth)` method +
  `void bottomUpCombineWithSkip()` helper.
- `bottomUpBoxes` kernel accepts `constant uint& maxDepth
  [[buffer(7)]]` and tracks per-thread depth; depth-check-before-
  atomic enables partial-depth runs.
- `bottomUpBoxesGPU` extended to take an `int maxDepth` arg;
  passes via `setBytes` at buffer(7).
- `BVH::build` and `BVH::refit` switch from
  `bottomUpBoxesGPU(sceneBox)` to `bottomUpHybrid(bottomUpHybridDepth)`.
  Default depth (2) is the new production path.
- Block 23 verifies hybrid correctness across D ∈ {0,1,2,1000}.
- New D-030 records the hybrid design + default + frontier scheme.
- Self-test count 42 → 43.

## Scope

### 1. GPU kernel extension — `bottomUpBoxes`

**`src/metal/bvh.metal::bottomUpBoxes`** (~line 442):

- Add parameter `constant uint& maxDepth [[buffer(7)]]`.
- Add local `int depth = 0;` after `int child = ...;`.
- Add `if ((uint)depth >= maxDepth) return;` **before** the
  `atomic_fetch_add_explicit` at each iteration.
- Add `depth++;` after the parent-AABB release fence, before
  `child = parent;`.

Update the kernel doc-block to mention the depth cutoff +
hybrid mode semantics. Keep all the existing Metal 3.2 seq_cst
fence content unchanged.

### 2. C++ driver extensions

**`src/main.cpp::BVH<METAL,PR,LINEAR,PRIMITIVE>`**:

- **Add field** `int bottomUpHybridDepth = 2;` (default depth
  for the hybrid path). Document inline that 0 = pure CPU,
  large value (e.g., 1000) = pure GPU = D-029 behavior.

- **Extend `bottomUpBoxesGPU` signature** from
  `bottomUpBoxesGPU(const AABB4&)` to
  `bottomUpBoxesGPU(const AABB4&, uint maxDepth)`. Add a
  `setBytes(maxDepth, 7)` call to bind the new uniform. Keep
  the existing zeroVisitCounts + bottomUpBoxes dispatch
  structure unchanged.

- **Add `bottomUpHybrid(int maxDepth)` method.** Orchestrates:
  - Early-return for `numPrimitives <= 1` (D-029 fix-turn
    invariant).
  - For `maxDepth == 0`: skip GPU dispatch, call
    `bottomUpCombine()` directly. (Pure-CPU path; no zero of
    visitCounts needed because `bottomUpCombine` doesn't read
    them.)
  - For `maxDepth > 0`: build the sceneBox, call
    `bottomUpBoxesGPU(sceneBox, (uint)maxDepth)`, commit
    + wait, then `bottomUpCombineWithSkip()`.

- **Add `bottomUpCombineWithSkip()` helper.** Recursive walk
  from root with `visitCounts == 2` skip-check (see §(c)
  above).

### 3. Call-site swap

**`src/main.cpp::BVH::build(int oid, ...)`** (~line 3475):

Replace `bottomUpBoxesGPU(sceneBox);` with
`bottomUpHybrid(bottomUpHybridDepth);` (deleting the inline
sceneBox construction since the hybrid method builds its own).
Or — simpler — keep the sceneBox construction outside and call
`bottomUpBoxesGPU(sceneBox, (uint)bottomUpHybridDepth)` + CPU
follow-up here directly. Generator picks the cleanest factoring.

**`src/main.cpp::BVH::refit()`** (~line 3727):

Same swap. The CPU follow-up happens **after** the GPU's
commitAndWait, on the same encoder boundary that already exists.
CM-011 is unaffected — the commitAndWait is still inside refit,
and the substep-loop staleness was forensic-resolved (the OLD
2-substep lag was an artifact, NOT a contract).

### 4. Block 23 — D-sweep correctness test

Append after Block 22.

```cpp
// ---- Block 23: D-030 — hybrid bottom-up matches CPU reference for D ∈ {0,1,2,1000}. ----
// Builds a tess=2 cube (same scene as Block 21). For each test D,
// resets leaves via buildLeafGPU and runs bottomUpHybrid(D); the
// resulting tree[] must be bit-equal at every interior node to the
// CPU bottomUpCombine() reference. D=0 → pure CPU. D=1, D=2 → hybrid
// frontier at depth 1 / depth 2. D=1000 → pure GPU (kernel walks to
// root, equivalent to D-029).
//
// Stricter than single-D — the sweep catches off-by-one frontier
// marking (depth check at wrong loop position; depth counter
// incremented at wrong site; treeVisitCounts not zeroed before
// dispatch).
{
    resetScene();
    sim.addCube(tinym::vec3(0.0f, 0.0f, 0.0f), /*tess=*/2,
                /*size=*/0.5f, /*mass=*/0.1f);
    sim.initialize();

    auto* mesh23 = Scene<Backend, Precision>::findById(0);
    auto& bp23 = sim.collisionPipeline.broadPhase;
    if (!mesh23 || bp23.objTrees.empty()) {
        fail("D-030 / hybrid bottom-up matches CPU reference for D ∈ {0,1,2,1000}",
             "mesh or objTrees[0] missing");
    } else {
        auto& objTree23 = bp23.objTrees[0];
        Index numPrimitives23 = mesh23->adjacency.facets.size / 3;
        Index numNodes23 = (numPrimitives23 > 0) ? (2*numPrimitives23 - 1) : 0;

        if (numNodes23 < 2) {
            fail("D-030 / hybrid bottom-up matches CPU reference for D ∈ {0,1,2,1000}",
                 "tess=2 cube produced numNodes=" + std::to_string(numNodes23));
        } else {
            // Capture CPU reference: reset leaves, then bottomUpCombine().
            objTree23.buildLeafGPU();
            MetalGlobalContext::commitAndWait();
            objTree23.bottomUpCombine();

            std::vector<float> ref(numNodes23 * 6);
            for (Index i = 0; i < numNodes23; ++i) {
                const auto& n = objTree23.tree[i];
                ref[i*6+0]=(float)n.min.x; ref[i*6+1]=(float)n.min.y; ref[i*6+2]=(float)n.min.z;
                ref[i*6+3]=(float)n.max.x; ref[i*6+4]=(float)n.max.y; ref[i*6+5]=(float)n.max.z;
            }

            // Sweep D.
            const int Ds[] = {0, 1, 2, 1000};
            bool allMatch = true;
            int failingD = -1;
            Index failingNode = -1;
            for (int d : Ds) {
                objTree23.buildLeafGPU();
                MetalGlobalContext::commitAndWait();
                objTree23.bottomUpHybrid(d);

                for (Index i = 0; i < numPrimitives23 - 1; ++i) {
                    const auto& n = objTree23.tree[i];
                    if ((float)n.min.x != ref[i*6+0] ||
                        (float)n.min.y != ref[i*6+1] ||
                        (float)n.min.z != ref[i*6+2] ||
                        (float)n.max.x != ref[i*6+3] ||
                        (float)n.max.y != ref[i*6+4] ||
                        (float)n.max.z != ref[i*6+5]) {
                        allMatch = false;
                        failingD = d;
                        failingNode = i;
                        break;
                    }
                }
                if (!allMatch) break;
            }

            if (allMatch) {
                pass("D-030 / hybrid bottom-up matches CPU reference for D ∈ {0,1,2,1000}");
            } else {
                const auto& n = objTree23.tree[failingNode];
                fail("D-030 / hybrid bottom-up matches CPU reference for D ∈ {0,1,2,1000}",
                     "D=" + std::to_string(failingD) + " node " +
                     std::to_string(failingNode) + " mismatch: hybrid min=(" +
                     std::to_string((float)n.min.x) + "," +
                     std::to_string((float)n.min.y) + "," +
                     std::to_string((float)n.min.z) + ") max=(" +
                     std::to_string((float)n.max.x) + "," +
                     std::to_string((float)n.max.y) + "," +
                     std::to_string((float)n.max.z) + ") vs ref min=(" +
                     std::to_string(ref[failingNode*6+0]) + "," +
                     std::to_string(ref[failingNode*6+1]) + "," +
                     std::to_string(ref[failingNode*6+2]) + ") max=(" +
                     std::to_string(ref[failingNode*6+3]) + "," +
                     std::to_string(ref[failingNode*6+4]) + "," +
                     std::to_string(ref[failingNode*6+5]) + ")");
            }
        }
    }
}
```

Pass label: `D-030 / hybrid bottom-up matches CPU reference for D ∈ {0,1,2,1000}`.

### 5. New D-030 in `docs/DECISIONS.md`

Standard format. file/function, decision (hybrid GPU+CPU bottom-
up with depth cutoff + treeVisitCounts==2 frontier marker +
default depth 2), alternatives-considered (compile-time D vs
runtime; separate fixed-depth kernel vs extending bottomUpBoxes;
per-node flag vs treeVisitCounts), rationale.

### 6. Bookkeeping

- `.agent/PROJECT_STATE.md` — "In flight" pointer → this slice.
  Add shipped entry for the D-029 fix-turn slice (commits
  `3feb747` + `0ff03a4`).
- `.agent/CURRENT_WORK.md` + `.agent/RESUME.md` — slice progress;
  note the runtime knob (`bottomUpHybridDepth`) so future readers
  can find it.

## Non-goals (this slice)

- **Performance benchmarking.** No `--bench-bottomup` flag, no
  wall-clock numbers in the slice. The knob is runtime-tunable;
  user / future slice measures.
- **Inspector widget** for the depth knob. The field is C++-
  settable; UI exposure is a separate small slice.
- **Hybrid `enlargeTrajectory`.** Out of scope (still CPU per
  D-029).
- **Auto-tuning D** based on numPrimitives. Heuristic could be
  added later; this slice just exposes the manual knob.
- **Per-mesh depth tuning** in the inspector. Per-BVH field
  exists but defaults to 2 globally; no UI for per-mesh override.
- **New BDD / FR.** This is an internal perf path slice; spec
  unaffected.

## Todo

Ordered. Generator executes top-to-bottom.

1. **Branch hygiene.** Already on `feat/bvh-bottomup-hybrid` (off
   `main` at `0ff03a4`). Commit prefix: `add:` (new feature
   path).

2. **Re-read the design call.** Five decisions settled; do not
   second-guess unless implementation surfaces a blocker (e.g.,
   `setBytes(uint, 7)` doesn't bind correctly — then stop and
   ask Planner).

3. **Extend `bottomUpBoxes` kernel** per §1. Add `maxDepth`
   uniform at buffer(7), local `depth`, check-before-atomic,
   `depth++` after parent write. Update kernel doc-block.

4. **Extend `bottomUpBoxesGPU` signature** per §2 — add
   `uint maxDepth` arg, `setBytes` at buffer(7) before
   `dispatchThreads(bottomUpBoxesPSO, ...)`.

5. **Add `bottomUpCombineWithSkip()` helper** per §2.

6. **Add `bottomUpHybrid(int maxDepth)` method** per §2.

7. **Add `int bottomUpHybridDepth = 2;` field** per §2.

8. **Update `BVH::build` and `BVH::refit` call sites** per §3
   to use `bottomUpHybrid(bottomUpHybridDepth)`. The N=1 guard
   (D-029 fix-turn) is inherited via the early-return in
   `bottomUpHybrid`.

9. **Author Block 23** per §4. Pass label
   `D-030 / hybrid bottom-up matches CPU reference for D ∈ {0,1,2,1000}`.

10. **Build cleanly.** `cmake --build build`. Expect zero new
    warnings. Watch for Metal kernel signature mismatch errors
    if buffer(7) is already used by some other kernel — grep
    `[[buffer(7)]]` across `src/metal/` if the dispatch errors.

11. **Run `./scripts/verify-light.sh`.** Doctest binaries should
    stay 159/159 + 1120/1120.

12. **Run `--self-test` 5+ times.** Expect **43/43 PASS**
    consistently (current 42 + Block 23).

13. **Bug-probe.** Three small probes:
    - **(a) Off-by-one in depth check.** Temporarily change
      `if ((uint)depth >= maxDepth) return;` to
      `if ((uint)depth > maxDepth) return;` (one off). Block 23
      should FAIL for at least one D value with mismatched
      interior nodes (D=1 case combines an extra level). Restore.
    - **(b) Skip-logic inversion.** Temporarily change
      `if (treeVisitCounts[nodeId] == 2u) return;` to
      `if (treeVisitCounts[nodeId] != 2u) return;` (semantically
      backwards). Block 23 should FAIL for D=1 and D=2 (CPU
      skips the wrong subtrees). Restore.
    - **(c) Forget the depth++ increment.** Remove `depth++;`
      from the kernel. Threads loop forever at depth 0;
      eventually all combines happen and visitCounts hits 2 at
      every level. Block 23 should PASS in this degenerate case
      (because the final tree[] still matches CPU reference) —
      but the slice's performance intent is broken. If (c) silently
      passes, document it as "test catches correctness but not
      perf intent" in CURRENT_WORK; (a) + (b) are the load-
      bearing probes for correctness.

14. **Add D-030 to `docs/DECISIONS.md`.** Standard format.

15. **Update `CURRENT_WORK.md` / `RESUME.md`** per §6.

16. **Stop and hand off to the Estimator (Codex)**.

## Course corrections

- **Stricter-than-spec assertion** (PLANNER.md step 7). Block 23
  sweeps D over 4 values instead of just 1. Catches off-by-one
  frontier-marking bugs that a single-D test would miss.

- **Architectural invariants applying here:**
  - **D-024** (BVH leaf-return semantics) — unaffected; this
    slice modifies BVH build, not query.
  - **D-026** (lifetimeId Float-mesh skip) — unaffected; the
    hybrid path runs inside the same `build()` / `refit()` call
    chains that the skip gate already governs.
  - **D-029** (single-dispatch atomic bottom-up + Metal 3.2
    fences + N=1 guard) — **extended.** This slice ships D-030
    as a generalization that subsumes D-029 (D=large is D-029
    behavior). D-029's invariants are preserved: relaxed
    atomics + seq_cst fences, treeParent reuse, treeVisitCounts
    as scratch.
  - **D-029 fix-turn (N=1 guard)** — applies. `bottomUpHybrid`
    early-returns for `numPrimitives <= 1`, same as
    `bottomUpBoxesGPU` did.
  - **CM-011** (substep-loop commit-boundary forensic) — applies
    but unchanged. The hybrid path adds CPU work AFTER the GPU
    commit, but inside the same `bottomUpHybrid` call (which
    is itself inside `refit` and called inside the substep
    loop). The relative commit-boundary timing matches D-029's
    refit. No new staleness regime.

- **`bottomUpHybridDepth` is a TUNING KNOB**, not a contract.
  The default value (2) is a starting point; user is expected
  to measure and adjust. Future slice may add a heuristic
  (e.g., `D = std::max(1, log2(numPrimitives) - 4)` to keep
  CPU work bounded) — but that's measurement-driven, not
  speculation.

- **Pure-CPU mode preserves the OLD slow path.** Setting
  `bottomUpHybridDepth = 0` routes through `bottomUpCombine()`
  (the existing CPU reference). This is a useful debugging
  fallback if a future kernel change breaks GPU-side
  correctness — set depth=0, ship still works, then investigate.

## What to read before writing code

- `src/metal/bvh.metal::bottomUpBoxes` (~line 442) — current
  kernel structure including the Metal 3.2 fences.
- `src/main.cpp::BVH<METAL,PR,LINEAR,PRIMITIVE>` (~line 3115
  onwards for the struct) — where `bottomUpHybridDepth` field
  goes.
- `src/main.cpp::BVH::bottomUpBoxesGPU` (~line 3264) — current
  driver to extend.
- `src/main.cpp::BVH::bottomUpCombine` (~line 3707) — reference
  CPU implementation to mirror in `bottomUpCombineWithSkip`.
- `src/main.cpp::BVH::build(int oid, ...)` (~line 3475 area) —
  call-site swap.
- `src/main.cpp::BVH::refit()` (~line 3727) — same.
- `src/main.cpp::runSelfTest` Block 21 — template for the
  reference-capture + per-interior-node diff loop in Block 23.
- `docs/DECISIONS.md::D-029` — the existing decision this slice
  generalizes. D-030 cites it.
- `MetalGlobalContext::setBytes` — for the `maxDepth` uniform
  bind at buffer(7). The existing kernel signature uses
  buffer(2) for sceneBox, buffer(4) for tree, buffer(5) for
  treeParent, buffer(6) for treeVisitCounts; buffer(7) is the
  next free slot.
