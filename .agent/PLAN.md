# Plan — BVH bottom-up GPU combine, fix-turn (`feat/bvh-bottomup-gpu`)

> Owner: **Planner** writes; **Generator** executes; **Estimator** judges.
> Updated: 2026-05-12

## Course note: Estimator BLOCKed turn 23

Estimator turn 23 returned **BLOCK** on D-029. Verdict-quote:

> One-primitive BVHs are not guarded: `bottomUpBoxesGPU` runs the new
> atomic kernel for `numPrimitives == 1`, but `buildTree_*` returns
> at `idx == numPrimitives - 1` without writing `treeParent[0]`. The
> kernel then reads uninitialized `treeParent[0]` and atomic-fetches
> at `treeVisitCounts[garbage]` — undefined behavior. Block 21 only
> exercises a tess=2 cube, so this supported edge case has no
> coverage.

Verified by re-reading `src/metal/bvh.metal:284-325` (`buildTree_Tri`):
the `if(idx == numPrimitives-1) return;` short-circuit at line 310 is
the only thread that runs for N=1, and the early-return is BEFORE the
`treeParent[childA]=id; treeParent[childB]=id;` writes at lines
323-324. Same shape in `buildTree_Edge` (line 377). For N=1 the tree
has 1 node (= 2*1-1) which is both leaf AND root; there are no
interior nodes to combine; the bottom-up walk is a no-op-needed but
the kernel still runs and reads garbage.

Per `GENERATOR.md`'s BLOCK-fix-turn convention: stay on the same
slice branch (`feat/bvh-bottomup-gpu`), commit prefix `fix:`.

Folded-in Estimator turn-23 NOTE (small, ~1 line):
- `docs/TEST_MATRIX.md:24` BDD-010 row prose still describes the
  old `objPair.query != objPair.target` assertion shape. Update
  to reflect the simplified `cumNarrow > 0` form. Status `pass`
  stays.

## Why this fix-turn (not a new slice)

BLOCK fix-turns are part of the same slice's commit history (per
GENERATOR.md). The fix is small (~3-line guard + ~50-line test
block + ~1-line prose update), so it stays on `feat/bvh-bottomup-gpu`
with `fix:` prefix; merge to main happens at slice close-out once
Codex returns NOTE/WARNING.

## Design call

Three guard placement options:

- **(a) Guard in `bottomUpBoxesGPU` C++ method.** Add
  `if (numPrimitives <= 1) return;` at the top. The kernel is never
  dispatched for N=1. Build path: `buildLeafGPU + buildTreeGPU`
  populate the single leaf node (= root), bottomUp is a no-op
  (correctly so for N=1, since there are no interior nodes to
  combine). Refit path: `buildLeafGPU` updates the single leaf,
  bottomUp again no-op. **~3 lines.**

- **(b) Guard inside the `bottomUpBoxes` kernel.** Add
  `if (numPrimitives <= 1) return;` at kernel entry. Same
  effect, but the dispatch still happens (1-thread wasted launch).

- **(c) Defensively initialize `treeParent[0] = -1` (sentinel).**
  Doesn't fix the bug — the kernel's loop body still tries to read
  `treeVisitCounts[-1]` (out of bounds). Even if -1 wraps to a
  valid index, the bottom-up logic for "leaf is root" isn't
  representable in the kernel's structure.

**Decision: (a).** Reasons:

1. **Smallest surface.** 3 lines in C++; no kernel changes; no
   structural shifts.
2. **Truer semantic.** For N=1, the tree IS just a leaf; there is
   no interior node to combine. Calling bottomUp is meaningless
   conceptually. Early-returning at the C++ layer says this
   explicitly.
3. **Symmetric with existing `numPrimitives == 0` guard.** The
   method already has `if (numPrimitives == 0) return;`; bumping
   to `<= 1` is a single-character widening of the same idea.

No new D-NNN — this is a guard refinement on D-029's existing
contract, not a new architectural decision. The existing D-029
entry's "Pre-condition: ..." block can absorb the clarification
in the Generator's diff (treeParent must be valid for all interior
nodes 0..numPrimitives-2; for numPrimitives==1 there are no
interior nodes and the kernel must not run).

## Goal

After this fix-turn:

- `Simulator::collisionPipeline.broadPhase.objTrees[i].bottomUpBoxesGPU`
  early-returns for `numPrimitives <= 1`. Build and refit both
  safely handle N=1 meshes.
- `runSelfTest` Block 22 exercises a single-triangle mesh through
  the full GPU build + refit pipeline. Asserts: no crash; tree[0]
  reflects the triangle's AABB; broad-phase queryClickRay finds
  the triangle.
- `docs/TEST_MATRIX.md` BDD-010 row prose updated to the
  `cumulativeNarrowCollisions > 0` shape (Estimator turn-23 NOTE
  fold-in).
- Self-test count grows 41 → 42.
- Estimator turn 24 verdict: NOTE-clean (BLOCK closed, NOTE folded).

## Scope

### 1. Production fix — N=1 guard

**`src/main.cpp::BVH<METAL,PR,LINEAR,PRIMITIVE>::bottomUpBoxesGPU`**
(~line 3265 — find by signature; the existing
`if (numPrimitives == 0) return;` is the guard line to widen):

```cpp
void bottomUpBoxesGPU(const AABB4& sceneBox) {
    Index numPrimitives = primitives.size / PRIMITIVE;
    // D-029 fix-turn: N=1 means the tree's single node IS the leaf-
    // root; there are no interior nodes to combine. buildTree_* also
    // skips treeParent writes for the leaf-root (early-return at
    // `idx == numPrimitives - 1`), so dispatching bottomUpBoxes
    // would read uninitialized treeParent[0]. Skip the dispatch.
    if (numPrimitives <= 1) return;
    // ... existing dispatch code unchanged ...
}
```

That's the entire production change. `buildLeafGPU` continues to
run for N=1 (it correctly writes the single leaf at tree[0]).

### 2. Block 22 — `runSelfTest` mechanization

Append after Block 21 (D-029 / GPU bottom-up combine matches CPU
reference). Goal: build a single-triangle mesh on the fly via the
existing `Simulator::importMesh` path, run sim.initialize +
sim.update, assert tree[0] correctness and ray-pick correctness.

Sketch (Generator may refine):

```cpp
// ---- Block 22: D-029 fix-turn — N=1 BVH (single-triangle import). ----
// Estimator turn-23 BLOCK: bottomUpBoxesGPU ran for N=1 without
// proper treeParent[0] initialization. Fix: bottomUpBoxesGPU early-
// returns for numPrimitives <= 1. This block exercises the N=1
// path end-to-end: write a single-triangle .obj to /tmp, import
// it, run sim.initialize and sim.update (exercises build + refit),
// then ray-pick to confirm the BVH walks correctly with a single
// leaf-root.
{
    // Compose a downward-facing triangle in the XZ plane.
    const std::string objPath = "/tmp/bdd_d029_n1.obj";
    {
        std::ofstream f(objPath);
        f << "v -0.5 0.0 -0.5\n"
          << "v  0.5 0.0 -0.5\n"
          << "v  0.0 0.0  0.5\n"
          << "f 1 2 3\n";
    }

    resetScene();
    std::string err;
    bool ok = sim.importMesh("/tmp", "bdd_d029_n1.obj",
                             /*scale=*/(Precision)1.0,
                             /*mass=*/(Precision)0.1, &err);
    if (!ok) {
        fail("D-029 fix / N=1 BVH safely bypasses bottom-up combine",
             "importMesh returned false: " + err);
    } else {
        sim.initialize();   // exercises BVH::build with N=1
        sim.update();       // exercises BVH::refit with N=1 (no crash)

        auto* mesh = Scene<Backend, Precision>::findById(0);
        auto& bp22 = sim.collisionPipeline.broadPhase;
        if (!mesh || bp22.objTrees.empty()) {
            fail("D-029 fix / N=1 BVH safely bypasses bottom-up combine",
                 "mesh or objTrees[0] missing after import + init");
        } else {
            // Sanity: tree[0] should reflect the triangle's AABB.
            // (Triangle at y=0; min/max y both 0 after BVH leaf
            // computation. x range [-0.5, 0.5]; z range [-0.5, 0.5].)
            auto& objTree = bp22.objTrees[0];
            const float aabbTol = 1e-5f;
            if (std::abs(objTree.tree[0].min.x - (-0.5f)) > aabbTol ||
                std::abs(objTree.tree[0].max.x - ( 0.5f)) > aabbTol ||
                std::abs(objTree.tree[0].min.z - (-0.5f)) > aabbTol ||
                std::abs(objTree.tree[0].max.z - ( 0.5f)) > aabbTol) {
                fail("D-029 fix / N=1 BVH safely bypasses bottom-up combine",
                     "tree[0] AABB doesn't match triangle bounds; "
                     "got min=(" + ... + ") max=(" + ... + ")");
            } else {
                // Stricter: ray-pick should hit the single triangle.
                Ray ray22;
                ray22.origin = tinym::vec3(0.0f, 10.0f, 0.0f);
                ray22.dir    = tinym::vec3(0.0f, -1.0f, 0.0f);
                Scene<Backend, Precision>::rayTracedData
                    .numClickRayCollisions[0] = 0;
                bp22.queryClickRay(ray22);
                Index nHits = Scene<Backend, Precision>::rayTracedData
                    .numClickRayCollisions[0];
                if (nHits == 0) {
                    fail("D-029 fix / N=1 BVH safely bypasses bottom-up combine",
                         "queryClickRay missed the single triangle "
                         "(broad-phase walk broken for N=1)");
                } else {
                    pass("D-029 fix / N=1 BVH safely bypasses bottom-up combine");
                }
            }
        }
    }

    // Cleanup the temp obj (best-effort; harmless if it fails).
    std::remove(objPath.c_str());
}
```

Pass label: `D-029 fix / N=1 BVH safely bypasses bottom-up combine`.

### 3. TEST_MATRIX.md BDD-010 row prose update (NOTE fold-in)

`docs/TEST_MATRIX.md` line 24 (BDD-010 row): drop the
`objPair.query != objPair.target` wording and reflect the actual
assertion form. Status stays `pass`.

Suggested wording (Generator may refine):

> `src/main.cpp::runSelfTest` Block 13 — two clauses PASS
> (`BDD-010 / overlapping AABBs produce a contact pair between two
> distinct objects`, `…/ non-overlapping AABBs produce empty
> constraint set`). The positive clause asserts
> `cumulativeNarrowCollisions > 0` across the frame; with the
> Simulator default `enableSelfCollisions = false`, a non-zero
> cumulative count by definition reflects at least one inter-object
> pair. The OLD `lastSubstep` iteration was an artifact of the OLD
> CPU refit's 2-substep BVH lag and is captured in CM-011.

### 4. Bookkeeping

- `.agent/CURRENT_WORK.md` / `RESUME.md` — fix-turn progress.
- `docs/DECISIONS.md::D-029` — Generator can append a brief
  "Fix-turn (Estimator turn 23): N=1 guard added in
  `bottomUpBoxesGPU` after the discovery that `buildTree_*` doesn't
  write `treeParent[0]` for single-primitive trees" sentence to the
  existing D-029 entry's rationale. No new D-NNN.
- `docs/mistakes/COMMON_MISTAKES.md` — consider adding a CM only
  if this trap pattern is likely to recur (e.g., another future
  kernel reading from a parallel array maintained by a separate
  kernel that has early-return paths). Generator's call; small
  pattern entry would be fine.

## Non-goals (this fix-turn)

- **GPU `enlargeTrajectory`** — separate slice candidate; not in
  this fix-turn.
- **Restructuring `buildTree_*`** to always write `treeParent[0]`
  (even for the leaf-root case). Rejected because the natural
  semantic is "treeParent only stores interior-node parent links";
  for N=1 the root has no parent, and the existing convention
  matches that. Guarding at the consumer (`bottomUpBoxesGPU`) is
  the right layer.
- **N=2 special-case investigation.** N=2 has 1 interior + 2
  leaves; `buildTree_*` writes `treeParent[childA]=0` and
  `treeParent[childB]=0` for the interior root. `bottomUpBoxes`
  works normally. No additional guard needed.
- **CPU `bottomUpCombine`** path — already handles N=1 correctly
  via `if (node.childA < 0) return;` recursion termination. No
  change.
- **Any new D-NNN.** The fix is a guard refinement on D-029's
  existing contract; D-029's rationale gets a one-sentence
  addendum from the Generator, not a new decision.
- **Other matrix rows, spec edits, Q-resolution.**

## Todo

Ordered. Generator executes top-to-bottom.

1. **Branch hygiene.** Stay on `feat/bvh-bottomup-gpu` (this is a
   BLOCK fix-turn; same branch, commit prefix `fix:`).

2. **Add the N=1 guard** to
   `BVH<METAL,PR,LINEAR,PRIMITIVE>::bottomUpBoxesGPU`. Widen the
   existing `if (numPrimitives == 0) return;` to
   `if (numPrimitives <= 1) return;`. Update the comment to
   explain why N=1 also short-circuits.

3. **Author Block 22** per §2 above. Pass label
   `D-029 fix / N=1 BVH safely bypasses bottom-up combine`. Make
   sure to include `<fstream>` and `<cstdio>` (for `std::remove`)
   if not already pulled in transitively.

4. **Update `docs/TEST_MATRIX.md`** BDD-010 row prose per §3.

5. **Build cleanly.** `cmake --build build`. Expect zero new
   warnings.

6. **Run `./scripts/verify-light.sh`.** Doctest binaries should
   stay 159/159 + 1120/1120.

7. **Run `--self-test` 5+ times.** Expect **42/42 PASS**
   consistently (current 41 + Block 22).

8. **Bug-probe.** Two probes:
   - **(a) Remove the N=1 guard** (revert step 2). Run self-test.
     Expected: Block 22 may FAIL with crash / wrong tree AABB / 0
     hits — depending on what garbage `treeParent[0]` happens to
     hold on this run. Behavior is non-deterministic but should
     surface at least intermittently. If it silently passes on
     Apple Silicon (zero-init buffers), note in CURRENT_WORK that
     the bug-probe was Apple-Silicon-masked; the production guard
     is still the right fix for spec correctness / portability.
     Restore.
   - **(b) Force `treeParent[0] = numNodes`** (set to an
     out-of-bounds value via temporary C++ write before
     `bottomUpBoxesGPU` dispatch — only valid if guard is reverted).
     Re-run. Expected: deterministic OOB write through
     `treeVisitCounts[numNodes]` which is past the buffer end →
     either crash or silent corruption surfaces in adjacent buffer
     (e.g., the next allocated VectorBase). Drop this probe if
     too invasive; (a) plus the structural guard is sufficient.

9. **Append D-029 fix-turn sentence** to `docs/DECISIONS.md::D-029`
   rationale section. No new D-NNN.

10. **(Optional) Add CM-012** to `docs/mistakes/COMMON_MISTAKES.md`
    if the trap pattern (kernel A's early-return leaves parallel
    array uninitialized; kernel B consumes the array assuming it's
    populated for all indices) is worth recording. Generator's
    call.

11. **Update `CURRENT_WORK.md` / `RESUME.md`** to reflect the
    fix-turn shipped. Drop the previous WARNING-pending state;
    note 42/42 PASS.

12. **Stop and hand off to Estimator (Codex)**. Expected verdict:
    NOTE-clean.

## Course corrections

- **Stricter-than-spec assertion** (PLANNER.md step 7). Block 22
  asserts not just "no crash" but also "tree[0] reflects the
  triangle's AABB" AND "queryClickRay finds the triangle". Each
  layer catches a different fail mode:
  - No-crash: catches OOB atomic write that crashes.
  - tree[0] AABB: catches silent garbage write to tree[0] via
    second-arrival path with garbage `parent` index.
  - queryClickRay: catches BVH structural corruption (the walk's
    leaf-recognition via `childA < 0` requires tree[0] to remain
    intact as the leaf-root).
  3 assertions, 1 block, deterministic + cheap.

- **Architectural invariants applying here:**
  - **D-024** (BVH leaf return) — `queryClickRay` walks the BVH;
    for N=1, tree[0] is the leaf (`childA < 0`), and the walk's
    leaf branch should fire immediately. Block 22's queryClickRay
    check exercises this.
  - **D-026** (lifetimeId Float-mesh skip) — unaffected; this
    fix is in `bottomUpBoxesGPU`, downstream of the skip gate.
  - **D-029** (GPU bottom-up) — the fix extends D-029's
    pre-condition: `bottomUpBoxes` requires `numPrimitives >= 2`
    (i.e., at least one interior node exists). Caller must guard
    via `bottomUpBoxesGPU`; the kernel itself is unchanged.
  - **CM-011** (substep-loop commit boundary) — unchanged; this
    fix doesn't touch the substep loop or commit timing.

- **No new BVH variant** is being introduced. The fix only
  refines the existing path's pre-condition; no kernel signature
  change, no buffer change, no template instantiation change.

- **`buildTree_*`'s treeParent invariant explicitly named.**
  `treeParent[i]` for `i ∈ [1, numNodes)` is set by
  `buildTree_*`'s interior-node branch. `treeParent[0]` (root)
  is NEVER set by `buildTree_*` because the root has no parent.
  This is fine when there's at least 1 interior node (the root
  itself, written as childA or childB of another interior node
  is impossible since root has no parent — so root's
  `treeParent[0]` slot stays effectively-undefined-but-never-read
  for N >= 2). For N=1, the only node IS the root, AND
  `buildTree_*` returns early without entering the interior-node
  branch at all. So treeParent[0] is undefined for N=1, and
  `bottomUpBoxes` must not read it. The guard enforces this.

## What to read before writing code

- `src/main.cpp::BVH<METAL,PR,LINEAR,PRIMITIVE>::bottomUpBoxesGPU`
  (~line 3261-area; find by signature). The 1-line guard widening
  goes here.
- `src/main.cpp::runSelfTest` Block 7 (~line 5891) — template for
  importMesh harness pattern. Block 22 mirrors the `.obj`
  on-the-fly composition + import path.
- `src/main.cpp::runSelfTest` Block 21 (just before the `if
  (failures == 0)` line) — placement reference for Block 22
  (append immediately after Block 21).
- `src/metal/bvh.metal::buildTree_Tri` lines 284-325 — confirms
  the early-return at `idx == numPrimitives - 1` before
  `treeParent` writes. `buildTree_Edge` lines 352-392 same shape.
- `src/main.cpp::Simulator::importMesh` (~line 4473-area) — the
  existing import path; takes (prefix, fileName, scale, mass,
  errOut).
- `docs/DECISIONS.md::D-029` — current entry to append the
  fix-turn sentence to.
- `docs/TEST_MATRIX.md:24` — BDD-010 row prose to update.
- Estimator turn 23 verdict (`docs/.agent/ESTIMATION.md` if not
  yet overwritten; current state is the BLOCK report).
