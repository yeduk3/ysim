# Plan — CM-006 vn-zero Gate + Slow-Touch Band (`fix/cloth-thickness-band`)

> Owner: **Planner** writes; **Generator** executes; **Estimator** judges.
> Updated: 2026-05-09

## Course note: previous slice's verdict

Estimator turn 8 on `fix/translate-pack-and-bdd019` (now merged at
`b0de64c`) returned **WARNING** (commit allowed, one item). The
WARNING is **deliberately not folded** into this slice — see Non-goals
section below. Reasoning: this slice touches Metal kernels in the
cloth integrator, which is the most numerically-sensitive area in the
project (50µm of drift in BDD-007 has BLOCKed past slices). Mixing in
a profiler refactor expands the slice's surface-area for marginal
gain. The turn-8 WARNING gets its own next slice.

CM-006 has been **deferred two slices in a row** (cloth-CCD turn 6 →
translate-object → translate-pack-and-bdd019). Per PLANNER.md
procedure step 6 (escape-pattern detection), three consecutive
deferrals would force a pivot. We close it here before that point.

## Goal

Close `CM-006` (the parked WARNING from cloth-CCD turn 6). After this
slice:

- `physics.metal::integrate_cloth` and `integrate_cloth_grid` gate
  the **vn-zero block** behind `(distance < thickness)`, matching the
  position-push gate that lives below it. Today the vn-zero block
  fires unconditionally on any narrow contact, so widening the
  slow-touch band drains normal-velocity off particles that are not
  actually penetrating.
- `BruteForce::narrow`'s call site passes `simulator.margin` (not
  `PR(0)`) so the kernel's slow-touch band is `radius + margin`
  instead of `radius`. This was the original cloth-CCD turn-6 plan;
  the call site was clamped to `0` in the prior slice because raising
  it without the kernel-side gate regressed BDD-007 by ~50µm.
- `BDD-007`'s tunneling clause (`no cloth vertex tunnels through
  ground`) **still PASSes**. That is the slice's non-negotiable
  acceptance gate — the change must not regress the standing 4/4
  cloth-on-ground clauses.
- `CM-006` graduates from `docs/mistakes/COMMON_MISTAKES.md` to
  `docs/mistakes/OLD_MISTAKES.md` per the file's promotion rule
  ("when an entry has not recurred for a while and the underlying
  cause is gone").

## Scope

- **`src/metal/physics.metal::integrate_cloth`** (line ~286). Move
  the vn-zero `if (vn < 0.0f) vel -= vn * n;` block from line ~327
  into the existing `if (distance < thickness) { ... }` block at
  line ~335, *before* the position push:

  ```cpp
  float distance = vertColFacets[i].collisionNormalAndDistance.w;
  float thickness = clothParams.thickness;
  if (distance < thickness) {
      if (vn < 0.0f) vel -= vn * n;       // <-- moved
      pos += (thickness - distance) * n;
  }
  ```

  Note that `vn` is computed at line ~325 from `vel` and `n` — it
  must continue to be computed *before* the gate (the values are
  read; the **mutation** of `vel` is what gets gated).

- **`src/metal/physics.metal::integrate_cloth_grid`** (line ~160).
  Same shape — the kernel is structurally identical in this region
  (lines ~199–211 mirror the triangular-cloth integrator). Apply
  the same gating.

- **`src/main.cpp::Simulator::update`** (~line 4609 / 4611). Replace
  the two `narrowAndSortByVertices(radius, PR(0))` calls with
  `narrowAndSortByVertices(radius, margin)`. Drop the multi-line
  comment about "thickness=0 preserves cloth-CCD turn-6 baseline" /
  "CM-006 / D-NNN" — that comment is now stale; the gate is in
  place. Replace with a one-liner pointing at D-016.

- **`src/main.cpp::BruteForce::narrow`** (~line 4067). Update the
  small comment block in front of `nparams.thickness = ...` to
  reflect the new state: slow-touch band is `radius + thickness`,
  caller passes a real value, kernel-side vn-zero is gated.

- **CM-006 graduation.** Move CM-006 from
  `docs/mistakes/COMMON_MISTAKES.md` to
  `docs/mistakes/OLD_MISTAKES.md` under a `## High-level cause:
  contact response gates inherit from older code paths` group (or
  similar). Format per OLD_MISTAKES.md's existing pattern: origin
  entry id, why it stopped, direction for similar problems.

- **DECISIONS — D-016.** Record the load-bearing pairing: kernel
  vn-zero gate matches position-push gate, both keyed on
  `(distance < thickness)`. The narrow phase's `inMargin` band is
  intentionally **wider** than the integrator's response gate; the
  asymmetry is fine because the *response* is gated by the
  integrator itself.

- **`runSelfTest` verification (no new assertions).** Block 6
  (BDD-007) and Blocks 9–10 (BDD-003 / BDD-019) all stay green. The
  slice's value is correctness-of-existing-behavior; no new test
  rows. The Generator confirms 23/23 PASS unchanged after the
  kernel rebuild + binding wiring.

## Non-goals (this slice)

- **Estimator turn-8 WARNING (BDD-019 pause check is proxy-level).**
  Defers to its own next slice. Reasoning: closing it cleanly likely
  needs a small refactor (extract the render-loop's `if
  (collectProfileFrame)` gate into a helper that the harness can
  also call). That refactor adds API surface and changes production
  code structure. The Metal-kernel work in this slice is risky
  enough that I want the slice's surface narrow to BDD-007 +
  cloth-related regression; mixing in profiler concerns is
  scope-creep. Recorded as the **next** standing candidate.

- **Per-mesh thickness plumbing into the kernel.** Today
  `nparams.thickness` is global across all cloth meshes. A future
  slice can plumb per-mesh thickness through the narrow kernel via
  packed buffers (similar to `packedMesh.statesOffsets`), but that
  is its own design problem and out of scope. `simulator.margin`
  (0.015) is a reasonable global fallback for v1.

- **Cloth-on-cloth (self) collision.** Still parked at PRD §4.

- **Any kernel change beyond the vn-zero gate move.** Don't refactor
  the contact loop body, don't tweak `nlen2 < 1e-12f` thresholds,
  don't reorder buffer bindings. The gate move is the slice; that
  is the entire kernel-side surface.

- **New BDD coverage.** No new matrix rows, no new assertions in
  `runSelfTest`. The slice is a structural correctness fix — the
  existing 23/23 PASS is the contract.

- **Resolving any of `Q1`, `Q2`, `Q4`, `Q5`, `Q6`, `Q7`.**

## Todo

Ordered. Generator executes top-to-bottom.

1. **Branch hygiene.** Already on `fix/cloth-thickness-band` (off
   `main` at `b0de64c`). No new branch. Commit prefix: `fix:`.

2. **Re-read CM-006.**
   `docs/mistakes/COMMON_MISTAKES.md` (lines 57–72) lays out the
   exact fix shape — the planner already cribbed from it; the
   Generator should re-read so the kernel edit isn't a paraphrase.

3. **Edit `src/metal/physics.metal::integrate_cloth_grid`** (line
   ~160, vn-zero around line ~202): move the
   `if (vn < 0.0f) vel -= vn * n;` line **inside** the
   `if (distance < thickness) { ... }` block, *above* the existing
   `pos += (thickness - distance) * n;` line. The `vn = dot(vel, n)`
   computation stays where it is (above the gate) — only the
   mutation of `vel` is gated.

4. **Edit `src/metal/physics.metal::integrate_cloth`** (line ~286,
   vn-zero around line ~327): same gate move. The two kernels are
   structurally identical in this region; do both in one read-write
   pass to ensure symmetry.

5. **Wire `simulator.margin` from the call site.**
   `src/main.cpp::Simulator::update` (~line 4609 and 4611): change
   `narrowAndSortByVertices(radius, PR(0))` →
   `narrowAndSortByVertices(radius, margin)`. Both arms of the
   `if (profiler) {...} else {...}` block.

6. **Update the comment block on `BruteForce::narrow`'s
   `nparams.thickness =` line** (~`src/main.cpp:4067`). Replace the
   "thickness=0 preserves cloth-CCD turn-6 baseline" text with one
   line pointing at D-016: `// Slow-touch band is radius + thickness;
   integrator gates vn-zero on (distance < thickness) per D-016.`
   And in `Simulator::update`'s narrow-phase scope, remove the stale
   multi-line block comment about "CM-006 / D-NNN" — it's now
   misleading.

7. **Run `./scripts/verify-light.sh`.** Confirm doctest binaries
   still green.

8. **Run `--self-test` 5+ times in a row.** Per the cloth-CCD slice
   precedent: BDD-007's tunneling tolerance is strict (0.0). The
   prior slice surfaced a 47µm drift that varied across builds. The
   integrator change here is small but in the same numerical
   neighborhood — confirm determinism across multiple runs (or note
   the variance band if any) before concluding the slice is safe.
   Expected: **23/23 PASS** consistently. If BDD-007's tunneling
   clause regresses, **stop and hand back to the Planner** — the
   gate move may be interacting with `subSteps=8` differently than
   expected, and bumping substeps further or revisiting the kernel
   edit is a Planner decision, not a Generator one.

9. **Graduate CM-006 to OLD_MISTAKES.md.** Move the entry under a
   new high-level-cause section
   `## High-level cause: contact-response gates inherit from older
   code paths` (or a name the Generator picks if better fits).
   Format per OLD_MISTAKES.md's pattern. Remove from
   COMMON_MISTAKES.md.

10. **Add D-016 to `docs/DECISIONS.md`.** Record the kernel-side gate
    pairing as a load-bearing decision: vn-zero and position-push
    must share the same `(distance < thickness)` gate. The asymmetry
    between the kernel's slow-touch band (`radius + thickness`) and
    the integrator's response gate (`distance < thickness`) is
    intentional — the **detection** can be wider than the
    **response**. File / function / decision /
    alternatives-considered / rationale per the standard format.

11. **Update CURRENT_WORK / RESUME.** Four-line max as work proceeds;
    write RESUME near end of turn.

12. **Stop and hand off to the Estimator.** Don't touch the BDD-019
    pause check, don't refactor the integrator beyond the gate move,
    don't change substep counts.

## Course corrections

- **Numerical sensitivity guard.** The cloth-CCD slice's last incident
  was a 47µm BDD-007 regression caused by setting `nparams.thickness
  > 0` *without* the integrator-side gate. This slice does both
  changes in one go. The kernel-side gate is *not* an optimization
  or refactor — it is a load-bearing pre-condition for raising
  `nparams.thickness`. If the Generator's verify run shows BDD-007
  drift even by tens of micrometers, that is a slice-critical
  signal, not noise. Stop and re-plan.

- **Both kernels in one pass.** `integrate_cloth` and
  `integrate_cloth_grid` are structurally identical in the contact
  loop region. Editing only one would silently break BDD-007 only
  for the cloth variant the harness uses. The harness uses
  `addCloth` (TriangularCloth → `integrate_cloth`), but the
  FastGridCloth path (`integrate_cloth_grid`) is also reachable via
  the GUI. Edit both. Do not assume one path is exercised by the
  harness and the other is dead.

- **Substep count stays at 8.** D-013's `subSteps = 8` was tuned to
  keep BDD-007 tunneling below the 0 threshold under the
  `nparams.thickness = 0` band. With the wider band (`+ margin`),
  the integrator's response fires on more contacts, generally
  *helping* tunneling. If anything, settling time may shorten. But
  do **not** lower substeps as a "speed-up" in this slice — that's
  scope creep with risk.

- **Gate move is symmetry, not behavior change.** The new shape
  (`if (distance < thickness) { vn-zero; position-push; }`) is what
  the loop *would have looked like* if it had been written from the
  start to match the position-push semantic. The asymmetry between
  vn-zero and position-push was an artifact of the cloth-CCD slice's
  swept-CCD upgrade — that slice introduced the new contact
  semantics (CCD can fire on plane-crossing regardless of `d_cur`)
  but only retrofitted the position-push to gate on
  `(distance < thickness)`. The vn-zero kept its old unconditional
  shape because it was working under the old detection rule. This
  slice closes that asymmetry.

## What to read before writing code

- `docs/mistakes/COMMON_MISTAKES.md::CM-006` (lines 57–72) — exact
  fix shape, prescribed by the entry itself.
- `src/metal/physics.metal::integrate_cloth_grid` (lines 160–218)
  and `integrate_cloth` (lines 286–344) — the two kernels to edit.
  Mirror images; do both.
- `src/main.cpp::Simulator::update` (~line 4609 / 4611) — the call
  site that currently passes `PR(0)`.
- `src/main.cpp::BruteForce::narrow` (~line 4067) — the comment
  block to clean up.
- `docs/mistakes/OLD_MISTAKES.md` — graduation format.
- `docs/DECISIONS.md::D-013` — the swept-CCD upgrade that
  introduced the asymmetry CM-006 documents. D-016 builds on it.
- `src/main.cpp::runSelfTest` Block 6 (BDD-007) — the load-bearing
  acceptance gate for this slice. Read so the Generator knows what
  the assertions look like before running and is not surprised by a
  PASS/FAIL message format mismatch.
- `.agent/RESUME.md` (translate-pack slice) — `subSteps = 8` is
  load-bearing for BDD-007's tunneling tolerance; don't touch.
