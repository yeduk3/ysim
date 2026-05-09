# Resume — Cloth Thickness Band Slice (CM-006 closed)

## Must remember

- **Branch:** `fix/cloth-thickness-band` (off `main` at `b0de64c`).
- **D-016 is load-bearing.** Both cloth integrator kernels (`integrate_cloth`, `integrate_cloth_grid`) gate vn-zero AND position-push behind the **same** `(distance < thickness)` condition. Re-introducing an unconditional vn-zero (or splitting the two gates) re-opens CM-006 — far-from-surface particles in the slow-touch band lose vy without any compensating push.
- **Detection band wider than response gate, on purpose.** `narrow_pt_tri`'s `inMargin = (d_cur < radius + thickness)` is wider than the integrator's `(distance < thickness)` gate. The asymmetry is correct: detection catches plane-crossings (D-013 swept-CCD) regardless of `d_cur`, but the integrator only responds when the particle is actually within `thickness` of the surface. Future slices that touch either side need to keep this asymmetry.
- **`simulator.margin = 0.015` is the v1 slow-touch band.** Per-mesh cloth thickness plumbing is parked. If the harness or a future slice surfaces a need for per-mesh values, plumb them through `packedMeshData` (similar to `statesOffsets`); don't reach into `clothParams` from the narrow kernel.
- **Substep count stays at 8.** D-013 tuned this to keep BDD-007 tunneling below the 0 threshold. Don't change it as a "speed-up" without a fresh BDD-007 multi-run determinism check.
- **Both kernels in tandem.** `integrate_cloth` (TriangularCloth, used by `addCloth`) and `integrate_cloth_grid` (FastGridCloth, GUI-reachable) have textually identical contact-loop bodies in this region. Future contact-loop changes need both kernels updated together; editing one silently breaks the other path.
- **CM-006 graduation breadcrumb** lives in `COMMON_MISTAKES.md` (line ~57). Future planners scanning the active list should see "graduated to OLD_MISTAKES" and not re-debate the open question.

## Last decisions + why

- **D-016** — vn-zero block moved inside the `(distance < thickness)` gate; call site passes `simulator.margin` instead of `PR(0)`. Closes CM-006 (parked since cloth-CCD turn 6). Rejected: ε-padded gate (no justification for the tunable), unconditional vn-zero with bigger constant (overcorrects approaching particles), shrinking detection band to match response gate (negates D-013's swept-CCD design). The asymmetry between detection and response gates is load-bearing and recorded.

## Next step you were about to take

Slice complete. Next concrete step is the **Estimator's** turn — `./scripts/verify.sh` should exit 0 cleanly with **23/23** self-test PASS lines. Expected verdict: NOTE level.

After this lands, planner-tracked candidates per `PROJECT_STATE.md`:

- **BDD-019 pause-gate refactor (Estimator turn-8 WARNING fold-up)** — extract render loop's `if (collectProfileFrame)` gate into a helper that the harness can also call, so Block 10's pause check exercises the actual gate instead of a proxy. Small (~10 lines + 1 assertion). Next slice.
- **BDD-102 Determinism mechanization** — extend the harness with two-runs-bit-identical assertion against a saved-scene baseline. Smallest slice.
- **Material editing UI (FR-005 / BDD-005)** — needs PBR preview shader.
- **Behavior assignment UI (FR-006 / BDD-006)** — in-place behavior switching reallocates per-mesh state.
- **Rigid body slice (FR-008 / BDD-008)** — blocked on Q4. When this lands it adds a fifth initializer subtype and triggers D-015's three-site cascade update.
- **Alembic export slice (FR-013 / BDD-013)** — blocked on Q5 + Q6.

See `.agent/PLAN.md` and `.agent/CURRENT_WORK.md` for full plan and progress.
