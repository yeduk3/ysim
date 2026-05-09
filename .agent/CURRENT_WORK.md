# Current Work — Cloth Thickness Band Slice (`fix/cloth-thickness-band`)

- File in flight: none — slice complete; ready for Estimator. **23/23 self-test PASS deterministic across 5 runs**, `./scripts/verify-light.sh` clean.
- How far: all 12 PLAN todos done.
  - **D-016** — `integrate_cloth` and `integrate_cloth_grid` move the vn-zero block inside the existing `if (distance < thickness)` gate, matching the position-push semantic. Detection band stays asymmetrically wider (`radius + thickness`) — that's the load-bearing invariant. Gate move applied to both kernels via single Edit with `replace_all=true` because the contact-loop bodies were textually identical.
  - **Call site wired** — `Simulator::update`'s two `narrowAndSortByVertices(radius, PR(0))` calls now pass `margin` instead. Stale multi-line "thickness=0 baseline / CM-006 / D-NNN" comment dropped; replaced with a one-liner pointing at D-016 in `BruteForce::narrow`.
  - **CM-006 graduated** — moved from `COMMON_MISTAKES.md` to `OLD_MISTAKES.md` under a new high-level-cause section "contact-response gates fall out of sync when the detection rule changes". Active list now has only CM-001/CM-002/CM-003/CM-004 + the two graduation breadcrumbs (CM-005, CM-006).
  - **BDD-007 tunneling clause stays PASS** — verified via 5 consecutive `--self-test` runs. No drift, no FAIL, no SKIP. The pre-flight worry from the cloth-CCD slice's 47µm regression did not materialize because the kernel-side gate was the missing piece.
- Non-goals respected: turn-8 WARNING (BDD-019 pause check) deliberately deferred; no kernel changes beyond the gate move; no substep tuning; no new BDD coverage.
- What's tested:
  - **23/23 self-test PASS** on macOS Apple Silicon, deterministic across 5 runs.
  - Doctest binaries unchanged (159 + 1120 assertions, both green).
  - `BDD-007 / no cloth vertex tunnels through ground` repeatedly PASSes — slice's non-negotiable acceptance gate.
- What's next: Estimator review. Expect verdict at NOTE level — CM-006 closure, BDD-007 stable, scope was kept narrow.
