# Resume — Headless Self-Test Harness Slice (post BLOCK-fix)

## Must remember

- **Branch:** `feat/sim-self-test` (off `main`, after `feat/render-state-decoupling` was fast-forwarded into `main`).
- **Harness assertions are authored from `docs/TESTS.md` "Then" clauses verbatim.** The first turn's BLOCK was caused by writing assertions from the compressed matrix-row labels instead. The matrix label is too compressed to drive the assertion off — open `TESTS.md` and read the "Then" sentence.
- **Metal-unavailable is a SKIP, not a FAIL.** Codex (the Estimator) runs in a Linux container without Metal; treating that as a hard failure made `verify.sh` unrunnable for the Estimator. The skip path is implementation detail under D-012, not a separate decision.
- **`Simulator::update` is pure simulation.** The render-side per-frame mesh-buffer upload lives in `Simulator::uploadMeshes` and is called by the GUI loop only. The harness must NOT touch GL.
- **`BDD-011`'s "no restart" clause is enforced by code shape**, not by an assertion: there is no `simulator.initialize()` between the two `pumpFrames` calls in Block 4. Future maintainers who add a "reset before phase 2" line to be tidy break the spec — please don't.
- **Tolerance constants in the harness** (`bdd011_tol = 0.05`, `bdd012_tol = 0.01`) are sized against `subh = h / subSteps = 1/240` and the synthetic 4×4 cloth mass `0.1`. Tightening / loosening them is fine; just keep them well above FP noise and well below "what 4 frames of the configured force / mass would produce".
- **The harness cloth is small (4×4)** so each pack/init is fast. If a future block needs more vertices, scale the cloth params at the top of `runSelfTest` rather than per-block.

## Last decisions + why

- **D-012** — Headless Metal harness via `--self-test`. Resolves Q-D in favor of reusing the shipping Metal path; rejects CPU backend implementation as v1 work. Unchanged by the BLOCK-fix turn.

## Next step you were about to take

Slice complete (BLOCK-fix turn). The next concrete step is the **Estimator's re-review**. After that lands, the next planner-tracked candidates per `PROJECT_STATE.md`:

- **BDD-007 cloth drapes onto rigid surface** — newly tractable with the harness; collision response correctness can be added as a Block 6 in the same `runSelfTest` once the planner scopes the slice.
- **BDD-002 Import .obj mesh via UI** — small slice; underlying paths already work.
- **BDD-102 determinism mechanization** — extend the harness with two-runs-bit-identical assertion.
- **Material editing UI / Behavior assignment UI / Rigid / Alembic** — each waits on its respective spec answer or shader work.

See `.agent/PLAN.md` and `.agent/CURRENT_WORK.md` for full plan and progress.
