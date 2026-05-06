# Resume — Verification & Polish Slice

## Must remember

- **Branch:** `feat/verify-and-load-warnings` (off `main`, after `feat/persistence` was fast-forwarded into `main`). Do not write feature code on `main`.
- **`scripts/verify.sh` vs `scripts/verify-light.sh`** are deliberately distinct. `verify.sh` is the Estimator's strict gate (builds **every** CMake target, including the GUI binary `ysim`); `verify-light.sh` is the Generator's quick loop (test target only). If a future change has reason to drop the strict gate down to "tests only", that is a planning question — do not silently merge them.
- **Load warnings are surfaced inline in `sceneIOStatus`,** not in a new ImGui window. Each warning message is prefixed with `"warning: "` on its own line, after the `"loaded: <path>"` first line. If a future slice needs richer formatting (collapsible list, color, etc.), it can split this string into a structured panel — but a single string was the right cheap call here, matching the PLAN.
- **`SceneSnapshot::warnings` is non-fatal by design** (D-009). Treat clamping as a soft signal to the user, not as a load failure.

## Last decisions + why

No new `DECISIONS.md` entries this slice — the changes are tooling and a one-line UX improvement, neither of which has a non-obvious tradeoff a future reader couldn't derive from the diff. The relevant prior entries are still load-bearing: D-009 (`LoadWarnings` channel), D-002 (doctest as the test framework, separate executable, no Metal device).

## Next step you were about to take

Slice complete. The next concrete step is the **Estimator's** turn — running `./scripts/verify.sh` (now actually present) and judging whether to merge or send back. The slice's surface area is intentionally small; if anything blocks it, the cause is structural, not implementation.

After that, the next big slice candidates (decision pending) per `PROJECT_STATE.md`:
- Test-harness slice (closes the persistence WARNING about app-level coverage; forces Q-D).
- Material editing UI slice (FR-005 / BDD-005).
- Behavior assignment UI slice (FR-006 / BDD-006).
- Rigid body slice (blocked on Q4).
- Alembic export slice (blocked on Q5/Q6).

See `.agent/PLAN.md` and `.agent/CURRENT_WORK.md` for full plan and progress.
