# Test Matrix

> Index that connects BDD scenarios to actual test functions and their pass/fail status.
>
> - **Planner** adds rows when authoring scenarios (test address blank, status `pending`).
> - **Generator** fills in the test address and updates status when running.
> - **Estimator** verifies status against the latest `./scripts/verify.sh` run.

Use this as the entry point — `docs/TESTS.md` will get long, so find scenarios here first.

## Matrix

| Behavior ID | Scenario | Test address | Status |
| --- | --- | --- | --- |
| <!-- BDD-001 --> | <!-- short name --> | <!-- test/foo.spec.ts::<fn name> --> | <!-- pending / pass / fail / skipped --> |

## Status legend

- `pending` — scenario exists, no test code yet
- `pass` — test exists and last run passed
- `fail` — test exists and last run failed (Estimator must classify as WARNING/BLOCK)
- `skipped` — test exists but is intentionally skipped; reason in `DECISIONS.md`
