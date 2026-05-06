# Tests

> Owner: **Planner** authors scenarios; **Generator** implements them; **Estimator** validates BDD-fidelity.

One block per behavior id from `docs/specs/BDD.md` (or per functional requirement from `FRD.md`). The Generator turns each block into a test function and records the test address in `docs/TEST_MATRIX.md`.

## Format

```
## <BDD-ID> — <short scenario name>

**Given** <preconditions>
**When**  <action>
**Then**  <observable outcome>

Notes: <any edge case or constraint that the test must cover>
```

Keep this file authoritative — if the BDD changes, update the corresponding block here, then let the matrix flag the divergence.

## Scenarios

<!-- BDD-001, BDD-002, ... -->
