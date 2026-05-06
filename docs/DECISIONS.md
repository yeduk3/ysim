# Decisions

> Owner: **Generator** (appends). Read by everyone. Numbered, append-only — never reorder.

A decision belongs here when a future reader could not derive *why* from the code alone. If the rationale is "obvious from the API docs", it does not belong here.

## Format

```
## D-NNN — <short title>

- File / function: <path>:<symbol>
- Decision: <what was chosen>
- Alternatives considered: <briefly>
- Rationale: <why this one — constraints, tradeoffs, prior incident>
- Date: <ISO date>
```

If a later decision supersedes an earlier one, add the new entry and reference the old one (`Supersedes D-007`). Do not edit or delete the old entry.

## Entries

<!-- D-001, D-002, ... -->
