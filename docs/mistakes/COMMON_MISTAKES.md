# Common Mistakes (Active)

> Owner: **Generator** appends; **Planner** and **Estimator** read.
>
> Active list of recurring failure modes. Criteria for what belongs here is in `docs/roles/GENERATOR.md` — do not pad this file with one-off bugs.

## Format

```
## CM-NNN — <short title>

- Where: <files / functions / boundary affected>
- Low-level cause: <the immediate bug pattern>
- High-level cause: <the underlying reason this keeps happening>
- Fix direction: <what to do instead, or what check to add>
- First seen: <ISO date>   Last seen: <ISO date>
```

When an entry has not recurred for a while and the underlying cause is gone (architecture changed, a check was added, a refactor removed the foot-gun), graduate it to `OLD_MISTAKES.md`.

## Entries

<!-- CM-001, CM-002, ... -->
