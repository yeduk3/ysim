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

## CM-001 — `::Material` shadowed by `scene_format::Material` inside `Simulator`

- Where: `src/main.cpp` — anywhere a `Simulator<BE,PR>` member references `Material` after `#include "scene_format.hpp"` brings `scene_format::Material` into a using-context.
- Low-level cause: both names resolve through ADL/qualified lookup; the compiler flags `Material` as ambiguous, not always at the first occurrence — the build can fail in a member function that *uses* `Material` long after the conflicting include.
- High-level cause: the persistence slice deliberately keeps `scene_format::*` POD types parallel to the C++-side runtime structs, but did not rename either side. The two will keep colliding as more fields are added.
- Fix direction: inside `Simulator` (and any future code that mixes both), qualify the C++-side struct as `::Material`. The same pattern likely applies to `Source`, `Transform`, `Object`, `Environment` if they ever grow runtime equivalents — check before adding a new collision.
- First seen: 2026-05-06   Last seen: 2026-05-06
