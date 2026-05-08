# Old Mistakes (Retired)

> Institutional memory. Entries graduate here from `COMMON_MISTAKES.md` once they stop recurring.
>
> Group by **high-level cause**, not by individual incident — the point is to record a consistent direction for future similar problems, not to relitigate every bug.

## Format

```
## High-level cause: <pattern>

- Origin entries: CM-NNN, CM-NNN
- Why it stopped: <what changed — refactor, check added, architecture shift>
- Direction for similar problems: <one short paragraph>
```

## High-level cause: snapshot-only collision tests miss fast-moving thin geometry

- Origin entries: **CM-005** — cloth tunnels through static ground despite broad/narrow contacts firing.
- Why it stopped: **D-013** replaced the point-vs-triangle distance snapshot in `narrow_pt_tri` with swept-segment CCD using a per-substep `xPrev` buffer (slot 10), and made the kernel emit **signed** distance so the integrator's `(thickness - distance) * n` push direction stays correct for tunneled particles. The prerequisite — `enlargeTrajectory(system.subh)` from the cloth-drape slice — inflates per-mesh AABBs by velocity × subh so broad phase still feeds candidate pairs into narrow during the transit. Together they removed the snapshot path entirely; the failure mode cannot recur in the same form.
- Direction for similar problems: any future contact / pickup / ray-test that consumes "is point X within radius of triangle T?" should consider whether the point's *trajectory* during the relevant time window can cross the surface between samples. If yes, run a swept test against the segment `[x_prev, x_cur]`, not just the snapshot. Always emit **signed** penetration depth so consumers can distinguish "above-but-close" from "below-and-must-be-pushed" without ambiguous abs.
