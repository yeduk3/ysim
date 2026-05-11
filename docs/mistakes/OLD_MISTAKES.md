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

## High-level cause: contact-response gates fall out of sync when the detection rule changes

- Origin entries: **CM-006** — narrow-phase slow-touch band widens but the integrator's vn-zero block fires unconditionally on any narrow contact, draining vy off particles that aren't penetrating.
- Why it stopped: **D-016** moved `integrate_cloth` and `integrate_cloth_grid`'s vn-zero `if (vn < 0.0f) vel -= vn * n;` block **inside** the existing `if (distance < thickness) { ... }` gate so vn-zero and the position-push share the same response gate. The narrow-phase's `inMargin` band stays at `radius + thickness` (D-013's swept-CCD invariant), but the integrator only reacts when the particle is actually within `thickness` of the surface — not just within the wider detection band. The asymmetry between detection (`radius + thickness`) and response (`distance < thickness`) is intentional and is itself a load-bearing invariant of D-016.
- Direction for similar problems: when a kernel's contact-firing semantic changes (e.g., a swept test fires for plane-crossings regardless of `d_cur`, where the prior snapshot test only fired when within range), every consumer of that contact's downstream effect needs to be re-audited at the same time. The kernel's "I detected a contact" claim is wider than "this particle has hit the surface"; the response side must distinguish the two. Concretely: any time `nparams.thickness` or its analogue widens, grep the integrator(s) for the contact-loop body and confirm every mutation of `vel` / `pos` is gated symmetrically.

## High-level cause: global RNG state leaks across scene reconstructions

- Origin entries: **CM-007** — `MeshGridInitializer`'s jiggle used global `rand()`, so two consecutive `buildSyntheticScene + initialize()` calls within one process produced different `state.x` at-init for the same scene definition. BDD-102's bit-equality assertion failed at byte 0 of state.x.
- Why it stopped: **D-018** replaced `rand()` with a local `std::mt19937` seeded from `params.seed`, where the seed is wired from `mesh.id` (`Scene::numMeshes` pre-call in `addCloth`; `o.id` in `loadScene`). The RNG is fully scoped to the `initialize()` call — no shared global state. Same scene + same mesh ids → bit-identical jiggle sequence. Save/load reproduces jiggle for free because `o.id` is already serialized; no scene-format change needed.
- Direction for similar problems: any future per-mesh randomness (Perlin-noise displacement, jiggled normals, scattered debris, particle emission) must be seeded from a deterministic, scene-stable identifier — `mesh.id` is the canonical choice for v1 because the persistence layer already round-trips it. Avoid global `rand()`, `time(NULL)`, `std::random_device`, or hash-of-pointer-address — all of these leak environmental state that breaks BDD-102. When adding a new initializer subtype with randomness, also add a `seed` field to its params struct mirroring `MeshGridInitializerParams::seed` and wire it from the same call sites that set `mesh.id`.

## High-level cause: skip optimizations silently inherit prior-iteration identity when slot indices align

- Origin entries: **CM-008** — `BroadPhase::build`'s Float-tagged per-mesh BVH skip optimization checked the previous slot's `objBehavior == Float` but not whether the slot still referred to the same mesh. After `resetScene()` + new `addCube×N` at the same `numMeshes` count, the new meshes are also Float and occupy the same slot indices, so the skip fired against stale tree data and click-ray queries hit nothing (or the old object's old position). The harness carried a workaround (`sim.collisionPipeline.broadPhase.objTrees.clear()`) in seven self-test blocks for the duration the production bug was deferred.
- Why it stopped: **D-026** added a never-resetting monotone counter `Scene<BE,PR>::lifetimeMeshCount`; each `addGeneralMesh` request gets `mesh.lifetimeId = lifetimeMeshCount++` (distinct from `mesh.id`, which keeps its `numMeshes`-derived semantic for the D-018 RNG seed). The per-mesh `TRI_LBVH` caches `builtForLifetimeId` at build time; the skip in `BroadPhase::build` gains a third conjunct `objTrees[i].builtForLifetimeId == scene.meshes[i].lifetimeId`. The harness's seven `objTrees.clear()` workaround sites were removed in the same slice (Block 19 mechanizes the scene-swap-at-same-count round-trip without the manual clear, bug-probe verified).
- Direction for similar problems: any future cross-iteration cache that gates "skip rebuild" on a tag-style comparison (behavior type, shape type, material id, etc.) must also include a stable per-instance identity check. Tag matches alone permit "different instance, same tag" to satisfy the gate falsely. The fix shape is invariant: store the cached state's owning identity at build time, compare against the current iteration's identity at gate time, and use a never-resetting counter for the identity (not a value that can be reused — vector indices, `std::vector::push_back` reused addresses, or counters that reset on a "scene clear" event all give false positives). The same trap pattern applies to any future scene-level resource that's keyed by slot index (per-mesh constraint lists, render-state slots in `MeshRenderState`, future GPU resource caches).
