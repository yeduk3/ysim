# Resume — Primitive Creation Slice

## Must remember

- **Branch:** `feat/primitive-creation` (off `main`, after `feat/verify-and-load-warnings` was fast-forwarded into `main`).
- **Geometry library is CPU-pure** (`include/primitive_geometry.hpp`). It deliberately does **not** depend on Metal, GLFW, tinym, or `MeshState`/`MeshAdjacency`. Tests live in a *second* doctest binary (`test/primitive_test.cpp` → `ysim_primitive_tests`) because each doctest source file owns its own `DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN` and one main per binary is the rule. Both verify scripts were updated to run both binaries — keep that in mind when adding a third test binary.
- **`tessellation` parameter has shape-specific meaning.** For sphere it's the longitude segment count *and* the latitude segment count (so a sphere with `tessellation=16` has 16×16 ish vertices). For cube it's the cells-per-face-edge count (so `tessellation=2` means a cube with 6×9=54 vertices and 6×8=48 triangles). The schema's single `tessellation` integer is fine; just remember the interpretation when reading scenes.
- **Cube faces are not vertex-shared.** Six independent face patches; the resulting mesh is a disconnected manifold. This is fine for v1 because new primitives default to `Float` behavior (no cloth physics over the surface), and the BVH operates triangle-by-triangle regardless of connectivity. If a future slice runs cloth physics on a cube, it will need a vertex-merging pass first.
- **`isReservedShape` is empty in v1** but the function still exists. Future shapes that aren't yet shipping go in there to opt into the loud-fail behavior — this is the D-003 / D-010 pattern still alive.
- **`tinym::vec4` order is `(x, y, z, w)`**, opposite of the schema's `[w, x, y, z]` quaternion order. Use the `Quat` struct in `src/main.cpp` (D-007) when storing rotation; do not re-derive the order from `vec4`.

## Last decisions + why

- **D-010** — `"sphere"`/`"cube"` ship as v1 primitives, amending D-003. Pattern (loud-fail on reserved names) preserved; only the membership of the reserved set narrows.

## Next step you were about to take

Slice complete. The next concrete step is the **Estimator's** turn — running `./scripts/verify.sh` and judging whether to merge or send back. After that, the candidate next slices (priority not yet decided; see `PROJECT_STATE.md`):

- Material editing UI (FR-005 / BDD-005) — needs PBR preview shader to satisfy "preview render reflects" clause.
- Behavior assignment UI (FR-006 / BDD-006) — in-place behavior switch reallocates per-mesh state; non-trivial.
- Rigid body slice (FR-008 / BDD-008) — blocked on Q4.
- Alembic export slice (FR-013 / BDD-013) — blocked on Q5/Q6.
- Test-harness slice — closes the parked persistence WARNING; forces Q-D.

See `.agent/PLAN.md` and `.agent/CURRENT_WORK.md` for full plan and progress.
