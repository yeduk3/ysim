# Plan — FR-005 material-edit data-layer subset (`feat/material-inspector-data-layer`)

> Owner: **Planner** writes; **Generator** executes; **Estimator** judges.
> Updated: 2026-05-11

## Course note: previous slice's verdict

Estimator turn 20 returned **NOTE** (no WARNING, no BLOCK) on the
CM-008 production-side fix slice. The single NOTE: `docs/TEST_MATRIX.md:31`
BDD-017 row still references the removed `objTrees.clear()`
workaround. Tiny (~1 line). Folded into this slice as the
bookkeeping todo.

## Why this slice now

Material editing UI (FR-005 / BDD-005) is the highest-priority
unblocked feature candidate. Q1 settled by D-005 — v1's OpenPBR
subset is base color, metallic, roughness, specular weight, emission
color. All five already round-trip through saveScene/loadScene
(BDD-014/015 mechanize this, pass). What's missing is a clean public
API surface for editing them at runtime + a Block that mechanizes
BDD-005's data-layer "Then" clauses.

The renderer-side clause from BDD-005 ("preview render reflects the
lower roughness") is **parked** — the PBR preview shader is a
separate, larger slice that this one explicitly does not attempt.
Matrix row promotes `pending → warning` (the documented-gap
convention used historically for BDD-007 pre-CCD, BDD-015 pre-harness,
BDD-009/011/012 pre-harness).

## Design call (the question that's been blocking this)

Inspector edits need to reach `mesh.material`. Two shapes:

- **Shape A — direct mutation.** The inspector holds a raw pointer
  to `mesh->material` and writes fields directly. Current code at
  `src/main.cpp:7367` already does this for `baseColor`. Persistence
  works because `toSnapshot` reads `mesh.material` directly.
  **Cost:** zero new API. **Gap:** no public mutator means no clean
  BDD-005 test address; pack-roundtrip survival depends on whether
  some other path happens to read mesh.material at the right
  moment.

- **Shape B — `Simulator::setMaterial(meshId, Material)` mutator.**
  Mirrors `translateObject` (D-014) and `rotateObject` (D-021):
  guard-by-findById, mutate live state, write to side-table for
  re-pack survival. The `pendingMaterials` map already exists
  (loadScene populates it; D-025 auto-calls `applyPendingMaterials`
  from `Simulator::initialize`). Mutator body is ~5 lines:
  ```cpp
  void setMaterial(int meshId, const Material& mat) {
      auto* mesh = Scene<BE,PR>::findById(meshId);
      if (!mesh) return;
      mesh->material = mat;
      pendingMaterials[meshId] = mat;
  }
  ```
  Inspector calls `sim.setMaterial(id, edited)` on every change.

**Decision: Shape B.** Reasons:

1. **Test address mirrors BDD-003/004.** Block 20's pass labels can
   call `sim.setMaterial(id, m)` and assert against `mesh.material`,
   matching the shape of Block 9 (translate) and Block 12 (quat
   composition). Without the mutator the test would either mutate
   `mesh.material` directly (which the user-facing flow never does)
   or invoke ImGui from the harness (which the harness doesn't run).
2. **Pack-roundtrip survival is free.** D-025's auto-applyPendingMaterials
   already restores `mesh.material` from `pendingMaterials[id]` after
   every `Simulator::initialize()`. Writing the side-table at mutator
   time piggybacks on that contract — addCube/addCloth/importMesh
   after a setMaterial preserve the edit, mirroring D-015/D-025.
3. **Inspector stays simple.** ImGui::ColorEdit3 / SliderFloat
   return `true` on change; the inspector fires `setMaterial` from
   the same code path. No special "commit on release" gate needed
   (unlike D-014's IsItemDeactivatedAfterEdit for translate, which
   exists because translate triggers a BVH refit-via-edit and is
   slightly heavier; material edits are pure CPU-side memcpy).

D-027 records the choice.

## Goal

After this slice:

- `Simulator::setMaterial(int meshId, const Material& mat)` exists
  in `src/main.cpp::Simulator`. Mutates `mesh->material` AND writes
  `pendingMaterials[meshId] = mat`.
- `MeshInspectorTarget` (in `include/mesh_inspector.hpp` or wherever
  it lives) gains fields/callbacks for the four currently-missing
  material parameters: `metallic`, `roughness`, `specular_weight`,
  `emission_color`. The existing `base_color` plumbing is preserved
  but rewired to go through `setMaterial`.
- The inspector renders 5 ImGui widgets (`ColorEdit3` for the two
  RGB triples, `SliderFloat` 0..1 for the three scalars). On change,
  each widget calls back to `simulator.setMaterial(id, current)`.
- Block 20 in `runSelfTest` mechanizes BDD-005's data-layer clauses:
  set non-default material via `setMaterial`, assert all 5 fields
  written; sim.update() (BDD-103 backend-boundary check); save to
  /tmp, reset, load, init, assert all 5 fields restored; re-pack
  survival (addCube + initialize, assert preserved).
- `docs/TEST_MATRIX.md` BDD-005 row promotes `pending → warning`
  with test address pointing at Block 20. The `warning` is the
  parked PBR-preview-shader clause (documented gap).
- BDD-017 row prose (line 31) updated: drop the stale
  `objTrees.clear()` sentence, replace with D-026 reference.
  ~1 line.
- New D-027 records the `setMaterial` mutator + pendingMaterials
  write-back invariant.

## Scope

### 1. Production fix — D-027 — Shape B

**`src/main.cpp::Simulator::setMaterial`** — new method on
`Simulator`, placed adjacent to `translateObject` / `rotateObject`
(~line 4540-4620 area, find the natural spot in the cluster).

```cpp
// D-027: edit-time material mutator. Mirrors translateObject (D-014)
// and rotateObject (D-021). Writes pendingMaterials[id] so the
// edit survives Scene::pack rebuild via D-025's auto-call from
// Simulator::initialize().
void setMaterial(int meshId, const Material& mat) {
    auto* mesh = Scene<BE,PR>::findById(meshId);
    if (!mesh) return;
    mesh->material = mat;
    pendingMaterials[meshId] = mat;
}
```

No `broadPhase.refit()` needed — material doesn't affect AABBs.

### 2. Inspector wiring — FR-005 surface

**`include/mesh_inspector.hpp`** (or whichever header owns
`MeshInspectorTarget`) gains the four missing pointer fields + a
material-edit callback. The existing `base_color` pointer field is
preserved but its consumer is rewired through the callback. Suggested
shape (Generator may tune naming to match existing convention):

```cpp
struct MeshInspectorTarget {
    // ... existing fields ...
    tinym::vec3* base_color = nullptr;            // existing
    float*       metallic = nullptr;              // NEW
    float*       roughness = nullptr;             // NEW
    float*       specular_weight = nullptr;       // NEW
    tinym::vec3* emission_color = nullptr;        // NEW
    std::function<void(int, Material)> on_material_edit;  // NEW
};
```

**`buildSelectedMeshTarget` lambda** (`src/main.cpp:7361`) populates
the 4 new pointer fields from `selectedMesh->material.*` and wires
`on_material_edit = [&simulator](int id, Material m) {
simulator.setMaterial(id, m); }`.

**The inspector rendering site** (wherever `MeshInspectorTarget` is
consumed by ImGui code) gains 5 widgets. Per-widget pattern:

```cpp
if (target.metallic && ImGui::SliderFloat("Metallic", target.metallic, 0.0f, 1.0f)) {
    Material edited{*target.base_color, *target.metallic, *target.roughness,
                    *target.specular_weight, *target.emission_color};
    if (target.on_material_edit) target.on_material_edit(target.mesh_id, edited);
}
```

Generator picks the cleanest factoring (e.g., a small helper that
constructs the `edited` snapshot once and reuses it across the 5
widgets' change branches).

**Clamping** is implicit via `ImGui::SliderFloat`'s 0..1 range —
the slider physically cannot drag out-of-range. No validation in
`setMaterial` itself; out-of-range values from save files are
preserved as authored (the user may have hand-edited JSON for
debugging or v2 LLM control). BDD-005 Notes mentions clamping —
the slider-range form of clamping satisfies it for the inspector
path; the JSON-load path intentionally does not clamp (philosophical
alignment with D-005's "data layer is the authority" stance).

### 3. Block 20 — `runSelfTest` mechanization

Append after Block 19 (the CM-008 / D-026 scene-swap block).

```cpp
// ---- Block 20: D-027 — setMaterial closes BDD-005's data-layer clauses. ----
// Renderer-side clause ("preview render reflects the lower
// roughness") is parked under the PBR-preview-shader slice; this
// block mechanizes the data-layer subset: setMaterial writes all 5
// fields to mesh.material; a sim step preserves them (BDD-103
// backend-boundary); save/load round-trips them; addCube re-pack
// preserves them (D-025 sister mechanization).
{
    resetScene();
    sim.addCube(tinym::vec3(0.0f, 0.0f, 0.0f), /*tess=*/2,
                /*size=*/0.5f, /*mass=*/0.1f);
    sim.initialize();
    const int matMeshId = 0;

    auto* m0 = Scene<Backend, Precision>::findById(matMeshId);
    if (!m0) {
        fail("BDD-005 / setMaterial writes all 5 fields to mesh.material",
             "cube id=" + std::to_string(matMeshId) + " not found");
    } else {
        // Default material assertion (sanity — D-005 defaults).
        const ::Material defaultMat;  // {1,1,1}, 0, 0.5, 1, {0,0,0}
        // ... (no assert, just baseline reference)

        // Phase 1: setMaterial writes all 5 fields.
        ::Material edited;
        edited.baseColor = tinym::vec3(0.2f, 0.3f, 0.4f);
        edited.metallic = 0.7f;
        edited.roughness = 0.1f;
        edited.specularWeight = 0.8f;
        edited.emissionColor = tinym::vec3(0.05f, 0.10f, 0.15f);
        sim.setMaterial(matMeshId, edited);

        const float matTol = 1e-6f;
        auto matEqual = [&](const ::Material& a, const ::Material& b) {
            return std::abs(a.baseColor.x - b.baseColor.x) < matTol
                && std::abs(a.baseColor.y - b.baseColor.y) < matTol
                && std::abs(a.baseColor.z - b.baseColor.z) < matTol
                && std::abs(a.metallic - b.metallic) < matTol
                && std::abs(a.roughness - b.roughness) < matTol
                && std::abs(a.specularWeight - b.specularWeight) < matTol
                && std::abs(a.emissionColor.x - b.emissionColor.x) < matTol
                && std::abs(a.emissionColor.y - b.emissionColor.y) < matTol
                && std::abs(a.emissionColor.z - b.emissionColor.z) < matTol;
        };

        if (!matEqual(m0->material, edited)) {
            fail("BDD-005 / setMaterial writes all 5 fields to mesh.material",
                 "mesh.material differs from edited after setMaterial: "
                 "roughness=" + std::to_string(m0->material.roughness));
        } else {
            pass("BDD-005 / setMaterial writes all 5 fields to mesh.material");
        }

        // Phase 2: a sim step preserves material (BDD-103 backend-boundary —
        // the simulation kernels must not clobber material fields).
        sim.update();
        auto* m0_postStep = Scene<Backend, Precision>::findById(matMeshId);
        if (!m0_postStep || !matEqual(m0_postStep->material, edited)) {
            fail("BDD-005 / material survives one sim step",
                 "material drifted after sim.update() — kernel clobber?");
        } else {
            pass("BDD-005 / material survives one sim step");
        }

        // Phase 3: save → reset → load → init round-trip.
        const std::string matSavePath = "/tmp/bdd005_material_roundtrip.ysim.json";
        sim.saveScene(matSavePath);
        resetScene();
        sim.loadScene(matSavePath);
        sim.initialize();   // auto-applyPendingMaterials writes mesh.material (D-025)

        auto* m0_postLoad = Scene<Backend, Precision>::findById(matMeshId);
        if (!m0_postLoad) {
            fail("BDD-005 / material round-trips through saveScene/loadScene",
                 "cube disappeared after load");
        } else if (!matEqual(m0_postLoad->material, edited)) {
            fail("BDD-005 / material round-trips through saveScene/loadScene",
                 "post-load material differs from saved edit");
        } else {
            pass("BDD-005 / material round-trips through saveScene/loadScene");
        }

        // Phase 4: addCube + sim.initialize() forces re-pack; setMaterial
        // edit must survive (D-025 sister mechanization for material).
        sim.addCube(tinym::vec3(5.0f, 0.0f, 0.0f), /*tess=*/2,
                    /*size=*/0.5f, /*mass=*/0.1f);
        sim.initialize();
        auto* m0_postRepack = Scene<Backend, Precision>::findById(matMeshId);
        if (!m0_postRepack) {
            fail("BDD-005 / material survives Scene::pack rebuild",
                 "cube disappeared after re-pack");
        } else if (!matEqual(m0_postRepack->material, edited)) {
            fail("BDD-005 / material survives Scene::pack rebuild",
                 "post-repack material differs — pendingMaterials write-back broken?");
        } else {
            pass("BDD-005 / material survives Scene::pack rebuild");
        }
    }
}
```

4 pass labels. Self-test count grows 36 → 40.

### 4. Matrix row promotion + Estimator turn-20 NOTE fold-in

**`docs/TEST_MATRIX.md` BDD-005 row**: status `pending → warning`.
Test address: `src/main.cpp::runSelfTest` Block 20 — four clauses:
setMaterial writes / survives sim step / round-trips save+load /
survives Scene::pack rebuild. Note: PBR-preview-shader clause from
TESTS.md#BDD-005 is parked; data-layer subset shipped this slice.

**`docs/TEST_MATRIX.md` BDD-017 row** (line 31): drop the
`Per-mesh BVH objTrees cleared between scenes to bypass the
build-time skip for Float meshes (see CM-008).` sentence and
replace with: `Per-mesh BVH cross-scene cache invalidated via
D-026's lifetimeId gate (CM-008 graduated).` Single-line edit.

### 5. Bookkeeping (slice's own)

- `docs/DECISIONS.md` — D-027: file/function/decision (Shape B mutator
  + pendingMaterials write-back) / alternatives-considered (Shape A
  direct mutation) / rationale.
- `.agent/PROJECT_STATE.md` — "In flight" pointer → this slice.
  Add shipped entry for CM-008 production-side fix (commits `668dc4f`
  + `de83164`, D-026). Update Standing feature candidates list
  (drop material editing UI data-layer subset; keep PBR preview
  shader as the remaining FR-005 work).
- `.agent/CURRENT_WORK.md` / `RESUME.md` — update for the slice;
  RESUME drops "Material editing UI (FR-005 / BDD-005)" from the
  carry-forward list, replaces with "PBR preview shader slice (FR-005
  renderer-side clause)".

## Non-goals (this slice)

- **PBR preview shader** (BDD-005's renderer-side "Then" clause).
  Parked under documented-gap convention. Matrix row stays
  `warning`, not `pass`, to reflect this.
- **Out-of-range value validation in `setMaterial`.** Clamping
  happens at the inspector slider widget range, not in the mutator.
  Save files with out-of-range hand-edits are preserved as authored.
- **Material presets / library / drag-drop / multi-select.** All
  parked; v1 is single-mesh single-edit only.
- **Behavior assignment UI (FR-006 / BDD-006).** Q2 still open.
- **Inspector ergonomics for rotation** (Euler / axis-angle).
- **Rigid body** (Q4 blocked).
- **Alembic export** (Q5 + Q6 blocked).
- **`Material` struct changes** (new fields, removed fields,
  reordering). D-005 settled the v1 subset; this slice consumes it,
  does not modify it.

## Todo

Ordered. Generator executes top-to-bottom.

1. **Branch hygiene.** Already on `feat/material-inspector-data-layer`
   (off `main` at `de83164`). Commit prefix: `add:` (this slice
   adds the `setMaterial` mutator + Block 20).

2. **Re-read the design call** above. Shape B mutator is the
   decision; the inspector calls back through `setMaterial`, not
   direct pointer mutation.

3. **Add `Simulator::setMaterial`** per §1. Adjacent to
   `translateObject` / `rotateObject` in the source order.

4. **Extend `MeshInspectorTarget`** per §2 to carry the four new
   material-field pointers + the `on_material_edit` callback. Keep
   the existing `base_color` pointer but route its edits through
   the callback too (no functional change for existing renderer
   consumers — the renderer still reads `mesh.material.baseColor`
   directly).

5. **Wire `buildSelectedMeshTarget`** at `src/main.cpp:7361` to
   populate the 4 new pointer fields + the callback that calls
   `simulator.setMaterial(id, ...)`.

6. **Add 5 ImGui widgets** to the inspector rendering site (find
   the consumer of `MeshInspectorTarget`'s `base_color` field;
   that's where the new widgets go). Per-widget pattern in §2.

7. **Author Block 20** per §3. Append after Block 19. Four pass
   labels. Use `/tmp/bdd005_material_roundtrip.ysim.json` for the
   save path (matches `/tmp` substitution pattern from BDD-019).

8. **Update `docs/TEST_MATRIX.md`** per §4: BDD-005 row promoted
   to `warning` with test address; BDD-017 row prose fixed.

9. **Run `./scripts/verify-light.sh`.** Doctest binaries should
   stay 159/159 + 1120/1120.

10. **Run `--self-test` 5+ times.** Expect **40/40 PASS**
    consistently (current 36 + 4 from Block 20).

11. **Bug-probe.** Temporarily skip the `pendingMaterials[meshId]
    = mat;` write inside `setMaterial`; confirm Block 20's Phase 4
    (re-pack survival) FAILs with the diagnostic showing material
    drift. Restore.
    Second bug-probe: temporarily flip the Phase 1 assertion's
    expected value (e.g., assert against the default material
    instead of `edited`); confirm Block 20 FAILs with the
    appropriate diagnostic. Restore. (This second probe proves the
    matEqual lambda actually compares the fields.)

12. **Add D-027 to `docs/DECISIONS.md`.** Standard format. Cite
    the symmetry with D-014/D-021 mutator shape + the D-025
    auto-applyPendingMaterials contract.

13. **Update CURRENT_WORK / RESUME / PROJECT_STATE** per §5. Drop
    "Material editing UI (FR-005 / BDD-005) data-layer" from
    RESUME's carry-forward; replace with "PBR preview shader slice
    (FR-005 render-side clause)".

14. **Stop and hand off to the Estimator.** Matrix row promoted to
    `warning` (not `pass`) — the PBR-preview clause is the parked
    gap.

## Course corrections

- **Stricter-than-spec assertions** (PLANNER.md step 7). BDD-005's
  TESTS.md "Then" wording mentions roughness specifically (changing
  0.5 → 0.1). Block 20's stricter form asserts ALL 5 fields, not
  just roughness, because the BDD Notes explicitly call out "also
  test base color, metallic, specular weight, and emission color".
  The label expansion to all 5 is the strictest meaningful form;
  asserting only roughness would let a regression silently break
  the other 4.

- **Architectural invariants applying here:**
  - **D-005** (v1 OpenPBR subset = 5 fields) — consumed; this slice
    does not re-debate or extend the subset.
  - **D-013** (xPrev parity) — not relevant; material edits do not
    touch state.x.
  - **D-014** (translateObject mutator shape) — **shape mirror.**
    setMaterial follows the same find-mesh + mutate-live +
    pending-side-table pattern.
  - **D-015** (initializer write-back for translate pack-roundtrip)
    — not applicable; material has no initializer hook (it's set
    after pack, never read at pack time). pendingMaterials is the
    sole survival mechanism, mirroring D-025.
  - **D-021** (rotateObject mutator shape) — **shape mirror.** Same
    as D-014.
  - **D-023** (refit after edit) — NOT applicable; material edits
    do not affect AABBs. setMaterial does NOT call refit().
  - **D-024** (BVH leaf as triangle test) — unchanged.
  - **D-025** (auto-applyPendingMaterials in initialize) —
    **load-bearing for re-pack survival.** Block 20's Phase 4
    (re-pack survival) works because Simulator::initialize() auto-
    calls applyPendingMaterials() which writes `mesh.material =
    pendingMaterials[id]`. Without D-025 the slice would need a
    separate write-back hook.
  - **D-026** (lifetimeId gate) — unchanged; this slice does not
    touch BVH.
  - **NEW D-027** — `Simulator::setMaterial(meshId, mat)` is the
    canonical edit-time material mutator. Writes `mesh->material`
    AND `pendingMaterials[meshId] = mat`. The pendingMaterials
    write is load-bearing for re-pack survival via D-025's
    auto-call. Inspector calls `setMaterial` on every change (no
    commit-on-release gate, unlike D-014's translate path).

- **Inspector commit timing.** D-014's translate uses
  `IsItemDeactivatedAfterEdit` to fire `translateObject` only on
  drag-release (the implicit reason: translate triggers
  broadPhase.refit() which is non-trivial work for large scenes).
  Material edits are pure CPU memcpy + map insert — cheap enough
  that per-frame firing during drag is fine. The widgets call
  `setMaterial` whenever ImGui returns true (any change event).

- **`pendingMaterials` semantic clarification.** Before this slice
  it served only `loadScene` (populated at load time, drained on
  the next `initialize()`). After this slice it ALSO serves
  edit-time, mirroring `pendingRotations`'s role under D-025.
  Document this in D-027 as the contract change.

- **Renderer-side parking is intentional.** BDD-005's "preview
  render reflects the lower roughness" clause requires a PBR
  preview shader to make roughness visually distinguishable. The
  current renderer (`mesh.material.baseColor` used in `draw`)
  reads only baseColor and ignores the other 4 fields. Updating
  the shader to consume all 5 fields with proper microfacet model
  is the PBR-preview-shader slice (out of scope here).

## What to read before writing code

- `src/main.cpp::Material` (~line 1538) — 5-field struct; do not
  modify.
- `src/main.cpp::Simulator::translateObject` (~line 4540 area) —
  template for the setMaterial mutator's shape.
- `src/main.cpp::Simulator::rotateObject` (~line 4630 area) —
  second template; both demonstrate the mutator + pending-side-table
  + auto-applyPendingMaterials flow.
- `src/main.cpp::Simulator::applyPendingMaterials` (~line 5284) —
  already writes `m.material = pendingMaterials[id]`. No change
  needed inside this function.
- `src/main.cpp::Simulator::loadScene` (~line 5258) — already
  populates `pendingMaterials[meshId]` per loaded mesh. Confirms
  the side-table is the canonical material persistence path.
- `src/main.cpp::Simulator::toSnapshot` (~line 5055) — reads
  `mesh.material` directly; setMaterial's `mesh->material = mat`
  ensures the snapshot captures the edit.
- `src/main.cpp::buildSelectedMeshTarget` (~line 7361) — where the
  inspector target is constructed; gains 4 new pointer fields +
  callback wiring.
- `include/mesh_inspector.hpp` (or wherever `MeshInspectorTarget`
  is declared) — struct gains 4 fields + callback.
- The ImGui rendering site for the inspector — find by grepping
  for `target.base_color` consumption; the 5 new widgets go
  adjacent to the existing baseColor widget.
- `docs/TESTS.md#BDD-005` — verbatim "Then" clauses are the
  authority for Block 20's assertions. Pass labels written from
  these clauses, not from the matrix-row labels.
- `docs/DECISIONS.md::D-014, D-021, D-025` — the precedent
  mutator-shape + pendingX side-table pattern this slice extends.
