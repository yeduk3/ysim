# Design — Scene file format (persistence slice)

> Per-slice design note. Drives `BDD-014`, `BDD-015`, `BDD-016`. See `.agent/PLAN.md`.
> Updated: 2026-05-06

## Format

- **File extension:** `.ysim.json`
- **Encoding:** UTF-8 JSON, pretty-printed (2-space indent) for human-diffability
- **Library:** `nlohmann/json` (single-header, drop into `include/` like the existing `stb_image.h`)
- **Version:** integer `format_version`. v1 = `1`. The version field is **required**; missing or unequal to a supported value → load fails (`BDD-016`).

## Top-level shape

```json
{
  "format_version": 1,
  "objects": [ /* see §Objects */ ],
  "environment": {
    "gravity": [0.0, -9.81, 0.0],
    "wind":    [0.0, 0.0, 0.0]
  }
}
```

Forward-compat: top-level object **may** grow new keys. Loaders ignore unknown top-level keys to support additive evolution; this does **not** apply to `format_version` or `objects` entries (see below).

## Objects

Each entry in `objects` is an object with these required keys:

```json
{
  "id":   0,
  "name": "sphere_1",
  "source":    { /* primitive or import — see §Source */ },
  "transform": { /* see §Transform */ },
  "material":  { /* see §Material */ },
  "behavior":  { /* see §Behavior */ }
}
```

- `id` is a stable integer identifier within the file. The loader does **not** assume `id` matches the in-memory object id at load time; it builds a remap. Self-references (none in v1) would use these ids.
- `name` is for UI display only. Not unique.

## Source

Discriminated by `type`:

```json
{
  "type": "primitive",
  "shape": "grid",
  "size": 1.0,
  "tessellation": 32,
  "direction": "XZPlane",
  "mass": 0.1,
  "jiggle": false
}
```

```json
{ "type": "import", "path": "assets/teapot.obj", "scale": 1.0, "mass": 0.1 }
```

### Primitive shape — v1 reality

v1 ships a single primitive shape, `"grid"`, because the only primitive-construction path the engine has today is `MeshGridInitializer` (a tessellated quad / cloth grid). The grid-specific keys (`direction`, `mass`, `jiggle`) are part of the v1 schema; on save they are always emitted and on load they default sensibly if omitted.

`"sphere"` and `"cube"` are reserved-but-not-shipped names: the v1 loader recognises them and fails with a clear `"shape X not available in this build"` error, mirroring the way the loader treats `Rigid`/`Elastic`/`Fluid`/`Generator` behaviors. They become real shipping shapes when the authoring slice that introduces them lands (`BDD-001`).

This is the same additive-evolution rule the rest of the schema uses: the on-disk surface accepts forward-compatible names and refuses to silently downgrade them. See `DECISIONS.md` D-003 for the rationale.

### Import

- `path` is interpreted **relative to the scene file's directory**. Absolute paths are accepted but discouraged (breaks portability). The loader joins relative paths against the directory of the file it was given, so a saved scene moved alongside its assets keeps loading correctly.
- v1 supports `.obj` only for `import` — the loader inspects the file extension and rejects unsupported extensions with a clear error (does not fall through silently).
- `scale` and `mass` carry the existing `MeshFileInitializerParams` fields so save/load round-trips them. Both default if omitted.

## Transform

```json
{
  "position": [0.0, 0.0, 0.0],
  "rotation": [1.0, 0.0, 0.0, 0.0]
}
```

- `position` is a 3-vector in world space.
- `rotation` is a unit quaternion as `[w, x, y, z]`. The loader **renormalizes on read** (cheap, removes any drift introduced by hand-editing). The order is `[w, x, y, z]`, **not** `[x, y, z, w]` — fix this in the encoder/decoder, not in any consumer.

## Material

OpenPBR subset, v1 (see `PRD §3.1`, Q1 default):

```json
{
  "base_color":      [1.0, 1.0, 1.0],
  "metallic":        0.0,
  "roughness":       0.5,
  "specular_weight": 1.0,
  "emission_color":  [0.0, 0.0, 0.0]
}
```

- All keys are **required** in v1 to keep the schema unambiguous. Future versions may make keys optional with documented defaults.
- Values are clamped to OpenPBR-defined ranges on read; out-of-range values are **not silently accepted** — the loader logs a warning and clamps. (Hard reject is overkill for material values.)

## Behavior

Discriminated by `type`. v1 user-facing types: `Float`, `TriangularCloth`, `FastGridCloth`.

```json
{ "type": "Float", "params": {} }
```

```json
{
  "type": "TriangularCloth",
  "params": {
    "stretch":   1000.0,
    "shear":      500.0,
    "bend":        50.0,
    "thickness":    0.01
  }
}
```

```json
{
  "type": "FastGridCloth",
  "params": {
    "particle_num_1d":  32,
    "stretch_rest":      1.0,
    "shear_rest":        1.4142,
    "bend_rest":         2.0,
    "k_stretch":      1000.0,
    "k_shear":         500.0,
    "k_bend":           50.0,
    "thickness":         0.01
  }
}
```

Field names map 1:1 to the C++ structs in `src/main.cpp` (`ClothBehaviorParams`, `FastGridClothBehaviorParams`, `FloatBehaviorParams`), with C++ camelCase converted to JSON snake_case (`particleNum1D` → `particle_num_1d`, `kstretch` → `k_stretch`).

### Reserved-but-not-shipped types

`Rigid`, `Elastic`, `Fluid`, `Generator` are reserved enum identifiers (`ARCHITECTURE.md §4.4`). The v1 loader accepts them as **known** values but immediately fails the load with a clear "behavior X not available in this build" error — it does **not** fall through to a default. This guards against silent corruption when v1 reads a future-format file.

## Error behavior (`BDD-016`)

The loader is responsible for the following error conditions, all of which leave the in-memory scene **unchanged**:

| Condition                                       | Error message must mention            |
| ----------------------------------------------- | ------------------------------------- |
| File not found / unreadable                     | path                                  |
| Not valid JSON                                  | parse-error location                  |
| `format_version` missing                        | "missing format_version"              |
| `format_version` not an integer                 | the offending type                    |
| `format_version != 1`                           | found vs. expected                    |
| `objects[i].source.type` unknown                | the offending value, the index `i`    |
| `objects[i].source.path` unsupported extension  | the path                              |
| `objects[i].behavior.type` unknown              | the offending value, the index `i`    |
| `objects[i].behavior.type` reserved-not-shipped | "behavior X not available in this build" |
| Any required key missing                        | the missing key, the path             |

The loader does **not** attempt partial recovery. Any error → no mutation, return Result::Err with the message above.

## Save semantics

- The save function walks the in-memory `Scene<METAL, PR>` (objects, transforms, materials, behavior tags + variant params, global forces) and produces the JSON above.
- Float values are written with `std::numeric_limits<PR>::max_digits10` precision so save→load round-trip is bit-identical when `PR` is the same (`BDD-015` and `BDD-102` both lean on this).
- Quaternions are renormalized **before** writing as well as on read.

## Out of scope (this slice)

- Per-frame simulation cache — that is the export slice (`BDD-013`, Alembic).
- Embedded mesh data — imports are by-path-only in v1.
- Format migration — there is exactly one version. v1→v2 migration is the future-slice job (`PRD Q7`).
- A binary/sidecar variant — premature for v1 (no heavy data in the file).
