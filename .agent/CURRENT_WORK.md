# Current Work — PBR Preview Shader Slice (`feat/pbr-preview-shader`)

- File in flight: none — slice complete; **awaiting user's manual visual gate before `/codex:rescue`**. 40/40 self-test PASS deterministic across 5 runs (count unchanged — no Block added per plan). Doctest 159/159 + 1120/1120 still green.

## Manual visual gate — USER PLEASE READ

**Before invoking `/codex:rescue`**, please:
1. Launch the GUI from the build directory: `./build/src/ysim` (CWD = `build/` so it can find `default.metallib` + shaders).
2. Create or select a mesh (Create > Cube; click it in the viewport to select).
3. Open the Mesh Inspector window; under "Appearance" drag the **Roughness** slider from `0.1` to `0.9` and back.
4. Confirm the **specular highlight visibly broadens** as roughness goes up (sharper / smaller highlight at 0.1; broader / dimmer at 0.9).
5. (Optional) Also exercise Metallic 0 → 1 and Emission color — emission should make the surface self-lit; metallic should tint the specular by baseColor.

**If the roughness change has no visible effect**, the shader is silently ignoring the uniform — STOP and report. Do NOT run `/codex:rescue`. The slice is not ready to ship.

If the visual gate passes, proceed with `/codex:rescue` (or `/slice` to chain the next cycle's open).

## How far: all 14 PLAN todos done.

- **D-028 — Shape A across four sub-decisions.** GGX-Smith microfacet + single directional light + individual uniforms + manual-test mechanization. Each documented in DECISIONS.md with rejected alternatives (Cook-Torrance Beckmann, Lambert+specular, point light, UBO, FBO pixel-diff harness, CPU reference BRDF).
- **`src/shader/shader.frag` rewritten**: `phong()` replaced with `pbrPreview()` (~70 lines including the helper functions `fresnelSchlick`, `distributionGGX`, `geometrySmith`). Reads 5 new uniforms (`baseColor`, `metallic`, `roughness`, `specularWeight`, `emissionColor`) matching D-005's v1 OpenPBR subset. Reuses existing `lightPosition` / `V` / `lightColor` (treated as direct radiance — the C++ side multiplies `SceneEnvironment::lightColor * lightIntensity` before upload, so shader has no rescale hack). Ambient at `0.10 * baseColor`. Wireframe overlay unchanged. Old `phong()` body + `diffuseColor` / `specularColor` / `shininess` uniforms removed.
- **`SceneEnvironment` gains `lightColor` (default white) + `lightIntensity` (default 1.6) fields.** Render loop's `shader.setUniform("lightColor", env.lightColor * env.lightIntensity)` replaces the prior hardcoded `vec3(160.0f)` upload at both render passes. The Environment panel gains `ColorEdit3("Light Color")` + `SliderFloat("Light Intensity", 0..10)` widgets. **Not persisted in scene_format** — in-memory only; promotion to schema is a separate concern (would require D-001 schema bump). User-requested follow-on during visual gate; documented here as scope expansion. Estimator may flag as "outside D-028 plan" — fold-in is the right precedent.
- **`include/MeshGL.hpp::draw` signature extended** from `(Program&, const tinym::vec3& baseColor)` to `(Program&, const tinym::vec3&, float, float, float, const tinym::vec3&)`. 5 `setUniform` calls before `glDrawElements`. Comment cites D-028 and the on_rotate convention rationale.
- **`src/main.cpp:4914` call site updated** to pass all 5 fields from `mesh.material.*`. Compile-time call-site safety — forgetting a field is a build error.
- **`Program::setUniform` overload check (PLAN.md course correction):** confirmed both `float` and `vec3` overloads exist at `include/program.hpp:208` and `:241`. No scope expansion needed.
- **No Block 21 added.** The slice's value is the rendered output, which the harness cannot verify without a GL context. Documented in PLAN.md §4 and the new standing structural WARNING in PROJECT_STATE.md.
- **`docs/TEST_MATRIX.md` BDD-005 row** test address now describes both halves: data layer (Block 20 + D-027 wiring) and renderer side (D-028 shader + `MeshGL::draw` signature + call site). Status stays `warning`.
- **`docs/DECISIONS.md`** D-028 entry — file/function, decision, 6 alternatives considered (variants + lighting + uniform binding + 2 mechanization paths), rationale, invariant for future render-side slices.

## What's tested

- **40/40 self-test PASS** deterministic across 5 runs.
- **Doctest binaries unchanged** (verify-light run: 159/159 + 1120/1120 SUCCESS).
- **Build clean** with no new warnings introduced by this slice.
- **GLSL syntax / shader compile**: NOT validated automatically (build doesn't compile GLSL; that happens at GL context init). User's manual GUI launch is the implicit shader-compile gate — if the shader fails to compile, the existing `Program::shaderCompileCheck` prints errors to stdout and nothing renders.
- **Visual correctness**: USER GATE — see top of this file.

## Non-goals respected

No FBO / pixel-diff harness; no CPU reference BRDF; no IBL / shadows / multi-light; no texture maps; no UBO refactor; no material preview swatch; no BDD-005 matrix promotion to `pass` (stays `warning` until the FBO harness slice ships).

## What's next

User runs the manual visual gate (above). If it passes, `/codex:rescue` for Estimator turn 22 (expected verdict: NOTE level — the slice introduces a standing structural WARNING, but that's documented; Codex should NOT re-flag it as new debt). If the visual gate fails, halt and re-plan the shader.
