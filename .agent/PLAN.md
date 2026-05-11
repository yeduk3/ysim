# Plan — PBR preview shader (`feat/pbr-preview-shader`)

> Owner: **Planner** writes; **Generator** executes; **Estimator** judges.
> Updated: 2026-05-11

## Course note: previous slice's verdict

Estimator turn 21 returned **WARNING** on the FR-005 material-edit
data-layer slice. Two WARNINGs:

1. BDD-005 intentionally left `warning` (renderer-side clause parked).
2. Skill infrastructure changes outside the FR-005 plan.

Both already addressed: (1) is this slice's target; (2) was split out
into commit `9b76f7b` (`chore: rename Planner/Generator/Estimator
skills to lowercase + add slice composite skill`). Nothing to fold
into this slice.

## Why this slice now

BDD-005 is the only `warning` row in the matrix that has a clear
unblock path. The data-layer side shipped (D-027 / `Simulator::setMaterial`),
but the rendered preview at `src/main.cpp:4914` consumes only
`mesh.material.baseColor` — the other 4 material fields (metallic,
roughness, specularWeight, emissionColor) are silently ignored. The
fix is a fragment-shader rewrite (Phong → GGX-Smith microfacet PBR)
plus extending the `MeshGL::draw` signature to plumb all 5 parameters.

## Design call (the question that's been blocking this)

Four open questions from the slice trigger. Resolving each:

### (a) PBR variant — GGX-Smith vs Cook-Torrance Beckmann vs Lambert+specular

**Decision: GGX-Smith microfacet.** Reasons:

1. **Anchors to OpenPBR.** D-005 settled the v1 OpenPBR subset; OpenPBR
   (and the Autodesk Standard Surface family it descends from) is a
   GGX-Smith BRDF. Aligning the preview shader with the underlying
   spec means "what the user sees is what the asset will render as in
   downstream PBR engines."
2. **Roughness is visually meaningful.** Lambert+specular has no
   microfacet distribution — roughness wouldn't change the specular
   lobe shape, only its intensity. BDD-005's load-bearing assertion is
   "roughness 0.5 → 0.1 makes a visible difference in preview." Only
   GGX (or Beckmann) gives that.
3. **Cost is negligible at v1 scene scale.** GGX-Smith is ~40 GLSL
   lines, ~5 multiplies + 4 divisions per fragment per light. v1
   scenes are tens-of-thousands of triangles with one light source.
   No hot-path concern.

**Rejected: Cook-Torrance Beckmann.** Renders subtly differently from
GGX; for a preview pinned to OpenPBR's GGX choice, the mismatch is
worse than the ~15-line savings.

**Rejected: Lambert+specular.** Cannot satisfy the BDD's "visible
roughness difference" clause without faking a roughness-scaled lobe.

### (b) Lighting model — directional vs point + ambient

**Decision: single directional light + small constant ambient.**
Reasons:

1. **DCC tool default.** Blender / Maya / Houdini viewport previews
   all use a "key light from upper-front" directional setup.
2. **No falloff anomalies.** A point light at fixed world coordinates
   (the current shader's `lightPosition = (50, 50, 30)` with
   `1/dot(l,l)` falloff) means small meshes near origin get strong
   light and meshes farther out get dim light — confusing for a
   roughness preview. Directional light removes this variable.
3. **Ambient prevents pitch-black shadowed sides.** Constant
   ambient term `~0.03 * baseColor` keeps the surface readable
   without an IBL setup. No rim/fill light (out of scope; preview is
   informative, not cinematic).

**No IBL / environment / shadows.** Out of scope. Single-light direct
illumination is sufficient for the "roughness changes the highlight
size" cue that BDD-005 requires.

### (c) Uniform binding strategy — individual uniforms vs UBO

**Decision: individual uniforms.** Extend `MeshGL::draw`'s signature
from `(shader, const tinym::vec3& baseColor)` to:

```cpp
void draw(Program& shader,
          const tinym::vec3& baseColor,
          float metallic,
          float roughness,
          float specularWeight,
          const tinym::vec3& emissionColor);
```

Pass primitives, not `Material&` — same convention as D-027's
`on_material_edit` callback to avoid coupling `MeshGL.hpp` to
`main.cpp`'s `Material` type.

Reasons:

1. **Smaller surface.** 5 `setUniform` calls in `draw()` vs.
   introducing a `MaterialParams` UBO + GL_UNIFORM_BUFFER state
   machine + per-mesh UBO update plumbing.
2. **Material field count is stable.** D-005 settled at 5 fields for
   v1. UBO refactor is justified only if we later add a texture-map
   slice (FR-005 mentions per-material textures are out of v1 scope)
   or a behavior-driven shader variant.
3. **Compile-time call-site safety.** Signature change forces the
   caller (`src/main.cpp:4914`) to pass all 5 fields; forgetting one
   is a compile error, not a silent-zero default.

### (d) Mechanization — pixel-diff vs uniform-upload check vs manual-test-only

**Decision: manual-test-only + standing structural WARNING.**
The harness cannot run GL without significant infrastructure expansion
(hidden GLFW window, FBO setup, framebuffer sampling) — at minimum
~150 lines of harness GL plumbing and a new "GL-available?" SKIP
branch parallel to D-012's Metal SKIP. That's a separate slice's worth
of work and out of scope here.

The substitution path:

- **Block N in `runSelfTest`: NONE added.** The shader's correctness
  is visually verifiable, not buffer-comparable.
- **Matrix row BDD-005: stays `warning`.** Test address updated to
  describe the shader path + explicitly tag the render-side clause as
  manual-test-only. Future-Estimator should NOT re-flag this as new
  debt — it becomes a **standing structural WARNING** alongside
  BDD-102's Alembic-bytes substitution (PROJECT_STATE records this).
- **Bug-probe substitute: manual GUI test by the user.** The Generator
  should explicitly request the user run the GUI, select a mesh,
  drag the roughness slider from 0.1 → 0.9, and confirm the visible
  highlight broadens (sharper lobe at 0.1; broader, lower-amplitude
  lobe at 0.9). The Generator notes this expectation in CURRENT_WORK.md
  and RESUME.md so future readers know the visual gate exists.

Two paths were rejected for this slice (but worth noting for the
future "GL harness" slice when it lands):

- **Pixel-diff via FBO**: would require an offscreen GL window
  (`glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE)` + small framebuffer +
  `glReadPixels` at sample points). Combined with a known mesh /
  view / light setup, asserting `pixelRGB(roughness=0.1) !=
  pixelRGB(roughness=0.9)` is feasible. ~150–200 lines of harness GL
  plumbing. **Defer to its own slice.**

- **CPU-side reference BRDF in `test/`**: implement the GGX-Smith
  BRDF a second time in C++; assert sample-point evaluations agree
  with the GLSL via known input pairs. Catches algebraic bugs but
  not "shader compiles wrong / wrong uniform name / blending math
  inverted." Maintenance cost (code clone). **Defer; cheaper to add
  later if pixel-diff isn't built.**

D-028 records the choice.

## Goal

After this slice:

- `src/shader/shader.frag` has a `pbrPreview()` function replacing
  `phong()`, computing GGX-Smith microfacet shading from 5 material
  uniforms (`baseColor`, `metallic`, `roughness`, `specularWeight`,
  `emissionColor`) + a single directional light + small constant
  ambient term. Output gamma-corrected (2.2 to match existing). The
  wireframe overlay (`mix(LineColor, FragColor, mixVal)`) at the end
  stays unchanged.
- `include/MeshGL.hpp::draw` signature extended to take all 5
  primitives; uploads all 5 to the shader via `setUniform`.
- `src/main.cpp:4914` call site passes all 5 fields from
  `mesh.material.*`.
- `docs/TEST_MATRIX.md` BDD-005 row's test address updated to mention
  the shader path; status stays `warning` (with documented-gap note).
- `.agent/PROJECT_STATE.md` records the "Standing structural WARNING:
  BDD-005 render clause is manual-test-only without GL harness" so
  future Estimator turns don't re-flag.
- New D-028 records (a) GGX-Smith over alternatives, (b) directional
  light, (c) individual uniforms, (d) manual-test mechanization.

## Scope

### 1. Shader rewrite — `src/shader/shader.frag`

Replace the `phong()` function body and uniform declarations. New
uniforms:

```glsl
uniform vec3  baseColor      = vec3(1.0);
uniform float metallic       = 0.0;
uniform float roughness      = 0.5;
uniform float specularWeight = 1.0;
uniform vec3  emissionColor  = vec3(0.0);
```

Keep `lightPosition`, `V`, `lightColor` (re-use as directional light
source magnitude). Remove `diffuseColor`, `specularColor`,
`shininess` (subsumed by the new uniforms).

PBR function shape (reference, Generator may rename / refactor):

```glsl
vec3 fresnelSchlick(float cosTheta, vec3 F0) {
    return F0 + (1.0 - F0) * pow(1.0 - cosTheta, 5.0);
}
float distributionGGX(vec3 N, vec3 H, float a) {
    float a2 = a * a;
    float NdotH = max(dot(N, H), 0.0);
    float d = NdotH * NdotH * (a2 - 1.0) + 1.0;
    return a2 / (3.14159265 * d * d);
}
float geometrySmith(vec3 N, vec3 V, vec3 L, float a) {
    float k = (a + 1.0); k = (k * k) / 8.0;
    float NdotV = max(dot(N, V), 0.0);
    float NdotL = max(dot(N, L), 0.0);
    float gv = NdotV / (NdotV * (1.0 - k) + k);
    float gl = NdotL / (NdotL * (1.0 - k) + k);
    return gv * gl;
}
vec4 pbrPreview() {
    vec3 lp = V * vec4(lightPosition, 1.0);
    vec3 L = normalize(lp.xyz - GPosition.xyz / GPosition.w);
    vec3 N = normalize(GNormal); if (!gl_FrontFacing) N = -N;
    vec3 Vv = normalize(-GPosition.xyz / GPosition.w);
    vec3 H  = normalize(L + Vv);

    float a = roughness * roughness;  // OpenPBR maps perceptual roughness -> alpha^2
    vec3  F0 = mix(vec3(0.04), baseColor, metallic) * specularWeight;
    vec3  F  = fresnelSchlick(max(dot(H, Vv), 0.0), F0);
    float D  = distributionGGX(N, H, a);
    float G  = geometrySmith(N, Vv, L, a);

    vec3  numerator = D * G * F;
    float denom     = 4.0 * max(dot(N, Vv), 0.0) * max(dot(N, L), 0.0) + 1e-4;
    vec3  specular  = numerator / denom;
    vec3  kS = F;
    vec3  kD = (vec3(1.0) - kS) * (1.0 - metallic);
    float NdotL = max(dot(N, L), 0.0);

    vec3 Li = lightColor / 256.0;   // normalize the existing (160,160,160) to ~unit
    vec3 ambient = vec3(0.03) * baseColor;
    vec3 outColor = ambient + (kD * baseColor / 3.14159265 + specular) * Li * NdotL + emissionColor;

    return vec4(pow(outColor, vec3(1.0 / 2.2)), 1.0);
}
```

`main()` calls `pbrPreview()` instead of `phong()`. The wireframe
overlay mix at the end stays.

### 2. `include/MeshGL.hpp::draw` signature

```cpp
void draw(Program& shader,
          const tinym::vec3& baseColor,
          float metallic,
          float roughness,
          float specularWeight,
          const tinym::vec3& emissionColor) {
    shader.setUniform("baseColor", baseColor);
    shader.setUniform("metallic", metallic);
    shader.setUniform("roughness", roughness);
    shader.setUniform("specularWeight", specularWeight);
    shader.setUniform("emissionColor", emissionColor);
    glBindVertexArray(vao);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, facetBuffer);
    glDrawElements(GL_TRIANGLES, facetNum * 3, GL_UNSIGNED_INT, 0);
}
```

### 3. `src/main.cpp:4914` call site

```cpp
renderState.getOrCreate(mesh).draw(
    shader,
    mesh.material.baseColor,
    mesh.material.metallic,
    mesh.material.roughness,
    mesh.material.specularWeight,
    mesh.material.emissionColor);
```

### 4. Matrix row update + documented-gap recording

**`docs/TEST_MATRIX.md` BDD-005 row** (~line 19): keep status
`warning`; update test address to add the shader-side reference:

> *Existing*: `src/main.cpp::runSelfTest` Block 20 — four clauses
> PASS … Renderer-side clause … is parked …
>
> *Add*: Shader-side: `src/shader/shader.frag::pbrPreview()` consumes
> all 5 material uniforms via the extended `MeshGL::draw` signature
> (D-028). Render-clause correctness is verified manually (no harness
> frame-compare without GL context — standing structural WARNING per
> PROJECT_STATE.md).

**`.agent/PROJECT_STATE.md`** "What the Estimator should know"
section: add a bullet:

> **BDD-005's render-side clause** ("preview render reflects the
> lower roughness") is verified manually, not in the harness, because
> `--self-test` has no GL context. The data-layer subset is fully
> mechanized in Block 20 (D-027); the shader-side correctness is the
> user's visual gate. Future Estimator turns should NOT re-flag this
> as new debt — it's a standing structural WARNING parallel to
> BDD-102's Alembic-bytes substitution.

### 5. Bookkeeping (slice's own)

- `docs/DECISIONS.md` — D-028 with file/function, decision (GGX-Smith
  + directional + individual uniforms + manual-test mechanization) /
  alternatives-considered (Lambert, Cook-Torrance, UBO, pixel-diff,
  CPU-reference BRDF) / rationale.
- `.agent/PROJECT_STATE.md` — "In flight" pointer → this slice; add
  shipped entry for FR-005 data-layer (commits `a252cf3` +
  `2c33599`, D-027); add the skill-infrastructure chore commit
  (`9b76f7b`) to the chronological log. Update Standing feature
  candidates list (drop "PBR preview shader" — closed by this
  slice's data-layer-mechanizable subset; add "FBO-based render
  harness slice (for BDD-005's render-side clause and future shader
  regression tests)" as a new candidate).
- `.agent/CURRENT_WORK.md` / `RESUME.md` — drop "PBR preview shader
  slice (FR-005 renderer-side)" from RESUME's carry-forward.

## Non-goals (this slice)

- **FBO / pixel-diff harness** for BDD-005's render-side clause.
  Documented as a future-slice candidate; not addressed here.
- **CPU-side reference BRDF in `test/`.** Code-clone maintenance
  cost not justified for v1.
- **IBL / environment / shadows.** Out of v1 scope; not in BDD-005.
- **Multi-light setup.** Single directional light is sufficient.
- **Texture maps** (FR-005 PRD notes per-material textures are
  outside v1 scope).
- **UBO refactor of material params.** 5 fields stay as individual
  uniforms; refactor candidate when behavior-driven shader variants
  arrive.
- **Material preview swatch / sphere** in the inspector. The
  selected mesh is the preview surface; no separate widget.
- **Promoting BDD-005 row to `pass`.** Stays `warning` — render-clause
  correctness is not mechanized.

## Todo

Ordered. Generator executes top-to-bottom.

1. **Branch hygiene.** Already on `feat/pbr-preview-shader` (off
   `main` at `9b76f7b`). Commit prefix: `add:` (new shader feature).

2. **Re-read the design call** above. GGX-Smith + directional +
   individual uniforms + manual-test mechanization. Do not
   second-guess; if implementation reveals a blocking issue (e.g.,
   `Program::setUniform` doesn't support `float`), stop and ask.

3. **Rewrite `src/shader/shader.frag::phong()` → `pbrPreview()`**
   per §1. Keep the wireframe overlay at the end. Remove
   `diffuseColor` / `specularColor` / `shininess` uniforms. Add the
   5 new ones with sane defaults.

4. **Extend `include/MeshGL.hpp::draw` signature** per §2. The 5
   `setUniform` calls happen before `glDrawElements`. Order does
   not matter functionally; pick the order in `Material` struct for
   consistency.

5. **Update `src/main.cpp:4914`** call site to pass all 5 fields
   from `mesh.material.*` per §3.

6. **Build cleanly.** `cmake --build build`. Expect zero new
   warnings. If the GLSL compiler reports errors (uniform name
   typos, missing builtins, syntax), fix and rebuild.

7. **Run `./scripts/verify-light.sh`.** Doctest binaries should
   stay 159/159 + 1120/1120. No new tests added.

8. **Run `--self-test` 5+ times.** Expect **40/40 PASS**
   consistently (no Block N added; existing 40 pass count is
   preserved).

9. **No bug-probe in the harness.** The slice has no harness
   assertion to probe. Instead: in CURRENT_WORK.md and RESUME.md,
   record an explicit ASK to the user: "Please launch the GUI,
   select a mesh, drag the roughness slider from 0.1 to 0.9, and
   confirm the specular highlight visibly broadens. If unchanged,
   the shader is silently ignoring `roughness` and the slice should
   not ship." This is the visual gate.

10. **Update `docs/TEST_MATRIX.md`** BDD-005 row per §4 (test
    address gains shader reference; status stays `warning`).

11. **Update `.agent/PROJECT_STATE.md`** per §5 (Standing structural
    WARNING bullet; in-flight pointer; shipped log).

12. **Add D-028 to `docs/DECISIONS.md`.** Standard format.

13. **Update CURRENT_WORK / RESUME** per §5. Include the manual
    visual gate language so the user knows what to test before
    `/codex:rescue`.

14. **Stop and hand off to the Estimator.** Matrix row stays
    `warning` (intentional). Estimator should understand this is a
    standing structural WARNING, not new debt.

## Course corrections

- **No stricter-than-spec mechanization is possible here.** PLANNER.md
  step 7 prefers stricter assertions when cost is near-zero — but
  the cost of a stricter assertion (FBO harness) is high. The slice
  ships honest manual-test mechanization with the documented gap
  recorded.

- **Architectural invariants applying here:**
  - **D-005** (5-field OpenPBR subset) — consumed; no structural
    changes.
  - **D-011** (MeshGL/MeshRenderState lifted out of GeneralMesh) —
    preserved; draw path stays through MeshGL.
  - **D-013, D-014, D-015, D-018, D-019, D-020, D-021, D-022,
    D-023, D-024, D-025, D-026** — none touched. This slice is
    pure render-path.
  - **D-027** (`setMaterial` writes `mesh.material`) — load-bearing
    here. Shader reads `mesh.material.*` via the extended draw
    signature.
  - **BDD-103** (backend-boundary) — preserved. GL render path
    stays on the presentation side; no Metal kernel touches.
  - **NEW D-028** — PBR preview shader uses GGX-Smith microfacet
    + single directional light + individual uniforms. Verified
    manually (standing structural WARNING per PROJECT_STATE).

- **Standing structural WARNING introduced.** After this slice,
  BDD-005's render clause joins BDD-102's Alembic-bytes
  substitution as the second documented "harness can't verify, user
  is the gate" entry. Future Estimator turns acknowledging the
  state should not re-flag — the gap is recorded.

- **`Program::setUniform` overload check.** Confirm the helper
  supports `float` and `vec3` — the existing call uses `vec3` for
  `diffuseColor`. If `float` is missing, that's the smallest
  scope-expansion this slice tolerates: add a `void setUniform(const
  char*, float)` overload (~3 lines). If `setUniform` requires more
  surgery, stop and report to Planner.

- **Shader compilation gate.** The GLSL compiler runs at GL context
  init (in `Program::link()` or similar). If the new shader fails
  to compile, the app won't run. Generator should launch the binary
  briefly (or rely on the existing build step calling shader
  validation) to confirm compilation before declaring done. This is
  the implicit "self-test" for this slice.

## What to read before writing code

- `src/shader/shader.frag` — current Phong fragment shader; replace
  the `phong()` function.
- `src/shader/shader.vert` — vertex shader; no changes (passes
  `GPosition`, `GNormal` via geometry shader to fragment shader).
- `include/MeshGL.hpp::draw` (~line 129) — current signature
  `(Program&, const tinym::vec3& baseColor)`; extend per §2.
- `src/main.cpp:4914` — call site `renderState.getOrCreate(mesh).draw(
  shader, mesh.material.baseColor)`; update per §3.
- `include/program.hpp::Program::setUniform` — confirm `float` and
  `vec3` overloads exist; add `float` if missing.
- `docs/specs/BDD.md::BDD-005` and `docs/TESTS.md#BDD-005` — the
  authority for "preview reflects roughness" — manual visual gate.
- `docs/DECISIONS.md::D-005, D-027` — the v1 OpenPBR subset and
  the setMaterial mutator that feeds this shader's uniforms.
- The user's GUI manual test is the visual gate; document the
  expectation in CURRENT_WORK / RESUME before `/codex:rescue`.
