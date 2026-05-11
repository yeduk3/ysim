# Resume — PBR Preview Shader Slice (D-028 lands; BDD-005 render path closed, render clause manual-test-only)

## Must remember

- **Branch:** `feat/pbr-preview-shader` (off `main` at `9b76f7b`).
- **D-028 is the canonical preview-shader semantic.** GGX-Smith microfacet + single directional light + small ambient term + 5 individual uniforms matching D-005's v1 OpenPBR subset. Implemented in `src/shader/shader.frag::pbrPreview()`; consumed via `MeshGL::draw`'s extended signature; called from `src/main.cpp:4914`. Anchors to OpenPBR — Cook-Torrance Beckmann and Lambert+specular were rejected for that reason.
- **Manual visual gate IS the slice's load-bearing assertion.** The harness has no GL context (D-011 decoupled MeshGL specifically to make `--self-test` headless), so render-clause correctness cannot be mechanized in this slice. CURRENT_WORK.md has the explicit user-facing gate instructions at the top.
- **BDD-005 row stays `warning`** in the matrix until the FBO render harness slice ships (separate slice candidate). This is a **standing structural WARNING** parallel to BDD-102's Alembic-bytes substitution — PROJECT_STATE.md records it under "What the Estimator should know" so future Estimator turns don't re-flag it as new debt.
- **Uniform name <-> Material field name parity is load-bearing.** The shader's `uniform vec3 baseColor`, `uniform float metallic`, `uniform float roughness`, `uniform float specularWeight`, `uniform vec3 emissionColor` mirror `Material::{baseColor, metallic, roughness, specularWeight, emissionColor}` exactly. Grep is the cross-reference tool. Any future Material schema change (D-005 amendment) updates BOTH ends in lockstep.
- **The `MeshGL::draw` signature change is compile-time call-site safe.** Forgetting a material field at the call site is a build error, not a silent zero — this was the load-bearing reason individual uniforms beat UBO (the UBO would have silently zero-filled missing fields).
- **`Program::setUniform` has the needed overloads** at `include/program.hpp:208` (vec3) and `:241` (float). No scope expansion was needed here, but flag for future shader slices: confirm the overload set before introducing exotic uniform types.

## Last decisions + why

- **D-028 — GGX-Smith over Cook-Torrance / Lambert+specular** (anchors to OpenPBR; only GGX provides visible roughness change). **Single directional light over point light** (DCC default; no falloff confound for a comparison preview). **Individual uniforms over UBO** (5-field count stable; compile-time call-site safety). **Manual-test mechanization over FBO pixel-diff / CPU reference BRDF** (harness has no GL context; FBO harness is its own ~150-line slice and is now tracked as a candidate; CPU reference BRDF has code-clone maintenance cost without catching real shader-side failure modes).

## Next step you were about to take

Slice implementation complete. **User must run the manual visual gate** described in CURRENT_WORK.md's top section before `/codex:rescue` is appropriate. The visual gate is the slice's load-bearing assertion; skipping it ships a shader nobody verified.

After visual gate passes:
- `/codex:rescue` for Estimator turn 22. Expected verdict: NOTE level. The Estimator should acknowledge the standing structural WARNING (BDD-005 render clause is manual-test-only) as documented gap, not new debt — PROJECT_STATE.md records this explicitly.

Standing feature candidates after this slice (per `PROJECT_STATE.md`):

- **FBO-based render harness slice** — gives `--self-test` a hidden GLFW window + offscreen framebuffer + `glReadPixels` sampling. Promotes BDD-005 row `warning → pass`. ~150–200 lines harness GL plumbing. The natural pixel-diff witness for BDD-005 is "select mesh; set material A (roughness 0.1) → sample center pixel → set material B (roughness 0.9) → sample center pixel → assert RGB differs above tolerance." With this slice's shader in place, the FBO harness has a real target.
- **Inspector ergonomics for rotation** — Euler / axis-angle input affordances per FR-004 Notes. Manual-test-only mechanization.
- **BDD-018 inspector live-edit propagation** — needs ImGui-side simulation or callable abstraction.
- **Behavior assignment UI (FR-006 / BDD-006)** — Q2 still open.
- **Rigid body slice (FR-008 / BDD-008)** — blocked on Q4. When this lands: D-014/D-015 + D-018 + D-019/D-022 + D-020 + D-021 + D-023 + D-024 + D-025 + D-026 + D-027 + D-028 all apply.
- **Alembic export slice (FR-013 / BDD-013)** — blocked on Q5 + Q6.
- **Role-doc maintenance pass** — `docs/roles/GENERATOR.md` still has stale `objTrees.clear()` gotcha entry (CM-008 graduated).

See `.agent/PLAN.md` and `.agent/CURRENT_WORK.md` for full plan and progress.
