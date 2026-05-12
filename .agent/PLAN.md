# Plan — FBO-based render harness + BVH GPU-only default (`feat/fbo-render-harness`)

> Owner: **Planner** writes; **Generator** executes; **Estimator** judges.
> Updated: 2026-05-12

## Course note: previous slice's verdict

Estimator turn 26 returned **WARNING** on the bvh-refit-bench
slice (D-031). Two doc-drift items folded into this slice as a
small Todo:

1. **D-031 measured-result prose error**: the claim "Hybrid D=1/D=2 never beat FullCPU" is contradicted by the populated CSV at 499,849 vertices (HybridD2 mean ≈ 673 ms vs FullCPU mean ≈ 731 ms). Affected files: `docs/DECISIONS.md` D-031 entry, `.agent/PROJECT_STATE.md` (the rolling summary line). The shipped/merged `.agent/CURRENT_WORK.md` is historical (this is its successor turn — the NEW CURRENT_WORK overwrites entirely).
2. **README placeholder drift**: `profiles/experiment/bvh-refit-2026-05-12/README.ko.md:110` calls `refit_bench.csv` a "header-only placeholder" but the file already has 160 rows committed.

Both are tiny prose fixes (~10 lines across 3 files). Per PLANNER.md small-WARNING folding rule, fold into this slice as Todo #16.

## Why this slice now

User's brief (verbatim):

> 우선 BVH는 GPU only bottom up combine 구조로 가자. 우선 벤치마킹이나 BVH 수정하던거는 잊고, 다음으로 시뮬레이션 엔진에서 구현해야 하는 부분을 파악해서 계획하자. 이전에 PBR까지 했었던 듯.

Three asks:

1. **Bake D-031's measured finding into the production default.** Raise `bottomUpHybridDepth` from `3` to `30` (the bench's "FullGPU" value; effectively pure-GPU walk-to-root). Closes the "default is unmeasured starting point" half of D-030's rationale (originally Estimator turn 25's WARNING).
2. **Stop the BVH/bench thread.** No new bench columns, no kernel changes, no source-file split, no `bottomUpHybrid` driver edits. Hybrid still callable; just a new default.
3. **Pick the next simulation-engine feature after PBR (D-028).** Closes the standing structural WARNING on BDD-005's render-side clause: "preview render reflects the lower roughness" was parked manual-test-only when D-028 shipped, with an FBO harness slice as the documented exit path.

## Design call

Five resolved decisions.

### (a) Headless context — hidden GLFW window

**Decision: `HiddenGLContext` (new class in `include/HiddenGLContext.hpp`).** GLFW with `GLFW_VISIBLE = GLFW_FALSE` before `glfwCreateWindow`, then GLEW init, then `glfwMakeContextCurrent`. Apple Silicon supports this (the macOS Window Server doesn't show it, but the GL context is real). Reasons:

- Same code path as production `YGLWindow` (production uses GLFW too).
- `GLFW_VISIBLE` is a one-line hint; the rest of GL init is unchanged.
- Per the make-means-add-new rule: new class next to existing `YGLWindow`, NOT a `createHidden()` factory inside `YGLWindow`. Both classes coexist.

Rejected: (i) CGLPBuffer (deprecated on macOS, deeper API churn), (ii) OSMesa (not available on Apple Silicon), (iii) headless GLFW (not supported on macOS).

### (b) FBO target — 256×256 RGBA8 + 24-bit depth renderbuffer

**Decision: reuse the existing `Framebuffer` struct from `include/framebuffer.hpp`.** Already supports `init` + `attachTexture2D(GL_RGBA8)` + `attachRenderBuffer(GL_DEPTH_COMPONENT24)` + bind/unbind. 256×256 is small enough to be fast (~65k pixels per render) but large enough that specular highlights span ~10-50 pixels — enough for the metric in (c) to discriminate.

Rejected: 1024×1024 (too slow for a regression test); 64×64 (specular hotspot under-resolved).

### (c) Discriminator metric — max per-channel byte diff between two renders

**Decision: render the same scene twice with `roughness ∈ {0.1, 0.9}`, glReadPixels each into RGBA8 byte buffers, then compute the maximum absolute byte difference across all pixel/channel positions. Assert `max_diff > 30` (~12% on 0..255).** Reasons:

- Direct signal: "preview render REFLECTS the roughness change" → at least one pixel differs perceptibly.
- Single threshold (one number), no compound parameters.
- Robust to bg constancy: backgrounds + diffuse-only pixels stay identical between the two renders, but the specular hotspot region differs by 50–150 byte values per channel between roughness 0.1 (concentrated highlight) and 0.9 (spread-out highlight). The MAX across the image is dominated by the most-changed pixel — typically 80–150 on 0..255.
- 30 is a load-bearing-but-conservative floor: noise from FP rounding is ~1–2 byte values; the specular delta is two orders of magnitude above noise. Bug-probe by rendering both passes with the SAME roughness → max diff = 0, well below 30 → FAIL with expected diagnostic.

Rejected alternatives:
- (i) Full-image RMS — gets diluted by the (mostly unchanged) bg + diffuse area; threshold harder to pick, more brittle.
- (ii) Specific-pixel sampling — couples to camera/lighting orientation; fragile under any rendering tweak.
- (iii) Threshold-pixel-count — two free parameters; max-diff with single threshold is simpler and equally discriminating.

### (d) Scene for the render test — single hand-built cube, no Simulator

**Decision: Block 25 builds a tiny cube mesh directly (8 vertex positions, 8 vertex normals, 12 triangle indices), uploads it to a fresh `MeshGL`, and renders that.** Reasons:

- No Simulator → no Scene → no `Scene<>::numMeshes` static state interaction with prior Blocks 1–24's leftover state. Block 25 is independent.
- No `Simulator::initialize` → no GL coupling concerns (D-011 lifted GL out of `mesh.initialize` precisely to make this kind of harness possible).
- The harness only needs to verify "the shader respects the roughness uniform" — that's a pure rendering claim, independent of physics.
- The cube primitive's normals are deliberately simple (one direction per face) so the specular highlight is well-defined and predictable.

Rejected: (i) call `sim.addCube` + `sim.initialize` (drags in the whole Metal/scene pipeline, irrelevant to the render claim); (ii) load an OBJ file (extra dependency on file paths from `build/` cwd).

### (e) Production default `bottomUpHybridDepth = 30` (new D-033)

**Decision: change `src/main.cpp:3181` from `int bottomUpHybridDepth = 3;` to `int bottomUpHybridDepth = 30;` and update the surrounding comment block (lines 3172–3180) to cite D-031's measurement data and link to `profiles/experiment/bvh-refit-2026-05-12/`.** Reasons:

- D-031's chart (`profiles/experiment/bvh-refit-2026-05-12/refit_chart_line.png`) shows FullGPU wins decisively at 100k (~1.8×) and 500k (~2.0×).
- At 1k–10k all four methods are noise-equivalent → no penalty for picking FullGPU as the default.
- HybridD2 marginally beats FullCPU at 500k (~673 ms vs ~731 ms; ~8% improvement), but loses to FullGPU by ~2× regardless. So HybridD2 is dominated.
- Value 30 ≥ log2 of any realistic tree depth (~21 for 1M leaves); kernel walks to root for ANY mesh.
- The `bottomUpHybridDepth` knob stays runtime-tunable — user can lower it to a hybrid value for measurement/debug. Just changes the default.

This is a new decision (D-033) because it's measurement-driven and persistent; reverting requires either new measurement data or a workload-shift argument. D-030's rationale text stays valid (the knob is still load-bearing); only the field initializer changes.

## Goal

After this slice:

- New `include/HiddenGLContext.hpp` provides offscreen GL context construction (GLFW hidden window + GLEW), self-contained ~50 lines.
- New Block 25 in `runSelfTest` exercises the PBR shader via FBO render at two roughness values; asserts max pixel diff > 30. Pass label `BDD-005 / FBO PBR render reflects roughness change`. SKIPs gracefully on GL-init or shader-load failure.
- Production `bottomUpHybridDepth` default raised from 3 → 30, recorded as D-033.
- New D-032 records the FBO harness design (HiddenGLContext + Block 25 + threshold + SKIP-vs-FAIL semantics).
- New D-033 records the depth default tuning, citing D-031's chart.
- D-031's prose error (Hybrid D=2 vs FullCPU at 500k) corrected in `docs/DECISIONS.md` D-031 entry + `.agent/PROJECT_STATE.md` rolling summary.
- `profiles/experiment/bvh-refit-2026-05-12/README.ko.md:110` "placeholder" language updated to reflect populated CSV.
- `docs/TEST_MATRIX.md` BDD-005 row promoted `warning → pass` (the standing structural WARNING introduced by D-028 is now closed).
- Self-test count 44 → 45.

## Scope

### 1. New `include/HiddenGLContext.hpp`

Header-only class. Constructor takes `width`, `height`. Body:

```cpp
struct HiddenGLContext {
    GLFWwindow* window = nullptr;
    bool ok = false;

    HiddenGLContext(int w, int h) {
        if (!glfwInit()) return;
#ifdef __APPLE__
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#endif
        glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
        window = glfwCreateWindow(w, h, "ysim-hidden", nullptr, nullptr);
        if (!window) { glfwTerminate(); return; }
        glfwMakeContextCurrent(window);
        if (glewInit() != GLEW_OK) {
            glfwDestroyWindow(window);
            glfwTerminate();
            window = nullptr;
            return;
        }
        ok = true;
    }

    ~HiddenGLContext() {
        if (window) glfwDestroyWindow(window);
        // NOTE: do NOT call glfwTerminate() here — production main() may
        // have its own YGLWindow active concurrently in a different
        // process state. For runSelfTest's one-shot usage, leaving GLFW
        // initialized at process exit is harmless.
    }

    HiddenGLContext(const HiddenGLContext&) = delete;
    HiddenGLContext& operator=(const HiddenGLContext&) = delete;
};
```

Does NOT touch `YGLWindow`. Parallel symbol; both classes remain callable.

### 2. Block 25 — FBO PBR roughness-diff smoke

Append after Block 24. Mechanization:

```cpp
// ---- Block 25: D-032 — FBO PBR render reflects roughness change. ----
// Builds a hidden GL context (GLFW + GLEW), loads the production
// shader (shader.vert/geom/frag), uploads a hand-built cube to a
// fresh MeshGL, allocates a 256x256 RGBA8 FBO with depth renderbuffer,
// then renders twice with different roughness uniforms (0.1 and 0.9).
// Asserts max per-channel byte diff between the two captures > 30.
//
// This closes BDD-005's render clause that was parked as standing
// structural WARNING when D-028 (PBR preview shader) shipped without
// an FBO harness.
//
// SKIP-safe: if GLFW/GLEW init fails or the shader can't load from
// cwd (running --self-test from outside build/), skip rather than
// FAIL — those are unsupported environments per ESTIMATOR.md.
//
// Bug-probe: render both passes at the same roughness (0.1, 0.1)
// → max diff = 0 → Block 25 FAILs with "max pixel diff 0 below
// threshold 30". Restore.
{
    HiddenGLContext glctx(256, 256);
    if (!glctx.ok) {
        skip("fbo-glfw-init",
             "glfwInit/createWindow/glewInit failed — no GL on this host");
    } else {
        Program shader;
        shader.loadShader("shader.vert", "shader.geom", "shader.frag");
        if (!shader.programID) {
            skip("fbo-shader-load",
                 "shader.vert/geom/frag not loadable from cwd");
        } else {
            // Build a hand-rolled cube directly into a MeshGL — 8 verts,
            // 24 normal-per-corner-of-face (because the shader's
            // normal interpolation needs per-face normals; lifting one
            // per vertex would smear the specular). Or simpler: use 8
            // verts + averaged corner normals, accept softer specular.
            // Generator picks; both work for the discriminator metric.
            MeshGL<CPU> mesh;
            // ... cube vertex/normal/index upload ...
            mesh.initialize(/*vertexCount=*/N, /*facetCount=*/M,
                            vertexPtr, normalPtr, indexPtr);

            // FBO setup — reuse include/framebuffer.hpp::Framebuffer.
            Framebuffer fbo;
            fbo.init(glctx.window, 256, 256);
            fbo.attachTexture2D(/*nTexture=*/1, GL_RGBA8, 256, 256);
            fbo.attachRenderBuffer(GL_DEPTH_COMPONENT24);

            // Camera / view / projection / light uniforms ---------
            shader.use();
            // Generator sets: model, view, projection, lightDir,
            // lightColor, lightIntensity, viewPos, ambient — whatever
            // the production shader currently expects (grep shader.frag
            // for `uniform` declarations).
            // ...

            auto renderAndCapture = [&](float roughness) -> std::vector<uint8_t> {
                fbo.bind();
                glViewport(0, 0, 256, 256);
                glEnable(GL_DEPTH_TEST);
                glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                mesh.draw(shader,
                          tinym::vec3(0.8f, 0.2f, 0.2f),  // baseColor: red
                          /*metallic=*/0.0f,
                          roughness,
                          /*specularWeight=*/1.0f,
                          tinym::vec3(0.0f));              // emissionColor
                glFinish();
                std::vector<uint8_t> px(256 * 256 * 4);
                glReadPixels(0, 0, 256, 256, GL_RGBA, GL_UNSIGNED_BYTE,
                             px.data());
                fbo.unbind();
                return px;
            };

            auto roughLo = renderAndCapture(0.1f);  // sharp specular
            auto roughHi = renderAndCapture(0.9f);  // broad specular

            int maxDiff = 0;
            for (size_t i = 0; i < roughLo.size(); ++i) {
                int d = std::abs((int)roughLo[i] - (int)roughHi[i]);
                if (d > maxDiff) maxDiff = d;
            }

            if (maxDiff > 30) {
                pass("BDD-005 / FBO PBR render reflects roughness change");
            } else {
                fail("BDD-005 / FBO PBR render reflects roughness change",
                     "max per-channel pixel diff " + std::to_string(maxDiff)
                     + " below threshold 30 — shader does not reflect "
                     "roughness uniform change");
            }
        }
    }
}
```

Pass label: `BDD-005 / FBO PBR render reflects roughness change`.

Self-test count: 44 → 45 (on macOS dev host with GL+Metal). On
Metal-less hosts the top-level SKIP returns before Block 25 reaches.

### 3. Production `bottomUpHybridDepth` default raised 3 → 30

**`src/main.cpp` line 3181:**

```cpp
// D-033: raised from 3 to 30 based on D-031 chart data
// (profiles/experiment/bvh-refit-2026-05-12/refit_chart_line.png):
// FullGPU wins ~1.8x at 100k vertices and ~2.0x at 500k on Apple
// Silicon Metal 3.2; hybrid values (1, 2, 3) lose to FullGPU at
// 100k+ and are noise-equivalent at smaller sizes. 30 >= log2 of
// any realistic tree depth (~21 for 1M leaves), so the kernel
// walks to root for ANY mesh. The runtime knob stays — user can
// lower to a hybrid value for measurement/debug.
int bottomUpHybridDepth = 30;
```

Comment block above the field (lines 3172–3180) updates to reference D-033 + D-031's chart.

### 4. D-032 + D-033 entries in `docs/DECISIONS.md`

Standard format. Append both at the end (after D-031).

- **D-032** — FBO render harness (HiddenGLContext + Block 25 + threshold + SKIP semantics). Alternatives considered: full-image RMS vs max-diff vs single-pixel sample vs threshold-pixel-count; CGLPBuffer vs OSMesa vs hidden GLFW window; Simulator-driven scene vs hand-built cube; YGLWindow-extension vs new HiddenGLContext (parallel-symbol rule).
- **D-033** — Production default depth raised to 30 (effectively pure GPU). Cite D-031's chart. Document that the runtime knob stays callable for hybrid measurement/debug.

### 5. `docs/TEST_MATRIX.md` BDD-005 row update

Promote `warning → pass`. Test address: Block 25 in `src/main.cpp`. Updates the standing structural WARNING the PBR slice (D-028) introduced.

### 6. Folded-in turn-26 WARNING fixes

- `docs/DECISIONS.md` D-031 entry "Measured result" paragraph (around line ~291): rewrite the "Hybrid D=1 and D=2 do NOT outperform FullCPU at any size" sentence to reflect that at 500k, HybridD2 (~673 ms) IS faster than FullCPU (~731 ms), but both are dominated by FullGPU (~353 ms) by ~2×. Adjust the tuning-recommendation sentence accordingly.
- `.agent/PROJECT_STATE.md`'s D-030/D-031 "Shipped previous slice" paragraph (around line ~47): same correction.
- `profiles/experiment/bvh-refit-2026-05-12/README.ko.md:110` "placeholder" sentence: rewrite to say the CSV is populated with the bench's 160-row sweep.

### 7. Bookkeeping

- `.agent/CURRENT_WORK.md` + `.agent/RESUME.md` — overwrite with this slice's progress.
- `.agent/PROJECT_STATE.md` — mark D-031 (bvh-refit-bench) as shipped; set in-flight pointer to this slice. Remove the "Tune-default-bottomUpHybridDepth" candidate (closed by D-033). Standing candidates that survive: source-file split (still deferred), strict-D-029-column (still on the shelf), inspector ergonomics, BDD-018 mechanization, FR-006/008/013 (blocked).

## Non-goals (this slice)

- **Multi-mesh FBO rendering.** Block 25 renders one cube. Compositing N meshes is a future slice if/when needed.
- **Wider material coverage** (metallic sweep, specularWeight sweep, baseColor diff). The render clause only requires "render reflects roughness change"; one diff is enough.
- **Camera control / animation.** Static camera + static cube + static light. No matrix tuning beyond the minimum to put the cube in view.
- **Anti-aliasing / MSAA.** Plain RGBA8 readback; pixel-exact comparison.
- **Pixel-exact reference image** committed to the repo. The harness is the discriminator; no golden image.
- **glReadPixels of depth or normal buffers.** Color only.
- **GUI integration of the harness.** `--self-test`-only.
- **Replacement of the strict D-029 path** with the partial kernel. D-029's `bottomUpBoxes` + `bottomUpBoxesGPU` stay callable (parallel-symbol rule).
- **Source-file split** (user-deferred per D-031 brief).
- **New `--bench-*` flags** (D-031 was the last for now).
- **New BDD / FR.** This slice closes coverage on existing BDD-005 + tunes the existing D-030 default.

## Todo

Ordered. Generator executes top-to-bottom.

1. **Branch hygiene.** Already on `feat/fbo-render-harness` (off `main` at `bb3b667`). Commit prefix: `add:`.

2. **Re-read the design call.** Five decisions settled; do not second-guess unless implementation surfaces a blocker (e.g., GLFW init fails inside `--self-test` because some other test already terminated GLFW — then add ordering guard, don't refactor).

3. **Author `include/HiddenGLContext.hpp`** per §1. Self-contained; depends only on GLFW + GLEW + (transitively) OpenGL headers. Header guard or `#pragma once`.

4. **Verify the cube primitive shape**: pick a vertex/normal/index layout. Simplest is the 24-vertex form (4 per face × 6 faces) so each face has its own normal — predictable specular. Generator's call; both 8-vert and 24-vert layouts produce a discriminating diff.

5. **Author Block 25** per §2. Read `src/shader/shader.frag` first to enumerate the uniforms the harness must set (camera matrices, light, view position). Set sensible defaults: camera at (0, 0, 3) looking at origin; directional light from (1, 1, 1) normalized; light color white; ambient via the shader's existing constant.

6. **Change the BVH default** per §3. One-line value change + multi-line comment update + nothing else.

7. **Add D-032 + D-033 to `docs/DECISIONS.md`** per §4.

8. **Promote BDD-005 row** in `docs/TEST_MATRIX.md` per §5. Test address `src/main.cpp::runSelfTest::Block 25`.

9. **Apply turn-26 WARNING fold-in fixes** per §6 (D-031 prose, PROJECT_STATE summary, README.ko.md line 110). Tiny.

10. **Build cleanly.** `cmake --build build`. Expect zero new warnings. Watch for GLFW/GLEW link-order issues — if Block 25 introduces a duplicate-`glewInit` linker complaint, that's a build-time discovery → small fix on-the-way (likely already linked; the production binary depends on the same).

11. **Run `./scripts/verify-light.sh`.** Doctest binaries should stay 159/159 + 1120/1120.

12. **Run `--self-test` 5+ times.** Expect **45/45 PASS** consistently (current 44 + Block 25). On a host where GLFW init fails the block SKIPs (44 PASS + 1 SKIP); the macOS dev host should hit 45/45.

13. **Bug-probe.** Three probes:
    - **(a) Render both passes at same roughness (e.g., 0.1, 0.1).** Block 25 should FAIL with "max diff 0 below threshold 30". Restore.
    - **(b) Comment out the roughness uniform setter in `mesh.draw`.** Block 25 should FAIL — the shader keeps stale roughness from the prior frame; second render is identical to first; max diff = 0. Restore. **This is the load-bearing probe** — it proves the PBR shader actually consumes the roughness uniform per-render (and that D-028's wiring is intact).
    - **(c) Set BVH default back to 3.** Self-test still passes (43 still pass, plus Block 25 unchanged because it doesn't touch BVH). Confirms the BVH knob change is value-only and independent. Restore.

14. **Run `--self-test` 5+ more times** after all probes restored. Expect 45/45 deterministic.

15. **(D-032 → D-033 sanity probe).** After the BVH default change, run the bench (`./build/src/ysim --bench-bvh-refit`) ONCE to confirm it doesn't regress the bench harness. The bench mutates `bottomUpHybridDepth` per run, so production default doesn't matter for the bench itself; this is a smoke-only sanity check. CSV gets overwritten; we don't commit the updated CSV unless the chart visibly changes (it shouldn't — same code paths, same hardware). **Generator's call: skip this if it adds friction. The Block 24 smoke covers the harness mechanism.**

16. **Apply turn-26 WARNING fold-in** if not done earlier (Todo #9). Re-verify the D-031 entry now reads consistent with the CSV.

17. **Update `CURRENT_WORK.md` / `RESUME.md`** per §7.

18. **Stop and hand off to the Estimator (Codex).**

## Course corrections

- **Stricter-than-spec assertion** (PLANNER.md step 7). Block 25's bug-probe (b) — "shader respects the roughness uniform per render" — is stronger than the literal BDD-005 wording ("preview render reflects the lower roughness"). The literal wording could be satisfied by ANY visual change (background tint, lighting shift); probe (b) verifies the change is *driven by the roughness uniform specifically*. This is the diff that caught D-028's initial under-tuning (lighting too dim before the C++-side `lightColor * lightIntensity` upload fix).

- **Architectural invariants applying here:**
  - **D-005 / D-027** (material data layer) — APPLIES; the harness sets all 5 D-005 uniforms even though only roughness varies between the two passes. Cleanest mirror of production `mesh.draw` signature.
  - **D-011** (render-state decoupled from `mesh.initialize`) — APPLIES. Block 25 can construct a `MeshGL` directly without going through Simulator.
  - **D-028** (PBR preview shader + 5 uniforms through `mesh.draw`) — APPLIES. The harness exercises the same shader path; pixel-diff is the validation. Closes the standing structural WARNING D-028 introduced.
  - **D-030** (parallel-symbol shape; runtime depth knob preserved) — UNAFFECTED by the value change. Both `bottomUpBoxes`/`bottomUpBoxesPartial` and `bottomUpBoxesGPU`/`bottomUpBoxesPartialGPU` stay callable.
  - **D-031** (bench harness, FrameProfiler-based timing) — UNAFFECTED. Bench still runs.
  - **D-032 / D-033** (introduced this slice).
  - **make-means-add-new rule** (`.claude/skills/slice/SKILL.md`) — APPLIES. The new `HiddenGLContext` class is a parallel symbol to `YGLWindow`; new Block 25 is parallel to Blocks 1–24; no modifications to `YGLWindow`, `MeshGL`, `Framebuffer`, `Program`, shader source files, or any existing Block.

- **HiddenGLContext does NOT call `glfwTerminate()` in its destructor.** The production binary's `YGLWindow` already manages GLFW lifecycle in `~YGLWindow()`. If the harness's HiddenGLContext terminated GLFW, a follow-up Block (Block 26+) re-using a HiddenGLContext would fail because GLFW is uninitialized. Single-shot use in Block 25 is enough; process-exit cleanup is implicit and harmless.

- **Self-test count discipline.** Expecting 45/45 on macOS dev host. On Linux container / Metal-less host, the top-level Metal SKIP returns 0 before Block 25 reaches, so the count there is 0 PASS + 1 SKIP (the existing pattern). No new SKIP-vs-FAIL discrimination needed.

- **Bug-probe (b) is load-bearing**, (a) and (c) are convenience probes. The Estimator should see clear bug-probe documentation for (b) in CURRENT_WORK.md.

- **Don't fix the Estimator-turn-26 WARNING prose by re-running the bench.** The bench numbers in `refit_bench.csv` are committed and authoritative; the rewrite is only in the DECISIONS.md / PROJECT_STATE.md / README.ko.md prose that *interprets* those numbers. The CSV stays unchanged. (If the user re-ran the bench, the numbers would shift by Apple Silicon thermal noise — that's a separate measurement-repeatability question outside this slice.)

## What to read before writing code

- `include/YGLWindow.hpp` — production GLFW + GLEW bring-up pattern to mirror in `HiddenGLContext`.
- `include/framebuffer.hpp::Framebuffer` — already supports the FBO setup the harness needs.
- `include/MeshGL.hpp::MeshGL::draw` (line 135) — production draw signature with 5 D-005 uniforms; Block 25 mirrors this for both renders.
- `include/program.hpp::Program::loadShader` — pattern for loading the production shader by file name (cwd is `build/` per CLAUDE.md).
- `src/shader/shader.frag` — enumerate uniforms (camera, light, view position, etc.) so Block 25 sets ALL of them, not just the 5 D-005 materials.
- `src/shader/shader.vert` + `src/shader/shader.geom` — the vert/geom side; Block 25 doesn't author shader code, just loads the existing pipeline.
- `src/main.cpp:3181` (`int bottomUpHybridDepth = 3;`) — line to change to 30.
- `src/main.cpp::runSelfTest` Blocks 22–24 — template for SKIP-vs-FAIL discipline (Block 22 N=1 path) + smoke-test discipline (Block 24).
- `docs/DECISIONS.md` D-028 entry — the PBR slice that introduced the standing structural WARNING this slice closes.
- `docs/DECISIONS.md` D-031 entry "Measured result" paragraph — text to correct per turn-26 WARNING fold-in.
- `profiles/experiment/bvh-refit-2026-05-12/README.ko.md` line 110 — placeholder text to rewrite.
- `docs/TEST_MATRIX.md` BDD-005 row — to promote `warning → pass`.
