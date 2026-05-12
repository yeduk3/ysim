# Resume — FBO PBR render harness + BVH GPU-only default (D-032 + D-033)

## Must remember

- **Branch:** `feat/fbo-render-harness` (off `main` at `bb3b667`).
- **D-032 closes BDD-005's render-side standing structural WARNING** (introduced by D-028 when the PBR preview shader shipped without an FBO harness). Block 25 in `runSelfTest` is now the canonical mechanization; the matrix row promotes `warning → pass`.
- **D-033 bakes D-031's measurement into the production default.** `bottomUpHybridDepth = 30` (was 3); kernel walks to root for any realistic mesh; runtime knob unchanged. Future workload-shift requiring a different default needs a new D-NNN tying the new value to a measurement.
- **`HiddenGLContext` is a NEW class** in `include/HiddenGLContext.hpp` — parallel symbol to `YGLWindow`. Does NOT modify `YGLWindow`. Destructor destroys window but deliberately does NOT call `glfwTerminate()` (process-exit cleanup is implicit). Both classes coexist.
- **Block 25 uses metallic=1 silver-ish material**, NOT non-metallic. At metallic=0 the GGX peak at low roughness is sub-pixel-thin and the harness's max-diff stays at noise level (~4 byte values). At metallic=1 F0=baseColor amplifies specular by 20×, max-diff lands at ~100+. The metallic choice is load-bearing for the threshold > 30.
- **Camera and light collocated at `(0.7, 0.7, 3.0)` looking at origin.** This puts H ≈ V; NdotH on the +Z face peaks near 1 at the near corner — the hotspot needed for GGX's narrow peak at low roughness to register in a rasterized image. Future tweaks must preserve this geometric property.
- **`Framebuffer::attachTexture2D(int, GLint, ...)` only auto-derives format+type for `GL_RGBA32F`.** For RGBA8 the harness constructs an explicit `TextureFormat{GL_RGBA8, GL_RGBA, GL_UNSIGNED_BYTE}` and uses the `TextureFormat`-taking overload. NOT a `Framebuffer` modification — both overloads were already present.
- **Block 25 SKIPs (does not FAIL) on:**
  - GLFW/GLEW init failure (`HiddenGLContext::ok == false`).
  - Shader load failure (`Program::programID == 0`, e.g., running `--self-test` from outside `build/`).
- **Bug-probe (b) is load-bearing** — disable `setUniform("roughness", ...)` in `MeshGL::draw` → Block 25 FAILs with `max diff 0`. This is the proximate verification BDD-005's standing-WARNING was waiting on. Probe (a) — both passes at same roughness — is a convenience probe that confirms the discriminator's symmetry.
- **D-031 prose-fix landed** in the Generator-rewritten "Measured result" paragraph: HybridD2 at 500k (~673 ms) DOES beat FullCPU (~731 ms) by ~8%, both dominated by FullGPU (~353 ms) by ~2×. HybridD1 is the worst at 500k.

## Last decisions + why

- **D-032 — FBO PBR render harness with max-pixel-diff > 30 threshold.**
  - HiddenGLContext as parallel symbol over YGLWindow-extension (make-means-add-new rule).
  - Hand-built cube over Simulator-driven scene (no static-Scene state coupling; independent of prior Blocks).
  - Metallic=1 silver over non-metallic red (specular dominance ensures GGX peak registers in rasterized output).
  - Collocated camera + light over off-axis (NdotH near 1 somewhere on visible face is necessary).
  - Max-pixel-diff threshold over RMS / single-pixel / threshold-count (one number, hotspot-location-invariant).

- **D-033 — Production default `bottomUpHybridDepth` raised 3 → 30 based on D-031's chart.** Runtime knob preserved; bench harness's per-run mutation still works. Future workload shifts that warrant a different default need a new D-NNN.

## Next step you were about to take

Slice complete. Next concrete step is the **Estimator's** turn (Codex) — `./scripts/verify.sh` should exit 0 with **45/45** self-test PASS lines on the macOS dev host. On the Estimator's Linux container the top-level Metal SKIP returns 0 before Block 25 reaches, so no Block 25 execution there. Expected verdict: NOTE or WARNING. Possible items:

- (i) Block 25's "metallic=1 silver-ish" choice means the test only exercises the metallic specular branch of the PBR shader, not the dielectric branch. A future "dielectric render coverage" slice could add a parallel render-test; recorded as a possible follow-up.
- (ii) The cwd discipline (run `--self-test` from `build/`) is enforced at the file-load level (shader files must be in cwd), not by the harness itself. If a future user runs from project root, both the smoke Block 24 and the new Block 25 SKIP gracefully — but via different mechanisms (Block 24 has the bench's own SKIP path; Block 25 SKIPs on `Program::programID == 0`).
- (iii) The build-time discovery about `Framebuffer::attachTexture2D` only auto-deriving format+type for `GL_RGBA32F` is documented inline in Block 25 + in D-032 — could also be a CM-NNN entry if a future caller is likely to hit it. Generator's judgment call: deferred unless it recurs.

After this lands, planner-tracked candidates per `PROJECT_STATE.md`:

- **Source-file split slice** — user-deferred per D-031 brief; still queued.
- **Strict-D-029-column bench slice** — only if measurement-vs-noise becomes a question.
- **Inspector ergonomics for rotation** — Euler / axis-angle input per FR-004 Notes.
- **BDD-018 inspector live-edit propagation** — implementation exists; mechanization needs ImGui-side simulation.
- **Behavior assignment UI (FR-006 / BDD-006)** — Q2 still open.
- **Rigid body (FR-008)** — Q4 blocked.
- **Alembic export (FR-013)** — Q5+Q6 blocked.
- **Role-doc maintenance pass.**

See `.agent/PLAN.md` and `.agent/CURRENT_WORK.md` for full plan and progress.
