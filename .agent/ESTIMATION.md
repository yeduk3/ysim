# Estimation — 2026-05-12 turn 27

Status: UPDATED

## Verdict
WARNING

## BLOCK
(none)

## WARNING
- `src/main.cpp:7959`, `src/main.cpp:7960`, `src/main.cpp:7961`, `include/program.hpp:85`, `include/program.hpp:180`, `include/program.hpp:183` — the new `skip("fbo-shader-load", ...)` branch assumes `Program::loadShader()` will return control on missing shader files, but the loader still routes failures through `linkShader()` and `printLog()`/`exit(1)`. The happy path is fine, but the documented unsupported-cwd case is not clearly skip-safe.

## NOTE
- `include/HiddenGLContext.hpp:31`, `include/HiddenGLContext.hpp:43`, `include/HiddenGLContext.hpp:51` — constructor failure paths return without `glfwTerminate()`, so early GLFW/GLEW setup failure leaves GLFW initialized until process exit. That is acceptable for the one-shot self-test, but future in-process reuse should harden the cleanup.

## Test matrix delta
- BDD-005: pass

## Verify output (summary)
`./scripts/verify.sh` configured and built successfully; both doctest suites passed (`159/159` and `1120/1120`). On this Metal-less host, `ysim --self-test` took the expected `metal-device` SKIP path, so the new Block 25 FBO test was not reached here. No failures were observed.
