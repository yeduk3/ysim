# Estimation - 2026-05-13 turn 29

Status: UPDATED

## Verdict
NOTE

## BLOCK
- none

## WARNING
- none

## NOTE
- [include/program.hpp](/Users/gyu/codes/ysim/include/program.hpp#L298) and [docs/DECISIONS.md](/Users/gyu/codes/ysim/docs/DECISIONS.md#L341) - D-034 says `cleanUp()` resets "all shader IDs", but the implementation still only deletes/resets `programID`, `vertexShaderID`, `geomShaderID`, and `fragShaderID`. The tessellation fields are still untouched, so the documented cleanup contract is slightly stronger than the code. Low risk today because those overloads have no call sites, but it is a real footgun if they get used again.
- [include/HiddenGLContext.hpp](/Users/gyu/codes/ysim/include/HiddenGLContext.hpp#L30) - the new per-instance `glfwInitialized` flag is correct for the current sequential `Block 25 -> Block 27` usage, but overlapping `HiddenGLContext` lifetimes would still terminate GLFW globally from the first destructor. The header documents that limitation, so this is a future-use hazard rather than a present regression.

## Test matrix delta
- none

## Verify output (summary)
`./scripts/verify.sh` rebuilt successfully, both doctest executables passed (`159/159` and `1120/1120`), and the top-level self-test took the expected Metal-device skip on this Linux/container host (`[self-test SKIP] metal-device: ...`). The new Block 27 loader-contract assertion was therefore not exercised here, but the gate stayed green. The planning docs ([`.agent/CURRENT_WORK.md`](/Users/gyu/codes/ysim/.agent/CURRENT_WORK.md), [`.agent/PLAN.md`](/Users/gyu/codes/ysim/.agent/PLAN.md), [`.agent/RESUME.md`](/Users/gyu/codes/ysim/.agent/RESUME.md), [`.agent/PROJECT_STATE.md`](/Users/gyu/codes/ysim/.agent/PROJECT_STATE.md)) were updated in lock-step with the code and stayed internally consistent.
