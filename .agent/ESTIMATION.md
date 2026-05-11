# Estimation — 2026-05-11 turn 20

Status: UPDATED

## Verdict
WARNING

## BLOCK
- None.

## WARNING
- [docs/TEST_MATRIX.md:19](/Users/gyu/codes/ysim/docs/TEST_MATRIX.md:19) and [docs/TESTS.md:55](/Users/gyu/codes/ysim/docs/TESTS.md:55): BDD-005 is intentionally left `warning`; the data-layer slice is done, but the spec's preview-render clause remains parked pending the PBR preview shader slice.
- [.claude/skills/estimator/SKILL.md:1](/Users/gyu/codes/ysim/.claude/skills/estimator/SKILL.md:1), [.claude/skills/generator/SKILL.md:1](/Users/gyu/codes/ysim/.claude/skills/generator/SKILL.md:1), [.claude/skills/planner/SKILL.md:1](/Users/gyu/codes/ysim/.claude/skills/planner/SKILL.md:1), and [.claude/skills/slice/SKILL.md:1](/Users/gyu/codes/ysim/.claude/skills/slice/SKILL.md:1) are outside the FR-005 plan; if intentional, split them out or call them out explicitly as separate maintenance.

## NOTE
- [include/MeshInspectorWindow.hpp:36](/Users/gyu/codes/ysim/include/MeshInspectorWindow.hpp:36), [src/mesh_inspector_gui.cpp:34](/Users/gyu/codes/ysim/src/mesh_inspector_gui.cpp:34), [src/main.cpp:4677](/Users/gyu/codes/ysim/src/main.cpp:4677), [src/main.cpp:7138](/Users/gyu/codes/ysim/src/main.cpp:7138), [src/main.cpp:7512](/Users/gyu/codes/ysim/src/main.cpp:7512), and [docs/DECISIONS.md:242](/Users/gyu/codes/ysim/docs/DECISIONS.md:242) implement D-027 and the Block 20 data-layer mechanization.
- [docs/TEST_MATRIX.md:31](/Users/gyu/codes/ysim/docs/TEST_MATRIX.md:31) updates BDD-017 to the D-026 lifetimeId gate.
- [.agent/PLAN.md:1](/Users/gyu/codes/ysim/.agent/PLAN.md:1), [.agent/CURRENT_WORK.md:1](/Users/gyu/codes/ysim/.agent/CURRENT_WORK.md:1), [.agent/RESUME.md:1](/Users/gyu/codes/ysim/.agent/RESUME.md:1), and [.agent/PROJECT_STATE.md:1](/Users/gyu/codes/ysim/.agent/PROJECT_STATE.md:1) were rewritten to describe the FR-005 slice and the renderer-side follow-up.

## Test matrix delta
- BDD-005: warning
- BDD-017: pass

## Verify output (summary)
`./scripts/verify.sh` exited 0. CMake rebuilt `ysim`, `ysim_tests`, and `ysim_primitive_tests`; doctest stayed green at 11/11 test cases with 159/159 assertions and 9/9 test cases with 1120/1120 assertions. The only non-pass line was the expected `[self-test SKIP] metal-device` on this non-Metal host, which is the documented behavior here.
