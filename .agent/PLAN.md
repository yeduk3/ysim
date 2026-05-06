# Plan — Verification & Polish Slice

> Owner: **Planner** writes; **Generator** executes; **Estimator** judges.
> Updated: 2026-05-06 (turn-2)

## Goal

Close the two persistence-slice WARNINGs from `.agent/ESTIMATION.md` (turn-2) without re-opening the slice itself:

1. The Estimator's full gate `./scripts/verify.sh` does not exist — only the Generator-grade `verify-light.sh` does. The Estimator was forced to fall back. We add the gate.
2. `SceneSnapshot::warnings` (D-009) carries non-fatal load messages, but the GUI's `sceneIOStatus` line never displays them. After a load that clamped a material, the user sees only "loaded: …" with no hint anything was adjusted. We surface the warnings.

The slice ships when both are visible: `./scripts/verify.sh` exits 0 and the user sees clamped-material warnings after a load.

## Scope

- Author `scripts/verify.sh`: the Estimator's strict gate — configure, build **every** CMake target (`ysim` + `ysim_tests`), run `ysim_tests`. Distinguish from `verify-light.sh` (which only builds the test target so the Generator can iterate without paying the GUI link cost).
- Thread `SceneSnapshot::warnings.messages` from `Simulator::loadScene` into the existing `sceneIOStatus` string in `src/main.cpp` so the I/O panel shows them. Existing channel — no new ImGui window.

Behaviors covered: tooling and a live-feedback follow-up under the existing `FR-018` ("inspector edits propagate live"); no new BDDs.

## Non-goals (this slice)

- App-level `Simulator::saveScene`/`loadScene` integration test. The Estimator's other WARNING is real, but closing it requires either a CPU-backend simulator path or a headless-Metal harness (Q-D in `docs/ARCHITECTURE.md §5`). That is its own slice; carving it into a tooling follow-up would be silent scope creep.
- Promoting the existing `scene_format.hpp` round-trip test to "BDD-015 fully verified". The matrix already says `pass`, and the JSON-layer round-trip is *part* of the BDD-015 contract — but not the whole. The Estimator can mark it `WARNING` again if they think the matrix overstates coverage; that is their decision to make, not ours to silently re-edit.
- Resolving `BDD-007/008/010/011/012` (cloth/rigid drape, collision, env forces) — those are simulation slices, not tooling.
- Resolving any of PRD Q1, Q2, Q4, Q5, Q6.

## Todo

Ordered. Generator executes top-to-bottom.

1. **Author `scripts/verify.sh`.** It must:
   - `set -euo pipefail`, `cd "$(dirname "$0")/.."`.
   - `cmake -B build` (configure; do not redirect stdout — the Estimator captures full output).
   - `cmake --build build -j` (build **all** targets, including `ysim` and the Metal kernel target).
   - `./build/test/ysim_tests` (run unit tests).
   - Mark executable. The Estimator's role doc mentions running `verify.sh`, so the file path must match.
2. **Surface load warnings in the GUI.** In `src/main.cpp` near the existing `simulator.loadScene(scenePathBuf)` call (≈ line 5350): when the load succeeds and `r.value.warnings.messages` is non-empty, append each message to `sceneIOStatus` (one per line). Keep the "loaded: …" prefix as the first line.
3. **Update `docs/TEST_MATRIX.md` if needed.** No new rows — the existing BDD-014/015/016 rows already point at `test/scene_io_test.cpp`. Touch only if the Generator finds a stale entry.
4. **Stop and hand off to the Estimator.** Do not pull the Metal-backed integration test in even if it looks small — see Non-goals above. If the warning-surface change requires a new ImGui window, stop and write the question into `CURRENT_WORK.md`; in-place into `sceneIOStatus` is the intended path.

## Course corrections

- **Persistence slice (turn-1 → turn-2):** Estimator BLOCKed on three issues (primitive schema drift, dropped rotation, unanchored import paths) plus one WARNING (silent clamping). All four were addressed in the turn-2 fix commit (`7cfc491`). The Estimator's turn-2 verdict was WARNING — slice was allowed to commit. This follow-up slice is the planner's response to the two remaining WARNINGs from that verdict; it does **not** re-open the persistence slice.
- **D-004 was superseded by D-007** mid-slice. Rotation now lives on `GeneralMesh` as `rotationQuat`. If a future slice wires rendering / sim through that field, it does **not** count as a backend-boundary violation (`BDD-103`) — the field exists, the new consumer is just reading what was already there.

## What to read before writing code

- `.agent/ESTIMATION.md` (turn-2) — the source of the two WARNINGs.
- `docs/roles/ESTIMATOR.md` step 1 — confirms `verify.sh` is the strict gate, separate from `verify-light.sh`.
- `src/main.cpp` around line 5290–5360 — where the `File > Save Scene…` / `Load Scene…` modals and `sceneIOStatus` live. Surface warnings in that string.
- `include/scene_format.hpp:LoadWarnings` — the channel D-009 added; messages are `std::vector<std::string>`.
- `scripts/verify-light.sh` — already present; the new file is its strict sibling, not a replacement.
