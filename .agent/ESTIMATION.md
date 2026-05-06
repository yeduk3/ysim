# Estimation — 2026-05-06 turn-1
Status: UPDATED

## Verdict
BLOCK

## BLOCK
- Primitive schema drift: [docs/design/scene_format.md](/Users/gyu/codes/ysim/docs/design/scene_format.md#L50), [include/scene_format.hpp](/Users/gyu/codes/ysim/include/scene_format.hpp#L26), [src/main.cpp](/Users/gyu/codes/ysim/src/main.cpp#L4610), [test/scene_io_test.cpp](/Users/gyu/codes/ysim/test/scene_io_test.cpp#L133) - the published scene-format doc still defines primitive shapes as `sphere`/`cube`, but the implementation hard-codes `grid` and rejects `sphere`/`cube`. The passing tests encode the narrowed contract, so this is a spec mismatch rather than a verified interpretation of the design.
- Rotation is dropped in the app save/load path: [src/main.cpp](/Users/gyu/codes/ysim/src/main.cpp#L4628), [src/main.cpp](/Users/gyu/codes/ysim/src/main.cpp#L4720), [docs/specs/FRD.md](/Users/gyu/codes/ysim/docs/specs/FRD.md#L163) - `toSnapshot()` always writes identity rotation, and `loadScene()` never stores the loaded quaternion in any runtime field. FR-014/FR-015 and BDD-014/BDD-015 require per-object transforms, including quaternion rotation, to round-trip.
- Imported mesh paths are not resolved from the scene file directory: [src/main.cpp](/Users/gyu/codes/ysim/src/main.cpp#L4735), [docs/design/scene_format.md](/Users/gyu/codes/ysim/docs/design/scene_format.md#L58) - the loader just re-splits the JSON path string and passes it through. The design says import paths are interpreted relative to the scene file directory, so moving a scene file breaks imports instead of resolving them against the file location.

## WARNING
- [test/scene_io_test.cpp](/Users/gyu/codes/ysim/test/scene_io_test.cpp#L92), [docs/TEST_MATRIX.md](/Users/gyu/codes/ysim/docs/TEST_MATRIX.md#L28) - the doctest file only exercises `scene_format.hpp` in isolation. It never calls `Simulator::saveScene/loadScene` or the GUI menu path, so the `pass` statuses for BDD-014/015/016 overstate app-level coverage.
- [include/scene_format.hpp](/Users/gyu/codes/ysim/include/scene_format.hpp#L114) - material clamping happens silently. The design note says out-of-range values should be clamped with a warning, so the current loader behavior is slightly weaker than specified.
- [scripts/verify-light.sh](/Users/gyu/codes/ysim/scripts/verify-light.sh#L10) - the repo only provides the light gate; the full `./scripts/verify.sh` mentioned by the Estimator role is absent, so the exact required verification step could not be run.

## NOTE
- [test/scene_io_test.cpp](/Users/gyu/codes/ysim/test/scene_io_test.cpp#L198) - the BDD-015 doctest already labels its first-step check as a proxy. Once the app-level save/load path is fixed, the cheapest follow-up is a smoke test that exercises `Simulator::saveScene/loadScene` end-to-end.

## Test matrix delta
- BDD-014: pass
- BDD-015: pass
- BDD-016: pass

## Verify output (summary)
`./scripts/verify.sh` was not present, so I ran `./scripts/verify-light.sh` instead; it configured, built `ysim_tests`, and passed all 6 doctest cases / 124 assertions. I also built `ysim` successfully with `cmake --build build -j --target ysim`. The slice still blocks on the three spec mismatches above: primitive shape contract drift, lost rotation in the runtime save/load path, and import-path resolution not being anchored to the scene file directory.
