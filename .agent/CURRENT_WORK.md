# Current Work — Persistence Slice (feat/persistence)

- File in flight: none — slice complete; ready for Estimator.
- How far: all 7 PLAN.md todos done. nlohmann/json + doctest vendored, `include/scene_format.hpp` is the testable JSON layer, `Simulator::saveScene/loadScene` plus a `File > Save/Load Scene...` ImGui menu wired in `src/main.cpp`. `scripts/verify-light.sh` and `test/CMakeLists.txt` created.
- What's tested: 6 doctest cases / 124 assertions, all green. BDD-014, BDD-015, BDD-016 (×4) addresses recorded in `docs/TEST_MATRIX.md`.
- What's next: Estimator reviews — `verify.sh` would also run the full `ysim` binary; we've only run the build (`cmake --build build` succeeds) and the unit-test executable.
