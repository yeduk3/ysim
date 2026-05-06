#!/usr/bin/env bash
# Strict verification gate used by the Estimator (docs/roles/ESTIMATOR.md):
# configure + build *every* CMake target (ysim, ysim_tests, MetalKernels) + run
# the unit tests. The Generator's lighter sibling is scripts/verify-light.sh,
# which only builds the test target so iteration doesn't pay the GUI link cost.
set -euo pipefail

cd "$(dirname "$0")/.."

cmake -B build
cmake --build build -j
./build/test/ysim_tests
