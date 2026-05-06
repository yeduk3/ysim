#!/usr/bin/env bash
# Light verification gate used by the Generator (docs/roles/GENERATOR.md):
# configure + build + run unit tests. The Estimator's verify.sh additionally
# exercises the GUI/Metal binary; that is intentionally NOT done here so the
# Generator can iterate fast without a Metal device.
set -euo pipefail

cd "$(dirname "$0")/.."

cmake -B build >/dev/null
cmake --build build -j --target ysim_tests ysim_primitive_tests
./build/test/ysim_tests
./build/test/ysim_primitive_tests
