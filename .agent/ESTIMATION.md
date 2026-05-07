# Estimation — 2026-05-07 turn 3
Status: UPDATED

## Verdict
BLOCK

## BLOCK
- `BDD-007 / no cloth vertex tunnels through ground`: `docs/TESTS.md:77`, `src/main.cpp:5492-5495`, `docs/TEST_MATRIX.md:21` — the new Block 6 still fails the no-tunneling clause, so the PLAN's goal to close BDD-007 is not met and the matrix row cannot be promoted to `pass`.

## WARNING
- `src/main.cpp:5414-5416` — the harness validates the BDD against a ground plane proxy instead of the spec's rigid sphere. The plan explicitly defers the literal sphere to a later rigid slice, but this remains a literal-spec gap to carry forward.

## NOTE
- `src/main.cpp:4052-4057` — the cumulative narrow-contact counter is a pragmatic harness shim; if more test paths read it later, keep the reset contract explicit so the self-test never depends on stale state.

## Test matrix delta
- BDD-007: fail

## Verify output (summary)
`./scripts/verify.sh` rebuilt `ysim`, `ysim_tests`, and `ysim_primitive_tests` successfully; both doctest binaries passed (11/11 and 9/9 cases). The `--self-test` step exited 0 with `[self-test SKIP] metal-device: MTL::CreateSystemDefaultDevice() returned null (non-macOS host or container without Metal)`, so this container did not exercise the new BDD-007 assertion path.
