# Conventions

> This file is **semi-auto generated**. Re-derive it from the actual code in `src/` and `test/` whenever the project's stack changes. Keep it short — anything the linter enforces does not belong here.

## Language / stack

<!-- Fill in once decided. Example:
- Runtime: Node.js 20, TypeScript 5
- Package manager: pnpm
- Test runner: vitest
- Linter / formatter: biome
-->

## What lives where

- `src/` — production code only. No throwaway scripts, no scratch files.
- `test/` — mirrors `src/` structure. One test file per source file unless behavior is split across modules, in which case mirror by behavior id.
- `scripts/` — repo automation only (verify, checkpoint, init). Not application code.

## Naming

- Files: <!-- e.g. kebab-case for modules, PascalCase for components -->
- Functions / variables: <!-- e.g. camelCase, no Hungarian prefix -->
- Test names: describe the behavior id when one exists, e.g. `BDD-014: rejects empty input`.

## Tests

- Tests are authored from `docs/TESTS.md`. Every test that maps to a behavior id should reference the id in its name or a comment so `TEST_MATRIX.md` can be reconciled by grep.
- Prefer integration over heavily-mocked unit tests at module boundaries. Mocks are reserved for external network/IO.
- A failing test is never deleted to make CI green. It is fixed, marked `WARNING` in the matrix with a reason, or reverted with the code that broke it.

## Comments

- Default to no comments. Add one only when *why* is non-obvious — a hidden constraint, a workaround for a specific bug, surprising behavior.
- Do not write comments that restate the code, reference the current task, or name callers.

## Decisions and mistakes

- A non-obvious tradeoff goes in `docs/DECISIONS.md` (numbered, with file/function and rationale).
- A recurring failure mode goes in `docs/mistakes/COMMON_MISTAKES.md` per the criteria in `docs/roles/GENERATOR.md`.

## Commits

- One slice per commit. The slice corresponds to one or more todo items in `.agent/PLAN.md`.
- Commit message body cites behavior ids touched (e.g. `Implements BDD-014, BDD-015`) so the matrix is greppable from git history.
