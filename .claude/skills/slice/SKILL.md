---
name: slice
description: Run a full Planner → Generator → Estimator slice cycle for the ysim 3-role workflow. Use when the user invokes /slice. Optional args: the slice topic (e.g. "BDD-018 mechanization"). Without args, picks the top unblocked candidate from PROJECT_STATE.md's "Standing feature candidates" list.
---

You are orchestrating one full slice cycle for the ysim 3-role workflow (Planner → Generator → Estimator). Follow these steps strictly and in order. Do not skip steps. Do not invent steps.

## Conventions you must respect

- The Estimator (`Agent(codex:codex-rescue)`) MUST run via Codex, not via the local `estimator` skill — independence between Generator's model and Estimator's model is load-bearing per `docs/roles/ESTIMATOR.md`. Never call `Skill(estimator)` inside this cycle.
- The Estimator's ESTIMATION.md output is committed at the **start** of the **next** cycle (Step 2 below), not at the end of the current cycle. This matches the existing pattern in git log: `chore: estimator turn N` commits sit beside the slice commit they review.
- Do not auto-trigger a follow-up `/slice` cycle. After Step 8 (Report), stop and let the user decide whether to run `/slice` again.
- If at any step the working tree is in an unexpected state, stop and report — do not improvise destructive recovery.

## Step 1: Inspect state

Run in parallel: `git status`, `git branch --show-current`, `git log --oneline -5`, and `cat .agent/ESTIMATION.md` (if it exists).

Classify into one of these states:

- **(A) Fresh.** On `main`, working tree clean. → Skip Step 2; proceed to Step 3.
- **(B) Previous slice's Estimator done, awaiting close.** On a slice branch (`feat/*` / `fix/*` / `chore/*`); `.agent/ESTIMATION.md` is in the unstaged modifications list; ESTIMATION.md's verdict is `NOTE` or `WARNING`. → Proceed to Step 2 to commit + merge.
- **(C) Previous slice's Generator done, Estimator not yet run.** On a slice branch; Generator output files (`src/`, `docs/`, `.agent/PLAN.md`, `.agent/CURRENT_WORK.md`, `.agent/RESUME.md`) are unstaged but `.agent/ESTIMATION.md` is unchanged from main. → STOP. Tell the user: "Previous slice's Estimator hasn't run yet. Run `/codex:rescue` first, then re-invoke `/slice`."
- **(D) Previous slice's Estimator returned BLOCK.** On a slice branch; ESTIMATION.md verdict is `BLOCK`. → STOP. Tell the user: "Previous slice is BLOCKed; run `/planner` for a BLOCK fix-turn plan, then `/generator`, then `/codex:rescue` again — `/slice` does not handle BLOCK fix-turns automatically (the slice stays on the same branch with `fix:` prefix per GENERATOR.md)."
- **(E) Any other state** (uncommitted infrastructure work, mid-merge, detached HEAD, etc.). → STOP. Print the detected state and ask the user how to proceed.

## Step 2: Close the previous slice (commit + merge)

Only reached if state was (B).

1. Parse ESTIMATION.md to extract:
   - Verdict (`NOTE` or `WARNING`).
   - Turn number from the header `# Estimation — <date> turn N`.
2. Parse the current slice's `.agent/CURRENT_WORK.md` or `.agent/PLAN.md` to determine:
   - Commit prefix (`add:` for new feature, `fix:` for bug fix, `chore:` for tooling — PLAN.md "Branch hygiene" todo usually states this).
   - Slice title (short summary for the commit subject).
   - Decision ID if any (e.g. `D-027`).
3. Stage slice files (everything *except* `.agent/ESTIMATION.md`). Use explicit `git add` for the files; do NOT use `git add -A`. Typical set: `src/`, `test/`, `docs/DECISIONS.md`, `docs/TEST_MATRIX.md`, `docs/mistakes/COMMON_MISTAKES.md`, `docs/mistakes/OLD_MISTAKES.md`, `.agent/PLAN.md`, `.agent/CURRENT_WORK.md`, `.agent/RESUME.md`, `.agent/PROJECT_STATE.md`, `include/`, `scripts/`.
4. Commit with HEREDOC body (see git-commit conventions for ysim — Co-Authored-By trailer required):
   ```
   <prefix>: <slice title> (<D-NNN if any>)

   <2-4 line summary pulled from CURRENT_WORK.md "How far" bullets>

   Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
   ```
5. Stage `.agent/ESTIMATION.md` and commit:
   ```
   chore: estimator turn <N> — <verdict> on <slice-short-name> slice

   <1-2 line summary of verdict + self-test PASS count + any noteworthy NOTE/WARNING items>

   Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
   ```
6. `git checkout main && git merge --ff-only <slice-branch>`. If ff-only fails, STOP and report.

## Step 3: Pick the next slice topic

If skill args is non-empty, use it verbatim as the planner request.

Otherwise:
1. Read `.agent/PROJECT_STATE.md` "Standing feature candidates" section.
2. Identify the top unblocked candidate. Skip items containing "blocked on Q*" or "Q* still open".
3. Cross-reference `.agent/RESUME.md` "Next step you were about to take" for any folded-in NOTE/WARNING from the just-merged slice that should be bundled (small-WARNING folding rule from PLANNER.md).
4. Derive a 1–3 sentence Planner request that:
   - Names the FR/BDD/CM the slice closes.
   - Calls out any design question the Planner must resolve (e.g. "Shape A vs Shape B").
   - References folded-in items from the prior Estimator's NOTE/WARNING if any.

## Step 4: Branch

Suggest a branch name from the topic:
- New feature → `feat/<short-kebab>`
- Bug fix or production-side fix for a CM → `fix/<short-kebab>`
- Tooling / refactor / role-doc maintenance → `chore/<short-kebab>`

Confirm `git status` shows clean tree (after Step 2 merge). Then `git checkout -b <branch>`.

## Step 5: Plan

Invoke `Skill(planner)` with the topic-derived planning request as args. Wait for the planner skill to complete. The planner will author `.agent/PLAN.md` and update `.agent/PROJECT_STATE.md`. It must NOT touch `src/` or `test/`.

After it returns, briefly verify PLAN.md has the four required sections (`Goal`, `Scope`, `Non-goals`, `Todo`). If a section is missing, report and stop.

## Step 6: Generate

Invoke `Skill(generator)` with `"진행"` as args. Wait for the generator skill to complete. The generator will implement code/tests, build, run `--self-test` deterministically, bug-probe, and update `.agent/CURRENT_WORK.md` and `.agent/RESUME.md`. It must NOT write to `.agent/PLAN.md` or `.agent/PROJECT_STATE.md`.

After it returns, verify:
- `--self-test` PASS count matches what PLAN.md predicted (PLAN.md's Todo step typically says "Expect N/N PASS").
- No `BUG-PROBE` markers remain in `src/` or `test/` (run a quick grep).
- `verify-light.sh` passes (Generator should have already done this; one more run is cheap insurance).

If any verification fails, STOP and report.

## Step 7: Estimator (Codex)

Invoke `Agent` with:
- `subagent_type: "codex:codex-rescue"`
- `description: "Codex Estimator review"`
- `prompt: "--wait --fresh @docs/roles/ESTIMATOR.md 에 정의된 ESTIMATOR 역할과 규칙에 따라 git diff 내용을 읽어줘. .agent/ESTIMATION.md 를 읽은 내용의 보고서로 만들거나 덮어쓰자. 소스파일은 건드리지 말 것."`

Always use `--fresh` to start a clean Codex thread for each slice. Do NOT prompt the user about thread continuation — automation is the point.

If Codex returns an auth failure, STOP and tell the user to run `/codex:setup` (or `codex logout && codex login`).

## Step 8: Report

Read the freshly-written `.agent/ESTIMATION.md` and summarize for the user in this shape:

```
Slice complete: <topic>
- Branch: <branch-name>
- Self-test: <N>/<N> PASS deterministic
- Estimator verdict: <NOTE | WARNING | BLOCK>
- <One-line per BLOCK/WARNING/NOTE item from ESTIMATION.md>

Next action:
- NOTE/WARNING → run `/slice` again to commit-merge-this and start the next cycle.
- BLOCK → run `/planner` with the BLOCK fix-turn brief, then `/generator`, then `/codex:rescue`. Stay on this branch.
```

Stop after the report. Do NOT auto-trigger the next cycle.

## Error-handling notes

- If `git merge --ff-only` fails in Step 2, the slice branch has diverged from main. Do not force-merge. Report and stop.
- If `Skill(planner)` or `Skill(generator)` returns control with an explicit "stop and hand back" note (e.g., Planner found a spec contradiction; Generator hit a build-time discovery requiring Planner re-scope), do NOT continue to the next step. Forward the message to the user and stop.
- Self-test FAILs are a hard halt. Do not commit, do not invoke Codex — the Generator should have caught and reported this before returning, but verify in Step 6.
- Do not run `./scripts/verify.sh` from inside `/slice` — that's the Estimator's gate. `verify-light.sh` is fine for the Generator-side sanity check.

## Out of scope for this skill

- Pushing to remote (`git push`). Always stays local.
- Creating pull requests.
- Deleting old slice branches (separate housekeeping).
- BLOCK fix-turns (handled manually via `/planner` + `/generator` + `/codex:rescue` on the same slice branch).
- Picking among multiple equally-priority candidates when args are empty (auto-picks the top; user can always pass args explicitly).
