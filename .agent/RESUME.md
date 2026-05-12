# Resume — BDD-018 inspector live-edit propagation

## Must remember

- **Branch:** `feat/bdd-018-inspector-live-edit` (off `main` at `01092b9`).
- **Closes BDD-018** matrix row 32 (`pending → pass`) via Block 26 in `runSelfTest`. No production-side changes — pure harness slice.
- **Block 26 = three pass clauses** (material / translate / rotate); 45 → 48 self-test count. Each clause asserts the corresponding inspector callback path: counter == 1 (exactly one setter call), live in-place mutation, and `sim.update()` survival without `sim.initialize()`.
- **Mechanization shape (b)** from the slice brief: harness constructs `mesh_inspector::MeshInspectorTarget` matching the production shape at `src/main.cpp:8349-8387` EXACTLY (same field assignments, same lambda bodies that wrap `Simulator::setMaterial / translateObject / rotateObject`). Harness invokes the callbacks directly with synthetic edit values. Options (a) ImGui event synthesis and (c) parallel `InspectorBindings` struct rejected with reasoning in PLAN.md.
- **Spec substitutions documented** (PLAN.md Scope + Block 26 comment + matrix row note):
  - "color or behavior tag" → harness covers color/material + translate + rotate. **Behavior-tag clause parked under BDD-006 / Q2** (in-place behavior switching not yet wired into `MeshInspectorTarget`). Returns to scope when BDD-006 ships.
  - "visible in the very next rendered frame" → mechanized as in-place pointer value the renderer reads each frame. NO FBO render — BDD-018's load-bearing claim is on the *input* data path, not output pixels.
  - "must not require pause/resume" → mechanized as no `sim.initialize()` between callback fire and pump.
- **Stricter-than-spec per PLANNER.md step 7:** per-callback counters (`int materialCalls / translateCalls / rotateCalls = 0`) captured by reference. Assertions require exact `== 1`. Catches double-fire bugs that a loose "value propagated somewhere" assertion would miss.
- **Pointer-aliasing assertion:** `target.base_color == &mesh->material.baseColor`. Production discipline — if these don't alias, the renderer reads from a different memory than the inspector writes through. Block 26 asserts the addresses match.
- **Witness math in rotate clause is hand-computed**, NOT via D-022's `rotateVector` — avoids "use the implementation to verify the implementation" tautology. The hand-math: post-rotate state.x[0] = (pivot.x - pre_y, pivot.y + pre_x, pivot.z + pre_z) for a 90°-Z rotation around `pivot = transformPosition = (1, 2, 3)` (set by the translate clause).
- **Bug-probe (b) cascades to rotate-clause FAIL** because the rotate's pivot expectation depends on translate having run. This is intentional cross-clause coupling — Block 26's clauses chain on shared scene state by design.
- **NOT folded** per user explicit direction: turn-27 WARNING on `Program::loadShader` skip-safety + turn-27 NOTE on `HiddenGLContext` destructor cleanup. Both queued for a future small `fix/loadshader-skip-safety` slice.
- **No new D-NNN, no new CM-NNN, no new BDD/FR** — pure mechanization slice.

## Last decisions + why

- **PLAN.md option (b) chosen** over (a) ImGui event synthesis and (c) parallel `InspectorBindings` struct. Rationale: option (b) mechanizes the load-bearing seam (callback → Simulator → in-place mutation) without an ImGui-side test harness; (a) is brittle and tests the widget layer that's already user-validated; (c) is redundant since `MeshInspectorTarget` already serves.
- **Witness vertex from `state.x[0]`** captured fresh before each clause that depends on it. Float behavior pins state.x against gravity so the prior `sim.update()` doesn't drift, but the harness re-snapshots defensively in case of future integrator changes.
- **Float cube over cloth** for Block 26: Float pins state.x against gravity (force=0), so one `sim.update()` leaves state.x unchanged. Cloth would complicate the survival assertion with mass-spring drift.

## Next step you were about to take

Slice complete. Next concrete step is the **Estimator's** turn (Codex) — `./scripts/verify.sh` should exit 0 with **48/48** self-test PASS lines on the macOS dev host. On the Estimator's Linux container the top-level Metal SKIP returns 0 before Block 26 reaches; doctest binaries pass unchanged. Expected verdict: NOTE or WARNING. Possible items:

- (i) Block 26 duplicates the production `buildSelectedMeshTarget` shape (src/main.cpp:8349-8387) rather than refactoring it into a shared helper — could be a NOTE about future deduplication when source-file split lands.
- (ii) Behavior-tag inspector edit spec-substituted out (BDD-006 / Q2 blocked) — a future BDD-006 slice will need to extend Block 26 or add a parallel block to mechanize the "if the behavior changed, the next simulation step dispatches through the new behavior" clause.
- (iii) Rotate-clause witness math is hand-computed (not via `rotateVector`) — catches D-022 regressions but any error in the hand math is on the harness side, not production. Trade-off documented in PLAN.md Course corrections.
- (iv) Bug-probe (b)'s rotate-clause cascade FAIL (translate failure breaks rotate's pivot expectation) is intentional cross-clause coupling. Could be NOTEd but is by-design.

After this lands, planner-tracked candidates per `PROJECT_STATE.md`:

- **Loadshader skip-safety fix slice** — addresses turn-27 WARNING + NOTE that this slice deliberately did NOT fold.
- **Inspector ergonomics for rotation** — Euler / axis-angle input per FR-004 Notes.
- **Behavior assignment UI (FR-006 / BDD-006)** — Q2 still open; would unblock BDD-018's behavior-tag clause.
- **Rigid body (FR-008)** — Q4 blocked.
- **Alembic export (FR-013)** — Q5+Q6 blocked.
- **Source-file split slice** — still user-deferred.
- **Strict-D-029-column bench slice** — only if measurement-vs-noise becomes a question.
- **Role-doc maintenance pass.**

See `.agent/PLAN.md` and `.agent/CURRENT_WORK.md` for full plan and progress.
