# PLAN — BLOCK fix-turn: axis-angle antipodal canonicalization (`feat/inspector-rotation-ergonomics`)

> Owner: **Planner** writes; **Generator** executes; **Estimator** judges.
> Updated: 2026-05-13

## Course note: previous slice's verdict

Estimator turn 30 returned **BLOCK** on the inspector rotation
ergonomics slice (D-035). Root cause: the axis-angle inverse helpers
(`quatToAxisAngle` in `src/main.cpp:~1635` and `quatWxyzToAxisAngleDeg`
in `include/MeshInspectorWindow.hpp:~61`) don't canonicalize the
`q.w < 0` antipodal case. A user-entered 360° rotation produces
`q = (-1, ~0, ~0, ~0)` from `axisAngleDegToQuatWxyz`; on the next
display pass the inverse computes `s = sqrt(1 - (-1)*(-1)) = 0` and
then `axis = wxyz[xyz] / 0 = NaN/±inf`. The garbage axis is then fed
back through the next callback, corrupting `mesh.rotationQuat` via
`Simulator::rotateObject` when the user adjusts any widget.

**BLOCK fix-turn rules apply (per GENERATOR.md):**
- Stay on the same branch `feat/inspector-rotation-ergonomics`.
- Commit prefix `fix:` (not `add:`).
- Slice closes the BLOCK; no scope expansion beyond the fix +
  regression-protection probes.
- No new D-NNN (D-035's "axis-angle output is in [0, π] / [0, 180°]"
  convention is correct as written; this fix-turn ENFORCES it —
  what shipped didn't honor the convention at the antipode).

## Goal

Canonicalize the input quaternion at the entry of both
`quatToAxisAngle` (Quat-based, `src/main.cpp`) and
`quatWxyzToAxisAngleDeg` (float-array, `include/MeshInspectorWindow.hpp`)
by negating all four components when `q.w < 0`. After the fix, every
caller sees `qw >= 0`, so the existing identity-fallback at
`angleRad < 1e-6` correctly catches both antipodes (`qw = ±1` after
canonicalization is `qw = +1` → `acos(1) = 0` → fallback fires).
Block 28's AxisAngle clause is extended with negative-w probes that
exercise the BLOCK case and assert antipodal equivalence on the
round-trip. Bug-probe re-verifies the fix is load-bearing by
temporarily reverting the canonicalization and confirming the new
probes loud-FAIL.

## Scope

**Single load-bearing change**, applied symmetrically to the two
parallel implementations:

```cpp
// At the top of both quatToAxisAngle and quatWxyzToAxisAngleDeg.
float qw = wxyz[0], qx = wxyz[1], qy = wxyz[2], qz = wxyz[3];
if (qw < 0.0f) {
    qw = -qw; qx = -qx; qy = -qy; qz = -qz;
}
// ... existing logic operates on qw/qx/qy/qz instead of wxyz[*].
```

**Why this works** (verification trace from the user's question):
- Input `q = (-1, 0, 0, 0)` → after canonicalization, `qw = +1` →
  `acos(+1) = 0` → `angleRad < 1e-6` fallback fires → `outAxis =
  (1, 0, 0)`, `outAngle = 0`. No divide-by-zero, no NaN.
- Input `q = (-cos(π/4), 0, -sin(π/4), 0)` (antipodal form of a 90°-Y
  rotation) → after canonicalization, `q = (cos(π/4), 0, sin(π/4),
  0)` → `acos(0.707...) ≈ π/4` → `angleRad ≈ π/2`, `s ≈ sin(π/4)
  ≈ 0.707`, no divide-by-zero. `outAngle ≈ 90°`, `outAxis ≈
  (0, 1, 0)`.
- Output `outAngle` is now bounded in `[0, π]` (since `qw ∈ [0, 1]`
  after canonicalization, `acos(qw) ∈ [0, π/2]`, doubled = `[0, π]`)
  — D-035's documented convention is now enforced by construction.

**NEW symbols this slice adds**: NONE. The fix is a 4-line insertion
at the top of two existing functions; D-035's documented convention
stays unchanged.

**MODIFIED symbols this slice changes in place**:

- `quatToAxisAngle` in `src/main.cpp:~1635` — antipodal
  canonicalization added at function entry; downstream logic
  unchanged.
- `quatWxyzToAxisAngleDeg` in `include/MeshInspectorWindow.hpp:~61`
  — same canonicalization, lockstep with the Quat version.
- Block 28's AxisAngle clause in `src/main.cpp::runSelfTest` —
  the existing comparison `quatComponentEqual(q, qBack, tol)` is
  replaced with antipodal-equivalent comparison
  `quatAntipodalEqual(q, qBack, tol)` (a new local lambda inside
  Block 28). The probe input array is extended with 2 negative-w
  cases: `q = (-1, 0, 0, 0)` (antipodal identity / 360° rotation)
  and `q = (-cos(π/4), 0, -sin(π/4), 0)` (antipodal 90°-Y).

**PRESERVED symbols**:

- `quatFromAxisAngle` / `quatFromEulerXYZ` / `quatToEulerXYZ` in
  `src/main.cpp` — bodies unchanged.
- `axisAngleDegToQuatWxyz` / `eulerXYZDegToQuatWxyz` /
  `quatWxyzToEulerXYZDeg` in `include/MeshInspectorWindow.hpp` —
  bodies unchanged.
- `Quat` struct, `Quat operator*`, `quatNorm`, `quatNormalize`,
  `quatConjugate`, `rotateVector` (D-019 + D-022) — unchanged.
- `mesh_inspector_gui.cpp` rotation panel + mode toggle — body
  unchanged; consumes the now-correct conversion output.
- `MeshInspectorTarget`, `MeshInspectorWindowState::rotation_input_mode`,
  `Simulator::rotateObject` — unchanged.
- All other Blocks (1–27 + Block 28's Euler clause) — unchanged.
- D-035's documented convention text in `docs/DECISIONS.md` —
  unchanged ("axis-angle output is in [0, π] / [0, 180°]" was
  always the intent; this fix enforces it).

**Antipodal-equivalence assertion semantics** (Block 28's new
comparison): two unit quaternions `a` and `b` represent the same
rotation iff `a == b` OR `a == -b` (componentwise within tolerance).
The lambda compares both:

```cpp
auto quatAntipodalEqual = [](const ::Quat& a, const ::Quat& b, float tol) {
    auto eq = [&](const ::Quat& x, const ::Quat& y) {
        return std::abs(x.w - y.w) < tol
            && std::abs(x.x - y.x) < tol
            && std::abs(x.y - y.y) < tol
            && std::abs(x.z - y.z) < tol;
    };
    return eq(a, b) || eq(a, ::Quat{-b.w, -b.x, -b.y, -b.z});
};
```

Replaces the existing positive-w-only `quatComponentEqual` inside
Block 28's AxisAngle clause. Block 28's Euler clause stays on
`quatComponentEqual` — the Euler compose path always produces
positive-w output (since `quatFromAxisAngle(axis, |angle|<π)` returns
`cos(half) > 0`), so antipodal equivalence isn't needed there.

## Non-goals

- **NO new D-NNN.** This is enforcement of D-035's existing
  convention. The DECISIONS.md text doesn't change.
- **NO change to `axisAngleDegToQuatWxyz` or `quatFromAxisAngle`.**
  The forward direction is allowed to produce negative-w outputs;
  the canonicalization happens on extraction. Symmetric and matches
  standard quaternion library convention.
- **NO change to `quatFromEulerXYZ` / `quatToEulerXYZ`.** The Euler
  path goes through `quatFromAxisAngle` internally with small
  angles, so it never produces `qw < 0` for the test inputs. The
  Euler clause assertion stays `quatComponentEqual`.
- **NO change to `mesh_inspector_gui.cpp` widget rendering.** The
  inspector simply consumes corrected output from the conversion
  helpers; no display-side logic touched.
- **NO new BDD/FR/CM.** Pure bug fix.
- **NO new self-test pass labels.** Block 28's existing AxisAngle
  clause gains new probes inside the same loop; the single PASS
  label fires if all probes (positive- AND negative-w) pass. Total
  count stays at 51.
- **NO folding of any other open NOTE.** Turn-30's NOTE on parallel
  implementations (lockstep maintenance) is a standing constraint
  that already applies; this fix-turn updates both sides in the same
  commit per that constraint. Turn-28 NOTE and turn-29 NOTE-2 also
  stay deferred.

## Todo

1. **Branch hygiene.** Already on `feat/inspector-rotation-ergonomics`
   (BLOCK fix-turn stays on the same branch per GENERATOR.md). Commit
   prefix: `fix:` (closes BLOCK).
2. **Modify `src/main.cpp::quatToAxisAngle`** (~line 1635):
    - At function entry, before the existing `clamp qw` line:
      ```cpp
      float qw = q.w, qx = q.x, qy = q.y, qz = q.z;
      if (qw < 0.0f) { qw = -qw; qx = -qx; qy = -qy; qz = -qz; }
      ```
    - Replace the existing references to `q.w`, `q.x`, `q.y`, `q.z` in
      the function body with the local `qw`, `qx`, `qy`, `qz`
      copies.
    - Comment: cite D-035's `[0, π]` convention; note the antipodal
      canonicalization makes the convention enforced.
3. **Modify `include/MeshInspectorWindow.hpp::quatWxyzToAxisAngleDeg`**
   (~line 61):
    - Same shape: copy the 4 components into locals at function
      entry; negate if `qw < 0`; use the locals for the rest of the
      function body.
    - Comment cites the same D-035 invariant.
4. **Extend Block 28's AxisAngle clause** in `src/main.cpp::runSelfTest`
   (~the section that starts with the `AxisAngleProbe` struct):
    1. Add a local `quatAntipodalEqual` lambda inside the block (right
       after the existing `quatComponentEqual` lambda).
    2. Replace the existing `quatComponentEqual(q, qBack, roundTripTol)`
       call inside the AxisAngle probe loop with
       `quatAntipodalEqual(q, qBack, roundTripTol)`.
    3. Add 2 new entries to the `aaProbes[]` array:
        - `{ tinym::vec3(1, 0, 0), 360.0f * π/180, "antipodal-form 360° around X" }`
          — but cast wisely; the input goes through `quatFromAxisAngle`,
          which produces `q ≈ (-1, 0, 0, 0)` (cos(π) = -1).
        - Alternative direct-construction probe: skip
          `quatFromAxisAngle` for the antipodal case and synthesize
          `q = (-1, 0, 0, 0)` and `q = (-cos(π/4), 0, -sin(π/4), 0)`
          directly. Build them as `::Quat{...}` literals, then run the
          existing `quatToAxisAngle` → `quatFromAxisAngle` round-trip,
          then `quatAntipodalEqual(q, qBack, ...)`.
       Generator picks the simpler shape — the direct-construction
       approach is cleaner because it ensures the antipodal form is
       actually exercised regardless of `quatFromAxisAngle`'s
       implementation choices.
    4. The identity-fallback edge check (existing) stays; verify it
       still passes after the canonicalization (it should — qw=+1
       canonicalization is a no-op).
5. **Build + verify deterministic.** `cmake --build build` then
   `./src/ysim --self-test` from `build/` 5 times in a row; expect
   `51/51 PASS` every time (unchanged count — the fix repairs the
   antipodal case without adding/removing PASS labels).
6. **Bug-probe the fix**:
    1. Temporarily remove the `if (qw < 0) negate` block from BOTH
       `quatToAxisAngle` and `quatWxyzToAxisAngleDeg`. Rebuild.
       `./src/ysim --self-test` → Block 28's AxisAngle clause should
       FAIL on the negative-w probes with `qBack` showing NaN/inf
       components in the diagnostic. **This is the load-bearing
       probe** — without it, we can't tell whether the canonicalization
       actually fixed anything vs the negative-w probes happening to
       round-trip via some other code path.
    2. Restore both fixes simultaneously (the parallel implementations
       are lockstep — never restore just one).
    3. Final `./src/ysim --self-test` confirms 51/51 PASS again.
7. **`verify-light.sh` cross-check.** Run from project root. Expect
   doctest `159/159 SUCCESS` + `1120/1120 SUCCESS` unchanged.
8. **Manual GUI test (user-driven, post-fix).** Before
   `/codex:rescue`: user launches `./build/src/ysim`, creates a
   cube, opens Inspector → Rotation → Axis-Angle mode, enters
   `angle = 360°`, commits. Cube should snap to identity (angle=0)
   rather than display NaN values in the axis fields. If the
   display shows reasonable values after the fix, the BLOCK is
   visibly resolved.
9. **Update `docs/DECISIONS.md` D-035 entry** with a short addendum
   paragraph at the end:
    - "Turn-30 fix: input quaternion is antipodally canonicalized at
      the entry of both `quatToAxisAngle` and
      `quatWxyzToAxisAngleDeg` (negate all four components if
      `q.w < 0`). This enforces the documented `[0, π]` / `[0, 180°]`
      output convention by construction and eliminates the
      divide-by-zero at `qw = -1` (the antipodal identity / 360°
      rotation case). Block 28's AxisAngle clause uses antipodal-
      equivalent comparison so positive- and negative-w inputs both
      round-trip cleanly."
10. **No new CM-NNN entry needed.** The antipodal-equivalence trap
    is well-known in quaternion code; D-035's enforcement-by-
    canonicalization is the standard fix. If the Estimator wants
    documentation, the addendum in D-035 covers it.
11. **No `docs/TEST_MATRIX.md` changes** — this fix doesn't touch
    any BDD.
12. **Update `.agent/CURRENT_WORK.md`** with: file in flight (none —
    fix-turn complete), how far (antipodal canonicalization landed
    in both parallel implementations + Block 28 extended with
    negative-w probes + bug-probe verified + D-035 addendum), what's
    tested (51/51 PASS deterministic + 2 negative-w probes + 1
    identity-fallback edge), what's next (manual GUI test then
    Estimator review).
13. **Update `.agent/RESUME.md`** with: must-remember (antipodal
    canonicalization at extractor entry; antipodal-equivalent
    assertion in Block 28; D-035 convention now enforced by
    construction), last decisions + why (fix-turn enforces existing
    D-035 convention; no new D-NNN), next step (Estimator review).

## Course corrections

- **BLOCK fix-turn discipline (per GENERATOR.md)**: stay on the same
  branch, commit prefix `fix:`, no scope expansion. The Estimator
  identified ONE BLOCK item (axis-angle inverse antipodal case); the
  fix-turn closes EXACTLY that. Don't bundle anything else.
- **Parallel implementations lockstep**: any change to the Quat-
  based version in `src/main.cpp` MUST mirror to the float-array
  version in `include/MeshInspectorWindow.hpp` in the same commit.
  The Estimator's turn-30 NOTE called this out as a standing
  constraint; this fix-turn is the first exercise of it. The
  Generator should grep both files to confirm the negation logic
  is byte-equivalent (modulo the `Quat` vs `float[4]` storage).
- **Antipodal-equivalent assertion is wider than positive-w
  assertion**, not narrower. It accepts everything
  `quatComponentEqual` accepts AND also accepts sign-flipped pairs.
  So the test set is widened, not relaxed. Caution: if a future
  regression silently sign-flips the output, the antipodal-equal
  comparison would still PASS — the test treats them as the same
  rotation, which is the geometric truth. If a future slice needs
  strict positive-w output (e.g., for persistence canonicalization),
  it would add its own assertion separately.
- **D-019's Hamilton convention unchanged.** `Quat operator*` is the
  load-bearing math; this fix-turn doesn't touch it.
- **`axisAngleDegToQuatWxyz` and `quatFromAxisAngle` are NOT
  changed** to produce positive-w output. The forward direction may
  produce `qw < 0` (cos(half) at angle > 180°); the extractor
  canonicalizes on read. This is the standard quaternion-library
  pattern.
- **D-035's parallel-implementation lockstep note from turn-30**:
  this fix-turn updates both `src/main.cpp` and
  `include/MeshInspectorWindow.hpp` in the same commit. Future
  changes to either side must continue to mirror until source-file
  split lands.

Expected matrix delta: none.
Expected self-test count: 51 → 51 (probes added to existing clause; no
new pass labels).
Expected verify.sh: exits 0 on macOS dev host. On Estimator's Linux
container the Metal SKIP path returns 0 before Block 28 reaches.
Expected Estimator verdict: NOTE (the BLOCK is closed; only the
existing parallel-implementation lockstep NOTE from turn-30 may
persist as a standing reminder).
