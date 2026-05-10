# Current Work — Triangle-precision Click-pick Slice (`fix/click-triangle-precision`)

- File in flight: none — slice complete; ready for Estimator. **34/34 self-test PASS** deterministic across 5 runs. Doctest 159/159 + 1120/1120 green.
- How far: all PLAN todos done.
  - **D-024 production fix** — `BVH::queryClickRay`'s leaf branch now calls Möller–Trumbore on the actual triangle (using the existing `positions.ptr` and `primitives.ptr` slice members already populated at build time) and writes the triangle's `t` value to `clickRayCollisions[]`. The interior-node AABB filter stays (cheap reject for traversal); only the leaf's "what value does the consumer rank by" semantic changes from AABB-tmin to triangle-tmin.
  - **`rayTriangleIntersect` helper** added next to `AABB::intersect` (~line 3046). Möller–Trumbore, ~20 lines, 1e-6 epsilon for the determinant test.
  - **Production callback + Block 14 unchanged** — both still do the smallest-tmin walk; the values they read are now triangle-precise. No two-stage walk, no consumer-side helper. (The user's question about "그냥 ray를 BVH에 쿼리 시키면 원래 잘 intersection 정보가 모일텐데" was the catalyst for picking Shape A — the BVH's leaf produces the triangle hit directly.)
  - **Block 17** mechanizes a discriminating scene (size-30 ground tilted 60°-X; cube at origin; click ray from z=10): Plane AABB z-extent ±7.5 makes old AABB-tmin = 2.5 < cube's 9.75; rotated triangle plane crosses ray at t = 10.577 > cube's 9.75. Pass label `FR-002 / click-pick selects nearest triangle, not nearest AABB`.
  - **Bug-probe verified:** writing `hit.tmin/tmax` instead of `triT, triT` makes Block 17 FAIL with `tilted ground stole the click; expected cube id=0 got 1 (groundId=1)`. Restored.
- What's tested:
  - **34/34 self-test PASS** on macOS Apple Silicon, deterministic across 5 runs.
  - Doctest binaries unchanged.
  - Existing Block 14 (BDD-017) and Block 16 (D-023) pass labels stay verbatim — for axis-aligned cubes, the new triangle-tmin ≈ old AABB-tmin within FP tolerance, so existing assertions are unaffected.
  - **No matrix-row promotion** — Block 17 is a triangle-precision sister to BDD-017's mechanization; no specific BDD row tracks "click selects geometric closest triangle."
- Forensic note: the leaf-AABB-tmin write was a **latent structural bug** since the file's history — no specific commit "broke" click-pick. D-021 (rotateObject) + D-023 (refit-on-rotate) made tilted poses reachable, exposing the latent gap.
- Non-goals respected: no two-stage walk, no consumer-side helper, no BVH refactor beyond reusing existing `positions`/`primitives` slices, no rotate pack-roundtrip closure, no CM-008 production-side fix, no inspector ergonomics, no other matrix rows.
- What's next: Estimator review. Expect verdict at NOTE level — D-024 fix is one focused leaf-branch rewrite + Möller–Trumbore helper; bug-probe-verified; CM-010 documents the trap for future BVH consumers.
