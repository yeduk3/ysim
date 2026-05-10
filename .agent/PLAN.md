# Plan — Triangle-precision click-pick (`fix/click-triangle-precision`)

> Owner: **Planner** writes; **Generator** executes; **Estimator** judges.
> Updated: 2026-05-10

## Forensic: what regressed and when

**Reported bug.** Plane (`addGround`)을 quat `(1, 2, 0, 0)` normalized로
회전시키면 (≈ 127° around X), Plane의 leaf AABB가 scene 전체를 감싸게
되어 모든 click이 Plane만 선택함. 사용자 기대: ray가 실제로 통과하는
triangle 중 가장 가까운 것을 선택.

**Root cause — latent, just exposed.** `BVH::queryClickRay`
(`src/main.cpp:3833-3855`, current numbering after D-020) writes the
**leaf AABB's** `tmin` / `tmax` into `clickRayCollisions[]`:

```cpp
RayHit hit;
if(! node.aabb.intersect(ray, hit)) return;          // line 3835
...
if(node.childA < 0) { // leaf — childB is the primitive id
    rayTraced[numTraced[0]] = {
        (Index)objid, (Index)node.childB, hit.tmin, hit.tmax};   // line 3843
    numTraced[0]++;
    return;
}
```

`hit`은 leaf의 **AABB**와 ray의 교차에서 나온 값이지 triangle과의
교차가 아님. 호출자(production callback `src/main.cpp:6996-7002`,
harness Block 14의 `pickClosest`)는 이 AABB-tmin을 기준으로
smallest-tmin object를 고름.

축이 정렬된 cube/ground에서는 leaf AABB가 triangle을 거의 딱 맞게
감싸니까 AABB-tmin ≈ triangle-tmin이 되어 표면적으로 정확. 회전된
triangle의 AABB는 실제 triangle보다 훨씬 커져서 어긋남. 회전된
Plane의 leaf AABB가 camera frustum까지 침범하면 ray의 AABB-tmin이
0 근처로 작아져서 항상 Plane이 이김.

**No specific commit "broke" click-pick.** `queryClickRay`의 leaf
write 형태는 파일 history 내내 AABB-tmin을 써왔음. 변한 건 visibility:

- **D-021** (commit `d550f82`)이 `Simulator::rotateObject`를 도입
- **D-023** (commit `9c8da75`)이 `rotateObject` 끝에 `broadPhase.refit()`을 추가

이 둘이 비축-정렬 pose를 reachable하게 만들면서 latent bug가 표면화됨.
"원래 그렇게 되어있었음"은 reachable한 scene 내에서는 사실: AABB
근사가 충분히 tight해서 click이 옳게 동작했음. tilted pose가 사용
가능해진 지금은 근사가 깨짐.

## Goal

Click-pick이 ray가 실제로 통과하는 **triangle**의 가장 가까운 것을
선택하도록 수정. 이 slice 후:

- `BVH::queryClickRay`의 leaf branch가 ray-vs-triangle (Möller–
  Trumbore)을 직접 돌려서 진짜 triangle 교차의 `t`를
  `clickRayCollisions[]`에 기록한다. AABB만 hit하고 triangle은
  miss인 leaf는 buffer에 기록되지 않음.
- 호출자(production callback, harness Block 14의 `pickClosest`)는
  **변경 없음** — 기존의 smallest-tmin walk가 그대로 정확한 답을
  돌려준다 (왜냐하면 이제 tmin이 진짜 triangle-tmin).
- Plane이 회전해서 AABB가 cube를 감싸도, click ray가 실제로
  Plane triangle을 통과하지 않으면 cube만 후보로 남는다.
- Block 17이 user-reported scenario를 그대로 재현 + 검증.

## Scope

### 1. New helper — `rayTriangleIntersect` (Möller–Trumbore)

`AABB::intersect` 근처(~`src/main.cpp:3014`)에 inline free function
으로 둔다. ~20 lines.

```cpp
// Returns true and writes outT (= ray.origin + outT * ray.dir 가 교차점)
// when the ray hits the triangle (p0, p1, p2) at outT > 0 with valid
// barycentric coords. Standard Möller–Trumbore; uses 1e-6 epsilon for
// the determinant test (smaller epsilons reject grazing cases that
// could be valid hits; this is large enough for v1's mesh sizes).
inline bool rayTriangleIntersect(const Ray& ray,
                                 const tinym::vec3& p0,
                                 const tinym::vec3& p1,
                                 const tinym::vec3& p2,
                                 float& outT) {
    const float kEps = 1e-6f;
    tinym::vec3 e1 = p1 - p0;
    tinym::vec3 e2 = p2 - p0;
    tinym::vec3 pvec = cross(ray.dir, e2);
    float det = dot(e1, pvec);
    if (std::abs(det) < kEps) return false;
    float invDet = 1.0f / det;
    tinym::vec3 tvec = ray.origin - p0;
    float u = dot(tvec, pvec) * invDet;
    if (u < 0.0f || u > 1.0f) return false;
    tinym::vec3 qvec = cross(tvec, e1);
    float v = dot(ray.dir, qvec) * invDet;
    if (v < 0.0f || u + v > 1.0f) return false;
    float t = dot(e2, qvec) * invDet;
    if (t <= 0.0f) return false;
    outT = t;
    return true;
}
```

`tinym`이 `cross`/`dot`을 노출하는지 확인 (likely yes; `vec3` ops 있음).
없으면 즉석에서 component-wise 계산.

### 2. BVH-side change — store mesh data refs + use them in leaf

`BVH<METAL, PR, BVHMODE::LINEAR, BVHPRIMITIVE::TRIANGLE>` (=
`TRI_LBVH`)에 두 개의 raw 포인터 멤버를 추가:

```cpp
const PR*    xPositions   = nullptr;  // points into mesh.state.x.ptr
const Index* facetIndices = nullptr;  // points into mesh.adjacency.facets.ptr
```

**Lifetime invariant.** state.x.ptr / facets.ptr 은 Scene::pack 단위로
안정적이고, BVH 자체도 Scene::pack 시 재빌드(`broadPhase.build(scene)`
in `Simulator::initialize`)되므로 BVH의 lifetime과 정확히 같다. D-023의
`broadPhase.refit()`은 state.x의 *값*만 다시 읽고, ptr 자체는 그대로
유지하므로 안전. 새 invariant: **TRI_LBVH가 살아있는 동안 xPositions/
facetIndices가 가리키는 mesh data가 유효해야 한다.** D-024가 명시한다.

`BVH::build(GeneralMesh& mesh)` (~`src/main.cpp:3164`)에서 이 두
포인터를 저장:

```cpp
void build(GeneralMesh<METAL, PR>& mesh) {
    xPositions   = mesh.state.x.ptr;
    facetIndices = mesh.adjacency.facets.ptr;
    build(mesh.id, mesh.state.x, mesh.adjacency.facets);
}
```

`build(int oid, ...)` overload도 같은 위치에 호환되는 wiring (or
공통 setter 호출)을 두지만, click-pick 경로에서 사용되는 path는
`build(GeneralMesh&)`이므로 거기만 챙겨도 충분. 다른 build overload는
ground-truth 비교용이거나 SCENE-level이므로 손대지 않음.

`BVH::queryClickRay(const Ray&, const BVHNode&)`의 leaf branch를
교체:

```cpp
if (node.childA < 0) {
    auto& rayTracedData = Scene<BE, PR>::rayTracedData;
    auto& rayTraced     = rayTracedData.clickRayCollisions;
    auto& numTraced     = rayTracedData.numClickRayCollisions;
    if (numTraced[0] >= rayTracedData.approxColsPerRay) return;

    // D-024: write the actual triangle-vs-ray hit, not the leaf
    // AABB intersect. Without this, a leaf AABB much larger than its
    // triangle (e.g., a rotated mesh) hijacks the smallest-tmin walk
    // even when the ray never crosses the triangle itself.
    Index triId = static_cast<Index>(node.childB);
    Index v0    = facetIndices[3 * triId + 0];
    Index v1    = facetIndices[3 * triId + 1];
    Index v2    = facetIndices[3 * triId + 2];
    tinym::vec3 p0(xPositions[3*v0+0], xPositions[3*v0+1], xPositions[3*v0+2]);
    tinym::vec3 p1(xPositions[3*v1+0], xPositions[3*v1+1], xPositions[3*v1+2]);
    tinym::vec3 p2(xPositions[3*v2+0], xPositions[3*v2+1], xPositions[3*v2+2]);

    float triT;
    if (!rayTriangleIntersect(ray, p0, p1, p2, triT)) return;

    rayTraced[numTraced[0]] = {(Index)objid, triId, triT, triT};
    numTraced[0]++;
    return;  // D-020 invariant — leaves don't recurse.
}
```

`hit.tmin` / `hit.tmax`는 더 이상 leaf write에 안 쓰이지만, AABB
filter는 그대로 (`if(! node.aabb.intersect(ray, hit)) return;`)
유지 — leaf까지 못 가는 nodes는 cheap한 AABB로 거름.

### 3. 호출자 측 변경 없음

Production mouse-callback (`src/main.cpp:6996-7002`)의 smallest-tmin
walk와 harness Block 14의 `pickClosest` lambda는 **그대로 둔다**.
이제 `tmin`이 진짜 triangle-tmin이라서 두 walk 모두 자동으로 정확.

### 4. Block 17 in `runSelfTest`

Block 16 뒤에 추가. user의 reported scenario를 그대로 재현:

```cpp
// ---- Block 17: D-024 — click-pick uses triangle-precision ranking. ----
// Reproduces the user-reported regression: a Plane rotated so its
// leaf AABBs envelop the scene must NOT steal clicks aimed at smaller
// nearby objects. queryClickRay's old leaf write used AABB-vs-ray
// tmin, which is tight enough for axis-aligned scenes but not for
// rotated triangles whose AABB grows much larger than the triangle.
// D-024 makes the leaf write the actual ray-vs-triangle hit.
{
    sim.collisionPipeline.broadPhase.objTrees.clear();
    resetScene();
    // Cube near the click target.
    sim.addCube(tinym::vec3(0.0f, 0.0f, 0.0f), /*tess=*/2,
                /*size=*/0.5f, /*mass=*/0.1f);
    // Ground below; rotated quat (1, 2, 0, 0)-normalized ≈ 127deg-X
    // so its AABB tilts to envelop the cube above it.
    sim.addGround(PlaneDirection::XZPlane, tinym::vec3(0, -1, 0),
                  /*size1D=*/4.0f);
    sim.initialize();
    sim.update();
    const Index cubeId   = 0;
    const Index groundId = 1;

    ::Quat tilted = quatNormalize(::Quat{1.0f, 2.0f, 0.0f, 0.0f});
    sim.rotateObject(groundId, tilted);

    // Click straight down at the cube.
    Ray rayClick;
    rayClick.origin = tinym::vec3(0.0f, 0.0f, 10.0f);
    rayClick.dir    = tinym::vec3(0.0f, 0.0f, -1.0f);
    Scene<Backend, Precision>::rayTracedData.numClickRayCollisions[0] = 0;
    sim.collisionPipeline.broadPhase.queryClickRay(rayClick);

    // Production-style smallest-tmin walk on the populated buffer.
    auto& rt = Scene<Backend, Precision>::rayTracedData;
    Index num = rt.numClickRayCollisions[0];
    int picked = -1;
    if (num > 0) {
        picked = static_cast<int>(rt.clickRayCollisions[0].obj);
        float bestT = rt.clickRayCollisions[0].tmin;
        for (Index i = 1; i < num; ++i) {
            if (rt.clickRayCollisions[i].tmin < bestT) {
                bestT = rt.clickRayCollisions[i].tmin;
                picked = static_cast<int>(rt.clickRayCollisions[i].obj);
            }
        }
    }

    if (picked != static_cast<int>(cubeId)) {
        fail("FR-002 / click-pick selects nearest triangle, not nearest AABB",
             "tilted ground stole the click; expected cube id=" +
             std::to_string(cubeId) + " got " + std::to_string(picked));
    } else {
        pass("FR-002 / click-pick selects nearest triangle, not nearest AABB");
    }
}
```

Pass label은 `FR-002` 태그(generic "click-pick selects an object"
의미). spec-vs-label discipline대로 BDD-017 변형이 아니라 별개의
clause로 두고, 매트릭스 row 갱신은 안 함 (BDD-017 row가 이 clause를
은유적으로 포함하긴 하지만, `triangle-precision` 자체가 별도 BDD가
아니므로 row promotion 안 함).

### 5. Bookkeeping

- **`docs/DECISIONS.md`** — new `D-024`: click-pick BVH leaf does
  ray-vs-triangle directly; mesh-data pointer invariant on
  `TRI_LBVH`; Möller–Trumbore as the canonical math helper for
  click-pick / ray-cast / future hit-test consumers.
- **`docs/mistakes/COMMON_MISTAKES.md`** — new `CM-010`: BVH
  consumer treating leaf AABB-tmin as triangle-tmin. Future BVH
  walks (shadow rays, hit-test API, pickup interactions) should
  do real primitive-vs-ray at the leaf, not just AABB.
- **`docs/TEST_MATRIX.md`** — no row promotion (Block 17 is a
  triangle-precision sister to BDD-017's mechanization). Add a
  one-line cross-reference in BDD-017's test address.
- **`.agent/CURRENT_WORK.md` / `RESUME.md`** — update for the
  slice.

## Non-goals (this slice)

- **Triangle-vs-ray for narrow-phase collision detection** — narrow
  phase already uses Metal kernel `narrow_pt_tri` (D-013). This
  slice only fixes click-pick (CPU side, separate code path).
- **Per-mesh BVH refactor.** Only TRI_LBVH gets the mesh-data
  pointers. SCENE-level BVH (object AABBs) stays unchanged — its
  leaves represent objects, not triangles, and click-pick walks
  through it via the existing `objTree.tree[0].aabb.intersect`
  filter at line 4067.
- **Optimize ray-vs-triangle.** Möller–Trumbore is fine for v1.
- **Optimize the BVH walk** to early-out when current best-t is
  smaller than node-AABB tmin. Useful for big scenes; defer.
- **Rotate pack-roundtrip closure.** Still deferred (4th slice
  carrying it now — graduates to highest-priority next-slice
  candidate).
- **CM-008 production-side fix.**
- **Inspector ergonomics for rotation.** Future slice.
- **Other matrix rows, spec edits.**
- **Resolving `Q1`, `Q2`, `Q4`, `Q5`, `Q6`, `Q7`.**

## Todo

Ordered. Generator executes top-to-bottom.

1. **Branch hygiene.** Already on `fix/refit-after-edit`'s successor
   `fix/click-triangle-precision` (the user will branch from main
   after the refit-after-edit slice merges). Commit prefix: `fix:`.

2. **Re-read** the diagnosis above and `docs/roles/PLANNER.md`
   step 7 (stricter-than-spec assertions). Block 17's pass label
   is intentionally affirmative ("selects nearest triangle, not
   nearest AABB").

3. **Add `rayTriangleIntersect` helper** near `AABB::intersect`
   (~`src/main.cpp:3014`). Möller–Trumbore. ~20 lines.

4. **Verify `tinym::vec3` exposes `cross` and `dot`.** If not,
   inline component-wise math in the helper. (Most tinym variants
   have both; quick grep before writing.)

5. **Add `xPositions` and `facetIndices` members to `TRI_LBVH`.**
   Both `const PR*` / `const Index*`, default `nullptr`.

6. **Wire `BVH::build(GeneralMesh&)` to populate the pointers.**
   `~src/main.cpp:3164`. Two-line addition before delegating to the
   `build(int oid, ...)` overload.

7. **Replace the leaf branch in `BVH::queryClickRay`.** Per §2
   above. AABB filter at the top stays; `hit.tmin/tmax` is now
   used only as a "did the ray reach this AABB?" check, not as
   the recorded value. Triangle-vs-ray result writes triangle-tmin
   to the buffer.

8. **Run `./scripts/verify-light.sh`.** Doctest binaries should
   stay 159/159 + 1120/1120.

9. **Run `--self-test` 5+ times.** Expect **34/34 PASS**
   consistently. Existing Block 14 (BDD-017) and Block 16 (D-023)
   pass labels stay verbatim — for axis-aligned cubes, the new
   triangle-tmin ≈ old AABB-tmin within FP tolerance, so existing
   assertions are unaffected. Block 17 is the new line.

10. **Bug-probe.** Temporarily revert the leaf write to use
    `hit.tmin/tmax` (the old AABB-only ranking); confirm Block 17
    FAILs with the diagnostic `tilted ground stole the click;
    expected cube id=0 got 1`. Restore.

11. **Add D-024 + CM-010.** Standard format for both. D-024
    captures the leaf triangle-vs-ray invariant + the TRI_LBVH
    mesh-pointer lifetime contract. CM-010 captures the trap
    pattern.

12. **Update CURRENT_WORK / RESUME.** Carry forward deferred
    follow-ups (rotate pack-roundtrip — graduating priority).

13. **Stop and hand off to the Estimator.** No matrix-row
    promotion, no spec edits, no scope expansion.

## Course corrections

- **Stricter-than-spec assertions** (PLANNER.md step 7).
  Block 17's "expected cube" form catches AABB-only ranking
  regressions. The bug-probe (todo 10) verifies the assertion is
  load-bearing.

- **Architectural invariants applying here** (PLANNER.md step 8):
  - **D-013** (xPrev parity) — unchanged; this slice doesn't touch
    state.x mutation paths.
  - **D-014 / D-015 / D-021 / D-023** — unchanged; the BVH consumes
    state.x through the new pointer, refit takes care of value
    updates.
  - **D-018** (`mesh.id` seed) — unrelated; this slice doesn't
    touch initializer randomness.
  - **D-019 / D-022** (Quat math) — used by `rotateObject` which
    Block 17 calls; unchanged.
  - **D-020** (BVH leaf-return) — preserved; new leaf branch still
    `return`s.
  - **NEW D-024** — TRI_LBVH stores raw `state.x.ptr` /
    `facets.ptr`. Lifetime: BVH is reborn on `Scene::pack`, same
    moment those pointers are reallocated. `refit()` re-reads
    values through the same pointer; ptr stability holds for the
    BVH's lifetime.

- **No two-stage walk needed.** The earlier draft of this plan
  proposed a `pickClosestTriangleHit` consumer-side helper
  (Shape B). User pointed out correctly that the BVH should
  produce triangle hits directly — the helper was redundant.
  Removed. The mouse callback's existing smallest-tmin walk and
  Block 14's `pickClosest` lambda both stay verbatim.

- **`hit.tmin/tmax` from `node.aabb.intersect` still used** as the
  AABB filter (cheap reject for nodes the ray doesn't enter at
  all). Just not as the recorded leaf value. The dual use is
  fine and matches how broad/narrow phase already split labor.

## What to read before writing code

- `src/main.cpp::BVH<...>::queryClickRay` (~line 3833) — the leaf
  write site to replace.
- `src/main.cpp::BVH::build` (~line 3164 for `GeneralMesh&`
  overload) — site to wire mesh-data pointers.
- `src/main.cpp::AABB::intersect` (~line 3014) — neighborhood for
  `rayTriangleIntersect`.
- `src/main.cpp` mouse callback at ~line 6996 — production
  consumer (unchanged but read to confirm).
- `src/main.cpp::runSelfTest` Block 14 (`pickClosest` lambda) and
  Block 16 (refit clauses) — confirm they stay green with the
  triangle-precision change for axis-aligned cubes.
- `include/tinym.hpp` — confirm `cross` and `dot` on `vec3`.
- `docs/DECISIONS.md::D-013, D-014, D-015, D-018, D-019, D-020,
  D-021, D-022, D-023` — recent invariants that constrain how
  this slice interacts with state.x, BVH, and rotation.
- `docs/mistakes/COMMON_MISTAKES.md::CM-009` — neighbor entry
  for the BVH-walk trap family.
