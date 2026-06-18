# BVH 버전 리팩터링 설계서 (arch-test)

> 이 문서는 `arch-test/` 아래에 **새 독립 엔진**을 세울 때의 BVH/broad-phase 설계 근거를 모은 OWNER'S EMPHASIS 문서다.
> 고정 전제(재논의 금지):
> - `src/main.cpp`는 **건드리지 않는다**. 새 엔진은 별도 트리(`arch-test/`)에 만든다.
> - 모든 곳에 **virtual interface**를 둔다. `IBVH` interface를 정의한다.
> - **karras12 LBVH = DEFAULT concrete**. `apetrei-agglomerative` + `subobject-multiroot` + `SpatialHashing` + `MultiLevelSpatialHashing`는 **각각 별도 concrete**.
> - `BVHFactory`가 (a) 어떤 **VERSION**을 instantiate할지, (b) build의 **PART별로 cpu-vs-gpu**를 고른다.
> - `src/metal/*.metal`은 **그대로 재사용**한다.
> - 코드 주석은 최소화하고, 근거(rationale)는 이 md에 둔다.

근거 출처: 서브시스템 맵(karras12 LBVH / scene-level BVH(TLAS)+alternative broad phases / Metal shader kernel census) + `src/main.cpp` 직접 grep으로 확인한 라인 범위. 매핑되지 않은 사실은 적지 않았다.

---

## (1) 현재 BVH 난잡함 진단

### 1.1 한 struct에 세 알고리즘이 엉켜 있다 — `BVH<BE,PR,BVHMODE::LINEAR,PRIMITIVE>` (main.cpp 5101–6931)

현재 per-object(BLAS) linear BVH는 **단일 struct** `BVH<BE, PR, BVHMODE::LINEAR, PRIMITIVE>`(5101)에 들어 있다. PRIMITIVE는 `BVHPRIMITIVE` enum 값(2=EDGE, 3=TRIANGLE; 4965–4977)으로, 같은 struct가 triangle BVH와 edge BVH 양쪽을 generate한다. 문제는 이 한 struct 안에 **서로 다른 build 알고리즘이 3개** 공존한다는 것이다.

1. **LINEAR / karras12** (Karras 2012) — **production default**.
   - hierarchy: `buildTreeGPU()` → `buildTree_Tri`/`buildTree_Edge` 커널 (5628; kernel bvh.metal:352).
   - combine: `bottomUpBoxesGPU(sceneBox)` (5418) — lock-free atomic walk-to-root.
   - CPU 레퍼런스: `buildCPU()`(6182), `fillMortonsCPU()`(5688), `radixSortCPU()`(5642), `bottomUpCombine()`(6413).

2. **APETREI 2014 agglomerative** — opt-in (`useAgglomerative`, default **false**, 5226).
   - `agglomerativeBuildGPU()`(5568) → `agglomerativeBuild_Tri`/`_Edge` 커널이 hierarchy+AABB combine을 **단일 커널로 fuse**한다.
   - post-pass `agglomerativeSwapRootGPU()`(5602) → `agglomerativeSwapRoot`로 root를 slot 0으로 옮겨 `tree[0]==root` 불변식을 복원한다.
   - 전용 state: `nodeVisitFlags / nodeRangeLeft / nodeRangeRight / rootIndexBuf`(5159–5184). **toggle ON일 때만** 할당/사용.

3. **SUB-OBJECT multi-root** — experimental, opt-in (`useSubObjectBVH`, default **false**, 5195).
   - material-space tile k=TR×TR로 grouped Karras를 돌리고 mini-TLAS(super-root)로 묶는다.
   - CPU 테이블 빌드: `computeSubObjectGroups / cpuStableGroupPartition / buildTopRec / buildSubObjectTopTree / topCombineCPU`(5735–5881).
   - GPU grouped 커널: `buildTree_Tri_Grouped / buildLeaf_Tri_Grouped / enlargeLeaf_Tri_Grouped / buildSweptLeaf_Tri_Grouped / bottomUpBoxesMultiRoot`.
   - 전용 state: `useSubObjectBVH / subBvhSplitS / subBvhP / numGroups / groupOfPrim / sortedPosToGroup / groupSize / groupPrimBase / groupNodeBase / subBvhNumNodes / subBvhSuperRoot / subBvhTopCombineOrder`(5186–5219).

build()의 분기(6075–6180)가 이 셋을 런타임에 골라 탄다: `subObjectActive()` → grouped, `useAgglomerative` → agglomerative, 그 외 → default Karras. 즉 **세 build 경로 + 두 query 경로(baseline `queryPoints` vs segmented `queryPointsSegmented`) + cpu/gpu 변형**이 하나의 거대한 struct(약 1830줄)에 toggle bool로 섞여 있다.

또한 이 struct에는 **GL 오염**이 정확히 한 군데 있다: `showBox()`(6891–6930)와 `debugBox/debugBoxLines`(`DebugLineGL<CPU>`, 5316–5317). 나머지는 GL-free. 새 엔진에서는 `showBox`를 인터페이스에서 **제외**한다(아래 §3).

### 1.2 toggle가 production default를 결정한다

`useAgglomerative`(5226)와 `useSubObjectBVH`(5195)가 **둘 다 default false**다. 따라서 출하 기본 경로는 정확히:
```
GPU fillMortons → GPU radix sort → GPU buildTree(Karras) → GPU bottomUpBoxes(walk-to-root) → 단일 commitAndWait
```
즉 **karras12가 사실상 유일한 production default**다. 새 설계에서 karras12를 DEFAULT concrete로 두는 것은 현행과 정합한다.

또 한 가지 난잡함: 이 toggle들이 **TLAS에서 매 프레임 push-down** 된다. `BVH<SCENE,OBJECT>::build/refit/...`이 `objTrees[i].useAgglomerative = useAgglomerative; objTrees[i].useSubObjectBVH = ...; objTrees[i].subBvhSplitS = ...`를 매번 다시 쓴다(6997–6999, 7051–7053). 새 설계에서는 **factory가 build 시점에 concrete를 한 번만 고정**하고, 매 프레임 toggle을 다시 밀어넣지 않는다.

### 1.3 TLAS wrapper — `BVH<BE,PR,BVHMODE::SCENE,BVHPRIMITIVE::OBJECT>` (main.cpp 6935–7361)

scene-level BVH(TLAS)는 별도 partial-spec struct다(6935). 구성:
- `std::vector<TRI_LBVH> objTrees`(6938) — mesh당 하나의 triangle LBVH(BLAS).
- `EDGE_LBVH tree`(6943) — per-object AABB 위의 top-level tree.
- `TRI_LBVH = BVH<...,LINEAR,TRIANGLE>`, `EDGE_LBVH = BVH<...,LINEAR,EDGE>`(6936–6937).

이게 **DEFAULT broad phase**다: `using BroadPhase = BVH<BE, PR, BVHMODE::SCENE, BVHPRIMITIVE::OBJECT>;`(7733). 동작:
- `build(Scene&)`(6974): dynamic object마다 subtree build(변하지 않은 Float mesh는 `lifetimeId` 캐시로 skip), object AABB 채우고 EDGE_LBVH TLAS build.
- `refit()`(7033): dynamic subtree refit(per-object commit 없음), static subtree는 `combineStaticOnce`, **단일 batched commitAndWait**, 그 다음 root AABB 읽어 TLAS 재build. (per-object commit이 ~34% 느려서 의도적으로 batch.)
- `enlargeTrajectory(dt)`(7096), `refitSwept(dt)`(7149): CCD trajectory inflation / fused refit+enlarge.
- `detectCollisions(margin, enableSelfCollisions, analyticEnabled)`(7207): ordered double loop로 object-vs-object AABB overlap 테스트 후 겹치는 쌍에 `queryPoints` 하강. `q==t`면 `checkSelfCollisions`.
- `detectCollisionsSegmented(...)`(7282): 같은 loop이되 std::set으로 unordered pair dedup + `queryPointsSegmented` 라우팅.

### 1.4 별도 broad-phase 두 개 — SpatialHashing / MultiLevelSpatialHashing

TLAS와 **인터페이스가 거의 같지만 별도 struct**인 broad phase가 둘 더 있다. 모두 같은 출력 규약(`Scene::packedCollisionData.broadCollisions / numBroadCollisions`)을 쓰고 typedef로 swap 가능하도록 surface를 맞춰 두었다.

- **`SpatialHashing<METAL,PR>`** (3538–4473): single-level Pabst uniform-grid hash. home+phantom insertion → radix sort → cell-prop mark/scan/fill → pair-count prefix → per-pair broad emit. 8개 GPU 커널 + CPU orchestration(`computeGrid` scene-AABB, `hostScanCellStarts`, `hostScanPairPrefix`, `findNumValid`).
- **`MultiLevelSpatialHashing<METAL,PR>`** (4476–4961): hgrid. **home-only insertion**(face당 fitting level 한 곳) + cross-level query walk(정렬된 entries binary search) + per-mesh AABB-diagonal floor exclusion. `ML_MAX_LEVELS=4`. `sh_reduceMaxRadius`와 single-level radix sorter를 재사용. cell-prop/pair-prefix 패스 없음(home-only).

Simulator는 이 셋을 런타임 selector로 고른다: `useSpatialHashing`(7744), `useMultiLevelSH`(7751, SH보다 우선), `useSegmentedBVHQuery`(7761), `useAnalyticPrimitive`(7768). 실제 분기는 substep loop(10043–10160)와 build 분기(9840–9853)에 흩어져 있다. 또한 `CollisionPipeline`(7717)은 `broadPhase` + `broadPhaseTest`(A/B용 두 번째 인스턴스, 7719) + `narrowPhase`를 들고 있고, Simulator는 그 옆에 `shBroadPhase`(7743)/`mlBroadPhase`(7750)를 **형제 멤버**로 또 들고 있다 — 즉 broad-phase 인스턴스가 동시에 여러 개 존재하고 분기로 켜진다.

### 1.5 진단 요약

| 난잡함 | 위치 | 새 설계에서의 처리 |
|---|---|---|
| 3 build 알고리즘이 1 struct에 toggle로 공존 | BVH LINEAR 5101–6931 | 3개 IBVH concrete로 분리 |
| toggle가 production default를 암묵 결정 | 5195/5226 default false | karras12 = 명시적 DEFAULT concrete |
| toggle를 TLAS가 매 프레임 push-down | 6997–6999, 7051–7053 | factory가 build 시 concrete 1회 고정 |
| broad-phase 인스턴스 다중 공존 + 분기 selector | 7717–7768, 10043–10160 | factory가 VERSION 1개 instantiate |
| detectCollisions surface 불균일(2-arg vs 3-arg) | SH/ML 4343/4875 vs BVH 7207 | IBVH가 surface 통일(§3) |
| GL 오염 | showBox/debugBox 5316–5317, 6891–6930 | 인터페이스에서 제외 |

---

## (2) 버전 census 표

모든 collision/broad-phase **VERSION**을 한 행으로. (소스 라인은 main.cpp; 커널은 그대로 재사용할 `src/metal/*.metal`.)

| VERSION | source lines | build algorithm | cpu | gpu | public surface (대표) | metal kernels |
|---|---|---|---|---|---|---|
| **karras12 LBVH** (DEFAULT) | 5101–6931 (LINEAR struct 내 default 경로) | Karras 2012: Morton → radix → `buildTree`(determineRange/findSplit) → `bottomUpBoxes`(atomic walk-to-root) | ✅ full (`buildCPU` 6182, `fillMortonsCPU` 5688, `radixSortCPU` 5642, `bottomUpCombine` 6413; query `queryAABB` 6581) | ✅ default (`fillMortonsGPU` 5718, `radixSortGPU` 5685, `buildTreeGPU` 5628, `bottomUpBoxesGPU` 5418; query `queryPoints` 6669) | build / refit / enlargeTrajectory / refitSwept / queryPoints / checkSelfCollisions / queryBegin / queryEnd | `fillMortons_Tri/_Edge`, `buildLeaf_Tri/_Edge`, `buildTree_Tri/_Edge`, `zeroVisitCounts`, `bottomUpBoxes`, `enlargeLeaf_Tri`, `buildSweptLeaf_Tri`, `queryPoints` |
| **apetrei-agglomerative** | LINEAR struct, `useAgglomerative` 경로 (state 5159–5184; build 5568–5613) | Apetrei 2014: hierarchy+combine을 단일 커널로 fuse, 그 후 root swap | ❌ (CPU twin 없음) | ✅ only (`agglomerativeBuildGPU` 5568, `agglomerativeSwapRootGPU` 5602) | build (그 외 refit/enlarge/query는 LINEAR 공유) | `agglomerativeBuild_Tri/_Edge`, `agglomerativeSwapRoot` |
| **subobject-multiroot** | LINEAR struct, `useSubObjectBVH` 경로 (state 5186–5219; fn 5735–6073) | grouped Karras over k=TR×TR tiles + mini-TLAS(super-root) | 부분 ✅ (group 테이블 `computeSubObjectGroups`/`cpuStableGroupPartition`/`buildTopRec`/`topCombineCPU` CPU) | ✅ (grouped 커널 + multi-root combine) | build / refit / enlarge / objectRootAABB(super-root) | `buildTree_Tri_Grouped`, `buildLeaf_Tri_Grouped`, `enlargeLeaf_Tri_Grouped`, `buildSweptLeaf_Tri_Grouped`, `bottomUpBoxesMultiRoot` |
| **BVH segmented query** (LINEAR의 query A/B) | `queryPointsSegmented` 6757–6825 | 3-커널 segmented: detect → scanReserve → compact (per-TG atomics) | ❌ | ✅ | queryPointsSegmented / checkSelfCollisionsSegmented | `queryPointsSegmented`, `scanReserveSegmented`, `compactSegmented` |
| **BVH-SCENE / TLAS** (현 DEFAULT broad phase) | 6935–7361 | N개 TRI_LBVH(BLAS) + 1개 EDGE_LBVH(top); object-vs-object AABB loop 후 queryPoints 하강 | 부분 ✅ (TLAS positions/indices fill, object-vs-object intersect loop, `combineStaticOnce`, `topCombineCPU` CPU) | ✅ (per-tree GPU build/refit) | build(Scene&) / refit / enlargeTrajectory / refitSwept / detectCollisions(3-arg) / detectCollisionsSegmented / checkSelfCollisions / queryBegin / queryEnd | (BLAS 커널 위임) |
| **SpatialHashing** (uniform hash) | 3538–4473 | single-level Pabst uniform grid: home+phantom insert → sort → cell-prop → pair-prefix → broad emit | 부분 ✅ (`computeGrid`/`hostScanCellStarts`/`hostScanPairPrefix`/`findNumValid` CPU) | ✅ (8 커널) | build(Scene&) / detectCollisions(2-arg) / refit(no-op) / enlargeTrajectory(no-op) / queryBegin / queryEnd | `sh_buildBV`, `sh_reduceMaxRadius`, `sh_assignCells`, `sh_markStarts`, `sh_fillCellProp`, `sh_computePairCount`, `sh_broadPhase` (+ radix sort) |
| **MultiLevelSpatialHashing** (hgrid) | 4476–4961 | multi-level hgrid: home-only insert(fitting level) + cross-level query walk + floor exclusion; cell-prop/pair-prefix 없음 | 부분 ✅ (`computeGrid`/`rebuildFaceExclude`/`effectiveLevels`/`findNumValid` CPU) | ✅ (3 커널 + `sh_reduceMaxRadius` 재사용) | build(Scene&) / detectCollisions(2-arg) / refit(no-op) / enlargeTrajectory(no-op) / queryBegin / queryEnd | `ml_buildBV`, `ml_assignCells`, `ml_broadPhase` (+ `sh_reduceMaxRadius` + radix sort) |
| **공유: radix sort** | `RadixSorter<METAL,Element>` 3473–3535 | 4-pass 8-bit LSD (BLOCK_SIZE=1024) | ❌ (BVH의 `radixSortCPU`는 별도 lambda) | ✅ | sort(src) | `radixCountBlock_8Bits`, `radixScanOffset_8Bits`, `radixScatter_8Bits`, `radixScatter_8Bits_PerBlock` |

> narrow phase(`BruteForce` / `narrow_pt_tri` / `narrow_pt_analytic`)는 broad-phase VERSION이 아니라 공통 소비자이므로 census에서 제외. 단 어떤 broad VERSION이든 동일한 `broadCollisions` 버퍼를 채워 narrow가 그대로 소비한다는 출력 규약은 §3/§5에서 고정한다.

---

## (3) IBVH 인터페이스 설계

매핑된 public surface에 근거해, Simulator가 기대하는 **공유 BroadPhase surface**를 virtual interface로 추출한다. `showBox`는 GL 오염이므로 **제외**한다(별도 `IDebugDraw`로 빼거나 default no-op).

### 3.1 surface가 아직 불균일하다 (핵심 문제)

맵이 확인하듯 네 broad phase가 거의 같은 surface를 공유하지만 **완전히 균일하지 않다**:

- `detectCollisions`가 **2-arg(SH/ML)** vs **3-arg(BVH, +`analyticEnabled`)** 로 갈린다.
  - SH/ML: `void detectCollisions(PR margin, bool enableSelfCollisions = true);` (4343 / 4875)
  - BVH-SCENE: `void detectCollisions(PR margin, bool enableSelfCollisions=true, bool analyticEnabled=false);` (7207)
- BVH-SCENE만 `detectCollisionsSegmented` / `checkSelfCollisions` / `refitSwept`를 갖고, hash 경로는 이를 stub(no-op)한다.
- `refit()`/`enlargeTrajectory()`는 hash 경로에서 **no-op**(grid가 매 detect마다 재build, 4330/4335/4869/4870)이지만 surface에는 존재한다 — 즉 surface에 있되 의미는 backend마다 다르다.

따라서 IBVH는 **공통 분모를 virtual로 두되, version-specific 인자(analytic/self-collision/segmented)는 멤버 플래그 또는 단일 호출 옵션 struct로 흡수**해 surface를 한 형태로 통일해야 한다.

### 3.2 IBVH virtual interface (제안)

```cpp
// arch-test: 모든 broad phase가 구현하는 공통 인터페이스.
// showBox는 제외(GL). version-specific 인자는 플래그/옵션으로 흡수해 surface를 통일한다.
template <typename PR>
struct IBVH {
    virtual ~IBVH() = default;

    // --- lifecycle / build ---
    virtual void build(Scene<METAL, PR>& scene) = 0;     // 전 mesh에 대한 (재)build
    virtual void refit()                = 0;             // topology 재사용; hash 계열은 no-op
    virtual void enlargeTrajectory(PR dt) = 0;           // CCD trajectory inflation; hash 계열은 no-op
    virtual void refitSwept(PR dt)      = 0;             // fused refit+enlarge; 미지원이면 refit+enlarge로 fallback

    // --- query ---
    // detectCollisions surface를 단일화: analytic / self-collision은 인자가 아니라
    // CollisionOptions로 흡수해 2-arg/3-arg 분열을 제거한다.
    struct CollisionOptions {
        bool enableSelfCollisions = true;
        bool analyticEnabled      = false;  // BVH-SCENE만 의미; 나머지는 무시
        bool segmented            = false;  // BVH-SCENE만 의미; 나머지는 무시
    };
    virtual void detectCollisions(PR margin, const CollisionOptions& opt) = 0;

    virtual void queryBegin() = 0;  // QueryFlag/numBroadCollisions zero
    virtual void queryEnd()   = 0;  // commitAndWait + overflow report
};
```

설계 근거:
- `build / refit / enlargeTrajectory / refitSwept / detectCollisions / queryBegin / queryEnd`만 인터페이스에 둔다(요구 사항). `checkSelfCollisions`/`detectCollisionsSegmented`는 별도 method가 아니라 `CollisionOptions`로 흡수 — segmented는 `opt.segmented`, self-collision은 `opt.enableSelfCollisions`로 라우팅(현행은 `q==t`에서 self-query, 7207의 분기를 그대로 옮긴다).
- hash 계열의 `refit/enlargeTrajectory`는 인터페이스 계약상 **합법적 no-op**(grid가 detect마다 재build). 즉 surface에 존재하되 구현이 비어 있어도 정합. 이는 현행과 동일(4330/4335/4869/4870).
- `refitSwept`는 BLAS의 swept-leaf 커널(`buildSweptLeaf_Tri`)이 없는 concrete에서 `refit()+enlargeTrajectory()`로 fallback(현행 LINEAR 5986, SCENE 7149의 fallback과 동일).
- 출력 규약은 **모든 concrete 공통**: `Scene::packedCollisionData.broadCollisions / numBroadCollisions`. narrow phase가 어떤 concrete든 동일 버퍼를 소비한다(census 각주). 이 규약을 인터페이스 docstring으로 못박는다.
- `showBox`/`showSceneBox`/`queryClickRay`는 인터페이스에서 제외. showBox는 GL(별도 `IDebugDraw`); queryClickRay는 picking GUI용이라 sim core가 아니다.

### 3.3 Simulator가 기대하는 형태

현행 Simulator는 `CollisionPipeline<BroadPhase, NarrowPhase>`(7717)에 `broadPhase`/`broadPhaseTest`를 들고, 형제로 `shBroadPhase`/`mlBroadPhase`를 둔 뒤 `useSpatialHashing`/`useMultiLevelSH`로 분기(10043–10160)한다. 새 설계에서는 이 분기·형제 멤버 전부를 **`std::unique_ptr<IBVH<PR>>` 하나**로 대체한다. Simulator는 어떤 concrete인지 모른 채 위 7개 virtual만 호출한다. 어떤 VERSION을 쥘지는 §4의 factory가 결정한다.

---

## (4) BVHFactory 설계 (가장 중요)

factory의 두 책임:
1. **어떤 VERSION concrete를 instantiate** 할지 선택 (karras12 / apetrei / subobject / SH / ML).
2. karras12(및 BLAS) 에 대해 **build PART별로 cpu-or-gpu**를 선택.

### 4.1 karras12 build PART별 cpu/gpu 매트릭스 (정확한 fn 이름)

맵의 bvh-core per-part cpu/gpu matrix에 근거. 각 PART마다 **cpu fn / gpu fn / 둘 다** 존재 여부를 정확한 main.cpp 함수명으로 적는다.

| PART | 단계 | CPU fn | GPU fn | 상태 |
|---|---|---|---|---|
| **PART1** | scene AABB | inline loop (`build` 6110–6112 / `buildCPU` 6195–6196) | **없음** (GPU reduction 미존재) | **CPU only** |
| **PART2** | fill Mortons | `fillMortonsCPU(sceneBox&)` 5688 | `fillMortonsGPU(sceneBox&)` 5718 | **둘 다** |
| **PART3** | radix sort | `radixSortCPU()` 5642 | `radixSortGPU()` 5685 (via `RadixSorter<METAL,MortonNode>` 3473) | **둘 다** |
| **PART4** | hierarchy (KARRAS) | `buildCPU` 내부 findSplit/determineRange + leaf/internal 루프 6280–6381 | `buildTreeGPU()` 5628 → `buildTree_Tri` (bvh.metal:352) | **둘 다** |
| **PART4** | hierarchy (APETREI) | **없음** (CPU twin 없음) | `agglomerativeBuildGPU()` 5568 + `agglomerativeSwapRootGPU()` 5602 | **GPU only** |
| **PART4** | hierarchy (SUBOBJECT) | group 테이블 CPU (`computeSubObjectGroups` 등) | `buildTreeGroupedGPU` / `buildLeafGroupedGPU` | grouped 전용 |
| **PART5** | combine (KARRAS) | `bottomUpCombine()` 6413 | `bottomUpBoxesGPU(sceneBox)` 5418 | **둘 다** |
| **PART5** | combine (HYBRID) | `bottomUpCombineWithSkip()` 5491 | `bottomUpBoxesPartialGPU(sceneBox,maxDepth)` 5457; driver `bottomUpHybrid(sceneBox,maxDepth)` 5523 | **둘 다(frontier 기준)** |
| **PART5** | combine (APETREI) | — | (build 커널에 fuse됨) | combine 없음 |
| **REFIT** | leaf 재계산 + combine | **없음** (`buildCPU`는 full-rebuild 전용) | `refit()` 6434 | **GPU only** |
| **ENLARGE** | swept leaf inflate | `enlargeTrajectory(dt)` 6489 CPU loop + `bottomUpCombine` (default) | `enlargeLeafGPU(dt)` 5915 + combine (`YSIM_GPU_ENLARGE` opt-in) | **둘 다(default CPU)** |
| **QUERY** | broad query | `queryAABB(queryBox[,node])` 6581 | `queryPoints(qIndex,margin)` 6669 (baseline) / `queryPointsSegmented` 6757 (A/B) | **둘 다(GPU broad-feed)** |

핵심 비대칭(설계에 반영):
- PART1 scene-AABB는 **CPU 전용**(GPU reduction 없음). factory가 PART1을 GPU로 고를 수 없다.
- APETREI hierarchy는 **GPU 전용**(CPU twin 없음). apetrei concrete는 PART4를 CPU로 고를 수 없다.
- REFIT은 **GPU 전용**(CPU refit 없음; `buildCPU`는 full-rebuild만). CPU-only 모드는 refit 대신 full rebuild로 돌아야 한다.
- ENLARGE의 GPU 경로는 **triangle-only이며 `YSIM_GPU_ENLARGE` opt-in**, default는 CPU.
- combine HYBRID는 frontier(`treeVisitCounts==2`)까지 GPU, 그 위는 CPU. `bottomUpHybridDepth` default 30 ≈ full-GPU. 정확성용이 아니라 perf knob.

### 4.2 동기화(sync) 규약 — factory가 cpu/gpu를 섞을 때 필수

- `build()`는 끝에 **정확히 한 번** `commitAndWait`. `refit()`은 **일부러 commit을 생략**(TLAS가 전 objTrees에 대해 한 번 batch, 7046–7061 / 7033).
- CPU PART가 GPU PART의 결과를 읽으려면 **그 직전에 commitAndWait** 필요(unified memory라 commit 후 `tree.ptr` 직접 읽기). 즉 factory가 "GPU PART4 → CPU PART5" 같은 혼합을 고르면 경계에 sync를 삽입해야 한다.
- `commitAndWait`는 null-encoder no-op guard(D-030)가 있어, CPU-only PART가 두 GPU PART 사이에 끼어도 안전.
- `combineStaticOnce()`(6575)는 static mesh의 large-tree under-combine을 가리는 **workaround**다. 새 combine은 이게 필요 없게 설계하는 게 목표(§5).

### 4.3 factory / config API 스케치

```cpp
// 어떤 VERSION을 instantiate할지.
enum class BVHVersion {
    Karras12,        // DEFAULT
    ApetreiAgglom,
    SubObjectMultiRoot,
    SpatialHashing,
    MultiLevelSpatialHashing,
};

// PART별 cpu/gpu 선택. "Auto"는 §4.1 매트릭스의 default를 따른다.
enum class PartBackend { Auto, CPU, GPU };

struct BVHBuildConfig {
    // karras12/BLAS에만 의미 있는 PART 다이얼.
    PartBackend mortons   = PartBackend::Auto;  // PART2: CPU/GPU 둘 다 가능
    PartBackend radixSort = PartBackend::Auto;  // PART3: CPU/GPU 둘 다 가능
    PartBackend hierarchy = PartBackend::Auto;  // PART4: Karras=둘 다, Apetrei=GPU-only
    PartBackend combine   = PartBackend::Auto;  // PART5: CPU/GPU/Hybrid
    PartBackend query     = PartBackend::Auto;  // CPU=queryAABB, GPU=queryPoints
    bool        segmentedQuery = false;         // GPU query A/B (queryPointsSegmented)
    int         hybridCombineDepth = 30;        // PART5 HYBRID frontier (default ≈ full-GPU)
    // 주의: PART1(sceneAABB)은 CPU-only, REFIT은 GPU-only — config로 못 뒤집음.
};

struct BVHFactory {
    BVHVersion     version = BVHVersion::Karras12;  // production default
    BVHBuildConfig build;

    // VERSION concrete를 instantiate하고, karras12면 PART 다이얼을 주입한다.
    template <typename PR>
    static std::unique_ptr<IBVH<PR>> create(const BVHFactory& cfg);
};
```

`create`의 동작(근거: §4.1 매트릭스 + §3 인터페이스):
1. `version`에 따라 concrete 선택 — `Karras12BVH` / `ApetreiBVH` / `SubObjectBVH` / `SpatialHashingBVH` / `MultiLevelBVH`. 각각 `IBVH<PR>` 구현.
2. `Karras12BVH`/`SubObjectBVH`에는 `BVHBuildConfig`를 주입. concrete는 PART별로 Auto면 default(GPU 중심)를, CPU/GPU면 해당 fn을 호출하도록 dispatch.
3. 불가능 조합은 **생성 시점에 거부 또는 강제 보정**:
   - `version==ApetreiAgglom && build.hierarchy==CPU` → 거부(Apetrei는 GPU-only PART4).
   - `build.mortons==GPU`인데 `combine==CPU` 등 cpu/gpu 혼합 → 경계 commitAndWait 삽입 책임을 concrete가 진다.
   - SH/ML에는 `BVHBuildConfig`의 PART 다이얼이 의미 없으므로 무시(그쪽은 자체 CPU orchestration + GPU 커널 구조).
4. **toggle을 매 프레임 push-down하지 않는다.** version/part 결정은 여기서 한 번. (현행 6997–6999의 매-프레임 toggle 재주입을 제거.)

이로써 §1.2의 "toggle가 default를 암묵 결정"과 §1.5의 "인스턴스 다중 공존"이 사라진다: Simulator는 `IBVH` 하나만 들고, factory가 그 하나의 정체와 PART backend를 결정한다.

---

## (5) 마이그레이션 권고

### 5.1 순서

1. **karras12 first** — DEFAULT이자 모든 것의 basis. `IBVH` + `Karras12BVH`(LINEAR struct의 default 경로만 떼어낸 것) + `BVHFactory`(PART 다이얼) + TLAS wrapper(BVH-SCENE)를 먼저 세운다. 이것만으로 현 production default 경로(GPU fillMortons→radix→buildTree→bottomUpBoxes→1 commit)가 재현되어야 한다.
2. **apetrei-agglomerative** — follow-up concrete. `agglomerativeBuild_*`/`agglomerativeSwapRoot` + range/visit-flag state. PART4 GPU-only 제약 반영.
3. **subobject-multiroot** — follow-up concrete. grouped 커널 + mini-TLAS + CPU group 테이블. square-cloth-only인 점 유지.
4. **SpatialHashing / MultiLevelSpatialHashing** — follow-up concrete. 자체 CPU orchestration + GPU 커널 구조 그대로. `refit/enlargeTrajectory`는 no-op로 IBVH 충족.

각 follow-up은 karras12를 건드리지 않고 **새 concrete를 추가**하는 방식(인터페이스 뒤). factory에 enum 한 줄 + 구현 추가.

### 5.2 반드시 그대로(verbatim) 보존할 것

데이터 ABI와 알고리즘 불변식은 GPU 커널과 byte-match/규약-match가 깨지면 **조용히 손상**되므로 verbatim 보존한다.

- **`MortonNode` 8B** — `struct alignas(8) { uint code; uint index; }; static_assert(sizeof==8)`. radix sorter의 layout-lock.
- **`BVHNode` 32B leaf sentinel** — `union { AABB4 aabb; struct alignas(32){ vec3 min; int childA; vec3 max; int childB; }; }; static_assert(sizeof==32)`. `childA==-1`이 leaf, leaf의 `childB`=primitive id. leaf-index 규약 `leafSlot=N+id-1`도 함께.
- **`AABB4` 32B** — union(min,i0,max,i1), `i0/i1`이 scratch(numPrimitives/query pid·objid/child) overload. `sceneBox._pad0/i0 == numPrimitives` 규약을 `bottomUpBoxes`가 읽는다.
- **`expandBits` / `mortonCode` 10-bit interleave CPU↔metal parity** — CPU(5691)와 metal(bvh.metal:26)이 **반드시 동일 비트 연산**. PART2를 CPU/GPU 어느 쪽으로 골라도 같은 Morton이 나와야 한다.
- **`findSplit` / `determineRange` Karras delta 규칙** — `+32 + clz(index ^ index)` tie-break 포함. PART4 hierarchy의 정확성.
- **`buildTree`의 `treeParent` 채움** — bottom-up combine이 부모를 거슬러 올라가는 데 의존.
- **`bottomUpBoxes` atomic + fence 메모리 순서 계약** (bvh.metal:510–586, WEDGE/INDEX guard 포함) — GPU lockup 방지용 load-bearing 안전 장치. 그대로.
- **`combineStaticOnce`** (6575) for static ground — large static tree의 under-combine을 막는 one-time CPU re-combine. 새 combine이 이를 불필요하게 만드는 게 이상적이지만, **그 전까지는 static mesh(고정 ground) 경로에서 보존**해야 cloth가 바닥을 뚫지 않는다.

### 5.3 metal 재사용 / surface 통일

- `src/metal/*.metal` + `common_types.metalh`는 GL/ImGui 0, CPU 오염 0 — **전부 verbatim 재사용**. metallib 빌드 룰(per `.metal`→`.air`→하나의 `default.metallib`)만 복제하고, 커널명마다 PSO 1개 생성.
- host param struct(`QueryPointsParams`, `SHParamsHost` 36B, `SHEntry` 8B, `SHBroadParamsHost` 20B, `MLParamsHost` 88B, `MLBroadParamsHost` 24B, `SimParams`, `RadixParams`)는 `static_assert`와 함께 **byte-match 유지**. setBytes로 넘기므로 drift = silent corruption.
- §3.1의 surface 불균일(2-arg vs 3-arg detectCollisions, segmented/checkSelfCollisions가 BVH에만)을 `CollisionOptions`로 흡수해 **IBVH 한 형태로 통일**하는 것이 마이그레이션의 인터페이스 측 핵심 작업이다.

### 5.4 제외 / stub 가능

- `showBox` / `showSceneBox` / `debugBox` / `debugBoxLines`(`DebugLineGL<CPU>`) — GL. 인터페이스 제외, 별도 `IDebugDraw`.
- `buildCPU`(6182) full CPU rebuild — hot path 아님. optional CPU backend 또는 test oracle로 유지.
- `validate*` / `printLastStats` / `LastRunStats` / `broadPhaseTest`(두 번째 인스턴스) — 디버그/텔레메트리. 초기엔 stub 가능.
- `RadixSortParamsCPU`(5394), 은퇴한 `radix*MortonBlocks` PSO, 빈 partial-spec(`BVH<SCENE,PRIMITIVE>` 7363) — drop.
