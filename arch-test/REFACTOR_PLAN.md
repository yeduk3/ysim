# arch-test 리팩터 설계 문서 (REVIEW용)

> 이 문서는 `arch-test/` 아래에 짓는 **새 독립 엔진**의 설계안이다.
> 기존 `src/main.cpp`(~약 4700줄로 인식되나 실측 ~20000줄 규모)는 **건드리지 않는다**.
> 결정된 아키텍처(가상 인터페이스 ICDPipeline / ISystem / IBVH, Scene=정적 기술, SimState=라이브 상태 분리,
> 팩토리로 concrete 선택, headless Runner)는 **확정 사항**으로 다시 논쟁하지 않는다.
> 근거는 코드 주석이 아니라 이 md 문서들에 기록한다(새 엔진의 코드 주석은 최소화).

---

## 1. 목표 요약

### 1.1 최소 목표 (Minimum goal)
- 기본/default scene이 자유 함수 `setupBasicScene(Simulator&)`가 되고,
- GL 없는 **headless Runner**가 그것을 구동한다 (`build System -> Simulator(system) -> setupBasicScene(sim) -> sim.initialize() -> loop { sim.update(); }`).
- 기준 default scene 레시피(고정, 그대로 재현):
  - obj0 = `addCloth(50, 1.0, {0,1.25,0}, kstretch=1e5, kshear=1e5, kbend=2e5, thickness=0.01, mass=0.1)` — TriangularCloth, XZ grid, jiggle=true, seed=mesh-id(0), 2500 verts / 4802 tris
  - obj1 = `addFloatMesh(assetRoot(), "Human.obj", {0,0.35,0}, scale=0.04, mass=0.1)` — Float
  - obj2 = `addGround(XZPlane, {0,0,0}, 50, mass=0.1)` — Float, 고정 2x2 quad (100m x 100m)
  - id는 반드시 0,1,2 순서 (id == 배열 슬롯 == 모든 packed-data 인덱스 == objPair)
  - environment 기본값 gravity(0,-9.81,0), wind 0
  - System: h=1/60, **subSteps=60** (main이 60 전달; ctor 디폴트는 50 — default-scene 값은 60), margin=0.015, radius=0.012, targetFrames=300
  - (참조 위치: main.cpp 18282-18288 / 18230-18233)

### 1.2 5개 청사진 목표 + 이번 패스 attempt vs defer

| # | 청사진 목표 | 이번 패스 |
|---|-------------|-----------|
| G1 | 가상 인터페이스 전면: `ICDPipeline`(virtual dcd/ccd), `ISystem`(virtual accumulate/integration/recoveryPenetration). Simulator가 `unique_ptr<ICDPipeline>` + `unique_ptr<ISystem>` 보유, 팩토리로 선택 | **부분 attempt**: 두 인터페이스 정의 + 각 1개 concrete 구현. dcd/ccd 분리는 인터페이스로 노출하되 내부는 기존 융합(narrow_pt_tri 안에 xPrev swept CCD)을 유지(아래 §1.3 참고). accumulate/integration/recoveryPenetration도 메서드로 노출하되 recoveryPenetration은 integrate 커널에 융합된 현 계약을 그대로(아래 §5) |
| G2 | `IBVH` 인터페이스, karras12 LBVH가 **DEFAULT** concrete; apetrei-agglomerative / subobject-multiroot / SpatialHashing / MultiLevelSpatialHashing은 **별도** concrete; `BVHFactory`가 버전 + part별 cpu-vs-gpu 선택 | **부분 attempt**: `IBVH` 정의 + `Karras12BVH`(LINEAR/TRIANGLE) 한 개만 구현. 나머지 4개는 **defer**(stub 헤더 또는 미생성). `BVHFactory`는 enum 골격만(현재는 Karras12만 반환) |
| G3 | Scene = passive 정적 기술(topology, groups, isStatic, materials). 라이브 pos/vel 없음. 라이브 GPU 상태 -> 별도 `SimState`를 substep마다 전달 | **attempt(핵심)**: `Scene`(정적)과 `SimState`(라이브 x/xPrev/v/f, +externalForces, n) 분리. m(mass)는 SimState에 immutable로 둠. realization이 정적 topology와 초기 SimState를 둘 다 시드 |
| G4 | 기존 `src/metal/*.metal` 셰이더 **변경 없이** 재사용 | **attempt(전부)**: 6개 .metal + common_types.metalh 그대로, metallib 빌드 룰만 복제. 호스트 mirror 구조체는 byte-identical 유지 |
| G5 | 코드 주석 최소, 근거는 md에. 애매한 결정은 그대로 두고 문서화(구현 안 함) | **attempt**: 본 문서가 그 기록처. 애매점(§1.3) 명시 |

### 1.3 이번 패스에서 "그대로 둔" 애매한 결정 (문서화만, 구현 안 함)
- **D-A1 dcd/ccd 분리 vs 융합**: 현 코드엔 별도 CCD 패스가 없다. CCD는 `narrow_pt_tri` 단일 DCD 커널 안에 `xPrev`(substep 시작 위치)로 swept point-triangle 테스트로 융합돼 있다(D-013). `ICDPipeline`은 `dcd()`/`ccd()` 가상 메서드를 **노출**하되, 첫 concrete는 `ccd()`를 no-op으로 두고 `dcd()`가 swept 융합 커널을 그대로 호출. 진짜 분리는 다음 패스.
- **D-A2 recoveryPenetration 위치**: 현 코드엔 standalone `recoveryPenetration()`이 없다. 침투 복원은 `integrate_cloth`/`integrate_cloth_grid` 커널 안에 `vertColFacets`를 읽어 vn-zeroing + (thickness-distance)*n push로 융합돼 있다. `ISystem::recoveryPenetration()`을 노출하되 첫 concrete에서는 integrate 융합 계약을 유지(naive whole-contact 재사용은 fixed-push integrator를 폭발시킴 — main.cpp 10017-10037 주석 근거).
- **D-A3 accumulate 위치**: gravity*mass + wind 누적은 System이 아니라 `Simulator::applyEnvironmentForces`가 mesh.externalForces에 미리 채운다. `ISystem::accumulate()`는 spring-force 커널 디스패치(force pass)로 매핑하고, 환경력 주입은 Simulator에 남긴다(현 구조 보존).
- **D-A4 GlobalAutoAllocator 프로세스-와이드 싱글톤**: BE만으로 키된 하드 싱글톤. 다중 독립 scene/allocator를 원하면 이게 최대 제약. 이번 패스는 현 구조(전역 싱글톤 + `Scene::pack` 의존) **그대로 사용**. 인스턴스화는 다음 패스 검토 항목으로 남김.
- **D-A5 Scene 정적 싱글톤 -> 인스턴스화**: 기존 `Scene<BE,PR>`는 inline static 멤버 전부(meshes/requests/packed*/counters/environment). 새 엔진은 인스턴스 소유로 가야 하나, packedMeshData/packedCollisionData가 여러 GPU 커널의 setBuffer 타깃이라 이번 패스는 **소유 인스턴스 + 그 안의 멤버**로만 옮기고, lifetimeId(BVH skip-cache용, 절대 리셋 안 됨) 의미는 보존.
- **D-A6 CPU VectorBase 슬라이싱 부재**: `pack()`의 sub-view 슬라이싱은 `VectorBase<METAL>(v,start,size)` ctor에만 있고 CPU엔 없다 -> 현재 CPU 백엔드는 pack 슬라이싱 경로 컴파일 불가 = 사실상 METAL 전용. 이번 패스는 **METAL 전용**으로 진행(진짜 cpu-vs-gpu-per-part는 CPU VectorBase에 슬라이싱 ctor 추가가 선결 — defer).

---

## 2. 새 엔진 파일 레이아웃

> 루트: `arch-test/`. 셰이더는 `src/metal/`(기존)에서 그대로 가져옴.
> 각 파일: 한 줄 목적 + 포팅 출처(main.cpp 라인 / metal 커널).

```
arch-test/
├── CMakeLists.txt                  새 타깃 arch_test_runner; src/metal metallib 재사용, backend 레이어 include
├── REFACTOR_PLAN.md                (본 문서)
└── src/
    ├── backend/
    │   ├── Backend.hpp             Backend/CPU/CUDA/METAL 태그 + Index=uint32_t  [main 50,54-57]
    │   ├── MemoryPool.hpp          MemoryBlock / ByteMemoryPool / FakeMemoryPool / DynamicByteMemoryPool / DynamicMemoryAllocator / GlobalAutoAllocator  [main 69-531]
    │   ├── VectorBase.hpp          VectorBase<CPU/METAL>(슬라이싱 ctor 포함), Matrix/SparseMatrix(CPU, stub 가능)  [main 122-642]
    │   └── MetalContext.hpp        MetalGlobalContext + MetalKernelContext (device/queue/encoder, PSO/lib 캐시)  [main 126-265]
    ├── core/
    │   ├── Types.hpp               IndexPair/BroadCollision/NarrowCollision/AnalyticShape, enums(ShapeType/BehaviorType/PlaneDirection), Material, SceneEnvironment  [main 761-780,1749-1892,2140-2146,2753-2769]
    │   ├── BehaviorParams.hpp      ClothBehaviorParams/FastGridClothBehaviorParams/FloatBehaviorParams + variant + recomputeFastGridRest  [main 1932-2010]
    │   ├── SimState.hpp            ★라이브 상태: contiguous x/xPrev/v/f/n/externalForces + per-mesh sub-view; m은 immutable  [main 789-809 MeshState에서 분리, 2904-2936 PackedMeshData에서 라이브분 분리]
    │   ├── Scene.hpp               ★정적 기술: topology(facets/edges/adjacency/offsets/vertObj/rest*), groups, isStatic, Material, fixedVertices, RequestGeneralMesh 블루프린트, pack()  [main 2771-3462 중 정적분 + 811-848 MeshAdjacency + 1894-1930 Constraints]
    │   └── Scene.cpp               pack()/initialize()/findById/realization 구현(라이브분은 SimState로 시드)  [main 3081-3450]
    ├── mesh/
    │   ├── IMeshInitializer.hpp    GeneralMeshInitializer 가상 인터페이스(initialize/getParams/populatePreview) + InitializerParams + MeshAdjacencyInitializer  [main 856-1171]
    │   ├── GridInitializer.hpp     MeshGridInitializer(+Params): NxN grid, 대각 교번 삼각화, jiggle  [main 1180-1391]
    │   ├── FileInitializer.hpp     MeshFileInitializer(+Params) OBJ 로더 (Assimp는 별 브랜치 필요 — sibling)  [main 1393-1473, 1479-1553]
    │   └── PreviewState.hpp        (기존 include/PreviewState.hpp 재사용 또는 얇은 래퍼) sim<->renderer 브리지, GL-free  [include/PreviewState.hpp:24-145]
    ├── collision/
    │   ├── ICDPipeline.hpp         가상 인터페이스: virtual dcd(SimState&,margin)/ccd(SimState&,dt)/build/refit; Simulator가 unique_ptr 보유  [main 7716-7724 CollisionPipeline을 인터페이스화 + 10080-10203 인라인 orchestration을 메서드화]
    │   ├── DefaultCDPipeline.hpp   concrete: IBVH broad + BruteForce narrow 묶음, dcd=swept융합 호출, ccd=no-op(D-A1)  [main 10080-10203 substep 흐름]
    │   ├── IBVH.hpp                ★가상 broad-phase 인터페이스: build/refit/enlargeTrajectory/refitSwept/detectCollisions/queryBegin/queryEnd (showBox 제외)  [main 6935-7361 SCENE TLAS 표면 + 5101-6934 LINEAR 표면]
    │   ├── Karras12BVH.hpp/.cpp    ★DEFAULT concrete: LINEAR/TRIANGLE LBVH (fillMortonsGPU+radixSortGPU+buildTreeGPU+bottomUpBoxesGPU) + SCENE TLAS 래퍼  [main 5101-6934(LINEAR), 6935-7361(TLAS)]
    │   ├── BVHFactory.hpp          enum(Karras12/Apetrei/SubObject/SpatialHash/MultiLevelHash) + part별 cpu/gpu; 현재 Karras12만 반환  [main 9588-9723 initialize의 YSIM_BROADPHASE 훅을 팩토리화]
    │   ├── RadixSorter.hpp         4-pass 8-bit LSD GPU 라디ㅡ스 정렬 (MortonNode/SHEntry)  [main 3470-3535]
    │   ├── AABB4.hpp               32B AABB4 + Ray/RayHit + rayTriangleIntersect  [main 5000-5096]
    │   └── narrow/
    │       ├── INarrowPhase.hpp    (얇은) narrow 인터페이스 — DefaultCDPipeline가 소유  [main 7371-7372]
    │       └── BruteForceMetal.hpp narrow_pt_tri + narrow_pt_analytic 디스패치 + per-vertex counting sort -> vertColFacets  [main 7393-7681]
    ├── system/
    │   ├── ISystem.hpp             ★가상 인터페이스: virtual accumulate(SimState&,Scene&)/integration(SimState&)/recoveryPenetration(SimState&,contacts); Simulator가 unique_ptr 보유  [main 11021-11022 + 표면 재정의]
    │   ├── ExplicitSystem.hpp/.cpp concrete METAL: 2-pass force/integrate, ref-constraint copy, anomalyFlag; recovery는 integrate 융합(D-A2)  [main 11172-11389]
    │   └── ClothBehavior.hpp       TriangularClothBehavior/FastGridClothBehavior<METAL> setBuffer/update (force 커널 바인딩 테이블)  [main 2010-2138]
    ├── sim/
    │   ├── Simulator.hpp           ★허브: Scene + SimState + unique_ptr<ICDPipeline> + unique_ptr<ISystem>; initialize()/update()/reset()/applyEnvironmentForces()  [main 7726-11020 중 SIM-CORE만; GL/GUI 전부 제외]
    │   └── Simulator.cpp           update() substep 루프(refit->enlarge->detect->narrow->xPrev->system) + 단일 commitAndWait  [main 9777-10241, 9588-9723, 9731-9776]
    ├── scenes/
    │   └── basic_scene.hpp         ★setupBasicScene(Simulator&) 자유 함수: addCloth/addFloatMesh/addGround 3개 큐잉  [main 18282-18288 + 8299-8317/8155-8162/8683-8697]
    └── runner/
        └── Runner.hpp             ★headless 구동기: System 생성 -> Simulator -> setupBasicScene -> initialize -> N프레임 update 루프(GL 없음)  [main 17710-18017 bench 하네스 형태 참조]
    └── main.cpp                    arch_test_runner 진입점: Runner 호출, frame count 외부 공급  [main 18019-18306 의 default 브랜치 최소화]
```

### 2.1 backend 재사용 정책 (포팅 난이도: easy)
- **near-verbatim 복사**(GL/ImGui 무접촉): Backend 태그, MemoryBlock, ByteMemoryPool<CPU/METAL>, FakeMemoryPool, DynamicByteMemoryPool, DynamicMemoryAllocator, GlobalAutoAllocator, VectorBase<CPU/METAL>, MetalGlobalContext, MetalKernelContext.
- **반드시 보존**: (1) `DynamicByteMemoryPool::resetMarkers` cursor-replay 불변(re-init이 동일 주소 재현 — D-041), (2) METAL 256B alignment round-up, (3) `commitAndWait` null-encoder no-op 가드(D-030 — mixed cpu/gpu part 사이 CPU-only 경로에서 필요), (4) Index=uint32_t 전역 일관.
- **격리(headless core에 링크 금지)**: `DebugLineGL<CPU>`/`DebugPointGL<CPU>`(main 660-749) — 순수 GL VAO/VBO. 데이터 생산측(host float*)만 core, GL 업로드/draw측은 별도 renderer 모듈로. 이번 패스는 **아예 미포팅**(Runner는 headless).
- **stub 가능**: SparseMatrix/Matrix(CPU 전용, GPU-first 엔진에서 미사용 가능), CUDA 태그(specialization 0개 — placeholder만).

---

## 3. 빌드 전략

### 3.1 새 CMake 타깃
- `arch-test/CMakeLists.txt`는 별도 executable `arch_test_runner`를 정의한다(기존 `ysim` 타깃과 독립).
- 루트 `CMakeLists.txt`에서 `add_subdirectory(arch-test)`를 추가하거나, arch-test를 자체 프로젝트로 둘 수 있다(REVIEW 결정 필요 — 일단 subdirectory 가정).
- 요구사항(기존과 동일): CMake 3.10+, C++17, Eigen 5.0+, Metal-cpp, Xcode CLI(`xcrun metal`). GLFW/GLEW/OpenGL/ImGui는 **링크하지 않는다**(headless).

### 3.2 metallib 재사용 (셰이더 변경 없음)
- 기존 셰이더(`src/metal/*.metal` + `src/metal/common_types.metalh`)를 **그대로** 컴파일한다. 새 .metal을 만들지 않는다.
- 루트 CMakeLists.txt의 metallib 룰(37-63: 각 .metal -> .air via `xcrun -sdk macosx metal -c`, 전 .air -> 단일 `default.metallib` via `xcrun -sdk macosx metallib`, `add_custom_target(MetalKernels ALL)`)을 arch-test 타깃에 **재사용/복제**한다.
- `MetalKernelContext::getLibrary()`가 `ysim_paths::runtimeFile`로 exe-dir 옆 `default.metallib`를 로드하므로, arch-test 빌드 디렉터리에 동일한 `default.metallib`가 놓이도록 custom command 출력 경로를 맞춘다.
- 커널 46개 전부에 대해 PSO 1개씩 `getPSO(name)`으로 생성(census 출처: src/metal 8개 subsystem map 마지막 항목). 이번 패스에서 실제로 디스패치되는 커널은 default scene 경로에 한정:
  - bvh-core: `fillMortons_Tri`, `buildLeaf_Tri`, `buildTree_Tri`, `zeroVisitCounts`, `bottomUpBoxes`, `enlargeLeaf_Tri`(또는 `buildSweptLeaf_Tri`)
  - edge TLAS: `fillMortons_Edge`, `buildTree_Edge`(TLAS top tree)
  - radix: `radixCountBlock_8Bits`, `radixScanOffset_8Bits`, `radixScatter_8Bits`
  - query: `queryPoints`
  - narrow: `narrow_pt_tri` (+ `fill_vf_offsets` 필요시)
  - physics: `compute_tri_spring_forces`, `integrate_cloth`, `ref_constraint_copy_pos`, `ref_constraint_copy_force`
  - (default scene엔 cloth=TriangularCloth만 -> grid/analytic/agglomerative/spatial-hash 커널은 PSO 생성은 하되 미디스패치)

### 3.3 Runner 테스트가 컴파일/링크되려면 필요한 것
- backend 레이어(§2.1) 전부.
- core(Types/SimState/Scene) + mesh(IMeshInitializer/Grid/File).
- collision: IBVH + Karras12BVH(+TLAS) + RadixSorter + AABB4 + BruteForceMetal + ICDPipeline + DefaultCDPipeline + BVHFactory.
- system: ISystem + ExplicitSystem + ClothBehavior.
- sim: Simulator.
- scenes/basic_scene.hpp + runner/Runner.hpp + main.cpp.
- **링크 불필요**: GL/ImGui/GLFW/GLEW, DebugLineGL, Bullet(default scene에 Rigid 없음 -> rigid backend는 stub/미포팅 — 아래 §5), Assimp(Human.obj는 MeshFileInitializer OBJ 로더로 충분 -> AssimpMeshFileInitializer 미포팅).
- **컴파일 성공 판정**: `arch_test_runner`가 빌드되고, 실행 시 GL 컨텍스트 없이 N프레임 `sim.update()`가 NaN/anomaly 없이 완주(기존 `--self-test`가 이미 GL 없이 initialize/update를 도는 선례 — main ~11401).

---

## 4. 포팅 순서 (의존성 존중)

> 각 단계는 이전 단계만 의존하도록 순서화. 각 단계 끝에 "컴파일 게이트"를 둔다.

1. **backend 레이어** (`backend/*.hpp`)
   - 복사: Backend 태그/Index, MemoryPool 일체, VectorBase(METAL 슬라이싱 ctor 포함), MetalContext.
   - 게이트: 빈 `main`에서 `GlobalAutoAllocator<METAL>::globalInitialize(N)` + `VectorBase<METAL,float>` 1개 alloc + `MetalGlobalContext::getDevice()` 호출이 빌드/실행.

2. **core 데이터 타입** (`core/Types.hpp`, `core/BehaviorParams.hpp`)
   - IndexPair/BroadCollision/NarrowCollision/AnalyticShape (alignas/필드 순서 **byte-identical** to common_types.metalh — static_assert 동반).
   - enums + Material + SceneEnvironment + behavior param 구조체(physics.metal ClothParams/ClothGridParams와 layout 일치, 이름 불일치는 그대로).
   - 게이트: static_assert(sizeof) 전부 통과.

3. **SimState / Scene 분리** (`core/SimState.hpp`, `core/Scene.hpp/.cpp`, `mesh/*`)
   - SimState = contiguous x/xPrev/v/f/n/externalForces + per-mesh sub-view, m immutable.
   - Scene = 정적 topology(MeshAdjacency) + RequestGeneralMesh 블루프린트 + Constraints(fixedParticles) + pack().
   - 초기화기(GeneralMeshInitializer 인터페이스 + Grid + File + MeshAdjacencyInitializer + recomputeRestLengths/recomputeFastGridRest).
   - pack() 재작성: 정적 topology는 Scene packed 버퍼에, **라이브분(x/xPrev/v/f/n/externalForces)은 SimState로** 슬라이싱. realization이 둘 다 시드(positions bake, xPrev=x, pins 적용).
   - dynamic_cast cascade 대신 initializer에 virtual `shapeType()` 도입(Assimp sibling 문제 회피 — 이번 패스 Assimp 미포팅이라 단순).
   - 게이트: 3-object 시드 후 SimState.x / Scene.facets가 올바른 크기·내용(CPU host ptr 읽기, METAL Shared).

4. **IBVH / Karras12** (`collision/AABB4.hpp`, `RadixSorter.hpp`, `IBVH.hpp`, `Karras12BVH.hpp/.cpp`, `BVHFactory.hpp`)
   - AABB4(32B static_assert) + Ray + rayTriangleIntersect.
   - RadixSorter<METAL,MortonNode>.
   - Karras12 LINEAR/TRIANGLE: build(sceneBox CPU + fillMortonsGPU + radixSortGPU + buildTreeGPU + zeroVisitCounts + bottomUpBoxesGPU, 1 commitAndWait) / refit(no commit) / enlargeTrajectory / queryPoints.
   - SCENE TLAS 래퍼: objTrees(per-mesh TRI_LBVH) + EDGE top tree; build/refit(배치 1 commitAndWait)/detectCollisions(ordered double loop, objectRootAABB intersect -> queryPoints)/queryBegin/queryEnd.
   - **보존 필수**: BVHNode leaf sentinel(childA==-1, childB=primitive id), MortonNode 8B, expandBits/mortonCode 10-bit interleave(CPU<->metal 일치), Karras findSplit/determineRange delta+tie-break, leafSlot=N+id-1, bottomUpBoxes atomic+fence 계약(bvh.metal 510-586 WEDGE/INDEX 가드), refit는 commit 안 함(caller 배치), `combineStaticOnce`(static mesh under-combine 워크어라운드 — ground에 필요).
   - BVHFactory: enum 정의 + 현재는 Karras12만 반환(나머지 throw/assert).
   - 게이트: default scene 3-object로 build/refit 후 objectRootAABB가 합리적, queryPoints가 broadCollisions에 후보 기록.

5. **ICDPipeline / narrow** (`collision/narrow/BruteForceMetal.hpp`, `INarrowPhase.hpp`, `ICDPipeline.hpp`, `DefaultCDPipeline.hpp`)
   - BruteForceMetal: narrow_pt_tri 디스패치 + CPU counting-sort -> packedCollisionData.vertColFacets/Offsets (analytic은 default scene에서 off).
   - ICDPipeline 가상: build/refit(+enlarge)/dcd/ccd. DefaultCDPipeline: dcd = (refit/enlargeTrajectory ->) detectCollisions -> narrowAndSortByVertices(swept 융합); ccd = no-op(D-A1).
   - **보존**: NarrowParams/AnalyticNarrowParams setBytes ABI, resetNarrow 순서, skipSphere 게이트, detectCollisions 비-dedup ordered double loop.
   - 게이트: dcd() 1회 호출이 vertColFacets를 채움(CPU 읽기 검증).

6. **ISystem / ExplicitSystem** (`system/ISystem.hpp`, `ClothBehavior.hpp`, `ExplicitSystem.hpp/.cpp`)
   - ExplicitSystem<METAL>: buildRefPairs(CPU) -> ref_constraint_copy_pos -> Pass1 force(compute_tri_spring_forces via TriangularClothBehavior::setBuffer/update) -> ref_constraint_copy_force -> Pass2 integrate(integrate_cloth, anomalyFlag slot20). 단일 encoder serial 순서로 fence.
   - ISystem 가상: accumulate(=force pass), integration(=integrate pass), recoveryPenetration(=integrate 융합, D-A2). 매핑은 §1.3.
   - **보존**: physics.metal 커널/inline penetration math, SimParams/ClothParams byte-layout, sanitizeIntegrateOutput world-bound NaN 가드 + anomalyFlag pause, copy_pos->force->copy_force->integrate 순서, single commitAndWait는 System이 아니라 Simulator가(프레임당 1회).
   - 게이트: 정지 cloth에 1 substep 적용 후 NaN 없음.

7. **Simulator** (`sim/Simulator.hpp/.cpp`)
   - 소유: Scene(인스턴스), SimState, `unique_ptr<ICDPipeline>`, `unique_ptr<ISystem>`(팩토리로 주입).
   - `initialize()`: pool reset(GlobalAutoAllocator::reset) -> Scene::pack(SimState 시드) -> CDPipeline.build(scene) -> 환경 훅(YSIM_* 선택적). rigid/kinematic 블록은 default scene에서 inert -> 미포팅/stub.
   - `applyEnvironmentForces()`: gravity*mass + wind -> SimState.externalForces (Float/Rigid/Kinematic 0).
   - `update()`: dirty 재init 게이트, anomaly 폴/auto-pause(headless에선 정지 신호만), substep 루프 [cdPipeline.dcd(simState,margin); xPrev 스냅(Float/Kinematic 예외); system step(accumulate/integration)] -> **단일 commitAndWait** -> frame++ -> targetFrames 도달 시 정지.
   - **카브 라인**: draw*/uploadMeshes/debug*/showBox/MeshRenderState/selection/ghost/Bullet/Kinematic GUI 전부 제외(headless core는 update + system + narrow만, 이미 GL-free 검증된 9777-10241 범위).
   - 게이트: 3-object scene으로 N substep/N frame 완주.

8. **setupBasicScene** (`scenes/basic_scene.hpp`)
   - 자유 함수: `void setupBasicScene(Simulator& sim)` = addCloth(50,1,{0,1.25,0},1e5,1e5,2e5,0.01,0.1) + addFloatMesh(assetRoot(),"Human.obj",{0,0.35,0},0.04) + addGround(XZPlane,{0,0,0},50). id 0,1,2 순서 보장. addX는 큐잉만(CPU).
   - addX 내부의 registerPreviewBinding / scene_log은 headless에서 no-op.
   - 게이트: setupBasicScene 후 requests 3개, id 0/1/2.

9. **Runner + main** (`runner/Runner.hpp`, `src/main.cpp`)
   - Runner: System(h=1/60, subSteps=60) 생성 -> Simulator(system) -> setupBasicScene(sim) -> sim.initialize() -> `for (int f=0; f<frames; ++f) sim.update();` (frames 외부 공급, 기본 targetFrames=300).
   - main: CLI에서 frame count만 받아 Runner 호출. GL/GLFW/ImGui 전무.
   - 게이트(**최소 목표 달성**): GL 컨텍스트 없이 default scene 300프레임 완주, anomalyFlag 미발화.

---

## 5. 이번 패스 done vs stubbed 표

| 항목 | 상태 | 비고 / 근거 |
|------|------|-------------|
| backend 태그/Index | **done** | near-verbatim 복사 |
| MemoryPool 일체(Byte/Dynamic/Fake/GlobalAuto) | **done** | resetMarkers replay + 256B align + null-encoder 가드 보존 |
| VectorBase<METAL>(슬라이싱 ctor) | **done** | pack 슬라이싱 의존 |
| VectorBase<CPU> 슬라이싱 ctor | **stubbed** | CPU엔 슬라이싱 ctor 부재(D-A6) -> METAL 전용 진행 |
| Matrix/SparseMatrix | **stubbed** | CPU 전용, GPU-first에서 미사용 |
| MetalGlobalContext / MetalKernelContext | **done** | PSO/lib 캐시, commitAndWait |
| Types(Index/Broad/Narrow/AnalyticShape) + enums | **done** | byte-identical static_assert |
| BehaviorParams(Cloth/FastGrid/Float) + recomputeFastGridRest | **done** | Cloth만 default 경로; FastGrid는 정의만 |
| **SimState / Scene 분리** | **done(핵심)** | 라이브 x/xPrev/v/f/n/externalForces -> SimState; topology -> Scene; m immutable |
| Scene::pack (realization, SimState 시드) | **done** | dynamic_cast 대신 virtual shapeType() |
| GeneralMeshInitializer 인터페이스 | **done** | initialize/getParams/populatePreview |
| MeshGridInitializer / MeshFileInitializer | **done** | grid 삼각화 + OBJ 로더(Human.obj) |
| AssimpMeshFileInitializer | **stubbed** | OBJ 로더로 충분, Assimp 미링크 |
| MeshKinematicInitializer / mograph / bvh::Motion | **stubbed** | default scene에 Kinematic 없음 |
| MeshAdjacencyInitializer + rest length | **done** | edge dedup/opp/CSR/rest |
| **IBVH 인터페이스** | **done** | build/refit/enlarge/refitSwept/detect/queryBegin/queryEnd (showBox 제외) |
| **Karras12BVH (LINEAR/TRIANGLE, DEFAULT)** | **done** | fillMortons/radix/buildTree/bottomUpBoxes + queryPoints |
| Karras12 SCENE TLAS 래퍼 (+EDGE top tree) | **done** | ordered double loop detect, 배치 1 commit, combineStaticOnce(ground) |
| BVHFactory | **부분 done** | enum + part별 cpu/gpu 골격; 현재 Karras12만 반환 |
| Apetrei agglomerative BVH | **stubbed** | 별도 concrete, 다음 패스 |
| SubObject multi-root BVH | **stubbed** | 별도 concrete, square-cloth 전용 |
| SpatialHashing<METAL> | **stubbed** | 별도 concrete |
| MultiLevelSpatialHashing<METAL> (hgrid) | **stubbed** | 별도 concrete |
| RadixSorter<METAL,MortonNode> | **done** | BVH 정렬에 필요 |
| CPU BVH 경로(buildCPU/fillMortonsCPU/radixSortCPU) | **stubbed** | GPU 디폴트; CPU는 part-factory 옵션으로 다음 패스 |
| queryPointsSegmented (segmented broad) | **stubbed** | A/B 실험, 미필요 |
| **BruteForce<METAL> narrow (narrow_pt_tri + sort)** | **done** | vertColFacets 생산 |
| narrow_pt_analytic / analytic path | **stubbed** | INERT, default scene off |
| BruteForce<CPU> narrow | **stubbed** | 원본도 빈 stub |
| **ICDPipeline 인터페이스 + DefaultCDPipeline** | **done** | dcd=swept 융합 호출 |
| ICDPipeline::ccd 분리 | **stubbed(no-op)** | CCD는 narrow_pt_tri에 융합(D-A1) |
| **ISystem 인터페이스 + ExplicitSystem<METAL>** | **done** | 2-pass force/integrate + ref-constraint |
| ISystem::recoveryPenetration 분리 메서드 | **stubbed(융합)** | integrate 커널 내 침투 복원 유지(D-A2) |
| ISystem::accumulate(환경력) 위치 | **부분** | force kernel=accumulate; 환경력 주입은 Simulator(D-A3) |
| ExplicitSystem<CPU> | **stubbed** | 원본도 dead stub |
| ClothBehavior (Triangular force 바인딩) | **done** | FastGrid는 미디스패치 |
| ref-constraint copy 커널 | **done** | pairCount==0이면 무해 |
| **Simulator (initialize/update/applyEnvironmentForces/reset)** | **done** | GL-free SIM-CORE만 |
| Simulator GL/GUI(draw*/upload/debug*/selection/ghost) | **stubbed(미포팅)** | headless core에서 제외 |
| Bullet rigid backend(rigid_) | **stubbed** | default scene Rigid 0개 -> inert |
| Kinematic FK 어드밴스 | **stubbed** | default scene Kinematic 0개 |
| Profiler(FrameProfiler) | **stubbed** | null=fast path |
| **setupBasicScene 자유 함수** | **done(최소 목표)** | addCloth/addFloatMesh/addGround id 0/1/2 |
| **headless Runner + main** | **done(최소 목표)** | System->Simulator->setupBasicScene->initialize->loop update |
| --scene RunConfig / SimulatorBuilder | **stubbed** | 대체 경로, 미필요 |
| --demo-uniform | **stubbed** | 대체 경로, 미필요 |
| GlobalAutoAllocator 인스턴스화(다중 scene) | **stubbed** | 전역 싱글톤 유지(D-A4) — 최대 제약, 다음 패스 |
| Scene 정적 싱글톤 -> 인스턴스 | **부분** | 소유 인스턴스로 이동, lifetimeId 의미 보존(D-A5) |
| metal 셰이더 재사용 (6 .metal + .metalh) | **done** | 변경 없이 metallib 빌드 룰 복제 |
| arch-test CMake 타깃 + metallib 룰 | **done** | GL/ImGui/Bullet/Assimp 미링크 |
