# arch-test 포팅 맵 (Source → Destination)

> 본 문서는 `arch-test/REFACTOR_PLAN.md`의 파일 레이아웃(§2)을 **목적지**로 삼아,
> 기존 `src/main.cpp`(~약 20000줄)와 `src/metal/*.metal`의 각 부분이 **어디로 어떻게** 옮겨가는지를
> source→destination으로 매핑한 문서이다. 아키텍처(ICDPipeline/ISystem/IBVH 가상 인터페이스 + 팩토리,
> karras12 LBVH 디폴트, Scene=정적 / SimState=라이브 분리, headless Runner가 `setupBasicScene(Simulator&)` 구동,
> `src/metal/*.metal` 무변경 재사용)는 **확정 사항**이며 여기서 다시 논쟁하지 않는다.
> 모든 라인 범위·커널명은 subsystem map JSON과 `src/main.cpp` / 루트 `CMakeLists.txt` 실측에 근거한다.
> `src/main.cpp`는 **건드리지 않는다**(새 엔진은 `arch-test/` 아래에만 짓는다).

포팅 액션 어휘:
- **verbatim-reuse**: 거의 그대로 복사(GL/ImGui 무접촉, 순수 compute).
- **port**: 로직 그대로 옮기되 정적 싱글톤→인스턴스, SimState/Scene 분리 등 구조 이동 동반.
- **adapt-to-virtual**: 가상 인터페이스(IBVH/ICDPipeline/ISystem) 뒤로 넣는 concrete로 변환.
- **stub**: 인터페이스/enum 골격만, concrete는 다음 패스(미구현).
- **defer**: 이번 패스 미포팅(default scene 경로에서 inert).

---

## 1. 매핑 표

### 1.1 backend + memory + Metal context (foundational, main 50–660)
포팅 난이도 easy. GL 무접촉(단, DebugLineGL/DebugPointGL 제외).

| src (main.cpp) | dest (arch-test/src/...) | action | gotchas |
|---|---|---|---|
| `Backend`/`CPU`/`CUDA`/`METAL` 태그 50,54-57; `using Index = uint32_t` 50 | backend/Backend.hpp | verbatim-reuse | Index=uint32_t는 전역 일관(모든 kernel arg 구조체). CUDA 태그는 specialization 0개 → placeholder/드롭. |
| `MemoryBlock<BE,PR>` 69-77(primary+CPU), 271-278(METAL) | backend/MemoryPool.hpp | verbatim-reuse | METAL은 buffer handle+byte offset(setBuffer 바인딩용) AND host ptr(contents()-mapped) 둘 다 보유 — 통합메모리 가정. |
| `ByteMemoryPool<CPU/METAL>` 78-120, 279-340 | backend/MemoryPool.hpp | verbatim-reuse | **METAL 256B alignment round-up 보존**(buffer 바인딩 필수). CPU=alignof(PR). |
| `FakeMemoryPool<BE>` 342-372 | backend/MemoryPool.hpp | verbatim-reuse | dry-run 사이즈 계산; 실제 alloc 전 총바이트 산정에 사용. |
| `DynamicByteMemoryPool<BE>` 374-487 (alloc 400-428, resetMarkers 482-485, pack 454-471) | backend/MemoryPool.hpp | verbatim-reuse | **resetMarkers cursor-replay 불변(D-041) 절대 보존** — re-init이 동일 주소 재현 안 하면 live buffer alias·corrupt. count==0 short-circuit 유지. |
| `DynamicMemoryAllocator<BE>` 489-507 | backend/MemoryPool.hpp | verbatim-reuse | 위임만. |
| `GlobalAutoAllocator<BE>` 509-531 (globalInitialize/reset/alloc 514-530) | backend/MemoryPool.hpp | port | **프로세스-와이드 하드 싱글톤(BE만으로 키)**. 다중 scene의 최대 제약(D-A4) — 이번 패스 그대로 유지, 인스턴스화는 defer. reset()은 Simulator::initialize 머리에서 호출. |
| `VectorBase<CPU/METAL>` 122-123,557-579,581-613 (METAL 슬라이싱 ctor 605) | backend/VectorBase.hpp | verbatim-reuse | **CPU엔 슬라이싱 ctor 부재(D-A6)** → pack 슬라이싱 경로는 METAL 전용 = 사실상 METAL-only로 진행. |
| `Matrix<BE,PR>` 617-627, `SparseMatrix<BE,PR>` 629-642 | backend/VectorBase.hpp | stub | CPU 전용, GPU-first에서 미사용. |
| `MetalGlobalContext` 126-199 (encoder 138-143, dispatchThreads 158-184, commitAndWait 185-197) | backend/MetalContext.hpp | verbatim-reuse | **commitAndWait null-encoder no-op 가드(D-030) 보존** — mixed cpu/gpu part 사이 CPU-only 경로에서 필요. commitAndWait = per-phase GPU sync barrier(sync-floor 병목). |
| `MetalKernelContext` 200-265 (getLibrary/getFunction/getPSO 201-264) | backend/MetalContext.hpp | verbatim-reuse | getLibrary가 `ysim_paths::runtimeFile`로 exe-dir 옆 `default.metallib` 로드(§3 참조). 실패 시 exit(1). |
| `DebugLineGL<CPU>`/`DebugPointGL<CPU>` 661-712,715-749 | (미포팅) | defer | **순수 GL VAO/VBO** — headless core에 링크 금지(§2 분리 지점). 데이터 생산측 host float*만 core, GL 업로드/draw는 별도 renderer. |

### 1.2 core/Types + BehaviorParams (main 761–2010)
포팅 난이도 medium. GL 무접촉.

| src | dest | action | gotchas |
|---|---|---|---|
| `ShapeType`/`BehaviorType`/`PlaneDirection` 761-780,1173-1177,1774-1780 | core/Types.hpp | port | **BehaviorType append-only(value 직렬화)**, ShapeType numeric order는 GPU-visible(meshShapes buffer). |
| `IndexPair` 1749-1760 (alignas(8) union + operator<) | core/Types.hpp | verbatim-reuse | metal mirror=uint2. dedup용 operator< 보존. |
| `BroadCollision` 1860-1865, `NarrowCollision` 1867-1873, `AnalyticShape` 1883-1892 | core/Types.hpp | verbatim-reuse | **byte-identical to common_types.metalh** — alignas(32/16) + 필드순서 정확히. static_assert(sizeof) 동반. AnalyticShape=80B. |
| `Material` 2140-2146, `SceneEnvironment` 2753-2769 | core/Types.hpp | port | 렌더-facing이나 GL 호출 0 — gravity/wind만 sim 사용, light/bg는 carry-only. |
| `SelectionMode` 1786-1789, `behaviorTypeName`/`shapeTypeName` 1814-1826,1849-1858, `bvhAssetDir`/`listBVHFiles` 1831-1847 | (미포팅) | defer | 렌더/picking/GUI 라벨/asset-dir — sim core 제외. |
| `Constraints<BE,PR>` 1894-1930 (fixedParticles만 active) | core/Scene.hpp | port | collision 멤버는 주석처리 상태(PackedCollisionData로 이주) — 부활 금지, fixedParticles 마스크만. |
| `FixedVertex` 1796-1799, `ReferencePointConstraint` 1809-1812 | core/Scene.hpp / core/Types.hpp | port | request에 저장, ref pair는 Explicit copy 커널이 소비. |
| `ClothBehaviorParams` 1932-1936, `FastGridClothBehaviorParams` 1952-1960, `FloatBehaviorParams` 1962-1963, `BehaviorParams` variant 1965-1970 | core/BehaviorParams.hpp | verbatim-reuse | **ClothBehaviorParams는 physics.metal ClothParams와 byte-identical(필드명만 stretch↔kstretch 다름)**, FastGrid는 ClothGridParams와 byte-identical. |
| `recomputeFastGridRest<PR>` 1982-2006 | core/BehaviorParams.hpp | verbatim-reuse | CPU, pack-time. default 경로엔 FastGrid 없음 → 정의만. |
| `TriangularClothBehavior<METAL>` 2009-2074, `FastGridClothBehavior<METAL>` 2076-2138 | system/ClothBehavior.hpp | port | setBuffer의 하드코딩 offset++ 인덱스 테이블이 MetalGlobalContext에 최밀착 — 백엔드 구현 뒤에. **vertColFacets slot5/Offsets slot6** 바인딩이 collision→integrator 데이터 경로(§4). FastGrid는 미디스패치. |

### 1.3 SimState / Scene 분리 + mesh initializers (main 789–3462)
포팅 난이도 hard. **이번 패스의 핵심**. GL 무접촉(Kinematic 제외).

| src | dest | action | gotchas |
|---|---|---|---|
| `MeshState<BE,PR>` 789-809 (x,xPrev,v,f,m,n) | core/SimState.hpp | port(분리) | **라이브 x/xPrev/v/f/n/externalForces → SimState로**, m(mass)는 immutable로 SimState. xPrev=substep 시작 스냅(swept CCD). |
| `Scene::PackedMeshData` 2904-2936 (live + topology 인터리브) | core/SimState.hpp(라이브분) + core/Scene.hpp(topology분) | port(분리) | **라이브 x/xPrev/v/f/n/externalForces는 SimState contiguous 버퍼+per-mesh sub-view로**, topology(facets/edges/offsets/vertObj)는 Scene에. |
| `MeshAdjacency<BE,PR>` 811-848 | core/Scene.hpp | port | 정적 topology: facets/edges/rest*/vertexAdj*(+Offsets)/vertexOpp*. pack이 packed 버퍼로 슬라이싱. |
| `ExternalForces<BE,PR>` 784-787 | core/SimState.hpp | port | per-frame externalForces(numPoints*3) — SimState에. |
| `Scene<BE,PR>` 2771-3462 (inline static 전부) | core/Scene.hpp | port | **정적 싱글톤→소유 인스턴스(D-A5)**. lifetimeId(BVH skip-cache용, 절대 리셋 안 됨)/numMeshes/nextMeshId 의미 보존. request-vs-realized 2-list(블루프린트가 persistent source) 유지. |
| `Scene::RequestGeneralMesh` 2792-2860 | core/Scene.hpp | port | pack/reset 가로지르는 블루프린트(initializer ptr 정식 소유자). |
| `GeneralMesh<BE,PR>` 2624-2733 (initialize 2723-2730) | core/Scene.hpp | port | realized mesh. GL-free(D-011). state는 SimState 뷰를 가리키도록 재배선. |
| `Scene::pack` 3081-3440 | core/Scene.cpp | port(분리) | **realization/compaction의 중심**: topology는 Scene packed 버퍼, **라이브분은 SimState로 슬라이싱**. positions bake, xPrev=x seed, pins 적용 = 둘 다 시드. **dynamic_cast cascade → initializer virtual `shapeType()`로 교체**(Assimp sibling 문제 회피). METAL 슬라이싱 ctor(605) 의존. |
| `Scene::initialize`/`findById`/`refreshAnalyticShapes`/`packAnalyticShapes` 3442-3450,3456-3461,3027-3068,3072-3079 | core/Scene.cpp | port | soft-reset/조회/analytic 재filling. analytic은 INERT(carry). |
| `InitializerParams<PR>` 856-871, `GeneralMeshInitializer<BE,PR>` 872-886 | mesh/IMeshInitializer.hpp | verbatim-reuse | **깨끗한 가상 seam(initialize/getParams/populatePreview)** — 새 팩토리가 미러. |
| `MeshAdjacencyInitializer` 888-1171 (initialize 892-1109, recomputeRestLengths 1110-1170) | mesh/IMeshInitializer.hpp | verbatim-reuse | CPU edge-dedup/opposite-vertex/CSR build + rest length. EdgeInfo 850-854 scratch 동반. |
| `MeshGridInitializer(+Params)` 1180-1391 (applyClothJiggle 1375-1388) | mesh/GridInitializer.hpp | verbatim-reuse | NxN grid, 대각 교번 삼각화, 결정적 seeded jiggle(seed^id). default cloth/ground 소스. |
| `MeshFileInitializer(+Params)` 1393-1473 | mesh/FileInitializer.hpp | verbatim-reuse | OBJ 로더(ObjData::loadObject) — Human.obj. |
| `AssimpMeshFileInitializer` 1479-1553 | (미포팅) | defer | MeshFileInitializer의 sibling(dynamic_cast 불일치). OBJ 로더로 충분, Assimp 미링크. |
| `MeshSphere/Cylinder/CubeInitializer` 1557-1746 | (미포팅) | defer | default scene에 primitive 없음(analytic INERT). |
| `MeshKinematicInitializer` 2305-2621 (create 2407-2421, writePose/writePoseBase 2460-2480) | (미포팅) | defer | default scene에 Kinematic 없음. GUI 재생/모션그래프 상태가 geometry에 섞임 — 헤드리스는 FK 코어만 분리 가치, 이번 패스 stub. |
| `PreviewState<PR>` (include/PreviewState.hpp:24-145) | mesh/PreviewState.hpp(기존 재사용/얇은 래퍼) | verbatim-reuse | heap-owned(pool 아님), GL-free, sim↔renderer 브리지. headless에선 syncPreview 선택적. |

### 1.4 collision: IBVH + Karras12 + TLAS (main 4965–7361, 3470–3535)
포팅 난이도 hard. GL은 showBox/showSceneBox/debugBox만.

| src | dest | action | gotchas |
|---|---|---|---|
| `AABB4` 5000-5096 (+ Ray/RayHit, rayTriangleIntersect 5073-5095) | collision/AABB4.hpp | verbatim-reuse | **32B union(min,i0,max,i1) 보존**(i0/i1=scratch: numPrimitives, query pid/objid). static_assert(sizeof==32). |
| `RadixSorter<METAL,Element>` 3473-3535 (primary 3470-3471) | collision/RadixSorter.hpp | verbatim-reuse | 4-pass 8-bit LSD(BLOCK_SIZE=1024). MortonNode로 인스턴스화. RadixParams ABI 보존. |
| `BVHMODE`/`BVHPRIMITIVE` enum 4967-4977 | collision/IBVH.hpp | port | LINEAR/SCENE/SAH; OBJECT=0/POINT=1/EDGE=2/TRIANGLE=3(PRIMITIVE 인자 AND 정점 stride). |
| `BVH<LINEAR,PRIMITIVE>::MortonNode` 5102-5106, `BVHNode` 5107-5136, `QueryFlag` 5319-5323 | collision/Karras12BVH.hpp | verbatim-reuse | **MortonNode 8B, BVHNode 32B**(childA==-1=leaf, childB=primitive id). static_assert 전부 carry. |
| `BVH<LINEAR,PRIMITIVE>` 5100-6931 의 Karras 기본경로: ctor 5326-5361, memoryAllocation 5363-5388, build 6075-6180, fillMortonsGPU 5718-5727, radixSortGPU 5685-5687, buildLeafGPU 5616-5627, buildTreeGPU 5628-5640, bottomUpBoxesGPU 5418-5436, refit 6434-6487, enlargeTrajectory 6489-6563, buildSweptLeaf/refitSwept 5951-6000, combineStaticOnce 6575-6579, queryPoints/queryBegin/queryEnd 6669-6718, objectRootAABB 6067-6073 | collision/Karras12BVH.hpp/.cpp | adapt-to-virtual (**DEFAULT concrete**) | **보존 필수**: expandBits/mortonCode 10-bit interleave(CPU↔metal 일치), Karras findSplit/determineRange delta+tie-break, leafSlot=N+id-1, treeParent population, **bottomUpBoxes atomic+fence 계약(bvh.metal 510-612, WEDGE/INDEX 가드)**. **build는 1 commitAndWait로 끝, refit은 commit 안 함**(caller가 배치). **combineStaticOnce**(static mesh under-combine 워크어라운드 — ground에 필요). |
| `buildCPU`/`fillMortonsCPU`/`radixSortCPU`/`bottomUpCombine`/`queryAABB`/`queryClickRay` 6182-6404,5688-5717,5642-5684,6413-6432,6581-6653,6835-6886 | collision/Karras12BVH.hpp (옵션) / (미포팅) | defer | CPU 레퍼런스/picking — GPU 디폴트라 hot path 아님. part-factory CPU 옵션은 다음 패스(CPU VectorBase 슬라이싱 선결). |
| Apetrei agglomerative state 5159-5184 + agglomerativeBuild/SwapRootGPU 5568-5613 | collision/(별도 concrete, 미생성) | stub | 별도 IBVH concrete, 다음 패스. |
| Sub-object multi-root 5186-5219,5735-6073 + grouped 커널 | collision/(별도 concrete, 미생성) | stub | square-cloth 전용 별도 concrete. |
| queryPointsSegmented/scanReserve/compact 6757-6825 | (미포팅) | defer | A/B 실험. |
| `BVH<SCENE,OBJECT>` (TLAS) 6935-7361: build(Scene&) 6974-7020, refit 7033-7094, enlargeTrajectory 7096-7141, refitSwept 7149-7189, detectCollisions 7207-7275, queryBegin/queryEnd 7199-7206,7337-7347, checkSelfCollisions 7191-7197, objectRootAABB intersect loop | collision/Karras12BVH.cpp (TLAS 래퍼) | adapt-to-virtual | objTrees(per-mesh TRI_LBVH) + EDGE top tree. **refit() commit 안 함 → 래퍼가 ONE commitAndWait 배치 후 root AABB 읽기(per-object commit은 ~34% 느림)**. **detectCollisions는 NON-dedup ordered double loop**(양방향 등록, 생성순서 의존 응답 수정). Float/Kinematic query skip. lifetimeId로 정적 Float subtree 캐시 skip. |
| `detectCollisionsSegmented` 7282-7336, `showBox`/`showSceneBox` 7349-7351,6891-6930, `BVH<SCENE,PRIMITIVE>` 빈 stub 7363-7365 | (미포팅) | defer | segmented=A/B, showBox=GL(§2), 빈 stub 드롭. |
| `BVHFactory` (신규; main 9588-9723 initialize의 `YSIM_BROADPHASE` 훅을 팩토리화) | collision/BVHFactory.hpp | stub | enum(Karras12/Apetrei/SubObject/SpatialHash/MultiLevelHash) + part별 cpu/gpu 골격; **현재 Karras12만 반환**(나머지 throw/assert). 매 프레임 toggle push 대신 build 시 concrete 1회 결정. |

### 1.5 대체 broad phases (main 3538–4961)

| src | dest | action | gotchas |
|---|---|---|---|
| `SpatialHashing<METAL>` 3538-4473 | collision/(별도 concrete, 미생성) | stub | 단일레벨 Pabst uniform-grid 별도 concrete. |
| `MultiLevelSpatialHashing<METAL>` (hgrid) 4476-4961 | collision/(별도 concrete, 미생성) | stub | hgrid 별도 concrete(home-only insert + cross-level walk). |

### 1.6 narrow phase + ICDPipeline (main 7371–7724, 10080–10203)
포팅 난이도 medium. **GL 무접촉**(전체 헤드리스).

| src | dest | action | gotchas |
|---|---|---|---|
| `BruteForce<METAL>` 7393-7714: ctor 7396-7399, narrow 7420-7456, narrowAndSortByVertices 7521-7581 (CPU counting-sort 7555-7580) | collision/narrow/BruteForceMetal.hpp | port | narrow_pt_tri 디스패치 + **CPU counting-sort → vertColFacets/Offsets**(global vid=statesOffsets[query]+point). NO commit(caller). resetNarrow 순서 보존. CPU sort는 METAL mapped memory 직접 walk(자연스러운 cpu/gpu seam). |
| `BruteForce<METAL>::NarrowParams` 7400-7408, `AnalyticNarrowParams` 7411-7419 | collision/narrow/BruteForceMetal.hpp | verbatim-reuse | **setBytes ABI byte-match common_types.metalh** — 필드순/타입 정확히. skipSphere 게이트(tri↔analytic 이중공급 방지). |
| `narrowAnalytic` 7477-7519, `narrowCPU`/`narrowAndSortByVerticesCPU` 7583-7713 | (미포팅) | defer | analytic=INERT(default off). narrowCPU=레거시 미사용 레퍼런스. |
| `BruteForce<CPU>` 7374-7389 (빈 stub) | (미포팅) | defer | 원본도 빈 stub — 포팅할 CPU narrow 없음. |
| `INarrowPhase`(신규; 얇은 인터페이스) 7371-7372 위치 | collision/narrow/INarrowPhase.hpp | stub | DefaultCDPipeline가 소유. |
| `CollisionPipeline<B,N>` POD 7716-7724 + Simulator substep 인라인 orchestration 10080-10203 | collision/ICDPipeline.hpp + DefaultCDPipeline.hpp | adapt-to-virtual | **CollisionPipeline은 메서드 없는 POD bag** — 진짜 dcd/ccd 흐름은 Simulator substep 루프에 인라인. 그걸 메서드화: ICDPipeline 가상 build/refit/dcd/ccd. DefaultCDPipeline: **dcd = (refit/enlargeTrajectory →) detectCollisions → narrowAndSortByVertices(swept 융합)**; **ccd = no-op(D-A1)**(CCD는 narrow_pt_tri에 xPrev로 융합). detectCollisions의 2-arg(SH)/3-arg(BVH+analytic) 표면차/checkSelfCollisions/refitSwept를 인터페이스로 화해. |

### 1.7 ISystem / ExplicitSystem (main 11021–11389)
포팅 난이도 medium. **GL 무접촉**(검증됨).

| src | dest | action | gotchas |
|---|---|---|---|
| `ExplicitSystem<METAL>` 11172-11389: ctor 11211-11229, buildRefPairs 11237-11273, dispatchRefKernel 11277-11287, update 11320-11388 | system/ExplicitSystem.hpp/.cpp | adapt-to-virtual (concrete) | symplectic-Euler 2-pass. **copy_pos → force → copy_force → integrate 단일 encoder serial 순서로 fence**(명시 barrier 없음). integrate에 anomalyFlag slot20(tri)/slot12(grid) 바인딩. |
| `ExplicitSystem::SimParams` 11289-11293, `ClothParams` 11303-11306, `ClothGridParams` 11295-11302 | system/ExplicitSystem.hpp | verbatim-reuse | **SimParams/ClothParams byte-match physics.metal**. ClothGridParams는 update()에서 dead(Behavior가 FastGridClothBehaviorParams를 slot9에) — 화해 전엔 carry 불필요. |
| `ISystem`(신규) 11021-11022 위치 + 표면 재정의 | system/ISystem.hpp | adapt-to-virtual | 가상 accumulate(=force pass)/integration(=integrate pass)/recoveryPenetration(=integrate 융합, D-A2). 매핑 근거 §아래. |
| `ExplicitSystem<CPU>` 11024-11168 (dead stub) | (미포팅) | defer | 원본도 dead. |
| `compute_forces`/`compute_spring_forces` PSO 캐시(미디스패치) | (미포팅) | defer | 레거시 — 캐시되나 update()가 디스패치 안 함. |

ISystem 가상 메서드 ↔ 현 코드 매핑(이름 불일치, D-A2/D-A3):
- `accumulate` = force pass(`compute_tri_spring_forces` 디스패치). **환경력(gravity*mass+wind) 주입은 System 아님 → Simulator::applyEnvironmentForces가 externalForces에 미리 채움(D-A3)**.
- `integration` = pass2 integrate 커널(`integrate_cloth`).
- `recoveryPenetration` = **standalone 없음**; integrate 커널 안 vertColFacets 읽어 vn-zeroing+(thickness-distance)*n push로 융합(D-A2). naive whole-contact 재사용은 fixed-push integrator를 폭발시킴(main 10017-10037 근거) — 융합 계약 유지.

### 1.8 Simulator 허브 (main 7726–11020 중 SIM-CORE만)
포팅 난이도 hard. **carve line은 깨끗·검증됨**(9777-10241은 gl*/ImGui* 0개).

| src | dest | action | gotchas |
|---|---|---|---|
| `Simulator<BE,PR,System>` 7726-11020 (SIM-CORE만) | sim/Simulator.hpp/.cpp | port | 소유: Scene(인스턴스) + SimState + `unique_ptr<ICDPipeline>` + `unique_ptr<ISystem>`(팩토리 주입). GL/GUI 전부 제외(§2). |
| `Simulator::initialize` 9588-9723 | sim/Simulator.cpp | port | **GlobalAutoAllocator::reset → Scene::pack(SimState 시드) → broadphase fresh build → anomalyFlag alloc**. YSIM_* 훅은 선택적. rigid/kinematic 블록 inert → defer. |
| `Simulator::update` 9777-10241 (substep 루프 10008-10204) | sim/Simulator.cpp | port | **보존**: substep 순서[dcd(refit→enlarge→detect, narrow) → xPrev 스냅(Float/Kinematic 예외) → system step] → **프레임당 단일 commitAndWait(10208)** → frame++ → targetFrames 정지. cadence gate refit/cdSubstepPeriod(i%period==0이 i==0에 발화). anomalyFlag poll/auto-pause(9795). |
| `Simulator::applyEnvironmentForces` 9731-9775 | sim/Simulator.cpp | port | gravity*mass+wind → SimState.externalForces. Float/Rigid/Kinematic는 0. |
| `Simulator::reset` 9577-9586 | sim/Simulator.cpp | port | kinematic 클록 rewind 후 initialize(). |
| `syncPreviewFromState` 10250-10266 | sim/Simulator.cpp(선택적 hook) | port | CPU-only(GL 무접촉)이나 render-oriented — hot step 밖 선택적 projection. |
| `draw`/`drawIds`/`drawDepth`/`drawPointIds`/`drawSelectablePoints`/`drawGhostPreviews`/`uploadMeshes`/`debug*`/`showDebugLines` 10272-10537,7927-7997 | (미포팅) | defer | **전부 GL/렌더(§2)** — headless core 제외. |
| `ensureGhostGL`/`ensurePreviewClips`/`advancePreviewPlayback` 7851-7925, `ensureRigidBackendBody`/`StaticGround` 8054-8120 | (미포팅) | defer | GUI/preview + Bullet rigid. default scene Rigid 0개 → inert. |
| `BulletRigidPhysicsBackend rigid_` (8006 member) | (미포팅) | defer | default scene Rigid 0개. |
| `translateObject`/`rotateObject`/editors 8733-9176, selection/hover 멤버 | (미포팅) | defer | GUI-edit 지원. |

### 1.9 scenes + runner + main (main 8146–8730, 18019–18306, 17710–18017)
포팅 난이도 easy. default scene 레시피 정확 재현.

| src | dest | action | gotchas |
|---|---|---|---|
| `addCloth` 8299-8317, `addFloatMesh` 8155-8162, `addGround` 8683-8697 | scenes/basic_scene.hpp | port | 자유 함수 `setupBasicScene(Simulator&)`로. **id 0/1/2 순서 보장**(id==배열슬롯==packed-data subscript==objPair). addX는 큐잉만(CPU). |
| `addGeneralMesh` 2870-2901 | core/Scene.cpp | port | 모든 addX 뒤 단일 식별/순서 소스. populatePreview 호출(headless에선 no-op 가능). |
| default scene 본문 main 18282-18288 + system/sim ctor 18230-18233 | scenes/basic_scene.hpp + runner/Runner.hpp | port | `addCloth(50,1,{0,1.25,0},1e5,1e5,2e5,0.01,0.1)` + `addFloatMesh(assetRoot(),"Human.obj",{0,0.35,0},0.04)` + `addGround(XZPlane,{0,0,0},50)`. **System h=1/60, subSteps=60**(main이 60 전달; ctor 디폴트 50 — default값은 60). margin=0.015, radius=0.012, targetFrames=300. cloth=2500 verts/4802 tris. |
| `addPlane`/`addClothGridFast`/`addSphere`/`addCube`/import* 8146-8730 | (미포팅) | defer | default scene 미사용. |
| registerPreviewBindingForLastRequest / scene_log::logObject | (no-op) | stub | addX 내부 — headless no-op/제거. |
| bench/self-test 하네스 17710-18017,11401+ (resetScene+buildDefaultScene+initialize+update loop) | runner/Runner.hpp (형태 참조) | port | **이미 near-headless 레퍼런스 루프**(GL 없이 initialize/update). Runner: `System(1/60,60) → Simulator(system) → setupBasicScene(sim) → sim.initialize() → for(f<frames) sim.update();` frames 외부 공급. |
| resetScene 정적 클리어 시퀀스 17719-17729 | core/Scene.cpp(인스턴스 dtor/clear로) | port | 정적 멤버 클리어(meshes/requests/numMeshes=0/nextMeshId=0/dirty=true/environment={}) — 인스턴스화 시 소유 dtor로 대체. |
| `--scene`/RunConfig/SimulatorBuilder 17651-17700,18255-18289, `--demo-uniform` | (미포팅) | defer | 대체 경로. |
| main() GLFW/ImGui 블록 18324+,20290-20326 | (미포팅) | defer | **§2 — main()의 ~2400줄 GLFW/FBO/ImGui 전부 sim core 무관**. |

---

## 2. GL / ImGui 분리 지점

headless 엔진이 **절대 링크/호출하지 말아야 할** entanglement point 전수. 모든 라인은 backend/simulator/bvh map 실측.

1. **DebugLineGL / DebugPointGL** — `src/main.cpp` 661-712(Line), 715-749(Point). 순수 GL VAO/VBO 래퍼(glGenVertexArrays/glGenBuffers/glBufferData/glVertexAttribPointer/glBufferSubData/glDrawArrays(GL_LINES/GL_POINTS)). GLuint은 YGLWindow.hpp→glew/OpenGL 경유. → **미포팅**. 데이터 생산측(sim이 채우는 host float*)만 core, GL 업로드/draw는 별도 renderer 모듈에 둔다.

2. **BVH showBox / showSceneBox** — LINEAR `BVH::showBox()` 6891-6930(debugBox/debugBoxLines = DebugLineGL<CPU> 멤버 5316-5317), TLAS `showBox()` 7349, `showSceneBox()` 7351. 노드 AABB를 12 line/node로 그림. → **IBVH 인터페이스에 노출 금지**(헤드리스 core). 필요하면 별도 IDebugDraw / render-adapter. (`SpatialHashing`/`ML`의 showBox/showSceneBox 4467-4468,4958-4959는 이미 빈 stub.)

3. **Simulator 렌더/GUI 메서드** (전부 RENDER-ONLY, sim core 제외):
   - `uploadMeshes` 10272-10284 (syncPreviewFromState + renderState.updateBuffer, GL 접촉; **update()가 호출 안 함** — 명시 분리됨)
   - `draw(Program&)` 10286-10332, `drawIds`/`drawDepth`/`drawPointIds`/`drawSelectablePoints` 10338-10416 (id-FBO/shadow depth/point-id/vertex-overlay 패스, glPointSize/glDepthFunc/glReadPixels)
   - `drawGhostPreviews` 7927-7997 (glEnable(GL_BLEND), ghostGL_.draw)
   - `debugEachBoxes`/`debugSceneBox`/`debugCollisions`/`showDebugLines` 10418-10537 (broadPhase.showBox/showSceneBox 호출 → GL)
   - `ensureGhostGL`/`ensurePreviewClips`/`advancePreviewPlayback` 7851-7925
   - 멤버: `renderState`(MeshRenderState, ~7840), ghost MeshGL, `selectedObj`/`hoveredObj`/`hovered*Vert`/`selectionMode`/`cameraFollowObjId`, Program `debugLineShader`, DebugLineGL 멤버. → **전부 미포팅**.
   - 경계선: `syncPreviewFromState` 10250-10266은 CPU-only(GL 무접촉)이나 render-oriented → hot step 밖 선택적 hook으로만.

4. **main() GLFW/ImGui 블록** — `src/main.cpp` 18324+ (YGLWindow(1600,900), shadow FBO, id-pass FBO, ImGui 패널, cursor/mouse 콜백), step driver 20290-20326. scene 블록(18019-18306) 아래 ~2400줄이 전부 render/editor. → Runner는 이 중 **아무것도** 포함 안 함(scene 큐잉 + initialize + `simulator.update()` 한 줄 호출만).

5. **검증된 carve line**: `update()` 9777-10241 + `ExplicitSystem::update()` 11320-11388 + `BruteForce::narrow` 7420-7681은 gl*/ImGui*/glfw* 0개(grep 검증). 기존 `--self-test`(main 18027 dispatch, 본체 11401+)가 **이미 GL 컨텍스트 없이** initialize/update를 돌린다 — carve가 안전하다는 선례.

---

## 3. shader 재사용

`src/metal/` 6개 .metal + `common_types.metalh`는 GL/ImGui/CPU-entanglement 0개 → **전부 verbatim 재사용**(난이도 easy). 새 .metal 안 만든다.

### 3.1 metallib 빌드 와이어링 (루트 CMakeLists.txt 37-63 복제)
- 각 `.metal` → `.air`: `xcrun -sdk macosx metal -c ${METAL_SOURCE} -o ${AIR_FILE}` (file(GLOB) src/metal/*.metal 루프, add_custom_command per-file).
- 전 `.air` → 단일 `default.metallib`: `xcrun -sdk macosx metallib ${METAL_AIR_FILES} -o ${CMAKE_CURRENT_BINARY_DIR}/default.metallib`.
- `add_custom_target(MetalKernels ALL DEPENDS ${METAL_LIBRARY})`.
- arch-test 타깃은 이 룰을 **재사용/복제**하되, `MetalKernelContext::getLibrary()`가 `ysim_paths::runtimeFile`로 **exe-dir 옆** default.metallib를 찾으므로 custom command 출력 경로를 arch_test_runner 실행파일 옆으로 맞춘다.
- **struct ABI lock**: common_types.metalh + per-file uniform(SimParams/SHParams/MLParams/CellProp/SHEntry/AnalyticShape/RadixParams)이 새 C++ host mirror와 byte-match해야 한다(드리프트=silent corruption). AnalyticShape=80B/16-aligned, MLParams=plain 4B scalar(implicit padding 회피), CellProp=atomic_uint(host는 uint32_t로 읽음).
- **fast-math**: `xcrun metal`은 기본 fast-math → isnan/clamp-on-NaN 불신; physics.metal은 bit-pattern `nonFinite3` + world-bound clamp(`YSIM_WORLD_BOUND 1.0e4f`)에 의존 — 컴파일 플래그·가드 보존.
- **buffer-index 계약**: `[[buffer(N)]]` 하드코딩이며 single-root vs grouped/multi-root 변종 간 인덱스가 한 칸씩 다름 — host arg 테이블을 정확히 와이어.

### 3.2 kernel → 소비 엔진 part 매핑 (전 46 커널, census 출처)
PSO는 46개 전부 `getPSO(name)`으로 생성 가능하나, **default scene 경로에서 실제 디스패치되는 것**만 표시.

| 커널 (파일:라인) | 소비 part (dest) | default scene |
|---|---|---|
| `fillMortons_Tri` (bvh 42-64) | Karras12BVH 빌드 stage2 | **디스패치** |
| `radixCountBlock_8Bits`/`radixScanOffset_8Bits`/`radixScatter_8Bits` (radix 21-113) | RadixSorter stage3 | **디스패치** |
| `buildTree_Tri` (bvh 352-395) | Karras12BVH stage4(hierarchy+leaf+treeParent) | **디스패치** |
| `buildLeaf_Tri` (bvh 260-288) | Karras12BVH refit leaf | **디스패치**(refit) |
| `zeroVisitCounts` (bvh 464-509), `bottomUpBoxes` (bvh 510-612) | Karras12BVH stage5 combine | **디스패치** |
| `enlargeLeaf_Tri` (bvh 289-322) / `buildSweptLeaf_Tri` (bvh 323-351) | Karras12BVH enlargeTrajectory/refitSwept | **디스패치**(둘 중 경로별 택1) |
| `fillMortons_Edge` (bvh 65+), `buildTree_Edge` (bvh 420-463), `buildLeaf_Edge` (bvh 396-419) | TLAS EDGE top tree | **디스패치**(EDGE top tree) |
| `queryPoints` (bvh 1383-1508) | Karras12BVH broad query → broadCollisions | **디스패치** |
| `narrow_pt_tri` (bruteforce 41-171) | BruteForceMetal DCD/swept | **디스패치** |
| `compute_tri_spring_forces` (physics 319-379) | ExplicitSystem pass1 (TriangularCloth) | **디스패치** |
| `integrate_cloth` (physics 380-480) | ExplicitSystem pass2 (+inline penetration recovery) | **디스패치** |
| `ref_constraint_copy_pos`/`ref_constraint_copy_force` (physics 481-498,499+) | ExplicitSystem ref-constraint | **디스패치**(pairCount==0이면 무해) |
| `fill_vf_offsets` (bruteforce 172-204) | BruteForceMetal (필요시) | PSO만(또는 디스패치) |
| `buildTree_Tri_Grouped`/`buildLeaf_Tri_Grouped`/`enlargeLeaf_Tri_Grouped`/`buildSweptLeaf_Tri_Grouped`/`bottomUpBoxesMultiRoot` (bvh 613-883) | SubObject concrete | PSO만(미디스패치 — defer concrete) |
| `agglomerativeBuild_Tri`/`agglomerativeBuild_Edge`/`agglomerativeSwapRoot` (bvh 884-1218) | Apetrei concrete | PSO만(defer) |
| `bottomUpBoxesPartial` (bvh 1219-1382) | hybrid combine | PSO만 |
| `queryPointsSegmented`/`scanReserveSegmented`/`compactSegmented` (bvh 1509-1584+) | segmented query | PSO만(A/B) |
| `sh_buildBV`/`sh_reduceMaxRadius`/`sh_assignCells`/`sh_markStarts`/`sh_fillCellProp`/`sh_computePairCount`/`sh_broadPhase` (spatialhashing 64-363+) | SpatialHashing concrete | PSO만(defer) |
| `ml_buildBV`/`ml_assignCells`/`ml_broadPhase` (mlspatialhashing 71-187+) | MultiLevelSpatialHashing concrete | PSO만(defer) |
| `radixScatter_8Bits_PerBlock` (radix 114+) | radix 변종 | PSO만 |
| `narrow_pt_analytic` (bruteforce 205+) | analytic narrow | PSO만(INERT, default off) |
| `compute_forces`/`compute_spring_forces` (physics 80-95,96-170) | 레거시 | PSO만(update가 미디스패치) |
| `compute_cloth_grid_forces_fast`/`integrate_cloth_grid` (physics 171-225,226-318) | FastGridCloth | PSO만(default cloth=Triangular) |

요약: default scene이 실제 디스패치하는 커널은 약 16개(bvh-core 6 + EDGE TLAS 3 + radix 3 + queryPoints + narrow_pt_tri + physics 4). 나머지 ~30개는 metallib에 동봉되고 PSO만 생성(또는 defer된 concrete가 생기면 디스패치). 셰이더는 무변경이므로 전부 동봉이 비용 없음.

---

## 4. 검증 (headless Runner assert)

basic scene이 제대로 step함을 증명할 항목. 모두 **기존 `--self-test`/bench 하네스 선례**(main 18027 dispatch, 본체 11401-17632; bench 17710-18017)에 근거 — 이들은 **이미 GL 컨텍스트 없이** `sim.initialize()` + update 루프를 돌리며 host 포인터(`state.x.ptr`/`state.v.ptr`)와 `packedCollisionData` 카운터를 직접 읽어 단언한다.

1. **positions finite / no-NaN** — N 프레임 후 cloth(id 0) `SimState.x`의 모든 원소가 `std::isfinite`. 선례: self-test가 `state.x.ptr[i]`에 `std::isfinite` 적용(KIN-1 등 "FK playback advances ... (finite)"), bench가 `qBack` 성분 finite 체크. Runner도 매 프레임/종료 시 cloth+ground 전 정점에 동일 적용.

2. **anomalyFlag 미발화** — `system.anomalyFlag.ptr && system.anomalyFlag[0] != 0u`가 한 번도 참이 되지 않음. 선례: update() 머리(9795)가 매 프레임 이 플래그를 poll해 auto-pause; integrate 커널이 world-bound/NaN 가드(sanitizeIntegrateOutput)에서 set. 헤드리스 Runner는 pause 대신 "비정상 정지 신호"로 fail.

3. **cloth settles** — y=1.25에서 출발한 cloth가 ground(y=0) 위로 떨어져 안정. 선례: bench의 `settleFrames`(20) 단일-root 드레이프 루프 + self-test의 cloth mean vx 변화 단언(BDD-011). Runner: 충분 프레임 후 (a) cloth 최저 y가 ground 두께(thickness 0.01)±margin 위에 정착(터널링 없음 = Human/ground 관통 안 함), (b) cloth mean speed가 초기 자유낙하 대비 감쇠(정지 수렴), (c) 초기 자유낙하 구간엔 mean vy<0(중력 작동).

4. **contact count sane** — `packedCollisionData.numBroadCollisions[0]` / `numNarrowCollisions[0]`가 0 < count < maxNumCollisions(오버플로/폭주 아님). 선례: bench가 `pc.numBroadCollisions[0]`/`pc.numNarrowCollisions[0]`를 per-frame 읽어 로깅(`lastBroad`/`lastNarrow`). Runner: cloth가 ground/Human에 닿은 뒤 narrow contact가 양수이되 상한 미만, broad가 narrow보다 많거나 같음(broad⊇narrow). queryEnd의 stack/buffer overflow 플래그(QueryFlag)도 0.

(추가 회귀 선례: self-test BDD-009가 Float(ground) `state.x`/`state.v`를 **bit-exact** 단언 — Runner도 ground가 안 움직이는지로 Float 경로 무결성 확인 가능.)
