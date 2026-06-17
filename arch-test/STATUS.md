# arch-test 엔진 — 이번 세션 실제 결과 (STATUS)

> 이 문서는 **실제로 이번 패스에 land 한 것**을 기록한다. REFACTOR_PLAN.md의
> done/stubbed 표는 *전체 포팅을 끝냈을 때*의 목표치이고, 이 STATUS는 *지금
> 빌드/실행되는 것*의 정직한 스냅샷이다. 재 리팩토링은 여기 "다음 포팅" 순서대로.

## 한 일 (git)
- 기존 작업(arch-test 청사진) 커밋 → `feat/ml-spatial-hashing` 푸시.
- 새 브랜치 `refactor/arch-engine` 생성, 그 위에서 작업.
- `src/main.cpp`(20,713줄)은 **건드리지 않음**. 새 엔진은 `arch-test/` 독립 빌드.

## 빌드 & 실행
```bash
cd arch-test
cmake -B build -S .        # 독립 프로젝트 (root ysim 빌드와 분리)
cmake --build build
./build/arch_runner [frames]   # 기본 300
```
- 독립 CMake: `../include` + `../include/metal-cpp` 재사용, `../src/metal/*.metal`로
  자체 `default.metallib` 빌드, Metal/Foundation/QuartzCore + Eigen만 링크.
  GL/ImGui/GLFW/Bullet/Assimp **미링크** (headless).
- 실행 결과 (M3): `objects=3 verts=2504`, 300프레임 완주, cloth가 중력으로 낙하
  (`clothY 1.25 -> -121.38`, free-fall ½·9.81·5² ≈ 122m 일치), positions finite, **PASS**.

## 최소 목표 — 달성
- ✅ 기본 scene이 자유 함수 `setupBasicScene(Simulator&)` (scenes/basic_scene.hpp).
- ✅ GL 없는 `Runner`가 그것을 구동 + 검증 (runner/Runner.hpp).
- ✅ 가상 인터페이스 전면: `ICDPipeline`/`ISystem`/`IBVH` + `BVHFactory`.
- ✅ Scene(정적) / SimState(라이브) 분리.
- ✅ karras12를 default IBVH concrete로, factory에서 per-part cpu/gpu config 노출.

## 실제 상태: REAL / PLACEHOLDER / DEFERRED

| 레이어 | 파일 | 상태 |
|--------|------|------|
| backend tags/Index/Precision | backend/Backend.hpp | **REAL** (verbatim) |
| MemoryPool 일체 | backend/MemoryPool.hpp | **REAL** (verbatim, 불변식 보존) |
| VectorBase/Matrix/SparseMatrix | backend/VectorBase.hpp | **REAL** (METAL 슬라이싱 ctor 포함) |
| MetalGlobalContext/KernelContext | backend/MetalContext.hpp | **REAL**, metallib 로드 검증됨 |
| 충돌/제약 타입 + enums + Material/Env | core/Types.hpp | **REAL** (byte-exact, static_assert 통과) |
| behavior params + recomputeFastGridRest | core/BehaviorParams.hpp | **REAL** (layout 보존) |
| SimState (x/xPrev/v/f/n/extF/m) | core/SimState.hpp | **REAL** (contiguous alloc) |
| Scene (정적 기술 + realize) | core/Scene.hpp | **REAL 구조** / grid·ground 지오메트리 CPU 시드 REAL / **topology(facets/adjacency) pack DEFERRED**, FileMesh(OBJ) 로드 DEFERRED |
| IBVH 인터페이스 + BVHFactory(per-part cpu/gpu) | collision/IBVH.hpp, BVHFactory.hpp | **REAL 인터페이스 + config 골격**; Karras12만 생성, 나머지 throw |
| Karras12BVH concrete | collision/Karras12BVH.hpp | **PLACEHOLDER** (no-op); GPU build pipeline 포팅 대상 |
| ICDPipeline + DefaultCDPipeline | collision/ICDPipeline.hpp, DefaultCDPipeline.hpp | **REAL seam**; narrow PLACEHOLDER, ccd no-op(D-A1) |
| ISystem 인터페이스 | system/ISystem.hpp | **REAL** |
| ExplicitSystem | system/ExplicitSystem.hpp | **PLACEHOLDER integrate** (CPU semi-implicit Euler free-fall, 스프링/접촉 없음) |
| Simulator (hub) | sim/Simulator.hpp | **REAL** (generic, virtual cd/sys 보유, substep 루프, frame당 1 commit) |
| setupBasicScene | scenes/basic_scene.hpp | **REAL** (id 0/1/2 레시피 정확) |
| Runner | runner/Runner.hpp | **REAL** (headless 구동 + 검증) |

PLACEHOLDER = 컴파일/실행되지만 실제 GPU 물리/충돌 미구현(seam만 입증).
각 PLACEHOLDER 파일에 포팅 출처(main.cpp 라인)를 1줄 `ponytail:` 주석으로 남김.

## 다음 포팅 (재 리팩토링 우선순위)
의존성 순서 = backend(완) → core(완) → **아래부터**:
1. **Scene topology pack** — MeshGridInitializer 삼각화 + MeshAdjacency(edges/CSR/restLengths)
   를 GPU 버퍼로 pack (main.cpp 1180-1391, 856-1171, 2771-3462). karras12·스프링의 선결.
2. **Karras12BVH GPU body** — fillMortons→radixSort→buildTree→bottomUpBoxes→queryPoints
   (main.cpp 5101-6934 + bvh.metal). RadixSorter, AABB4 동반. BVH_VERSIONS.md §4.
3. **BruteForce<METAL> narrow** — narrow_pt_tri + per-vertex counting sort → vertColFacets
   (main.cpp 7393-7681). DefaultCDPipeline.dcd에 연결.
4. **ExplicitSystem GPU 2-pass** — compute_tri_spring_forces + integrate_cloth(접촉 융합)
   (main.cpp 11172-11389 + physics.metal). behavior setBuffer 테이블.
5. **FileMesh(OBJ) 로드** — Human.obj (Float collider). assimp 불필요, OBJ 로더로.
6. 그 다음 concretes: Apetrei / SubObject / SpatialHashing / MultiLevelSH (BVHFactory에 추가).

## 보류된 모호 결정
DECISIONS.md (2)절 A1–A12 그대로 — LUT 버스, render-split(IRenderer), loadFromConfig,
ConstraintSet 통합, Profiler 레벨, CUDA, hot-loop virtual-vs-template, Simulator GL carve
범위, DCD/CCD 분리, GlobalAutoAllocator 인스턴스화, detectCollisions surface 통일,
Kinematic 분리. 미구현, 문서화만.

## 참고
- `arch-test/src/*.hpp` 루트의 빈 스케치 파일(BVHFactory/Scene/Simulator/SceneBuilder 등)은
  **원래 사용자 초안**. 실제 엔진은 `arch-test/src/{backend,core,collision,system,sim,scenes,runner}/`
  서브디렉터리. 빌드는 .cpp만 GLOB하므로 루트 .hpp 스케치는 빌드에 미포함(무해).
- 설계 근거 문서: REFACTOR_PLAN.md (레이아웃/순서), DECISIONS.md (clear/ambiguous),
  BVH_VERSIONS.md (BVH 버전 + per-part factory), PORT_MAP.md (src→dst 매핑).
