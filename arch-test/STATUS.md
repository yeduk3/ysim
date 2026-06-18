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
| Karras12BVH concrete | collision/Karras12BVH.hpp, AABB4.hpp, RadixSorter.hpp | **REAL** build/refit GPU (fillMortons→radix→buildTree→bottomUpBoxes); tree 검증(root AABB encloses, 2N-1 nodes). detectCollisions/queryPoints는 narrow 단계 |
| Scene topology (facets/edges/CSR/restLen) | core/Topology.hpp | **REAL** (grid 삼각화 + adjacency; buildFacetsOnly for colliders) |
| OBJ 로더 (Human.obj) | mesh/ObjLoader.hpp | **REAL** (24461 verts/48918 tris, scale+origin) |
| SceneBVH TLAS (cross-mesh broad) | collision/SceneBVH.hpp, BroadBuffers.hpp | **REAL** per-mesh BLAS + obj cull + queryPoints → BroadCollision pairs |
| Narrow phase (BruteForce<METAL>) | collision/DefaultCDPipeline.hpp | **REAL** narrow_pt_tri + CPU counting sort → vertColFacets |
| **충돌 drape (전체 default scene)** | (통합) | **REAL**: cloth가 Human.obj에 drape (clothY 1.25→0.48, broad 66k/narrow 549, coherent) |
| ICDPipeline + DefaultCDPipeline | collision/ICDPipeline.hpp, DefaultCDPipeline.hpp | **REAL seam**; narrow PLACEHOLDER, ccd no-op(D-A1) |
| ISystem 인터페이스 | system/ISystem.hpp | **REAL** |
| ExplicitSystem<METAL> | system/ExplicitSystem.hpp, KernelParams.hpp | **REAL** GPU 2-pass: compute_tri_spring_forces + integrate_cloth(접촉 융합); anomaly guard. CPU 템플릿은 free-fall 참조 |
| 통합 prereqs | core/SimState.hpp | **REAL** fixedParticles(pin)/빈 contact 버퍼/statesOffsetsGPU; Scene jiggle |
| Simulator (hub) | sim/Simulator.hpp | **REAL** (generic, virtual cd/sys 보유, substep 루프, frame당 1 commit, owns pool+LUT) |
| setupBasicScene | scenes/basic_scene.hpp | **REAL** (id 0/1/2 레시피 정확) |
| Runner | runner/Runner.hpp | **REAL** (headless run() + runMulti() N-sim demo) |
| **N-Simulator (per-sim allocator)** | backend/MemoryPool.hpp, sim/Simulator.hpp | **REAL** (O2): GlobalAutoAllocator active-pool swap; 2 sims coexist 검증 (sim0 buffers valid after sim1) |
| **LUT 런타임 데이터버스** | core/LUT.hpp | **REAL** (O1): typed name→handle dict + type_index + UpdatePolicy; Simulator.publish() 바인딩 |

PLACEHOLDER = 컴파일/실행되지만 실제 GPU 물리/충돌 미구현(seam만 입증).
각 PLACEHOLDER 파일에 포팅 출처(main.cpp 라인)를 1줄 `ponytail:` 주석으로 남김.

## 다음 포팅 (재 리팩토링 우선순위)
DONE: backend, core, spine, **N-Simulator(allocator)**, **LUT**, **Scene topology pack**,
**karras12 LBVH(GPU)**, **ExplicitSystem GPU 2-pass(springs)**, **OBJ 로더**,
**SceneBVH TLAS(cross-mesh broad)**, **narrow phase**, **A-full 충돌 drape**.
→ 전체 default scene이 새 엔진에서 end-to-end 동작: cloth가 karras12 broad+narrow+springs+
  recovery로 Human.obj에 drape (headless, N-Simulator, build green). **최소 목표 + A-full 달성.**

남은 (polish / 확장):
1. **swept CCD 분리(O5)** — narrow_pt_tri는 xPrev swept(needsXPrev). 별도 ccd 패스로 분리(셰이더).
2. **detect = 3 독립 스텝(O7)** — 현재 detectCollisions 안에 broad 묶임. broad/candidate/narrow 노출.
3. **OpenGL renderer + LUT 피드(O3)** — GUI 단계. SimState.x 핸들 LUT로.
4. **GUI 에디터(O8)** — add*/translate/rotate/scale CPU 큐.
5. **다른 BVH concretes** — Apetrei / SubObject / SpatialHashing / MultiLevelSH (BVHFactory에).
6. **per-part cpu/gpu 실구현(C6)** — 현재 BVHPartConfig는 골격(GPU만). CPU build/sort/combine 실연결.
7. **정리**: "[Pool] Tried to allocate more than tha capacity" 로그는 양성(pool auto-grow 신호)이나
   오해 소지 — 메시지 정리. clangd unused-include 경고(양성).

결정됐으나 미구현 (REFACTOR_REVIEW.md 2차):
- **DCD/CCD 분리**(O5) — swept 테스트를 별도 ccd 패스로(셰이더 작업). 4번 narrow와 함께.
- **detect = 3 독립 스텝**(O7) — broad/candidate/narrow를 ICDPipeline에 분리 노출. 3·4와 함께.
- **OpenGL renderer + LUT 피드**(O3) — GUI 붙일 때. headless 동안 보류.
- **GUI 에디터 CPU 큐**(O8) — add*/translate/rotate/scale CPU 부분 유지. GUI 단계에서.

## 보류/해소된 모호 결정
- **해소(REFACTOR_REVIEW.md 2차)**: LUT(O1,구현), N-Simulator(O2,구현), renderer=OpenGL+LUT(O3),
  pin/coincidence/contact 셋 다 유지(O4), DCD/CCD 분리(O5), loadFromConfig 폐기→setup(O6),
  detect=3스텝(O7), GUI 에디터 유지(O8).
- **여전히 보류**: Profiler 레벨, CUDA backend, hot-loop virtual-vs-template(실측 후), Kinematic 분리.

## 알려진 함정 (verify 지적)
- LUT "pos"/"vel"/"nrm"는 버퍼 ptr **값**을 bind → SimState realloc/rebuild 시 stale.
  현재는 publish()가 initialize() 끝에 재호출돼 안전(rebuild 경로 없음). **rebuild 추가 시 publish() 재호출 필수**.
- runMulti() distinct 판정이 float `==` — 데모용. 두 config가 근접하면 취약.

## 참고
- `arch-test/src/*.hpp` 루트의 빈 스케치 파일(BVHFactory/Scene/Simulator/SceneBuilder 등)은
  **원래 사용자 초안**. 실제 엔진은 `arch-test/src/{backend,core,collision,system,sim,scenes,runner}/`
  서브디렉터리. 빌드는 .cpp만 GLOB하므로 루트 .hpp 스케치는 빌드에 미포함(무해).
- 설계 근거 문서: REFACTOR_PLAN.md (레이아웃/순서), DECISIONS.md (clear/ambiguous),
  BVH_VERSIONS.md (BVH 버전 + per-part factory), PORT_MAP.md (src→dst 매핑).
