# REFACTOR.md 리뷰 — 의문 해소 / 잔여 의문

> `REFACTOR.md`(산문 청사진)를 실제 코드 + 이번 세션에 지은 spine(`arch-test/`)에
> 비춰 검토. **RESOLVED** = 코드로 확인됨(근거 라인 명시). **OPEN** = 여전히 의문.
> 섹션 번호는 REFACTOR.md 기준.

---

## RESOLVED (코드로 확인)

### R1 — `step() = dcd → accumulate → integrate → ccd → recover`는 *현재 코드엔 그대로 없다* (§2, §4·d 무관)
- `recoveryPenetration` 함수: **grep 결과 0건**. 존재하지 않음.
- 별도 `ccd` 패스/커널: 없음. CCD는 D-013 **swept 테스트가 `narrow_pt_tri` (DCD) 커널에 융합**
  (xPrev = 서브스텝 시작 위치, slot 10).
- 침투 복원: 별도 함수 아님 — `integrate_cloth` 커널이 `vertColFacets`(slot 5/6)를 읽어 in-kernel push.
- **결론**: 5단계 파이프라인은 *목표 시그니처*다. 실제는 `dcd(=swept 융합)` + `accumulate(force)` +
  `integration(침투복원 융합)` 3개. 새 엔진은 `ICDPipeline::ccd`를 노op, `ISystem::recoveryPenetration`을
  integrate 융합으로 매핑(현 spine이 이렇게 함). 진짜 분리하려면 셰이더 재작성 필요 → OPEN O5 참고.

### R2 — `ConstraintSet` 타입은 존재하지 않는다 (§2 ConstraintSet, §5 param-passing)
- 실제 "제약"은 3곳에 분산: (a) `Constraints::fixedParticles`(pin mask, main 1896),
  (b) `ReferencePointConstraint`(coincidence, main 1809), (c) `packedCollisionData.vertColFacets`(de-facto contact set).
- 청사진의 "Simulator-owned scratch, ccd writes / recoveryPenetration reads"는 (c) **한 가지에만** 부분적으로 맞음.
- **버퍼는 재사용하되 contents는 매 서브스텝 재계산** — naive 전체-contact 재사용은 fixed-push integrator를 폭주시킴
  (main 10017-10037, DECISIONS C21). "reused scratch"의 정확한 의미 = 버퍼 reuse, contact geometry recompute.

### R3 — `accumulate`는 환경력을 포함하지 않는다 (§2 System)
- gravity*mass + wind는 `Simulator::applyEnvironmentForces`가 `externalForces`에 채움(System 아님, D-A3).
- `System::accumulate` = spring force 커널(`compute_tri_spring_forces`) 디스패치만.
- 현 spine이 이 분담을 그대로 따름(Simulator.applyEnvironmentForces + ExplicitSystem.accumulate).

### R4 — **Scene·Allocator가 프로세스-와이드 싱글톤** → "Runner owns N Simulators"는 현재 불가 (§2 Runner, §3 goal, diagram)
- `Scene<BE,PR>`의 **모든 필드가 `inline static`**: numMeshes/meshes/environment/`packedMeshData`/
  `packedCollisionData`/referenceConstraints/dirty … (main 2773-3014; 7034 주석 "Scene<>의 모든 필드가 inline static").
- behavior는 `Scene<METAL,PR>::packedCollisionData`를 **정적으로** 참조(main 2037, 2103).
- `GlobalAutoAllocator<BE>`도 `inline static globalPool`(main 511), BE로만 keying.
- **결론**: 지금 N개 Simulator를 만들면 **모두 같은 Scene·같은 pool을 공유**하고 `reset()`이 서로를 짓밟음.
  Runner가 N Simulator를 소유하려면 Scene de-static화 + allocator 인스턴스화가 선결(연쇄 영향 큼). → OPEN O2.

### R5 — backend/memory/MetalContext "keep"는 검증됨 (§1, §5 keep-what-works)
- 그대로 이식 → 컴파일·링크·실행 확인(M3에서 metallib 로드 OK). spine `backend/*.hpp`가 실증.

### R6 — Simulator GL carve line은 깨끗하다, 단 새 엔진은 carve 안 함 (§1 Simulator, §6 step 2)
- update() 코어 path는 GL-free(--self-test가 GL 컨텍스트 없이 init/update 구동, 선례). carve 가능 확인.
- **단** 사용자 결정으로 **greenfield**(arch-test 신축)로 전환 → "carve GL out of Simulator"는 비적용.
  새 Simulator엔 애초에 GL이 안 들어옴. → §6 attack order는 in-place 전제라 폐기, STATUS.md의 port order로 대체.

### R7 — SimulatorBuilder dropped 확인 (§1, §5 No Builder)
- main 17652 존재(partial). 새 엔진은 free-fn `setupBasicScene(Simulator&)`로 대체 — **이미 동작**(spine).

### R8 — substep 수치 정정 (§5 virtual cost)
- 기본 scene `subSteps = 60`(ctor 디폴트 50 아님). 비용 ≈ 60×2×60fps ≈ 7,200 calls/s. 논지(무시 가능) 유지.

### R9 — narrow `BruteForce<CPU>`는 빈 stub (§1 Narrow phase)
- METAL narrow만 동작. CPU narrow/sort는 live 미연결 legacy. "BVH/SH/hgrid/BruteForce become parts"에서
  BruteForce는 **METAL만** 실재(DECISIONS C19).

### R10 — "virtual at phase granularity" 논지 타당 (§5)
- spine이 실증: vtable 비용은 commitAndWait(프레임당 1회 GPU sync) 대비 무의미. 규칙(inner loop엔 virtual 금지) 유효.

---

## STILL OPEN (여전히 의문)

### O1 — LUT 설계가 통째로 미정 (§2 LUT, §3 goal 5, §4·a, §5)
코드 어디에도 LUT 구현 없음. "render-out + params 두 view", "bind<T>", "cache the KEY"는 전부 *의도*.
무엇을 추상화하나? (a) GPU 버퍼 바인딩 테이블 / (b) GUI 파라미터 dict / (c) contact bus — 셋이 섞여 서술됨.
**해소 조건**: LUT가 실제로 무엇을 통과시키는지 owner가 1개 구체 use-case로 고정(예: "wind/gravity 파라미터 +
cloth position 버퍼 핸들"). 현 spine 미구현.

### O2 — Runner의 N-Simulator 모델 (R4의 귀결) (§2, §3, diagram, §7)
Scene·allocator 싱글톤 때문에 N Simulator 동시 보유 불가. 선택 필요:
- (안A) Runner = **단일 Simulator + config 순차 재초기화**(현실적, 현 구조 유지). 진짜 병렬 아님(§5 honesty와 일치).
- (안B) Scene de-static화 + allocator 인스턴스화 → 진짜 다중 Simulator(대공사, packedData 정적 참조 전부 수정).
**해소 조건**: sweep이 동시 N-sim을 요구하는가, 순차로 충분한가.

### O3 — render-split (IRenderer / SceneDescription) 경계 (§2, §3 goal 3, §6 step 2-3)
headless가 우선이라 범위 밖. GUI 붙일 때 render-facing 데이터(PreviewState/Material/light/bg)와 core 경계 미정.
**해소 조건**: 이후 GUI를 기존 OpenGL 재사용으로 갈지 새 백엔드로 갈지.

### O4 — ConstraintSet 통합 + 수명 모델 (§2, R2의 귀결)
pin(정적) vs coincidence(정적) vs contact(서브스텝 재계산) — 수명이 근본적으로 다른 셋을 한 타입으로 묶을지.
**해소 조건**: 통합 타입의 수명/소유(누가 clear, 언제 recompute)를 owner가 정의.

### O5 — DCD/CCD 진짜 분리 여부 (§2, R1의 귀결)
CCD가 DCD 커널에 융합돼 있어, `ccd()`를 진짜 별도 패스로 만들면 셰이더 재작성 필요 →
"src/metal verbatim 재사용" 전제와 충돌.
**해소 조건**: 분리 swept-CCD가 정확도/성능상 필요한가. 필요하면 verbatim 전제 부분 완화.

### O6 — loadFromConfig 직렬화 단위 (§2, §5 No Builder, §7)
Scene/SimState 분리 후 "무엇이 persist되나"가 바뀜(정적 topology만? 초기 SimState도?).
**해소 조건**: 분리 확정 후 직렬화 스키마 재정의(기존 RunConfig 재사용 범위).

### O7 — `detectCollisions` surface 비균질 (§2 CDPipeline parts)
BVH는 `detectCollisions(margin,self,analytic)` 3-arg + segmented/checkSelfCollisions/refitSwept,
SH/ML은 2-arg + 나머지 stub. 단일 인터페이스로 통일 시 capability 플래그 vs vtable 메서드 미정(DECISIONS A11).
**해소 조건**: broad-phase별 지원 매트릭스를 owner가 확정.

### O8 — Simulator의 editor/GUI 부분 영구 drop 범위 (§1 Simulator ~3300줄, §6)
step path만 새 엔진 core. add*/translate/rotate/scale/selection/ghost/draw* 중 어디까지 CPU 큐로 살리고
어디를 영구 버릴지 미정(headless-only면 add*의 CPU 큐만 필요). DECISIONS A8.
**해소 조건**: 새 엔진이 GUI 에디터를 가질지.

---

## 요약
- 청사진의 **데이터 흐름 시그니처(5단계, ConstraintSet, recoveryPenetration)는 목표지 현실 아님** — 현 코드는
  3단계 + 융합 + 분산 제약. 새 엔진은 인터페이스로 노출하되 매핑은 융합 계약 유지(R1·R2·R3).
- **가장 큰 미해결 구조 충돌 = Scene·allocator 싱글톤 vs Runner-owns-N**(R4 → O2). 다른 모든 OPEN보다 우선 결정 필요.
- §6 attack order(in-place carve)는 greenfield 전환으로 **폐기** → STATUS.md 포팅 순서가 대체.

---

## 의문 해소 — 2차 (사용자 결정 2026-06-18)

O1–O8 전부 결정됨. 각 결정 + 새 엔진에 미치는 영향(=추가 작업).

### O1 → DECIDED: LUT는 만든다 (범용 런타임 dict)
- LUT = `name → handle` dictionary. renderer / gui / framesnapshot 등 **여러 곳에서** 사용.
  Simulator ↔ GUI 런타임 데이터 교환 통로.
- **영향**: LUT를 실제 구현(이전엔 defer). `bind<T>(name,&v,policy)` + key-cache(invariant a) + dirty
  (live/rebuild-paused/restart-frame-0, 청사진 §5). Simulator가 소유, render-out 핸들 + param 핸들 둘 다.

### O2 → DECIDED: N Simulator 지원 필수 (병렬 아님, 실험 셋팅 보조)
- 동시에 step 돌리는 병렬 아님 — 여러 실험 셋업을 동시에 들고 전환하는 용도.
- **영향(구조 변경, 큼)**: R4의 싱글톤 둘을 인스턴스화해야 함.
  1. `Scene` 정적 멤버 제거 → Simulator-소유 인스턴스. (현 spine은 이미 인스턴스 Scene — 절반 완료.)
     단 `packedMeshData`/`packedCollisionData`를 정적 참조하던 behavior setBuffer(main 2037,2103)를 인스턴스
     참조로 전부 수정해야 함.
  2. `GlobalAutoAllocator<BE>` 싱글톤 → **per-Simulator allocator**. `VectorBase` ctor가 전역 싱글톤 대신
     allocator 핸들을 받도록 변경(전역 침투 변경). 또는 Simulator가 활성일 때 alloc 컨텍스트를 swap.
     → **가장 큰 잔여 작업. 다음 포팅의 선결.**

### O3 → DECIDED: GUI = 기존 OpenGL 재사용, renderer는 LUT로 먹임
- renderer에 줄 scene 데이터도 LUT에 저장 가능. **포인터를 LUT에 bind → renderer가 핸들로 데이터 pull**
  (invariant a, 매 프레임 re-lookup). Sim은 GL 타입 0개.
- **영향**: render 어댑터 = 기존 OpenGL 경로 재사용 + LUT 소비자로. 새 GL 백엔드 불필요.

### O4 → DECIDED: pin / coincidence / contact 셋 다 유지
- **pin**: 유지 — 코드레벨 고정부터 런타임 해소(release)까지. (`Constraints::fixedParticles`, runtime `fixParticle/releaseParticle`.)
- **coincidence** = `ReferencePointConstraint`(main 1809): follower 정점이 leader 정점 위치를 매 integration step
  추종(점-선택 패널 기능). pin처럼 영속, scene_format 라운드트립. → **유지**.
- **contact**: 매 서브스텝 재계산(버퍼만 reuse). 확정.
- **영향**: ConstraintSet 단일화 대신 **수명별 3종 분리 유지** — pin(영속, 런타임 가변) /
  coincidence(영속) / contact(서브스텝 scratch). 한 타입 강제 통합 안 함. coincidence는 매 integration step
  follower←leader 복사 스텝을 System/Simulator에 둠(현 코드 계약 보존).

### O5 → DECIDED: DCD / CCD 분리한다
- 분리 가능 → `ICDPipeline::dcd` / `ccd`를 진짜 별도 패스로. 현 코드의 swept 융합(D-013) 대신.
- **영향**: 셰이더 작업 필요(swept 테스트를 별도 ccd 커널/패스로 추출) → "src/metal verbatim 재사용" 전제를
  **CCD에 한해 완화**. narrow_pt_tri의 xPrev swept 분기를 분리 ccd로 떼어냄.

### O6 → DECIDED: loadFromConfig 폐기 → setup으로 대체
- JSON 경로(loadFromConfig / RunConfig / SimulatorBuilder) **안 만듦**. scene 구성은 `setup*()` free-fn 단일 경로.
- sweep/repro도 코드(setup 함수 + 루프)로. JSON 직렬화 친화 부담 제거.
- **영향**: 직렬화 스키마 재정의 불필요. 청사진 goal 2의 "두 경로"는 **setup 단일 경로**로 축소.

### O7 → DECIDED: detectCollisions = 독립된 3 스텝
- 단일 flagged surface로 통일하지 않음. 충돌을 **3개 독립 스텝**(broad / candidate(pair) / narrow)으로 노출.
  버전별 차이(self/analytic/segmented)는 각 스텝 내부에. 청사진 (A) composed 스타일이 기본.
- **영향**: `ICDPipeline`이 detectCollisions 한 메서드 대신 broad/narrow를 분리 스텝으로. capability 플래그
  vtable 고민(A11) 소거.

### O8 → DECIDED: 새 엔진은 GUI 에디터를 가진다 (선택적이나 필요)
- **영향**: Simulator의 add*/translate/rotate/scale 등 에디터 메서드의 **CPU 큐 부분 유지**(영구 drop 아님).
  GL/preview 바인딩 부분만 renderer/LUT 경유로 분리. headless Runner는 에디터 미사용으로 동작.

### 2차 결정이 바꾸는 것 (다음 포팅에 추가)
- **선결**: allocator 인스턴스화 + Scene 정적 참조 제거(O2) — N-sim의 토대.
- **추가 구현 대상**: LUT(O1), OpenGL renderer 어댑터 + LUT 피드(O3), 분리 ccd 패스(O5, 셰이더),
  CD를 broad/narrow 분리 스텝으로(O7), 에디터 CPU 큐 유지(O8).
- **폐기**: loadFromConfig/JSON 경로(O6) — setup free-fn 단일 경로로 대체.
- STATUS.md "다음 포팅" 순서는 allocator-instancing을 최상단에 넣어 갱신 필요(빌드 재개 시).
