# Analytic broad-skip (c-2) vs BVH 재측정 (2026-06-05)

이전 c-1 실험([`analytic-collision-2026-06-04`](../analytic-collision-2026-06-04/README.ko.md))
은 "analytic 토글이 BVH 대비 무의미하다 — broad(BVH)를 못 건드리고 narrow 만
오히려 1.38× 느려진다"로 끝났다. 그 결론이 지목한 두 원인을 직접 고친 **c-2**
구현을 **동일한 방법**(같은 bench·같은 씬·18 case × 30 frame·같은 차트 3종)으로
재측정한다. 결론부터: **이제는 유의미한 이득이다.** analytic ON 에서
broad(BVH) 비용이 **0 으로 사라지고**, narrow 도 빨라져 physics_total 이 c-1 On
대비 평균 0.77×, 최선 **0.29× (3.4배)**.

## c-2 가 바꾼 것

| # | c-1 문제 (이전 실험이 지목) | c-2 수정 |
|---|---|---|
| 1 | broad(BVH)가 충돌비용을 지배하는데 analytic 토글이 broad 를 0% 절감 — 구가 BVH 에 남아 cloth-vs-구-삼각형 쌍을 그대로 생성 | broad 단계에서 (cloth, sphere) object-AABB 가 겹치면 **구의 triangle-BVH 로 descend 하지 않음**(`detectCollisions`). 구는 object-level 테스트만 받고 `analyticPairs` 마커로 남는다 |
| 2 | analytic ON 이 substep 마다 `commitAndWait` 2회(triangle + analytic) | tri + analytic 을 **같은 command buffer** 로 인코딩 → **commitAndWait 1회** (`narrowAndSortByVertices`) |
| 3 | narrow 가 매 프레임 **모든 cloth 정점 × 모든 shape** 를 무조건 검사 (overlap 없어도) | **마커 구동**: 겹친 (cloth, sphere) 쌍마다 1 dispatch, shape **1개**만 정점마다 테스트 (`narrow_pt_analytic`). 낙하 구간엔 마커 0 ⇒ dispatch 0 |

> **선결 버그**: `BVH<…>::build()` 가 `objShape` 를 **항상 `ShapeType::Mesh`** 로
> 하드코딩해 broad 가 primitive 를 영영 인식 못 했다 (c-1 이전엔 broad 가 도형
> 종류를 알 필요가 없었음). `objShape = mesh->shapeType` 로 고쳐야 위 #1 스킵이
> 동작한다. 덤으로, c-1 에서 `BroadCollision.shapePair.target` 도 Mesh 로 찍혀
> `narrow_pt_tri` 의 `skipSphere`(shapePair==Sphere 검사)가 실제론 구를 못 걸러
> **구에 triangle + analytic 이중 응답**이 가능했던 latent 버그도 교정된다.

## 씬 구성 (c-1 과 동일)

| 객체 | 형태 | 분할수 | 동작 | 비고 |
|---|---|---|---|---|
| Cloth | 1×1 XZ-plane, 중심 (0, 0.7, 0) | a × a | `FastGridCloth` | `addClothGridFastAt` |
| Sphere | 직경 1 (반지름 0.5), 중심 원점 | UV b | `Rigid` (mass=0, static) | `applyGravity=false`, `applyWind=false` |

- **a (cloth 분할)** ∈ {20, 50, 100}, **b (sphere 분할)** ∈ {20, 50, 100} → 9 조합
- × **analytic {Off, On}** = **18 케이스**, 케이스당 **30 프레임**(+ warmup 1), substep 60
- 측정 스코프: `broad_detect`, `narrow_phase`, `physics_total` (c-1 과 동일)

## 실행 방법

```sh
cmake --build build -j
( cd build && ./src/ysim --bench-analytic )   # → analytic_bench.csv
python3 chart.py                              # → *.png + summary.md
```

CSV 컬럼은 c-1 과 동일. 시간은 substep 60 회분 누적 프레임 합계.

## 결과 — 충돌 프레임(narrow > 0) 평균, 단위 ms

| a | b | broad Off | broad On | narrow Off | narrow On | phys Off | phys On | bcol On | ncol Off | ncol On |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 20  | 20  | 38.4  | **0.0** | 26.9 | 10.5 | 178.3 | 63.8  | 0 | 9,892   | 6,291   |
| 20  | 50  | 49.7  | **0.0** | 36.0 | 23.0 | 219.5 | 128.0 | 0 | 32,072  | 6,291   |
| 20  | 100 | 76.2  | **0.0** | 53.1 | 40.6 | 339.5 | 217.5 | 0 | 108,479 | 6,291   |
| 50  | 20  | 57.2  | **0.0** | 34.9 | 30.6 | 224.7 | 163.3 | 0 | 94,255  | 82,274  |
| 50  | 50  | 83.1  | **0.0** | 47.7 | 36.5 | 289.5 | 186.9 | 0 | 126,957 | 82,274  |
| 50  | 100 | 42.0  | **0.0** | 23.3 | 40.7 | 141.0 | 212.1 | 0 | 248,790 | 82,274  |
| 100 | 20  | 79.4  | **0.0** | 61.8 | 48.2 | 344.4 | 231.3 | 0 | 455,885 | 401,259 |
| 100 | 50  | 113.7 | **0.0** | 77.6 | 52.5 | 421.9 | 252.4 | 0 | 493,951 | 401,259 |
| 100 | 100 | 53.0  | **0.0** | 36.2 | 22.2 | 184.5 | 113.7 | 0 | 622,491 | 401,259 |

(c-1 과 같이 **실제 충돌 프레임**만 평균. broad/phys 의 Off↔On 절대값엔 첫 충돌
이후 접촉 모델 차이로 인한 트레젝토리 노이즈가 섞임 — 정성 결론은 견고.)

## 관찰

### (1) analytic ON 이 broad(BVH)를 완전히 제거한다 — c-1 의 #1 한계 해소

- `broad_detect On = 0.0 ms`, `broad_collisions On = 0` — **전 케이스**.
  이 씬에서 충돌 가능한 broad 기여자는 구뿐인데, 그 구를 object-level 에서
  스킵하므로 triangle descent 도, 그것이 뱉던 vertex×triangle BroadCollision
  도 사라진다 (`headline_bars.png` 의 연하늘 broad-On 막대 = 빈 자리).
- c-1 에서 broad 는 physics_total 의 ~25–32%. c-2 On 은 그 덩어리를 통째로
  제거한다. (cloth self-collision 이 있는 씬이라면 self 쌍은 그대로 남으므로
  broad On > 0 이 된다 — 스킵은 sphere 한정.)

### (2) analytic narrow 가 sphere 분할수 b 와 무관 = 진짜 analytic 동작 증거

- `ncol On` 은 cloth 분할 a 에만 의존하고 **b 와 완전히 무관**: cloth20 →
  b20/b50/b100 모두 **6,291** 로 동일. 반면 `ncol Off`(삼각형 소프)는 b 가
  커질수록 폭증 (b20 9,892 → b100 108,479).
- 의미: analytic 은 정점마다 ellipsoid 최근접 **1 contact** 만 만든다 — 구의
  tessellation 과 무관. triangle-soup 은 겹친 sphere 삼각형마다 contact 를
  만들어 fine sphere 에서 과다 생성. 거친 구(b20)에선 `ncol On ≈ ncol Off`,
  fine 구(b100)에선 `On ≪ Off`. **c-2 의 analytic 경로가 올바르게 동작**하며
  narrow 작업량을 b 와 분리시킨다.

### (3) narrow 도 빨라지고 commitAndWait 가 절반

- `narrow On` 은 대부분 `narrow Off` 보다 빠르다 (`narrow_timeseries.png`).
  마커 구동(overlap 쌍만) + 단일-shape + substep 당 동기화 1회 감소가 합쳐진
  결과. c-1 On 의 narrow(평균 50ms 급)가 c-2 On 에서 20–50ms 로 내려간다.

### (4) c-1 → c-2 before/after (physics_total, analytic ON)

`c1_vs_c2.png`. 9 케이스 중 8 개에서 c-2 가 빠르다.

| | 평균 | 최선 (a100b100) | 최악 (a100b20) |
|---|---:|---:|---:|
| c-2 On / c-1 On | **0.77×** | **0.29× (3.4배)** | 1.02× (노이즈) |

a100b20 만 거의 동등 — Off/On 트레젝토리 발산으로 충돌 프레임 선택이 갈린
케이스(측정 한계). 절감폭이 큰 케이스(broad 가 두껍던 a≥50, fine 구)에서 이득이
가장 크다.

## 결론

1. **c-2 로 analytic 활성화는 BVH 대비 명백히 유의미해졌다.** c-1 결론의 정반대.
   - broad(BVH) 비용 **0% → 100% 제거** (object-level 스킵).
   - narrow 작업량을 sphere 분할수와 분리 + commitAndWait 절반.
   - physics_total 평균 0.77×, 큰 씬 최대 3.4배.
2. **정확성 유지**: cloth 가 구 위에 정상 드레이프(매 충돌 프레임 contact 발생),
   contact 수가 정점당 1 로 b 와 무관 — analytic 모델 그대로.
3. **남은 일 / 한계**
   - cube·cylinder 는 아직 triangle-soup (c-3). 스킵은 Sphere 한정.
   - SH(spatial-hash) broad 경로는 마커를 안 만들므로 SH + analytic 은
     triangle-soup fallback (기본 broad 는 BVH 라 일반 사용엔 영향 없음).
   - per-vertex AABB 사전 필터(정점 단위로도 구 AABB 밖 정점 skip)는 추가
     최적화 여지 — 현재는 overlap 한 cloth 의 **전 정점**을 테스트.
   - 측정은 c-1 과 같은 트레젝토리-발산 노이즈를 가짐.

## 파일

- `analytic_bench.csv` — c-2 원본 프로파일 (540 행 = 18 case × 30 frame)
- `analytic_bench_c1_baseline.csv` — c-1 데이터 (before/after 비교용)
- `chart.py` — 분석 스크립트 (numpy + pandas + matplotlib)
- `headline_bars.png` — 9 케이스 broad Off/On + narrow Off/On (broad On → 0)
- `time_breakdown.png` — 3×3, physics_total stacked (On 의 broad 칸 소멸)
- `narrow_timeseries.png` — 3×3, 프레임별 narrow_phase Off vs On
- `c1_vs_c2.png` — physics_total ON: c-1 → c-2 before/after
- `summary.md` — chart.py 출력 요약 표
