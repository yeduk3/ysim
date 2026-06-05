# Analytic-primitive on/off vs BVH 비교 (2026-06-04)

`useAnalyticPrimitive` 토글(키 `A` / Profiler 체크박스)을 켜고 끄는 것이
충돌 탐지 시간에 **BVH(broad phase) 대비 얼마나 유의미한 영향**을 주는지
측정한다. 결론부터: **유의미한 이득은 없다.** broad(BVH)가 지배적인데
analytic 토글은 broad 를 전혀 건드리지 못하고, narrow 만 바꾸되 현재
구현에서는 오히려 **평균 1.38× 느려진다**.

## 씬 구성

| 객체 | 형태 | 분할수 | 동작 | 비고 |
|---|---|---|---|---|
| Cloth | 1×1 XZ-plane, 중심 (0, 0.7, 0) | a × a | `FastGridCloth` | `addClothGridFastAt` |
| Sphere | 직경 1 (반지름 0.5), 중심 원점 | UV b | `Rigid` (mass=0, static) | `applyGravity=false`, `applyWind=false` |

cloth 가 y=0.7 에서 중력으로 떨어져 구 정점 y=+0.5 위에 안착·드레이프한다.
구는 강체 + 중력/바람 OFF (Bullet static, 위치 고정).

- **a (cloth 분할)** ∈ {20, 50, 100}, **b (sphere 분할)** ∈ {20, 50, 100} → 9 조합
- × **analytic {Off, On}** = **18 케이스**, 케이스당 **30 프레임**(+ warmup 1), substep 60
- 측정 스코프: `broad_detect`(BVH 광역), `narrow_phase`(narrow_pt_tri +
  narrow_pt_analytic + CPU 정렬), `physics_total`(`sim.update()` 전체)

> **선결 버그 수정**: 이 측정 직전, `refreshAnalyticShapes` 가 `params.size`
> (=직경)를 반지름으로 그대로 써서 analytic 구가 메시 구보다 **2× 컸던**
> 버그를 고쳤다 (`primitive::sphere` 는 `r = size*0.5`). 수정 후 analytic-on/off
> 충돌이 기하학적으로 일치해 비교가 유의미해졌다. 또한 sphere 분할수 b 가
> 비균일 스케일까지 다뤄지도록 narrow 경로를 구→**타원체**로 일반화했다.

## 실행 방법

```sh
cmake --build build -j
( cd build && ./src/ysim --bench-analytic )   # → analytic_bench.csv (절대경로 출력)
python3 chart.py                              # → *.png + summary.md
```

CSV 컬럼: `analytic, cloth_n, sphere_n, frame_index, broad_detect_ms,
narrow_phase_ms, physics_total_ms, broad_collisions, narrow_collisions`.
시간은 모두 substep 60 회분 누적 프레임 합계.

## 결과 — 충돌 프레임(narrow > 0) 평균, 단위 ms

| a (cloth) | b (sphere) | broad Off | broad On | narrow Off | narrow On | narrow 비율 | phys Off | phys On |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 20  | 20  | 30.14  | 30.12  | 14.87 | 28.74 | **1.93×** | 120.63 | 137.56 |
| 20  | 50  | 40.97  | 43.56  | 19.05 | 38.14 | **2.00×** | 143.18 | 178.97 |
| 20  | 100 | 61.84  | 65.14  | 33.60 | 50.94 | 1.52× | 212.65 | 236.79 |
| 50  | 20  | 53.90  | 47.99  | 28.16 | 36.50 | 1.30× | 185.72 | 177.04 |
| 50  | 50  | 79.39  | 70.43  | 34.98 | 42.28 | 1.21× | 234.51 | 211.09 |
| 50  | 100 | 126.09 | 110.52 | 50.01 | 49.32 | **0.99×** | 323.35 | 273.49 |
| 100 | 20  | 72.07  | 63.71  | 43.54 | 48.51 | 1.11× | 250.98 | 225.77 |
| 100 | 50  | 104.08 | 93.02  | 53.26 | 61.23 | 1.15× | 306.44 | 282.35 |
| 100 | 100 | 150.21 | 154.34 | 65.74 | 79.17 | 1.20× | 369.89 | 393.06 |

(전체 30 프레임 평균이 아니라 **실제 충돌이 일어난 프레임**만 평균. 낙하
구간(보통 frame 0~10)은 narrow≈0 이라 제외.)

## 관찰

### (1) BVH(broad)가 충돌 비용을 지배하고, analytic 토글은 그걸 못 건드린다

- `broad_detect` 는 `physics_total` 의 평균 **~32%**, `narrow_phase`(Off)는 ~15%.
  `time_breakdown.png` 에서 파란(broad) 칸이 주황(narrow)보다 항상 크다.
- analytic 토글은 **broad 를 전혀 바꾸지 않는다.** 구가 여전히 BVH 에 들어
  있어 broad phase 는 양 모드 모두 cloth-vs-구-삼각형 쌍을 동일하게
  생성한다. 표의 broad Off↔On 차이(예: a50b100 126↔111)는 **충돌 응답이
  달라 cloth 드레이프가 갈라지는 트레젝토리 차이**일 뿐, 토글의 직접 효과가
  아니다.

### (2) analytic ON 은 narrow 를 오히려 느리게 한다 (평균 1.38×)

`narrow_timeseries.png` 에서 On(빨강)이 대부분 Off(파랑) 위에 있다. 원인:

- `narrowAndSortByVertices` 가 analytic ON 일 때 **substep 마다
  `commitAndWait` 를 2회**(triangle 경로 + analytic 경로) 호출한다. OFF 는
  1회. 60 substep × 추가 GPU 동기화(round-trip)가 고정 오버헤드로 붙는다.
- 그런데 구는 BVH 에 남아 있으므로 broad 는 여전히 구 쌍을 만들고,
  `narrow_pt_tri` 는 그 쌍들을 (skip 하더라도) 전부 순회한다. 즉 analytic
  은 broad 절감 0, narrow 는 "삼각형 테스트 생략 − 추가 dispatch/sync".
- analytic 커널 자체는 per-vertex × shape(=1) 라 연산은 무시할 수준 →
  느려짐의 정체는 **여분의 GPU 동기화**다.

### (3) 단, scale 이 커질수록 penalty 가 줄어든다

narrow 비율이 작은 씬 1.93×~2.00× → 큰 씬 0.99×~1.20× 로 수렴한다.
삼각형 쌍이 많아질수록 "구 삼각형 skip" 절감이 커져 고정 sync 오버헤드를
상쇄하기 때문. a50b100 에서 0.99× (break-even). 더 큰 씬이면 narrow 단독
으로는 analytic 이 이길 여지가 있으나, **broad(BVH)가 손대지지 않는 한
전체 이득의 상한은 narrow 비중(~15%)에 갇힌다.**

## 결론

1. **이 워크로드에서 analytic 활성화는 BVH 대비 유의미하지 않다.**
   충돌 시간의 지배 요인은 broad(BVH, ~32%)인데 analytic 은 이를 0% 절감.
2. **현재 구현의 analytic ON 은 net 손해** — narrow 평균 1.38× 느림(작은
   씬 최대 2×). 주범은 substep 당 추가 `commitAndWait`.
3. analytic 을 실익으로 만들려면 두 가지가 필요:
   - **구를 BVH 에서 제외**해 broad phase 가 구 쌍을 만들지 않게 한다
     (그래야 비로소 broad 절감이 생긴다).
   - **analytic dispatch 를 같은 command buffer 로 융합**해 두 번째
     `commitAndWait` 를 없앤다 (혹은 구만 있는 substep 은 triangle dispatch
     자체를 건너뛴다).
4. **측정 한계**: Off/On 은 첫 충돌 이후 접촉 모델이 달라(삼각형 다중 접촉
   vs per-vertex 단일 접촉) 트레젝토리가 갈라진다 → 케이스별 broad/phys
   비교에는 트레젝토리 노이즈가 섞인다. (1)(2) 의 정성적 결론은 견고하다.

## 파일

- `analytic_bench.csv` — 원본 프로파일 (540 행 = 18 case × 30 frame)
- `chart.py` — 분석 스크립트 (numpy + pandas + matplotlib)
- `headline_bars.png` — 9 케이스 broad(BVH) vs narrow Off/On (핵심 그래프)
- `time_breakdown.png` — 3×3 그리드, physics_total stacked (broad+narrow+그 외)
- `narrow_timeseries.png` — 3×3 그리드, 프레임별 narrow_phase Off vs On
- `summary.md` — chart.py 가 출력한 요약 표
