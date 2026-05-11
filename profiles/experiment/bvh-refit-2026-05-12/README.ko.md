# BVH refit 벤치마크 — 2026-05-12

## 실험 개요

ysim의 BVH `refit()` 시간을 네 가지 방법, 네 가지 cloth particle 수에
걸쳐 측정한다. 이 데이터는 D-030이 도입한 runtime 노브
`bottomUpHybridDepth`의 기본값을 측정 기반으로 정하기 위한 입력이다.

- 상위 결정: D-030 (hybrid GPU+CPU bottom-up combine)
- 측정 결정: D-031 (벤치 하네스 설계 + 출력 스키마)
- 이전 슬라이스의 WARNING: turn 25 — "Hybrid default depth is unmeasured."
  이 실험이 그 WARNING을 닫는다.

## 방법론

### 네 가지 refit 방법

| 라벨        | `bottomUpHybridDepth` | 동작                                                                 |
|------------|----------------------|---------------------------------------------------------------------|
| `FullCPU`  | 0                    | GPU dispatch 생략 → `bottomUpCombine()` 재귀 CPU 결합                     |
| `HybridD1` | 1                    | GPU가 leaf로부터 1단계 결합 → CPU가 root까지 마무리                          |
| `HybridD2` | 2                    | GPU가 leaf로부터 2단계 결합 → CPU가 root까지 마무리                          |
| `FullGPU`  | 30                   | GPU kernel이 root까지 진행 (`30 >= log2(largest tree depth)`); CPU skip-walk가 root에서 즉시 단락 |

#### "FullGPU"에 대한 honest note

`FullGPU`는 사실상 D-029의 `bottomUpBoxesGPU`와 동일한 경로지만 코드
경로는 `bottomUpBoxesPartialGPU(30)`을 거친다 (D-030 parallel-symbol 규약).
두 경로의 차이는 kernel loop iteration당 register read 1개 + compare 1개의
overhead이며, 같은 iteration에 들어가는 atomic + 2 seq_cst fence에 비하면
sub-noise이다. 측정 결과에서 의미 있는 차이는 보이지 않을 것이라 예상한다.
만약 후속 슬라이스에서 strict-D-029 column이 필요하다는 측정 질문이 나오면
다섯 번째 column을 추가할 수 있다 (PROJECT_STATE의 "Strict-D-029-column
bench slice" 후보).

### 네 가지 cloth 해상도 (FastGridCloth, `N x N` grid)

| 라벨   | `particleNum1D` | Vertex 수 | Triangle 수 (`2*(N-1)^2`) |
|-------|-----------------|----------|---------------------------|
| 1k    | 32              | 1,024    | 1,922                     |
| 10k   | 100             | 10,000   | 19,602                    |
| 100k  | 316             | 99,856   | 198,450                   |
| 500k  | 707             | 499,849  | 996,072                   |

차트의 x축은 사용자 친화적으로 "particle 수" (= vertex 수)를 쓰지만,
실제 BVH tree size를 결정하는 것은 triangle 수 (= leaf 수)이다.
Refit 시간 scaling은 triangle 수에 비례한다.

### 측정 절차

- 각 `(method, particle_count)` 조합마다:
  1. Scene reset → cloth 한 장만 (`addClothGridFast`) → `sim.initialize()`.
  2. `objTree.bottomUpHybridDepth = depthForMethod(method)` 설정.
  3. `sim.profiler`에 local `FrameProfiler` 연결.
  4. 1 frame warmup (cold-start 흡수, 첫 build cost 제외) — 측정에서 제외.
  5. 10 frame 측정. 각 frame은 60 substep을 포함하며, `broad_refit`
     scope가 60번의 substep refit time을 합산하여 1 row로 기록.
  6. CSV row: `method, particle_count, frame_index (0..9), refit_time_ms`.

- 총 4 method x 4 size x 10 frame = 160 row.
- Sim parameter는 `addClothGridFast`의 default 값을 모든 해상도에서 동일하게
  사용 (kstretch=1e5, kshear=1e5, kbend=3e5, thickness=0.001, mass=0.1).
- Substep 수는 60으로 고정.
- 충돌은 비활성화 (cloth-only scene; `enableSelfCollisions = false` default).

## 실행 방법

빌드 후 `build/` directory에서:

```sh
./src/ysim --bench-bvh-refit
```

소요 시간은 hardware-dependent이지만 1k/10k는 수 초, 100k는 분 단위,
500k는 분~십수 분 단위로 예상한다 (60 substep x 11 frame). CSV는
`profiles/experiment/bvh-refit-2026-05-12/refit_bench.csv`에 덮어쓰기로
기록된다.

차트 생성:

```sh
cd profiles/experiment/bvh-refit-2026-05-12/
python3 chart.py
```

`refit_chart_line.png` (log-log line chart) + `refit_chart_bar.png`
(grouped bar chart)가 같은 directory에 떨어진다.

## 캐비어트

- **첫 frame warmup discard.** Metal command queue cold-start + 첫 buffer
  allocation effect 흡수. `BroadPhase::build`는 frame 0과 frame 10마다
  실행되지만 `bvh_build` scope는 별도이므로 `broad_refit` 측정에는 섞이지
  않는다. 다만 frame 0의 `broad_refit`은 다른 frame과 동일한 60 substep
  비용을 측정함에 유의.
- **Cloth instability acceptable.** 500k cloth는 default sim parameter로
  발산할 수 있으나 refit 시간은 particle 위치의 수치적 안정성과 무관하다.
  Tree topology는 build time에 fixed이고 leaf AABB만 매 substep 다시
  쓰이므로 발산하더라도 측정 값은 representative하다.
- **Hardware dependence.** Apple Silicon Metal 3.2 host에서 측정된 값.
  Discrete GPU나 다른 OS에서는 다른 결과 예상.
- **`bottomUpHybridDepth` mutation은 per-mesh BVH에만 적용.** Scene-level
  BVH (`broadPhase.tree`)도 동일 값으로 설정하지만 cloth-only scene에서는
  leaf 1개라 무관.
- **Cumulative narrow / broad collision 카운터는 cloth-only scene에서
  0이다** (`enableSelfCollisions = false`).

## 산출물

- `refit_bench.csv` — header row만 있는 placeholder 상태로 commit됨.
  `--bench-bvh-refit` 실행 후 160 row로 populate됨.
- `chart.py` — 차트 생성 스크립트 (matplotlib + csv stdlib only;
  pandas dependency 없음).
- `refit_chart_line.png` — log-log line chart, x = vertex count,
  y = mean refit_ms across 10 measured frames per method, error bars =
  stddev. `chart.py` 실행 결과로 생성.
- `refit_chart_bar.png` — grouped bar chart, 각 group이 particle_count
  값, 각 group 내 4 bar = 4 method. `chart.py` 실행 결과로 생성.
