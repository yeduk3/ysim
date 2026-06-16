# Sub-object (multi-root) LBVH — cloth-on-sphere 종합 (Phase 1 + 2a + 2b, 2026-06-16)

square cloth 를 material-space 에서 k 개 타일 그룹으로 쪼개 그룹마다 독립 Karras
트리(= k roots)를 만드는 sub-object BVH 를, **구 위에 천이 떨어지는 일반 씬**에서
refit·query 양쪽으로 측정한 종합. [subobject-bvh-2026-06-16](../subobject-bvh-2026-06-16/)
(Phase 1 combine, flat·정적 cloth)의 후속:

1. **Phase 1 재측정** — flat cloth 가 아니라 구에 **draped 된** 상태 + 일반 BVH
   기준 경로인 **fused `refitSwept`** 로 combine 측정.
2. **Phase 2a** — multi-root query 정확화 (group-0-only 버그 3종 수정).
3. **Phase 2b** — mini-TLAS 로 query 비용을 k-무관(O(log N))화.

## 씬 / 방법

cloth(분할 P, 중심 y=0.7) 가 직경 1 구(원점, Rigid static, 윗면 y=+0.5) 위로
떨어진다. **물리는 항상 single-root 로 구동**(Phase 2 query 가 들어오기 전엔
multi-root 물리가 깨지므로) settleFrames=20 으로 draped 시킨 뒤, 그 상태에서
`objTrees[0]` 을 single / multi(s=1..16)로 재빌드해 측정.

- `--bench-bvh-subobject-scene` → `scene_refit_bench.csv`
  - `combine_us_amortized`: combine 커널만 (한 command buffer 에 reps 회 인코딩 후
    1회 commit) — `commitAndWait` sync floor 제거.
  - `refitswept_us_per_call`: fused `refitSwept` 1회(내부 commit 포함) — 실제
    substep 당 비용.
- `--bench-bvh-subobject-query` → `query_cost_bench.csv`
  - `detect_us_per_call`: `detectCollisions` e2e(queryBegin+CPU 객체루프+GPU
    queryPoints+queryEnd commit) 평균 — 1회 broad detect 의 현실값. self OFF
    (일반 씬 기본값); multi-root cloth 트리는 sphere→cloth 방향 query 로 순회됨.
  - **correctness gate**: `broad_collisions` 가 single 과 **전 s 에서 일치**해야 함.

## Phase 1 — refit combine (draped 씬, fused refitSwept)

`combine_vs_s.png`. combine µs/dispatch, best (full split):

| P | prims | single | best multi | combine 배속 | refit/frame(×60) single→multi |
|---|---|---|---|---|---|
| 32  | ~2k  | 20.7µs | 6.5µs  | **3.2×** | 12.4 → ~8.5 ms (~30%) |
| 100 | ~20k | 36.9µs | 11.7µs | **3.15×** | 14.6 → 12.9 ms (~11%) |
| 200 | ~79k | 193.6µs | 69µs  | **2.8×** | 27.3 → 24.9 ms (~8%) |

combine 자체는 일관되게 **2.8–3.2× 빠름** (divergence-cut = 그룹 트리 깊이 붕괴).
단 fused `refitSwept` e2e 에는 per-call `commitAndWait` 왕복 지연이 끼어 이득이
희석 — @79k combine 절감 120µs 가 e2e 엔 ~39µs 만 표면화. **프레임당 refit 은
전 크기에서 빨라지나(7–30%) sync floor 가 천장.**

## Phase 2a — query 정확화 (버그 3종)

multi-root 인데 query 가 **slot 0 = group 0 만** 순회 → 나머지 그룹 tunneling.
수정점 (전부 group-0-only):

1. `queryAABB` 커널 (bvh.metal) — entry 를 group 0 이 아니라 k root 전부.
2. `queryAABBSegmented` 커널 — 동일.
3. **object-cull** (`detectCollisions`/`detectCollisionsSegmented`) — `tree[0].aabb`
   (group 0 박스) → `objectRootAABB()` (k root union). 안 고치면 객체쌍 누락.

검증(`--bench-bvh-subobject-validate`): cloth-on-sphere single vs multi(s) →
clothY(min/mean/max)·broad·narrow **bit-identical**, tunneling 없음. self-test
`YSIM_SUBOBJECT=2` 99 PASS/2 FAIL = plain 과 동일(기존 D-042 R-3/R-4).

## Phase 2b — mini-TLAS: query 비용 k-무관

**문제(2a):** entry 가 점마다 k root 전부 AABB-cull = **O(k)/점**. combine 은
full split(k≈N/2)을 원하는데 query 가 거기서 폭발 → **충돌**.

**해법:** k group root 를 **하나의 super-root 아래로 stitch**. group 서브트리는
slot [0, 2N−k) 를 쓰고, **tail [2N−k, 2N−1)(정확히 k−1 free)** 에 k root 위의
top binary tree 를 깐다. query 는 다시 super-root 단일 entry(O(log N)). top tree
topology 는 고정 타일 그리드라 **1회 build**, refit 마다 k−1 개 CPU min/max
union(`topCombineCPU`)만. GPU multi-root combine 은 group root 에서 멈추고 tail 을
안 건드려 그대로(bit-identical).

`query_cost_2a_vs_2b.png` (P=100, single detect 812µs):

| k (num_groups) | Phase 2a detect | Phase 2b detect |
|---|---|---|
| 1 (single) | 1576µs | 812µs |
| 4 | 1446 | 848 |
| 625 | 3085 | 802 |
| 2500 | 4542 | 797 |
| **9801 (full split)** | **7700 (4.9×)** | **790 (≈single)** |

**O(k) tax 소멸** — detect 가 전 k 에서 flat. P=50 도 동일(full split 2a 2.3× →
2b ≈single). broad count 는 전 s 에서 single 과 일치(정확성 유지).

## 핵심 결론

- **충돌 해소**: max split 이 combine **3.2×** 이면서 query 가 **≈single**. sub-object
  의 두 이득(divergence-cut refit + 완전·저렴 query)이 **동시 성립**.
- **남은 천장 = per-subframe `commitAndWait` sync floor** (refit). sub-object 와
  무관한 별도 레버 — 프레임당 refit 이득이 약한 진짜 이유.
- **whole-frame wall-clock 은 측정 불가**: 같은 씬 3회에 single 81–142ms /
  multi 95–225ms (**±50%**). refit/query 델타 ≪ 노이즈 ⇒ **PerFrame/whole-frame
  프로파일로는 sub-object 이득 안 보임. InFrame 섹션 / 마이크로벤치만 유효.**
- **P=200 query 점은 degenerate** (broad=0, 20프레임 안에 천이 구에 못 닿음 —
  scene-scale 이슈, query 무관). P=50/100 이 깨끗한 신호.

## Sync floor — 남은 천장 / 다음 레버 (기록)

sub-object 의 두 이득(combine 3.2×, query k-무관)이 다 확보됐는데도 **프레임당
refit 이득이 7–30% 에 그치고 whole-frame 은 노이즈에 묻히는** 근본 원인 = 매
substep 의 `commitAndWait` **sync floor**.

**현상.** broad refit/query 가 substep 마다 GPU 를 flush + CPU 대기:
- `refit()` / `refitSwept()` / `enlargeTrajectory()` 각각 끝에 `commitAndWait`
  (main.cpp 의 grouped/single 분기 모두).
- narrow phase 도 substep 마다 commit.
- 60 substep × (refit sync + narrow sync + …) = 프레임당 수십 회 동기.
- ProfileLevel 과 **무관** (None/PerFrame/InFrame 다 동일). 9451 의 프레임당
  commit 은 렌더 readback 일 뿐, substep sync 는 그대로.

**왜 천장인가.** refitSwept 1콜의 wall-time = buildSweptLeaf + combine +
`commitAndWait` 왕복지연. 측정상 P=100 에서 combine 절감 25µs / P=200 에서 120µs
가 e2e 엔 일부만 표면화 — per-call CPU↔GPU 왕복(~100–200µs 고정)이 combine 이득을
가린다. combine 을 3.2× 줄여도 프레임 refit 은 그만큼 안 줄어든다.

**다음 레버 (미구현).** substep 내 GPU 워크를 **배칭**해 sync 횟수를 줄인다:
- refit·narrow 의 per-substep `commitAndWait` 제거 → substep 경계(또는 cdP 경계)
  에서 1회만 sync. CPU 가 결과를 즉시 안 읽어도 되는 dispatch 는 큐에 쌓고 지연
  커밋. (단 narrow 응답이 다음 substep 적분에 필요하므로 의존성 분석 필요 —
  positions 읽기 시점이 sync 지점.)
- 기대: combine 3.2× 이득이 프레임 예산에 **실제로** 실린다.
- 사용자가 처음 떠올린 "sync 는 렌더 전에만" 직관이 맞는 방향 — 단 profile 모드
  전환이 아니라 **refit/narrow 의 sync 지점을 코드에서 옮기는** 변경. sub-object
  와 직교(별도 실험). 관련: [bvh-refit-2026-05-12](../bvh-refit-2026-05-12/) 의
  sync floor 진단.

## 산출물

- `scene_refit_bench.csv` — Phase 1 combine + fused refitSwept (draped 씬).
- `query_cost_bench.csv` / `query_cost_bench_phase2b.csv` — Phase 2b detect (k-무관).
- `combine_vs_s.png` — Phase 1 combine µs vs s.
- `query_cost_2a_vs_2b.png` — **Phase 2a O(k) vs 2b flat** detect (the money chart).

> Phase 2a 의 detect CSV 는 그 코드가 2b 로 대체되어 보존 안 됨. 차트의 2a 곡선은
> 당시 벤치 출력(P=100)을 `chart.py:PHASE2A_P100` 리터럴로 박은 값.

## 재현

```bash
./build/ysim --bench-bvh-subobject-scene      # scene_refit_bench.csv (Phase 1)
./build/ysim --bench-bvh-subobject-query      # query_cost_bench.csv  (Phase 2b)
./build/ysim --bench-bvh-subobject-validate 50 4 40   # 정확성(bit-identical) 확인
python3 profiles/experiment/subobject-scene-2026-06-16/chart.py
```
런타임 토글(인터랙티브): **B** = on/off, **N** = s 순환.
