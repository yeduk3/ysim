# Segmented vs Atomic broad-phase query 비교 (2026-05-26)

`useSegmentedBVHQuery` 토글 두 모드를 동일 씬·동일 프레임 수로 측정해
**충돌이 적은 케이스부터 많은 케이스까지** broad-phase 쿼리 시간이 어떻게
달라지는지 본다.

## 씬 구성

| 객체 | 형태 | 분할수 | 동작 | 비고 |
|---|---|---|---|---|
| Cloth | 1x1 XZ-plane, 중심 (0, 0.7, 0) | n × n | `TriangularCloth` | `addCloth` 기본 강성 |
| Sphere | 직경 1 (반지름 0.5), 중심 원점 | UV n | `Rigid` (mass=0, static) | `applyGravity=false`, `applyWind=false` |

`addSphere` 의 `size` 인자는 직경 — `primitive::sphere` 안에서 `r = size * 0.5`. cloth 가
y=0.7 에서 떨어지면 구의 정점 y=+0.5 위에 안착한다. 구를 강체 + 중력/바람 OFF
로 둔 이유는 Bullet 의 바닥 반발 처리 때문에 동적 구가 떠오르는 현상을 피하기
위해 (mass=0 → Bullet static body, 위치 고정).

n ∈ {20, 50, 100} 세 단계 × {Atomic, Segmented} 두 모드 = **총 6 케이스, 케이스당 30
프레임, substep 60**. (n=200 케이스는 시간상 제외.)

## 실행 방법

```sh
# (worktree root) cmake --build build -j
build/src/ysim --bench-bvh-segquery       # → source.csv
python3 chart.py                          # → detect_time.png, collisions.png
```

CSV 컬럼: `query_mode, particle_count, frame_index, broad_detect_ms,
broad_collisions, narrow_collisions`. `particle_count` 는 cloth 의 정점 수
(n²) — sphere 정점은 별도. `broad_detect_ms` 는 `Simulator::update` 안의
`broad_detect` 프로파일러 스코프 — substep 60 회분이 누적된 프레임 합계.

## 결과 요약

### 전체 30 프레임 평균

| Mode | n | broad_detect avg (ms) | broad_detect max (ms) | narrow avg | narrow max |
|---|---:|---:|---:|---:|---:|
| Atomic    | 20  | **14.10** | 47.98  | 5,859    | 15,969   |
| Atomic    | 50  | 65.70     | 203.87 | 66,382   | 130,082  |
| Atomic    | 100 | 76.92     | 145.18 | 326,515  | 661,199  |
| Segmented | 20  | 44.36     | 77.58  | 3,601    | 11,040   |
| Segmented | 50  | **42.63** | 81.72  | 43,932   | 88,535   |
| Segmented | 100 | **47.96** | 86.80  | 222,894  | 468,161  |

### "충돌이 실제로 일어난 프레임" 만 평균 (narrow > 0, 보통 frame 11..29)

| Mode | n | broad_detect avg (ms) | narrow avg |
|---|---:|---:|---:|
| Atomic    | 20  | **22.25** | 9,251    |
| Atomic    | 50  | 103.73    | 104,814  |
| Atomic    | 100 | 121.40    | 515,550  |
| Segmented | 20  | 69.98     | 5,686    |
| Segmented | 50  | **67.26** | 69,366   |
| Segmented | 100 | **75.68** | 351,938  |

## 관찰

### (1) 두 모드의 교차점이 n=50 부근

- **n=20** (충돌쌍 평균 ~9k): Atomic 이 **3 배 빠르다** (22 ms vs 70 ms).
  Segmented 는 충돌이 적어도 항상 segment scan + reserve + compact 의 fixed
  overhead 를 치러야 한다. Atomic 은 충돌이 적으면 atomic-add 의 contention
  자체가 거의 없어 비용도 거의 zero.
- **n=50** (~105k 충돌쌍): Segmented 가 **1.5 배 빠르다** (67 ms vs 104 ms).
- **n=100** (~515k 충돌쌍): Segmented 가 **1.6 배 빠르다** (76 ms vs 121 ms).

### (2) Segmented 는 충돌 수가 늘어도 시간이 거의 평평하다

n 이 20 → 50 → 100 으로 늘면서 narrow collisions 가 9k → 105k → 515k 로
약 57 배 증가하지만 Segmented 의 detect time 은 70 → 67 → 76 ms 로 거의
변하지 않는다. fixed overhead 가 dominant 인 영역.

반면 Atomic 은 22 → 104 → 121 ms 로 늘어난다 — 충돌쌍 수에 따라 atomic
contention 이 누적되는 패턴. 다만 9k → 105k 구간 (5× 증가) 에서 22 → 104 ms
(약 4.7×) 인 데 비해 105k → 515k 구간 (5× 증가) 에서는 104 → 121 ms 로
saturate 하는 모양 — atomic 자체보다 동반되는 BVH traversal cost 가 커진
탓으로 보임.

### (3) narrow_collisions 카운트가 두 모드에서 다르다

| n | Atomic narrow avg | Segmented narrow avg | 비율 |
|---:|---:|---:|---:|
| 20  | 9,251   | 5,686   | 0.61 |
| 50  | 104,814 | 69,366  | 0.66 |
| 100 | 515,550 | 351,938 | 0.68 |

Segmented 가 **약 60-70 % 만** 검출. 동일한 broad-phase 입력(같은 BVH, 같은
margin)에 대해 collision pair set 자체는 같아야 하므로 segmented path 의
**버퍼 capacity** 또는 **per-TG slice cap** 이 묵묵히 잘라내고 있는 것으로
보인다 (`maxNumCollisions` / per-TG `perTGCap`). detect 시간 비교에는 영향이
적지만, 정확도가 다르다는 점은 본 측정에서 명시적으로 짚어둘 필요가 있다.

→ 다음 슬라이스 후보: Segmented path 의 capacity / spill 카운터 export 해
"드롭된 충돌쌍이 몇 개인지" 를 CSV 에 추가하고, 동일 충돌 수가 되도록
용량을 맞춘 뒤 detect 시간만 깔끔히 비교.

## 결론

1. **충돌이 적으면 (n≈20) Atomic 이 결정적 우세** — segment 자료구조
   준비 비용을 보상할 만큼의 contention 이 발생하지 않음.
2. **충돌이 많아지면 (n≥50) Segmented 가 우세** — 절대값도 빠르고 충돌 수
   scaling 에 둔감.
3. **교차점이 본 워크로드(1×1 cloth + sphere)에서 n=20 과 n=50 사이**.
   양쪽 path 모두 켜둘 가치가 있고, 작은 씬은 Atomic, 큰 씬은 Segmented
   라는 휴리스틱 (또는 runtime auto-switch) 가 자연스러운 결론.
4. narrow_collisions 가 두 모드에서 일치하지 않는 결함 발견 — 다음 슬라이스
   에서 segmented buffer capacity 조사 후 재측정 권장.

## 파일

- `source.csv` (== `segquery_bench.csv`) — 원본 프로파일 (180 행 = 6 case × 30 frame)
- `chart.py` — 분석 스크립트 (matplotlib)
- `detect_time.png` — broad_detect 시간 per frame (6 시리즈)
- `collisions.png` — narrow_collisions per frame (6 시리즈)
