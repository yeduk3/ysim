# BVH 빌드 모드 3-way 비교 (2026-05-26)

`source.csv` 는 ysim 한 세션에서 90 프레임(연속)을 캡처한 것으로,
프로파일러 출력에서 케이스를 바꿔가며 30 프레임씩 측정했다.

| 케이스 | Refit 토글 | 빌더 | 의미 |
|---|---|---|---|
| 1 | ON  | Karras 2012 | 기본 동작 — 10 프레임마다 rebuild, 그 외 frame 은 substep 마다 refit |
| 2 | OFF | Karras 2012 | 매 refit 호출이 SCENE::refit() 내부 fallback 으로 build() 재실행 |
| 3 | OFF | Apetrei 2014 | 매 refit 호출이 fallback 으로 build(), 빌드는 agglomerative 단일 커널 |

CSV의 `frame_sequence` 는 케이스별로 0..29 로 다시 시작한다.

## 실행 방법

```sh
python3 chart.py
# → build_refit.png, query.png 생성
```

## 차트 해석

### 1. `build_refit.png` — 프레임당 `bvh_build + broad_refit`

스코프 이름 매핑:
- `bvh_build` : Simulator::update() 의 10프레임 주기 build() 콜 (line 7540)
- `broad_refit` : 매 substep 의 broadPhase.refit() 콜 (line 7679).
  케이스 2,3 에서는 enableRefit=false 라서 이 콜이 내부적으로 build() 로 fallback 되지만
  프로파일러 스코프 이름이 그대로라 같은 컬럼에 누적된다.

관찰:
- **케이스 1** ~58 ms 정상치, 첫 프레임만 ~100 ms (초기 build + warmup).
- **케이스 2** ~45 ms — 의외로 매 substep rebuild 가 케이스 1 의 refit 보다 빠르다.
  refit 의 `bottomUpHybrid` 는 GPU 부분 + CPU 마무리 구조라 `commitAndWait`
  비용이 있고, build 의 buildTreeGPU + bottomUpHybrid 도 같은 commit 을 갖지만
  토폴로지 재생성이 단순한 sort + binary search 라 substep refit 보다 가볍게 떨어지는
  지점이 있는 것으로 보임.
- **케이스 3** ~1 ms 미만 — **GPU 비동기 디스패치의 측정 아티팩트**.
  agglomerativeBuild 커널은 단일 커널이고 dispatch 후 `commitAndWait` 가 없다.
  스코프는 호스트에서 dispatch 호출이 리턴하는 시점에 닫히므로 실제 GPU 워크는
  다음 sync 지점(여기선 `broad_detect`)으로 비용이 이동한다.

### 2. `query.png` — 프레임당 `broad_detect`

관찰:
- 케이스 1, 2 는 ~5 프레임 동안 거의 0 ms 인데, 이는 초기 정착 단계에서 broad
  collision pair 가 아직 생기지 않은 시간대로 보인다. 이후 ~22 ms 로 안착,
  큰 충돌 클러스터 형성 시 ~50 ms.
- 케이스 3 는 프레임 0 부터 ~88 ms 로 시작한다. 케이스 1,2 의 query 와 비교하면
  +~50 ms 이상 — 이것이 케이스 3 의 build_refit 차트에서 "사라진" 비용이다.
  즉 build_refit + query 의 **총합**으로 보면 세 케이스가 비슷한 영역에 모인다.

## 결론

차트 1만 보면 Apetrei 가 압도적으로 빠른 것처럼 보이지만, 차트 2 와 함께 보면
GPU 디스패치 비동기성에 의한 비용 이동임을 알 수 있다. 정확한 빌드 비용을 측정
하려면:

1. agglomerative 빌드 후 명시적으로 `commitAndWait` 를 추가해 같은 sync 기준을
   강제하거나,
2. Metal performance counter / GPU timeline 으로 GPU 측 실시간을 측정하거나,
3. build_refit + query 합으로 비교 (case 3 의 query 가 다음 sync 까지 모두
   흡수한다고 가정).

본 비교의 진짜 신호는 **case 1 vs case 2** — 매 substep rebuild 가 substep refit
보다 약 13 ms 가볍다는 것. 이는 현재 refit 구현(`bottomUpHybrid` CPU 마무리
포함)이 substep-수렴 시나리오에서는 build 보다 비효율적일 수 있다는 단서다.

## 파일

- `source.csv` — 원본 프로파일 (90 행)
- `chart.py`   — 분석 스크립트
- `build_refit.png` — 차트 1
- `query.png` — 차트 2
