# BVH 빌드 모드 3-way 비교 — 수정판 (2026-05-26 15:41)

[bvh-build-modes-2026-05-26](../bvh-build-modes-2026-05-26/) 의 재측정.
직전 측정에는 두 가지 측정 결함이 있었음:

1. **GPU 비동기 측정 아티팩트** — agglomerative 빌드 후 `commitAndWait`
   가 없어 GPU 비용이 다음 sync(broad_detect)로 흡수돼 build_refit 시간이
   실제보다 작게, query 시간이 실제보다 크게 잡혔다.
2. **agglomerativeBuild 커널의 메모리 오더링 race** — 매 프레임 rebuild
   시 10~20 프레임 후 GPU 무한 루프가 발생 (이 폴더의 케이스 3 는 부분적
   결과 또는 stall 직전 데이터).

두 결함 모두 수정 후 동일 시나리오로 재측정한 결과가 본 폴더의 source.csv.

| 케이스 | Refit 토글 | 빌더 | 의미 |
|---|---|---|---|
| 1 | ON  | Karras 2012 | 10 프레임마다 rebuild, 그 외 substep refit |
| 2 | OFF | Karras 2012 | 매 refit 호출이 build() 로 fallback |
| 3 | OFF | Apetrei 2014 | 매 refit 호출이 build() 로 fallback, agglomerative 빌더 |

각 케이스 30 프레임. `frame_sequence` 는 케이스별로 0..29.

## 실행 방법

```sh
python3 chart.py
# → build_refit.png, query.png 생성
```

## 결과 요약 (프레임 1..29 평균; 프레임 0 은 warmup 스파이크라 제외)

| 케이스 | build+refit | broad_detect | enlarge_trajectory |
|---|---:|---:|---:|
| 1. Refit ON          | **58.20 ms** | 24.85 ms | 58.56 ms |
| 2. Karras rebuild    | **45.93 ms** | 25.77 ms | 53.53 ms |
| 3. Apetrei rebuild   | **46.44 ms** | 25.42 ms | 53.15 ms |

## 관찰

### (1) Apetrei ≈ Karras (build+refit 약 46 ms)

직전 측정에서 Apetrei 가 압도적으로 빨라 보였던 것은 GPU 비동기 측정
아티팩트였음. 같은 sync 기준에서 측정하니 두 빌더의 총 비용 차이는 ~0.5 ms
(노이즈 수준). 이 워크로드(메시 수 ~3-4 개, ~24K 정점)에서는 두 방식의
주된 비용이 Morton 코드 생성 + radix sort + dispatch 오버헤드에 있고,
실제 hierarchy 생성 단계의 차이는 측정 가능한 영역 밖.

Apetrei 의 이론적 강점(`buildTree_* + bottomUpBoxes` 두 dispatch → 단일
dispatch 로 통합) 은 큰 primitive 수에서 dispatch overhead 가 줄어드는
이득이 있어야 보이지만, 본 워크로드에선 sort + leaf 빌드가 dominate.

### (2) 매 프레임 rebuild 가 refit 보다 가볍다 (의외)

케이스 1 (refit-주축) 의 build+refit 이 ~58 ms 인데, 케이스 2/3 (매번
full rebuild) 은 ~46 ms — **rebuild 쪽이 12 ms 더 가볍다**. 이유:
- refit 의 `bottomUpHybrid` 는 GPU 부분 워크 + CPU 마무리 구조. CPU 마무리
  단계에서 `commitAndWait` 가 강제되어 substep 마다 GPU pipeline 이 stall.
- build 의 buildTreeGPU + bottomUpHybrid 도 같은 commit 을 갖지만, 1프레임에
  여러 substep refit 이 누적되는 case 1 보다, 1프레임 1번의 build 인
  case 2/3 의 commit 횟수가 적다.

즉 본 워크로드에서는 **substep 마다 refit 하는 비용 > 프레임 시작마다
한 번 full rebuild** — refit 의 이론적 강점(토폴로지 재사용)이 CPU 마무리
오버헤드에 가려진다. 이는 `bottomUpHybrid` 의 CPU 단계를 제거(pure-GPU 풀워크)
하면 refit 이 다시 우세해질 수 있다는 단서.

### (3) Query 비용은 세 케이스 모두 동일 (~25 ms)

build 모드와 무관하게 query 시간이 일정 — broad-phase 의 비용은 트리
quality 보다는 충돌쌍 개수에 의해 dominate. 또한 GPU sync 가 정상이라
build 비용이 query 로 흡수되지 않음.

## 결론

1. **Apetrei vs Karras**: 본 워크로드에서 측정 차이 없음. 큰 primitive
   수에서 dispatch overhead 가 의미있어지면 다시 측정 필요.
2. **Refit vs Rebuild**: 의외로 **rebuild 가 가볍다**. 이는 현재
   `bottomUpHybrid` 의 CPU 마무리 단계가 refit 의 발목을 잡는다는 시그널.
   → 다음 실험 후보: `bottomUpHybrid` 를 pure-GPU 풀워크로 (Apetrei 의 atomic-gate
   bottom-up 패턴 활용) 바꾸고 refit 비용을 다시 측정.

## 파일

- `source.csv` — 원본 프로파일 (90 행, 수정 후 측정)
- `chart.py`   — 분석 스크립트 (직전 폴더와 동일)
- `build_refit.png` — 차트 1
- `query.png` — 차트 2
