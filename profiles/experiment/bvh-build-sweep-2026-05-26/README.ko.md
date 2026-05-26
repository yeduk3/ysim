# BVH 빌드 모드 × Particle Count 스윕 (2026-05-26)

`bvh-build-modes-fixed-2026-05-26` 의 후속. 단일 워크로드(메시 3-4개,
~24k 정점) 에서 봤던 "rebuild ≥ refit" 의외 결과가 particle count 에
따라 어떻게 바뀌는지 확인하기 위한 스윕.

## 변경 사항

- `bottomUpHybrid(sceneBox, 30)` 호출부를 `bottomUpBoxesGPU(sceneBox) +
  commitAndWait` 로 swap. 직전 실험에서 의심한 CPU 마무리 단계
  (`bottomUpCombineWithSkip`) 의 substep 누적 stall 을 제거.
- 헤드리스 bench `--bench-bvh-build` 신설. 기존 `--bench-bvh-refit`
  골격을 따르며 simulator 플래그 스윕으로 3 케이스 측정.

## 케이스

| 케이스 | enableRefit | useAgglomerative | 의미 |
|---|---|---|---|
| 1. RefitON_Karras   | true  | false | 10프레임마다 rebuild, 그 외 substep 은 refit |
| 2. RebuildKarras    | false | false | 모든 substep 마다 Karras rebuild |
| 3. RebuildApetrei   | false | true  | 모든 substep 마다 Apetrei rebuild |

각 케이스 × 4 particle 수 × 30 프레임 × 60 substep. 첫 측정 프레임(0)
은 워밍업 영향을 받아 분석에서 제외.

## 실행 방법

```sh
# 1. bench
./build/src/ysim --bench-bvh-build
# → build_bench.csv 생성

# 2. chart
python3 chart.py
# → build_chart_line.png, build_chart_bar.png 생성
```

## 결과 요약 (mean ± stddev, 프레임 1..29)

| Particle | RefitON Karras | Rebuild Karras | Rebuild Apetrei |
|---:|---:|---:|---:|
|     1,024 | **55.2 ± 6.2** ms | 44.1 ± 1.6 ms | **43.4 ± 1.4** ms |
|    10,000 | **38.7 ± 12.5** ms | 64.7 ± 6.3 ms | 61.3 ± 1.5 ms |
|    99,856 | **72.3 ± 3.1** ms | 286.0 ± 33.0 ms | 272.2 ± 3.2 ms |
|   499,849 | **287.4 ± 9.3** ms | 1200.7 ± 13.5 ms | 1130.6 ± 5.6 ms |

(굵게 = 해당 particle 수에서 가장 가벼운 케이스)

## 관찰

### (1) 크로스오버: ~1k-10k particle 사이에서 refit 우세 시작

- **1,024 particle**: rebuild ~44 ms, refit ~55 ms — 작은 메시에선 매 substep
  refit 의 fixed cost(60회 dispatch overhead) 가 build 비용을 추월.
- **10,000 particle**: refit ~39 ms, rebuild ~62 ms — 이미 refit 이 1.6× 빠름.
- **99,856 particle**: refit ~72 ms, rebuild ~286 ms — refit 이 **약 4× 빠름**.
- **499,849 particle**: refit ~287 ms, rebuild ~1201 ms — refit 이 **4.2× 빠름**.

직전 실험(메시 3-4개 ~24k 정점) 에서 rebuild 가 refit 보다 12ms 가벼웠던
것은 CPU 마무리 stall 때문 — pure-GPU swap 후 본 스윕에선 10k 부터 refit 이
우세. 직전 실험의 워크로드에서도 swap 후 refit 이 이겼을 가능성이 큼.

### (2) Apetrei vs Karras: 5-10% 일관된 우위

| Particle | Apetrei / Karras (rebuild only) |
|---:|---:|
|     1,024 | 0.984 (1.6% faster) |
|    10,000 | 0.947 (5.3% faster) |
|    99,856 | 0.951 (4.9% faster) |
|   499,849 | 0.942 (5.8% faster) |

큰 메시일수록 우위가 약간 커지는 경향. 단일 dispatch 통합의 이득이
sort + leaf 비용에 비례하지 않고 dispatch overhead 가 줄어드는 효과로
보임. 큰 메시에서 5-10% 라는 작은 우위는 ysim 전체 워크로드에서 본격
도입할 가치가 있는 정도 — 단, 이 측정은 `commitAndWait` 강제 동기 환경
이라 비동기 dispatch 가 자연스러운 실제 파이프라인에선 더 큰 차이가 날
가능성도 있음.

### (3) Refit 비용 모양

Refit 그래프(파란선) 는 1k → 10k 구간에서 살짝 감소(38.7 ms) 후 100k → 500k
구간에서 거의 4× 만 증가(72 → 287 ms). 거의 선형 — `buildLeafGPU` +
`bottomUpBoxesGPU` 둘 다 O(N) 이며 sort 가 없기 때문. 반면 rebuild 는 sort
포함 O(N log N) 이라 500k 에서 1200 ms 까지 폭증.

## 결론

1. **본 프로젝트 워크로드(대부분 cloth ~10k+) 에선 refit ON 이 default 로
   적절**. 단, ysim 의 cloth 가 ~1k 수준이고 강한 외력으로 토폴로지가 크게
   왜곡되는 시나리오면 rebuild 가 더 안정적.
2. **Apetrei 빌더는 큰 메시일수록 우위 (~5-6%)**. 영구 도입 시 작지만 일관된
   이득. 다만 단일 워크로드에 묻혀 ROI 는 제한적.
3. **다음 단계 후보**:
   - 본 swap 으로 `bottomUpHybrid` 의 D-030 hybrid 변형 (Hybrid D1/D2) 은
     더 이상 production 에서 안 쓰임 → bench-only 함수로 디귤레이션 검토.
   - Refit 자체의 atomic-gate bottom-up 도 Apetrei 와 동일 패턴이므로
     `bottomUpBoxesGPU` 내부에 추가 최적화 여지(예: warp-level scan)
     가 있는지 별도 측정.

## 파일

- `build_bench.csv` — bench 출력 (3 × 4 × 30 = 360 행)
- `chart.py` — 분석 스크립트
- `build_chart_line.png` — log-log line chart (particle count 스윕)
- `build_chart_bar.png` — particle count 별 그룹 막대그래프
