# ysim Projective Dynamics 시스템 수식·알고리즘 복원 및 구현 감사

- 감사 대상: `ysim` commit `c035149`
- 기준일: 2026-07-30
- 주 구현: [`include/sim/pd_system.hpp`](include/sim/pd_system.hpp)
- 실행 경로: [`include/sim/simulator.hpp`](include/sim/simulator.hpp)
- 비교 대상:
  - [Liu et al., *Fast Simulation of Mass-Spring Systems*, 2013](https://graphics.berkeley.edu/papers/Liu-FSM-2013-11/Liu-FSM-2013-11.pdf)
  - [Bouaziz et al., *Projective Dynamics: Fusing Constraint Projections for Fast Simulation*, 2014](https://infoscience.epfl.ch/server/api/core/bitstreams/a884fb0a-5fb5-49b4-a0ea-b88bd19a7e47/content)

## 1. 결론

### 종합 판정

**일반 `TriangularCloth`의 스프링 기반 PD 핵심은 올바르게 구현되어 있다.**  
관성 예측, Hooke 스프링의 local projection, 상수 라플라시안 행렬 조립, sparse LDLT 사전 분해, local/global 반복은 Liu 2013 및 Bouaziz 2014의 식과 일치한다.

다만 **현재 `PdSystem` 전체를 “논문식 PD가 완전하게 구현됐다”고 판정할 수는 없다.**

| 영역 | 판정 | 핵심 이유 |
|---|---|---|
| 관성·외력 항 | 적합 | \(s=x+h v+h^2M^{-1}f_{\mathrm{ext}}\)를 그대로 사용 |
| stretch/bend 스프링 PD | 대체로 적합 | local projection과 global matrix/RHS가 표준 mass-spring PD와 일치 |
| 행렬 캐시·재분해 | 적합 | topology, \(h\), 질량, pin, stiffness 변경을 감지하고 재분해 |
| 정적 장애물 one-sided contact | 대체로 적합 | unilateral half-space의 최근접점 투영을 PD 에너지에 포함 |
| cloth–cloth/self contact | 근사 확장 | 정확한 4-vertex PD 제약이 아니라 네 개의 독립 대각 위치 제약으로 분해 |
| `FastGridCloth` 지원 | **부적합** | `kshear`와 FastGrid 고유 스프링 토폴로지·방향별 rest length를 무시 |
| 길이 0 스프링 처리 | **부적합** | local projection이 제약집합 \(\|d\|=r\) 밖의 \(d=0\)을 선택 |
| 테스트 증거 | 부분 충분 | PD-1~PD-8은 통과하지만 위 두 결함과 정확한 two-way 에너지 성질은 미검증 |

따라서 가장 정확한 표현은 다음과 같다.

> 현재 구현은 `TriangularCloth`를 위한 실용적인 spring Projective Dynamics solver이며, 정적 접촉은 PD식으로 통합되어 있다. 반면 FastGrid 모델과 동적 양방향 접촉까지 포함한 “일반 PD 시스템”으로는 아직 모델 불일치와 근사 처리가 남아 있다.

### 핵심 소스 근거 위치

| 내용 | 근거 |
|---|---|
| PD solver dispatch | [`simulator.hpp` L3571–3597](include/sim/simulator.hpp#L3571-L3597) |
| 스프링 생성·rest length | [`pd_system.hpp` L508–611](include/sim/pd_system.hpp#L508-L611) |
| momentum target·base matrix | [`pd_system.hpp` L645–825](include/sim/pd_system.hpp#L645-L825) |
| contact set·contact diagonal | [`pd_system.hpp` L858–1071](include/sim/pd_system.hpp#L858-L1071) |
| spring local step·RHS | [`pd_system.hpp` L1088–1218](include/sim/pd_system.hpp#L1088-L1218) |
| one-sided/two-way contact target | [`pd_system.hpp` L1221–1407](include/sim/pd_system.hpp#L1221-L1407) |
| global solve | [`pd_system.hpp` L1419–1450](include/sim/pd_system.hpp#L1419-L1450) |
| rigid coupling·velocity commit | [`pd_system.hpp` L1453–1583](include/sim/pd_system.hpp#L1453-L1583) |
| GPU FastGrid의 실제 3-family 모델 | [`physics.metal` L162–218](src/metal/physics.metal#L162-L218) |
| PD-1~PD-8 tests | [`self_test_inline.hpp` L7386–8117](test/self_test_inline.hpp#L7386-L8117) |

## 2. 시스템이 실제로 푸는 문제

### 2.1 기호

| 기호 | 의미 | 코드 |
|---|---|---|
| \(n\) | 활성 cloth mesh의 정점 수 | `mesh.state.x.size / 3` |
| \(q\in\mathbb{R}^{n\times3}\) | 이번 substep의 미지 위치 | `MeshCache::q` |
| \(x,v\) | substep 시작 위치와 속도 | `mesh.state.x`, `mesh.state.v` |
| \(M=\operatorname{diag}(m_i)\) | lumped mass matrix | `mesh.state.m[i*3]` |
| \(h\) | substep 크기 | `system.subh`, `PdSystem::step(..., dt)` |
| \(f_{\mathrm{ext}}\) | 중력·바람 외력 | `mesh.externalForces.externalForces` |
| \(e=(a,b)\) | 스프링 양 끝 정점 | `Spring::a`, `Spring::b` |
| \(r_e\) | 스프링 rest length | `Spring::rest` |
| \(k_e\) | stretch 또는 bend stiffness | `kS`, `kB` |
| \(B_e\) | \(a\)에 \(+1\), \(b\)에 \(-1\)인 incidence row | `Incident::sign` |
| \(d_e\) | local step의 projected spring vector | `MeshCache::springD` |

### 2.2 관성 예측

각 자유 정점의 momentum target은 다음과 같다.

$$
s_i=x_i+h v_i+h^2m_i^{-1}f_{\mathrm{ext},i}.
$$

코드는 이를 다음 순서로 계산한다.

```text
accel = externalForce / mass
s     = x + (v + h * accel) * h
q⁰    = s
```

고정 정점은 \(m_i^{-1}=0\)으로 취급하여 \(s_i=x_i\)이다. 이 \(s\)를 초기 iterate로 사용하는 것은 Bouaziz Algorithm 1의 momentum warm start와 일치한다.

외력 버퍼에는 [`Simulator::applyEnvironmentForces`](include/sim/simulator.hpp)가

$$
f_{\mathrm{ext},i}=m_i g+f_{\mathrm{wind}}
$$

를 채운다. 따라서 PD 쪽에서 다시 \(m_i^{-1}\)을 곱하는 것은 올바르며, 중력이 이중 적용되지 않는다.

### 2.3 스프링 에너지

각 스프링은 고전적인 Hooke potential을 사용한다.

$$
E_e(q)=\frac{k_e}{2}
\left(\left\|q_a-q_b\right\|-r_e\right)^2.
$$

Liu 2013의 auxiliary variable을 도입하면 다음과 동치다.

$$
E_e(q)=
\min_{\|d_e\|=r_e}
\frac{k_e}{2}\left\|(q_a-q_b)-d_e\right\|^2.
$$

접촉을 제외한 코드의 실제 목적함수는 다음과 같다.

$$
\begin{aligned}
F_{\mathrm{smooth}}(q,d)
={}&
\frac{1}{2h^2}\sum_i m_i\|q_i-s_i\|^2 \\
&+\frac{w_{\mathrm{pin}}}{2}
\sum_{i\in\mathcal P}\|q_i-x_i\|^2 \\
&+\frac12\sum_{e\in\mathcal E}
k_e\|B_eq-d_e\|^2,
\end{aligned}
$$

여기서 \(w_{\mathrm{pin}}=10^8\)이다.

### 2.4 Local step

정상적인 스프링, 즉 \(\|q_a-q_b\|>10^{-9}\)이면 코드가 계산하는 projection은

$$
d_e
=r_e\frac{q_a-q_b}{\|q_a-q_b\|}.
$$

이는 반지름 \(r_e\)인 구면 \(\{d:\|d\|=r_e\}\)에 대한 정확한 최근접점 투영이다.

### 2.5 Global step

고정된 \(d_e\)와 contact target \(p_c\)에 대해 목적함수는 \(q\)의 이차식이다. 코드가 푸는 선형계는

$$
Aq=b
$$

이며,

$$
A=
\frac{M}{h^2}
+\underbrace{\sum_e k_eB_e^\mathsf TB_e}_{L}
+w_{\mathrm{pin}}P
+D_{\mathrm{contact}},
$$

$$
b=
\frac{M}{h^2}s
+w_{\mathrm{pin}}Px
+\sum_e k_eB_e^\mathsf Td_e
+\sum_c w_cS_c^\mathsf Tp_c.
$$

- \(L\): stiffness-weighted graph Laplacian
- \(P\): pinned vertex diagonal selector
- \(D_{\mathrm{contact}}\): 정점별 contact weight 합을 대각에 더한 행렬
- \(S_c\): contact가 걸린 정점을 고르는 selector

코드의 \(M/h^2+L\) 형식은 Liu 논문의 \(M+h^2L\)을 전체적으로 \(h^2\)로 나눈 것과 같으므로 수학적으로 동일하다.

행렬은 좌표축과 무관한 scalar \(n\times n\) 행렬이다. 같은 LDLT factor를 사용해 \(x,y,z\) 세 RHS를 독립적으로 back-substitution한다.

## 3. 스프링 토폴로지

### 3.1 Stretch

`buildSprings`는 `mesh.adjacency.edges`의 모든 유효 edge를 stretch spring으로 만든다.

- rest length: `adjacency.restEdgeLengths`
- weight: `stretch * clothStiffnessScale`
- 삼각 격자의 diagonal edge도 stretch spring으로 분류

따라서 `ClothBehaviorParams::shear`는 PD에서 사용되지 않는다. 현재 GPU `TriangularCloth` 경로도 모든 mesh edge에 `kstretch`를 적용하므로 이 부분은 두 경로가 같은 제한을 가진다. 그러나 UI/직렬화에 존재하는 `shear` 값이 실제 `TriangularCloth` 물성에 영향을 주지 않는다는 점은 명시할 필요가 있다.

### 3.2 Bend

내부 edge를 공유하는 두 삼각형에서 edge 반대편 정점 \(p_3,p_4\)를 찾아 거리 스프링을 만든다.

$$
E_{\mathrm{bend}}
=\frac{k_b}{2}
\left(\|q_{p_3}-q_{p_4}\|-r_b\right)^2.
$$

이는 dihedral-angle bending이나 Bouaziz의 mean-curvature bending element가 아니라 **Provot식 opposite-vertex distance spring**이다.

- 장점: scalar graph Laplacian을 유지하므로 구현과 분해가 단순하다.
- 한계: 실제 곡률/dihedral 에너지와 동일하지 않으며, mesh tessellation 의존성이 있다.
- bend rest length는 topology cache 최초 생성 시의 현재 자세에서 측정된다.

## 4. 한 substep의 전체 알고리즘

```text
입력: xⁿ, vⁿ, mass, fixed mask, external force, narrow-phase contacts, h

0. GPU 작업 완료를 기다린다.

1. 모든 활성 cloth mesh에 대해
   1.1 topology가 바뀌면 stretch/bend spring과 incidence CSR을 재구성한다.
   1.2 h, stiffness, mass, pin mask가 바뀌면
       A_base = M/h² + L + w_pin P 를 재조립·분해한다.
   1.3 s = xⁿ + h vⁿ + h² M⁻¹ f_ext 를 계산한다.
   1.4 q⁰ = s 로 warm start한다.

2. 이번 substep의 contact set을 만든다.
   2.1 정적/rigid/비활성 target → one-sided plane constraint
   2.2 활성 cloth target 또는 self → two-way vertex-triangle row
   2.3 정점별 contact weight 합이 이전 epoch와 다르면
       A = A_base + diag(contactW)를 numeric refactor한다.

3. iterations 회 반복한다.
   3.1 LOCAL spring:
       각 spring을 rest-length sphere에 projection하여 d_e 계산
   3.2 spring RHS:
       각 정점이 incident spring의 ±k d_e를 gather
   3.3 LOCAL contact:
       one-sided projection target과 two-way 분할 target을 RHS에 추가
   3.4 GLOBAL:
       각 mesh, 각 좌표축에 대해 A q = b 풀이

4. cloth→rigid positional coupling을 substep당 한 번 누적한다.

5. v = (q - xⁿ)/h 로 갱신하되
   contact depenetration이 과도한 발사 속도가 되지 않도록 normal 성분을 제한한다.
   마지막으로 v *= (1 - damping), x = q 를 commit한다.
```

`Simulator::update`는 narrow phase를 매 substep 갱신한 뒤 `pd.step(scene, system.subh)`를 호출한다. 제공 PD 장면의 기본 운용점은 frame당 3 substeps, substep당 10 local/global iterations이다.

## 5. 접촉 처리 복원

### 5.1 One-sided contact: PD식 unilateral plane

대상이 정적 장애물, Float, analytic shape, Rigid body 또는 비활성 cloth이면 정점 하나에 대한 half-space 제약으로 처리한다.

substep 시작 위치 \(x_i\)에서 narrow phase가 준 signed distance를 \(d_0\), 정규화 normal을 \(n\), cloth thickness를 \(t\)라 하면 live iterate의 거리는

$$
d(q_i)=d_0+n^\mathsf T(q_i-x_i).
$$

동적 rigid coupling이 있으면 이미 누적된 rigid 이동량도 뺀다.

제약집합은

$$
\mathcal C=\{z:d(z)\ge t\}.
$$

Local projection은

$$
p_i=
\begin{cases}
q_i, & d(q_i)\ge t,\\
q_i+(t-d(q_i))n, & d(q_i)<t.
\end{cases}
$$

따라서 접촉 에너지는

$$
E_c(q_i)
=\min_{p_i\in\mathcal C}\frac{w_c}{2}\|q_i-p_i\|^2
=\frac{w_c}{2}\max(0,t-d(q_i))^2.
$$

이는 Bouaziz 2014의 unilateral plane collision과 같은 구조다. 만족된 제약은 \(p_i=q_i\)가 되어 sticking을 만들지 않는다.

가중치는 물성 기반 contact modulus가 아니라 정점 관성항의 배수다.

$$
w_c=\alpha\frac{m_i}{h^2},
\qquad
\alpha=\texttt{kContactWeightScale},
$$

기본값은 \(\alpha=1\)이다. 한 정점이 여러 contact row를 가지면 각 row가 이 가중치를 하나씩 추가한다.

### 5.2 Cloth–cloth/self contact: 4개 대각 target으로 분해한 근사

query vertex를 \(q\), target triangle을 \(p_1,p_2,p_3\)라 한다. 현재 triangle normal \(\hat n\)의 방향은 narrow-phase entry normal과 내적하여 고정한다.

$$
n_s=\sigma\hat n,\qquad
\sigma=\operatorname{sign}(n_{\mathrm{entry}}^\mathsf T\hat n).
$$

두 cloth 두께 중 큰 값을 \(t\)라 하면 코드의 unilateral 값은

$$
C=n_s^\mathsf T(q-p_1)-t.
$$

\(C\ge0\)이면 identity target을 사용한다. \(C<0\)이면 query point를 triangle plane에 투영해 clamped barycentric 좌표 \((b_1,b_2,b_3)\)를 구하고,

$$
W=w_q+b_1^2w_1+b_2^2w_2+b_3^2w_3
$$

에 대해 PBD식 mass-weighted correction을 계산한다.

$$
\Delta q=-\frac{Cw_q}{W}n_s,
$$

$$
\Delta p_j=\frac{Cw_jb_j}{W}n_s.
$$

여기서 \(w_q,w_j\)는 inverse mass이다. 그 뒤 하나의 결합 제약을 그대로 넣지 않고

$$
p_q=q+\Delta q,\qquad
p_j=p_j+\Delta p_j
$$

라는 **네 개의 독립 \(A=B=I\) 위치 target**으로 각 mesh RHS에 넣는다.

이는 실용적인 반복 선형화이지만 정확한 4-vertex PD contact는 아니다. 정확한 한 행의 선형화는 대략

$$
A=[I,-b_1I,-b_2I,-b_3I]
$$

형태이고, \(A^\mathsf TA\)는 query와 triangle 정점 사이의 off-diagonal block을 만든다. 현재 구현은 이 block들을 만들지 않고 모든 contact 항을 대각에만 더한다.

결과적으로:

- self contact도 contact 자체는 대각 근사이며 spring matrix를 통해서만 간접 결합된다.
- 서로 다른 두 cloth는 각자의 별도 선형계를 풀므로 mesh 사이에서는 Jacobi식 fixed-point iteration이다.
- 표준 PD의 고정 목적함수에 대한 단조 감소 보장은 적용되지 않는다.
- 질량가중 target 자체는 합리적이지만, 여러 row가 겹치면 global response의 정확한 선·각운동량 보존은 보장되지 않는다.

코드 주석도 이 처리가 Bouaziz 논문의 정확한 4-vertex constraint 또는 merged multi-mesh solve가 아님을 인정하고 있다.

### 5.3 속도와 rigid coupling

표준 PD는 마지막에

$$
v^{n+1}=\frac{q^{n+1}-x^n}{h}
$$

를 사용한다. 현재 구현은 큰 초기 관통 보정이 발사 속도로 변환되는 것을 막기 위해 contact normal 방향의 displacement를 접근 속도 범위로 clamp한다.

이는 안정성을 위한 명시적 후처리이며 원 목적함수의 일부는 아니다. 마찰과 restitution 모델은 구현되어 있지 않다.

cloth→dynamic rigid 반작용도 PD 선형계나 impulse solve에 함께 들어가지 않는다. substep에서 실현된 위치 보정의 일부를 `rigidDelta`에 누적하고 frame 끝에 Bullet body를 이동시키는 별도 coupling이다. 실용적이지만 엄밀한 동시 운동량 보존 접촉 solve는 아니다.

## 6. 행렬 캐시와 병렬화

### 6.1 Base matrix 재구성 조건

다음이 변하면 스프링 또는 base matrix를 다시 만든다.

- mesh `lifetimeId`
- 정점·facet·edge count
- \(h\)
- stretch/bend stiffness
- vertex mass
- pin mask

topology가 같으면 stretch/bend rest length를 다시 측정하지 않는다.

### 6.2 Contact epoch

contact는 모두 정점 대각에만 가중치를 추가하므로 sparse pattern은 변하지 않는다. 구현은 정점별 contact weight 합

$$
\gamma_i=\sum_{c\ni i}w_c
$$

을 epoch key로 사용한다.

- \(\gamma\)가 이전 substep과 같으면 기존 factor 재사용
- 다르면 \(A_{\mathrm{base}}\)를 복사하고 diagonal만 변경
- symbolic analysis는 재사용하고 numeric `factorize()`만 실행

이는 rank update 대신 구현 복잡도를 줄인 합리적인 선택이다. 다만 contact 수와 분포가 매 substep 변하면 numeric factorization 비용이 계속 발생한다.

### 6.3 병렬화

- spring projection: spring별 병렬
- RHS gather: vertex별 병렬
- global solve: 좌표축 3개 병렬
- 1024 정점 미만은 dispatch overhead를 피하려고 직렬 실행
- contact RHS assembly는 충돌 row 수가 작고 같은 vertex row에 쓰기 때문에 직렬

## 7. 확인된 문제와 위험

### F1. `FastGridCloth`의 PD 모델이 선택된 물성 모델과 다르다 — 높음

`springConstantsOf`는 FastGrid에서 `kstretch`, `kbend`만 읽고 `kshear`를 버린다. `buildSprings`도 FastGrid 전용 구조를 사용하지 않고 일반 triangle adjacency를 사용한다.

그러나 GPU FastGrid 모델은 다음 세 family를 명시적으로 구분한다.

1. 수평·수직 1-hop stretch: `kstretch`
2. 두 방향 diagonal shear: `kshear`
3. 수평·수직 2-hop bend: `kbend`

또한 비균일 scale을 지원하기 위해 여섯 개의 방향별 rest length를 가진다.

현재 PD FastGrid는:

- 모든 triangle edge, diagonal 포함 → `kstretch`
- interior edge의 opposite-vertex pair → `kbend`
- `kshear` → 미사용
- FastGrid의 2-hop bend rest length → 미사용

따라서 `FastGridCloth`에서 solver만 GPU Symplectic ↔ PD로 바꾸면 같은 물성을 암시하지만 실제로는 다른 스프링 네트워크를 푼다. 현재 PD 테스트는 모두 `addCloth`, 즉 `TriangularCloth`만 사용하여 이 문제를 검출하지 못한다.

**권고:** FastGrid 전용 spring builder를 구현하기 전에는 PD 선택을 `TriangularCloth`로 제한하거나 UI에 “PD에서 FastGrid는 근사 변환됨”을 명시해야 한다.

### F2. coincident spring의 local projection이 잘못되어 복원력을 잃는다 — 높음

현재 코드는

```cpp
if (len > 1e-9) {
    d = rest * edge / len;
} else {
    d = edge; // zero
}
```

를 사용한다.

rest length \(r>0\)인 스프링의 local 제약은 \(\|d\|=r\)인데, edge가 0이면 코드가 선택하는 \(d=0\)은 제약집합 위에 있지 않다.

진짜 Hooke energy는

$$
E=\frac{k}{2}r^2
$$

이지만, debug objective가 계산하는 \(\frac{k}{2}\|0-0\|^2\)는 0이다. 즉:

- local step이 정확한 최소화가 아니다.
- objective probe도 실제 Hooke energy를 보고하지 않는다.
- 완전히 접힌/압축된 스프링은 임의의 방향 폭발을 피하는 대신 스스로 펴질 방향도 잃는다.
- “각 local/global step이 목적함수를 약하게 감소시킨다”는 증명 조건이 이 경우 깨진다.

**권고:** `Spring`에 rest-pose 방향을 저장하고 coincident 시 그 방향을 사용하거나, 직전 유효 `d_e`를 유지한 뒤 deterministic fallback axis를 사용해야 한다. 어느 경우든 \(\|d_e\|=r_e\)를 지켜야 한다.

### F3. two-way/self contact는 정확한 PD contact가 아니다 — 중간

이는 코드가 숨기지 않고 주석으로 인정한 설계상의 근사다. 그러나 사용자 관점에서 `PD`라는 이름만 보면 merged contact energy를 푼다고 오해할 수 있다.

영향:

- off-diagonal contact coupling 부재
- mesh 간 Jacobi 수렴
- contact가 많은 장면에서 느린 수렴 또는 residual penetration 가능
- 전체 목적함수 단조 감소 및 운동량 보존을 일반적으로 증명할 수 없음

현재 PD-5/PD-6은 특정 stack/fold 장면이 유한하고 뚫리지 않으며 과도한 에너지를 만들지 않는지만 확인한다. 일반적인 각도, 질량비, 다중 접촉, 고속 충돌에 대한 정량 검증은 아니다.

### F4. PD-8 검증 논리가 일반 보장과 맞지 않는다 — 중간

PD-8은 contact를 포함한 global solve를 수행하면서 `momentum + pin + spring` 부분만 단조 감소한다고 gate한다. 하지만 global solve는 **전체 목적함수**를 최소화하므로 contact 항을 줄이기 위해 smooth 부분만 증가할 수 있다. smooth 부분 단독의 단조 감소는 수학적으로 보장되지 않는다.

반대로 PD-8의 실제 장면은 one-sided floor contact뿐이므로, 이 경우에는 one-sided contact auxiliary까지 포함한 **전체 목적함수**가 정확한 block coordinate descent의 검사 대상이다. 현재 실행에서는 full sequence도 감소했지만 test는 이를 gate하지 않는다.

two-way contact가 들어간 경우에는 full objective도 고정된 정확한 PD energy가 아니므로 별도의 fixed-point residual, penetration, momentum drift 검사가 필요하다.

### F5. pin은 solve 안에서는 soft, commit에서는 hard다 — 낮음

행렬에는 \(w_{\mathrm{pin}}=10^8\)의 penalty를 넣지만 최종 commit에서는 pinned vertex의 \(q\)를 쓰지 않고 기존 \(x\)를 그대로 유지한다.

따라서:

- 최종 출력 위치에서 pin은 정확하다.
- 이웃 정점은 아주 조금 움직인 soft-pin iterate를 보고 계산된다.
- 반환된 mesh state는 엄밀하게는 solve한 목적함수의 해와 동일하지 않다.

현재 stiffness 규모에서는 오차가 작고 테스트도 pin drift 0을 확인하므로 실용상 낮은 위험이다.

### F6. solver 적용 범위가 UI 선택 범위보다 좁다 — 낮음

PD는 동적 `TriangularCloth`와 `FastGridCloth`만 처리한다.

- `Elastic`, `Fluid`, `Generator` 등은 처리하지 않는다.
- `Scene::referenceConstraints`의 leader/follower coincidence constraint는 PD 경로에 없다.
- 마찰과 restitution이 없다.

solver를 런타임 전역 토글로 제공하므로, 지원되지 않는 제약이나 behavior가 있는 장면에서 조용히 기능이 빠질 수 있다.

## 8. 실행 검증 결과

현재 `build/ysim --self-test`를 실행했다.

### PD 전용 결과

다음 8개는 모두 통과했다.

1. PD-1: pinned sheet가 유한하고 pin을 유지하며 중력으로 처짐
2. PD-2: stretch 평균 오차 제한
3. PD-3: 자유낙하 center-of-mass 속도
4. PD-4: 초기 관통 cloth의 depenetration launch 방지
5. PD-5: cross-cloth two-way stack
6. PD-6: self-contact folded sheet
7. PD-7: 반복 횟수 변화에 따른 준정적 처짐 차이 제한
8. PD-8: 현재 장면에서 objective sequence 감소

관측된 주요 수치:

- PD-7: 10회와 40회 반복의 min-y 차이 약 \(3.31\times10^{-4}\,\mathrm m\)
- PD-8 smooth objective: 약 \(1.08914\rightarrow1.06552\)
- PD-8 full objective: 약 \(1.13255\rightarrow1.09477\)
- PD-8 contact row: 52개, 반복 10회

### 전체 self-test 상태

프로세스 종료 코드는 실패였다. 총 2개 실패가 있었으나 둘 다 PD와 무관한 Preview 동기화 테스트다.

- `D-042 R-3 / Scene::pack memcpys preview x ...`
- `D-042 R-4 / rotateObject writes preview.x ...`

따라서 “전체 테스트 suite 통과”라고 말할 수는 없지만, 현재 포함된 PD 전용 gate는 모두 통과했다.

### 현재 테스트가 다루지 않는 항목

- `FastGridCloth`에서 `kshear` 변화가 PD 결과에 반영되는지
- GPU FastGrid와 PD의 spring family/rest-length parity
- rest length가 양수인 coincident spring의 복원
- one-sided contact를 포함한 전체 목적함수의 일반 단조성
- two-way/self contact의 fixed-point residual
- two-way contact 전후의 선운동량/각운동량 drift
- 큰 질량비, 여러 contact normal, 고속 cloth–cloth 충돌
- reference-point constraint가 있는 장면의 PD 동작

## 9. 우선순위별 개선안

### P0 — 모델 정확성

1. **FastGrid 전용 PD spring topology 구현**
   - structural X/Y 1-hop → `kstretch`
   - diagonal A/B → `kshear`
   - X/Y 2-hop → `kbend`
   - 여섯 방향별 rest length 사용

2. **coincident spring projection 수정**
   - rest-pose direction 저장
   - 직전 유효 projection 재사용
   - 항상 \(\|d_e\|=r_e\) 보장

3. 위 둘을 독립 회귀 테스트로 추가

### P1 — 접촉의 수학적 일관성

1. one-sided 장면에서는 full objective 단조 감소를 gate
2. two-way 접촉은 정확한 \(A=[I,-b_1I,-b_2I,-b_3I]\) block을 사용하는 merged solve 또는 명시적인 outer fixed-point residual 기준 도입
3. cloth–cloth contact 전후 COM, 운동량, penetration을 정량 검사
4. `PD contact`와 `approximate two-way PD contact`를 UI/문서에서 구분

### P2 — 기능 일관성

1. hard Dirichlet pin elimination 또는 정확한 constrained solve 검토
2. PD 경로에 reference-point constraint 추가
3. 지원하지 않는 behavior/constraint가 있을 때 경고
4. 마찰·restitution을 명시적으로 구현하거나 미지원 상태를 UI에 표시

## 10. 최종 요약

소스가 푸는 기본 식은 명확한 spring Projective Dynamics다.

$$
\min_{q,\{\|d_e\|=r_e\}}
\frac{1}{2h^2}\|M^{1/2}(q-s)\|^2
+\sum_e\frac{k_e}{2}\|B_eq-d_e\|^2
+E_{\mathrm{pin}}
+E_{\mathrm{contact}}.
$$

local step에서 spring vector와 contact target을 projection하고, global step에서

$$
\left(
\frac{M}{h^2}+L+w_{\mathrm{pin}}P+D_{\mathrm{contact}}
\right)q=b
$$

를 반복해서 푼다. 이 핵심은 잘 구현되어 있다.

하지만 현재 품질 판정은 다음 조건부다.

- **`TriangularCloth` + 정상 길이 스프링 + one-sided obstacle contact:** 구현 신뢰도 높음
- **cloth–cloth/self contact:** 작동 테스트를 통과한 근사 확장
- **`FastGridCloth`:** 선택된 물성 모델과 불일치하므로 수정 전에는 신뢰하기 어려움
- **완전히 겹친 스프링:** local projection 오류가 있으므로 수정 필요

즉, PD의 중심 알고리즘은 맞지만, 모든 지원 표면과 퇴화 상태까지 포함해 “잘 구현 완료”라고 선언하기 전에는 F1과 F2를 먼저 해결하는 것이 적절하다.
