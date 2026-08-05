# PD continuum cloth와 coupled contact 변경 설계

> 상태: **구현 전 설계 기준**
>
> 작성: 2026-07-30
>
> 대상: CPU `PdSystem`의 `TriangularCloth`
>
> 관련 문서: [Collision Pipeline](../COLLISION_PIPELINE.md),
> [PBD/충돌 handoff](pbd-system-handoff.md)

## 1. 목적과 판정 기준

현재 [`PdSystem`](../../include/sim/pd_system.hpp)은 local/global 반복, 관성 항,
상수 행렬 조립과 sparse factorization이라는 PD의 계산 구조를 갖고 있다. 하지만
옷감의 탄성은 edge stretch spring과 opposite-vertex bend spring으로 모델링되어
있다. 이 모델은 Liu et al.의 mass-spring PD에는 해당하지만, Bouaziz et al.이
삼각형 mesh에 제시한 continuum strain 및 mean-curvature bending potential은
아니다.

이 변경의 목표는 다음과 같다.

1. `TriangularCloth`의 면내 탄성을 삼각형별 strain potential로 교체한다.
2. bending을 opposite-vertex spring에서 cotangent mean-curvature potential로
   교체한다.
3. 고정 장애물 접촉은 unilateral constraint로 유지하되, cloth–cloth와
   self-contact는 접촉에 참여하는 모든 정점을 하나의 coupled constraint로
   구성한다.
4. 접촉 강성을 런타임에 조절할 수 있게 유지하고, PD 선택 시에만 GUI에
   노출한다.
5. 접촉 검출 실패와 접촉 solve의 잔류 침투를 서로 다른 문제로 계측한다.

이 문서에서 “논문식 PD 구현 완료”는 단순히 local/global loop가 존재한다는 뜻이
아니다. 아래 strain, bending, coupled contact 목적함수가 실제 global system에
조립되고 각각의 검증 조건을 통과해야 한다.

## 2. 목표 목적함수

한 substep의 미지 위치를 \(q\), 시작 위치와 속도를 \(x^n,v^n\), substep 크기를
\(h\), lumped mass matrix를 \(M\), 외력을 \(f_{\mathrm{ext}}\)라 하자. 관성
예측점은

$$
s=x^n+h v^n+h^2M^{-1}f_{\mathrm{ext}}
$$

이다. 목표 시스템은 다음 에너지를 local/global 반복으로 최소화한다.

$$
\begin{aligned}
E(q)={}&
\frac{1}{2h^2}\left\|M^{1/2}(q-s)\right\|^2 \\
&+\sum_{t\in\mathcal T}E_t^{\mathrm{strain}}(q)
+\sum_{i\in\mathcal V}E_i^{\mathrm{bend}}(q)\\
&+\sum_{c\in\mathcal C}E_c^{\mathrm{contact}}(q)
+E_{\mathrm{pin}}(q).
\end{aligned}
$$

삼각형 strain, cotangent bending, contact의 선형 operator는 rest data 또는 한
contact epoch 동안 고정한다. 따라서 local projection을 고정하면 global step은
여전히 선형 solve가 된다.

## 3. 삼각형 strain potential

### 3.1 Rest data

각 rest triangle \(t=(i,j,k)\)를 로컬 2차원 평면에 등거리로 펼친 좌표를
\(\bar u_i,\bar u_j,\bar u_k\in\mathbb R^2\)라 한다. 다음 값을 mesh topology
cache에 저장한다.

$$
D_m=
\begin{bmatrix}
\bar u_j-\bar u_i & \bar u_k-\bar u_i
\end{bmatrix}
\in\mathbb R^{2\times2},
$$

$$
D_m^{-1},\qquad
A_t=\frac12|\det D_m|.
$$

면적이 허용 오차 이하이거나 \(D_m\)이 역행렬을 가질 수 없는 퇴화 삼각형은
constraint를 만들지 않고 진단 카운터에 기록한다.

### 3.2 Deformation gradient와 에너지

현재 위치에서

$$
D_s(q)=
\begin{bmatrix}
q_j-q_i & q_k-q_i
\end{bmatrix}
\in\mathbb R^{3\times2}
$$

이고 삼각형의 deformation gradient는

$$
F_t(q)=D_s(q)D_m^{-1}\in\mathbb R^{3\times2}
$$

이다. \(F_t(q)=G_tq_t\)가 되도록 상수 operator \(G_t\)를 미리 계산한다.

삼각형 strain potential은

$$
E_t^{\mathrm{strain}}(q)=
\frac{w_sA_t}{2}
\min_{P_t\in\mathcal M}
\left\|F_t(q)-P_t\right\|_F^2
$$

로 둔다. 여기서 \(\mathcal M\)은 허용할 in-plane deformation 집합이다.

### 3.3 Local projection

thin SVD

$$
F_t=U\Sigma V^\mathsf T
$$

를 계산한다.

- corotated 모델은 \(P_t=UV^\mathsf T\)로 투영한다. \(3\times2\) 표현에서는
  두 열이 직교하는 rotation frame, 즉 Stiefel 집합으로의 투영이다.
- strain limiting은

  $$
  P_t=U\,
  \operatorname{clamp}(\Sigma,\sigma_{\min},\sigma_{\max})
  V^\mathsf T
  $$

  로 투영한다.

첫 구현은 논문과 옷감 용도에 직접 대응하는 singular-value limiting을 기본으로
한다. 기본 \(\sigma_{\min},\sigma_{\max}\)와 UI 노출 여부는 구현 slice에서
결정하되, 값은 solver iteration 수와 독립적인 물성 파라미터여야 한다.

SVD가 수치적으로 불안정한 삼각형은 직전 \(P_t\)를 사용하거나 rest rotation
frame으로 되돌린다. NaN을 global RHS에 넣어서는 안 된다.

### 3.4 Global assembly

고정된 \(P_t\)에 대해 삼각형 하나가 추가하는 항은

$$
K\mathrel{+}=w_sA_tG_t^\mathsf TG_t,
\qquad
b\mathrel{+}=w_sA_tG_t^\mathsf TP_t.
$$

\(G_t\)의 계수는 공간축에 공통이므로 contact가 없는 continuum elasticity
부분은 현재와 같이 하나의 scalar sparse matrix factorization을 \(x,y,z\)
세 RHS에 재사용할 수 있다.

### 3.5 기존 구현에서 제거할 것

`TriangularCloth`의 PD 경로에서는 다음 항을 재료 모델로 사용하지 않는다.

- mesh edge별 stretch distance spring
- diagonal edge를 암묵적으로 shear처럼 사용하는 방식
- stretch spring과 동일한 `Spring` 자료구조를 사용하는 bending

`FastGridCloth`의 기존 GPU/스프링 모델을 동시에 바꾸는 것은 이 변경의 범위가
아니다. 지원하지 않는 조합을 조용히 다른 물성으로 해석하지 말고 solver/behavior
지원표에 명시해야 한다.

## 4. Cotangent mean-curvature bending potential

### 4.1 Rest cotangent operator

정점 \(i\)의 one-ring 이웃을 \(N(i)\), rest mesh에서 계산한 mixed
Voronoi area를 \(A_i\)라 한다. 내부 edge \((i,j)\)의 cotangent weight는

$$
\tilde c_{ij}=\frac12(\cot\alpha_{ij}+\cot\beta_{ij})
$$

이며 경계 edge는 존재하는 한 각도만 사용한다. 면적 정규화를 포함한
Laplace–Beltrami row를

$$
L_iq=
\frac{1}{A_i}
\sum_{j\in N(i)}
\tilde c_{ij}(q_j-q_i)
$$

로 정의한다. 부호 convention은 구현 전체에서 하나로 고정하면 되고 에너지에는
제곱으로 들어가지만, rest vector와 current vector가 반드시 같은 convention을
사용해야 한다.

rest curvature vector는

$$
v_i^0=L_i\bar q
$$

이며 topology/rest-pose cache에 저장한다. cotangent와 \(A_i\)는 논문의
near-isometry 가정에 따라 rest mesh에서 계산해 고정한다.

### 4.2 에너지와 local projection

bending potential은

$$
E_i^{\mathrm{bend}}(q)=
\frac{w_bA_i}{2}
\min_{\|p_i\|=\|v_i^0\|}
\left\|L_iq-p_i\right\|^2
$$

이다. \(v_i=L_iq\)라 하면 local projection은

$$
p_i=
\|v_i^0\|\frac{v_i}{\|v_i\|}
$$

이다.

\(\|v_i\|\)가 허용 오차 이하일 때는 0으로 나누지 않는다. 이 경우 직전 방향,
rest curvature 방향, 또는 길이가 0인 경우 \(p_i=0\)을 사용하는 결정적 fallback을
사용한다.

### 4.3 Global assembly

고정된 \(p_i\)에 대해

$$
K\mathrel{+}=w_bA_iL_i^\mathsf TL_i,
\qquad
b\mathrel{+}=w_bA_iL_i^\mathsf Tp_i.
$$

이를 사용하면 현재 opposite-vertex distance spring을 제거하면서도 global
matrix는 상수이고 공간축별 factorization 재사용이 가능하다.

## 5. Contact formulation

### 5.1 개념 구분

`unilateral`은 constraint가 밀어내기만 하고 당기지 않는다는 뜻이다.
상대 물체가 반작용을 받지 않는 `one-way coupling`과 같은 의미가 아니다.
부등식 constraint에 양쪽 물체의 자유도를 모두 넣으면 unilateral이면서
two-way인 contact를 만들 수 있다.

Bouaziz et al.의 collision 예시는 한 변형체 정점과 고정된 half-space를
사용한다. 논문은 deformable–deformable, self-contact, 동적 rigid body의
결합 global system을 직접 정의하지 않는다. 아래 coupled constraint는 그
구조를 ysim의 목표에 맞게 확장한 것이다.

### 5.2 고정 또는 kinematic 장애물

고정 평면의 법선과 표면점을 \(n,b\), 접촉 두께를 \(d\)라 하면

$$
\mathcal H=\{p\mid n^\mathsf T(p-b)\ge d\}
$$

에 대한 정점 위치의 projection을 사용한다.

$$
E_c(q)=\frac{w_c}{2}
\min_{p\in\mathcal H}\|q_i-p\|^2.
$$

고정 장애물은 one-way이고, 움직이는 kinematic 장애물도 pose를 한 substep
동안 고정하면 같은 형태로 처리할 수 있다. 이 경우 장애물에는 반작용이 없다.

### 5.3 Coupled vertex–triangle contact

query vertex를 \(v\), target triangle을 \((1,2,3)\), 접촉점의 barycentric
좌표를 \(\beta_1,\beta_2,\beta_3\)라 한다.

$$
r_c(q)=q_v-\sum_{a=1}^{3}\beta_aq_a=A_cq,
\qquad
\sum_a\beta_a=1.
$$

상대 위치가 만족해야 할 half-space는

$$
\mathcal H_c=\{r\in\mathbb R^3\mid n^\mathsf Tr\ge d\}
$$

이고 contact potential은

$$
E_c^{\mathrm{contact}}(q)=
\frac{w_c}{2}
\min_{p_c\in\mathcal H_c}
\|A_cq-p_c\|^2
$$

이다.

Local step:

$$
p_c=\Pi_{\mathcal H_c}(A_cq).
$$

Global step:

$$
K\mathrel{+}=w_cA_c^\mathsf TA_c,
\qquad
b\mathrel{+}=w_cA_c^\mathsf Tp_c.
$$

중요한 차이는 \(A_c^\mathsf TA_c\)가 query vertex와 target triangle의 세
정점 사이 off-diagonal coupling을 만든다는 점이다. 현재 구현처럼 네 개의
독립적인 identity target과 대각 weight로 분해해서는 안 된다.

### 5.4 Self-contact와 여러 cloth

self-contact도 같은 \(A_c\)를 같은 cloth의 네 정점에 조립한다. 인접 triangle,
동일 one-ring, 잘못된 자기 참조는 collision 단계에서 제외한다.

서로 다른 cloth 사이 접촉을 정확히 풀려면 해당 substep에 활성인 모든 cloth
정점을 하나의 global unknown에 포함하거나, 그와 수학적으로 동등한 coupled
block solve가 필요하다. mesh별 독립 factorization과 상대 mesh의 이전 iterate를
읽는 Jacobi 교환은 근사 경로로만 분류한다.

### 5.5 Edge–edge contact

cloth self-collision과 grazing contact를 완성하려면 vertex–triangle만으로
충분하지 않다. 두 edge상의 보간 좌표를 \(s,t\)라 하면

$$
r_c(q)=
(1-s)q_1+s q_2-(1-t)q_3-t q_4=A_cq
$$

로 같은 half-space projection과 \(A_c^\mathsf TA_c\) 조립을 사용한다.
검출 단계에는 swept edge–edge 또는 CCD가 별도로 필요하다.

### 5.6 Contact epoch와 factorization

\(A_c\)는 barycentric/edge interpolation 계수에 의존한다. 첫 구현은 다음
active-set 정책을 사용한다.

1. substep 시작 또는 contact outer iteration에서 contact pair를 검출한다.
2. 한 contact epoch 동안 triangle id, \(\beta\) 또는 edge의 \(s,t\)를 고정한다.
3. 법선과 half-space projection target은 local step에서 다시 평가할 수 있다.
4. contact set 또는 보간 계수가 바뀌면 numeric factorization을 갱신한다.

보간 계수를 매 local step마다 변경하면 matrix도 매번 바뀌어 PD의 factorization
재사용 이점을 잃는다. 반대로 너무 오래 고정하면 sliding contact의 접촉점이
부정확해진다. 우선 substep 단위로 고정하고 침투량 및 접선 drift를 측정한 뒤
contact epoch 길이를 조정한다.

## 6. Contact stiffness와 GUI

### 6.1 파라미터 의미

접촉은 유한 penalty weight를 갖는 soft constraint이므로, 접촉이 검출됐어도
elasticity와 관성 항에 의해 잔류 침투가 생길 수 있다. 런타임 파라미터
`contact stiffness scale` \(s_c\)를 두고

$$
w_c=s_c\frac{m_{\mathrm{eff}}}{h^2}
$$

로 정의한다.

coupled vertex–triangle에서 inverse mass를 \(w_i^{m}=1/m_i\)라 하면

$$
m_{\mathrm{eff}}=
\frac{1}{
w_v^{m}
+\beta_1^2w_1^{m}
+\beta_2^2w_2^{m}
+\beta_3^2w_3^{m}}
$$

를 기준값으로 사용한다. 고정된 target vertex는 inverse mass가 0이다. 이
정의는 질량 분포가 다른 두 물체에서도 하나의 scale 값이 비슷한 의미를 갖게
한다. 모든 참여 정점의 inverse mass가 0이면 constraint를 만들지 않는다.

현재 시스템에는 이미 다음 런타임 경로가 있다.

- solver 값:
  [`PdSystem::kContactWeightScale`](../../include/sim/pd_system.hpp)
- GUI:
  [`PD 접촉 가중치`](../../src/main.cpp)
- 현재 기본값과 범위: `1.0`, 로그 슬라이더 `0.1–8.0`
- 노출 조건: `simulator.usePd == true`

coupled contact로 변경할 때 GUI 경로는 유지하되 내부 weight 기준을 위의
\(m_{\mathrm{eff}}/h^2\)로 바꾼다. 이름은 `PD 접촉 강성`으로 표시하고 tooltip에
`× m_eff/h²`와 높은 값의 locking/조건수 위험을 설명한다.

### 6.2 stiffness로 해결되는 것과 안 되는 것

접촉 강성 증가는 “접촉 row는 존재하지만 global minimum에 침투가 남는” 경우에
효과가 있다. 다음 문제는 강성을 올려도 해결되지 않는다.

- broad/narrow phase가 contact pair를 놓침
- 빠른 운동에서 CCD가 없어 완전히 통과함
- 잘못된 normal, barycentric 좌표 또는 contact side
- 독립 mesh solve 때문에 반작용이 늦거나 누락됨
- vertex–triangle만 있고 edge–edge 검출이 없음

유한 \(w_c\)는 정확한 non-penetration을 보장하지 않는다. 매우 높은 값이
필요하면 slider 범위를 계속 넓히기보다 coupled formulation, active set,
substep/CCD를 먼저 점검한다. 엄격한 비침투가 요구되면 향후
augmented-Lagrangian/KKT 또는 barrier 방식이 별도 연구 범위가 된다.

### 6.3 GUI 진단값

PD 선택 시 좌측 시뮬레이션 환경에 다음을 함께 표시한다.

- `PD 접촉 강성`: 런타임 가변 로그 슬라이더
- 활성 one-sided / vertex–triangle / edge–edge contact 수
- 이번 frame의 최대 침투 깊이와 평균 침투 깊이(mm)
- contact set 변화로 발생한 numeric refactor 횟수
- contact factorization 실패 횟수

강성 변경은 solver 재초기화 없이 적용되어야 한다. contact matrix가 이미
factorization되어 있다면 다음 안전한 epoch 경계에서 numeric refactor한다.
강성을 드래그하는 동안 매 substep 불필요하게 재분해하지 않도록 GUI edit 완료
시점 또는 dirty flag를 사용할 수 있다.

## 7. Dynamic rigid body coupling

고정/kinematic collider와 달리 동적 rigid body는 6-DoF generalized coordinate와
회전 Jacobian을 갖는다. 정확한 monolithic coupling을 하려면 cloth 위치와 rigid
translation/rotation을 하나의 system에 넣고 매 contact epoch에서 rigid contact
Jacobian을 선형화해야 한다. 이는 현재의 scalar \(n\times n\) cloth factorization
범위를 벗어난다.

구현 단계는 다음처럼 분리한다.

1. continuum cloth + 고정/kinematic one-way contact
2. 한 global cloth system의 self/cloth–cloth coupled contact
3. partitioned cloth–rigid reaction을 명시적인 근사 모드로 유지·검증
4. 필요할 때 rigid 6-DoF를 포함한 monolithic solve를 별도 설계

현재 `rigidDelta` 방식은 3단계의 partitioned approximation으로 문서화하고,
정확한 coupled PD라고 부르지 않는다.

## 8. 한 substep의 목표 알고리즘

```text
입력:
  xⁿ, vⁿ, mass, pin
  triangle rest operators G_t
  cotangent rows L_i와 rest curvature
  narrow-phase contact candidates
  h, material params, contact stiffness scale

1. s = xⁿ + h vⁿ + h² M⁻¹ f_ext

2. contact active set 생성
   - static/kinematic: one-vertex half-space
   - cloth/self: coupled vertex–triangle
   - edge–edge가 구현된 경우 coupled four-vertex row
   - contact epoch 동안 interpolation 계수 고정

3. global matrix 조립 또는 cache 갱신
   K = M/h²
     + Σ_t w_s A_t G_tᵀG_t
     + Σ_i w_b A_i L_iᵀL_i
     + Σ_c w_c A_cᵀA_c
     + K_pin
   factorize(K)

4. q⁰ = s

5. local/global 반복
   local strain:
     F_t = G_t q
     SVD 후 singular values projection → P_t

   local bending:
     v_i = L_i q
     |v_i⁰| 구면으로 projection → p_i

   local contact:
     r_c = A_c q
     unilateral half-space로 projection → p_c

   global RHS:
     b = M s/h²
       + Σ_t w_s A_t G_tᵀP_t
       + Σ_i w_b A_i L_iᵀp_i
       + Σ_c w_c A_cᵀp_c
       + b_pin

   solve Kq = b

6. vⁿ⁺¹ = (q - xⁿ)/h
   접촉 restitution/friction 정책 적용

7. pin을 보존하고 q, v commit
```

## 9. 검증 요구사항

### 9.1 Strain

- 강체 이동 및 회전에서 strain energy가 허용 오차 내 0이다.
- 알려진 \(F=\operatorname{diag}(s_1,s_2)\) 입력에서 singular value clamp가
  정확한 예상값을 낸다.
- global matrix가 대칭 positive definite이고 factorization이 성공한다.
- 같은 직사각형을 다른 삼각분할/해상도로 만들었을 때 총 에너지와 변형이
  spring 모델보다 mesh resolution에 덜 의존한다.
- solver iteration 수를 늘려도 material stiffness 정의가 바뀌지 않는다.

### 9.2 Bending

- 강체 이동과 회전에서 bending energy가 변하지 않는다.
- 평평한 rest cloth는 평평한 상태에서 잔류 bending force가 없다.
- 알려진 곡률의 strip/cylinder test에서 rest curvature가 복원된다.
- 경계 정점, obtuse triangle, 퇴화 one-ring에서 NaN/Inf가 발생하지 않는다.
- 기존 opposite-vertex spring을 비활성화했을 때도 동일 테스트가 통과한다.

### 9.3 Contact

- 고정 평면에서 접촉이 검출된 경우 contact stiffness 증가에 따라 최대 잔류
  침투가 단조 감소한다.
- contact stiffness를 런타임에 바꿔도 재초기화 없이 다음 epoch에 반영된다.
- contact가 검출되지 않은 tunneling test에서는 stiffness가 해결책이 아님을
  별도 카운터로 구분한다.
- dynamic cloth 두 장의 vertex–triangle contact에서 양쪽 mesh가 같은 solve
  안에서 반응한다.
- self-contact가 query 및 target 정점 모두에 off-diagonal coupling을 만든다.
- 인접 triangle과 자기 자신을 contact로 만들지 않는다.
- contact가 분리된 뒤 identity projection이 인력을 만들지 않는다.
- edge–edge 구현 후 grazing contact 회귀 테스트가 통과한다.
- 높은 stiffness에서도 factorization 실패, locking, 속도 폭발을 계측한다.

## 10. 구현 순서와 완료 조건

1. **Triangle rest cache와 strain element**
   - 완료: spring stretch를 끈 `TriangularCloth`가 strain test를 통과한다.
2. **Cotangent bending element**
   - 완료: opposite-vertex spring 없이 bending test를 통과한다.
3. **접촉 계측과 GUI 정리**
   - 완료: 강성, contact 종류별 수, 최대 침투가 PD 선택 시 표시된다.
4. **단일 global cloth system의 coupled vertex–triangle contact**
   - 완료: cloth–cloth 및 self-contact가 \(A_c^\mathsf TA_c\)로 조립된다.
5. **Edge–edge detection/constraint**
   - 완료: grazing contact 회귀가 통과한다.
6. **Dynamic rigid coupling 재설계**
   - 완료 기준은 partitioned approximation과 monolithic coupling 중 선택한
     계약을 별도 decision record에 남긴 뒤 정한다.

## 11. 참고 문헌

- Mathieu Bouaziz, Sofien Martin, Tiantian Liu, Ladislav Kavan, Mark Pauly,
  [*Projective Dynamics: Fusing Constraint Projections for Fast Simulation*,
  2014](https://infoscience.epfl.ch/server/api/core/bitstreams/a884fb0a-5fb5-49b4-a0ea-b88bd19a7e47/content).
- Tiantian Liu, Adam W. Bargteil, James F. O'Brien, Ladislav Kavan,
  [*Fast Simulation of Mass-Spring Systems*,
  2013](https://graphics.berkeley.edu/papers/Liu-FSM-2013-11/Liu-FSM-2013-11.pdf).
