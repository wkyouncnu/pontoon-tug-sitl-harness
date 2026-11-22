# 강화학습 규약 — 관측 · 행동 · 보상 · 추력배분

- 대상: 시나리오 3 (무인터그 2 대 좌현 결합, 폰툰 정밀 정렬)
- 정책 파일: `out/ppo_sc3_final.onnx` (PPO, stable-baselines3 → ONNX, 19 → 128 → 128 → 3, tanh)
- 본 문서는 **학습 환경(L1)과 배포 제어기가 같은 규약을 쓴다**는 것을 식으로 고정한 것임
- 인용 위치
  - 학습 환경: `../pontoon_rl/env/pontoon_env.py` (`_obs` 138–146 · `step` 148–175)
  - 배포 제어기: `../nuc/controller.py` (`build_obs` 125–145)
  - 추력배분: `../pontoon_rl/env/latch_qp.py` (`LatchQP`)
  - 파라미터: `../pontoon_rl/params/sc3.yaml`

## 1. 제어 구조

| 층 | 주기 | 입력 | 출력 | 구현 |
|---|---|---|---|---|
| 정책 (RL) | 10 Hz | 관측 19 차 | 행동 3 차 | ONNX. NUC 에서 실행 |
| 추력배분 (QP) | 50 Hz | 폰툰 목표력 3 차 | 터그 6 성분 | Latch-aware QP (quadprog) |
| 터그 하위 역행렬 | 터그 Pixhawk | 터그별 3 차 | 추진기 3 개 | 고정 역행렬. RL 이 건드리지 않음 |

- 정책은 **폰툰 전체에 가할 일반화력 3 성분만** 결정함. 터그 2 대에 어떻게 나눌지는 QP 가 정함
- 이 분리가 전자석 이탈을 막는 **유일한 하드 보장**임 (§4)

### 1.1 기호

| 기호 | 뜻 | 차원 |
|---|---|---|
| p = (x, y), psi | 폰툰 위치(NED) · 선수각 | m, rad |
| nu = (u, v, r) | 폰툰 바디속도 | m/s, rad/s |
| p_ref, psi_ref | 목표 위치 · 목표 선수각 | m, rad |
| a_t | 시각 t 의 정책 행동 | 3 |
| tau_d, tau_a | 요구 일반화력 · QP 가 실제로 낸 일반화력 | N, N·m |
| rho = tau_a − tau_d | QP 잔차 | N, N·m |
| m = (m_1, m_2) | 터그별 래치 여유 최솟값 | — |
| r_1, r_2 | 터그 부착위치 (폰툰 바디좌표) | m |

## 2. 관측 19 차 — 생성 방법

- 학습 환경 `_obs()` 와 배포 `build_obs()` 는 **같은 식**이며 동일성 시험으로 고정함 (`tests/test_same_code.py`)

$$
\mathbf{o}_t=\Big[\;
\mathbf{e}_b^{\mathsf{T}},\;
\cos e_\psi,\;
\sin e_\psi,\;
\boldsymbol{\nu}^{\mathsf{T}},\;
\mathbf{a}_{t-1}^{\mathsf{T}},\;
\big(\boldsymbol{\rho}\oslash\boldsymbol{\tau}_{\max}\big)^{\mathsf{T}},\;
\mathbf{m}^{\mathsf{T}},\;
\mathbf{r}_1^{\mathsf{T}},\;
\mathbf{r}_2^{\mathsf{T}}
\;\Big]^{\mathsf{T}}\in\mathbb{R}^{19}
$$

여기서 $\oslash$ 는 성분별 나눗셈이며, 바디좌표 위치오차와 방위오차의 정의는 다음과 같다.

$$
\mathbf{e}_b=\mathbf{R}(\psi)^{\mathsf{T}}\big(\mathbf{p}_{\mathrm{ref}}-\mathbf{p}\big),
\qquad
e_\psi=\mathrm{wrap}\big(\psi_{\mathrm{ref}}-\psi\big)
$$

$$
\mathbf{R}(\psi)=
\begin{bmatrix}
\cos\psi & -\sin\psi\\[2pt]
\sin\psi & \cos\psi
\end{bmatrix},
\qquad
\mathrm{wrap}(\theta)=\big(\theta+\pi\big)\ \mathrm{mod}\ 2\pi-\pi
$$

| 색인 | 기호 | 차원 | 뜻 | 실물에서의 출처 |
|---|---|---|---|---|
| 0–1 | $\mathbf{e}_b$ | 2 | 월드 위치오차를 폰툰 바디좌표로 회전 | 52501 의 `x, y, x_ref, y_ref` |
| 2–3 | $\cos e_\psi,\ \sin e_\psi$ | 2 | 방위오차. 각도 불연속 제거용 | 52501 의 `psi, psi_ref` |
| 4–6 | $\boldsymbol{\nu}$ | 3 | 바디속도 $(u,v,r)$ | 52501 의 `u, v, r` |
| 7–9 | $\mathbf{a}_{t-1}$ | 3 | 직전 스텝의 행동 | **제어기 내부 상태** |
| 10–12 | $\boldsymbol{\rho}\oslash\boldsymbol{\tau}_{\max}$ | 3 | QP 가 못 낸 몫 | **QP 출력** |
| 13–14 | $\mathbf{m}$ | 2 | 터그별 래치 여유 최솟값 (1 = 여유, 0 = 한계) | **QP 출력** |
| 15–16 | $\mathbf{r}_1$ | 2 | 터그 1 부착위치 | 설정 상수 `attach.r1 = [0.4894, −1.08]` |
| 17–18 | $\mathbf{r}_2$ | 2 | 터그 2 부착위치 | 설정 상수 `attach.r2 = [−0.6106, −1.08]` |

### 2.1 설계 근거

| 항목 | 이유 |
|---|---|
| 바디좌표 오차 $\mathbf{e}_b$ | 월드좌표를 그대로 넣으면 정책이 방위각마다 다른 함수를 배워야 함 |
| $\cos e_\psi,\ \sin e_\psi$ | $\psi$ 를 그대로 넣으면 $\pm\pi$ 에서 불연속임. 2 성분으로 펴서 넣음 |
| $\mathbf{a}_{t-1}$ 포함 | 행동 변화율에 벌점을 주기 위함(§3.2). **정책이 상태를 가짐** → 에피소드 경계에서 0 으로 초기화해야 함 |
| $\boldsymbol{\rho}\oslash\boldsymbol{\tau}_{\max}$ | QP 가 한계에 걸려 요구를 못 냈다는 사실을 정책에 되먹임함. 정규화해 스케일을 맞춤 |
| $\mathbf{m}$ | 전자석 한계까지 남은 여유. 정책이 **스스로 여유를 남기도록** 유도함 |
| $\mathbf{r}_1,\ \mathbf{r}_2$ | 부착위치가 **본 과제의 핵심 불확실성**임. 학습 시 $\pm 0.15$ m 로 흔들고(`dr.r_jit`) 관측에 실어 적응시킴 |
| `VecNormalize` 미사용 | 정규화 상수 파일이 없음 → 배포 시 ONNX 1 개만 옮기면 됨 |

### 2.2 실물 경로에서의 조립 순서 (10 Hz)

| 순서 | 동작 | 코드 |
|---|---|---|
| 1 | 마더십 → NUC `MOTHERSHIP_STATE`(52501) 수신 | `vendor/tug_bridge/mothership_rx.py` (+ 패치) |
| 2 | $(x,y,\psi)$ · $(u,v,r)$ · 목표 $(x_{\mathrm{ref}},y_{\mathrm{ref}},\psi_{\mathrm{ref}})$ 추출 | `sitl/nuc_nodes.py::RlPolicyNode` |
| 3 | `build_obs` → 19 차 | `nuc/controller.py:125` |
| 4 | ONNX 추론 → $\mathbf{a}$ → $\boldsymbol{\tau}_d$ | `nuc/onnx_policy.py` |
| 5 | 50 Hz QP 가 $\boldsymbol{\tau}_d$ 를 받아 6 성분 생성 → `TUG_COMMAND`(52503) 송신 | `nuc/controller.py:170` |

- `rc_on_off = 1` 또는 `on_off = 0` 이면 정책을 돌리지 않고 $\boldsymbol{\tau}=\mathbf{0}$ 을 보내며 $\mathbf{a}_{t-1}$ 을 0 으로 되돌림 (수동개입 · 자율해제 경로)

### 2.3 19 개 중 밖에서 주어야 하는 값은 9 개뿐임

| 구분 | 개수 | 성분 | 누가 만드는가 |
|---|---|---|---|
| **외부 입력** | **9** | $x, y, \psi, u, v, r$ 와 $x_{\mathrm{ref}}, y_{\mathrm{ref}}, \psi_{\mathrm{ref}}$ | **마더십이 52501 로 보냄.** 이것만 공급하면 됨 |
| 설정 상수 | 4 | $\mathbf{r}_1,\ \mathbf{r}_2$ | `sc3.yaml` 의 `attach.r1` · `attach.r2`. 매 스텝 같은 값 |
| 제어기 내부 | 6 | $\mathbf{a}_{t-1}$, $\boldsymbol{\rho}$ | 직전 `policy_step` · `allocate` 가 남긴 값 |
| QP 출력 | 2 | $\mathbf{m}$ | 직전 `allocate` 가 남긴 값 |

- 19 차는 **사람이 채우는 벡터가 아님.** 위 9 개 수를 넣으면 나머지 10 개는 제어기가 자기 상태에서 채움
- $\mathbf{e}_b$ 와 $\cos e_\psi,\ \sin e_\psi$ 는 입력이 아니라 **계산 결과임** — 6 개 값에서 `build_obs` 가 4 개를 만듦 (바디좌표 회전 + 각도 wrap)
- `reset()` 직후 값은 $\mathbf{a}_{t-1}=\mathbf{0}$, $\boldsymbol{\rho}=\mathbf{0}$, $\mathbf{m}=\mathbf{1}$ 임. **에피소드·자율모드 진입 때 반드시 `reset()` 을 호출해야 함** (정책이 상태를 갖기 때문)
- 실행 가능한 최소 예제: [`obs_howto.py`](obs_howto.py) (`..\plan_pontoon_rl\.venv\Scripts\python.exe -X utf8 docs\obs_howto.py`)

```python
ctl = NucController()                     # sc3.yaml + ONNX 적재
ctl.set_goal([x_ref, y_ref, psi_ref])     # 52501 의 목표자세
r1, r2 = ctl.c["attach"]["r1"], ctl.c["attach"]["r2"]

tau_d, a = ctl.policy_step([u, v, r], [x, y, psi], r1, r2)   # 10 Hz
u6, tau_a = ctl.allocate(tau_d)                              # 50 Hz → 52503
```

- 위 2 줄이 실물 NUC 에서 도는 전부임. `build_obs` 는 `policy_step` 안에서 호출됨
- 실측 예 (`obs_howto.py` 출력): 같은 상태를 두 번 넣어도 $\mathbf{a}_{t-1}$, $\boldsymbol{\rho}$, $\mathbf{m}$ 8 성분이 바뀜 → **같은 센서값이 같은 관측을 주지 않음.** 호출 순서를 지켜야 함

## 3. 행동과 보상함수

### 3.1 행동

$$
\mathbf{a}_t\in[-1,1]^3,
\qquad
\boldsymbol{\tau}_d=\mathbf{a}_t\odot\boldsymbol{\tau}_{\max},
\qquad
\boldsymbol{\tau}_{\max}=
\begin{bmatrix}60\ \mathrm{N}\\[2pt] 60\ \mathrm{N}\\[2pt] 60\ \mathrm{N\,m}\end{bmatrix}
$$

- $\odot$ 는 성분별 곱임. 의미는 폰툰 바디좌표의 전후력 · 좌우력 · 회두모멘트임
- 출력층이 $\tanh$ 이므로 포화가 구조적으로 보장됨. 별도 클리핑이 필요 없음

### 3.2 보상 (학습 전용)

$$
\begin{aligned}
r_t=\;&-0.40\,\lVert\mathbf{e}_p\rVert
      -0.80\,\lvert e_\psi\rvert
      -0.10\,\lVert\boldsymbol{\nu}\odot\mathbf{w}_\nu\rVert\\[4pt]
     &-0.05\,\lVert\mathbf{a}_t-\mathbf{a}_{t-1}\rVert
      -0.30\,\big\lVert\boldsymbol{\rho}\oslash\boldsymbol{\tau}_{\max}\big\rVert\\[4pt]
     &+1.5\,\mathbf{1}_{\mathrm{align}}
      -20.0\,\mathbf{1}_{\mathrm{detach}}
\end{aligned}
$$

$$
\mathbf{w}_\nu=\big(1,\ 1,\ 0.5\big),
\qquad
\mathbf{1}_{\mathrm{align}}=
\begin{cases}
1, & \lVert\mathbf{e}_p\rVert<0.30\ \mathrm{m}\ \ \text{and}\ \ \lvert e_\psi\rvert<5^\circ\\[2pt]
0, & \text{otherwise}
\end{cases},
\qquad
\mathbf{1}_{\mathrm{detach}}=
\begin{cases}
1, & \lVert\mathbf{e}_p\rVert>12\ \mathrm{m}\\[2pt]
0, & \text{otherwise}
\end{cases}
$$

| 항 | 계수 | 단위 | 목적 |
|---|---|---|---|
| 위치오차 $\lVert\mathbf{e}_p\rVert$ | **−0.40** | m | 목표 위치로 끌어감 |
| 방위오차 $\lvert e_\psi\rvert$ | **−0.80** | rad | 위치보다 2 배 무겁게 둠. ㄷ 형 도킹부는 자세가 틀리면 들어가지 않음 |
| 속도 $\lVert\boldsymbol{\nu}\odot\mathbf{w}_\nu\rVert$ | −0.10 | — | 과속 접근 억제. 회두는 절반 가중 |
| 행동 변화 $\lVert\mathbf{a}_t-\mathbf{a}_{t-1}\rVert$ | −0.05 | — | 추력 떨림 억제. 이 항 때문에 $\mathbf{a}_{t-1}$ 이 관측에 필요함 |
| QP 잔차 $\lVert\boldsymbol{\rho}\oslash\boldsymbol{\tau}_{\max}\rVert$ | **−0.30** | — | 요구를 QP 가 낼 수 있는 범위 안에 두도록 유도함. 전자석 한계를 정책이 간접 학습하는 통로임 |
| 정렬 완료 보너스 | **+1.5** | — | 허용범위 안의 **스텝마다** 지급 |
| 이탈 종료 벌점 | **−20.0** | — | 이탈·발산 시 1 회 |

- 보너스는 종료 보상이 아니라 **체류 보상**임 → 정책이 허용범위에 들어간 뒤 **머무르도록** 학습됨
- 위치오차는 m, 방위오차는 rad 로 들어가므로 계수비 $0.80/0.40$ 이 그대로 가중비가 아님. $1^\circ=0.0175$ rad 이므로 **같은 단위로 환산하면 방위 벌점이 위치보다 훨씬 작음** — 그래서 계수에서 2 배를 준 것임
- ⚠️ **보상은 학습에서만 계산됨.** 배포 코드(`nuc/`)와 본 하네스는 보상을 계산하지 않음. 영상·그래프가 보여 주는 위치오차 · 방위오차 · 래치 여유가 보상 항에 1:1 로 대응하는 관측 가능량임

### 3.3 에피소드 조건

| 항목 | 값 | 출처 |
|---|---|---|
| 정책 주기 | 0.10 s (10 Hz) | `env.dt_ctrl` |
| 물리 적분 | RK4, 5 서브스텝 → 50 Hz | `env.n_sub` |
| 길이 | 60 s = 600 정책스텝 = 3 000 물리틱 | `env.T_ep` |
| 초기 위치 | 균일 $\pm 3.0$ m | `env.init_pos` |
| 초기 방위 | 균일 $\pm 0.52$ rad ($\pm 30^\circ$) | `env.init_yaw` |
| 합격 허용 | 위치 0.30 m · 방위 $5^\circ$ | `env.pos_tol` · `yaw_tol` |

## 4. Latch-aware 추력배분 QP

정책이 낸 $\boldsymbol{\tau}_d$ 를 터그 2 대의 일반화력으로 나눈다. 결정변수는 $\mathbf{u}=[\boldsymbol{\tau}_1;\ \boldsymbol{\tau}_2]\in\mathbb{R}^6$ 이고, $\boldsymbol{\tau}_i=(X_i,\ Y_i,\ N_i)$ 는 터그 $i$ 가 부착점에서 내는 힘·모멘트이다.

$$
\min_{\mathbf{u}\in\mathbb{R}^{6}}\;
\big(\mathbf{A}\mathbf{u}-\boldsymbol{\tau}_d\big)^{\mathsf{T}}
\mathbf{Q}
\big(\mathbf{A}\mathbf{u}-\boldsymbol{\tau}_d\big)
+w\,\lVert\mathbf{u}\rVert^{2}
$$

$$
\text{s.t.}\quad
\begin{aligned}
f_{\min}\,\mathbf{1}\;&\le\;\mathbf{B}_F^{-1}\boldsymbol{\tau}_i\;\le\;f_{\max}\,\mathbf{1}
&&\text{(추진기 포화, 3 행)}\\[4pt]
-Y_i\;&\le\;F_t
&&\text{(전자석 인장, 1 행)}\\[4pt]
\lvert X_i\rvert\;&\le\;F_s
&&\text{(전자석 전단, 2 행)}\\[4pt]
\lvert N_i\rvert\;&\le\;M_{\max}
&&\text{(전자석 박리, 2 행)}
\end{aligned}
\qquad i=1,\ 2
$$

부착점의 힘을 폰툰 원점의 일반화력으로 옮기는 사상과 터그 추진기 배치행렬은 다음과 같다.

$$
\mathbf{A}=\big[\;\mathbf{H}(\mathbf{r}_1)\quad\mathbf{H}(\mathbf{r}_2)\;\big]\in\mathbb{R}^{3\times 6},
\qquad
\mathbf{H}(\mathbf{r})=
\begin{bmatrix}
1 & 0 & 0\\[2pt]
0 & 1 & 0\\[2pt]
-r_y & r_x & 1
\end{bmatrix}
$$

$$
\mathbf{B}_F=
\begin{bmatrix}
1 & 1 & 0\\[2pt]
0 & 0 & 1\\[2pt]
d & -d & l_b
\end{bmatrix},
\qquad
d=0.30\ \mathrm{m},\quad
l_b=0.50\ \mathrm{m},\quad
\det\mathbf{B}_F=2d=0.6
$$

래치 여유는 전자석 5 행의 정규화 여유 중 터그별 최솟값이다.

$$
m_i=\min_{k\in\mathcal{K}}\;
\frac{h_k-\mathbf{H}^{\mathrm{lat}}_{k}\,\boldsymbol{\tau}_i}{h_k},
\qquad
\mathcal{K}=\{\mathrm{tension},\ \mathrm{shear}^{+},\ \mathrm{shear}^{-},\ \mathrm{peel}^{+},\ \mathrm{peel}^{-}\}
$$

| 기호 | 값 | 근거 |
|---|---|---|
| $\mathbf{Q}$ | $\mathbf{I}_3$ | `LatchQP(Q=(1,1,1))` |
| $w$ | $10^{-3}$ | `w_reg` |
| $f_{\max},\ f_{\min}$ | $+35.85$ N, $-28.33$ N | T200 정·역추력 한계 |
| $F_t,\ F_s$ | 40 N, 40 N | **추정치** |
| $M_{\max}$ | 10 N·m | **추정치** |

- 터그당 제약 11 행(추진기 상·하한 6 + 전자석 5) → 총 22 행
- 설정값 $F_t=F_s=40$ N, $M_{\max}=10$ N·m 은 **전부 추정치**임. 유일한 실측 근거는 **PWM 1700 µs 에서 이탈($\approx 18$ N)** 이며 작용 방향(인장/전단)이 불명임 — 확인 필요
- 한계를 낮춰 가며 재평가한 결과는 `../out/n1/SWEEP_LATCH.md` 에 있음. **임계는 전단 $F_s=8$ N 이고 박리는 스윕 범위 내에서 깨지지 않음**
- 정정: 이전 판에서 전자석 제약을 "6 행" 으로 적었으나 실제는 **5 행**임 (`latch_qp.py:33–37` 인장 1 · 전단 2 · 박리 2)

## 5. 학습 환경과 배포의 의도적 차이

| 항목 | 학습 환경 (L1) | 배포 · 하네스 |
|---|---|---|
| 플랜트 | 3-DOF 파이썬 모델 | 동일 모델 (`plant/`) — 하네스는 플랜트를 바꾸지 않음 |
| 부착위치 | 플랜트는 흔든 값, QP 는 **공칭값** 사용 | 동일. 이 불일치를 정책이 흡수하는 것이 RL 의 존재 이유임 |
| 전송 | 함수 호출 | **MAVLink v2 over UDP** (52501 / 52502 / 52503) |
| 도메인 무작위화 | 학습 시 on | 합격판정은 `--dr off`. ⚠️ `--dr on` 평가는 하지 않음 |
| 보상 | 계산함 | 계산하지 않음 |

- 합격판정은 **재학습 없이(zero-shot)** 수행함 → 전송계층을 끼워도 성능이 유지되는지를 본 것임

## 6. 실물 시험에 필요한 것

### 6.1 공급해야 하는 신호 — 9 개

| 신호 | 단위 · 좌표 | 출처 후보 | 상태 |
|---|---|---|---|
| $x,\ y$ | m, NED | 마더십 항법 (RTK-GNSS 또는 실내 모션캡처) | 공급 필요 |
| $\psi$ | rad, 북기준 시계방향 + | 마더십 AHRS | ⚠️ Simulink `psi_time_log` 단위가 rad/deg 미확인 |
| $u,\ v,\ r$ | m/s, m/s, rad/s (바디) | 마더십 항법 | 공급 필요 |
| $x_{\mathrm{ref}},\ y_{\mathrm{ref}},\ \psi_{\mathrm{ref}}$ | m, rad | 기존 LOS 와 **같은 출처** | ⚠️ 절대/상대 좌표 미확인 |

- 송신주기 **50 Hz** 가정. 제어기는 10 Hz 로 소비하므로 더 낮아도 동작하나 지연이 늘어남
- 폰툰 자세를 직접 재는 센서가 없으면 **마더십 자세에서 변환해야 하며, 그 변환오차가 그대로 정렬오차가 됨**

### 6.2 측정해야 하는 물리량

| 항목 | 현재 값 | 왜 필요한가 | 우선순위 |
|---|---|---|---|
| 전자석 전단 한계 $F_s$ | 40 N (추정) | 스윕상 **임계 제약**. 8 N 미만이면 시나리오 3 이 깨짐 | **1** |
| 전자석 인장 한계 $F_t$ | 40 N (추정) | 6 N 까지는 20/20 유지 | 2 |
| 부착위치 $\mathbf{r}_1,\ \mathbf{r}_2$ | $[0.4894,\ -1.08]$, $[-0.6106,\ -1.08]$ (추정) | QP 의 $\mathbf{A}$ 가 이 값으로 만들어짐. 학습은 $\pm 0.15$ m 까지 흡수함 | 2 |
| 폰툰 관성 $I_z$ | 147 kg·m² (균일판 가정) | 플랜트 모델의 회두 응답 | 3 ⚠️ 457–914 kg·m² 와 불일치 |
| 박리 한계 $M_{\max}$ | 10 N·m (추정) | 스윕 범위 내에서 깨지지 않음 | 4 (측정가치 낮음) |

- 전단이 1 순위인 이유: 터그 2 대가 **같은 좌현 변에 1.1 m 간격**으로 붙어 회두모멘트를 **두 터그 전단력의 차**로 만들기 때문에, 모멘트 수요가 전단 수요로 전가됨

### 6.3 소프트웨어 준비

| 항목 | 내용 |
|---|---|
| 다이얼렉트 | `mavlink/usv_tug.xml` 에 `MOTHERSHIP_STATE`(52501) 추가 → 패치 `patch/0001-add-mothership-state-52501.patch` (가산 전용 +118 / −0) |
| 수신 노드 | `vendor/tug_bridge/mothership_rx.py` 에 52501 분기와 `/mothership/state` 퍼블리셔 추가 (가산) |
| 메시지 패키지 | `tug_interfaces/msg/MothershipState.msg` 신규 + `CMakeLists.txt` 등록 **필수** |
| NUC 실행환경 | Python 3.8+, `onnxruntime`, `quadprog`, `numpy`, `pyyaml`. 정책은 `out/ppo_sc3_final.onnx` 1 개 |
| 터그측 | **변경 없음.** 터그는 `TUG_COMMAND`(52503) 만 받으며 `crc_extra` 불변 |

### 6.4 시험 절차와 중단 조건

| 단계 | 내용 | 중단 조건 |
|---|---|---|
| 1. 통신 확인 | 52501 수신율 · 52503 송신율 계상 | 수신율 90 % 미만 |
| 2. 개루프 추력 확인 | 자율 off 상태에서 $\boldsymbol{\tau}_d$ 만 기록 | 지령이 $\boldsymbol{\tau}_{\max}$ 에 상시 포화 |
| 3. 저출력 폐루프 | $\boldsymbol{\tau}_{\max}$ 를 1/3 로 낮춰 1 회 | 전자석 이탈 징후, 위치오차 발산 |
| 4. 정격 폐루프 | 공칭 설정으로 반복 | 과도구간 전단력이 실측 한계 초과 |

- **자율 진입 때마다 `reset()`** — 정책이 $\mathbf{a}_{t-1}$ 을 들고 있으므로 이전 에피소드의 값이 남으면 첫 스텝이 튐
- 안전: 입력 단절 · NaN/inf · 종료 시 `tug_tx` 가 $\boldsymbol{\tau}=\mathbf{0}$ 을 10 회 보냄. ⚠️ **Pixhawk 에 수신 타임아웃이 없어 마지막 값을 유지하므로, Wi-Fi 가 끊기면 $\boldsymbol{\tau}=\mathbf{0}$ 조차 도착하지 못함**
- 영상의 과도구간(0–3 s) 전단력이 약 26 N 으로 실측치 18 N 을 넘으므로, **1 회차는 초기오차를 1 m 이내로 잡아 과도구간을 줄이는 것이 안전함**

### 6.5 실물에서 달라질 수 있는 것

| 항목 | 학습 · 하네스 | 실물에서의 차이 |
|---|---|---|
| 플랜트 | 3-DOF 파이썬 모델 | 유체력 계수 오차, 파·바람 외란 |
| 부착 | 강체 결합 가정 | 전자석 미끄러짐 · 유격 |
| 지연 | 하네스에서 RTT 400 ms 까지 확인 | 실제 Wi-Fi 지연 분포 미확인 |
| 상태추정 | 참값 | 항법 잡음 · 좌표변환 오차 |

- 위 4 항은 **하네스가 검증하지 못한 영역**임. 실물 1 회차는 성능시험이 아니라 **신호·안전 확인**으로 보는 것이 타당함
