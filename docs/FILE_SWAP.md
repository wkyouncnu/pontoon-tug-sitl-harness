# 가상 → 실물 파일 교체표 (FILE_SWAP)

- 문서 목적: 하네스(`harness_sitl/`)의 어느 파일이 **실물 NUC 에 그대로 올라가고**, 어느 파일이 **실물에서는 사라지는지**를 파일 단위로 못박는다.
- 설계 원칙 (사용자 확정): **가상과 실물의 차이는 "전송계층"과 "플랜트" 두 가지뿐이다.** 제어 수식은 한 글자도 바뀌지 않는다.
- 작성: 2026-11-22 (S4). 검증 시험: `tests/test_same_code_both_paths.py`.

---

## 1. 그대로 올라가는 것 (production)

| 파일 | 역할 | 의존성 |
|---|---|---|
| `nuc/__init__.py` | 경계 선언 | — |
| `nuc/controller.py` | 관측 19차 조립 + 정책 호출 + Latch-aware QP 호출 | numpy, onnxruntime, quadprog |
| `nuc/onnx_policy.py` | ONNX 추론기 (obs 19 → act 3) | numpy, onnxruntime |
| `../pontoon_rl/env/latch_qp.py` | Latch-aware QP **본체** | numpy, quadprog |
| `../pontoon_rl/params/sc3.yaml` | 기하·한계·τ_max 설정 | — |
| `out/ppo_sc3_final.onnx` | 학습된 정책 가중치 | — |

- 공통 성질
  - `transport` · 토픽 · 소켓 · MAVLink · `rclpy` 를 **import 하지 않는다.**
  - `stable_baselines3` · `torch` 를 **import 하지 않는다.** 실물 NUC 의 의존성 집합을 작게 유지하는 것이 목적이다 (재학습 = ONNX 파일 교체).
  - 입출력이 **plain numpy** 다. 호출자가 전송을 담당한다.
- `latch_qp.py` 는 **수정 없이 `pontoon_rl` 째로 동반배포**한다 (사용자 지시: `pontoon_rl/` 은 수정하지 말고 import 만 한다). 복사본을 만들면 학습환경과 배포본이 갈라져 두 번 고치게 된다.

### 1.1 ⚠️ 실물 배치 시 반드시 할 두 가지

1. **`sys.path` 에 패키지 루트를 넣어라.**
   `nuc/controller.py` 의 부트스트랩은 `<ROOT>/pontoon_rl` **만** 추가한다. 하네스에서는 `nodes/*` 와 `tests/_common.py` 가 `harness_sitl` 를 넣어주기 때문에 동작하지만, 실물 노드에서는 그 보장이 없다. ROS 2 패키지의 `setup.py` 또는 `PYTHONPATH` 로 **`nuc/` 의 부모 디렉터리와 `pontoon_rl/` 두 곳**을 올려라.
2. **ONNX 경로를 명시해 넘겨라.**
   `nuc/onnx_policy.py::DEFAULT_ONNX` 는 `harness_sitl/out/ppo_sc3_final.onnx` 를 가리키는 **하네스 편의용 기본값**이다. 실물에서는 `NucController(onnx_path=<ROS 2 파라미터>)` 로 경로를 받아라. 기본값에 의존하면 배포 디렉터리 구조에 숨은 결합이 생긴다.

### 1.2 ⚠️ 확인 필요 — `pontoon_env` 의존

- `nuc/controller.py` 는 설정 로더 `_load` · 기본경로 `CFG_DEFAULT` · 각도 래핑 `wrap` 세 개를 **`pontoon_rl/env/pontoon_env.py` 에서** 가져온다. 그 모듈에는 **폰툰 운동모델(M2 하이브리드 UDE)도 같이 들어 있다** — 실물 NUC 에는 필요 없는 코드다.
- 현재는 동작에 문제가 없다(단순 import, 부작용 없음). 다만 배포 깔끔함을 위해서는 세 함수를 작은 설정 모듈로 분리하는 것이 맞다.
- **지금 고치지 않는다.** `pontoon_rl/` 수정 금지 지시가 우선이고, 분리하면 학습측 import 경로가 바뀐다. **배포 직전 과제로 기록한다.**

---

## 2. 실물에서 사라지는 것 (실물이 대신한다)

| 파일 | 실물에서 무엇이 대신하나 |
|---|---|
| `plant/tug_lower.py` | **터그 Pixhawk 내부의 하위 역행렬** (우리가 배포하지 않는다) |
| `plant/__init__.py` | — |
| `nodes/node_a_plant.py` | **실제 폰툰과 물** |
| `nodes/node_b_mothership.py` | **마더십 Pixhawk** (상태 추정·수동개입) |
| `nodes/node_c_nuc.py` | **ROS 2 노드** (`create_subscription` / `create_publisher`) → 안의 `NucController` 호출부는 그대로 |
| `nodes/node_d_esp32.py` | **실제 ESP32 ×3 (DroneBridge Wi-Fi/UDP) + 터그 Pixhawk** |
| `iface/transport.py` | `rclpy` (노드 내부) + UDP 소켓 (`tug_bridge/mav_link.py`) |
| `iface/mavlink_wire.py` | **`pymavlink` + 생성된 dialect** (`usv_tug.py`) |
| `iface/ros2_stub.py` | 실제 ROS 2 |
| `iface/messages.py` | 실제 `tug_interfaces/msg/*.msg` + MAVLink dialect |
| `run_harness.py` · `tests/` · `tools_sweep_comm.py` | — (하네스 전용) |
| `policy/export_onnx.py` | — (학습측 전용. `stable_baselines3`·`torch` import) |

- `plant/` 는 **삭제되는 것이 정상**이다. 실물 전환에서 이 패키지가 남아 있으면 무언가를 잘못 배포한 것이다.
- ⚠️ `plant/tug_lower.py` 의 하위 역행렬은 **RL 로 대체하지 않는다** (사용자 확정). 정방행렬이고 `det = 2d = 0.6 ≠ 0` 이라 닫힌형 역해가 수학적으로 정확하다 (왕복오차 실측 1.8e-15).

---

## 3. 전송계층 대응표

| 하네스 토픽 | 실물 전송 | 양자화 |
|---|---|---|
| `/plant/state` | **없음** (물리 계측 자체. 와이어가 아니다) | — |
| `/pontoon/state` | 마더십 Pixhawk uORB → (S5) MAVLink **52501** | **float32** |
| `/mothership_data` | ROS 2 `tug_interfaces/MothershipData` ← MAVLink **52502** | **float32** (`<Qfff`) |
| `/tug_command` | ROS 2 → `tug_tx.py` → MAVLink **52503** | 없음 (`<Qdddddd`, float64) |
| `/allocation_debug` | ROS 2 (로깅 전용) | — |
| `/tug/thrust` | **없음** (터그 Pixhawk 내부) | — |

- ★ **터그 쪽은 전혀 바뀌지 않는다.** 터그는 `52503` 만 받고, S5 가 추가하는 것은 상행 `52501` 뿐이다. 근거: `tug_tx.py` diff 0.

---

## 4. 어댑터가 지켜야 하는 계약

- `nodes/node_c_nuc.py` 가 하는 일은 **세 가지뿐**이다. 실물 ROS 2 노드도 똑같이 세 가지만 해야 한다.
  1. 구독한 메시지에서 숫자 꺼내기 — `st.nu()`, `st.eta()`, `st.r1()`, `st.r2()`
  2. 제어기 호출 — `ctl.policy_step(nu, eta, r1, r2)` → `ctl.allocate(tau_d)`
  3. 결과를 발행 — `MothershipData.from_tau`, `TugCommand.from_tau6`
- 어댑터가 **숫자를 만들면 안 된다.** 제어기 상태(`a_prev`, `res`, `marg`)는 property 로 읽기만 한다.
- 이 계약이 깨지지 않았음을 `tests/test_same_code_both_paths.py` 가 확인한다 — 같은 입력에 대해 **제어기 직접 호출**과 **노드 경유 호출**이 같은 `τ_i` 를 내는지 본다.
