# SITL 경로 상세 (MAVLink over UDP)

- 본 문서는 `sitl/` 경로의 설계 근거를 기술함
- 참조 위치: `sitl/nuc_nodes.py:34` (양자화) · `sitl/run_sitl.py:46` (`--dr` 기본값) · `sitl/wire.py:129` (`r1`·`r2`)
- 시스템 전체 설명은 [../README.md](../README.md)

## 1. 두 경로의 구분

| 경로 | 프로세스 | 전송 | 용도 |
|---|---|---|---|
| inproc | 1 | `iface/transport.py::InprocTransport` | 기준선. 빠른 회귀시험 |
| SITL | 2 (플랜트 · NUC) | MAVLink v2 over UDP (`127.0.0.1`) | 실물과 동일한 프레이밍·노드 구성 |

- 두 경로가 **같은 `nuc/` 코드**를 호출함. 차이는 전송계층과 플랜트뿐임
- SITL 경로에는 벤더 노드(`mothership_rx`, `tug_tx` ×2)가 실제로 기동함. inproc 경로에는 기동하지 않음

## 2. 양자화 차이 (`sitl/nuc_nodes.py:34`)

| 신호 | inproc | SITL | 실물 |
|---|---|---|---|
| 폰툰 상태 (`/pontoon/state`) | float32 (`PontoonState.to_wire()`) | float32 (MAVLink 52501) | float32 (MAVLink 52501) |
| 정책 출력 τ_d | **float32** (`MothershipData.from_tau`) | **float64** (ROS 토픽, `.msg` 가 float64) | float64 |
| 터그 명령 τ (52503) | float64 | float64 (`<Qdddddd`) | float64 |

- τ_d 는 **와이어로 나가지 않음**. NUC 내부 ROS 토픽이므로 실물에 양자화가 없음
- inproc 경로는 같은 자리에서 float32 로 자름. 실물에 **없는 오차를 더하는 보수적 설정**임
- 실측 영향: 직접 호출 대 inproc 전송경유 `u6` 차이 최대 **7.38e-06 N** (상한 1e-02), τ_d 차이 **1.43e-05 N** (`|τ_d|max 60.0 N`, 상대 2.4e-07)
- 기준선 수치를 무효화하지 않기 위해 inproc 쪽을 고치지 않음. SITL 경로가 실물의 수치를 보임
- 따라서 inproc 합격 판정(평균 0.00252 m)과 SITL 합격 판정(평균 0.00104 m)의 차이는 플랜트 차이가 아니라 **이 양자화 차이**임

## 3. 부착점 `r1`·`r2` 와 `--dr` 기본값 (`sitl/run_sitl.py:46`, `sitl/wire.py:129`)

- `MOTHERSHIP_STATE`(52501)에 부착점 필드가 **없음** (사용자 확정 필드집합)
- 실물 NUC 은 `config` 의 공칭값을 사용함. 측정 경로가 없음
- `NucController.__init__` 은 항상 설정 공칭값으로 `LatchQP` 를 구성함. `build_obs(nu, eta, r1, r2)` 만 인자로 받음. 즉 `r1`·`r2` 격차는 **관측에만** 영향함

| 설정 | 정책이 보는 `r1`·`r2` | 공정성 |
|---|---|---|
| `--dr off` (기본) | 공칭값 = 참값 | 두 경로를 공정하게 비교 가능 |
| `--dr on` | inproc 은 난수화된 참값, 실물 NUC 은 공칭값 | 같은 조건이 아님. **별도 측정으로 보고해야 함** |

- 공칭 실측값: `r1 = [0.4894, -1.08]`, `r2 = [-0.6106, -1.08]` (단위 m, 선체좌표)
- `--dr on` 의 SITL 측정은 **아직 수행하지 않음** (잔여 작업)

## 4. 프로세스 구성과 틱 순서

`PlantWorld.tick()` 의 순서가 실물과 같음.

| 순서 | 동작 | 대역 대상 |
|---|---|---|
| ① | `TugRx.drain()` ×2 — 52503 수신, 자기 몫 선택 | 터그 ESP32 + 터그 Simulink |
| ② | `allocate_pair` → `/tug/thrust` 발행 | 터그 Pixhawk 하위 역행렬 |
| ③ | `PlantNode.tick()` — RK4 적분 | 폰툰 + 물 |
| ④ | `tx.state(...)` 52501, `tx.data(...)` 52502 | 마더십 Simulink + 마더십 ESP32 |
| ⑤ | `tx.drain_hb()` — NUC HEARTBEAT 수거 | 마더십 ESP32 |
| ⑥ | 1 Hz HEARTBEAT 송신 | 마더십 ESP32 |

- `reset()` 이 `last_tau` 를 지우지 않음. 터그 Pixhawk 가 마지막 값을 유지하는 성질을 모사하는 것이 목적임
- 터그 2대가 포트를 공유하지 않고 각자 `:14651`·`:14652` 를 바인드함. 실제 구성과 같음

## 5. 플랫폼 대역 (`sitl/bootstrap.py`)

- 벤더 `relay_common.py:42` 가 import 시점에 `signal.SIGHUP` 을 참조함. Windows 에 해당 시그널이 없음
- `bootstrap.setup()` 이 `signal.SIGHUP = 1` 을 정의하고 `signal.signal` 을 감쌈. `shim/`·`vendor/` 를 `sys.path` 에 넣음. 멱등함
- 이것이 **전송계층·플랜트에 이은 세 번째 가상·실물 차이**임
- 검증 범위: τ=0 ×10 송신 로직은 와이어에서 확인됨. **SSH 단절(SIGHUP)이 그 로직을 실제로 깨우는지는 가상으로 확인되지 않음**

## 6. Windows 타이밍

- `time.sleep` 분해능이 약 15.6 ms 임
- winmm `timeBeginPeriod(1)` 호출 + shim `rclpy` 의 데드라인 기준 스케줄링으로 보정함
- 실측 `worst_lag_ms` 16–17, `overrun 0`

## 7. 배속 실행의 제약

- `--speed > 1` 은 플랜트 벽시계만 압축함. NUC 노드의 `rclpy` 타이머는 실벽시계이므로 제어기가 굶음
- 실측 `--speed 8 --steps 40` → `ctl_policy 5/40`, `dpos 0.927 m`
- `summary["pass"]` 가 `speed == 1.0` 을 요구함. `perf_valid` 플래그로 표시함
- 배속은 프레이밍·손실·카운터 확인에만 사용함

## 8. 주의 사항

| 항목 | 내용 |
|---|---|
| HEARTBEAT 포트 | `14660` 을 비워 두면 ICMP port-unreachable 이 NUC 수신소켓을 재바인드시켜 상행이 약 50 % 소실됨. `drain_hb()` 가 필수임 |
| 벤더 코드 | `tug_tx.py` · `mav_link.py` · `relay_common.py` 는 diff 0 을 유지함. 고치지 않음 |
| `pack_and_advance` | pymavlink 의 `msg.pack(mav)` 는 seq 를 올리지 않음. 벤더 함수를 사용해야 함 |
| 설정 파일 | `*_s` · `*_hz` 를 `float` 로 기재해야 노드가 기동함 |
| 20 에피소드 | 서로 다른 초기자세에서 공통 수렴점에 도달함. 정확도의 독립표본 20 개가 아님 |
