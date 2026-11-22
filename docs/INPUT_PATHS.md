# 외부입력 9 개 수신경로 — 신설 메시지(52501) 와 PX4 표준

- 작성: 충남대학교 자율운항시스템공학과 윤원근 교수
- 대상: `nuc/px4_state.py` · `sitl/px4_state_node.py` · `tools/sniff_mothership.py`
- 참조 구현: 캡스톤 UGV `auto_usv_pkg/coordinate_transformer_node.py`

## 1. 개요

- 정책이 요구하는 **외부입력 9 개**: `x, y, psi` (NED 자세) · `u, v, r` (바디속도) ·
  `x_ref, y_ref, psi_ref` (목표자세)
- 수신경로를 **2 개로 구현**하고, 같은 `/mothership/state` 토픽으로 올린다
- 정책·QP·터그 경로는 토픽만 보므로 **경로를 갈아도 diff 가 0** 임
- 선택 기준: 마더십 Pixhawk 가 PX4 표준 텔레메트리를 내보내면 표준경로가
  **변경량 0** 으로 이김. 안 내보내면 52501 경로를 씀

| 항목 | 신설 52501 | PX4 표준 |
|---|---|---|
| 와이어 메시지 | `MOTHERSHIP_STATE` 1 개 (payload 47 B) | `ATTITUDE` · `LOCAL_POSITION_NED` · `POSITION_TARGET_LOCAL_NED` |
| 다이얼렉트(`usv_tug.xml`) 변경 | 필요 (+16 줄) | **없음** (`<include>development.xml</include>` 에 이미 있음) |
| 마더십 Simulink 변경 | 필요 (송신 블록 추가) | **없음** (PX4 `mavlink` 모듈이 원래 내보냄) |
| 타인 저장소 노드 변경 | `mothership_rx.py` 가산 +32 줄 | **없음** (`sitl/px4_state_node.py` 신설) |
| `tug_interfaces` 변경 | `MothershipState.msg` 신설 + `CMakeLists.txt` +1 | 같음 (같은 메시지를 씀) |
| `psi_ref` 출처 | 마더십 LOS 와 **같은 출처** | 설정점 메시지의 `yaw`, 없으면 LOS 대체(옵션) |
| `rc_on_off` · `on_off` | 와이어에 포함 | 포함 안 됨 — 별도 경로 필요 (§5) |
| 현장 검증 상태 | 패치 산출 완료, 적용은 소유자 조율 | **확인 필요** — 마더십이 실제로 내보내는지 미측정 |

## 2. 메시지 대응

| 정책 입력 | PX4 표준 메시지 · 필드 | 변환 |
|---|---|---|
| `psi` | `ATTITUDE.yaw` | 없음 (NED, 위에서 시계방향 + = FRD) |
| `r` | `ATTITUDE.yawspeed` | 없음 (선체고정 z 각속도 = Fossen `r`) |
| `x, y` | `LOCAL_POSITION_NED.x, .y` | 없음 |
| `u, v` | `LOCAL_POSITION_NED.vx, .vy` | 2×2 회전 (NED → 바디) |
| `x_ref, y_ref` | `POSITION_TARGET_LOCAL_NED.x, .y` | 없음 |
| `psi_ref` | `POSITION_TARGET_LOCAL_NED.yaw` | 없음 |
| (대체) `x, y` | `GLOBAL_POSITION_INT.lat, .lon` | LLA → ECEF → NED (기준점 필요) |

- 속도 변환식 (`nuc/px4_state.py::body_from_ned`)

      u =  cos(psi)*vx + sin(psi)*vy
      v = -sin(psi)*vx + cos(psi)*vy

- UGV 노드는 3 축 쿼터니언 역회전을 씀(`coordinate_transformer_node.py:183–196`).
  roll = pitch = 0 평면에서 두 결과가 같음을 시험으로 고정
  (`tests/test_px4_state.py::test_quat_matches_rotation_matrix`, 최대차 1.67e-16)

## 3. UGV 예제와의 차이

- UGV 예제(`coordinate_transformer_node.py`)를 **구조 그대로** 따름:
  PX4 상태를 받아 좌표변환만 하고 다시 토픽으로 올리는 노드, 소비 노드는 출처를 모름
- UGV 예제가 **주지 않는 2 개**를 보충함

| 항목 | UGV 예제 | 본 하네스 |
|---|---|---|
| 각속도 `r` | 없음 (`/usv/state/*` 4 종에 미포함) | `ATTITUDE.yawspeed` / `VehicleAngularVelocity.xyz[2]` |
| 목표 자세각 `psi_ref` | 없음 (목표 위치만) | `POSITION_TARGET_LOCAL_NED.yaw`, 없으면 LOS 대체 |
| 위치 기준 | LLA → NED (원점 파라미터) | `LOCAL_POSITION_NED` 우선, LLA 는 대체경로 |
| 전송 | uXRCE-DDS (`px4_msgs`) | MAVLink v2 over UDP (실험 체인과 동일) · DDS 변형 동봉 |

- ⚠️ **전송계층이 다름**: 캡스톤 UGV 는 uXRCE-DDS, 예인 실험 체인은
  MAVLink v2 over UDP + DroneBridge ESP32 임. 그래서 **좌표변환·조립 논리만**
  가져오고 전송은 실험 체인의 벤더 코드(`MavUdpLink`)를 그대로 씀

## 4. 구현 파일

| 파일 | 역할 | 전송 의존성 |
|---|---|---|
| `nuc/px4_state.py` | 좌표변환 순수함수 + `Px4StateAssembler` (조각 → 상태) | **없음** (numpy·math 만) |
| `sitl/px4_state_node.py::Px4StdStateNode` | MAVLink UDP → `/mothership/state` | 벤더 `MavUdpLink` · `is_verified` |
| `sitl/px4_state_node.py::Px4DdsStateNode` | `px4_msgs`(DDS) → `/mothership/state` | `rclpy` · `px4_msgs` |
| `sitl/wire.py::MothershipTx.state_px4std` | 마더십 모사 송신 (표준 3 개) | SITL 전용 |
| `tools/sniff_mothership.py` | 마더십이 실제로 보내는 메시지 계수 | 소켓만 (읽기 전용) |

- `nuc/px4_state.py` 는 `nuc/` 규율(전송 모듈 import 금지)을 지킴 —
  `tests/test_same_code_both_paths.py` 가 검사함
- 수신 필터·재바인드 순서는 `vendor/tug_bridge/mothership_rx.py:129–175` 와 동일

## 5. 미해결 · 확인 필요

| 항목 | 현재 처리 | 확인 대상 |
|---|---|---|
| 마더십이 표준 3 개를 내보내는가 | 미측정 | `tools/sniff_mothership.py --port 14600 --seconds 30` |
| `rc_on_off` · `on_off` | 표준경로에는 없음 → 기본값 `0 / 1` 유지 | RC 채널(`RC_CHANNELS`) 또는 52501 경로 |
| `psi_ref` 가 설정점에 실리는가 | 없으면 발행 중단 (옵션으로 LOS 대체) | 마더십 설정점 블록의 `yaw` 사용 여부 |
| 포트 공유 | 같은 포트를 두 노드가 bind 할 수 없음 | `forward_addr` 로 기존 52502 경로에 전달 |
| DDS 변형(`Px4DdsStateNode`) | **이 환경에서 실행 못 함** (Windows·`shim/rclpy`) | NUC 에서 `px4_msgs` 빌드 후 1 회 확인 |
| `psi_time_log` 단위 | 표준경로는 rad 고정 (`ATTITUDE.yaw`) | Simulink 로그의 rad/deg 여부 |

## 6. 검증 결과

| 시험 | 기준 | 결과 |
|---|---|---|
| `tests/test_px4_state.py` 9 항 | 전부 통과 | 9/9 PASS |
| 외부입력 9 개 두 경로 동치 | < 1e-9 | **4.44e-16** (랜덤 200 점) |
| NED↔바디 왕복 | < 1e-12 | 1.11e-16 |
| UGV 3 축 역회전과 평면 2×2 | < 1e-12 | 1.67e-16 |
| LLA→NED 행렬 | UGV 원문과 동일 | 원소 최대차 0 |
| LLA→NED 100 m 변위 | < 0.01 m | N 100.001 m · E 99.999 m |
| 전체 시험 | 0 FAIL | 63 PASS / 0 FAIL |
| SITL 합격시험 (`--state-src px4std`) | 20/20 · 평균 ≤ 0.01 m | **20/20 · 0.00104 m** (§6.1) |

### 6.1 SITL 합격시험 — 실측

실행

    python sitl/run_sitl.py --episodes 20 --speed 1 --dr off --seed0 1000 \
        --state-src px4std --out out/SITL_RUN_px4std.json

| 항목 | 기준 | 52501 경로 | **PX4 표준 경로** |
|---|---|---|---|
| 합격 에피소드 | 20/20 | 20/20 | **20/20** |
| 평균 위치오차 | ≤ 0.01 m | 0.00104 m | **0.00104 m** |
| 최대 위치오차 | — | 0.00104 m | **0.00104 m** |
| 평균 yaw 오차 | — | 0.0172 deg | **0.0172 deg** |
| 제어스텝 | 3000/3000 | 전 에피소드 | **전 에피소드** |
| `overrun` 합 | — | 50 (ep 19 만) | **0** |
| 최대 틱지연 | — | 29.0 ms | **17.0 ms** |

- 두 경로의 위치·요 오차가 **소수 다섯째 자리까지 같음** — 외부입력 9 개가 수치적으로
  동일하므로(§6 의 4.44e-16) 제어기 이하가 같은 궤적을 그림
- 표준경로는 `overrun` **0** · 최대지연 17.0 ms (제어주기 20 ms 의 85 %).
  52501 경로에서 ep 19 에만 나온 `overrun 50` · 29.0 ms 지터는 표준경로에서 재현되지 않음
  → 위치오차가 두 경로 동일하므로 **제어성능이 아니라 Windows 스케줄링 지터**임이 확인됨
- 수신 통계: `rx_pkt` 243 609 · `rx_state` 60 599 · `rl_pub` 12 120 ·
  `rl_stale` **0** · `rl_override` **0** · `tx_err` **0** · `n_sat` **0**
- 정책·QP·터그 코드 **재학습·수정 없음** (zero-shot)
- 원본: `out/SITL_RUN_px4std.json` · `out/SITL_px4std.txt`
