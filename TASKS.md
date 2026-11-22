# TASKS — harness_sitl 가상 테스트 하네스

- 최종 갱신: 2026-11-23
- 이전 원장 백업: `out/TASKS_prev_20261122.md` (209 줄)
- **요약과 충돌하면 이 파일이 이긴다.** compact 직후 이 파일을 가장 먼저 읽는다.
- 운용 규칙: 스킬 `auto-compact-resume` / `session-continuity` / `context-budget`

---

## 1. 완료 — 실측값

| 항목 | 결과 | 파일 |
|---|---|---|
| 폐루프 A→B→C→D→A 20 에피소드 | **20/20**, 평균 0.00252 m (최대 0.00517), yaw 0.0296° (최대 0.0765°), 14.9 s (0.75 s/ep) → **PASS** | `out/acceptance_20ep.txt` |
| 테스트 스위트 | **32 PASS / 0 FAIL** (S4 에서 6 케이스 추가) | `out/test_suite.txt` |
| ONNX 내보내기 자체 시험 (2000 샘플) | max abs 9.090e-07 / mean 3.618e-08 (허용 1.0e-05), opset 17 | `out/onnx_parity.txt` |
| ONNX 온매니폴드 롤아웃 대조 (n=2404) | max abs 6.557e-07 / mean 8.232e-08 | `tests/test_onnx_parity.py` |
| sb3 대 ONNX 유일한 차이 | **행동 클리핑뿐** (클립 전 6.627e+00 → 클립 후 6.557e-07) | `test_clipping_is_the_only_difference` |
| 와이어 양자화 고정 | float32 상대오차 2.0e-7 미만, float64 경로 오차 정확히 0, CRC 0x6F91 | `tests/test_wire_roundtrip.py` |
| 레이트 분리 실측 | 물리 50 Hz : 제어 10 Hz = 5.00 : 1 | `tests/test_rates_match_env.py` |
| 하위 역행렬 검증 | 오차 1.8e-15 (`det T = 2d = 0.6`) | `nodes/node_d_esp32.py` |
| 통신 열화 스윕 18 케이스 | 포락선 state 지연 10틱(200 ms) · cmd 지연 5틱(100 ms) · 손실 50 % / **왕복 RTT 400 ms 에서 붕괴** | `out/SWEEP_COMM.md` |
| 상행 대 하행 등가 | IAE 8.35 = 8.35 → 예산은 **RTT 로 적어야 함** | `out/SWEEP_COMM.md` |
| 영상 재사용 | `pontoon_rl/viz/make_video.py` 무수정, 600 프레임 20.0 s | `out/harness_sc3.mp4` |
| 래치 한계 스윕 | 임계는 **전단 `F_s` 8 N**, 박리 `M_max` 아님 | `pontoon_rl/out/n1/SWEEP_LATCH.md` |
| `pymavlink 2.4.50` 설치 | `fastcrc 0.5.0`, `lxml 6.1.3` 동반 | venv |
| 자동 compact·재개 스킬 | 57 줄 | `C:\Users\admin\.claude\skills\auto-compact-resume\SKILL.md` |

### 1.1 확정된 다이얼렉트 (`cooperative-tug-experiment-main/experiment/mavlink/usv_tug.xml`)

| 메시지 | id | 포맷 | payload | frame | crc_extra |
|---|---|---|---|---|---|
| `MOTHERSHIP_STATE` | **52501** (신설) | `<QfffffffffBBB` | 47 B | 59 B | **119** (측정 완료) |
| `MOTHERSHIP_DATA` | 52502 | `<Qfff` | 20 B | 32 B | 123 |
| `TUG_COMMAND` | 52503 | `<Qdddddd` | 56 B | 68 B | 16 |

- ★ 52501 **측정 완료** (2026-11-22): `CRC_EXTRA 119` · `<QfffffffffBBB` · payload 47 B · 프레임 59 B.
  같은 산출물에서 52502(123)·52503(16)도 **회귀검사 통과** → 메시지 추가가 기존 두 메시지의
  CRC 를 바꾸지 않음을 **측정으로 확인**했다. 재현: `python mavlink/gen_dialect.py` → `out/gen_dialect.txt`.
  와이어 필드순서(= 선언순서) : `timestamp, x, y, psi, u, v, r, x_ref, y_ref, psi_ref, scenario_mode, rc_on_off, on_off`
- 52501 의 선언 필드순서는 MAVLink 의 크기 내림차순 와이어순서와 이미 일치 → **재배치 없음**, `MothershipState.msg` 를 1:1 로 둘 수 있다.
- 송신자는 `MOTHERSHIP_DATA` 와 **같은 sysid 1 / compid 1 / 같은 포트 14600**.
- `<version>3</version>` 은 올리지 않는다. 추가 사실은 README·changelog 에만 적는다.
- 52502 는 **한 줄도 건드리지 않는다.** 터그는 52503 만 받으므로 터그 Pixhawk·Simulink·다이얼렉트는 **전혀 바뀌지 않는다.**

### 1.2 실주소·레이트 (`config/tug_bridge.yaml`, `tug_bridge_loopback.yaml`)

- NUC `own_sysid 250` / `own_compid 191`, 마더십 1 / 터그 2·3, `TUG_ACCEPTED_SYSID = 250`
- NUC bind `14600`(마더십 수신) · `14601`(터그1 송신) · `14602`(터그2 송신), HB `14660`
- 실물 ESP32 `192.168.68.201/.202/.203` : **원격 포트는 항상 14550** (DroneBridge 고정)
- 루프백: `14601 → 127.0.0.1:14651`, `14602 → 127.0.0.1:14652`
- `send_rate_hz 50.0`, `input_timeout_s 0.5`, HB 1 Hz / `heartbeat_timeout_s 3.0`, `shutdown_zero_count 10`, `stats_period_s 2.0`, `rx_timeout_s 1.0`, `rebind_after_s 5.0`, QoS `best_effort`
- `filter_sysid 1`, **`filter_compid 0`** → compid 1 짜리 52501 도 통과

### 1.3 S6 Simulink — **실패로 종결** (보고서에 그대로 적는다)

- 되는 것: 모델 열기, 구조 열거 (블록 1024, 마스크 50, BusSelector 11)
- 안 되는 것: **컴파일 불가** (`px4:blockmask:InvalidTopicName` 13 건), **한 번도 돌지 않았다**, **바이패스 탭은 추가되지 않았다**
- 근본원인: 모델 마스크값이 CamelCase (PX4 >= v1.14) · 이 PC 펌웨어 트리 `C:\PX4\home\Firmware` 는 snake_case. 게이트는 `<build>/uORB/topics/<mask>.h` 에 대한 문자 그대로의 `fopen`
- 결정적 측정: `getBusDefnForuORBMsg('MissionResult','')` → FAIL / `('mission_result','')` → `[OK] 15 elements`
- 해소책은 이 과제 범위 밖. 상세 기록은 `out/TASKS_prev_20261122.md` §5

---

### 1.4 ★ SITL 상행 손실 약 50 % — 원인규명·수정 완료 (2026-11-22)

- **증상**: SITL 에서 NUC 가 받는 `52501/52502` 가 약 **절반** 사라진다. 송신율과 **무관**.
  `parse_errors`·`drop_msgid`·`drop_sysid`·`dropped_ip` 가 **전부 0** 이라 카운터가 원인을 가린다.
- **원인**: `tug_bridge.yaml` 의 `heartbeat_target_port: 14660` 을 **하네스가 아무도 듣지 않았다.**
  NUC 의 1 Hz HEARTBEAT 가 죽은 포트로 가면 ICMP port-unreachable 이 돌아오고, **Windows 는
  그것을 같은 UDP 소켓의 `recvfrom` 에 `WSAECONNRESET`(10054) 로 올린다.** 벤더 `MavUdpLink` 는
  이를 비일시 오류로 보아 소켓을 닫고 재바인드한다(`reopen_count`) → 닫힌 창에 도착한
  데이터그램이 전부 사라진다. **실측 `rx_reopen 10` / 10 s** (= HB 1 Hz).
- **수정**: `sitl/wire.py::MothershipTx` 가 `14660` 을 **듣기만 하고 버리는** 소켓으로 열고
  `plant_proc.tick()` 이 `drain_hb()` 로 비운다 = **실물에 실제로 있는 마더십 ESP32 를 모사**.
  **벤더 코드 diff 는 0 을 유지**했다.
- **전/후**: `rx_pkt 561/1071 → 1071/1071` · `rx_reopen 10 → 0` · 손실 50 % → **0 %** ·
  1 에피소드 `dpos 0.0156 → 0.00081 m`.
- **교훈 2 개**: ① **하네스 충실도 결함이 플랜트 성능저하로 위장**한다 (벤더 버그가 아니었다).
  ② 진단용 프로브가 손실 0 % 였던 것은 **프로브가 죽은 포트로 송신하지 않았기 때문**이다.
- ⚠️ **`--speed > 1` 은 지연만 바꾸는 것이 아니라 제어주기를 바꾼다.** 플랜트 벽시계만 압축되고
  NUC 노드의 `rclpy` 타이머는 실벽시계라 제어기가 굶는다. 실측 `--speed 8` → `ctl_policy 5/40`,
  `dpos 0.927 m`. **성능판정은 `--speed 1` 에서만 유효**하다 (`summary["pass"]` 가 이를 강제한다).

---
## 2. 잔여 — 교차세션 지시 3건 반영분

- [x] S1 자동 compact·재개 스킬 추가
- [x] S2 TASKS.md 원장 갱신 (이 파일)
- [x] S3 **참조 소스 타깃 grep** — 8 파일 전부 `grep -n`/`sed -n` 으로만 확인 완료.
      확정사항은 §1.1·§1.2·§3 에 반영했다. 전문 읽기는 하지 않았다.
- [x] S4 **C·D 를 `harness_sitl/nuc/` production 모듈로 분리** — ★ **산출물은 전부 있다.**
      `nuc/__init__.py` · `nuc/controller.py`(187) · `nuc/onnx_policy.py`(69) 신설 (production,
      전송계층 import 0). `plant/tug_lower.py`(120) 신설 — 터그 하위 역행렬을 노드 D 에서 분리.
      `nodes/node_c_nuc.py`(177) · `nodes/node_d_esp32.py`(133) 는 **얇은 전송 어댑터**로 다시 씀
      (위임 상태 전부 **읽기전용 property** — 외부 이름 호환 유지). `docs/FILE_SWAP.md`(86) 작성.
      `tests/test_same_code_both_paths.py`(226) 작성 — 6 케이스(경계 2 · 단독동작 1 · 두경로 일치 1 ·
      양자화 크기 1 · property 1).
      **마감 완료**: `MODULES` 등록 · 묶음 재실행 **32 PASS / 0 FAIL** (`out/test_suite.txt`) ·
      `tools_sweep_comm.py:138` 인용 수정(`plant/tug_lower.py`).
      실측: 두 경로 `u6` 오차 **0.0e+00** (양자화를 손으로 적용했을 때) · 전송 양자화만 남겼을 때
      **7.38e-06 N** (상한 1e-02) · 비포화 3/24 스텝 · `nuc/*.py` 3개 금지 import **0건**.
      ⚠️ `test_nuc_deps_are_numerics_only` 가 처음에 FAIL 했다 — import 정규식의 `\w` 가
      한글을 먹어 docstring 의 "import 하지 않는다" 를 모듈명으로 읽었다. **ASCII 식별자로 제한해 해결.**
- [x] S5 **52501 + pymavlink 실물경로** —
      ① `tools/fake_mothership.py` 자리에 우리 플랜트(터그2 + 폰툰, M2 UDE), `tools/fake_tug.py` 자리에
         플랜트의 터그 입력단. `mothership_rx.py`/`tug_tx.py` 양 끝만 바꾼다
      ② `relay_common.py` + `mav_link.py` 를 **import** (자체 패킷 구현 금지). rclpy·`tug_interfaces` shim 필요
      ③ 52501 블록을 `usv_tug.xml` **사본**에 추가 + `MothershipState.msg`(ROS `float32`/`uint8`, 1:1) +
         `simulink/MothershipState.msg` 사본
      ④ `mothership_rx.py` **L163 과 L164 사이**에 52501 분기 삽입 + `/mothership/state` 퍼블리셔.
         카운터는 **새로 만든다**(`state_total`/`last_state_mono`) — 기존 `[stats]` 줄을 안 건드리려면 필수
      ⑤ `rc_on_off = 1` → RL 출력 무시하고 τ=0. **그 경로를 시험한다**
      ⑥ 안전동작 재현 (0.5 s 공백 → τ=0 · NaN/inf 한 필드면 **메시지 전체** τ=0 · 종료 시 τ=0 **10회** ·
         SIGINT/SIGTERM/SIGHUP 중복에도 종료 버스트가 끊기지 않음)
      ★ **완료 2026-11-22.** 산출물: `sitl/{bootstrap,wire,plant_proc,nuc_nodes,run_sitl}.py` ·
      `shim/{rclpy,tug_interfaces}` · `vendor/tug_bridge/` 사본 + `vendor/README.md` ·
      `tests/test_safety.py` (7 PASS / 0 FAIL).
      **벤더 diff 재실측** (`test -f` 로 양쪽 입력 존재를 먼저 증명한 뒤 `diff --strip-trailing-cr`):
      `tug_tx.py` **0** · `mav_link.py` **0** · `relay_common.py` **0** · `mothership_rx.py` **+32/−0 (가산만)**.
      → `tug_tx.py` 가 0 이므로 **터그 Pixhawk·Simulink 는 전혀 바뀌지 않는다.**
      ⚠️ **POSIX 시그널 대역(`sitl/bootstrap.py`)은 전송·플랜트 에 이은 세 번째 가상/실물 차이**다
      (Windows 에 `SIGHUP` 이 없다). 가상으로 증명된 것은 "τ=0 ×10 로직"이고, 증명되지
      **않은** 것은 "SSH 끊김(SIGHUP)이 실제로 그 로직을 깨우는가"다 → S8 보고서에 그대로 적는다
- [x] S6 **동일코드 증명 + 합객기준 재실행** — ★ **완료.**
      ① **52501 13필필드 웕복** — `tests/test_wire_roundtrip.py` (웕복 · 뒤쪽 0
        truncation 복원 · `CRC_EXTRA` 3 개 고정) → 그 모듈 **13 PASS**.
        `CRC_EXTRA`: 52501 **119**(신설) / 52502 123 · 52503 16 **변지 없음** → 하위호환 근거.
      ② **동일코드 패리티 시행 실측** — `tests/test_sitl_parity.py` **3 PASS** (새로 만들었다).
        도구: 토픽 구독큐를 `_drain()` 로 **지정해 부른다 (spin 없음)** → 벽시계가 개업할 수 없다.
        · `two_adapters_same_tau_exactly` : 40 스툹 × 6 성분, τ_d · τ_i 차이 **정확히 0**
          (|τ_i|max 35.85 N — 둘 다 0 이어서 통과하는 것이 아니라는 것을 같이 단정한다).
        · `a_prev_evolves_identically` : 12 스툹 동일, 차이 **정확히 0** → 상태를 가진 정책이
          어댑터에 따라 갈라지지 않는다.
      ③ ★ **발견 — inproc 전송은 상태를 float32 로 양자화한다.**
        `iface/transport.py:94` 의 `publish` 가 `PontoonState.to_wire()` (`WIRE_DTYPE=float32`)
        를 통한다. SITL/rclpy 경로는 그 양자화가 **없다**(shim 은 float64 로 들고 있다).
        → **두 경로의 전송계증까지 포함한 무조건 동일은 원리적으로 불가능**하다.
        실측 τ_d 차이 **1.43e-05 N** (|τ_d|max 60.0 N, 상대 **2.4e-07**, 상한 1e-03 N).
        해석: inproc 하네스는 실물보다 **나쪼 입력**으로 돌아간다 → 그 합부 통지는 **보수적**이다.
        ⚠️ 이것은 설계상의 성질이지 부업가 아니다. 보고서에 수치로 적는다.
      ④ **20 에피소드 SITL 합객시행 — PASS** (`out/SITL_ACCEPT.json` · `out/sitl_accept.txt`).
        `--speed 1 --dr off --seed0 1000` → **20/20 · 평군 0.00104 m (최대 0.00104) ·
        방위 0.0172° · overrun 0 · worst_lag 16–17 ms** (기준 0.01 m).
        `pass: true` · `perf_valid: true` · steps 600/ep.
        계상기: `rx_pkt 122409` · **`rx_reopen 0`** · `rx_parse_err 0` · `rx_drop_sysid 0` ·
        **`tx_state 60600 == rx_state 60600` → 상행 손실 0 %** (§1.4 수정이 산에서 지형됨) ·
        `rl_stale 0` · `qp_stale 0` · `rl_override 122` (에피소드 사이 `on_off=0` 경로가 실제로 돌았다) ·
        `tx_err 0` · **`n_sat 0`** (터그 하위배분 미포화).
      ⑤ **전수 시행: 45 PASS / 0 FAIL** (42 + 신설 패리티 3).
        inproc 기준선 `test_closed_loop_acceptance` 20/20 · 평군 0.00252 m (최대 0.00517) ·
        하위배분 웕복 1.8e-15 · 레이트분리 5.00 : 1 · 새로운 7 시행 전부 PASS.
      ⑥ **올바른 해석 — 20 시드는 서로 다른 초기자세에서 시작한다.**
        `pontoon_rl/env/pontoon_env.py::reset(seed)` 가 `init_pos 3.0 m` · `init_yaw 0.52 rad`
        (`../pontoon_rl/params/sc3.yaml:33-34`) 로 군일분포를 농는다. 그런데도 마지막
        오차가 8 번째 소수자리까지 같다 → **공통 인인자(attractor) 수렴**이지
        시드버그가 아니다. 단 그러바에 **20 수값은 정확도의 독립표본 20 개가 아니다**
        (20 개의 다른 출발에서 전부 수렴했다는 진술이다). 보고서에 그대로 적는다.
- [x] S7 **다이어그램 SVG 생성** — ★ **완료. 4 장 생성됨** (`docs/img/*.svg`,
      XML well-formed 확인, 7.9–18.4 kB). `make_diagrams.py` 를 처음으로 실행했다.
      고칠 점 — 그림은 보고서에 실리므로 문구를 사실과 대조했다:
      · 전송계증 반드 `ros2 (스툹, 확인 필요)` → **`SITL = MAVLink v2 over UDP`**
        (DDS 전제 폐기. inproc 은 "하네스 내부, 상태를 float32 로 양자화" 로 명시)
      · B→C 화살표 `/mothership_data <Qfff · 52502` → **`/mothership/state <Q9f3B · 52501
        (+52502 공존)`** — RL 은 52501 로 상태를 받는다
      · 필드표 그림에 **52501 행 신설** (`<QfffffffffBBB` · `crc_extra 119` · **47 B**) —
        세 값 전부 `mavlink/gen/usv_tug.py:6777-6782` 에서 직접 지형했다. 등록효과로 H 330→394
      · 정책 그림의 `19 - 128 - 128 - 3 · tanh · opset 17 · 78 kB` 는 **사실**이다 —
        `out/ppo_sc3_final.onnx` 실제 가중치 `[128,19] [128,128] [3,128]`,
        ops `Gemm/Tanh/Gemm/Tanh/Gemm`, opset 17, **78 867 B**
      ⚠️ **새로 발견한 공백**: `docs/SITL.md` 를 `sitl/nuc_nodes.py:34` · `sitl/run_sitl.py:46` ·
      `sitl/wire.py:129` 가 가리키지만 **그 파일은 없다.** `docs/manual/` 도 뵄 비어 있다
      → S8 에서 `docs/SITL.md` 를 만들어 세 참조를 해소한다.
      실행 전에 전송계층 밴드의 `ros2 (스텁)` 문구를 **MAVLink v2 over UDP** 로 고친다 (DDS 전제 폐기)
- [x] S8 **README + PDF 보고서** — ★ **완료 2026-11-22.**
      산출물: `README.md`(10 장, 개조식, 그림 4 장, 표 15종) · `docs/SITL.md`(8 절) ·
      `docs/make_manual.py` · `docs/manual/harness_sitl.pdf`(557 154 B) ·
      `docs/manual/check/page_full.png`.
      수치는 전부 로그에서 다시 뜨였다 (`out/t_all.txt` **45 PASS / 0 FAIL**).
      §7.5 검증의 한계 · §7.6 하네스 충실도 결함 사례 · §10.1 미확인 항목을 그대로 적었다.
      `docs/SITL.md` 가 S7 의 세 참조(`nuc_nodes.py:34` · `run_sitl.py:46` · `wire.py:129`)를 해소한다.
      ⚠️ **스킬에서 벗어난 점 2건 (숨기지 않는다)**:
      ① `marked.js` 대신 파이썬 `markdown` 모듈로 렌더한다 — 이 PC 에 node·marked 가
         없고 CDN 의존을 만들지 않기 위함. A4 PDF 생성은 스킬대로 Chrome 헤드리스.
      ② PyMuPDF 가 설치되어 있지 않아 **PDF 쪽 단위 PNG 검사를 못 했다.** 대신 A4 폭(794 px)
         HTML 전체를 PNG 으로 떠 10 조각으로 나눠 전수 확인했다 (취소선 · 표 넘침 · 그림 잘림
         없음). **쪽나눔(page break) 자체는 이 방법으로 검증되지 않는다.**
      수정 이력: 그림 1 의 화살표 라벨이 상자를 침범해 간격을 80/80/40 px → 100/120/100 px 로
      다시 잡았다(`make_diagrams.py` 의 `xs`). `--episodes`·`--steps` 기본값을 `run_sitl.py`
      argparse 와 대조해 고쳤다(1→20, 600→0).
- [x] S9 **run_all + git 준비** — ★ **완료 2026-11-22.** `run_all.ps1`(CRLF+BOM) ·
      `run_all.sh`(LF, 실행권한) · `.gitignore` · `docs/REPO_SCOPE.md`.
      - 6 단계: `dialect` → `onnx` → `diagrams` → `tests` → `sitl` → `manual`.
        `--list` / `--only <step>`, 로그 `out/run_all_<step>.txt`, 실패 시 꼬리 20 줄 출력 후 중단(rc 1)
      - **전체 재현 실측 (bash, 2026-11-22): 6/6 PASS, 1 246 s.**
        dialect 1 s · onnx 3 s · diagrams 0 s · tests 26 s · **sitl 1 213 s** · manual 3 s
      - **SITL 합격판정: 20/20 · 평균 0.001 04 m (최대 0.001 04) · yaw 평균 0.017 2 deg → PASS.**
        매 에피소드 `k 3000/3000`, `worst` 16–17 ms
      - ⚠️ **`overrun 50` 은 전부 ep 19 (seed 1019) 한 에피소드에서 나왔다** (`worst 29.0 ms`).
        나머지 19 에피소드는 `overrun 0`. 위치오차는 모든 에피소드가 동일(0.0010 m)하므로
        **제어 성능이 아니라 Windows 스케줄링 지터**다. 원인 미규명 — **확인 필요**
      - `.gitignore`: `mavlink/gen/` · `out/*` 제외하되 `ppo_sc3_final.onnx` ·
        `SITL_RUN.json` · `SWEEP_COMM.md` 되살림. `*_steps.zip` 제외 / `ppo_sc3_final.zip` 포함.
        `vendor/tug_bridge/` · `simulink/*.slx`·`*.m` 기본 제외
      - CONFIDENTIAL 판단: `docs/REPO_SCOPE.md` §3. **사용자 결정(2026-11-22) = 비공개 저장소 전제**
        → ⚠️ 4 항(`config/tug_bridge.yaml` 실주소 · `mavlink/usv_tug.xml` · `README.md` §2.2 ·
        `out/ppo_sc3_final.onnx`) **포함**, 기본제외 2 항 **그대로 제외** (§3.1 에 기록)
      - ⚠️ 함정 2 건: ① PowerShell 5.1 은 **BOM 없는 UTF-8 을 cp949 로 읽는다** → `run_all.ps1`
        은 CRLF+BOM, `run_all.sh` 는 LF 로 써야 한다 ② PowerShell 에서 **스칼라 해시테이블의
        `.Count` 는 키 개수**를 돌려준다 → `Where-Object` 결과를 `@(...)` 로 감싸야 한다
      - **`git init` · 커밋 · push 는 하지 않았다.** 사용자 승인 전까지 준비만 한 상태다
- [x] S10 **패치 산출** — ★ **완료 2026-11-22.** `patch/0001-add-mothership-state-52501.patch`
      (9 754 B, **+118 / −0**), 생성기 `patch/make_patch.py`, 절차서 `patch/README.md`.
      - 대상 **5 파일**: ① `usv_tug.xml` +16 ② `mothership_rx.py` +32 ③ `msg/MothershipState.msg`
        신규 +31 ④ `tug_interfaces/CMakeLists.txt` +1 ⑤ `simulink/MothershipState.msg` 신규 +38
      - ★ ④ 는 **설계에 없던 대상이었다.** `CMakeLists.txt` 에 `"msg/MothershipState.msg"` 를
        등록하지 않으면 패키지가 새 메시지를 빌드하지 않아 ② 의 import 가 실패한다
      - **전 파일 가산 전용 (−0).** `make_patch.py` 가 삭제 줄 수를 검사해 0 이 아니면 실패 반환한다
      - 검증(실측): 원본 3 파일을 **격리 사본**에 복사해 git 저장소를 만들고 `git apply`
        → **성공, 경고 0**. 적용본과 하네스 사본 바이트 비교 → `mothership_rx.py` ·
        `MothershipState.msg` ×2 **동일**. 적용본 5 파일 전부 **LF**. 적용본 `usv_tug.xml` 은
        하네스 `mavlink/usv_tug.xml` 과 주석 제외 **동일**(빈 줄 1 개 차이뿐)
      - `CRC_EXTRA` 실측: 52501 = **119**(신설) · 52502 = **123** · 52503 = **16**.
      - ⚠️ **줄끝 측정 정정**: 앞서 "입력 전부 CRLF" 로 적었던 측정은 **틀렸다** (`\n` 을 CRLF 로 세었다). **대상 저장소는 전부 LF** 이고, CRLF 였던 것은 우리 `mavlink/usv_tug.xml` 하나뿐이다
        → LF 로 변환했다. 변환 후 `run_all --only dialect` **재실행 PASS**:
        52502 CRC 123 회귀 OK · 52503 CRC 16 회귀 OK · 52501 CRC 119 payload 47 B
        → **LF 로 되돌린 xml 이 같은 다이얼렉트를 만든다 (실측 2026-11-22)**
        **52502/52503 은 패치 전후 불변** → 구 펌웨어·신 다이얼렉트 혼재에도 기존 두 메시지는 통한다
      - 영향범위: 마더십 Simulink(송신 블록 추가) · NUC `tug_interfaces`(재빌드) ·
        NUC `mothership_rx`(가산 분기) **만**. `tug_tx.py` diff **0**,
        **터그 Pixhawk·Simulink·ESP32 전부 변경 없음**(터그는 52503 만 받는다).
        마더십 ESP32 는 코드 변경 없으나 상행 대역 **+2.4 kB/s**(47 B @ 50 Hz)
      - ⚠️ **정정**: 앞서 "입력 전부 CRLF" 로 적었던 측정은 틀렸다. 대상 저장소는 **전부 LF** 다
        (`
` 을 CRLF 로 세었다). 그래서 하네스 사본을 b 측으로 그대로 쓰면 줄끝이 섞인
        혼합 파일이 나왔다 → `make_patch.py` 가 원본 줄끝에 맞춰 정규화하고,
        우리 `mavlink/usv_tug.xml` 도 LF 로 되돌렸다
      - ⚠️ **푸시하지 않았다.** 적용·커밋·PR 은 **저장소 소유자 조율 사항**이다
      - ⚠️ 확인 필요 5 항(`psi` 단위 · `rc_on_off`/`on_off` 의미 · `x_ref/y_ref` 절대/상대 ·
        송신 주기 50 Hz · 터그 Pixhawk 구성)은 `patch/README.md` §8 에 표로 남겼다

### 2.1 범위 규칙 (합격기준의 측정대상)

| 파일 | 허용 diff |
|---|---|
| `tug_tx.py` · `mav_link.py` · `relay_common.py` | **0** |
| `mothership_rx.py` | **가산만** (52501 분기 + `/mothership/state` 퍼블리셔). 52502 경로의 기존 줄은 **단 한 줄도 변경 금지** |

---

## 3. 확인 필요 — 추측을 사실로 적지 않는다

| 항목 | 현재 가정 | 확인 대상 |
|---|---|---|
| 폰툰 `I_z` | `sc3.yaml` **147 kg·m²** 사용 | `CLAUDE.md` §9 는 457~914. 3~6배 차이. 같은 물체인지 미확인. **임의로 고치지 않는다** |
| `psi_time_log` 단위 | 미확정 | `mothership_test_v0_261104.slx` 에서 직접 확인 (컴파일 불필요) |
| `rc_on_off`/`on_off` | 52501 에서 `uint8` | Simulink 는 **float32**, `scenario_mode` 는 **float64** → 축소 타당성 |
| `x_ref`/`y_ref` | NED 절대 | 상대일 가능성 |
| 터그 Pixhawk | 미확정 | Simulink 생성앱인지 PX4 순정 모듈인지 |
| `tug_bridge.yaml` **(가정)** 태그값 | 원작자 표기 그대로 | **태그를 지우지 말고 보고서로 운반** |
| 52501 `crc_extra` | 미측정 | `tools/gen_dialect.sh` 로 **재야 한다. 추측 금지** |
| 지터 모델 | 없음 | 스윕은 고정지연만. 실물 Wi-Fi 지터 미반영 |
| 관측 차원 | 구현 **19차** | `BEST_PLAN` §7.3 계획은 52차 |
| `r1`/`r2` 관측값 | 지터 적용된 **참값** | 실물에서는 추정해야 함 |
| PWM↔추력 곡선 | 없음 | 계류추력 시험 미실시 |
| `/pontoon/state` | 하네스 내부 토픽 | **실배포 시스템에 존재하지 않는다** |
| `setpref` 경로 2건 | 적용됨 | 출처 파일이 스스로 "For testing" 이라 표기 |
| 펌웨어 `.msg` 수 | 158 | 다른 집계는 161 |

---

## 4. 환경 함정 — 재현에 필요

- 파이썬은 **반드시** `../plan_pontoon_rl/.venv/Scripts/python.exe`. 맨 `python` 은 3.8.2 REPL 로 떨어져 먹통
- ⚠️ **`restoredefaultpath` 금지** — MATLAB MCP 통신이 영구히 깨진다
- ⚠️ `cooperative-tug-experiment-main/` 은 **타인 저장소**. 읽기전용·비신뢰. **`cd` 로 들어가지 않는다.**
  `harness_sitl` 에서 `R="../cooperative-tug-experiment-main/experiment/..."` 상대루트로 읽고,
  그 안에서 인터프리터·빌드도구를 **실행하지 않는다.** 필요한 파일은 복사해 쓰고 출처를 `파일:줄` 로 적는다
- yaml 의 `*_s` / `*_hz` 는 **float**(`50.0`) 로 써야 노드가 뜬다
- 콘솔 cp949 → 한글·en dash 출력 전에 `_console.utf8_stdout()`
- venv 결손: `pytest` 없음(`tests/run_tests.py` 가 직접 호출) · `PyMuPDF`(`fitz`) 없음(보고서 §5 육안검증 막힘) ·
  `rclpy`/`tug_interfaces` 없음(노드 무수정 실행에 shim 필요 — shim 경로가 diff 0 을 지킨다)
- 2 분 넘는 스윕은 처음부터 `run_in_background`
- Bash 함정: `VAR="$VAR$(printf '...\n')"` 는 끝 개행을 **조용히 삭제**한다.
  큰 다중행 파일은 heredoc 이 두 번 깨졌으므로 **`Write` 도구로 쓴다**
- Chrome: `C:\Program Files\Google\Chrome\Application\chrome.exe` / `md2pdf.sh` 는 Dropbox `_tools\`
- 그림 비율 h/w <= 1.0 (PDF `max-height: 248mm`)
- `mcp__matlab__evaluate_matlab_code` 는 `code`·`project_path` 만 받는다 (`description` 주면 거부)

---

## 5. 다음 한 걸음

### 5.1 다이어그램 4장 재작성 — 완료 (2026-11-22 밤, 사용자 "다이어그램이 이해가 않가")

- `docs/img/src/make_diagrams.py` 를 **전면 재작성**했다. 4 장 모두 재생성·PNG 렌더로 육안 확인.
  설계 규칙: ① 모든 그림에 **범례** ② 신호 흐름에 **번호 ①②③…** ③ 블록마다 **하네스 파일명**
  ④ 반복 칸을 채우지 않고 1 주기만 그림 ⑤ 되먹임선은 **만드는 블록에서 출발**.
- ★ **사실관계 2건을 고쳤다** (렌더해서 보기 전에는 드러나지 않았다):
  ① 구버전 그림 3 의 되먹임 `res·marg` 가 **플랜트에서 출발**해 있었다 → 실제로는 **QP 가 만든다**.
  ② 블록에 적힌 파일명이 **전부 틀렸다** (`node_b_bypass` · `node_d_tug` · `plant/` 는 없는 파일).
  실제는 `node_a_plant` · `node_b_mothership` · `node_c_nuc` · `node_d_esp32` · `plant/tug_lower.py`.
- 레이아웃 변경: 그림 3 에서 **QP 를 우하단, 플랜트를 좌하단**으로 바꿔 되먹임 간선 3 개가
  겹치던 것을 **좌·우 바깥 간선 2 개**로 줄였다. 크기 overview 1180×760 · tick 1120×520 ·
  policy 1180×1000 · frame 1120×690 → **h/w 전부 ≤ 1.0** (PDF 쪽넘김 제약 충족).
- README §3.1 관측표에 **색인·출처 열**을 추가하고 `docs/obs_howto.py` · `RL_SPEC.md` §2.3 링크를 걸었다.
  그림 3 캡션을 `관측 19차 조립 · 정책 · 추력배분 QP` 로 바꿨다.

### 5.2 남은 것 — **전부 완료 (2026-11-22)**

1. README 확장 — 완료 (파일별 기능 표 §1.6, 되는 것/안 되는 것 §1.5, 영상 §7.7)
2. `out/sitl_sc3.mp4` — **포함**으로 결정 (`.gitignore` 예외). 근거 `docs/REPO_SCOPE.md` §3.2.1
3. PDF 재생성 — 완료 (1 308 276 B, 수식 102 식 · katex-error 0)
4. **공개 저장소 생성 + push** — 완료. §9 참조. 타인 저장소 패치는 푸시하지 않았음

---


## 6. 작업규칙 — 컨텍스트 (2026-11-22 추가, compact 반복 대응)

- **진단**: `CLAUDE.md` 가 959줄/72.9 kB 였고 **세션 시작·compact 직후마다 전문이 재주입**되는
  고정비였다. 그 중 약 2/3 가 스스로 "폐기·무효·철회" 선언한 VRX 시절 절이었다.
  → **쪼갰다**: live `CLAUDE.md` **435줄/37.5 kB** (49 % 감축) + `CLAUDE_VRX_ARCHIVE.md` 598줄.
  **내용은 한 글자도 바꾸지 않았다. 위치만 옮겼다.** 원본 백업 `out/CLAUDE_full_20261122.md.bak`.
  행수 검산 389 + 570 = 959 (원본과 정확히 일치).
- **가변비 규칙** (`CLAUDE.md` §10 항목 7·8 과 동일. 여기에도 적어 compact 를 넘긴다):
  1. **자기가 방금 쓴 파일을 되읽지 않는다.** `Write`/`Edit` 가 실패하면 도구가 에러를 낸다.
  2. 확인이 필요하면 **범위읽기만** (`sed -n 'A,Bp'` / `grep -n`). 전문 읽기는 처음 한 번뿐.
  3. 시험·스윕 출력은 **파일로 빼고 꼬리만 읽는다** (`... > out/x.txt 2>&1` → `tail -20`).
  4. 근거·설계논의는 `docs/` 에, 소스 docstring 은 얇게.
  5. `HW_SYSID_LITERATURE.md` · `PAPER_NOTES.md` · `HW_SYSID_PLAN.md` · 논문 PDF 는 **열지 않는다**
     (이 작업에 불필요 — 사용자 지시).

---

## 7. PX4 표준 입력경로 (2026-11-22 밤, 사용자 요청 ⑪)

> 사용자 기준: **"실제 실험할 때 지금 시뮬레이션한 코드를 그대로 쓸 수 있는가. 그게 제일 중요해."**
> 참조: 캡스톤 UGV `UGV_Collision_Avoidance_Final/.../coordinate_transformer_node.py`

### 7.1 확정 사실 (실측)

- **기존 다이얼렉트에 표준 메시지가 이미 다 있다.** `usv_tug.xml` 의
  `<include>development.xml</include>` → 생성 클래스 **224 종**에
  `ATTITUDE` · `LOCAL_POSITION_NED` · `GLOBAL_POSITION_INT` ·
  `POSITION_TARGET_LOCAL_NED` · `ATTITUDE_QUATERNION` · `HEARTBEAT` 포함.
  → **PX4 표준 입력경로는 다이얼렉트·마더십·벤더코드 변경이 0 이다.**
  52501 신설은 **필수가 아니라 선택**으로 격하된다 (패치는 산출돼 있다).
- 대응: `ATTITUDE.yaw`→psi · `ATTITUDE.yawspeed`→r (선체고정 z = Fossen r) ·
  `LOCAL_POSITION_NED.x,y`→x,y · `.vx,.vy`(NED)→u,v (2×2 회전) ·
  `POSITION_TARGET_LOCAL_NED.x,y,yaw`→x_ref,y_ref,psi_ref.
- **UGV 예제가 안 주는 2 개를 보충했다**: 각속도 `r` 과 목표자세각 `psi_ref`.
  UGV `/usv/state/*` 4 종에는 둘 다 없다.
- ⚠️ **전송계층이 다르다.** 캡스톤 UGV = uXRCE-DDS(`px4_msgs`), 예인 실험 체인 =
  MAVLink v2 over UDP + DroneBridge. **좌표변환 논리만** 가져오고 전송은 벤더
  `MavUdpLink` 를 그대로 썼다. (벤더 클래스명은 `UdpLink` 가 아니라 **`MavUdpLink`**)

### 7.2 산출물

| 파일 | 내용 |
|---|---|
| `nuc/px4_state.py` | 순수 좌표변환 + `Px4StateAssembler` (전송 의존성 0) |
| `sitl/px4_state_node.py` | `Px4StdStateNode`(MAVLink) · `Px4DdsStateNode`(px4_msgs) |
| `sitl/wire.py::state_px4std` | 마더십 모사 송신 (표준 3 개) |
| `tools/sniff_mothership.py` | 마더십이 실제로 보내는 메시지 계수 (읽기 전용) |
| `tests/test_px4_state.py` | 9 항 — 9/9 PASS |
| `docs/INPUT_PATHS.md` | 두 경로 대조표 · 확인 필요 목록 |
| `sitl/run_sitl.py --state-src {52501,px4std}` | 경로 전환 스위치 |

### 7.3 검증 수치

- **외부입력 9 개가 두 경로에서 동일**: 랜덤 200 점 최대 절대차 **4.44e-16** (기준 1e-9)
- NED↔바디 왕복 1.11e-16 · UGV 3축 역회전과 평면 2×2 차 1.67e-16 ·
  LLA→NED 행렬 원소차 **0** · 100 m 변위 N 100.001 m / E 99.999 m
- 전체 시험 **63 PASS / 0 FAIL**
- SITL `--state-src px4std` 20 에피소드 (zero-shot, `--dr off`, seed 1000–1019):
  **20/20 PASS · 평균 위치오차 0.001 04 m · yaw 0.017 2 deg · `overrun` 0 ·
  최대 틱지연 17.0 ms** → `out/SITL_RUN_px4std.json`
- 52501 경로와 **위치·요 오차가 소수 다섯째 자리까지 동일**. 52501 의 ep 19
  `overrun 50`(29.0 ms) 은 표준경로에서 재현되지 않음 → **Windows 스케줄링 지터**로 확정

### 7.4 확인 필요 (닫지 못한 것)

1. **마더십이 표준 3 개를 실제로 내보내는가** — 미측정. 현장에서
   `python3 tools/sniff_mothership.py --port 14600 --seconds 30` 이 30 초에 답한다.
2. `rc_on_off` · `on_off` 는 표준경로 와이어에 **없다** → 기본값 `0/1` 유지.
   RC 채널 또는 52501 경로가 필요하다.
3. `psi_ref` 가 설정점 메시지에 실리지 않으면 **발행을 멈춘다** (임의값을 만들지 않는다).
   `psi_ref_from_los=True` 로 목표방위각 대체 가능 — 출처를 `psi_ref_src` 에 남긴다.
4. 같은 포트를 두 노드가 bind 할 수 없다 → 기존 52502 경로와 공존은 `forward_addr` 로.
5. **`Px4DdsStateNode` 는 이 환경에서 실행하지 못했다** (Windows·`shim/rclpy`).
   조립식은 MAVLink 경로와 같은 코드라 시험되지만 **구독 배선은 미검증**이다.

---

## 8. 부착 기하 외부화 (2026-11-22 밤, 사용자 요청 ⓬)

### 8.1 확정 사실

- `r1` · `r2` 의 기준좌표계: **폰툰 바디 FRD**. 원점 = 폰툰 **기하중심**
  (`Lx` 2.60 × `By` 2.15 m 판의 가운데), `+x` 선수, `+y` 우현 → **좌현은 음수**,
  회전 + = 위에서 봤을 때 시계방향. 출처 `pontoon_rl/params/sc3.yaml` 의 `attach` 주석
- 가리키는 점은 **터그 선체 원점**이며 전자석 패드 접촉면이 아님
- 기본값 `r1 [ 0.4894, -1.08]` · `r2 [-0.6106, -1.08]` m.
  `y = -1.08` ≈ 반폭 1.075 m, `x` 간격 1.10 m = 터그 전장 (두 대가 맞닿은 배치)
- 와이어(52501 · PX4 표준) **어느 쪽에도 부착점 필드가 없음** → 설정으로만 들어감

### 8.2 산출물

| 파일 | 내용 |
|---|---|
| `config/geometry.yaml` | **현장에서 고치는 유일한 파일** (65 줄, 머리말에 좌표계·재는 방법) |
| `nuc/cfg.py` | 병합 계층 `load_cfg` · `load_geometry` · `geometry_summary` (161 줄) |
| `docs/GEOMETRY.md` | 기준좌표계 · 재는 방법 · 입력 검사 · 확인 필요 (113 줄) |
| `docs/img/diagram_geometry.svg` | 평면도 — 좌표계 · 부착점 · 치수 · yaml 예 |
| `tests/test_geometry_cfg.py` | 9 항 (173 줄) |
| `README.md` §3.4 | 요약 + 되돌리기 방법 |
| `sitl/run_sitl.py --geometry {off,<경로>}` | 전환 스위치 |

### 8.3 검증 수치

- `tests/test_geometry_cfg.py` **9/9 PASS**, 전체 **63 PASS / 0 FAIL**
- 덮어쓰기 끈 값(`--geometry off`) = `sc3.yaml` 값 **동일**
- 덮어쓰기 → QP 기하행렬 `qp.A` 최대차 **< 1e-12** (`body_force_map` 직접 비교)
- 덮어쓰기 → 플랜트 `env.r1/r2` 최대차 **< 1e-12**
  (이걸 안 걸면 기하를 고쳐도 플랜트가 학습값으로 남아 **시험이 거짓양성**이 됨)
- 원본 `sc3.yaml` 사전 **불변** (`copy.deepcopy`)
- 오타(`attatch`·`r3`·`rjit`) · `frame` 변경 · mm 단위 · 길이 1 벡터 · NaN/inf ·
  `r_jit` 범위 밖 → 전부 `ValueError` 로 **거부** (조용히 무시하지 않음)
- SITL 머리줄에 `부착기하 r1=(…) r2=(…) 간격 … [폰툰 바디 FRD, 원점=기하중심]` 를 찍어
  **어느 기하로 돌았는지 로그에 남김**. 결과 json 에 `"geometry"` 항목
- **SITL 종단시험** — 같은 시드(1000) 로 2 회:
  기본 기하 간격 1.100 m → `dpos` **0.00104 m**, 이동 기하(`r1 [0.80,-1.00]` ·
  `r2 [-0.40,-1.00]`) 간격 1.200 m → `dpos` **0.00727 m**. 둘 다 1/1 PASS.
  기하만 바꿨는데 궤적이 바뀌므로 **설정이 실제로 플랜트까지 도달**함이 확인됨.
  단 오차가 7 배 커졌으므로 **기하가 크게 바뀌면 재학습 필요**
  (로그 `out/SITL_geomdef.txt` · `out/SITL_geomshift.txt`)

### 8.4 확인 필요 (닫지 못한 것)

1. **`r1` · `r2` 실측 재확인** — `sc3.yaml` 은 `measured(y)`(실측) 로 표기했으나 같은
   파일의 `dr.r_jit` 주석은 "`r_i` 가 불확실한 것이 RL 을 쓰는 이유"라고 적음. **상충**
2. `y = -1.08` m 는 반폭 1.075 m 보다 **0.005 m 바깥** — 패드 두께인지 반올림인지 미확인
3. 전자석 패드 중심이 터그 선체 중심에서 떨어져 있으면 **보정 필요**
4. 측정 산포가 `dr.r_jit` 0.15 m 를 넘으면 **재학습 필요**
5. 두 터그가 **다른 변**에 붙는 비대칭 배치는 학습 범위 밖 — 미시험


---

## 9. 공개 GitHub 저장소 — 완료 (2026-11-22, 사용자 승인 "공개로 올려도 되어")

### 9.1 확정 사실

| 항목 | 값 |
|---|---|
| 저장소 | <https://github.com/wkyouncnu/pontoon-tug-sitl-harness> |
| 가시성 | **PUBLIC** (`gh repo view --json visibility` 실측) |
| 기본 브랜치 | `main` |
| 최초 커밋 | `50a6be1` (126 파일 · 15 300 줄) |
| 현재 HEAD | `dbb1acc` |
| 라이선스 | **MIT** (`licenseInfo.key = "mit"` 실측 — GitHub 자동인식 성공) |
| 원격 트리 blob 수 | **126** (`git/trees/main?recursive=1` 실측) |
| 계정 | `wkyouncnu`. 커밋 신원은 저장소 로컬 설정 `wkyouncnu@users.noreply.github.com` (개인 이메일 비공개) |

### 9.2 공개 전환으로 새로 판단한 것

- 근거 전문: `docs/REPO_SCOPE.md` §3.2 · §3.2.1 (§3.1 의 비공개 전제를 대체함)
- **포함**: `config/tug_bridge.yaml` 사설대역 주소(RFC 1918) · `out/ppo_sc3_final.onnx`(정책 가중치) ·
  `out/sitl_sc3.mp4`(4.60 MB, 보고서 그림 5) · `docs/manual/vendor/katex/`(MIT, 오프라인 PDF 재생성) ·
  문서가 인용하는 `out/` 로그 10 개
- **제외**: `vendor/tug_bridge/*.py` · `simulink/*.slx`·`*.m` · `mavlink/gen/` · `out/harness_sc3.mp4` ·
  `out/*.bak` · `out/TASKS_prev_*.md` · `docs/manual/_eqsheet.html` — 원격 트리에서 부재 확인함
- 신규 파일: `LICENSE`(MIT) · `NOTICE`(제3자 구성요소) · `.gitattributes`(`*.sh` LF / `*.ps1` CRLF 고정)

### 9.3 확인 필요 (닫지 못한 것)

1. ⚠️ **`mavlink/usv_tug.xml` 의 `52502`·`52503` 정의 재배포 권한 미확인.**
   원본 `JiwoongHa/cooperative-tug-experiment` 는 공개이나 라이선스 파일이 없음
   (`licenseInfo: null`, 실측). `NOTICE` §1 에 출처를 적고 MIT 허락범위에서 제외했으며,
   소유자가 요청하면 즉시 제거함. **소유자 동의는 받지 않았음**
2. ⚠️ **발주처(삼성중공업) 과제 산출물의 공개 허락 여부 미확인.** 사양서·Simulink 모델은
   제외했으나 계약상 공개 제약이 있으면 저장소를 **비공개로 되돌려야 함**
   - **2026-11-23 (사용자 판단)**: 위 미확인을 알고도 **공개 유지**로 결정함.
     계약상 제약이 확인되면 `gh repo edit --visibility private` 로 되돌림
     (이력은 남으므로 그때는 `mavlink/usv_tug.xml`·ONNX 도 다시 판단함)
3. `patch/` 의 52501 패치는 **실험 저장소에 적용하지 않았고 PR 도 올리지 않았음** —
   저장소 소유자 조율 사항임 (사용자 상시 지시)

---

## 10. 2026-11-23 — 실행 순서 문서 3 편 + 전 MD 단일 PDF 통합

### 10.1 새로 만든 문서

| 문서 | 내용 | 줄수 |
|---|---|---|
| `docs/RUN_ORDER.md` | 시뮬레이션 6 단계 · 실물 예인시험 10 단계. 단계별 실행 파일 · 합격 판정 · 실측값 · 건너뛰었을 때의 결과 | 약 200 |
| `docs/FIELD_CHECKLIST.md` | 인쇄용 현장 점검표 A–J. 항목마다 통과 기준 | 약 130 |
| `docs/DEBUG.md` | 계층 6 단 분리 · 계수기 실측 해석 · 시뮬 13 증상 / 현장 14 증상 조치표 · 흔한 실수 5 건 | 약 140 |

- README 첫 장(`## 실행 순서 요약`)에 두 순서를 표로 올림. 라이선스 줄과 구성도 사이
- ⚠️ 스킬 `formal-report-readme` 의 첫 장 고정 순서(제목 → 메타 → 구성도 → 목차)에서
  **의도적으로 벗어남**. 사용자 지시가 "두 실행 순서를 첫 페이지에 강조" 였기 때문임

### 10.2 매뉴얼 PDF — 저장소 전 MD 를 부록으로 바인드

- `docs/make_manual.py` 를 고쳐 부록 목록 `APXS` 12 편을 묶음
  (A `RL_SPEC` · B `RUN_ORDER` · C `FIELD_CHECKLIST` · D `DEBUG` · E `SITL` ·
  F `FILE_SWAP` · G `INPUT_PATHS` · H `GEOMETRY` · I `REPO_SCOPE` ·
  J `patch/README` · K `vendor/README` · L `TASKS`)
- 부록마다 제목 번호를 `A.n` 처럼 바꾸고 상대링크 접두사를 원본 폴더로 교정함
- 구성도 앞에 `.pagebreak` 를 넣어 **PDF 1 쪽이 실행 순서만 담도록** 함

| 항목 | 값 (실측) |
|---|---|
| 부록 | 12 편 |
| 수식 | 102 식 (블록 11 · 인라인 91) |
| `katex-error` | **0** |
| PDF 크기 | 약 3.4 MB (통합 전 1 308 276 B). 부록 L 이 이 파일이므로 재생성마다 조금씩 바뀜 |
| 부록 앵커 `id="apx-a"`–`"apx-l"` | 12 개 전부 존재 |
| `<pre><code>` / `<code>` | 23 / 2 051 |
| 남은 치환자 `ZZ*` | 0 |

### 10.3 생성기에서 고친 결함 3 건 (계수기는 2 차까지 감)

1. **인라인코드·펜스코드가 역따옴표째 글자로 찍혔음.** `restore()` 가 코드 치환자를
   마크다운 원문으로 되돌리는데 그 시점에는 변환이 끝나 있었음 →
   `<code>` · `<pre><code>` 로 되돌리도록 고침 (`fence_html()` 신설).
   **통합 전 PDF 전부에 있던 결함임**
2. **`katex-error` 계수기 오탐 — 두 번 고쳤음.** 본문에 `katex-error` 라는 **글자**가
   있으면 오류로 셌음(새 문서가 합격 기준으로 그 낱말을 적었기 때문) → 1 차로
   `class=` 를 붙여 속성만 셌으나, 이 파일 §10.3 자신이 그 속성을 인라인코드로 적자
   다시 오탐이 났음(`esc()` 는 따옴표를 escape 하지 않으므로 `<code>` 안에 속성이 그대로 남음)
   → 2 차로 **KaTeX 가 실제로 그린 여는 태그**만 셈. `<` 는 `&lt;` 로 escape 되므로
   코드·본문의 글자는 구조적으로 셀 수 없음
3. **수식 점검표가 셸 변수를 식으로 셌음.** 인라인코드 안의 `$VAR$` 를 수식으로 잡아
   103 식이 나왔음 → 점검표에서도 인라인코드를 먼저 지움 (102 식으로 복귀)

### 10.4 확인 필요

1. **PDF 쪽나눔을 기계로 검증하지 못함** — 이 환경에 PyMuPDF 가 없음.
   1 쪽은 PNG 로 눈으로 확인했으나 부록 경계 쪽은 확인하지 못했음
2. 부록 L(`TASKS.md`)은 **이 파일 자신**이므로 PDF 를 다시 뽑을 때마다 내용이 바뀜

### 10.5 전체 6 단계 종단 재현 (문서 통합 후)

| 단계 | 결과 | 시간 |
|---|---|---|
| 1 `dialect` | PASS | 0 s |
| 2 `onnx` | PASS | 3 s |
| 3 `diagrams` | PASS | 0 s |
| 4 `tests` | PASS | 25 s |
| 5 `sitl` | **PASS** | 1 214 s |
| 6 `manual` | PASS (계수기 2 차 교정 후 재실행) | 6 s |

- SITL 실측: **20/20** · 평균 위치오차 **0.001 044 m** · 최대 0.001 044 m ·
  yaw **0.017 23 deg** · `state_src 52501` · `geometry config/geometry.yaml`
- **`overrun_total 0`** — 이전 실행에서 ep 19 에만 났던 `overrun 50`(`worst 29.0 ms`)이
  이번 실행에서는 **나오지 않았음**. 재현되지 않으므로 Windows 스케줄링 지터로 남겨 둠
- `--dr on`(도메인 무작위화) 평가는 **하지 않았음**
- 6 단계만 따로 돌린 재실행 로그: `out/run_all_manual.txt`, 전체 로그 `out/run_all_full.txt`
