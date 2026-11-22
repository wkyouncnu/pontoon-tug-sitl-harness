# 실행 순서 — 시뮬레이션 전체 재현 · 실물 예인시험

- 작성: 충남대학교 자율운항시스템공학과 윤원근 교수
- 대상: `run_all.sh` · `sitl/run_sitl.py` · `tools/sniff_mothership.py` · `config/geometry.yaml`
- 이 문서 하나로 **어떤 파일을 어떤 순서로 실행하는지**가 결정됨. 증상별 조치는 `docs/DEBUG.md`,
  현장 점검 항목은 `docs/FIELD_CHECKLIST.md`

## 1. 두 순서 요약

| 구분 | 첫 명령 | 마지막 산출물 | 실측 소요 |
|---|---|---|---|
| 시뮬레이션 (전체 재현) | `bash run_all.sh` | `out/SITL_RUN.json` · `docs/manual/harness_sitl.pdf` | 1 246 s (6 단계) |
| 실물 예인시험 | `python tools/sniff_mothership.py --port 14600 --seconds 30` | `/tug_command`(52503) 송신 · 비행로그 | 현장 1 일 (조립 포함) |

- 시뮬레이션은 **한 명령으로 끝남**. 단계별로 쪼갤 때만 `--only <단계>` 를 씀
- 실물은 **첫 단계가 측정**임. 마더십이 실제로 무엇을 내보내는지 모른 상태에서
  코드를 올리면 입력 경로 선택이 틀어짐 (`docs/INPUT_PATHS.md`)

## 2. 사전 조건

| 항목 | 값 | 확인 명령 |
|---|---|---|
| 인터프리터 | `../plan_pontoon_rl/.venv/Scripts/python.exe` (Python 3.8.2) | `..\plan_pontoon_rl\.venv\Scripts\python.exe -V` |
| 필수 패키지 | numpy · quadprog · onnxruntime · pymavlink 2.4.50 · pyyaml · markdown · matplotlib | `..\plan_pontoon_rl\.venv\Scripts\python.exe -m pip list` |
| 학습 체크포인트 | `../pontoon_rl/out/n1/ppo_sc3_final.zip` (ONNX 내보내기 입력) | `dir ..\pontoon_rl\out\n1\ppo_sc3_final.zip` |
| 브라우저 | Chrome 또는 Edge (PDF 생성, 헤드리스) | `docs/make_manual.py` 가 자동 탐색 |
| 셸 | `run_all.sh` 는 Git Bash 또는 리눅스 bash | `bash --version` |

- ⚠️ 맨 `python` 은 이 PC 에서 **3.8.2 대화형 REPL** 로 떨어짐. 경로를 명시해야 함
- ⚠️ 콘솔 코드페이지가 cp949 이므로 한글 출력이 깨지면 `-X utf8` 을 붙임
- `pytest` 는 설치되어 있지 않음. 시험은 수작업 러너 `tests/run_tests.py` 로 돌림

## 3. 시뮬레이션 실행 순서

### 3.1 전체 재현 (권장)

```powershell
bash run_all.sh
```

- 6 단계를 순서대로 돌리고 **실패하면 그 자리에서 중단**함
- 단계 로그는 `out/run_all_<단계>.txt`. 실패 시 꼬리 20 줄을 화면에 띄움
- 단계 목록만 보려면 `bash run_all.sh --list`, 한 단계만 돌리려면 `bash run_all.sh --only sitl`

### 3.2 단계별 상세

| 순서 | 단계 | 실행 파일 | 산출물 | 합격 판정 | 실측 |
|---|---|---|---|---|---|
| 1 | `dialect` | `mavlink/gen_dialect.py` | `mavlink/gen/usv_tug.py` | `CRC_EXTRA` 52501 = 119 · 52502 = 123 · 52503 = 16 | 회귀 OK |
| 2 | `onnx` | `policy/export_onnx.py` | `out/ppo_sc3_final.onnx` (78 867 B, opset 17) | sb3 대 ONNX `max|Δ| < 1e-5` | 9.090e-07 |
| 3 | `diagrams` | `docs/img/src/make_diagrams.py` | `docs/img/*.svg` | 4 장 생성 | 8 845 – 18 248 B |
| 4 | `tests` | `tests/run_tests.py` | `out/test_suite.txt` | 0 FAIL | 63 PASS / 0 FAIL |
| 5 | `sitl` | `sitl/run_sitl.py --episodes 20 --speed 1 --dr off --seed0 1000` | `out/SITL_RUN.json` | 20/20 · 평균 ≤ 0.01 m | 20/20 · 0.001 04 m · yaw 0.017 2 deg |
| 6 | `manual` | `docs/make_manual.py` | `docs/manual/harness_sitl.pdf` | `katex-error` 0 | 부록 12 편 · 수식 102 식 · 오류 0 |

- 단계 1 을 건너뛰면 단계 5 의 MAVLink 프레이밍이 **import 는 되지만 파싱이 안 됨**
- 단계 2 를 건너뛰면 단계 5 가 정책 파일을 찾지 못함
- 단계 5 는 **재학습을 하지 않음**. 학습된 정책을 zero-shot 으로 평가함 (20 에피소드 × 60 s)

#### 3.2.1 단계별 개별 명령

```powershell
bash run_all.sh --only dialect
```

```powershell
bash run_all.sh --only onnx
```

```powershell
bash run_all.sh --only diagrams
```

```powershell
bash run_all.sh --only tests
```

```powershell
bash run_all.sh --only sitl
```

```powershell
bash run_all.sh --only manual
```

- bash 없이 PowerShell 에서 직접 돌릴 때는 같은 파일을 인터프리터로 호출함

```powershell
..\plan_pontoon_rl\.venv\Scripts\python.exe sitl\run_sitl.py --episodes 20 --speed 1 --dr off --seed0 1000
```

### 3.3 SITL 러너 옵션

| 옵션 | 기본값 | 용도 |
|---|---|---|
| `--episodes` | 20 | 에피소드 수. 합격판정은 20 |
| `--seed0` | 1000 | 첫 시드. 1000–1019 가 합격판정 시드 |
| `--speed` | 1.0 | 실시간 배속. **1 이 아니면 합격판정이 무효** |
| `--dr` | `off` | 도메인 무작위화. `on` 은 미평가 |
| `--steps` | 0 | 에피소드 길이 단축 (빠른 점검용, 예 50) |
| `--state-src` | `52501` | 입력 경로. `px4std` 는 PX4 표준 3 메시지 경로 |
| `--geometry` | `config/geometry.yaml` | `off` 면 학습 당시 기하, 경로를 주면 그 파일 |
| `--traj` · `--traj-ep` | 없음 · 0 | 궤적 npz 저장 (영상 입력) |
| `--qp-hz` · `--policy-hz` | 50 · 10 | 레이트 분리 변경 (열화 시험) |
| `--no-52502` | 꺼짐 | 마더십 τ 경로를 끊고 상태만 받는 경우 |
| `--onnx` · `--out` | 기본 경로 | 정책 파일 · 결과 json 경로 |

- ⚠️ `--speed 8` 실측: 정책 호출이 `ctl_policy 5/40` 로 줄고 `dpos` 가 0.927 m 로 커짐.
  `summary["pass"]` 는 `speed == 1.0` 일 때만 참으로 둠

### 3.4 선택 실행 (보고·검증 보조)

| 목적 | 명령 | 산출물 |
|---|---|---|
| 빠른 합격판정 (전송계층 없이) | `python tests\run_tests.py --only acceptance` | 20/20 inproc |
| 1 에피소드 연막 시험 | `python sitl\run_sitl.py --episodes 1 --steps 50` | `out/SITL_RUN.json` |
| 궤적 저장 | `python sitl\run_sitl.py --episodes 1 --traj out\traj_sitl.npz` | `out/traj_sitl.npz` |
| 가시화 영상 | `python viz\make_video_sitl.py --traj out\traj_sitl.npz --out out\sitl_sc3.mp4` | `out/sitl_sc3.mp4` |
| PX4 표준 입력 경로 | `python sitl\run_sitl.py --state-src px4std --episodes 20` | `out/SITL_RUN_px4std.json` |
| 기하 교체 확인 | `python sitl\run_sitl.py --episodes 1 --geometry config\geometry_v2.yaml` | 머리줄의 부착기하 |
| 통신 열화 포락선 | `python tools_sweep_comm.py` | `out/SWEEP_COMM.md` |
| 52501 패치 재생성 | `python patch\make_patch.py` | `patch/0001-add-mothership-state-52501.patch` |

## 4. 실물 예인시험 실행 순서

- 전제: 터그 2 대가 폰툰 좌현에 전자석으로 부착된 시나리오 3 배치
- ⚠️ **터그 Pixhawk 는 수신 타임아웃이 없어 마지막 명령값을 유지함.** Wi-Fi 가 끊기면
  τ=0 조차 도착하지 못하므로 **물리적 비상정지 수단이 반드시 있어야 함**

### 4.1 순서표

| 순서 | 하는 일 | 실행 파일 · 대상 | 통과 기준 |
|---|---|---|---|
| 0 | 마더십이 내보내는 메시지 측정 | `tools/sniff_mothership.py` | 어느 입력 경로를 쓸지 결정됨 |
| 1 | 부착점 실측 → 설정 반영 | `config/geometry.yaml` | SITL 머리줄의 간격이 실측과 일치 |
| 2 | 다이얼렉트 생성 (NUC) | `mavlink/gen_dialect.py` | `CRC_EXTRA` 123 · 16 불변 |
| 3 | (52501 경로만) 패치 적용 | `patch/0001-….patch` → 타인 저장소 | `/mothership/state` 토픽 생성 |
| 4 | NUC 에 제어 코드 배치 | `nuc/` · `../pontoon_rl/` · `out/*.onnx` | import 성공, 전송 모듈 의존 0 |
| 5 | 수신 확인 (육상) | `tools/sniff_mothership.py` · 노드 계수기 | `rl_stale` 0 · `rx_parse_err` 0 |
| 6 | 통전 전 소프트 점검 | `rc_on_off = 1` 로 τ=0 확인 | 추력 명령이 전부 0 |
| 7 | 수동 조종으로 기동 확인 | 마더십 기존 LOS·PID 경로 | 터그 2 대가 명령에 반응 |
| 8 | 자율 전환 | `rc_on_off = 0` | 폰툰 위치오차가 줄어듦 |
| 9 | 종료·기록 | τ=0 ×10 송신 후 소켓 종료 | 비행로그 · 결과 json 회수 |

### 4.2 0 단계 — 입력 경로 결정 (가장 먼저)

```powershell
..\plan_pontoon_rl\.venv\Scripts\python.exe tools\sniff_mothership.py --bind 0.0.0.0 --port 14600 --seconds 30 --json out\sniff.json
```

| 측정 결과 | 쓰는 경로 | 추가 작업 |
|---|---|---|
| `ATTITUDE` · `LOCAL_POSITION_NED` · `POSITION_TARGET_LOCAL_NED` 가 보임 | **PX4 표준** (`--state-src px4std`) | 다이얼렉트·Simulink·타인 저장소 변경 **없음** |
| 위 3 개가 없고 `MOTHERSHIP_DATA`(52502) 만 보임 | **52501 신설** | 패치 적용 + Simulink 송신 블록 추가 (소유자 조율) |

- 두 경로 모두 NUC 안에서 `/mothership/state` 라는 **같은 토픽**으로 수렴함
- ⚠️ PX4 표준 경로에는 `rc_on_off` · `on_off` 필드가 **없음**. 수동개입 신호를 다른
  수단으로 받아야 함 (`docs/INPUT_PATHS.md` §1)
- ⚠️ 마더십이 실제로 표준 3 메시지를 내보내는지는 **미확인**. 30 s 측정으로 닫힘

### 4.3 1 단계 — 부착점 측정

- 재는 방법은 `docs/GEOMETRY.md` §3. 폰툰 기하중심 기준, 좌현은 `y` 음수, 소수 2 자리
- 고치는 파일은 `config/geometry.yaml` 한 개뿐이며 제어기 · QP · 플랜트 · 관측이 함께 바뀜

```yaml
frame: pontoon_body_frd
attach:
  r1: [ 0.49, -1.08]
  r2: [-0.61, -1.08]
```

- 반영 확인은 SITL 머리줄 또는 결과 json 의 `"geometry"` 항목
- ⚠️ 기하가 크게 바뀌면 재학습이 필요함. 실측 0.1 m 이동에서 오차가 0.001 04 → 0.007 27 m
  로 7 배 커졌음 (합격기준 0.01 m 안에는 남음)

### 4.4 2–3 단계 — 다이얼렉트와 패치

```powershell
..\plan_pontoon_rl\.venv\Scripts\python.exe mavlink\gen_dialect.py
```

- 52501 경로를 쓸 때만 3 단계를 함. 절차는 `patch/README.md` §4
- ⚠️ **대상 저장소에 push 하지 않음.** 반영은 저장소 소유자 조율 사항이며 하네스는
  패치 파일과 설명만 산출함
- ⚠️ 패치 적용 후 **다이얼렉트 재생성을 빠뜨리면** import 는 되지만 프레임 파싱이 되지 않음

### 4.5 4 단계 — NUC 배치

| 배치 대상 | 경로 | 비고 |
|---|---|---|
| 상위 제어기 | `nuc/controller.py` · `nuc/onnx_policy.py` · `nuc/cfg.py` · `nuc/px4_state.py` | 전송계층 의존 0 |
| 추력배분 QP | `../pontoon_rl/env/latch_qp.py` | 그대로 import |
| 학습 설정 | `../pontoon_rl/params/sc3.yaml` | 읽기전용 |
| 현장 기하 | `config/geometry.yaml` | 현장에서 고치는 유일한 파일 |
| 정책 | `out/ppo_sc3_final.onnx` | ROS 2 파라미터로 경로 주입 |

- `sys.path` 또는 `PYTHONPATH` 에 **`nuc/` 의 부모 디렉터리와 `pontoon_rl/` 두 곳**을 넣음
- `NucController(onnx_path=<ROS 2 파라미터>)` 로 호출함. `DEFAULT_ONNX` 는 하네스 편의값임
- 어댑터는 **숫자를 만들지 않음**. 숫자 꺼내기 → `ctl.policy_step(nu, eta, r1, r2)` →
  `ctl.allocate(tau_d)` → 발행 (`docs/FILE_SWAP.md` §4)

### 4.6 5–6 단계 — 통전 전 점검

| 점검 | 보는 값 | 정상 |
|---|---|---|
| 프레임 파싱 | `rx_parse_err` · `rx_drop_msgid` | 0 |
| 상태 수신 | `rx_state` 가 50 Hz 로 증가 | 1 s 에 약 50 |
| 정책 신선도 | `rl_stale` | 0 |
| 수동개입 경로 | `rc_on_off = 1` 일 때 명령 | τ = 0 |
| 추력 포화 | `n_sat` | 0 또는 설명 가능한 값 |

- 계수기 의미는 `docs/DEBUG.md` §3

### 4.7 7–9 단계 — 수조·해상 시험

1. 수동 조종으로 터그 2 대의 추력 방향·부호를 확인함 (bow +, starboard +, 시계방향 +)
2. `rc_on_off = 0` 으로 자율 전환함. 전환 직후 10 s 간 위치오차 추이를 봄
3. 이상 시 **조종기 개입이 1 순위**이고, 그 다음이 급전 차단임
4. 종료는 τ=0 을 10 회 보낸 뒤 소켓을 닫음 (`tug_tx.py` 워치독과 동일 규약)
5. 비행로그 · NUC 로그 · 결과 json 을 회수함

## 5. 파일별 실행 요약

| 파일 | 언제 실행 | 한 줄 설명 |
|---|---|---|
| `run_all.sh` | 시뮬 전체 재현 | 6 단계 순차 실행, 실패 시 중단 |
| `mavlink/gen_dialect.py` | 시뮬 1 단계 · 실물 2 단계 | MAVLink 다이얼렉트 생성 및 `CRC_EXTRA` 회귀 확인 |
| `policy/export_onnx.py` | 시뮬 2 단계 | sb3 체크포인트 → ONNX, 수치 동등성 확인 |
| `docs/img/src/make_diagrams.py` | 시뮬 3 단계 | 보고서 그림 SVG 생성 |
| `tests/run_tests.py` | 시뮬 4 단계 | 단위 · 동일코드 · 안전 · inproc 합격판정 |
| `sitl/run_sitl.py` | 시뮬 5 단계 | MAVLink over UDP 종단 합격판정 |
| `docs/make_manual.py` | 시뮬 6 단계 | README + 저장소 전 MD(부록 A–L) → A4 PDF |
| `tools/sniff_mothership.py` | 실물 0 · 5 단계 | 마더십 송신 메시지 측정 |
| `patch/make_patch.py` | 실물 3 단계 준비 | 52501 가산 패치 생성 (push 하지 않음) |
| `viz/make_video_sitl.py` | 보고 자료 | 궤적 npz → mp4 |
| `tools_sweep_comm.py` | 열화 검증 | 지연 · 손실 포락선 스윕 |

## 6. 확인 필요 사항

| 항목 | 현재 | 확인 대상 |
|---|---|---|
| 마더십 송신 메시지 | 미측정 | 0 단계 30 s 측정으로 닫힘 |
| 마더십 송신 주기 | 50 Hz 가정 | 실측 필요 |
| `psi` 단위 | rad 가정 | Simulink `psi_time_log` 확인 |
| `x_ref` · `y_ref` | NED 절대 가정 | 상대좌표면 `set_goal` 앞에 변환 필요 |
| `rc_on_off` · `on_off` | 의미 미확인 (Simulink `float32`) | 원작자 확인 |
| 터그 Pixhawk 구성 | 미확인 | Simulink 생성 앱인지 PX4 순정 모듈인지 |
| `--dr on` 평가 | 미실시 | 도메인 무작위화 하에서의 성능 |
| `overrun 50` (ep 19) | 원인 미확인 | Windows 스케줄링 지터로 추정, 위치오차는 전 에피소드 동일 |
