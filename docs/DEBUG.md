# 디버그 — 계층 분리, 계수기 해석, 증상별 조치

- 작성: 충남대학교 자율운항시스템공학과 윤원근 교수
- 대상: `out/SITL_RUN.json` 계수기 · `tools/sniff_mothership.py` · `tests/run_tests.py`
- 실행 순서는 `docs/RUN_ORDER.md`, 현장 점검 항목은 `docs/FIELD_CHECKLIST.md`

## 1. 원칙 — 아래에서 위로 한 계층씩 끊는다

| 순서 | 계층 | 끊는 질문 | 보는 값 |
|---|---|---|---|
| 1 | 와이어 | 바이트가 오는가 | `rx_pkt` · `tools/sniff_mothership.py` |
| 2 | 파싱 | 프레임이 해석되는가 | `rx_parse_err` · `rx_drop_msgid` · `rx_drop_sysid` |
| 3 | 상태 | 제어기가 상태를 받는가 | `rx_state` · `rl_state` · `rl_stale` |
| 4 | 정책 | 정책이 도는가 | `ctl_policy` · `rl_pub` · `ctl_override` |
| 5 | 배분 | QP 가 푸는가 | `ctl_qp` · `qp_pub` · `qp_stale` |
| 6 | 추력 | 터그가 받는가 | `tug1_rx` · `tug2_rx` · `n_sat` |

- **위 계층부터 의심하지 않음.** 정책 출력이 이상할 때 가장 흔한 원인은 1–3 계층임
- 계층을 끊을 때는 **한 번에 하나만** 바꿈. 두 개를 같이 바꾸면 어느 쪽이 원인인지 못 가림

## 2. 계층별 격리 명령

| 끊고 싶은 것 | 명령 | 통과하면 |
|---|---|---|
| 전송계층 전체 | `python tests\run_tests.py --only acceptance` | 제어 논리는 정상. 문제는 전송계층 |
| MAVLink 프레이밍 | `python tests\run_tests.py --only wire_roundtrip` | 직렬화·역직렬화 정상 |
| 하네스 대 실물 동일코드 | `python tests\run_tests.py --only same_code_both_paths` | `nuc/` 가 전송 모듈을 안 씀 |
| 정책 수치 | `python tests\run_tests.py --only onnx_parity` | ONNX 가 sb3 와 같음 |
| 안전 경로 | `python tests\run_tests.py --only safety` | τ=0 전이 정상 |
| 입력 경로 | `python tests\run_tests.py --only px4_state` | PX4 표준 변환 정상 |
| 기하 설정 | `python tests\run_tests.py --only geometry_cfg` | `geometry.yaml` 이 끝까지 도달 |
| 짧은 종단 | `python sitl\run_sitl.py --episodes 1 --steps 50` | 전체 경로가 살아 있음 |
| 마더십 입력 | `python tools\sniff_mothership.py --port 14600 --seconds 30` | 와이어에 무엇이 오는지 |

- 시험 목록은 `python tests\run_tests.py` (인자 없이) 로 전체 실행함

## 3. 계수기 해석표

### 3.1 NUC 쪽 (`nuc_last`)

| 계수기 | 뜻 | 정상 (20 에피소드 실측) | 비정상의 뜻 |
|---|---|---|---|
| `rx_pkt` | 수신 패킷 수 | 122 409 | 0 이면 와이어 단절 |
| `rx_reopen` | 소켓 재개방 | 0 | 0 보다 크면 포트 충돌·단절 |
| `rx_dropped_ip` | 허용 외 IP 로 버린 수 | 0 | 0 보다 크면 `allowed_src_ip` 설정 불일치 |
| `rx_parse_err` | 프레임 파싱 실패 | 0 | **다이얼렉트 불일치** (가장 흔함) |
| `rx_drop_msgid` | 모르는 메시지 id | 0 | 다이얼렉트에 메시지가 없음 |
| `rx_drop_sysid` | 허용 외 sysid | 0 | 마더십 sysid 설정 불일치 |
| `rx_state` | 상태 메시지 수신 | 60 600 | 0 이면 52501·PX4 표준 어느 쪽도 안 옴 |
| `rx_data` | τ 메시지(52502) 수신 | 60 600 | 0 이면 마더십 배분 경로 단절 |
| `rl_state` | 정책 노드가 본 상태 | 60 599 | `rx_state` 와 크게 다르면 토픽 단절 |
| `rl_pub` | 정책 발행 수 | 12 120 | `rl_state / 5` 근처 (10 Hz) |
| `rl_stale` | 오래된 상태로 돈 횟수 | 0 | 0 보다 크면 상태 수신 주기 부족 |
| `rl_override` | 수동개입으로 τ=0 | 113 | 자율 구간에서 증가하면 `rc_on_off` 오염 |
| `qp_tau` | QP 가 받은 τ_d | 12 120 | `rl_pub` 와 같아야 함 |
| `qp_pub` | QP 발행 수 | 60 596 | 50 Hz |
| `qp_stale` | 오래된 τ_d 로 푼 횟수 | 0 | 0 보다 크면 정책 노드 지연 |
| `ctl_policy` | 제어기 정책 호출 | 12 007 | 10 Hz × 시간 |
| `ctl_qp` | 제어기 QP 호출 | 60 596 | 50 Hz × 시간 |
| `ctl_override` | 제어기 τ=0 반환 | 113 | 리셋 직후 구간 |

### 3.2 플랜트·터그 쪽 (`plant_last`)

| 계수기 | 뜻 | 정상 | 비정상의 뜻 |
|---|---|---|---|
| `k` | 적분 틱 | 3 000 (= 600 × 5) | 적으면 조기 종료 |
| `cmd_rx` | 명령 수신 | 121 232 | 0 이면 하행 단절 |
| `tug1_rx` · `tug2_rx` | 터그별 수신 | 60 616 각 | 한쪽만 0 이면 그 ESP32 경로 문제 |
| `tug1_drop_sysid` · `tug2_drop_sysid` | sysid 불일치 | 0 | `TUG_ACCEPTED_SYSID` 설정 확인 |
| `tx_state` · `tx_data` | 상행 송신 | 60 600 각 | 0 이면 마더십 송신 정지 |
| `tx_hb` · `hb_rx` | 하트비트 | 1 209 · 1 212 | 정지하면 링크 감시 불능 |
| `tx_err` | 송신 오류 | 0 | 0 보다 크면 소켓·주소 문제 |
| `n_sat` | 추력 포화 | 0 | 많으면 과대 명령 또는 기하 불일치 |

### 3.3 성능 계수기

| 계수기 | 뜻 | 실측 |
|---|---|---|
| `overrun` | 틱 마감 초과 횟수 | 에피소드 0–18 은 0, **ep 19 (seed 1019) 만 50** |
| `worst_lag_ms` | 최대 지연 | 17.0 ms (ep 0–18) · **29.0 ms (ep 19)** |
| `perf_valid` | 성능 측정 유효 | `true` (`--speed 1` 일 때만) |

- ⚠️ ep 19 의 `overrun 50` 은 **원인 미확인**. 위치오차가 전 에피소드에서 동일하므로
  제어 성능이 아니라 Windows 스케줄링 지터로 추정함
- `--speed > 1` 로 돌리면 `overrun` 이 커지고 `perf_valid` 가 거짓이 됨. 성능 판정에 쓰지 않음

## 4. 증상별 조치 — 시뮬레이션

| 증상 | 원인 | 조치 |
|---|---|---|
| `python` 이 대화형 REPL 로 떨어짐 | 시스템 Python 3.8.2 | `..\plan_pontoon_rl\.venv\Scripts\python.exe` 명시 |
| 한글 출력이 깨짐 | 콘솔 cp949 | `-X utf8` 추가 또는 `_console.py::utf8_stdout()` |
| `ModuleNotFoundError: env` | `pontoon_rl` 경로 누락 | `PYTHONPATH` 에 `../pontoon_rl` 추가 |
| `FileNotFoundError: ppo_sc3_final.onnx` | 2 단계 미실행 | `bash run_all.sh --only onnx` |
| `rx_parse_err` 가 증가 | 1 단계 미실행 (다이얼렉트) | `bash run_all.sh --only dialect` |
| `CRC_EXTRA` 불일치 | 다이얼렉트 정의 변경 | 52502 = 123 · 52503 = 16 회귀 확인 |
| SITL 위치오차가 큼 | 기하 변경 또는 `--speed > 1` | `--geometry off` 로 학습 기하 복귀, `--speed 1` |
| `ctl_policy` 가 비정상적으로 적음 | 배속 실행 | `--speed 1` 로 돌림 |
| `summary["pass"]` 가 거짓 | 20 에피소드 미만 또는 배속 | 기본 인자로 재실행 |
| PDF 수식이 글자로 나옴 | KaTeX 로드 실패 | `docs/manual/vendor/katex/` 존재 확인 |
| PDF 생성이 실패 | 브라우저 미탐색 | Chrome 또는 Edge 설치 경로 확인 |
| `quadprog` import 실패 | 패키지 미설치 | venv 에 설치 |
| 시험이 통과하는데 SITL 만 실패 | 전송계층 문제 | §2 의 `wire_roundtrip` · `sitl_parity` 로 좁힘 |

## 5. 증상별 조치 — 실물

| 증상 | 원인 | 조치 |
|---|---|---|
| `rx_pkt` = 0 | Wi-Fi 단절 · 포트 불일치 | ESP32 LED, bind 포트 14600, 방화벽 확인 |
| `rx_dropped_ip` 증가 | 송신 IP 가 허용목록 밖 | `allowed_src_ip` 를 실측 주소로 수정 |
| `rx_parse_err` 증가 | 다이얼렉트 불일치 | NUC 에서 다이얼렉트 재생성 |
| `rx_drop_msgid` 증가 | 52501 정의 누락 | 패치 적용 후 **다이얼렉트 재생성** |
| `rx_state` = 0, `rx_data` > 0 | 상태 메시지 경로 없음 | 입력 경로 재결정 (`--state-src px4std`) |
| `rl_stale` 증가 | 상태 수신 주기 부족 | 마더십 송신 주기 실측, Wi-Fi 세기 확인 |
| `rl_override` 가 자율 구간에 증가 | `rc_on_off` 값 오염 | 조종기 스위치 상태, 필드 의미 확인 |
| 터그가 명령에 반응하지 않음 | 하행 단절 · sysid 불일치 | `tug1_rx`·`tug2_rx`, `TUG_ACCEPTED_SYSID 250` |
| 한쪽 터그만 반응 | 해당 ESP32 경로 | 포트 14601 / 14602 와 IP 대응 확인 |
| 터그가 마지막 명령을 계속 유지 | **Pixhawk 수신 타임아웃 없음** | 조종기 개입 → 자율 해제 → 급전 차단 |
| 전자석이 이탈 | 전단력 초과 | 추력 상한 하향, 부착 간격 재확인 |
| 폰툰이 목표에서 멀어짐 | 목표좌표 성질 오해 | `x_ref`·`y_ref` 절대/상대 확인 |
| yaw 가 반대로 수렴 | `psi` 단위 또는 부호 | rad/deg 확인, FRD 시계방향 + 확인 |
| 위치오차가 시뮬보다 훨씬 큼 | 기하 불일치 · 모델 오차 | `config/geometry.yaml` 실측 반영, 재학습 검토 |
| `n_sat` 가 큼 | 과대 명령 · 기하 불일치 | 추력 한계, 부착점 재측정 |

## 6. 보는 파일

| 파일 | 내용 |
|---|---|
| `out/run_all_<단계>.txt` | 단계별 실행 로그 (실패 시 꼬리 20 줄이 화면에도 나옴) |
| `out/SITL_RUN.json` | 합격판정 · 에피소드별 결과 · 계수기 최종값 |
| `out/test_suite.txt` | 시험별 PASS/FAIL |
| `out/sniff.json` | 마더십 송신 측정 결과 |
| `docs/manual/check/dom.txt` | PDF 생성 시 KaTeX 오류 검사 |
| `docs/manual/check/*.png` | PDF 첫 장 · 수식 점검표 렌더 |

## 7. 자주 틀리는 것

- **패치만 적용하고 다이얼렉트를 재생성하지 않음** → import 는 되지만 프레임 파싱이 안 됨
- **`--speed` 를 올려 놓고 성능을 판정함** → `perf_valid` 가 거짓이 되고 오차가 커짐
- **기하를 바꾸고 재학습 영향을 보지 않음** → 0.1 m 이동에서 오차가 7 배 커진 실측이 있음
- **포화율만 보고 위험하다고 판단함** → 한계를 `1e6` 으로 풀어도 성능이 같았음.
  경계에 닿는 행과 성능을 결정하는 행은 다름 (`pontoon_rl/out/n1/SWEEP_LATCH.md`)
- **`pontoon_rl/` 을 고침** → 하네스는 import 만 함. 고치면 학습 재현성이 깨짐

## 8. 확인 필요 사항

| 항목 | 현재 | 확인 대상 |
|---|---|---|
| ep 19 `overrun 50` | 원인 미확인 | Windows 스케줄링 지터 추정, 리눅스에서 재측정 |
| `--dr on` 하의 계수기 거동 | 미측정 | 도메인 무작위화 평가 |
| 실물 지연·손실 포락선 | 시뮬 스윕만 있음 | 현장 Wi-Fi 에서 재측정 |
| `rc_on_off` · `on_off` 의미 | 미확인 | 원작자 확인 |
| PX4 표준 경로의 수동개입 신호 | 필드 없음 | 대체 수단 결정 필요 |
