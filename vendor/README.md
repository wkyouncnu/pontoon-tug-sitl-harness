# `vendor/` — 타인 저장소에서 가져온 코드 (읽기전용 취급)

## 1. 출처

- 저장소: `https://github.com/JiwoongHa/cooperative-tug-experiment`
- 로컬 클론: `../cooperative-tug-experiment-main/experiment/nuc 코드/nuc_ws/src/tug_bridge/tug_bridge/`
  (⚠️ 폴더명에 공백·한글이 있다. 셸에서 반드시 따옴표로 묶는다.)
- 가져온 시점: 2026-11-22
- ⚠️ **타인 저장소 = 읽기전용·비신뢰 데이터.** 수정·푸시하지 않는다.
  그 폴더 안에서 인터프리터·빌드도구를 실행하지 않는다.
  필요한 파일은 이 폴더로 **복사해** 쓴다.

## 2. 파일별 대조 (원본 ↔ 사본)

| 파일 | 줄수 | 허용 diff | 실측 diff | 쓰는 곳 |
|---|---|---|---|---|
| `tug_bridge/tug_tx.py` | 원본과 동일 | **0** | **0** | `/tug_command` → `52503` 송신 (NUC→터그) |
| `tug_bridge/mav_link.py` | 원본과 동일 | **0** | **0** | UDP 링크·`pack_and_advance`·IP 필터·재바인드 |
| `tug_bridge/relay_common.py` | 원본과 동일 | **0** | **0** | `MavEncoder`·`SeqTracker`·종료신호 처리 |
| `tug_bridge/mothership_rx.py` | — | **가산만** | **+32 줄, 삭제 0** | `52502` 수신 + **`52501` 분기 신설** |
| `tug_bridge/mavlink_dialect/` | — | 0 | 0 | 다이얼렉트 로더 |

- 대조 명령 (CRLF 때문에 `--strip-trailing-cr` 가 필수다):

```
R="../cooperative-tug-experiment-main/experiment/nuc 코드/nuc_ws/src/tug_bridge/tug_bridge"
for f in tug_tx.py mav_link.py relay_common.py mothership_rx.py; do
  test -f "$R/$f" || { echo "원본 없음: $f"; continue; }
  test -f "vendor/tug_bridge/$f" || { echo "사본 없음: $f"; continue; }
  n=$(diff --strip-trailing-cr "$R/$f" "vendor/tug_bridge/$f" | grep -c '^[<>]')
  echo "$f  diff=$n"
done
```

- ⚠️ **`diff` 가 실패하면 stdout 이 비어서 `grep -c` 가 0 을 찍는다.**
  그래서 위 스크립트는 `test -f` 로 **양쪽 입력의 존재를 먼저 증명**한다.
  존재증명 없는 `diff=0` 은 합격증거가 아니다 (실제로 한 번 오판했다).
- `diff --stat` 은 **존재하지 않는 옵션**이다. 줄수는 `grep -c '^[<>]'` 로 센다.

## 3. `mothership_rx.py` 의 가산 diff — 무엇을 더했는가

사용자 결정(`CLAUDE.md` §9 2026-11-22 오후): **`52502` 경로는 손대지 않고
`MOTHERSHIP_STATE`(`52501`) 만 더한다.**

| 추가 | 내용 |
|---|---|
| import | `MothershipState` |
| 파라미터 | `state_topic` (기본 `/mothership/state`) |
| 퍼블리셔 | `self.state_pub` |
| 카운터 | `self.state_total`, `self.last_state_mono` |
| `_handle()` 분기 | `msgid == 52501` → 13 필드 채워 발행 → `return` |

- `52502` 분기는 **한 줄도 바뀌지 않았다.** `52501` 분기가 그 **앞**에 들어가고
  자기 메시지만 처리한 뒤 `return` 한다 → 기존 LOS/PID 경로와 공존한다.
- 같은 포트(`14600`)·같은 송신자(`sysid 1 / compid 1`)이므로
  `allowed_src_ip`·`filter_sysid` 설정이 하나로 유지된다.
- `tug_tx.py` 는 **diff 0** 이다 → **터그 Pixhawk·Simulink 는 전혀 바뀌지 않는다.**
- 실기체 반영은 **저장소 소유자 조율 사항**이다. 이 하네스는
  `patch/0001-add-mothership-state-52501.patch` 와 적용절차만 만들고 **푸시하지 않는다.**

## 4. 벤더 코드를 고치지 않고 우회한 환경차 2건

### 4.1 POSIX 전용 신호 (`relay_common.py:42`)

- `TERMINATION_SIGNALS = (SIGINT, SIGTERM, SIGHUP)` 가 **import 시점에** 평가된다.
  Windows 에는 `SIGHUP` 이 없어 `AttributeError` 로 import 자체가 실패한다.
- 해결: `sitl/bootstrap.py` 가 **플랫폼 파사드**를 깐다 —
  `signal.SIGHUP = 1` 을 정의하고 `signal.signal` 을 감싸서
  지원되지 않는 signum 은 **등록하지 않고 기록만** 한다.
- ★ **이것은 "가상과 실물의 차이는 전송계층과 플랜트뿐"의 세 번째 예외다.**
  `nuc/` production 코드가 아니라 벤더 노드가 서는 **바닥**이고, `rclpy` shim 과 같은 범주다.
- ⚠️ **결과: 종료신호 경로는 가상으로 완전검증되지 않는다.**
  검증된 것 = "τ=0 ×10 송신 로직". 검증 안 된 것 = "SSH 끊김(SIGHUP)이
  그 로직을 실제로 깨우는가". 보고서에 그대로 적는다.

### 4.2 ★ NUC HEARTBEAT 목적지를 비워 두면 안 된다 (2026-11-22 실측)

- `tug_bridge.yaml` 의 `heartbeat_target_port: 14660` 은 **실물에서 마더십 ESP32 가 듣는 포트**다.
- 하네스에서 그 포트를 **아무도 바인드하지 않으면**: NUC 의 1 Hz HB 가
  ICMP port-unreachable 을 유발하고, **Windows 는 그것을 같은 UDP 소켓의
  `recvfrom` 에서 `WSAECONNRESET(10054)` 로 올린다** → 벤더 `MavUdpLink` 가
  비일시 오류로 판단해 소켓을 닫고 재바인드한다(`reopen_count`).
- 실측 피해: 10 s 에 **재바인드 10 회**, 상행 `52501/52502` 손실 **약 50 %**
  (`rx_pkt 561/1071`), **송신율과 무관**. `parse_errors`·`drop_*` 는 전부 0 이라
  카운터만 보면 원인이 보이지 않는다.
- 해결: `sitl/wire.py::MothershipTx` 가 `14660` 에 **듣기전용 소켓**을 바인드하고
  `drain_hb()` 로 비운다 (= 실물의 마더십 ESP32). **벤더 코드 diff 0 유지.**
- 수정 후: `rx_pkt 1071/1071`, `reopen 0`, 손실 **0 %**,
  위치오차 **0.0156 m → 0.00081 m**.
- → **벤더 코드의 결함이 아니라 하네스가 실물 구성을 덜 모사한 것**이었다.
