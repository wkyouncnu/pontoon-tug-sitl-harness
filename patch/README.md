# MOTHERSHIP_STATE (52501) 반영 패치 — 적용 절차

- 대상 저장소: `JiwoongHa/cooperative-tug-experiment` (**타인 소유**)
- 패치 파일: [`0001-add-mothership-state-52501.patch`](0001-add-mothership-state-52501.patch) (9 754 B)
- 생성기: [`make_patch.py`](make_patch.py) — 재생성 가능
- ⚠️ **본 작업은 패치 파일만 생성함. 적용·커밋·push 를 하지 않음.**
  실기체 반영은 **저장소 소유자 조율 사항**임

## 1. 목적

- NUC 에서 도는 RL 제어기가 관측 19 차를 만들기 위해 마더십 상태(`x, y, psi, u, v, r`)와
  목표자세(`x_ref, y_ref, psi_ref`)를 받아야 함
- 현재 와이어에는 `MOTHERSHIP_DATA`(52502)의 **τ 3 필드만** 흐름 → 상태를 받을 경로가 없음
- 해법: **새 메시지 `MOTHERSHIP_STATE`(52501) 신설.** 52502 는 그대로 둠
  → 마더십 수동제어·기존 배분 경로가 깨지지 않고 RL 경로만 추가됨

## 2. 패치 내용

| # | 파일 | 변경 | 내용 |
|---|---|---|---|
| ① | `nuc 코드/nuc_ws/src/tug_bridge/mavlink/usv_tug.xml` | +16 / −0 | 52501 `<message>` 블록을 52502 **앞**에 삽입 (id 오름차순) |
| ② | `nuc 코드/nuc_ws/src/tug_bridge/tug_bridge/mothership_rx.py` | +32 / −0 | 52501 분기 + `/mothership/state` 퍼블리셔 추가 |
| ③ | `nuc 코드/nuc_ws/src/tug_interfaces/msg/MothershipState.msg` | 신규 +31 | ROS 메시지 정의 |
| ④ | `nuc 코드/nuc_ws/src/tug_interfaces/CMakeLists.txt` | +1 / −0 | `"msg/MothershipState.msg"` 등록 |
| ⑤ | `simulink/MothershipState.msg` | 신규 +38 | Simulink 쪽 정의 (주석에 송신 필드 대응 명시) |
| | **합계** | **+118 / −0** | **삭제·수정 0 줄 — 전 파일 가산 전용** |

- ④ 는 당초 설계에 없었음. `CMakeLists.txt` 에 등록하지 않으면 `tug_interfaces` 패키지가
  새 메시지를 빌드하지 않아 ② 의 `from tug_interfaces.msg import MothershipState` 가 실패함

## 3. 바꾸지 않는 것

| 대상 | 이유 |
|---|---|
| `MOTHERSHIP_DATA`(52502) · `TUG_COMMAND`(52503) 정의 | 한 글자도 안 바꿈. `CRC_EXTRA` 불변 (§5 표) |
| `tug_tx.py` · `mav_link.py` · `relay_common.py` | diff **0**. 52501 메시지 id 상수는 `mothership_rx.py` 안에서 `getattr` 로 받음 |
| 터그 Pixhawk · 터그 Simulink | 터그는 `52503` 만 수신함 → **영향 없음** |
| `<version>3</version>` | 올리지 않음. 메시지 추가는 하위호환이고 `<version>` 은 정보성 필드임 |
| `mothership_rx.py` 의 52502 처리 경로 | 분기를 앞에 끼워넣기만 함. 기존 분기·로그·통계는 그대로임 |

## 4. 적용 절차

### 4.1 패치 적용

```powershell
cd <cooperative-tug-experiment 클론 루트>/experiment
git apply --check "<harness_sitl>/patch/0001-add-mothership-state-52501.patch"
```

```powershell
git apply "<harness_sitl>/patch/0001-add-mothership-state-52501.patch"
```

- 경로에 공백과 한글(`nuc 코드`)이 있으나 `git apply` 가 정상 처리함 (§6 에서 실측)
- `git apply` 가 거부되면 `patch -p1 --binary < …` 를 쓰지 말고 **원본 줄끝이 LF 인지 먼저 확인**할 것

### 4.2 다이얼렉트 재생성 — 이 순서를 지킬 것

| 순서 | 명령 | 확인 |
|---|---|---|
| 1 | 패치 적용 (§4.1) | `usv_tug.xml` 에 52501 블록 1 개 |
| 2 | `python -m pymavlink.tools.mavgen --lang=Python --wire-protocol=2.0 --output=<gen> usv_tug.xml` | 생성 성공 |
| 3 | `MAVLINK_MSG_ID_MOTHERSHIP_STATE == 52501` 확인 | §5 표의 `crc_extra` 3 개 일치 |
| 4 | `colcon build --packages-select tug_interfaces tug_bridge` | ④ 등록이 반영됨 |
| 5 | `source install/setup.bash` → 노드 기동 | `/mothership/state` 토픽 생성 |

- 본 하네스의 동일 작업은 `mavlink/gen_dialect.py` (또는 `gen_dialect.sh`) 가 수행함
- **2 단계를 빼먹으면** ② 의 `getattr(mavlink, 'MAVLINK_MSG_ID_MOTHERSHIP_STATE', 52501)` 가
  폴백 52501 을 쓰므로 import 는 되지만 **프레임 파싱이 되지 않아 분기를 타지 못함**

### 4.3 마더십 Simulink

- `MOTHERSHIP_STATE` 송신 블록을 추가함. 송신자는 기존 `MOTHERSHIP_DATA` 와 **동일**
  (`sysid 1` / `compid 1`, 목적 포트 `14600`) → NUC 파서·`allowed_src_ip` 를 하나로 유지함
- 필드 대응은 ⑤ `simulink/MothershipState.msg` 주석에 적음
- 송신 주기는 52502 와 동일한 **50 Hz** 를 전제함

## 5. CRC_EXTRA (실측)

| 메시지 | id | 필드 수 | `crc_extra` | 비고 |
|---|---|---|---|---|
| `MOTHERSHIP_STATE` | 52501 | 13 | **119** | 신설 |
| `MOTHERSHIP_DATA` | 52502 | 4 | **123** | 패치 전후 **동일** |
| `TUG_COMMAND` | 52503 | 7 | **16** | 패치 전후 **동일** |

- 측정: `mavlink/gen/usv_tug.py` 의 `mavlink_map` 직접 조회
- 52502/52503 의 `crc_extra` 가 불변이므로 **구 펌웨어와 신 다이얼렉트가 혼재해도 기존 두 메시지는 정상 통신됨**

## 6. 생성·검증 방법 (실측)

| 항목 | 방법 | 결과 |
|---|---|---|
| 가산 전용 | `make_patch.py` 가 삭제 줄 수를 검사하고 0 이 아니면 실패 반환 | **+118 / −0** |
| 적용 가능성 | 원본 3 파일을 **격리 사본**에 복사해 git 저장소를 만들고 `git apply` | **성공, 경고 0** |
| 적용 결과 | 적용본과 하네스 사본을 바이트 비교 | `mothership_rx.py` · `MothershipState.msg` ×2 **동일** |
| 줄끝 | 적용본 5 파일 전부 | **LF** (대상 저장소 관례와 일치) |
| 다이얼렉트 등가 | 적용본 `usv_tug.xml` 과 하네스 `mavlink/usv_tug.xml` 을 주석 제외 비교 | **동일** (빈 줄 1 개 차이뿐) |

- 격리 사본 위치는 세션 스크래치패드임. **대상 저장소에는 쓰지 않았음**
- 재현: `..\plan_pontoon_rl\.venv\Scripts\python.exe patch\make_patch.py`

## 7. 영향 범위

| 구성요소 | 변경 | 근거 |
|---|---|---|
| 마더십 Simulink | **있음** — 52501 송신 블록 추가 | §4.3 |
| 마더십 ESP32 (DroneBridge) | **없음** — 투명 UDP 브리지임. 단 상행 대역이 증가함 | 52501 payload 47 B @ 50 Hz ≈ 2.4 kB/s 추가 |
| NUC `tug_interfaces` | **있음** — 메시지 1 종 추가 후 재빌드 | ③ ④ |
| NUC `mothership_rx` | **있음** — 가산 분기 | ② |
| NUC `tug_tx` | **없음** (diff 0) | §3 |
| 터그 ESP32 ×2 | **없음** | 하행 경로 불변 |
| 터그 Pixhawk ×2 · 터그 Simulink | **없음** | `52503` 만 수신함 |

## 8. 확인 필요 사항

| 항목 | 현재 전제 | 확인 대상 |
|---|---|---|
| `psi` 단위 | **rad** 로 가정함 | Simulink `psi_time_log` 가 rad 인지 deg 인지 |
| `rc_on_off` · `on_off` | `uint8`, 1 = 활성으로 가정함 | Simulink 는 `float32` 임 → 정확한 의미·값 |
| `x_ref` · `y_ref` | **절대 NED 좌표**로 가정함 | 상대 오프셋일 가능성 |
| 송신 주기 | 52502 와 동일한 50 Hz 로 가정함 | 마더십 모델의 실제 송신 레이트 |
| 터그 Pixhawk 구성 | 본 패치와 무관하나 미확정 | Simulink 생성 앱인지 PX4 순정 모듈인지 |

- 위 5 항은 **추정**임. 실기체 적용 전 확인이 필요하며, 틀리면 ⑤ 의 필드 대응을 고쳐야 함
