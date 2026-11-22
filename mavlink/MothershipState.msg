# tug_interfaces/msg/MothershipState.msg
#
# MAVLink MOTHERSHIP_STATE (id 52501) 과 **1:1** 이다. 필드 이름·순서·형이 같다.
#   payload 47 B · format <QfffffffffBBB · CRC_EXTRA 119 (측정값, mavlink/gen_dialect.py)
#
# 왜 float32 인가: 와이어가 float32 다. ROS 쪽을 float64 로 두면 노드가 받은 값이
# 실제보다 정밀해 보여, 양자화가 어디서 일어나는지 코드로 추적할 수 없게 된다.
# (기존 MothershipData.msg 는 와이어가 float32 인데 ROS 가 float64 라 이 혼동이 있다.)
#
# 좌표계: NED 월드 + FRD 바디. bow + / starboard + / 위에서 봤을 때 시계방향 +.
#         Fossen/SNAME 과 동일하므로 부호 반전이 없다.

uint64 timestamp     # 송신자 부팅 후 경과시간 [us]

# 상태 (마더십 CG)
float32 x            # NED north [m]
float32 y            # NED east  [m]
float32 psi          # 방위 [rad]  ⚠️ 확인 필요: Simulink `psi_time_log` 가 rad 인지 deg 인지
float32 u            # 바디 surge [m/s]
float32 v            # 바디 sway  [m/s]
float32 r            # 요레이트 [rad/s]

# 목표자세 — 기존 LOS 유도와 **같은 출처**에서 받는다
float32 x_ref        # [m]  ⚠️ 확인 필요: 절대좌표인지 현재위치 기준 상대값인지
float32 y_ref        # [m]  ⚠️ 동일
float32 psi_ref      # [rad]

# 모드 플래그
uint8 scenario_mode  # 시나리오 선택자
uint8 rc_on_off      # 1 = 수동 RC 개입 중 → NUC 는 RL 출력을 버리고 τ=0 을 낸다
uint8 on_off         # 1 = 마더십 제어기 동작 중
