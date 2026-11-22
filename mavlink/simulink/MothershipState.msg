# simulink/MothershipState.msg — 마더십 Simulink 모델이 52501 로 보낼 필드.
#
# 기존 `simulink/MothershipData.msg` 와 **다른 메시지**다. 그 파일은 손대지 않는다.
# 52502(MOTHERSHIP_DATA)도 그대로 둔다 → 수동제어·기존 배분 경로는 안 깨진다.
#
# ⚠️ 형이 기존 Simulink 메시지와 다르다 (그쪽은 float64/float32 혼용).
#    여기서는 **와이어 형과 같게** float32/uint8 로 쓴다. 와이어가 float32 이므로
#    모델 안에서 double 로 들고 있다가 송신 직전에 single 로 캐스팅하면 된다.
#
# 매핑 — 기존 모델의 로그 신호를 그대로 쓴다 (새로 계산할 것이 없다):
#   x        ← x_time_log          u      ← u_time_log
#   y        ← y_time_log          v      ← v_time_log
#   psi      ← psi_time_log  ⚠️    r      ← r_time_log
#   x_ref    ← x_ref_time_log      y_ref  ← y_ref_time_log
#   psi_ref  ← psi_ref_time_log
#   scenario_mode ← scenario_mode (double → uint8)
#   rc_on_off     ← rc_on_off     (single → uint8)
#   on_off        ← on_off        (single → uint8)
#
# ⚠️ `psi_time_log` 의 단위가 rad 인지 deg 인지 확인되지 않았다. 52501 은 **rad** 로
#    정의했으므로 모델이 deg 를 들고 있으면 송신 직전에 deg→rad 변환이 필요하다.

uint64 timestamp

float32 x
float32 y
float32 psi
float32 u
float32 v
float32 r

float32 x_ref
float32 y_ref
float32 psi_ref

uint8 scenario_mode
uint8 rc_on_off
uint8 on_off
