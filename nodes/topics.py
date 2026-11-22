# -*- coding: utf-8 -*-
"""harness_sitl/nodes/topics.py — 토픽 이름 상수 한 곳 모음.

노드는 문자열을 직접 쓰지 않고 여기서 가져온다 → 전송계층 교체 시 이름이 한 군데.

실물 대응
---------
 /plant/state      A→B.  **실물에는 ROS 2 토픽이 아니다** — 마더십 Pixhawk 내부의
                   센서+EKF 경로(uORB)다. 하네스에서만 노드 경계로 쓴다. (확인 필요 아님,
                   의도적 구분)
 /pontoon/state    B→C.  확인 필요 (실물 .msg 미정의, BEST_PLAN §16.1 규약만 존재)
 /pontoon/goal     미션→C. 확인 필요
 /mothership_data  C 내부 (정책→QP). 확정 (tug_interfaces/msg/MothershipData)
 /tug_command      C→D.  확정 (tug_interfaces/msg/TugCommand)
 /allocation_debug C 로깅. 확정 (tug_interfaces/msg/AllocationDebug)
 /tug/thrust       D→A.  실물에는 없다 — 터그 Pixhawk 내부 신호 (확인 필요)

★ S5 추가 — **실물·SITL 전용 이름** (inproc 하네스는 쓰지 않는다)
-----------------------------------------------------------------
 /mothership/state  vendor/mothership_rx → NUC.  MAVLink **52501** 의 ROS 쪽 이름.
                    확정 (우리가 신설한 `mavlink/MothershipState.msg`).
                    위 `/pontoon/state` 의 **실물 대응물**이다 — 하네스에서는
                    딕셔너리 토픽, SITL·실물에서는 이 ROS 토픽.
 /rl/tau_d          rl_policy → latch_qp.  NUC **내부** 토픽. 와이어가 아니다.
                    위 `/mothership_data` 의 실물 대응물이되 **이름을 바꿨다**:
                    `/mothership_data` 는 벤더가 이미 쓰는 이름(마더십 Pixhawk 의
                    τ, 52502)이라 같은 이름을 쓰면 기존 경로와 충돌한다.
                    ⚠️ `.msg` 가 float64 라 **이 경로에는 양자화가 없다**
                    (inproc 쪽 float32 는 실물에 없는 보수적 오차다).
"""
TOPIC_PLANT = "/plant/state"
TOPIC_STATE = "/pontoon/state"
TOPIC_GOAL = "/pontoon/goal"
TOPIC_TAU_D = "/mothership_data"
TOPIC_CMD = "/tug_command"
TOPIC_DBG = "/allocation_debug"
TOPIC_THRUST = "/tug/thrust"

# ★ S5: 실물·SITL 쪽 이름 (shim rclpy 브로커에서 쓰인다)
TOPIC_MSHIP_STATE = "/mothership/state"
TOPIC_MSHIP_DATA = "/mothership/data"     # 벤더 기존 경로 (52502). RL 은 쓰지 않는다
TOPIC_RL_TAU_D = "/rl/tau_d"

ALL = [TOPIC_PLANT, TOPIC_STATE, TOPIC_GOAL, TOPIC_TAU_D,
       TOPIC_CMD, TOPIC_DBG, TOPIC_THRUST]

# SITL·실물에서만 나타나는 이름. `ALL` 과 섞지 않는다 —
# `ALL` 은 inproc 하네스의 토픽집합이고 이쪽은 ROS 2 쪽 집합이다.
ALL_SITL = [TOPIC_MSHIP_STATE, TOPIC_MSHIP_DATA, TOPIC_RL_TAU_D,
            TOPIC_CMD, TOPIC_DBG]
