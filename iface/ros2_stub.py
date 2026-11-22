# -*- coding: utf-8 -*-
"""harness_sitl/iface/ros2_stub.py — ROS 2 / uXRCE-DDS 전송계층 스텁.

★ 상태: **스텁이다.** 이 PC 에 ROS 2 가 설치되어 있지 않다 (확인 필요).
   `rclpy` import 에 성공하면 실제 pub/sub 을 만들고, 실패하면
   `Ros2Unavailable` 를 던진다. **노드 본체는 전혀 고치지 않는다** —
   `InprocTransport` 와 메서드 시그니처가 같다.

왜 스텁이라도 두는가
--------------------
실물 전환 시 "무엇을 어떤 타입·QoS 로 주고받을지"가 미리 고정되어 있어야
전송계층만 갈아끼울 수 있다. 아래 `TOPIC_MAP` 이 그 고정이다.

토픽 ↔ 타입 대응 (근거)
-----------------------
* **확정** (배포된 `nuc_ws/src/tug_interfaces/msg/` 에 실재):
    /mothership_data   → tug_interfaces/msg/MothershipData
    /tug_command       → tug_interfaces/msg/TugCommand
    /allocation_debug  → tug_interfaces/msg/AllocationDebug
* **확인 필요** (BEST_PLAN §16.1 이 규약만 정했고 실물 .msg 가 아직 없다):
    /pontoon/state     → pontoon_interfaces/msg/PontoonState   (신규 정의 필요)
    /pontoon/goal      → pontoon_interfaces/msg/PontoonGoal    (신규 정의 필요)
    /fmu/in/mothership_mode → 커스텀 uORB (BEST_PLAN §5.2 전례 참조)
* **확정** (PX4 ↔ NUC, uXRCE-DDS 표준):
    /fmu/out/vehicle_local_position    px4_msgs/msg/VehicleLocalPosition
    /fmu/out/vehicle_attitude          px4_msgs/msg/VehicleAttitude
    /fmu/out/vehicle_angular_velocity  px4_msgs/msg/VehicleAngularVelocity
    /fmu/out/vehicle_status            px4_msgs/msg/VehicleStatus
"""
from __future__ import annotations

from typing import Any, Dict, Optional

from .messages import QOS, TOPICS


class Ros2Unavailable(RuntimeError):
    pass


# 토픽 → (하네스 dataclass, 실물 ROS 2 타입 문자열, 확정 여부)
TOPIC_MAP: Dict[str, Dict[str, Any]] = {
    "/pontoon/state": dict(
        cls=TOPICS["/pontoon/state"],
        ros2_type="pontoon_interfaces/msg/PontoonState",
        confirmed=False, note="확인 필요 — .msg 신규 정의 필요"),
    "/pontoon/goal": dict(
        cls=TOPICS["/pontoon/goal"],
        ros2_type="pontoon_interfaces/msg/PontoonGoal",
        confirmed=False, note="확인 필요 — .msg 신규 정의 필요"),
    "/mothership_data": dict(
        cls=TOPICS["/mothership_data"],
        ros2_type="tug_interfaces/msg/MothershipData",
        confirmed=True, note="배포본에 실재"),
    "/tug_command": dict(
        cls=TOPICS["/tug_command"],
        ros2_type="tug_interfaces/msg/TugCommand",
        confirmed=True, note="배포본에 실재"),
    "/allocation_debug": dict(
        cls=TOPICS["/allocation_debug"],
        ros2_type="tug_interfaces/msg/AllocationDebug",
        confirmed=True, note="배포본에 실재"),
    "/tug/thrust": dict(
        cls=TOPICS["/tug/thrust"],
        ros2_type="없음 — 터그 Pixhawk 내부 신호",
        confirmed=False, note="확인 필요 — 실물에서는 ULog 로만 남는다"),
    "/fmu/in/mothership_mode": dict(
        cls=TOPICS["/fmu/in/mothership_mode"],
        ros2_type="확인 필요 — 커스텀 uORB",
        confirmed=False, note="BEST_PLAN §5.2 전례(px4_ros2_communication_example.slx)"),
}

# PX4 → NUC 업스트림 (상태 합성에 쓰인다). 전부 표준 px4_msgs 라 확정.
PX4_UPSTREAM = {
    "/fmu/out/vehicle_local_position":   "px4_msgs/msg/VehicleLocalPosition",
    "/fmu/out/vehicle_attitude":         "px4_msgs/msg/VehicleAttitude",
    "/fmu/out/vehicle_angular_velocity": "px4_msgs/msg/VehicleAngularVelocity",
    "/fmu/out/vehicle_status":           "px4_msgs/msg/VehicleStatus",
}

WATCHDOG = dict(cmd_timeout_sec=1.5,      # BEST_PLAN §4.3 safety_node
                input_timeout_s=0.5,      # tug_tx.py 실측 파라미터
                shutdown_zero_count=10)   # tug_tx.py


def available() -> bool:
    try:
        import rclpy  # noqa: F401
        return True
    except Exception:
        return False


def describe() -> str:
    """사람이 읽는 규약 표. 문서·로그에 그대로 싣는다."""
    out = ["ROS 2 전송 규약 (harness_sitl/iface/ros2_stub.py)",
           "%-26s %-44s %-8s %s" % ("topic", "type", "확정", "QoS")]
    for t, d in TOPIC_MAP.items():
        q = QOS.get(t, {})
        out.append("%-26s %-44s %-8s %s/%s/KEEP_LAST(%d)" % (
            t, d["ros2_type"], "O" if d["confirmed"] else "확인필요",
            q.get("reliability", "?"), q.get("durability", "?"), q.get("depth", 1)))
    out.append("")
    out.append("PX4 업스트림 (uXRCE-DDS, BEST_EFFORT/VOLATILE/KEEP_LAST(1)):")
    for t, ty in PX4_UPSTREAM.items():
        out.append("  %-36s %s" % (t, ty))
    out.append("")
    out.append("워치독: %s" % WATCHDOG)
    return "\n".join(out)


class Ros2Transport:
    """`InprocTransport` 와 동일한 인터페이스의 ROS 2 구현 (스텁).

    rclpy 가 없으면 생성 시점에 `Ros2Unavailable` 를 던진다.
    """

    def __init__(self, node_name: str = "harness_sitl", **kw):
        if not available():
            raise Ros2Unavailable(
                "rclpy 를 import 할 수 없다. 이 PC 에 ROS 2 가 없다 (확인 필요).\n"
                "하네스는 transport='inproc' 로 돌려라. "
                "토픽/타입/QoS 규약은 ros2_stub.describe() 에 고정돼 있다.")
        import rclpy
        from rclpy.node import Node
        if not rclpy.ok():
            rclpy.init()
        self._rclpy = rclpy
        self._node = Node(node_name)
        self._pubs: Dict[str, Any] = {}
        self._held: Dict[str, Any] = {}
        self._n: Dict[str, Dict[str, int]] = {}

    # -- 미구현 경고: 실제 .msg 바인딩은 실물 워크스페이스에서 한다 ----------
    def _ros_msg_type(self, topic: str):
        d = TOPIC_MAP.get(topic)
        if d is None:
            raise KeyError("토픽 미등록: %s" % topic)
        if not d["confirmed"]:
            raise Ros2Unavailable(
                "토픽 %s 의 ROS 2 타입이 아직 확정되지 않았다 (%s). "
                "실물 워크스페이스에서 .msg 를 정의한 뒤 이 함수를 채워라."
                % (topic, d["note"]))
        raise Ros2Unavailable(
            "타입 %s 바인딩은 실물 nuc_ws 를 source 한 환경에서만 된다. "
            "이 PC 에서는 inproc 를 쓴다." % d["ros2_type"])

    def publish(self, topic: str, msg: Any) -> None:
        self._ros_msg_type(topic)

    def latest(self, topic: str) -> Optional[Any]:
        return self._held.get(topic)

    def tick(self) -> None:
        self._rclpy.spin_once(self._node, timeout_sec=0.0)

    def stats(self) -> Dict[str, Dict[str, int]]:
        return self._n


if __name__ == "__main__":
    print(describe())
    print("\nrclpy available:", available())
