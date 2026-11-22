# -*- coding: utf-8 -*-
"""`rclpy.qos` 대역.

`relay_common.make_qos()` 가 쓰는 것만 있다. ⚠️ 프로세스 안이라 `reliable` 과
`best_effort` 는 **동작이 같다** — 손실·지연 주입은 와이어에서 한다
(`tools_sweep_comm.py`).
"""
from __future__ import annotations


class HistoryPolicy(object):
    KEEP_LAST = 1
    KEEP_ALL = 2
    SYSTEM_DEFAULT = 0


class ReliabilityPolicy(object):
    RELIABLE = 1
    BEST_EFFORT = 2
    SYSTEM_DEFAULT = 0


class DurabilityPolicy(object):
    VOLATILE = 2
    TRANSIENT_LOCAL = 1


class QoSProfile(object):
    def __init__(self, *, history=HistoryPolicy.KEEP_LAST, depth=10,
                 reliability=ReliabilityPolicy.RELIABLE, durability=None, **kw):
        self.history = history
        self.depth = int(depth)
        self.reliability = reliability
        self.durability = durability

    def __repr__(self):
        return "QoSProfile(depth=%d, reliability=%r)" % (self.depth, self.reliability)
