# -*- coding: utf-8 -*-
"""`rclpy.executors` 대역. 벤더 노드는 예외 클래스만 쓴다."""
from __future__ import annotations

from rclpy import spin, spin_once  # noqa: F401


class ExternalShutdownException(Exception):
    """`rclpy.shutdown()` 이 외부에서 불렸을 때 `spin` 이 던지는 예외."""


class ShutdownException(Exception):
    pass


class SingleThreadedExecutor(object):
    """참고용. 하네스는 노드별 실행기(`rclpy._Executor`)를 직접 돈다."""

    def __init__(self, *a, **kw):
        self._nodes = []

    def add_node(self, node):
        self._nodes.append(node)

    def spin(self):
        for n in self._nodes:
            spin(n)
