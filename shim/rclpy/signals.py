# -*- coding: utf-8 -*-
"""`rclpy.signals` 대역.

벤더 노드는 `rclpy.init(signal_handler_options=SignalHandlerOptions.NO)` 로
**rclpy 의 신호 처리를 끄고** `relay_common.install_shutdown_signal_handlers()`
로 직접 잡는다 (종료 시 τ=0 을 10회 보내야 하므로). 열거값만 있으면 된다.
"""
from __future__ import annotations


class SignalHandlerOptions(object):
    NO = 0
    SIGINT = 1
    SIGTERM = 2
    ALL = 3
