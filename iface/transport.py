# -*- coding: utf-8 -*-
"""harness_sitl/iface/transport.py — 전송계층 추상화 + inproc 구현.

노드 A/B/C/D 는 `Transport` 인터페이스만 쓴다.
실물 전환 시 `Ros2Transport` (ros2_stub.py) 로 **갈아끼우기만** 하고
노드 본체는 고치지 않는다.

지연/드랍 모델
--------------
* 지연은 **초가 아니라 틱 수**로 센다 → 결정론적이고 CI 재현 가능.
* 기본값 0 (무지연·무손실). 실물 지연예산(§schema.yaml latency_budget_ms)은
  전부 "확인 필요"이므로 기본값으로 켜지 않는다.
* 드랍은 토픽별 전용 RNG(시드 고정)로 뽑는다 → 에피소드 간 재현된다.
* `latest()` 는 **마지막으로 도착한 메시지를 계속 들고 있는다**(sample-and-hold).
  이것이 실물 Pixhawk 동작과 같다 — tug_tx.py 주석: "Pixhawk 는 수신 타임아웃이
  없어 마지막 값을 유지한다".
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Any, Dict, Optional

import numpy as np


@dataclass
class LinkConfig:
    """한 토픽의 전송 특성."""
    delay_ticks: int = 0      # 전송 지연 (틱). 0 = 즉시
    drop_prob: float = 0.0    # 패킷 손실 확률 [0,1]
    serialize: bool = True    # True 면 pack/unpack 왕복 (양자화 재현)


class Transport:
    """전송계층 인터페이스. 구현체는 이 4개만 제공하면 된다."""

    def publish(self, topic: str, msg: Any) -> None:
        raise NotImplementedError

    def latest(self, topic: str) -> Optional[Any]:
        """가장 최근 도착 메시지 (sample-and-hold). 아직 없으면 None."""
        raise NotImplementedError

    def tick(self) -> None:
        """시뮬레이션 틱 1회 진행. 지연 큐를 전진시킨다."""
        raise NotImplementedError

    def stats(self) -> Dict[str, Dict[str, int]]:
        raise NotImplementedError


class InprocTransport(Transport):
    """프로세스 내 큐 기반 전송계층 (기본값).

    Parameters
    ----------
    links : {topic: LinkConfig}
        지정하지 않은 토픽은 기본 LinkConfig (무지연·무손실·직렬화 ON).
    seed : int
        드랍 추출용 시드.
    """

    def __init__(self, links: Optional[Dict[str, LinkConfig]] = None,
                 seed: int = 0, default: Optional[LinkConfig] = None):
        self._cfg: Dict[str, LinkConfig] = dict(links or {})
        self._default = default or LinkConfig()
        self._rng = np.random.default_rng(seed)
        self._inflight: Dict[str, deque] = {}
        self._held: Dict[str, Any] = {}
        self._n_pub: Dict[str, int] = {}
        self._n_drop: Dict[str, int] = {}
        self._n_rx: Dict[str, int] = {}
        self._t = 0

    # -- 설정 ---------------------------------------------------------------
    def cfg(self, topic: str) -> LinkConfig:
        return self._cfg.get(topic, self._default)

    def set_link(self, topic: str, cfg: LinkConfig) -> None:
        self._cfg[topic] = cfg

    # -- 전송 ---------------------------------------------------------------
    def publish(self, topic: str, msg: Any) -> None:
        c = self.cfg(topic)
        self._n_pub[topic] = self._n_pub.get(topic, 0) + 1

        if c.drop_prob > 0.0 and float(self._rng.random()) < c.drop_prob:
            self._n_drop[topic] = self._n_drop.get(topic, 0) + 1
            return

        if c.serialize:
            # 실물과 같은 경로: wire dtype 으로 깎고 → pack → unpack
            w = msg.to_wire() if hasattr(msg, "to_wire") else msg
            msg = type(w).unpack(w.pack())

        if c.delay_ticks <= 0:
            self._held[topic] = msg
            self._n_rx[topic] = self._n_rx.get(topic, 0) + 1
        else:
            self._inflight.setdefault(topic, deque()).append(
                (self._t + c.delay_ticks, msg))

    def latest(self, topic: str) -> Optional[Any]:
        return self._held.get(topic)

    def tick(self) -> None:
        self._t += 1
        for topic, q in self._inflight.items():
            while q and q[0][0] <= self._t:
                _, m = q.popleft()
                self._held[topic] = m
                self._n_rx[topic] = self._n_rx.get(topic, 0) + 1

    # -- 진단 ---------------------------------------------------------------
    def stats(self) -> Dict[str, Dict[str, int]]:
        keys = set(self._n_pub) | set(self._n_rx) | set(self._n_drop)
        return {k: dict(pub=self._n_pub.get(k, 0),
                        rx=self._n_rx.get(k, 0),
                        drop=self._n_drop.get(k, 0)) for k in sorted(keys)}

    def reset(self) -> None:
        self._inflight.clear()
        self._held.clear()
        self._n_pub.clear()
        self._n_drop.clear()
        self._n_rx.clear()
        self._t = 0


def make_transport(kind: str = "inproc", **kw) -> Transport:
    """팩토리. 노드가 전송계층을 몰라도 되게 한다."""
    kind = (kind or "inproc").lower()
    if kind == "inproc":
        return InprocTransport(**kw)
    if kind == "ros2":
        from .ros2_stub import Ros2Transport
        return Ros2Transport(**kw)
    raise ValueError("unknown transport: %r (inproc | ros2)" % kind)
