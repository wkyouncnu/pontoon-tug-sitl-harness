# -*- coding: utf-8 -*-
"""harness_sitl/shim/rclpy — ROS 2 `rclpy` 의 **최소 대역**. 가상 전용.

왜 있나
-------
S5 의 목표는 **`vendor/tug_bridge/` 의 노드를 한 글자도 고치지 않고 돌리는 것**
이다. 그 노드들은 `rclpy` 와 `tug_interfaces` 를 import 한다. 이 PC 에는 ROS 2
가 없다 (Windows + 파이썬 3.8). 그래서 **노드가 실제로 쓰는 API 만** 흉내낸다.

> ⚠️ 이것은 ROS 2 가 아니다. `rclpy` 의 진짜 구현도 아니다. 하네스가 벤더 노드를
> 실행하기 위한 **얇은 대역**이고, 실물 NUC 에서는 **진짜 `rclpy` 가 이 자리에
> 들어간다**. → `docs/FILE_SWAP.md` §2.

구현 범위 (벤더 노드가 실제로 호출하는 것 전부)
-----------------------------------------------
| 벤더가 쓰는 것 | 여기서 |
|---|---|
| `rclpy.init(args=, signal_handler_options=)` | 컨텍스트 생성 |
| `rclpy.spin(node)` · `rclpy.try_shutdown()` | 단일스레드 실행기 |
| `rclpy.node.Node` | 파라미터·로거·pub/sub·타이머 |
| `rclpy.parameter.Parameter(.Type)` | 타입 열거 + 오버라이드 |
| `rclpy.qos.QoSProfile/HistoryPolicy/ReliabilityPolicy` | 큐 깊이만 실제로 쓴다 |
| `rclpy.executors.ExternalShutdownException` | 예외 클래스 |
| `rclpy.signals.SignalHandlerOptions` | 열거 (신호는 하네스가 안 쓴다) |

의도적으로 **다르게** 둔 것
---------------------------
* **노드별 단일스레드 실행기**다. 콜백은 노드마다 직렬화되고 발행은 구독자 큐에
  넣기만 한다(`KEEP_LAST` 깊이 적용). 진짜 `SingleThreadedExecutor` 와 같은
  의미이고, 발행 스레드에서 남의 콜백이 실행되는 사고를 막는다.
* **`reliable` 과 `best_effort` 를 구분하지 않는다.** 같은 프로세스 안이라
  손실이 없다. 손실·지연 주입은 `tools_sweep_comm.py` 가 와이어에서 한다.
* 파라미터 오버라이드는 **진짜 API 인 `parameter_overrides=`** 로 받는다.
  `tug_tx` 두 인스턴스가 포트만 다르게 떠야 하므로 필요하다.

⚠️ Windows 타이머 분해능
------------------------
파이썬 3.8 의 `time.sleep` 은 Windows 에서 기본 **약 15.6 ms** 단위로 깬다.
50 Hz(20 ms) 타이머가 그대로면 레이트가 무너진다. 그래서 `init()` 이
`timeBeginPeriod(1)` 로 분해능을 1 ms 로 올리고 `shutdown()` 이 되돌린다.
남은 오차는 마감시각 기준 스케줄링(아래 `_Executor`)이 흡수한다 —
**주기가 밀리면 다음 주기를 당겨서 평균 레이트를 유지한다.**
"""
from __future__ import annotations

import collections
import copy
import sys
import threading
import time
from typing import Any, Callable, Deque, Dict, List, Optional, Tuple

# ---------------------------------------------------------------- 타이머 분해능
_WINMM = None


def _timer_resolution(on: bool) -> None:
    """Windows 멀티미디어 타이머 분해능을 1 ms 로 올린다/되돌린다."""
    global _WINMM
    if not sys.platform.startswith("win"):
        return
    try:
        import ctypes
        if _WINMM is None:
            _WINMM = ctypes.WinDLL("winmm")
        (_WINMM.timeBeginPeriod if on else _WINMM.timeEndPeriod)(1)
    except Exception:                                    # noqa: BLE001
        pass                                             # 올리지 못해도 동작은 한다


# -------------------------------------------------------------------- 컨텍스트
class Context(object):
    """`node.context.ok()` 만 쓰인다 (벤더 노드의 종료 판정)."""

    def __init__(self) -> None:
        self._ok = True

    def ok(self) -> bool:
        return self._ok

    def shutdown(self) -> None:
        self._ok = False


_context = Context()
_inited = False


def init(args: Optional[list] = None, *, context: Optional[Context] = None,
         signal_handler_options: Any = None, domain_id: Any = None) -> None:
    global _inited
    _context._ok = True
    _inited = True
    _timer_resolution(True)


def ok(context: Optional[Context] = None) -> bool:
    return _context.ok()


def shutdown(*, context: Optional[Context] = None) -> None:
    global _inited
    if _inited:
        _timer_resolution(False)
    _inited = False
    _context.shutdown()


def try_shutdown(*, context: Optional[Context] = None) -> None:
    shutdown()


def get_default_context() -> Context:
    return _context


# -------------------------------------------------------------------- 토픽 broker
class _Broker(object):
    """프로세스 안의 토픽 테이블. 발행 = 구독자 큐에 넣기."""

    def __init__(self) -> None:
        self._subs: Dict[str, List["Subscription"]] = {}
        self._lock = threading.Lock()
        self.published: Dict[str, int] = {}

    def add(self, topic: str, sub: "Subscription") -> None:
        with self._lock:
            self._subs.setdefault(topic, []).append(sub)

    def remove(self, sub: "Subscription") -> None:
        with self._lock:
            for lst in self._subs.values():
                if sub in lst:
                    lst.remove(sub)

    def publish(self, topic: str, msg: Any) -> None:
        with self._lock:
            targets = list(self._subs.get(topic, ()))
            self.published[topic] = self.published.get(topic, 0) + 1
        for s in targets:
            s._enqueue(copy.copy(msg))   # ROS 는 항상 복사해 전달한다


_broker = _Broker()


def broker() -> _Broker:
    """하네스 진단용 (벤더 노드는 쓰지 않는다)."""
    return _broker


# ------------------------------------------------------------------------ 로거
class Logger(object):
    """`info/warn/error(msg, throttle_duration_sec=...)` 만 제공한다.

    출력은 `sink` 로 보낸다. 하네스는 파일로 빼고 꼬리만 읽는다
    (`CLAUDE.md` §10-8 컨텍스트 규율).
    """

    sink: Optional[Callable[[str], None]] = None
    verbose = False

    def __init__(self, name: str) -> None:
        self.name = name
        self._last: Dict[Tuple[str, str], float] = {}
        self.counts = {"info": 0, "warn": 0, "error": 0, "debug": 0}

    def _emit(self, level: str, msg: str, throttle: Optional[float]) -> None:
        self.counts[level] = self.counts.get(level, 0) + 1
        if throttle:
            key = (level, msg[:40])
            now = time.monotonic()
            if now - self._last.get(key, -1e9) < float(throttle):
                return
            self._last[key] = now
        line = "[%s] [%s] %s" % (level, self.name, msg)
        if Logger.sink is not None:
            Logger.sink(line)
        elif Logger.verbose:
            print(line)

    def info(self, msg: str, throttle_duration_sec: Optional[float] = None) -> None:
        self._emit("info", msg, throttle_duration_sec)

    def warn(self, msg: str, throttle_duration_sec: Optional[float] = None) -> None:
        self._emit("warn", msg, throttle_duration_sec)

    warning = warn

    def error(self, msg: str, throttle_duration_sec: Optional[float] = None) -> None:
        self._emit("error", msg, throttle_duration_sec)

    def debug(self, msg: str, throttle_duration_sec: Optional[float] = None) -> None:
        self._emit("debug", msg, throttle_duration_sec)


# ------------------------------------------------------------- pub / sub / timer
class Publisher(object):
    def __init__(self, topic: str, msg_type: Any) -> None:
        self.topic = topic
        self.msg_type = msg_type
        self.n = 0

    def publish(self, msg: Any) -> None:
        self.n += 1
        _broker.publish(self.topic, msg)


class Subscription(object):
    def __init__(self, topic: str, msg_type: Any,
                 callback: Callable[[Any], None], depth: int) -> None:
        self.topic = topic
        self.msg_type = msg_type
        self.callback = callback
        self.q: Deque[Any] = collections.deque(maxlen=max(1, int(depth)))
        self.n_dropped = 0
        self._lock = threading.Lock()

    def _enqueue(self, msg: Any) -> None:
        with self._lock:
            if len(self.q) == self.q.maxlen:
                self.n_dropped += 1      # KEEP_LAST: 가장 오래된 것이 밀린다
            self.q.append(msg)

    def _drain(self) -> int:
        n = 0
        while True:
            with self._lock:
                if not self.q:
                    return n
                msg = self.q.popleft()
            self.callback(msg)
            n += 1


class Timer(object):
    def __init__(self, period_s: float, callback: Callable[[], None]) -> None:
        self.period = float(period_s)
        self.callback = callback
        self.next_due = time.perf_counter() + self.period
        self.n = 0
        self.canceled = False

    def cancel(self) -> None:
        self.canceled = True


# ---------------------------------------------------------------------- 실행기
_SPIN_SLACK_S = 0.0015       # 이 시간 안에 마감이면 sleep 대신 양보만 한다


class _Executor(object):
    """노드 하나를 도는 단일스레드 실행기. 마감시각 기준 스케줄링."""

    def __init__(self, node: "Node") -> None:
        self.node = node

    def spin(self) -> None:
        node = self.node
        while _context.ok() and not node._destroyed:
            self.spin_once()

    def spin_once(self) -> None:
        node = self.node
        now = time.perf_counter()

        # 1) 마감이 지난 타이머
        for t in list(node._timers):
            if t.canceled:
                continue
            if now >= t.next_due:
                # 밀린 주기는 **건너뛰지 않고 당겨서** 평균 레이트를 유지한다.
                t.next_due += t.period
                if t.next_due < now:                 # 크게 밀렸으면 기준을 다시 잡는다
                    t.next_due = now + t.period
                    node._timer_overruns += 1
                t.n += 1
                t.callback()

        # 2) 구독 큐
        drained = 0
        for s in list(node._subs):
            drained += s._drain()

        # 3) 다음 마감까지 쉰다
        if drained:
            return
        due = min((t.next_due for t in node._timers if not t.canceled),
                  default=time.perf_counter() + 0.001)
        dt = due - time.perf_counter()
        if dt > _SPIN_SLACK_S:
            time.sleep(dt - _SPIN_SLACK_S)
        else:
            time.sleep(0)                            # 양보만 (GIL)


def spin(node: "Node") -> None:
    _Executor(node).spin()


def spin_once(node: "Node", *, timeout_sec: Optional[float] = None) -> None:
    _Executor(node).spin_once()


# ------------------------------------------------------------------------ Node
class Node(object):
    """벤더 노드가 상속하는 클래스. 쓰는 API 만 있다."""

    def __init__(self, node_name: str, *, parameter_overrides: Optional[list] = None,
                 context: Optional[Context] = None, **kwargs: Any) -> None:
        from rclpy.parameter import Parameter          # 순환 import 회피
        self._name = str(node_name)
        self._logger = Logger(self._name)
        self._overrides: Dict[str, Any] = {}
        for p in (parameter_overrides or ()):
            if isinstance(p, Parameter):
                self._overrides[p.name] = p.value
            elif isinstance(p, (tuple, list)) and len(p) >= 2:
                self._overrides[str(p[0])] = p[1]
        self._params: Dict[str, Parameter] = {}
        self._pubs: List[Publisher] = []
        self._subs: List[Subscription] = []
        self._timers: List[Timer] = []
        self._timer_overruns = 0
        self._destroyed = False
        self.context = context or _context

    # -- 이름·로거 ----------------------------------------------------------
    def get_name(self) -> str:
        return self._name

    def get_logger(self) -> Logger:
        return self._logger

    # -- 파라미터 -----------------------------------------------------------
    def declare_parameter(self, name: str, value: Any = None, descriptor: Any = None):
        """`value` 가 `Parameter.Type` 이면 **기본값 없음**(오버라이드 필수)."""
        from rclpy.parameter import Parameter
        # 선언 타입: `Parameter.Type` 을 넘겼으면 그것, 아니면 기본값에서 추론한다.
        if isinstance(value, Parameter.Type):
            type_ = value
            default = None               # `declare_required` 가 여기서 실패해야 한다
        else:
            type_ = Parameter.Type.infer(value)
            default = value
        v = self._overrides.get(name, default)
        # ★ 타입이 틀리면 여기서 죽는다 (yaml `50` vs `50.0` 함정 재현).
        p = Parameter(name, type_, v)
        self._params[name] = p
        return p

    def get_parameter(self, name: str):
        return self._params[name]

    # -- pub / sub / timer --------------------------------------------------
    def create_publisher(self, msg_type: Any, topic: str, qos: Any = 10) -> Publisher:
        pub = Publisher(topic, msg_type)
        self._pubs.append(pub)
        return pub

    def create_subscription(self, msg_type: Any, topic: str,
                            callback: Callable[[Any], None], qos: Any = 10) -> Subscription:
        depth = getattr(qos, "depth", qos if isinstance(qos, int) else 10)
        sub = Subscription(topic, msg_type, callback, depth)
        self._subs.append(sub)
        _broker.add(topic, sub)
        return sub

    def create_timer(self, period_s: float, callback: Callable[[], None]) -> Timer:
        t = Timer(period_s, callback)
        self._timers.append(t)
        return t

    # -- 수명주기 -----------------------------------------------------------
    def destroy_node(self) -> None:
        if self._destroyed:
            return
        self._destroyed = True
        for t in self._timers:
            t.cancel()
        for s in self._subs:
            _broker.remove(s)
