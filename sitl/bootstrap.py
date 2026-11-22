# -*- coding: utf-8 -*-
"""harness_sitl/sitl/bootstrap.py — 벤더 노드를 이 PC 에서 돌리기 위한 환경 준비.

`setup()` 하나가 두 가지를 한다. **둘 다 실물 NUC 에는 없다.**

1. `sys.path` 에 `shim/` 와 `vendor/` 를 올린다 → `import rclpy` ·
   `import tug_interfaces` · `import tug_bridge` 가 걸린다.
   실물 NUC 에서는 **진짜 ROS 2 가 그 자리에 있다.**
2. ★ **POSIX 신호 대역을 깐다** (Windows 전용).

왜 2번이 필요한가 — 정직하게 적는다
------------------------------------
`relay_common.py:41` 이 모듈 로드 시점에
```
TERMINATION_SIGNALS = (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)
```
를 실행한다. **Windows 파이썬에는 `signal.SIGHUP` 이 없다** → `import
tug_bridge.relay_common` 자체가 `AttributeError` 로 죽는다. 이어서
`install_shutdown_signal_handlers()` 가 세 신호 전부에 `signal.signal()` 을
걸지만 Windows 는 `SIGINT/SIGTERM/SIGBREAK/SIGABRT/SIGFPE/SIGILL/SIGSEGV`
만 받는다.

벤더 코드는 **고칠 수 없다** (`relay_common.py` diff 0 이 S5 의 검수기준이다).
그래서 **플랫폼 쪽을 벤더가 기대하는 모양으로 맞춘다.**

> ⚠️ **이것은 "가상과 실물의 차이는 전송계층과 플랜트뿐"의 예외가 아니다.**
> 그 주장은 **`nuc/` production 코드**에 대한 것이고(`tests/
> test_same_code_both_paths.py` 가 검사한다), 여기서 바꾸는 것은 **벤더
> 노드를 얹는 바닥(ROS 2 + OS)**이다. `rclpy` 대역과 같은 범주다.
> 실물 NUC 은 **리눅스**라 `SIGHUP` 이 진짜로 있고 이 대역은 **쓰이지 않는다.**
> 따라서 **종료 신호 경로는 가상에서 완전히 검증되지 않는다** —
> 가상에서 검증되는 것은 "τ=0 을 10회 보내는 송신 로직"이고, 검증되지 않는
> 것은 "SSH 끊김(SIGHUP)이 실제로 그 로직을 깨우는가"다. S6 표에 그렇게 적는다.
"""
from __future__ import annotations

import os
import signal
import sys
import threading
from typing import Callable, Dict, Optional

HERE = os.path.dirname(os.path.abspath(__file__))
HARNESS = os.path.dirname(HERE)
ROOT = os.path.dirname(HARNESS)

# 대역이 흉내낸 신호에 등록된 핸들러 (OS 가 전달해 주지 않으므로 우리가 보관한다)
_emulated: Dict[int, object] = {}
_real_signal: Optional[Callable] = None
_SIGHUP_FALLBACK = 1          # POSIX 의 SIGHUP 번호
_enabled = False


def _install_posix_signal_facade() -> None:
    """Windows 에 없는 종료 신호를 흉내낸다. 리눅스에서는 아무것도 하지 않는다."""
    global _real_signal, _enabled
    if _enabled or not sys.platform.startswith("win"):
        _enabled = True
        return

    if not hasattr(signal, "SIGHUP"):
        signal.SIGHUP = _SIGHUP_FALLBACK            # type: ignore[attr-defined]

    # Windows `signal.signal()` 이 실제로 받는 번호들
    supported = set()
    for nm in ("SIGINT", "SIGTERM", "SIGBREAK", "SIGABRT", "SIGFPE",
               "SIGILL", "SIGSEGV"):
        s = getattr(signal, nm, None)
        if s is not None:
            supported.add(int(s))

    _real_signal = signal.signal

    def _signal(signum, handler):
        n = int(signum)
        if n in supported:
            return _real_signal(signum, handler)    # 진짜로 건다
        prev = _emulated.get(n, signal.SIG_DFL)     # 흉내만 낸다
        _emulated[n] = handler
        return prev

    signal.signal = _signal                         # type: ignore[assignment]
    _enabled = True


def raise_emulated(signum: int) -> bool:
    """흉내낸 신호를 **동기적으로** 전달한다 (시험용).

    Windows 가 `SIGHUP` 을 보내줄 수 없으니, "SSH 끊김이 왔을 때 벤더 핸들러가
    무엇을 하는가"는 핸들러를 직접 불러서만 볼 수 있다. 진짜 전달이 아니라
    **핸들러 호출**임을 이름으로 못박는다.

    반환: 핸들러가 실제로 등록되어 있었는가.
    """
    h = _emulated.get(int(signum))
    if h is None or h in (signal.SIG_DFL, signal.SIG_IGN):
        return False
    if not callable(h):
        return False
    h(int(signum), None)
    return True


def emulated_disposition(signum: int):
    """흉내낸 신호의 현재 처리방식 (`SIG_IGN` 인지 확인하는 시험용)."""
    return _emulated.get(int(signum), signal.SIG_DFL)


def setup() -> None:
    """경로 + 신호 대역. 여러 번 불러도 안전하다."""
    for p in (os.path.join(HARNESS, "shim"),
              os.path.join(HARNESS, "vendor"),
              HARNESS,
              os.path.join(ROOT, "pontoon_rl")):
        if p not in sys.path:
            sys.path.insert(0, p)
    _install_posix_signal_facade()


def main_thread() -> bool:
    return threading.current_thread() is threading.main_thread()
