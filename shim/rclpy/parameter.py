# -*- coding: utf-8 -*-
"""`rclpy.parameter` 대역.

`relay_common.declare_required()` 가 의지하는 두 가지 성질을 **그대로** 지킨다:

1. `node.declare_parameter(name, Parameter.Type.X)` 는 **기본값을 만들지 않는다.**
   yaml 에 없으면 `.value` 가 `None` 이고 `declare_required` 가 시작을 실패시킨다.
2. **타입이 틀리면 거부한다.** 이것이 중요한 이유 —
   `CLAUDE.md` §9 2026-11-22 에 기록된 함정: `tug_bridge.yaml` 의 `*_s` / `*_hz`
   는 **`50.0` 처럼 float 로 써야 노드가 뜬다.** `50` (int) 으로 쓰면 진짜
   rclpy 가 `DOUBLE` 선언과 어긋나 죽는다. 대역이 이걸 조용히 통과시키면
   **가상에서만 뜨고 실물에서 죽는 yaml** 을 만들게 된다. 그래서 흉내낸다.

> ⚠️ bool 이 int 의 하위형이라 파이썬에서는 `isinstance(True, int)` 가 참이다.
> `INTEGER` 선언에 `True` 가 들어오는 것을 막으려면 bool 을 먼저 걸러야 한다.
"""
from __future__ import annotations

from enum import IntEnum
from typing import Any, Optional


class ParameterTypeException(TypeError):
    """선언 타입과 주어진 값의 타입이 다르다."""


class _Type(IntEnum):
    """진짜 `rclpy.parameter.Parameter.Type` 과 같은 정수값을 쓴다."""
    NOT_SET = 0
    BOOL = 1
    INTEGER = 2
    DOUBLE = 3
    STRING = 4
    BYTE_ARRAY = 5
    BOOL_ARRAY = 6
    INTEGER_ARRAY = 7
    DOUBLE_ARRAY = 8
    STRING_ARRAY = 9

    def check(self, value: Any) -> bool:
        if value is None:
            return True                      # 미설정은 타입검사 대상이 아니다
        if self is _Type.NOT_SET:
            return True
        if self is _Type.BOOL:
            return isinstance(value, bool)
        if self is _Type.INTEGER:
            return isinstance(value, int) and not isinstance(value, bool)
        if self is _Type.DOUBLE:
            # ★ 진짜 rclpy 와 같다: int 는 DOUBLE 이 아니다 (yaml `50` vs `50.0`)
            return isinstance(value, float)
        if self is _Type.STRING:
            return isinstance(value, str)
        if self is _Type.BOOL_ARRAY:
            return isinstance(value, (list, tuple)) and all(
                isinstance(v, bool) for v in value)
        if self is _Type.INTEGER_ARRAY:
            return isinstance(value, (list, tuple)) and all(
                isinstance(v, int) and not isinstance(v, bool) for v in value)
        if self is _Type.DOUBLE_ARRAY:
            return isinstance(value, (list, tuple)) and all(
                isinstance(v, float) for v in value)
        if self is _Type.STRING_ARRAY:
            return isinstance(value, (list, tuple)) and all(
                isinstance(v, str) for v in value)
        return True

    @staticmethod
    def infer(value: Any) -> "_Type":
        if value is None:
            return _Type.NOT_SET
        if isinstance(value, bool):
            return _Type.BOOL
        if isinstance(value, int):
            return _Type.INTEGER
        if isinstance(value, float):
            return _Type.DOUBLE
        if isinstance(value, str):
            return _Type.STRING
        return _Type.NOT_SET


class Parameter(object):
    """`name` · `type_` · `value` 세 속성만 쓰인다."""

    Type = _Type

    def __init__(self, name: str, type_: Optional[_Type] = None,
                 value: Any = None) -> None:
        self._name = str(name)
        if type_ is None:
            type_ = _Type.infer(value)
        self._type = type_
        if not type_.check(value):
            raise ParameterTypeException(
                "파라미터 %s: %s 선언인데 값이 %s (%r) 다 — "
                "yaml 에서 정수/실수 표기를 확인하라 (예: 50 → 50.0)"
                % (name, type_.name, type(value).__name__, value))
        self._value = value

    @property
    def name(self) -> str:
        return self._name

    @property
    def type_(self) -> _Type:
        return self._type

    @property
    def value(self) -> Any:
        return self._value

    def get_parameter_value(self) -> Any:
        return self._value

    def __repr__(self) -> str:
        return "Parameter(%r, %s, %r)" % (self._name, self._type.name, self._value)


__all__ = ["Parameter", "ParameterTypeException"]
