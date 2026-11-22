# -*- coding: utf-8 -*-
"""`tug_interfaces/msg` 대역 — ROS 2 메시지 클래스 4개.

출처 (타인 저장소. 읽기전용으로 베껴 적었다)
--------------------------------------------
* `tug_interfaces/msg/MothershipData.msg:1-4`   — `timestamp` + `tau_x/y/n`
* `tug_interfaces/msg/TugCommand.msg:1-7`       — `timestamp` + 터그 2대 × `τ`
* `tug_interfaces/msg/AllocationDebug.msg:1-26` — QP 진단 (로깅 전용)
* `MothershipState` 는 **우리가 신설한다** → `mavlink/MothershipState.msg`
  (MAVLink 52501 과 1:1. S5 의 추가분이고 기존 메시지는 건드리지 않는다.)

진짜 `rosidl` 생성 클래스와 **같게 둔 것**
------------------------------------------
* 인수 없이 만들 수 있고 모든 필드가 **0 으로 초기화**된다
  (`out = MothershipData()` 뒤에 필드를 하나씩 넣는 벤더 코드가 그래야 돈다).
* 필드는 **쓰기 가능**하고 `getattr(msg, name)` 으로 읽힌다
  (`tug_tx._on_send` 가 `TUG_FIELD_NAMES` 를 `getattr` 로 돈다).
* `__slots__` 로 **오타 필드를 거부**한다. 진짜 ROS 메시지도 거부한다 —
  대역이 허용하면 가상에서만 돌아가는 코드가 생긴다.

⚠️ 의도적으로 **다른 것**: `float32` 필드를 파이썬 float(배정도)에 담는다.
진짜 ROS 도 파이썬 측은 배정도이고 **직렬화에서** 잘린다. 이 하네스에서 자르는
곳은 MAVLink 와이어(`<Qfff`, `<QfffffffffBBB`)뿐이다 — 양자화 지점을 한 곳으로
모아두는 것이 목적이다. → `docs/FILE_SWAP.md` §3.
"""
from __future__ import annotations


class _Msg(object):
    """필드 목록을 `_FIELDS` 로 선언하는 작은 메시지 기반틀."""

    __slots__ = ()
    _FIELDS = ()          # (이름, 형) 쌍. 형은 int/float/bool
    _INT = ()

    def __init__(self, **kw):
        for name, kind in self._FIELDS:
            setattr(self, name, kind())
        for k, v in kw.items():
            if k not in dict(self._FIELDS):
                raise AttributeError("%s 에 없는 필드: %s" % (type(self).__name__, k))
            setattr(self, k, v)

    def __repr__(self):
        inner = ", ".join("%s=%r" % (n, getattr(self, n)) for n, _ in self._FIELDS)
        return "%s(%s)" % (type(self).__name__, inner)

    def __eq__(self, other):
        if type(other) is not type(self):
            return NotImplemented
        return all(getattr(self, n) == getattr(other, n) for n, _ in self._FIELDS)

    def get_fields_and_field_types(self):
        """진짜 rosidl 클래스에도 있는 조회 메서드 (진단용)."""
        kinds = {int: "uint64", float: "float64", bool: "bool"}
        return dict((n, kinds.get(k, "unknown")) for n, k in self._FIELDS)


class MothershipData(_Msg):
    """마더십 → NUC 의 일반화력. MAVLink **52502** 와 1:1 (`<Qfff`)."""

    __slots__ = ("timestamp", "tau_x", "tau_y", "tau_n")
    _FIELDS = (("timestamp", int), ("tau_x", float), ("tau_y", float),
               ("tau_n", float))


class TugCommand(_Msg):
    """NUC → 터그 2대의 일반화력. MAVLink **52503** 와 1:1 (`<Qdddddd`, 양자화 없음)."""

    __slots__ = ("timestamp", "tau_x_one", "tau_y_one", "tau_n_one",
                 "tau_x_two", "tau_y_two", "tau_n_two")
    _FIELDS = (("timestamp", int),
               ("tau_x_one", float), ("tau_y_one", float), ("tau_n_one", float),
               ("tau_x_two", float), ("tau_y_two", float), ("tau_n_two", float))


class MothershipState(_Msg):
    """★ S5 신설. 마더십 → NUC 의 **상태**. MAVLink **52501** 와 1:1.

    RL 관측 19차가 `x, y, psi, u, v, r` 을 필요로 하는데 52502 에는 τ 뿐이라
    받을 길이 없었다. → `CLAUDE.md` §9 2026-11-22(오후).
    """

    __slots__ = ("timestamp", "x", "y", "psi", "u", "v", "r",
                 "x_ref", "y_ref", "psi_ref",
                 "scenario_mode", "rc_on_off", "on_off")
    _FIELDS = (("timestamp", int),
               ("x", float), ("y", float), ("psi", float),
               ("u", float), ("v", float), ("r", float),
               ("x_ref", float), ("y_ref", float), ("psi_ref", float),
               ("scenario_mode", int), ("rc_on_off", int), ("on_off", int))


class AllocationDebug(_Msg):
    """QP 진단. 로깅 전용이고 와이어로 나가지 않는다."""

    __slots__ = ("timestamp", "tau_x", "tau_y", "tau_n",
                 "tau_real_x", "tau_real_y", "tau_real_n",
                 "error_x", "error_y", "error_n",
                 "ok", "saturated", "solve_time_us",
                 "load_x_one", "load_y_one", "load_x_two", "load_y_two")
    _FIELDS = (("timestamp", int),
               ("tau_x", float), ("tau_y", float), ("tau_n", float),
               ("tau_real_x", float), ("tau_real_y", float), ("tau_real_n", float),
               ("error_x", float), ("error_y", float), ("error_n", float),
               ("ok", bool), ("saturated", bool), ("solve_time_us", float),
               ("load_x_one", float), ("load_y_one", float),
               ("load_x_two", float), ("load_y_two", float))


__all__ = ["MothershipData", "TugCommand", "MothershipState", "AllocationDebug"]
