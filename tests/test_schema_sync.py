# -*- coding: utf-8 -*-
"""스키마 ↔ dataclass 동기 시험.

이 시험이 존재하는 이유
-----------------------
`iface/schema.yaml` 하나로 양쪽(노드·실물 .msg)이 같은 정의를 쓰게 하는 것이
설계 목표다. 그런데 YAML 은 주석 문서이고 dataclass 는 실행 코드이므로
**둘이 조용히 어긋날 수 있다.** 이 시험이 그걸 막는다.

실제로 한 번 어긋났다: schema.yaml 이 YAML 로 파싱조차 되지 않는 상태였다
(`type: float64[3]` 가 flow mapping 안에서 flow sequence 로 읽혔고,
`"/mothership_data":{...}` 는 콜론 뒤 공백이 없었다). 아래 첫 시험이
그것을 잡는다.
"""
from __future__ import annotations

import dataclasses
import os

import yaml

import _common  # noqa: F401

from iface import messages as M
from iface import ros2_stub

SCHEMA = os.path.join(_common.HARNESS, "iface", "schema.yaml")


def _load():
    with open(SCHEMA, encoding="utf-8") as f:
        return yaml.safe_load(f)


def test_schema_parses():
    """schema.yaml 이 YAML 로 읽혀야 한다 (구조 문서가 아니라 단일 정의원이다)."""
    d = _load()
    for k in ("schema_version", "frame_convention", "messages",
              "transports", "rates", "latency_budget_ms"):
        assert k in d, "schema.yaml 에 %s 절이 없다" % k
    return "v%s, 메시지 %d종, 전송 %d종" % (
        d["schema_version"], len(d["messages"]), len(d["transports"]))


def test_fields_match_dataclasses():
    """7개 메시지의 필드 이름·순서가 dataclass 와 정확히 같아야 한다."""
    d = _load()["messages"]
    checked = 0
    for name, spec in d.items():
        cls = getattr(M, name, None)
        assert cls is not None, "dataclass %s 가 없다" % name
        assert dataclasses.is_dataclass(cls), "%s 가 dataclass 가 아니다" % name
        want = [f["name"] for f in spec["fields"]]
        got = [f.name for f in dataclasses.fields(cls)]
        assert want == got, ("%s 필드 불일치\n  schema : %s\n  dataclass: %s"
                             % (name, want, got))
        checked += 1
    assert checked == 7, "메시지 수가 7 이 아니다: %d" % checked
    return "%d개 메시지 필드 일치" % checked


def test_topics_registered():
    """TOPICS / QOS / ros2_stub.TOPIC_MAP 이 같은 토픽 집합을 덮어야 한다."""
    t = set(M.TOPICS)
    q = set(M.QOS)
    r = set(ros2_stub.TOPIC_MAP)
    assert t == r, "TOPICS vs TOPIC_MAP 불일치: %s" % (t ^ r)
    missing = t - q
    assert not missing, "QoS 미정의 토픽: %s" % sorted(missing)
    return "%d개 토픽 (TOPICS=QOS=TOPIC_MAP)" % len(t)


def test_rates_match_env():
    """schema 의 레이트가 pontoon_rl 환경의 실제 값과 같아야 한다."""
    d = _load()["rates"]
    from env.pontoon_env import PontoonSc3Env          # noqa: WPS433
    e = PontoonSc3Env(dr=False)
    phys = 1.0 / e.hsub
    pol = 1.0 / e.dt
    assert abs(phys - d["physics_hz"]) < 1e-6, (
        "물리 레이트 불일치: schema %s vs env %.3f" % (d["physics_hz"], phys))
    assert abs(pol - d["policy_hz"]) < 1e-6, (
        "제어 레이트 불일치: schema %s vs env %.3f" % (d["policy_hz"], pol))
    return "물리 %.0f Hz / 제어 %.0f Hz (env 와 일치)" % (phys, pol)


def test_frame_convention_recorded():
    """좌표계 규약이 문서에 박혀 있어야 한다 (PX4 = NED/FRD = Fossen)."""
    fc = _load()["frame_convention"]
    s = yaml.safe_dump(fc, allow_unicode=True)
    assert "NED" in s and "FRD" in s, "NED/FRD 규약이 schema 에 없다"
    return "NED 월드 + FRD 바디 (부호 반전 없음)"


def test_unconfirmed_topics_flagged():
    """확정되지 않은 토픽은 반드시 confirmed=False 로 남아 있어야 한다.

    실물 .msg 가 없는 것을 '있다'고 적는 것이 이 과제에서 가장 위험한
    문서 오류다. 아래 3개는 확인 필요 상태여야 한다.
    """
    must_be_unconfirmed = ["/pontoon/state", "/pontoon/goal",
                           "/tug/thrust", "/fmu/in/mothership_mode"]
    bad = [t for t in must_be_unconfirmed
           if ros2_stub.TOPIC_MAP[t]["confirmed"]]
    assert not bad, "확인 필요인데 확정으로 표시된 토픽: %s" % bad
    conf = [t for t, v in ros2_stub.TOPIC_MAP.items() if v["confirmed"]]
    assert sorted(conf) == ["/allocation_debug", "/mothership_data",
                            "/tug_command"], "확정 토픽 목록이 바뀠다: %s" % conf
    return "확정 3 / 확인필요 4 (배포본 .msg 기준)"
