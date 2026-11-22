# -*- coding: utf-8 -*-
"""harness_sitl/tools/sniff_mothership.py — 마더십이 실제로 무엇을 보내는지 센다.

용도
----
`docs/INPUT_PATHS.md` 의 **확인 필요 1 항**("마더십 Pixhawk 가 `ATTITUDE` ·
`LOCAL_POSITION_NED` · `POSITION_TARGET_LOCAL_NED` 를 내보내는가")을 현장에서
1 분에 닫는다. 추측하지 않고 **세어서** 답한다.

실행 (NUC 에서)
---------------
    python3 tools/sniff_mothership.py --port 14600 --seconds 30
    python3 tools/sniff_mothership.py --port 14600 --seconds 30 --json out/sniff.json

- **읽기만 한다.** 아무것도 보내지 않으므로 기존 `mothership_rx` 가 돌고 있어도
  무해하다. 단 같은 포트를 두 프로세스가 bind 하면 한쪽만 받으므로, 기존 노드를
  내리고 재거나 `--port` 로 포워딩 포트를 잰다.
- 한 메시지라도 `POSITION_TARGET_LOCAL_NED` 가 안 보이면 `psi_ref` 출처를
  정해야 한다 (`--psi-ref-los` 또는 52501 경로).

출력
----
메시지별 **개수 · 추정 주기(Hz) · 송신 sysid/compid**, 그리고 PX4 표준경로
요구조건 3 개의 충족 여부.
"""
from __future__ import annotations

import argparse
import json
import os
import socket
import sys
import time
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
HARNESS = os.path.dirname(HERE)
for _p in (HARNESS, os.path.join(HARNESS, "mavlink"),
           os.path.join(HARNESS, "vendor")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

# 다이얼렉트 — 하네스 사본을 먼저, 없으면 NUC 설치본을 쓴다.
try:
    from gen import usv_tug as dialect
except ImportError:                                    # NUC 설치 환경
    from tug_bridge.mavlink_dialect import usv_tug as dialect

NEEDED = ("ATTITUDE", "LOCAL_POSITION_NED", "POSITION_TARGET_LOCAL_NED")
ALT_POS = "GLOBAL_POSITION_INT"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=14600)
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--json", default=None, help="결과 json 저장경로")
    a = ap.parse_args(argv)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((a.bind, a.port))
    sock.settimeout(0.2)
    mav = dialect.MAVLink(None)
    mav.robust_parsing = True

    n = defaultdict(int)
    src = {}
    first = {}
    last = {}
    n_bad = 0
    n_pkt = 0
    t0 = time.monotonic()
    print("듣는다: %s:%d  %.0f s" % (a.bind, a.port, a.seconds))
    while time.monotonic() - t0 < a.seconds:
        try:
            buf, addr = sock.recvfrom(4096)
        except socket.timeout:
            continue
        n_pkt += 1
        for msg in (mav.parse_buffer(buf) or []):
            t = msg.get_type()
            if t == "BAD_DATA":
                n_bad += 1
                continue
            now = time.monotonic()
            n[t] += 1
            first.setdefault(t, now)
            last[t] = now
            src[t] = "%d/%d  from %s" % (msg.get_srcSystem(),
                                         msg.get_srcComponent(), addr[0])
    sock.close()

    rows = []
    for t in sorted(n, key=lambda k: -n[k]):
        span = max(last[t] - first[t], 1e-9)
        hz = (n[t] - 1) / span if n[t] > 1 else 0.0
        rows.append({"msg": t, "count": n[t], "hz": round(hz, 2),
                     "src": src[t]})

    print("")
    print("%-30s %7s %8s  %s" % ("메시지", "개수", "Hz", "sysid/compid"))
    print("-" * 72)
    for r in rows:
        print("%-30s %7d %8.2f  %s" % (r["msg"], r["count"], r["hz"],
                                       r["src"]))
    print("-" * 72)
    print("패킷 %d · BAD_DATA %d" % (n_pkt, n_bad))

    missing = [m for m in NEEDED if n.get(m, 0) == 0]
    ok = not missing
    print("")
    print("PX4 표준 입력경로 요구조건")
    for m in NEEDED:
        print("  %-30s %s" % (m, "있음 (%d)" % n[m] if n.get(m) else "없음"))
    if not ok:
        print("  대체경로 %-18s %s" % (ALT_POS,
                                       "있음 (%d)" % n[ALT_POS]
                                       if n.get(ALT_POS) else "없음"))
        print("")
        print("판정: 표준경로 **불가** — 없는 메시지: %s" % ", ".join(missing))
        if "POSITION_TARGET_LOCAL_NED" in missing and len(missing) == 1:
            print("  → `--psi-ref-los` 로 목표방위각을 쓰거나 52501 경로를 쓴다.")
    else:
        print("")
        print("판정: 표준경로 **가능** — 다이얼렉트·마더십·벤더코드 변경 0.")

    if a.json:
        with open(a.json, "w", encoding="utf-8") as f:
            json.dump({"bind": a.bind, "port": a.port, "seconds": a.seconds,
                       "packets": n_pkt, "bad_data": n_bad,
                       "messages": rows, "missing": missing,
                       "px4std_possible": ok}, f,
                      ensure_ascii=False, indent=1)
        print("저장: %s" % a.json)
    return 0 if ok else 2


if __name__ == "__main__":
    sys.exit(main())
