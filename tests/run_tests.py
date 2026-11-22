# -*- coding: utf-8 -*-
"""harness_sitl/tests/run_tests.py - 자체 테스트 러너 (pytest 불필요).

왜 직접 만들었는가
------------------
`plan_pontoon_rl/.venv` 에 pytest 가 없다 (확인: pytest MISSING). "새 의존성을
추가하지 말고 기존 것을 활용하라"는 지시에 따라 pytest 를 설치하지 않고,
`test_*.py` 안의 `test_*` 함수를 직접 호출한다. 함수 이름이 pytest 규약이므로
나중에 pytest 를 설치하면 수집도 그대로 된다.

실행
----
    ../../plan_pontoon_rl/.venv/Scripts/python.exe run_tests.py
    ../../plan_pontoon_rl/.venv/Scripts/python.exe run_tests.py --fast
        --fast : 오래 걸리는 시험(합격시험 20 에피소드)을 건너뛴다

종료코드 0 = 전부 통과, 1 = 하나라도 실패.
"""
from __future__ import annotations

import argparse
import importlib
import os
import sys
import time
import traceback

import _common  # noqa: F401  (sys.path 설정)
from _console import utf8_stdout

HERE = os.path.dirname(os.path.abspath(__file__))

# (모듈, 느린 시험인가)
MODULES = [
    ("test_schema_sync", False),
    ("test_wire_roundtrip", False),
    ("test_onnx_parity", False),
    ("test_px4_state", False),
    ("test_geometry_cfg", False),
    ("test_same_code_both_paths", False),
    ("test_sitl_parity", False),
    ("test_safety", False),
    ("test_acceptance", True),
]


def run_module(name):
    mod = importlib.import_module(name)
    names = sorted(n for n in dir(mod) if n.startswith("test_"))
    rows = []
    for n in names:
        fn = getattr(mod, n)
        if not callable(fn):
            continue
        t0 = time.perf_counter()
        try:
            msg = fn()
            rows.append((n, True, msg or "", time.perf_counter() - t0, None))
        except Exception as e:                                  # noqa: BLE001
            rows.append((n, False, str(e), time.perf_counter() - t0,
                         traceback.format_exc()))
    return rows


def main():
    utf8_stdout()
    ap = argparse.ArgumentParser(description="harness_sitl 테스트 러너")
    ap.add_argument("--fast", action="store_true",
                    help="느린 시험(합격시험 20 에피소드) 건너뛰기")
    ap.add_argument("--only", default=None, help="모듈 하나만 실행")
    a = ap.parse_args()

    sys.path.insert(0, HERE)
    plan = [(m, slow) for m, slow in MODULES
            if (a.only is None or a.only in m) and not (a.fast and slow)]

    print("harness_sitl 테스트 (pytest 없이 직접 실행)")
    print("  python : %s" % sys.executable)
    print("  모듈   : %s" % ", ".join(m for m, _ in plan))
    if a.fast:
        print("  --fast : 느린 시험 제외 (%s)"
              % ", ".join(m for m, s in MODULES if s))
    print("")

    npass = nfail = 0
    fails = []
    for m, _ in plan:
        print("[%s]" % m)
        try:
            rows = run_module(m)
        except Exception:                                       # noqa: BLE001
            print("  !! 모듈 import 실패")
            print(traceback.format_exc())
            nfail += 1
            fails.append((m, "<import>", traceback.format_exc()))
            continue
        for n, ok, msg, el, tb in rows:
            print("  %-34s %s  %6.2fs  %s"
                  % (n, "PASS" if ok else "FAIL", el, msg))
            if ok:
                npass += 1
            else:
                nfail += 1
                fails.append((m, n, tb))
        print("")

    if fails:
        print("실패 상세")
        print("---------")
        for m, n, tb in fails:
            print("%s::%s" % (m, n))
            print(tb)

    print("합계: %d PASS / %d FAIL" % (npass, nfail))
    return 0 if nfail == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
