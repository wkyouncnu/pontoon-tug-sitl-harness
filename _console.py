# -*- coding: utf-8 -*-
"""harness_sitl/_console.py — Windows cp949 콘솔에서 한글/기호 출력 보호.

이 PC 의 기본 콘솔 코드페이지는 cp949 라 '—' 같은 문자를 print 하면
UnicodeEncodeError 가 난다. 모든 실행 스크립트 맨 앞에서 이 함수를 부른다.
"""
from __future__ import annotations

import sys


def utf8_stdout() -> None:
    for st in (sys.stdout, sys.stderr):
        try:
            st.reconfigure(encoding="utf-8", errors="replace")
        except Exception:
            pass
