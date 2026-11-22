# -*- coding: utf-8 -*-
"""테스트 공통 경로 설정.

pytest 는 이 venv 에 설치되어 있지 않다 (확인: pytest MISSING). 새 의존성을
추가하지 않는다는 지시에 따라 `tests/run_tests.py` 가 직접 `test_*` 함수를
호출한다. 함수 이름을 pytest 규약으로 맞춰 두었으므로 나중에 pytest 를
설치하면 그대로 수집된다.
"""
from __future__ import annotations

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
HARNESS = os.path.dirname(HERE)
ROOT = os.path.dirname(HARNESS)

for _p in (HARNESS, os.path.join(ROOT, "pontoon_rl")):
    if _p not in sys.path:
        sys.path.insert(0, _p)
