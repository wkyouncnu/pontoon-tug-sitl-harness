# -*- coding: utf-8 -*-
"""harness_sitl/nuc/ — 마더십 NUC 에서 **실물에서 그대로 실행되는** 제어 모듈.

경계 규칙 (사용자 확정 지시)
----------------------------
* 이 패키지 안의 파일은 **가상/실물 구분이 없다.** 하네스도 실물 NUC 도 같은
  파일을 import 한다. 한 글자도 다르지 않다.
* 그래서 이 패키지는 **전송계층을 모른다.** 토픽·소켓·MAVLink·ROS 2 를
  import 하지 않는다. 입력은 평범한 숫자·배열, 출력도 평범한 배열이다.
* 의존성은 **numpy + onnxruntime + quadprog** 뿐이다 (`pontoon_rl/env/latch_qp.py`
  를 통해 quadprog). `stable_baselines3`·`torch` 는 import 하지 않는다 —
  실물 NUC 에 설치하지 않는 것이 목적이다.

바뀌는 것 / 안 바뀌는 것
------------------------
`harness_sitl/docs/FILE_SWAP.md` 에 표로 고정했다.
"""
