# -*- coding: utf-8 -*-
"""52501 (MOTHERSHIP_STATE) 반영 패치를 생성한다.

- 대상 저장소: `JiwoongHa/cooperative-tug-experiment` (**타인 소유 — 읽기전용**)
- 본 스크립트는 **읽기만** 한다. 대상 저장소에 쓰지 않고 push 하지 않는다
- 산출물: `patch/0001-add-mothership-state-52501.patch`
- 적용 절차는 `patch/README.md`

생성 규칙
- 기준(a) = 실험 저장소 원본, 변경(b) = 본 하네스가 쓰는 사본
- 두 파일은 **합성**한다 (하네스 사본을 그대로 쓰면 전체 교체 패치가 되기 때문)
  - `usv_tug.xml` — 원본에 52501 `<message>` 블록만 삽입
  - `tug_interfaces/CMakeLists.txt` — 원본에 `"msg/MothershipState.msg"` 한 줄만 삽입
- 줄끝은 원본의 지배적 줄끝에 맞춘다 (대상 저장소는 **전부 LF** 임을 실측)
- 결과는 **전 파일 가산(+만)** 이어야 한다. 삭제가 나오면 합성 규칙이 틀린 것이다

실행:
    ..\\plan_pontoon_rl\\.venv\\Scripts\\python.exe patch\\make_patch.py
"""
from __future__ import annotations

import difflib
import io
import os
import sys

CR = chr(13)
LF = chr(10)
CRLF = CR + LF

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
EXP = os.path.join(ROOT, "..", "cooperative-tug-experiment-main", "experiment")
SRC = os.path.join(EXP, "nuc 코드", "nuc_ws", "src")

OUT = os.path.join(HERE, "0001-add-mothership-state-52501.patch")

NUC = "nuc 코드/nuc_ws/src"

# (저장소 상대경로, 원본 절대경로 또는 None(신규), 변경본 절대경로 또는 SYNTH_*)
ITEMS = [
    (NUC + "/tug_bridge/mavlink/usv_tug.xml",
     os.path.join(SRC, "tug_bridge", "mavlink", "usv_tug.xml"),
     "SYNTH_XML"),
    (NUC + "/tug_bridge/tug_bridge/mothership_rx.py",
     os.path.join(SRC, "tug_bridge", "tug_bridge", "mothership_rx.py"),
     os.path.join(ROOT, "vendor", "tug_bridge", "mothership_rx.py")),
    (NUC + "/tug_interfaces/msg/MothershipState.msg",
     None,
     os.path.join(ROOT, "mavlink", "MothershipState.msg")),
    (NUC + "/tug_interfaces/CMakeLists.txt",
     os.path.join(SRC, "tug_interfaces", "CMakeLists.txt"),
     "SYNTH_CMAKE"),
    ("simulink/MothershipState.msg",
     None,
     os.path.join(ROOT, "mavlink", "simulink", "MothershipState.msg")),
]


def read_lines(path):
    return io.open(path, "rb").read().decode("utf-8").splitlines(True)


def target_nl(a_lines):
    """원본의 지배적 줄끝. 실측 결과 대상 저장소는 전부 LF 이다."""
    if not a_lines:
        return LF
    n = sum(1 for l in a_lines if l.endswith(CRLF))
    return CRLF if n * 2 > len(a_lines) else LF


def norm_nl(lines, nl):
    out = []
    for l in lines:
        s = l.rstrip(CR + LF)
        out.append(s + nl if l != s else s)
    return out


def synth_xml(orig_lines):
    """52501 블록만 떼어 `52502` 메시지 바로 앞에 넣는다 (id 오름차순)."""
    ours = read_lines(os.path.join(ROOT, "mavlink", "usv_tug.xml"))
    beg = [i for i, l in enumerate(ours) if 'message id="52501"' in l]
    if len(beg) != 1:
        raise RuntimeError("52501 여는 줄을 1 개 찾지 못했다: %r" % beg)
    end = None
    for j in range(beg[0] + 1, len(ours)):
        if "</message>" in ours[j]:
            end = j
            break
    if end is None:
        raise RuntimeError("52501 닫는 줄을 찾지 못했다")
    block = ours[beg[0]:end + 1]

    at = [i for i, l in enumerate(orig_lines) if 'message id="52502"' in l]
    if len(at) != 1:
        raise RuntimeError("원본에서 52502 여는 줄을 1 개 찾지 못했다: %r" % at)
    new = list(orig_lines)
    new[at[0]:at[0]] = block
    return new


def synth_cmake(orig_lines):
    """`msg/MothershipData.msg` 바로 아래에 한 줄을 넣는다."""
    key = "msg/MothershipData.msg"
    hit = [i for i, l in enumerate(orig_lines) if key in l]
    if len(hit) != 1:
        raise RuntimeError("CMakeLists 에서 %s 를 1 개 찾지 못했다: %r" % (key, hit))
    i = hit[0]
    line = orig_lines[i]
    nl = CRLF if line.endswith(CRLF) else LF
    indent = line[:len(line) - len(line.lstrip())]
    new = list(orig_lines)
    new.insert(i + 1, indent + '"msg/MothershipState.msg"' + nl)
    return new


SYNTH = {"SYNTH_XML": synth_xml, "SYNTH_CMAKE": synth_cmake}


def main():
    missing = [p for _, p, _ in ITEMS if p and not os.path.isfile(p)]
    missing += [p for _, _, p in ITEMS
                if p not in SYNTH and not os.path.isfile(p)]
    if missing:
        for m in missing:
            print("[FAIL] 입력이 없다: %s" % m)
        return 1

    chunks = []
    stat = []
    for rel, apath, bpath in ITEMS:
        if apath is None:
            a = []
            afrom = "/dev/null"
        else:
            a = read_lines(apath)
            afrom = "a/" + rel

        b = SYNTH[bpath](a) if bpath in SYNTH else read_lines(bpath)
        b = norm_nl(b, target_nl(a))

        body = list(difflib.unified_diff(a, b, fromfile=afrom,
                                         tofile="b/" + rel, n=3))
        if not body:
            print("[skip] 차이 없음: %s" % rel)
            continue
        head = "diff --git a/%s b/%s" % (rel, rel) + LF
        if apath is None:
            head += "new file mode 100644" + LF
        chunks.append(head + "".join(body))
        plus = sum(1 for l in body
                   if l.startswith("+") and not l.startswith("+++"))
        minus = sum(1 for l in body
                    if l.startswith("-") and not l.startswith("---"))
        stat.append((rel, plus, minus, apath is None))

    text = "".join(chunks)
    io.open(OUT, "w", encoding="utf-8", newline="").write(text)

    tot_m = sum(s[2] for s in stat)
    print("생성: %s (%d B)" % (OUT, len(text.encode("utf-8"))))
    print("")
    print("%-62s %6s %6s" % ("file", "+", "-"))
    for rel, p, m, isnew in stat:
        print("%-62s %6d %6d %s" % (rel, p, m, "(신규)" if isnew else ""))
    print("")
    print("합계 +%d / -%d" % (sum(s[1] for s in stat), tot_m))
    if tot_m != 0:
        print("[FAIL] 삭제가 %d 줄 있다 — 가산 전용이어야 한다" % tot_m)
        return 1
    print("가산 전용 확인")
    return 0


if __name__ == "__main__":
    sys.exit(main())
