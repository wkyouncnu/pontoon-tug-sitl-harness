# -*- coding: utf-8 -*-
"""README.md + docs/RL_SPEC.md -> docs/manual/harness_sitl.pdf (A4, 단일 PDF).

- 원본은 `README.md` 와 `docs/RL_SPEC.md` 두 개다. 후자는 **부록 A** 로
  제목단계를 한 칸 낮춰 붙인다 (`## 1.` -> `### A.1`).
- `marked.js` 대신 파이썬 `markdown` 모듈을 쓴다 (이 PC 에 node·marked 가
  없고, CDN 의존을 만들지 않기 위함). 표·펜스코드 확장만 켠다.
- 수식은 LaTeX 로 쓰고 **동봉한 KaTeX**(`docs/manual/vendor/katex`)로 렌더한다.
  `markdown` 모듈이 역슬래시를 먹으므로 변환 전에 수식을 치환자로 빼내고
  변환 후에 되돌린다. 치환자는 영숫자만 쓴다.
- SVG·PNG 는 `file:///` 절대경로로 바꿔 넣는다.

실행:
    ..\\plan_pontoon_rl\\.venv\\Scripts\\python.exe docs\\make_manual.py
"""
from __future__ import annotations

import io
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SRC = os.path.join(ROOT, "README.md")
APX = os.path.join(HERE, "RL_SPEC.md")          # 수식 점검 호환용
APX_TITLE = "부록 A. 강화학습 규약 — 관측 · 행동 · 보상 · 추력배분"

# 저장소의 모든 MD 를 부록으로 바인드한다 (사용자 지시: 단일 PDF 통합).
#   (문자, 경로(ROOT 기준), 상대링크 접두사, 제목)
APXS = [
    ("A", "docs/RL_SPEC.md", "docs/",
     "부록 A. 강화학습 규약 — 관측 · 행동 · 보상 · 추력배분"),
    ("B", "docs/RUN_ORDER.md", "docs/",
     "부록 B. 실행 순서 — 시뮬레이션 · 실물 예인시험"),
    ("C", "docs/FIELD_CHECKLIST.md", "docs/", "부록 C. 현장 점검표"),
    ("D", "docs/DEBUG.md", "docs/",
     "부록 D. 디버그 — 계층 분리 · 계수기 해석 · 증상별 조치"),
    ("E", "docs/SITL.md", "docs/", "부록 E. SITL 종단 시험"),
    ("F", "docs/FILE_SWAP.md", "docs/", "부록 F. 파일 교체 경계"),
    ("G", "docs/INPUT_PATHS.md", "docs/", "부록 G. 입력 경로 — 52501 · PX4 표준"),
    ("H", "docs/GEOMETRY.md", "docs/", "부록 H. 부착 기하"),
    ("I", "docs/REPO_SCOPE.md", "docs/", "부록 I. 저장소 공개 범위"),
    ("J", "patch/README.md", "patch/", "부록 J. 52501 패치 적용 절차"),
    ("K", "vendor/README.md", "vendor/", "부록 K. 제3자 저장소 사본"),
    ("L", "TASKS.md", "", "부록 L. 작업 원장 — 기준선 수치 · 잔여작업"),
    ("M", "out/SWEEP_COMM.md", "out/",
     "부록 M. 통신 스윕 — 주기 · 지연 · 손실"),
    ("N", "out/TASKS_prev_20261122.md", "out/",
     "부록 N. 작업 원장 (2026-11-22 스냅샷, 보존용)"),
]
MANUAL_DIR = os.path.join(HERE, "manual")
HTML = os.path.join(MANUAL_DIR, "_harness_sitl.html")
EQHTML = os.path.join(MANUAL_DIR, "_eqsheet.html")
PDF = os.path.join(MANUAL_DIR, "harness_sitl.pdf")
PNG_DIR = os.path.join(MANUAL_DIR, "check")
KATEX = "vendor/katex"          # HTML 기준 상대경로

CHROME = [
    r"C:\Program Files\Google\Chrome\Application\chrome.exe",
    r"C:\Program Files (x86)\Google\Chrome\Application\chrome.exe",
    r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
]

CSS = """
@page { size: A4; margin: 18mm 15mm 16mm 15mm; }
* { box-sizing: border-box; }
body { font-family: "Malgun Gothic", "Noto Sans CJK KR", sans-serif;
       font-size: 9.6pt; line-height: 1.55; color: #1a1a1a; margin: 0; }
code, pre, kbd { font-family: Consolas, "Courier New", monospace; font-size: 8.8pt; }
code { background: #f2f4f7; padding: 0 2px; }
pre { background: #f7f8fa; border: 1px solid #dfe3e8; padding: 7px 9px;
      white-space: pre-wrap; word-break: break-all; page-break-inside: avoid; }
pre code { background: none; padding: 0; }
h1 { font-size: 17pt; margin: 0 0 4px 0; color: #1f3a5f; line-height: 1.3; }
h2 { font-size: 12.5pt; margin: 18px 0 6px 0; padding-bottom: 3px;
     border-bottom: 1.2px solid #1f3a5f; color: #1f3a5f;
     page-break-after: avoid; page-break-inside: avoid; }
h2.appendix { page-break-before: always; }
.pagebreak { page-break-after: always; height: 0; }
h3 { font-size: 10.6pt; margin: 12px 0 4px 0; color: #22364d;
     page-break-after: avoid; }
h4 { font-size: 9.9pt; margin: 10px 0 3px 0; color: #22364d;
     page-break-after: avoid; }
p, li { margin: 3px 0; }
ul { padding-left: 17px; margin: 4px 0; }
.docdate { font-size: 9pt; color: #555; margin: 0 0 10px 0; }
table { border-collapse: collapse; width: 100%; margin: 7px 0 10px 0;
        font-size: 8.9pt; page-break-inside: avoid; table-layout: fixed; }
th, td { border: 0.8px solid #b9c2cc; padding: 3.2px 5px; text-align: left;
         vertical-align: top; word-break: break-word; overflow-wrap: anywhere; }
th { background: #e9eef5; color: #1f3a5f; font-weight: 600; }
img { max-width: 100%; height: auto; display: block; margin: 8px auto 2px auto;
      page-break-inside: avoid; }
em { color: #333; font-style: normal; }
p > em:only-child { display: block; text-align: center; font-size: 8.8pt;
                    color: #444; margin: 0 0 12px 0; }
hr { border: none; border-top: 0.8px solid #cfd6de; margin: 14px 0; }
a { color: #1f3a5f; text-decoration: none; }
figure, .figwrap { page-break-inside: avoid; }
.eqblk { margin: 9px 0 10px 0; text-align: center;
         page-break-inside: avoid; overflow-x: hidden; }
.katex { font-size: 1.02em; }
.katex-display { margin: 0; }
td .katex, th .katex, li .katex, p .katex { font-size: 0.98em; }
"""

# KaTeX 는 `\(...\)` · `\[...\]` 만 본다. 본문의 `$` 는 건드리지 않는다.
KATEX_HEAD = (
    '<link rel="stylesheet" href="%s/katex.min.css">\n'
    '<script defer src="%s/katex.min.js"></script>\n'
    '<script defer src="%s/contrib/auto-render.min.js"></script>\n'
    % (KATEX, KATEX, KATEX))
KATEX_BOOT = """
<script>
document.addEventListener("DOMContentLoaded", function () {
  renderMathInElement(document.body, {
    delimiters: [
      {left: "\\\\[", right: "\\\\]", display: true},
      {left: "\\\\(", right: "\\\\)", display: false}
    ],
    throwOnError: false,
    errorColor: "#cc0000",
    strict: false
  });
  document.title = "harness_sitl (rendered)";
});
</script>
"""


def esc(s: str) -> str:
    """KaTeX 는 textContent 를 읽으므로 escape 해도 원문이 복원된다."""
    return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def demote_appendix(md: str, L: str = "A", pfx: str = "docs/") -> str:
    """MD 를 부록 L 로: h1 제거, 번호를 L.n 으로, 한 단계 낮춤."""
    out = []
    for ln in md.split("\n"):
        if ln.startswith("# "):
            continue                                    # h1 은 부록 제목으로 대체
        if ln.startswith("- 작성: 충남대학교"):
            continue                                    # 작성자 표기는 첫 장에만
        m = re.match(r"^### (\d+)\.(\d+)\s+(.*)$", ln)
        if m:
            out.append("#### %s.%s.%s %s" % ((L,) + m.groups()))
            continue
        m = re.match(r"^## (\d+)\.\s+(.*)$", ln)
        if m:
            out.append("### %s.%s %s" % ((L,) + m.groups()))
            continue
        # 부록 안의 상대링크는 docs/ 기준이므로 README 기준으로 옮긴다
        ln = re.sub(r"\]\((?!https?:|file:|#|\.\./)([^)]+)\)",
                    lambda m: "](" + pfx + "%s)" % m.group(1), ln)
        out.append(ln)
    return "\n".join(out)


def protect(md: str):
    """펜스코드 · 인라인코드 · 수식을 영숫자 치환자로 빼낸다."""
    store = {}
    n = [0]

    def put(kind, text):
        n[0] += 1
        key = "ZZ%s%04dZZ" % (kind, n[0])
        store[key] = text
        return key

    md = re.sub(r"```.*?```", lambda m: put("CODE", m.group(0)),
                md, flags=re.S)
    md = re.sub(r"`[^`\n]+`", lambda m: put("TICK", m.group(0)), md)
    md = re.sub(r"\$\$(.+?)\$\$", lambda m: put("MBLK", m.group(1)),
                md, flags=re.S)
    md = re.sub(r"(?<!\$)\$([^$\n]+?)\$(?!\$)",
                lambda m: put("MINL", m.group(1)), md)
    return md, store


def fence_html(text: str) -> str:
    """펜스코드 원문을 pre/code 블록으로 바꾼다."""
    lines = text.split(chr(10))
    if lines and lines[0].lstrip().startswith(chr(96) * 3):
        lines = lines[1:]
    if lines and lines[-1].lstrip().startswith(chr(96) * 3):
        lines = lines[:-1]
    return "<pre><code>" + esc(chr(10).join(lines)) + "</code></pre>"


def restore(body: str, store: dict):
    """치환자를 되돌린다. 수식은 KaTeX 구분자로 감싼다."""
    blk = inl = 0
    for key in list(store):
        if key.startswith("ZZCODE"):
            body = body.replace("<p>" + key + "</p>", key)
    for key, text in store.items():
        if key.startswith("ZZMBLK"):
            blk += 1
            body = body.replace(
                "<p>%s</p>" % key,
                '<div class="eqblk">\\[%s\\]</div>' % esc(text))
            body = body.replace(key, '<div class="eqblk">\\[%s\\]</div>'
                                % esc(text))
        elif key.startswith("ZZMINL"):
            inl += 1
            body = body.replace(key, "\\(%s\\)" % esc(text))
        elif key.startswith("ZZTICK"):
            body = body.replace(key, "<code>" + esc(text[1:-1]) + "</code>")
        elif key.startswith("ZZCODE"):
            body = body.replace(key, fence_html(text))
        else:
            body = body.replace(key, text)
    return body, blk, inl


def build_html():
    import markdown

    md = io.open(SRC, encoding="utf-8").read()

    # 첫 h1 바로 아래에 작성일 한 줄 (스킬 규칙)
    date = time.strftime("%Y-%m-%d")
    lines = md.split("\n")
    for i, ln in enumerate(lines):
        if ln.startswith("# "):
            lines.insert(i + 1, "")
            lines.insert(i + 2, "<p class=\"docdate\">작성일 %s</p>" % date)
            break
    md = "\n".join(lines)

    # 첫 장이 실행 순서만 담도록 구성도 앞에서 쪽을 끊는다 (PDF 전용)
    pb = "<div class=\"pagebreak\"></div>"
    img1 = "![시스템 구성](docs/img/diagram_overview.svg)"
    if img1 in md and pb not in md:
        md = md.replace(img1, pb + "\n\n" + img1, 1)

    # 목차에 부록 전부 추가 (README 본문은 건드리지 않는다 — PDF 전용)
    toc_last = "10. [확인 필요 사항](#10-확인-필요-사항)"
    toc = [toc_last]
    for i, (L, rel, pfx, title) in enumerate(APXS, 11):
        toc.append("%d. [%s](#apx-%s)" % (i, title, L.lower()))
    if toc_last in md:
        md = md.replace(toc_last, "\n".join(toc), 1)

    # 부록 붙이기 — 저장소의 모든 MD
    nap = 0
    for L, rel, pfx, title in APXS:
        path = os.path.join(ROOT, rel.replace("/", os.sep))
        if not os.path.isfile(path):
            print("[md  ] 부록 %s 원본 없음: %s" % (L, rel))
            continue
        apx = demote_appendix(io.open(path, encoding="utf-8").read(), L, pfx)
        md = (md + "\n\n<h2 class=\"appendix\" id=\"apx-" + L.lower() + "\">"
              + title + "</h2>\n\n" + apx + "\n")
        nap += 1
    print("[md  ] 부록 %d 편 바인드" % nap)

    md, store = protect(md)
    body = markdown.markdown(
        md, extensions=["tables", "fenced_code", "sane_lists", "attr_list"])
    body, nblk, ninl = restore(body, store)

    # 상대 이미지 경로 -> file:/// 절대경로
    def fix(m):
        src = m.group(2)
        if src.startswith(("http", "file:", "data:")):
            return m.group(0)
        p = os.path.normpath(os.path.join(ROOT, src.replace("/", os.sep)))
        return "%s%s%s" % (m.group(1), "file:///" + p.replace("\\", "/"),
                           m.group(3))

    body = re.sub(r'(<img[^>]*\ssrc=")([^"]+)(")', fix, body)

    html = ("<!DOCTYPE html>\n<html lang=\"ko\"><head>"
            "<meta charset=\"utf-8\">"
            "<title>harness_sitl</title>\n%s<style>%s</style></head><body>\n"
            "%s\n%s</body></html>\n" % (KATEX_HEAD, CSS, body, KATEX_BOOT))
    if not os.path.isdir(MANUAL_DIR):
        os.makedirs(MANUAL_DIR)
    io.open(HTML, "w", encoding="utf-8").write(html)
    print("[md  ] 수식 블록 %d · 인라인 %d" % (nblk, ninl))
    return nblk, ninl


def build_eqsheet():
    """수식 QA 용: LaTeX 원문과 렌더 결과를 나란히 둔 점검표."""
    rows = []
    srcs = [("README.md", SRC)]
    for L, rel, pfx, title in APXS:
        p = os.path.join(ROOT, rel.replace("/", os.sep))
        if os.path.isfile(p):
            srcs.append((os.path.basename(rel), p))
    for name, path in srcs:
        src = io.open(path, encoding="utf-8").read()
        src = re.sub(r"```.*?```", "", src, flags=re.S)
        # 인라인코드 안의 셸 변수는 수식이 아니다
        tick = chr(96)
        src = re.sub(tick + "[^" + tick + "]+" + tick, "", src)
        for m in re.finditer(r"\$\$(.+?)\$\$", src, flags=re.S):
            rows.append((name, "block", m.group(1)))
        for m in re.finditer(r"(?<!\$)\$([^$\n]+?)\$(?!\$)", src):
            rows.append((name, "inline", m.group(1)))

    tr = []
    for i, (fn, kind, tex) in enumerate(rows, 1):
        d = ("\\[%s\\]" % esc(tex)) if kind == "block" \
            else ("\\(%s\\)" % esc(tex))
        tr.append("<tr><td>%d</td><td>%s<br>%s</td>"
                  "<td><pre>%s</pre></td><td>%s</td></tr>"
                  % (i, fn, kind, esc(tex).strip(), d))
    html = ("<!DOCTYPE html>\n<html lang=\"ko\"><head><meta charset=\"utf-8\">"
            "<title>eqsheet</title>\n%s<style>%s\nbody{font-size:9pt}"
            "td{vertical-align:top}pre{font-size:7.2pt;white-space:pre-wrap;word-break:break-all;margin:0}"
            "table.eqt{table-layout:auto;width:100%%}table.eqt td:last-child{overflow-wrap:anywhere}table.eqt .katex{font-size:0.9em}table.eqt .katex-display{margin:0.2em 0;text-align:left}table.eqt .katex-display>.katex{text-align:left}"
            "</style></head><body>\n"
            "<table class=\"eqt\"><tr><th style=\"width:3%%\">#</th>"
            "<th style=\"width:11%%\">출처</th>"
            "<th style=\"width:28%%\">LaTeX 원문</th>"
            "<th style=\"width:58%%\">KaTeX 렌더</th></tr>\n%s</table>\n%s"
            "</body></html>\n"
            % (KATEX_HEAD, CSS, "\n".join(tr), KATEX_BOOT))
    io.open(EQHTML, "w", encoding="utf-8").write(html)
    print("[eq  ] 점검표 %d 식" % len(rows))
    return len(rows)


def chrome() -> str:
    for c in CHROME:
        if os.path.isfile(c):
            return c
    raise RuntimeError("Chrome/Edge 실행파일을 찾지 못했다")


def run(args, timeout=240):
    p = subprocess.Popen(args, stdout=subprocess.PIPE,
                         stderr=subprocess.STDOUT)
    out, _ = p.communicate(timeout=timeout)
    return p.returncode, (out or b"").decode("utf-8", "replace")


def main() -> int:
    nblk, ninl = build_html()
    neq = build_eqsheet()
    url = "file:///" + HTML.replace("\\", "/")
    equrl = "file:///" + EQHTML.replace("\\", "/")
    exe = chrome()
    if not os.path.isdir(PNG_DIR):
        os.makedirs(PNG_DIR)

    rc, log = run([exe, "--headless=new", "--disable-gpu", "--no-sandbox",
                   "--no-pdf-header-footer", "--virtual-time-budget=30000",
                   "--print-to-pdf-no-header",
                   "--print-to-pdf=" + PDF, url])
    ok = os.path.isfile(PDF) and os.path.getsize(PDF) > 20000
    print("[pdf ] rc=%d size=%s ok=%s" % (
        rc, os.path.getsize(PDF) if os.path.isfile(PDF) else -1, ok))
    if log.strip():
        print("[log ] " + log.strip()[:300])

    # 수식 깨짐 자동검사: 렌더된 DOM 에 katex-error / 원문 역슬래시가 남는지
    dom = os.path.join(PNG_DIR, "dom.txt")
    rc3, dumped = run([exe, "--headless=new", "--disable-gpu", "--no-sandbox",
                       "--virtual-time-budget=30000", "--dump-dom", url])
    io.open(dom, "w", encoding="utf-8").write(dumped)
    # 본문·코드에 katex-error 라는 글자가 있을 수 있으므로 KaTeX 가 실제로 그린 span 만 센다
    nerr = dumped.count('<span class="katex-error"')
    nmath = dumped.count("class=\"katex\"")
    print("[qa  ] katex span %d · katex-error %d · 식 %d (블록 %d + 인라인 %d)"
          % (nmath, nerr, neq, nblk, ninl))

    # 눈으로 볼 수 있게 전체 PNG 2 장 (본문 / 수식 점검표)
    for tag, u, wh in (("page_full", url, "794,1123"),
                       ("eqsheet", equrl, "1700,6000")):
        shot = os.path.join(PNG_DIR, tag + ".png")
        rc2, _ = run([exe, "--headless=new", "--disable-gpu", "--no-sandbox",
                      "--window-size=" + wh, "--hide-scrollbars",
                      "--virtual-time-budget=30000",
                      "--screenshot=" + shot, u])
        print("[png ] %s rc=%d size=%s" % (
            tag, rc2, os.path.getsize(shot) if os.path.isfile(shot) else -1))

    return 0 if (ok and nerr == 0) else 1


if __name__ == "__main__":
    sys.exit(main())
