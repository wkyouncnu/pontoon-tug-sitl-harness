#!/usr/bin/env bash
# run_all.sh - harness_sitl 전체 재현 (6 단계)
#
#   ./run_all.sh                전체 실행 (실패하면 그 자리에서 중단)
#   ./run_all.sh --only sitl    한 단계만 실행
#   ./run_all.sh --list         단계 목록만 출력
#
# 로그는 out/run_all_<단계>.txt 에 남는다. 실패 시 꼬리 20 줄을 화면에 보인다.
# Windows Git Bash 와 리눅스 양쪽에서 동작한다.

set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"

if [ -x "../plan_pontoon_rl/.venv/Scripts/python.exe" ]; then
    PY="../plan_pontoon_rl/.venv/Scripts/python.exe"      # Windows
elif [ -x "../plan_pontoon_rl/.venv/bin/python" ]; then
    PY="../plan_pontoon_rl/.venv/bin/python"              # POSIX
else
    echo "[FAIL] 인터프리터가 없다: plan_pontoon_rl/.venv"
    exit 1
fi

NAMES=(dialect onnx diagrams tests sitl manual)
DESCS=(
    "MAVLink 다이얼렉트 생성 (52501 포함)"
    "sb3 체크포인트 -> ONNX 내보내기"
    "보고서 그림 4 장 생성"
    "단위 - 동일코드 - 안전 - inproc 합격판정"
    "SITL 합격판정 20 에피소드 (MAVLink over UDP)"
    "README -> A4 PDF 매뉴얼"
)
ARGS=(
    "mavlink/gen_dialect.py"
    "policy/export_onnx.py"
    "docs/img/src/make_diagrams.py"
    "tests/run_tests.py"
    "sitl/run_sitl.py --episodes 20 --speed 1 --dr off --seed0 1000"
    "docs/make_manual.py"
)

show_steps() {
    echo "단계 목록"
    for i in "${!NAMES[@]}"; do
        printf "  %-9s %s\n" "${NAMES[$i]}" "${DESCS[$i]}"
    done
}

ONLY=""
while [ $# -gt 0 ]; do
    case "$1" in
        --list) show_steps; exit 0 ;;
        --only) ONLY="${2:-}"; shift ;;
        *) echo "[FAIL] 모르는 인자: $1"; show_steps; exit 1 ;;
    esac
    shift
done

IDX=()
if [ -n "$ONLY" ]; then
    for i in "${!NAMES[@]}"; do
        if [ "${NAMES[$i]}" = "$ONLY" ]; then IDX+=("$i"); fi
    done
    if [ ${#IDX[@]} -eq 0 ]; then
        echo "[FAIL] 그런 단계가 없다: $ONLY"
        show_steps
        exit 1
    fi
else
    for i in "${!NAMES[@]}"; do IDX+=("$i"); done
fi

mkdir -p out
N=${#IDX[@]}
K=0
T0ALL=$(date +%s)

for i in "${IDX[@]}"; do
    K=$((K + 1))
    LOG="out/run_all_${NAMES[$i]}.txt"
    echo ""
    echo "[$K/$N] ${NAMES[$i]} - ${DESCS[$i]}"
    echo "        log: $LOG"
    T0=$(date +%s)
    # shellcheck disable=SC2086
    "$PY" ${ARGS[$i]} > "$LOG" 2>&1
    RC=$?
    EL=$(( $(date +%s) - T0 ))
    if [ $RC -eq 0 ]; then
        echo "        PASS  (${EL}s)"
    else
        echo "        FAIL  (rc=$RC, ${EL}s)"
        echo "        --- 로그 꼬리 20 줄 ---"
        tail -20 "$LOG" | sed 's/^/        /'
        echo ""
        echo "중단함. 위 단계를 고친 뒤 --only ${NAMES[$i]} 으로 다시 돌린다."
        exit 1
    fi
done

echo ""
echo "전체 $N 단계 PASS ($(( $(date +%s) - T0ALL ))s)"
echo "합격판정: out/SITL_RUN.json - 매뉴얼: docs/manual/harness_sitl.pdf"
exit 0
