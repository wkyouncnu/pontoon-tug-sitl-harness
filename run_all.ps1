# run_all.ps1 - harness_sitl 전체 재현 (6 단계)
#
#   .\run_all.ps1                 전체 실행 (실패하면 그 자리에서 중단)
#   .\run_all.ps1 -Only sitl      한 단계만 실행
#   .\run_all.ps1 -List           단계 목록만 출력
#
# 로그는 out\run_all_<단계>.txt 에 남는다. 실패 시 꼬리 20 줄을 화면에 보인다.

param(
    [string]$Only = "",
    [switch]$List
)

$ErrorActionPreference = "Stop"
$Here = Split-Path -Parent $MyInvocation.MyCommand.Path
$Py = Join-Path $Here "..\plan_pontoon_rl\.venv\Scripts\python.exe"
$OutDir = Join-Path $Here "out"

$Steps = @(
    @{ name = "dialect";  desc = "MAVLink 다이얼렉트 생성 (52501 포함)";
       args = @("mavlink\gen_dialect.py") },
    @{ name = "onnx";     desc = "sb3 체크포인트 -> ONNX 내보내기";
       args = @("policy\export_onnx.py") },
    @{ name = "diagrams"; desc = "보고서 그림 4 장 생성";
       args = @("docs\img\src\make_diagrams.py") },
    @{ name = "tests";    desc = "단위 - 동일코드 - 안전 - inproc 합격판정";
       args = @("tests\run_tests.py") },
    @{ name = "sitl";     desc = "SITL 합격판정 20 에피소드 (MAVLink over UDP)";
       args = @("sitl\run_sitl.py", "--episodes", "20", "--speed", "1",
                "--dr", "off", "--seed0", "1000") },
    @{ name = "manual";   desc = "README -> A4 PDF 매뉴얼";
       args = @("docs\make_manual.py") }
)

function Show-Steps {
    Write-Host "단계 목록"
    foreach ($s in $Steps) {
        Write-Host ("  {0,-9} {1}" -f $s.name, $s.desc)
    }
}

if ($List) { Show-Steps; exit 0 }

if (-not (Test-Path $Py)) {
    Write-Host "[FAIL] 인터프리터가 없다: $Py" -ForegroundColor Red
    Write-Host "       plan_pontoon_rl\.venv 가 있어야 한다."
    exit 1
}
if (-not (Test-Path $OutDir)) { New-Item -ItemType Directory $OutDir | Out-Null }

$Run = @($Steps)
if ($Only -ne "") {
    $Run = @($Steps | Where-Object { $_.name -eq $Only })
    if ($Run.Count -eq 0) {
        Write-Host "[FAIL] 그런 단계가 없다: $Only" -ForegroundColor Red
        Show-Steps
        exit 1
    }
}

$i = 0
$t0all = Get-Date
foreach ($s in $Run) {
    $i++
    $log = Join-Path $OutDir ("run_all_" + $s.name + ".txt")
    Write-Host ""
    Write-Host ("[{0}/{1}] {2} - {3}" -f $i, $Run.Count, $s.name, $s.desc) -ForegroundColor Cyan
    Write-Host ("        log: {0}" -f $log)

    $t0 = Get-Date
    Push-Location $Here
    & $Py $s.args 2>&1 | Out-File -FilePath $log -Encoding utf8
    $rc = $LASTEXITCODE
    Pop-Location
    $el = [int]((Get-Date) - $t0).TotalSeconds

    if ($rc -eq 0) {
        Write-Host ("        PASS  ({0}s)" -f $el) -ForegroundColor Green
    }
    else {
        Write-Host ("        FAIL  (rc={0}, {1}s)" -f $rc, $el) -ForegroundColor Red
        Write-Host "        --- 로그 꼬리 20 줄 ---"
        Get-Content $log -Tail 20 | ForEach-Object { Write-Host ("        " + $_) }
        Write-Host ""
        Write-Host "중단함. 위 단계를 고친 뒤 -Only $($s.name) 으로 다시 돌린다." -ForegroundColor Red
        exit 1
    }
}

$elall = [int]((Get-Date) - $t0all).TotalSeconds
Write-Host ""
Write-Host ("전체 {0} 단계 PASS ({1}s)" -f $Run.Count, $elall) -ForegroundColor Green
Write-Host "합격판정: out\SITL_RUN.json - 매뉴얼: docs\manual\harness_sitl.pdf"
exit 0
