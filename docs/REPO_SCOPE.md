# 저장소 공개 범위 — 포함·제외 목록

- 본 문서는 `harness_sitl` 을 git 저장소로 만들 때 **무엇을 올리고 무엇을 올리지 않는지**를 기술함
- **저장소 생성·push 는 수행하지 않음.** 사용자 승인 전까지 준비만 함
- 실제 규칙은 [`../.gitignore`](../.gitignore) 에 있으며 본 문서는 그 근거임

## 1. 포함 (본 과제 자체 산출물)

| 경로 | 내용 | 근거 |
|---|---|---|
| `nuc/` | production 모듈 (controller · onnx_policy) | 실물 NUC 에 그대로 올라감. 본 과제 산출물 |
| `plant/` · `nodes/` · `iface/` | 플랜트 · inproc 노드 · 전송 어댑터 | 본 과제 산출물 |
| `sitl/` · `shim/` | SITL 프로세스 · rclpy·tug_interfaces 대역 | 본 과제 산출물 |
| `policy/` | sb3 체크포인트 → ONNX 내보내기 | 본 과제 산출물 |
| `tests/` | 수작업 러너 + 시험 모듈 7 종 | 본 과제 산출물 |
| `tools_sweep_comm.py` · `run_harness.py` | 통신 열화 스윕 · inproc 러너 | 본 과제 산출물 |
| `config/*.yaml` | `tug_bridge.yaml`(실주소) · `tug_bridge_loopback.yaml` | ⚠️ §3 참조 |
| `mavlink/usv_tug.xml` · `mavlink/*.msg` | 다이얼렉트 원본 + 52501 신설분 | 52501 은 본 과제 신설. ⚠️ §3 참조 |
| `docs/` (`SITL.md` · `FILE_SWAP.md` · `REPO_SCOPE.md` · `img/`) | 설계 근거 · 그림 생성기 | 본 과제 산출물 |
| `docs/manual/harness_sitl.pdf` | A4 매뉴얼 | README 에서 생성. 배포물 |
| `patch/` | 52501 반영 패치 + 적용 절차 | 본 과제 산출물. **적용·push 는 저장소 소유자 조율 사항** |
| `README.md` · `TASKS.md` | 보고서 · 잔여작업 원장 | 본 과제 산출물 |
| `run_all.ps1` · `run_all.sh` | 6 단계 재현 스크립트 | 본 과제 산출물 |
| `vendor/README.md` | 벤더 사본의 출처·diff 기록 | 사본 자체가 아니라 기록임 |
| `out/ppo_sc3_final.onnx` (78 867 B) | 배포 정책 | 이것이 실물에 올라가는 파일. 재학습 금지 전제상 포함해야 함 |
| `out/SITL_RUN.json` | 합격판정 결과 | 보고서 수치의 근거 |
| `out/SWEEP_COMM.md` | 통신 열화 포락선 | 보고서 §7.4 의 근거 |

## 2. 제외 (재생성 가능)

| 경로 | 이유 |
|---|---|
| `__pycache__/` · `*.pyc` | 생성물 |
| `mavlink/gen/` | `run_all` 1 단계가 다시 만듦 (`gen_dialect.py`) |
| `out/` 의 나머지 전부 | 로그·중간 json·백업(`*.bak`). `run_all` 이 다시 만듦 |
| `out/harness_sc3.mp4` | 대용량 영상. 필요 시 별도 전달 |
| `docs/manual/_harness_sitl.html` · `docs/manual/check/` | PDF 생성 중간물·검사용 PNG |
| `*_steps.zip` | 강화학습 중간 체크포인트. 최종본만 쓰면 됨 |

## 3. ★ 사용자 판단 필요 (CONFIDENTIAL 후보 — 기본은 **제외**)

| 경로 | 무엇인가 | 왜 판단이 필요한가 | 현재 기본값 |
|---|---|---|---|
| `vendor/tug_bridge/*.py` | `JiwoongHa/cooperative-tug-experiment` 의 노드 사본 (`tug_tx` · `mothership_rx` · `mav_link` · `relay_common`) | **타인 저장소 코드임.** 재배포 권한이 확인되지 않음. 다만 이것이 없으면 클론만으로 SITL 을 돌릴 수 없음 | **제외** |
| `simulink/mothership_bypass_v1.slx` · `init_tug_2fixed_bow_experiment.m` | 마더십 Simulink 모델·초기화 스크립트 | 발주처(삼성중공업) 과제 산출물. 공개 범위 미결 | **제외** |
| `config/tug_bridge.yaml` | 실주소 `192.168.68.201/.202/.203`, 포트 `14600–14602`, `14651/14652` | **실험망 내부주소가 그대로 드러남.** 사설대역이라 외부 접속은 불가하나 망 구성이 노출됨 | 포함 (⚠️ 확인 요망) |
| `mavlink/usv_tug.xml` | `52502` · `52503` 은 타 저장소 정의, `52501` 만 본 과제 신설 | 규약 자체가 공동 산출물임 | 포함 (⚠️ 확인 요망) |
| `README.md` §2.2 | 위 실주소가 표로 들어 있음 | 위와 동일 사유 | 포함 (⚠️ 확인 요망) |
| `out/ppo_sc3_final.onnx` | 학습된 정책 가중치 | 과제 핵심 산출물. 공개 저장소면 정책이 그대로 공개됨 | 포함 (⚠️ 공개 저장소면 재검토) |

### 3.1 결정 — 2026-11-22 (사용자)

- **비공개(private) 저장소를 전제함** → 위 표의 ⚠️ 4 항(`config/tug_bridge.yaml` ·
  `mavlink/usv_tug.xml` · `README.md` §2.2 · `out/ppo_sc3_final.onnx`)은 **기본값대로 포함**함
- 기본 **제외** 2 항(`vendor/tug_bridge/*.py` · `simulink/*.slx`·`*.m`)은 **그대로 제외**함.
  재배포 권한·발주처 공개범위는 비공개 저장소라는 사실과 무관하게 미결이기 때문임
- ⚠️ **저장소를 공개로 전환할 경우 위 4 항을 전부 다시 판단해야 함.**
  특히 `out/ppo_sc3_final.onnx` 는 학습된 정책 가중치 자체임

### 3.2 결정 — 2026-11-22 (사용자, 공개 전환)

- 사용자 지시: "다 완료되면 다 github에 올려줘 공개로 올려도 되어" → **공개(public) 저장소**로 확정함
- §3.1 의 비공개 전제는 이 결정으로 **대체**됨. ⚠️ 4 항을 공개 기준으로 다시 판단한 결과가 아래임

| 항목 | 재판단 | 근거 |
|---|---|---|
| `config/tug_bridge.yaml` 내부주소 | **포함** | `192.168.68.0/24` 는 RFC 1918 사설대역임. 공개망에서 라우팅되지 않으므로 실험망 밖에서 도달 불가함. 자격증명·키·좌표 없음 |
| `README.md` §2.2 동일 주소 | **포함** | 위와 같음 |
| `mavlink/usv_tug.xml` | **포함 + 출처 명시** | 48 줄 중 `52502`·`52503` 정의가 `JiwoongHa/cooperative-tug-experiment` 원본임. 그 저장소는 공개이나 **라이선스 파일이 없음** (`gh repo view` → `licenseInfo: null`) → ⚠️ **재배포 권한 미확인**. 실물 체인과 통신하는 데 필수이므로 포함하되 `NOTICE` 에 출처를 적고 MIT 허락범위에서 제외함. **소유자 동의 확인 필요** |
| `out/ppo_sc3_final.onnx` | **포함** | 학습된 정책 가중치 자체임. 다만 보고서의 20/20 을 제3자가 재현하는 유일한 수단이고, 발주처 사양·유체력 계수는 들어 있지 않음 (가중치 78 867 B 뿐). 공개 전제에서 사용자가 승인함 |

#### 3.2.1 공개 기준으로 새로 판단한 항목

| 항목 | 크기 | 처리 | 근거 |
|---|---|---|---|
| `out/sitl_sc3.mp4` | 4.60 MB | **포함** | README §7.7 그림 5 가 참조하는 산출물임. 제외하면 보고서 링크가 깨짐 |
| `out/harness_sc3.mp4` | 1.38 MB | 제외 | 문서에서 참조하지 않는 구버전 영상임 |
| `docs/manual/vendor/katex/` | 612 KB | **포함** | MIT. 인터넷 없이 PDF 재생성이 되어야 함 (`run_all.sh --only manual`) |
| `out/SITL_RUN_px4std.json` · `out/SITL_px4std.txt` | 20 KB | **포함** | `docs/INPUT_PATHS.md` 가 인용하는 근거임 |
| `out/SITL_geomdef.*` · `out/SITL_geomshift.*` | 4 KB | **포함** | `docs/GEOMETRY.md` §6 이 인용하는 근거임 |
| `out/TESTS_*.txt` | 13 KB | **포함** | 시험 통과 로그. 보고서 §7 의 근거임 |
| `out/*.bak` · `out/TASKS_prev_*.md` | 99 KB | 제외 | 작업 중 사본임 |
| `docs/manual/_eqsheet.html` | 21 KB | 제외 | 수식 점검용 중간 산출물임 (재생성됨) |
| `LICENSE` · `NOTICE` | 신규 | 추가 | 공개 저장소 전제임 |

- 라이선스: **MIT**. 저작권 표기는 `2026 윤원근 (충남대학교 자율운항시스템공학과)` 임
- ⚠️ **확인 필요**: 발주처(삼성중공업) 과제 산출물의 공개 허락은 별도 확인 사항임. 본 저장소는
  하네스 코드·시뮬레이션 결과만 담고 발주처 사양서·Simulink 모델(`simulink/*.slx`·`*.m`)은
  제외하므로 공개 가능하다고 판단했으나, 과제 계약상 공개 제약이 있으면 저장소를 비공개로
  돌려야 함

## 4. 제외하면 생기는 결과

| 제외 항목 | 클론 후 못 하는 것 | 우회 |
|---|---|---|
| `vendor/tug_bridge/` | SITL 경로(`run_all --only sitl`) 실행 불가 | 실험 저장소를 따로 클론해 `vendor/tug_bridge/` 에 복사. 절차는 `vendor/README.md` |
| `simulink/*.slx` | 마더십 바이패스 모델 열람 불가 | 하네스 검증에는 불필요 (S6 은 실패로 종결) |
| `mavlink/gen/` | 최초 1 회 `run_all --only dialect` 필요 | pymavlink 만 있으면 재생성됨 |

## 5. 승인 상태

| 행위 | 상태 |
|---|---|
| `git init` · 공개 원격 저장소 생성 · 최초 커밋 · push | **승인됨** (2026-11-22, §3.2) |
| `patch/` 의 패치를 실험 저장소에 적용하거나 PR 로 올리는 것 | **하지 않음 — 저장소 소유자 조율 사항임** |
| `mavlink/usv_tug.xml` 의 `52502`·`52503` 재배포 | ⚠️ **미확인** — `NOTICE` 에 출처를 적었으나 소유자 동의는 받지 않음 |
