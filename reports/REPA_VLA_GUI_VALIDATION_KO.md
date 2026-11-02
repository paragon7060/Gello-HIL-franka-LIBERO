# GELLO Studio GUI 검증 — 2026-11-02

`REPA-VLA`의 기존 native Position 연결 위에 영어 Qt6 수집기를 추가했다.
기존 검증 기록은 [REPA_VLA_VALIDATION_KO.md](REPA_VLA_VALIDATION_KO.md)에 보존한다.
이번 보고의 대상은 GUI·수동 확인·재시도 흐름이다.

## 구현 결과

- 초기 native 장면, 검색 가능한40과제/shot1–10 목록, 수집 버튼과 시도 이력을 한 창에 배치했다.
- 문구를 영어로 통일하고 Qt6 폰트·다크 배경·청록색 주요 버튼을 사용했다.
- Motor assist/Start collection/Pause/Stop & review가 기존 A6/session 루프에 연결된다.
- 이미 가까운 자세의 Motor assist는 수집을 자동 시작하지 않고 별도 Start 클릭을 기다린다.
- 실패/중단 원본을 보존하고 Retry shot으로 같은 task/init에 새 시도를 준비한다.
- 성공 후보를 Confirm & next로 확인한 뒤에만 accepted/export에 포함한다.
- 기존 채택 슬롯을 재시도해도 새 확인 전에는 이전 채택을 유지한다. 확인 이력은 append-only
  hash chain이며 각 확인이 원본5개 파일 SHA를 고정한다. 원본은 덮어쓰지 않는다.
- 확인 시 큰 NPZ 검증과 SHA 계산은 GUI 밖의 worker thread에서 수행하고 SHA는 streaming으로 읽는다.
- 단일 pool lock으로 collection/confirmation을 직렬화한다. 장치 설정은 사용 중 변경하지 못한다.
- GUI는 유한 child 하나를 소유하며 종료 시 원래 force 해제·저장 경로를 기다린다.
- legacy OpenCV GUI와 읽기 전용 Dynamixel allow-list, 원래 A6 motor register 제한을 유지한다.

## 자동 검사

실제 사용 Python3.10 환경에서 **94 passed, 0 failed, 0 skipped**를 확인했다.
기존 실험 테스트와 새 confirmation/export gate, 원본/확인 변조 감지, frozen pool,
GUI IPC/phase gate, 단일 child 소유권, 모의 수집→검토→확인→export를 포함한다.
추가 Qt offscreen 검사에서40과제/10shot, 영어 문구, 확인 버튼과 다음 shot 이동을 확인했다.
near-pose Assist→별도 Start 경로도 모의 motor transport에서 write0/physics step0으로 확인했다.
Ruff(`src tests scripts`)와 `git diff --check`도 통과했다.

JUnit 원본은 `runs/collector_gui_validation_20261102_final_tests/pytest.xml`에 보존한다.
기계 판독 요약은 [REPA_VLA_GUI_validation_20261102.json](REPA_VLA_GUI_validation_20261102.json)이다.

## 실제 데스크톱 시각 검사

두 검사를 별도 테스트 pool에서 순서대로 실행하고 PNG를 직접 검토했다.

| 검사 | 결과 | 로컬 evidence |
| --- | --- | --- |
| 합성 실패/성공 영상 검토 | 재생·seek, 실패 보존, Confirm & next, 다음 shot 이동 통과 | `runs/collector_gui_validation_20261102_final_review/` |
| 공식 Spatial00/init0 장면 | 실제 native 초기 장면 표시, motor 버튼 비활성, child 정상 종료 통과 | `runs/collector_gui_validation_20261102_final_native/` |

후자의 source는 pinned Pro commit `eafdb809426b13153aa1e4c42d6601844217dfec`와 공식 자산이다.
화면에 보인 초기 장면은 collection action0이며 성공 demo로 세지 않는다.
합성 검사의 accepted1도 QA fixture일 뿐 실제400개 pool에 포함하지 않는다.

이 데스크톱에서는 Mesa software EGL device 선택과 glfw/EGL backend 조합이 실패했다.
해당 실패 기록을 로컬에 보존한 뒤 **MUJOCO_GL=glx/PYOPENGL_PLATFORM=glx**,
`LIBGL_ALWAYS_SOFTWARE=1`, `__GLX_VENDOR_LIBRARY_NAME=mesa` 조합으로 native preview를 확인했다.
설치된 robosuite source는 변경하지 않았다. Qt6는 OpenCV Qt5 plugin 경로와 분리한다.

PNG·원본 로그·장치별 경로·calibration·영상은 Git에 올리지 않는다.

## 물리 검증 경계

이번 GUI 검사에는 실제 serial 장치를 사용하지 않았고 **실제 motor register write0**이다.
실물 A6 motor assistance와 현재 조립의 calibration/sign/offset 검증은 아직 수행하지 않았다.
실제 성공 시연은 **0/400**이며 학습 job도 실행하지 않았다.
기존 N/W 실험, 게시 결과와 PAUSED된 heartbeat를 재개하지 않았다.

사용법: [English GELLO Studio guide](../docs/collector-studio.md),
[40과제 수집과 k-shot 실험](../docs/position40-pool10.ko.md).
