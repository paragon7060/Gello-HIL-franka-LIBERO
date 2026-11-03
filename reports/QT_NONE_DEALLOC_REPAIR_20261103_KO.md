# GUI none_dealloc 코어 덤프 수정 — 2026-11-03

사용 환경은 CPython3.10.14 / PySide6-Qt6-Shiboken6.12.0이었다.
`poll()`의 QLabel.setText 또는 QWidget.setEnabled 실행 중
`Fatal Python error: none_dealloc: deallocating None`으로 GUI가 종료됐다.
이 문서는 base commit `2d832c63ba876ea221e3eea232e7a954e793a373` 위의 수정 근거다.

## 원인 재현

장치나 시뮬레이터 없이 같은 Python에서 Qt 버튼/레이블만 만들고
`sys.getrefcount(None)`을 측정했다. `setEnabled`, `setText`, `setObjectName`을
각200번 호출하면 각각 **-200**이었다. Qt의 void 함수가 반환하는 None의 참조가
누적해서 소모된다. 최종적으로 None의 참조 수가0이 되면 CPython이 강제 중단한다.

Stop 버튼만의 분기 오류가 아니므로, 마지막 Python stack의 줄 번호가 달라도 같은
누적 문제로 설명된다. 이번에는 Stop 뒤 버튼을 다시 활성화하는 poll 줄에서 드러났다.
[다른 PySide 사용 프로젝트의 동일 버전 제외 기록](https://github.com/mlgid-project/mlgidGUI/blob/main/pyproject.toml)
도 참고했지만, 이번 원인 판단과 수정 검증은 위의 로컬 직접 재현을 근거로 한다.

기존 GUI 테스트는 짧은 동작만 확인하여 이 누적 native reference 문제를 잡지 못했다.
최초 GUI 의존성 범위를 넓게 둔 부분과 검증 절차를 함께 수정했다.

## 수정

- GUI extra를 **PySide6-Essentials==6.8.3**으로 고정해 같은 Shiboken6.8.3을 설치한다.
- 로컬 실행기는 기존6.12 파일을 보존한 채 별도6.8.3 runtime 디렉터리를 사용한다.
  기존 conda 패키지나 simulator/data/calibration은 교체하지 않았다.
- `qt_runtime.verify_qt_runtime()`이 알려진 위험 조합과 PySide/Shiboken 불일치를
  거부하고, 실제 Qt void64회 호출에서 None 참조가 감소하는지도 검사한다.
  실패하면 pool 생성/scene worker/serial 열기 전에 명확한 오류로 종료한다.
- native 메모리를 덮어쓰거나 None 참조를 임의로 보충하는 우회는 사용하지 않는다.
- CI에 실제 GUI Stop/review 반복 검사를 추가했다. 기존 부모 PID 상실 테스트는
  PID namespace가 초기 PPID1을 반환해도 같은 조건을 검사하도록 가짜 부모 PID를 명시했다.

## 검증

| 검사 | 결과 |
| --- | --- |
| 교체 전6.12.0 setEnabled / setText 각200회 | None 참조 각각-200 |
| 교체 후6.8.3 setEnabled / setText 각100,000회 | None 참조 각각0 변화, 정상 종료 |
| 위험6.12.0을 사용한 GUI startup | 즉시 거부, pool 생성0/serial0 |
| GUI 수집→Stop→저장→영상 검토20회 | 모두 통과, 합성 원본20개 보존 |
| 위 검사의 명시적 poll4,000회 | crash0, child 정상 회수 |
| 전체 pytest | **117 passed, 0 failed, 0 skipped** |
| Ruff / diff whitespace | 통과 |

Stop 검사는 실제 Qt window와 소유된 finite subprocess를 사용했다. child는 합성
fixture만 만들며 실제 serial/GPU/simulator/motor는 사용하지 않았다.
기계 판독 결과는 [QT_NONE_DEALLOC_REPAIR_20261103.json](QT_NONE_DEALLOC_REPAIR_20261103.json)이다.
로컬 상세 근거는 `runs/qt_runtime_repair_20261103/`에 보존한다.

## 중단 시연 원본

사용자 마지막 Stop 시도는 GUI가 종료되기 전에 별도 수집 worker가 저장을 완료했다.
`success=false`, `aborted=true`, `error=null`이며,
action367×7 / encoder367×8 / 양쪽 RGB368×256×256×3 / state368×9 /
sim state368×92 / 관측시간368 / action시간367로 T+1/T 헤더가 일치한다.
attempt.json, trajectory.npz, observations.npz, calibration.json, operator-view.mp4가
모두 존재하고 nonempty다. 파일은 변경하지 않았다. 실패/중단 데이터를 성공 시연으로
바꾸거나 자동 재수집하지 않았다.

이 확인은 파일 보존과 배열 정렬 검사이며, 중단 시연의 성공 인정이나 새 물리 검증을
뜻하지 않는다. 실행은 기존 `start_position40_pool10.sh --apply` 명령을 그대로 사용한다.
