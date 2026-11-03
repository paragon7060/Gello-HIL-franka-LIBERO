# 과제별 참고 영상과 GUI 검증 — 2026-11-03

Position 40과제 각각에 원본 LIBERO 시연 영상 1개를 연결했다. 과제를 선택하면
오른쪽 **Task example** 탭의 영상이 바뀐다. **Play example / Replay / Loop /
0.5x·1x·1.5x / Enlarge**를 제공한다. 확대 창도 과제 변경을 따라가며 닫을 때
디코더와 timer를 정리한다. 손목 카메라는 같은 영역의 별도 탭으로 유지한다.

참고 영상은 수집 화면 및 시도 검토 영상과 별도 디코더·timer를 사용한다.
재생 버튼은 worker나 모터 명령을 호출하지 않는다. 이전 Qt/Shiboken 6.8.3 고정과
None 참조 손실 사전 검사는 그대로 유지했다.

## 출처와 의미

- 원본: `yifengzhu-hf/LIBERO-datasets`
- 고정 revision: `f13aa24a3da8c43c7225569f28c562979fa0e35a`
- 선택 규칙: 정확히 대응하는 suite/task 이름의 `demo_0` 하나씩. task 번호로 추정하지 않는다.
- HDF5 과제 이름 및 마지막 done/reward를 검사한다. 다른 episode로 자동 대체하지 않는다.
- 기존 로컬 HDF5 7개는 고정 원본의 전체 SHA256과 일치함을 확인했다.
- 나머지 33개는 HTTPS Range로 필요한 episode만 읽었다. 원본 revision/파일 크기는
  고정하지만, 부분 읽기를 전체 HDF5 SHA 검증으로 표현하지 않는다.
- 원본 OpenGL 영상은 세로 반전만 해 upright/unmirrored 영상으로 저장한다.
- 40개 MP4의 합계는 6,872,060 bytes이며 각 MP4 SHA256을 manifest에 기록했다.

원본 시연은 128×128이므로 확대해도 세부 정보가 늘지는 않는다. **원본 과제의
목표 동작 안내이며 Position의 물체 배치는 다를 수 있다.** 이 문구를 패널 상단에
항상 표시한다. 참고 영상은 새 GELLO support/query 시연이나 400-demo 수집 성과로
계산하지 않으며 pool의 확인·학습 export에 들어가지 않는다.

## 검증

- 전체 회귀: **131 passed**. 이후 공개 원본 checksum catalog 검사 포함 참고 영상
  테스트 **14 passed**. Ruff 및 `git diff --check` 통과.
- 실제 준비된 40개 MP4: 정확한 task 매칭/SHA, FPS/프레임 수, 첫·중간·끝 프레임 디코딩 통과.
- 40개 중간 프레임 contact sheet와 실제 Qt 초기 장면/확대 창을 시각 확인했다.
  패널 버튼 겹침·잘림을 수정하고 화면 내 좌표 검사도 통과했다.
- 실제 초기 장면 옆에서 참고 영상을 재생하고 확대 창을 열었다. 이 QA는 calibration과
  실제 장치 연결 없이 CPU 렌더링으로 수행했다.
- 합성 시도 영상과 참고 영상의 동시 재생, Confirm→다음 shot, 실패 기록 보존 통과.
- 참고 영상 재생 중 Stop→저장→검토 **5회**, GUI poll **1,000회** 통과.
  합성 aborted 원본 5개와 검토 영상이 보존됐고 worker가 정상 정리됐다.
  None 참조 수는 11,241→11,256으로 누적 감소가 없었다.
- QA의 실제 serial 접근과 모터 register write는 모두 0이다. 실물 수집 검증을 뜻하지 않는다.

로컬 근거는 Git에서 제외된 `runs/task_example_qa_20261103/`의
`assets-validation.json`, `pytest.xml`, `native-layout-final/validation.json`,
`review/validation.json`, `stop-stress/validation.json` 및 화면 PNG에 있다.
영상과 source manifest는 별도의 `runs/task-examples/`에 보관한다.

다른 컴퓨터의 준비/실행 절차는 [GUI 문서](../docs/collector-studio.md#watch-the-task-before-collecting)에 있다.
