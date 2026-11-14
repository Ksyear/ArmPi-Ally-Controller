# ArmPi Ally Controller 화면 디자인 개선 계획 v001

작성일: 2026-11-14 | 대상: `armpi-ally.exe` (Win32 + Direct3D 11 + Dear ImGui 1.92.9b) | 상태: 계획(코드 변경 없음)

이 문서는 디자인 계획서다. 기능·안전 규칙(`docs/SAFETY_KO.md`, `docs/CONTROLS_KO.md`)은 그대로 두고 보이는 것만 바꾼다. 공개 저장소용 문서이므로 주소·계정·기기 이름을 적지 않는다.

**읽은 자료**: `docs/images/drive_mock.png`, `docs/images/arm_mock.png`(둘 다 직접 열어 봄), `src/app/gui_app.cpp`(읽은 시점 약 1,270줄; 읽는 동안 보안 수정이 들어가 줄 번호가 조금 바뀜. 화면 구조는 동일), `src/app/gui_app.hpp`, `src/app/gui_main.cpp`, `docs/CONTROLS_KO.md`, `docs/SAFETY_KO.md`, `docs/ARCHITECTURE.md`, `README.md`, `THIRD_PARTY_NOTICES.md`, `third_party/imgui/imgui.h`·`imconfig.h`(1.92.9b API 확인).

**표기 규칙**: "사실"은 화면 사진·코드·열어 본 웹 자료에서 확인한 것, "판단"은 디자이너로서의 의견이다. 열지 못한 자료는 그렇게 적었다.

---

## 0. 한눈에 보기

- 지금 화면이 촌스러워 보이는 핵심 원인(판단): **모든 것이 같은 모양·같은 무게**다. 상태 칩 12개, 탭 4개, 버튼 30여 개가 전부 "둥근 사각형에 글자"이고, 녹색·파랑·주황 채도 높은 색이 정상 상태에까지 칠해져 있어 위계가 없다. 글자 크기는 거의 한 단계(22 px)이고 숫자는 비례폭이라 값이 바뀔 때마다 흔들린다. 여백은 4/8 격자가 아니고 왼쪽 가장자리가 네 군데(12, 50, 20, 10 px)로 어긋난다.
- 방향: **"조용한 계기판 위의 카메라"**. 회색 단계로 구조를 만들고, 색은 이상 상태와 선택에만 쓴다(ISA-101식). 카메라와 비상정지만 눈에 띄게 둔다. 게임패드 글자 배지(Ⓐ Ⓑ LB)를 한 가지 방식으로 그린다.
- 구현은 ImGui만으로 충분하다. 필요한 것은 `theme.cpp`(토큰·스타일), `widgets.cpp`(배지·링·세그먼트·카드·토스트·OSD 글자), 내장 글꼴(Pretendard + JetBrains Mono + Lucide) 세 가지다.

---

## 1. 현재 디자인 진단

근거는 사진 두 장과 코드다. 줄 번호는 읽은 시점 기준이며 ±20줄 차이가 날 수 있다.

### 1.1 위계·정보 구조

| # | 문제 | 근거(사실) | 왜 문제인가(판단) |
|---|---|---|---|
| H1 | 상단에 같은 크기의 칩 12개가 한 줄 | `draw_top_bar()`: 정상/연결/차체/팔/조종권/rosbridge/패드/로봇 V/Ally %/ACK/영상/속도를 모두 `chip()`으로 그림. 사진에서 녹색 5개·파랑 3개·회색 4개 | 가장 중요한 것(연결됨? 비상정지?)과 덜 중요한 것(fps, Ally %)이 같은 무게. 정상 상태가 전부 "켜진" 녹색이라 이상 상태가 생겨도 눈에 띄지 않는다 |
| H2 | 자리가 모자라면 칩을 조용히 버림 | `chip_limit_x_` 넘는 칩은 `Dummy`로 생략 | 어떤 정보가 사라졌는지 사용자가 모른다 |
| H3 | 속도 상한이 세 곳에 중복 | 상단 칩 "속도 30 %", 조종 탭 슬라이더, 오른쪽 열 30/60/100 버튼 | 한 값에 세 UI. 자리 낭비, 어느 것이 "진짜"인지 불명 |
| H4 | 조종 탭이 진단 정보로 채워짐 | `draw_tab_drive()`: 패킷 수, 늦음, 오류, 루프 Hz, 밀림, 차단 사유 표 | 조종 중 읽을 정보가 아니다. 진단 탭 소관. 반면 조종 탭 아래 약 40 %는 빈 공간(사진) |
| H5 | 팔 탭은 넘쳐서 스크롤 | 사진에 세로 스크롤바, 관절 막대가 잘림, 도움말 문장이 오른쪽에서 잘림("…그 점으로"), 표의 "로봇이 쓰는 목표" 라벨이 190 px 고정 열에서 잘림 | 한 탭은 한 화면에 들어가야 한다. 1080p·7인치에서 스크롤은 조작 중 쓸 수 없다 |
| H6 | 주황이 네 가지 뜻 | `kAccent`: 선택된 탭, 눌린 버튼(`ButtonActive`), 가상 스틱 활성, 팔 켜기 토글, 패드 미니 표시 점, 작업공간 상자(110 알파) | "지금 선택" "지금 누름" "입력 중" "경고에 가까운 영역"이 같은 색 |
| H7 | 파랑도 세 가지 뜻 | `kBlue`: idle 상태 칩, 속도 칩·속도 선택, 그리퍼 버튼, 내 목표 점, 토스트 정보색, 진단 송신 로그 | 상태색과 조작색이 섞임 |

### 1.2 색·대비

사실(계산): 현재 상수(`gui_app.cpp` 상단)와 흰 글자의 WCAG 대비비.

| 바탕 | 값 | 흰 글자 대비 | 판정(본문 4.5:1, 큰 글자 3:1) |
|---|---|---|---|
| `kGreen` | #27AE60 | 2.87 | 실패 |
| `kYellow` | #E6AA1E | 2.07 | 실패(길게 누르기 활성 시 흰 글자) |
| `kAccent` | #FF8C28 | 2.32 | 실패(선택 탭·팔 켬 글자) |
| `kBlue` | #3484E4 | 3.77 | 본문 실패, 큰 글자만 통과 |
| `kRed` | #D63031 | 4.85 | 통과 |
| `kGray` | #4E5460 | 7.61 | 통과 |

- 녹색·노랑·주황 칩과 버튼의 흰 글자는 모두 기준 미달이다. 밝은 바탕에는 어두운 글자를 써야 한다.
- 근접 검정이 세 가지다: 루트 `kDark` #181A1F, 패널 `kPanel` #22252C, 카메라 바탕 #0C0D10, 그리고 `gui_main.cpp` clear color (0.09, 0.10, 0.12). 단계가 아니라 우연이다(판단).
- 색약 고려가 없다: 차체/팔 상태가 색 칩 하나(녹/파/빨/노)로만 구분된다. 글자는 있지만 "정상"과 "경고"가 같은 모양이라 색을 못 보면 훑어볼 수 없다.

### 1.3 글자

- 사실: 기본 22 px(`FontSizeBase = 22`), 제목 26/28/30/34/40 px, 칩은 전부 굵게. 보조 글자는 크기가 아니라 색(`kDim`)으로만 구분.
- 사실: 맑은 고딕을 실행 시 읽음. 숫자가 비례폭이라 `ACK 31/94 ms` → `30/32 ms`처럼 값이 바뀌면 칩 너비와 뒤 칩들이 흔들린다(사진 두 장을 비교하면 ACK 칩 폭이 다르다).
- 판단: 7인치 1080p는 약 315 ppi다(7" 16:9 대각선 계산값, 제조사 ppi 미공개). 22 px은 em 높이 약 1.75 mm다. 비교하면 휴대폰 본문(16 sp ≈ 420 dpi에서 42 px ≈ 2.5 mm)보다 작고, XAG 101의 모바일 규칙(18 px@100 DPI 선형 증가)을 315 DPI로 외삽하면 약 57 px이 "최소 기본"이 된다(2.2). 손에 든 7인치 화면은 휴대폰보다 조금 멀리(35–40 cm) 보므로 본문 26 px(≈2.1 mm), 보조 22 px, 가장 작은 글자 18 px(HUD 단위·배지 글자)로 올리고, 설정에서 글자 배율 1.0–1.3을 고를 수 있게 하는 것을 제안한다.

### 1.4 여백·정렬·모양

- 사실: 왼쪽 가장자리가 상단 칩 12 px, 카메라 이미지 약 50 px(레터박스 중앙 정렬), 오른쪽 열 10 px, 하단 스틱 20 px로 제각각. 카메라 아래에 버튼 두 개와 "수신 중" 글자 때문에 이미지 밑에 어두운 띠가 남는다(사진, `avail_h = size.y - S(64)`).
- 사실: 모서리 반경이 `FrameRounding 10`, `ChildRounding 8`, 칩 8, 비상정지 16, 작업공간 상자 0으로 네 가지. 간격은 `ItemSpacing 10`, `FramePadding 12×8`, 패널 패딩 14×10, 오른쪽 열 10, 하단 20/30으로 4/8 격자가 아니다.
- 사실: 하단 팔 버튼 8개가 4×2 격자(앞·왼·위·손목↑ / 뒤·오른·아래·손목↓)로 축별 짝이 세로로 떨어져 있다. 패드 미니 표시의 "LB"는 원 중심에서 −40 px, "RB"는 −20 px 위치라 좌우가 비대칭이다. 하단 오른쪽 끝 약 150 px이 빈다.
- 판단: 전부 같은 둥근 사각형이라 "탭", "칩", "버튼", "선택 상태"가 모양으로 구분되지 않는다.

### 1.5 터치 크기

- 사실: 버튼 높이 44–56 px, 팔 탭 `InputFloat`의 −/+ 버튼은 글자 높이 기준 약 36 px, 속도 버튼 52 px, 팔 버튼 104×92, 그리퍼 150×92, 비상정지 약 190×190.
- 판단: 315 ppi에서 1 mm ≈ 12.4 px. 44 px ≈ 3.5 mm, 52 px ≈ 4.2 mm. 손가락 끝 접촉면(7–9 mm 권고, 3절 조사 참고)에는 작다. 자주 누르는 것은 88 px(≈7 mm) 이상, 보조는 64 px + 간격 8 px 이상으로 올린다. 비상정지·가상 스틱·팔 버튼은 이미 충분하다.

### 1.6 게임패드 안내

- 사실: 패드 힌트가 글자 괄호로 섞여 있다: "영상 끄기 (X)", "크게 (Y)", "저속 (RB)", "그리퍼 닫기 (RT)", "B · Space", "View 0.8초", 탭은 키보드 "F1–F4"만 표기. 패드로 탭을 바꾸는 입력은 Menu(설정 열기/닫기)뿐이라 탭 버튼의 "F1"은 개발용 힌트가 주 입력 자리에 들어간 것이다.
- 사실: ImGui 내비게이션(키보드·패드)은 의도적으로 꺼져 있다(`io.ConfigFlags &= ~(NavEnableKeyboard | NavEnableGamepad)`, 패드는 로봇 조종에 쓰기 때문). 따라서 "포커스 이동"은 없고 "고정 단축키"만 있다. 안내는 포커스 링이 아니라 **정적 배지**여야 한다.
- 판단: 배지를 한 가지 양식(원 안의 글자, 둥근 사각형 안의 LB/RB)으로 통일하고 라벨 왼쪽 또는 아래 고정 위치에 둔다. 괄호 표기는 없앤다.

### 1.7 카메라 HUD

- 사실: 왼쪽 위 상태 라벨(30 px 굵게, 검정 150 알파 상자), 오른쪽 위 팔 목표 두 줄(22 px, 같은 상자), 왼쪽 아래 주행 벡터 원, 중앙 아래 토스트, 중앙 십자선. 글자에 외곽선·그림자가 없다.
- 판단: 상자형 HUD는 영상을 가린다. 밝은 영상에서 상자 없이 흰 글자만 쓰면 안 읽히므로, OSD 관례대로 **외곽선 글자 + 얇은 가장자리 스크림**으로 바꾼다. 팔 목표 수치는 팔이 꺼져 있을 때 보일 이유가 없다(지금은 항상 표시 + "(팔 꺼짐)" 꼬리표).

### 1.8 일관성·마감

- 관절 막대는 ImGui 기본 `ProgressBar` 색(노랑 채움, 파란 바탕)이라 팔레트 밖이다.
- 표 헤더는 ImGui 기본 `TableHeadersRow` 스타일(어두운 띠)이라 다른 카드와 어울리지 않는다.
- 토스트는 1개만 보이고(`notes_.back()`) 종류별 아이콘이 없다. 색 테두리로만 구분한다.
- 연결 화면의 찾은 로봇 버튼은 한 버튼에 두 줄 텍스트를 넣은 것이라 "카드"로 읽히지 않는다.

---

## 2. 조사 결과 요약

조사는 2026-11-14에 웹 검색·페이지 열람·PDF 추출로 했다. "열어 봄"은 본문을 실제로 읽은 것, "요약만"은 검색 결과 요약만 본 것이다. 열지 못한 자료는 2.5에 모았다. 상용 제품은 화면을 베끼지 않고 원칙만 가져온다. 로고·아이콘·고유 레이아웃은 쓰지 않는다.

### 2.1 로봇 원격 조종 UI

| 사례 | 자료(접근) | 관찰한 사실 | 배울 점 | 우리 앱에 적용 | 가져오지 않을 것 |
|---|---|---|---|---|---|
| Foxglove | [Teleop 패널](https://docs.foxglove.dev/docs/visualization/panels/teleop), [Image 패널](https://docs.foxglove.dev/docs/visualization/panels/image), [설정](https://docs.foxglove.dev/docs/settings) (열어 봄) | 4방향 버튼 + 중앙 Stop, "Stop on release" 옵션. 영상 위 주석은 픽셀 좌표 계층, 오버레이별 불투명도. 색 테마 light/dark/OS, 기본 다크(변경 기록 요약만) | 놓으면 멈춤 + 명시적 정지. 오버레이를 영상과 분리하고 불투명도를 둔다 | HUD 스크림·글자 불투명도를 설정값으로(5.4) | 자유 분할 패널·탭 그룹(7인치 패드 환경에 과함) |
| QGroundControl | [Fly View](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/fly_view/fly_view.html), [Fly Tools](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/fly_view/fly_tools.html), [Toolbar](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/fly_view/fly_view_toolbar.html) (열어 봄) | 위험 동작은 "click and hold until accepted" + chevron. Emergency Stop은 Flight Status 드롭다운 안의 빨간 버튼, press-and-hold. 상태 색 녹/황/적, 메시지 심각도 적/주황/파랑 별도. 영상 패킷 손실 시 지시자 주황 | 확인은 슬라이드가 아니라 **길게 누름 + 진행 표시**. 상태 색과 메시지 색을 분리 | 해제 길게 누르기에 진행 링(6.5). 토스트 색은 상태 색과 다른 계열(info) | 비상정지를 드롭다운에 숨기는 배치 |
| Mission Planner | [GCS 개요](https://ardupilot.org/planner/docs/mission-planner-ground-control-station.html), [Flight Data](https://ardupilot.org/planner/docs/mission-planner-flight-data.html) (열어 봄) | HUD 좌상단 15항목, 좌하단 Quick 탭은 소수 텔레메트리를 **큰 글자**로. 아이콘/텍스트 표시 선택. 중요 메시지는 HUD에도 중복 | 숫자는 적게, 크게 | 링크 카드의 숫자 타일 3개(5.7), 상단 계기 축약 2단계(5.2) | 탭이 많은 마우스형 패널, Arm 버튼이 다른 버튼과 같은 크기 |
| Clearpath IndoorNav/OutdoorNav | [Driving](https://docs.clearpathrobotics.com/docs_indoornav_user_manual/using_indoornav/driving_the_robot), [Assisted teleop](https://docs.clearpathrobotics.com/docs_outdoornav_user_manual/1.0.0/features/assisted_teleoperation) (열어 봄) | 가상 조이스틱 우하단, 전원 후 neutral 해제 필요. 보조 토글이 조이스틱 옆, 경고 문구가 원인까지("OFF due to sensor timeout"). 이 기능은 E-stop이 아님을 명시 | 상태 문구에 원인을 적는다. 소프트웨어 정지와 물리 비상정지를 문구로 구분 | 토스트·상태 줄 문구 규칙("통신 끊김 — 다시 붙는 중"처럼 원인+조치), 6.4의 "소프트웨어 정지" 꼬리표 | — |
| Vizanti / nipple.js teleop | [Vizanti README](https://github.com/MoffKalast/vizanti), [roslibjs 튜토리얼](https://msadowski.github.io/ros-web-tutorial-pt1) (열어 봄) | 가상 스틱: threshold 0.1, 반경 정규화, 25 ms 주기 재전송, 놓으면 (0,0) 1회 전송 | 가상 스틱의 데드존·정규화·즉시 복귀(이미 구현됨) | 가상 스틱 시각은 손 떼면 즉시 중앙(6.7) | 데모용 검은 단색 스틱 |
| Webviz | [README](https://github.com/cruise-automation/webviz) (열어 봄) | 패널별 "?" 도움말 | 도움말 진입점 | 설정 탭의 "조작표" 항목에 배지 범례 | — |
| ROS rqt / rviz | wiki.ros.org, docs.ros.org (봇 차단으로 **열지 못함**) | Clearpath 문서에서 간접: rviz Interact 마커, 입력 우선순위 mux | — | — | 근거 부족으로 참고하지 않음 |
| Boston Dynamics Spot 태블릿 | [User Guide R2.0 PDF](https://www.generationrobots.com/media/spot-boston-dynamics/spot-user-guide-r2.0-va.pdf) 제3자 사본(PDF 추출), 공식 지원 페이지(404) | 우상단 빨간 STOP(모터 전원 차단) + 확인 1단계, 일반 정지는 B 버튼으로 구분. "To stop movement, release fingers from joysticks." 속도 단계 선택(초보 Slow). Touch-to-go: 영상 위 터치는 이동 목표 지정 | 비상정지는 항상 노출·우상단·가장 큼. 정지 종류가 다르면 버튼과 결과 라벨을 다르게. 연속 조종은 스틱, 터치는 의도 입력 | 안전 열 구성 유지(5.5). 팔 그림의 터치 = 목표 지정(이미 그렇다) | STOP = 붕괴라는 의미(팔 로봇엔 다름), 레이아웃 전체 |
| DJI Fly / DJI RC 2 | [Mini 4 Pro 매뉴얼 PDF](https://www.adorama.com/col/productManuals/DJIM4PRO.pdf) p.93–96(PDF 추출), [제3자 가이드](https://dronesgator.com/dji-fly-app-guide) (열어 봄) | 영상 전면 + 얇은 상단 상태바(모드·시스템 상태·배터리 % + 남은 시간·링크·비전·GNSS·설정). 비전 아이콘 정상 **흰색**, 불가 **빨강**. 하단 수치 4개(D·H·속도). 자동 이착륙/RTH는 "press and hold". 물리 Pause 버튼 한 번 = 정지, 길게 = RTH | 아이콘 색 2값(정상 흰/이상 빨강). 큰 동작은 길게 | 상단 계기 아이콘 색 규칙(5.2), 배터리에 전압+막대 | 촬영 UI, 작은 터치 아이콘 밀도, RTH 아이콘 |
| Unitree Go | [매뉴얼 p.28](https://www.manualslib.com/manual/2596669/Unitree-Go-I.html?page=28), [제품 페이지](https://www.unitree.com/mobile/app/go2) (열어 봄, 화면 배치 없음) | 좌상단 녹색 연결 배지(요약만), 물리 컨트롤러=연속 조종, 앱=명령 권장 | 터치/패드 역할 분담 | 이미 같은 분담 | 시각 스타일(근거 없음) |
| Anki Vector | [리뷰](https://www.macrumors.com/review/anki-vector-robot/) (열어 봄) | 공식 앱에 조종 기능 없음 | 참고 가치 낮음 | — | — |

### 2.2 휴대용 게임기·게임패드 UI

| 사례 | 자료(접근) | 관찰한 사실 | 배울 점 | 우리 앱에 적용 | 가져오지 않을 것 |
|---|---|---|---|---|---|
| Steam Deck Big Picture / QAM | Valve 개발자 문서·리뷰 **모두 열지 못함**(404·연결 실패). 패치노트 요약만 | 하단 footer가 상황별 "글리프 + 동사" 힌트 줄(키보드 사용 시 키 글리프로 교체, 슬라이더 미세 조정 힌트). 연결된 컨트롤러 종류에 맞는 글리프 | 힌트는 글리프+동사 짝으로, 고정 위치 | PadBadge + 동사 라벨(6.2). 포커스 색·수치는 미확인이라 쓰지 않음 | Steam 글리프 아트, 소셜 탭 구조 |
| ROG Ally Command Center | [ASUS 팁 글](https://rog.asus.com/uk/articles/rog-ally/15-tips--shortcuts-to-set-up-and-optimize-your-rog-ally/) (열어 봄) | 세로 오버레이 패널에 토글+슬라이더+실시간 모니터(CPU/GPU/FPS/배터리). Armoury Crate SE는 RT 탭 이동, A 실행, LB/RB 탐색 | 모니터는 숫자 몇 개만 | 상단 계기 4개 한도(5.2) | 육각 아이콘·브랜드 색 |
| Xbox Game Bar / UWP 게임패드 가이드 | [Game Bar 위젯 UI](https://learn.microsoft.com/en-us/gaming/game-bar/designguide/widgetui), [Gamepad interactions](https://learn.microsoft.com/en-us/windows/apps/design/input/gamepad-and-remote-interactions), [Designing for TV](https://learn.microsoft.com/en-us/windows/apps/design/devices/designing-for-tv) (열어 봄) | 버튼 상태: Rest / Hover +10 % / Press −10 % / **Focus = Rest + 테두리** / Inactive 50 %. A 선택·B 뒤로 고정, X/Y 가속키, 범퍼는 패널 간 이동. 10-foot 본문 최소 15 epx(200 %에서 30 px), 보조 12 epx, TV 안전 여백 상하 27·좌우 48 epx(≈5 %). B가 뒤로면 화면에 뒤로 버튼을 그리지 않음 | 상태는 밝기 ±10 %와 테두리로. 가속키 배정 관례 | 버튼 Rest/Press 밝기 단계 = surface2/surface3(6.13). B는 비상정지에 쓰이므로 "뒤로"는 쓰지 않는다는 점을 설정 탭 조작표에 명시 | Segoe Xbox 글리프 글꼴(Xbox 전용), Reveal 광원 효과 |
| Xbox Accessibility Guidelines 101/102, XR-022 | [XAG 101](https://learn.microsoft.com/en-us/gaming/accessibility/xbox-accessibility-guidelines/101), [XAG 102](https://learn.microsoft.com/en-us/gaming/accessibility/xbox-accessibility-guidelines/102), [XR-022](https://devdocs.xbox.com/publishing/certification/xr/xr-022) (열어 봄) | 글자 크기는 body height(디센더–어센더)로 측정. 최소: 콘솔 26 px@1080p, PC 18 px@1080p, 모바일 18 px@100 DPI에서 선형 증가. 글리프 안 글자도 같은 기준. 대비 4.5:1/큰 글자 3:1, 영상 같은 불균일 배경은 **가장 낮은 대비 영역** 기준, 해법은 외곽선·불투명도 조절 배경판·흑백 이중선. XR-022: 버튼 이름은 A/B/X/Y·LB/RB/LT/RT·View/Menu(색 규정 없음) | 영상 위 글자는 외곽선. 배지 안 글자도 최소 크기. 버튼 명칭 표준 | OsdText(6.11), fMicro 18 px 하한, 배지 글자 fMicro, 명칭 통일. 글자 배율 설정(FontScaleMain 1.0–1.3)을 선택지로 제안(부록 B) | 면 버튼 색(A 초록 등)을 식별 수단으로 쓰는 것 — 공식 출처 미확인이고 상태색과 충돌 |
| 차량·드론·메카 게임 HUD | 1차 자료 **미발견**(요약만: Elite Dangerous 콕핏 UI 초점 전환 피로, 디제시스 UI 인지 부담) | — | 영상 평면에 바로 그리는 비디제시스 HUD, 전체 화면 점유 패널 최소화 | 이미 그렇다 | 홀로그램·스캔라인·글로우 장식 |

### 2.3 FPV·드론 OSD

| 사례 | 자료(접근) | 관찰한 사실 | 배울 점 | 우리 앱에 적용 | 가져오지 않을 것 |
|---|---|---|---|---|---|
| Betaflight OSD | [공식 OSD 탭 문서](https://www.betaflight.com/docs/wiki/app/osd-tab), [Oscar Liang 가이드](https://oscarliang.com/betaflight-osd) (열어 봄) | 격자 PAL 30×16, HD 53×20. Warnings 요소 하나에 여러 오류를 모음. 미니멀 구성 = 배터리 전압·RSSI·타이머 3개. 알람 임계치에서 값이 깜빡임. "Try not to overcrowd", 요소 겹침 금지. 모서리 배치 규칙은 공식 문서에 **없음**(커뮤니티 관행) | 요소 수를 줄이고, 경고는 한 줄에 모으고, 값만 깜빡임 | HUD 모서리당 최대 2줄, 토스트 상단 중앙 1줄(5.4), warn 값만 색 변경 | 비트맵 각진 글꼴, 비행 모드 약어 |
| DJI/Walksnail/HDZero OSD | [WTFOS 가이드](https://oscarliang.com/fpv-wtf-dji-goggles-osd) (열어 봄), 공식 매뉴얼 요약만 | 요소를 가장자리보다 안쪽으로. 기본 글꼴 단색 흰. 해상도별 글리프 세트 | 흰 단색 기본, 안쪽 여백 | HUD 글자 text1 기본, 상태색 소수 | 60×22 셀 고정, 컬러 장식 글꼴 |
| 영상 위 글자 가독성 | [EBU R 95 PDF](https://tech.ebu.ch/docs/r/r095.pdf) (PDF 추출), [Safe area](https://en.wikipedia.org/wiki/Safe_area_(television)) (열어 봄) | action safe 3.5 %, graphics safe 5 %(1080p에서 좌우 96·상하 54 px). 현대 평판은 오버스캔이 거의 없음 | 안전 여백은 오버스캔이 아니라 시선·손 가림 이유로 재해석 | HUD 안전 여백 32 px(5 % 전체를 쓰면 7인치에서 공간 손실이 큼 — 판단), 밝은 영상에서 최악 대비로 검증(10.3) | 방송용 10 %/20 % 여백 |

### 2.4 Dear ImGui 사례·산업 HMI·접근성

| 사례 | 자료(접근) | 관찰한 사실 | 배울 점 | 우리 앱에 적용 |
|---|---|---|---|---|
| ImGui Gallery #6478 / #7959 / #8333 | [Nin Online HUD](https://github.com/ocornut/imgui/issues/6478#issuecomment-1592346842), [Remedy Northlight](https://github.com/ocornut/imgui/issues/6478#issuecomment-1629343428), [nged](https://github.com/ocornut/imgui/issues/6478#issuecomment-1747288625), [Geargrafx](https://github.com/ocornut/imgui/issues/8333#issuecomment-2628486570), [ImMobile 터치](https://github.com/ocornut/imgui/issues/8333#issuecomment-2798143401), [로봇 굴삭기 UI](https://github.com/ocornut/imgui/issues/7959#issuecomment-2585272203) (댓글 본문을 API로 확인, 이미지는 보지 못함) | 게임 HUD 전체를 ImGui로 그린 사례, "not very default"로 보이게 WPF 앱과 맞춘 에디터, 터치 전용 ImGui 환경, 로보틱스 제어 패널 | ImGui로 기본 모양을 완전히 숨길 수 있다. 다듬어진 사례의 공통점은 커스텀 드로우·일관 간격·글꼴 교체 | 6절 위젯 전부 ImDrawList. 기본 위젯은 스타일로, 모양이 다른 것은 직접 그림 |
| 테마 스레드 #707, ImThemes, Hello ImGui | [#707](https://github.com/ocornut/imgui/issues/707) (89개 댓글 확인), [ImThemes](https://github.com/Patitotective/ImThemes), [hello_imgui](https://github.com/pthom/hello_imgui) (열어 봄) | 거의 모든 테마가 `Colors[]`·Rounding·Padding·글꼴 교체만으로 구성. CorporateGrey("clean and easy on the eyes"), Deep Dark, r-lyeh V3("grey theme with colours sparkled at some places") | 회색 바탕 + 드문 색 포인트가 ImGui 테마 커뮤니티에서도 "정돈돼 보이는" 공식 | ApplyTheme 한 함수(6.13). 그림자·그라디언트·링·외곽선 글자는 ImGuiStyle로 안 되므로 커스텀 |
| ImDrawList·그림자 | master `imgui.h` grep, [이슈 #1329](https://github.com/ocornut/imgui/issues/1329) 요약 | `AddShadowRect`는 master에 **없음**(비공식 브랜치). `AddRectFilledMultiColor`, `PathArcTo`, `AddImageRounded`, 폰트·크기 지정 `AddText` 존재 | 그림자에 의존하지 않는다 | 토큰 `shadow = 없음`(4.3), 스크림은 MultiColor 그라디언트 |
| IconFontCppHeaders | [저장소](https://github.com/juliettef/IconFontCppHeaders) (열어 봄) | FA·Material Symbols·Lucide 등 코드포인트 헤더 제공. 예제: `MergeMode=true, PixelSnapH=true, GlyphMinAdvanceX=iconSize`, 아이콘 크기는 글자의 2/3 권장 | 코드포인트 헤더 생성 방식 | 8.2 병합 설정 |
| ISA-101 / 고성능 HMI | [ISA101 위원회](https://www.isa.org/standards-and-publications/isa-standards/isa-standards-committees/isa101), [Ignition HPHMI 기법](https://docs.inductiveautomation.com/docs/8.1/ignition-modules/vision/common-tasks-in-vision/high-performance-hmitechniques) (열어 봄; 규격 본문은 유료라 미열람) | "Gray-scale colors are used instead of the traditional bright red, green, and blue." 알람은 문제 있을 때만 색 도형, 중요도별 모양+숫자(빨간 사각 1, 노란 삼각 2). 색맹 대응: 색+글자+고유 모양 | 정상은 무채색, 색은 이상에만, 모양을 함께 | 3절 콘셉트, 4.1 색약 규칙(●○▲■) |
| 비상정지 표시 규격 해석 | [Control Design 2021](https://www.controldesign.com/articles/2021/standards-and-options-for-e-stops/), [Control Design 2023](https://www.controldesign.com/safety/safety-components/article/21526010/standards-guide-the-use-of-estops), [Schneider FAQ](https://www.se.com/uk/en/faqs/FAQ000283554/) (열어 봄; ISO 13850·IEC 60204-1 원문 미열람) | 규격은 빨간 버섯 머리 + 노란 바탕, 직접 개방 동작을 요구하며 "touchscreen buttons"·"graphic representations based on software applications"(NFPA 79 인용)는 비상정지로 인정하지 않음. 빨강-노랑 조합은 비상정지 전용 | 화면 버튼은 물리 비상정지가 아니다. **노란 바탕·버섯 도형을 흉내 내지 않는다**(판단: 안전 등급 장치로 오인 방지). `SAFETY_KO.md`도 "소프트웨어 정지"라고 적고 있다 | 6.4: 노란 바탕 금지, 라벨 아래 "소프트웨어" 꼬리표. 명칭 변경 여부는 사용자 결정(부록 B) |
| Hold-to-confirm | [Blue Robotics Cockpit PR #830](https://github.com/bluerobotics/cockpit/pull/830) (열어 봄) | 슬라이드 확인이 번거롭다는 불만 → 2초 길게 누름으로 대체 논의. 게임패드 물리 경로와 터치 경로가 같은 확인 상태를 공유 | 길게 누름은 패드·터치가 하나의 진행도를 공유 | 이미 그렇다(`clear_progress_pad_`/`_touch_` 중 큰 값). 링 하나에 표시(6.5) |
| WCAG 2.2 | [1.4.3](https://www.w3.org/WAI/WCAG22/Understanding/contrast-minimum.html), [1.4.11](https://www.w3.org/WAI/WCAG22/Understanding/non-text-contrast.html), [2.5.8](https://www.w3.org/WAI/WCAG22/Understanding/target-size-minimum.html) (열어 봄) | 글자 4.5:1, 큰 글자 3:1. UI 구성요소 식별·상태(선택·포커스) 3:1, 비활성 예외. 목표 크기 최소 24×24 CSS px(AA), 44×44(향상) | 선택 표시도 3:1 | 4.1 대비표. 선택 표시는 accent(6.4:1) + 위치(밑줄) |
| 터치 크기·화면 환산 | [Android 접근성 도움말](https://support.google.com/accessibility/android/answer/7101858) (열어 봄), Apple HIG(JS 렌더로 본문 미수신, 44 pt는 요약만) | "at least 48dp… about 9mm". Ally X 315 ppi(리뷰 요약만, 제조사 미공개) → 1 mm ≈ 12.4 px, 9 mm ≈ 111 px, 7 mm ≈ 87 px | 7인치 1080p에서 44–52 px은 3.5–4.2 mm | touch 88 / touchL 112(4.3) |
| 색약 팔레트 | [Okabe & Ito](https://jfly.uni-koeln.de/color/) (열어 봄), Material 다크(본문 미수신, 요약만), [Carbon 색 사용](https://carbondesignsystem.com/elements/color/usage/) (열어 봄) | 중요 정보를 색으로만 전하지 말 것, 어두운 바탕에 빨간 글자 지양, 모양·라벨 병행. Carbon: 다크 테마는 base + layer 01–03 | 레이어 밝기 단계, 빨간 글자는 크게·굵게만 | 4.1 표면 5단계, danger 글자 ≥ 24 px 굵게 |

### 2.5 열지 못한 자료·한계

- Valve Steam Deck 개발자 문서·리뷰(404·연결 실패), gameaccessibilityguidelines.com(연결 실패), wiki.ros.org·docs.ros.org(봇 차단), Boston Dynamics 공식 지원 페이지(404), Apple HIG·material.io(JS 렌더로 본문 없음), controleng.com·automation.com(403), Font Awesome 라이선스 페이지(제목만 반환).
- ISO 13850, IEC 60204-1, ANSI/ISA-101.01 원문은 유료라 읽지 않았다. 규격 서술은 전부 2차 자료 인용이다.
- Gallery·#707은 댓글 텍스트만 확인했고 스크린샷 이미지는 보지 못했다.
- Spot 가이드는 R2.0 구판 제3자 사본, DJI는 Mini 4 Pro 2023 매뉴얼이다. 최신 앱과 다를 수 있다.
- 게임 HUD(Titanfall 2, Armored Core VI 등) 1차 디자인 자료는 찾지 못했다.

---

## 3. 디자인 방향

**콘셉트 한 문장**: "어두운 계기판 위에 영상 하나, 빨간 버튼 하나" — 구조는 회색 단계와 선으로, 색은 상태와 선택에만, 주 입력은 패드이므로 화면은 **확인하는 곳**이지 누르는 곳이 아니다(터치는 보조).

**무드**: 산업용 HMI의 절제(ISA-101) + 휴대용 게임기 UI의 큰 터치 목표와 배지(Steam Deck/Xbox식 관례) + FPV OSD의 가장자리 배치. 광택·그라디언트·네온은 쓰지 않는다. 그림자도 거의 쓰지 않고 1 px 경계선과 표면 밝기 단계로 깊이를 만든다.

**참고 사례(2절에서)**: Foxglove의 패널 분리·어두운 중립 표면, QGroundControl의 "영상 위에 최소 정보 + 모서리 계기", Steam Deck의 하단 버튼 힌트 줄과 포커스 색 하나, Betaflight OSD의 모서리 배치·중앙 비움, ISA-101의 "색은 이상 상태에만".

**가져오지 않을 것**: 제품 로고·아이콘 세트·고유 레이아웃(Spot의 좌우 스틱 레이아웃, DJI의 RTH 아이콘, Xbox 버튼 로고). 게임 HUD의 디제시스 장식(스캔라인, 육각 패턴, 글로우).

---

## 4. 디자인 토큰

모든 값은 1920×1080, `scale_ = 1.0` 기준 px. 다른 해상도는 지금처럼 `S()`로 곱한다. 토큰은 `theme.hpp`에 `constexpr`로 두고, 코드에서 숫자를 직접 쓰지 않는다.

### 4.1 색 (다크 테마가 기본이자 유일)

표면은 거의 무채색(약간 파란 기운)으로 5단계. 위로 올라올수록 밝다(Material 다크 테마의 "높이 = 밝기" 관례).

| 토큰 | 값 | 용도 |
|---|---|---|
| `bg0` | #0B0D10 | 카메라 캔버스, 전체화면 모드 바탕 |
| `bg1` | #13161B | 앱 바탕(루트 창, D3D clear color도 이 값으로 통일) |
| `surface1` | #1B1F26 | 패널·카드 |
| `surface2` | #242932 | 버튼 기본, 입력란, 카드 안의 카드 |
| `surface3` | #2E3440 | 버튼 hover/터치 중, 선택된 세그먼트의 바탕 |
| `border1` | #363D49 | 카드·버튼 1 px 경계 |
| `border2` | #4B5566 | 강조 경계, 슬라이더 트랙 |
| `text1` | #EDF0F4 | 본문·값 |
| `text2` | #A7B0BD | 라벨·보조 |
| `text3` | #6F7A8A | 단위·비활성 설명(장식 수준, 대비 3.4–4.2:1) |
| `accent` | #5EA2FF | 선택·현재 모드·조작 중 입력(스틱 활성, 슬라이더 손잡이). **상태 의미 없음** |
| `accentInk` | #0B1A2E | accent 채움 위의 글자 |
| `ok` | #3DBE6A | 정상·활성(점·글자에만, 넓은 면 채움 금지) |
| `warn` | #E8B83A | 주의(ACK 늦음, 중립 확인 중, 범위 끝, 배터리 낮음) |
| `warnInk` | #2A1E02 | warn 채움 위의 글자 |
| `danger` | #E5484D | 위험 글자·경계(고장, 다른 조종기, 패드 분리) |
| `stop` | #D7262C | 비상정지 버튼 채움(흰 글자 대비 5.0:1) |
| `info` | #8FB5FF | 정보 토스트 글자(accent보다 밝음) |
| `disabled` | #59616E | 비활성 글자(대비 의도적으로 낮음, 2.3–2.9:1) |
| `scrim` | #000000 α 0.35 | HUD 가장자리 그라디언트, 영상 정지 시 덮개 |

대비비(계산, WCAG 2.x 공식):

| 조합 | 비율 | 비고 |
|---|---|---|
| text1 / surface1 | 14.5 | |
| text2 / surface2 | 6.7 | 보조 글자도 AA 통과 |
| text3 / surface2 | 3.4 | 단위·장식만. 본문 금지 |
| accent / surface1 | 6.4 | 선택 표시·글자 모두 가능 |
| ok / surface1 | 6.9 | |
| warn / surface1 | 8.9 | |
| danger / surface1 | 4.2 | 글자는 24 px 이상 또는 굵게 |
| accentInk / accent | 6.7 | 선택된 세그먼트 글자 |
| warnInk / warn | 8.8 | 해제 버튼 활성 글자 |
| 흰색 / stop | 5.0 | 비상정지 라벨 |
| 흰색 / danger | 3.9 | 큰 글자(≥24 px 굵게)만 |
| border1 / surface1 | 1.5 | 장식 경계(단독 정보 전달 금지) |

색약 대응 규칙: 상태는 항상 **색 + 모양 + 글자** 세 가지로 전한다. 점(●) = 정상, 고리(○) = 대기/미확인, 삼각(▲) = 주의, 사각(■) = 위험·정지. 빨강/초록만으로 구분되는 요소를 두지 않는다.

### 4.2 글자 크기 단계

| 토큰 | px | 굵기 | 글꼴 | 용도 |
|---|---|---|---|---|
| `fDisplay` | 44 | Bold | Pretendard | 비상정지 라벨, 연결 화면 제목 |
| `fTitle` | 30 | SemiBold | Pretendard | 패널 제목, 모드 이름, HUD 상태 단어 |
| `fBody` | 26 | Regular | Pretendard | 본문, 버튼 라벨 |
| `fLabel` | 22 | Medium | Pretendard | 라벨, 상단 상태 줄, 배지 옆 설명 |
| `fMicro` | 18 | Medium | Pretendard | HUD 단위, 배지 안 글자(LB/RB), 표 머리. **이보다 작은 글자는 없음** |
| `fNum` | 26 | Regular | JetBrains Mono | 표·HUD 숫자(고정폭) |
| `fNumL` | 34 | Medium | JetBrains Mono | 속도·ACK 큰 숫자 |
| `fNumXL` | 56 | Medium | JetBrains Mono | 전체화면 HUD 속도 |

- 글자 배율: `style.FontScaleMain`에 사용자 설정 1.0–1.3을 곱한다(XAG 101의 "200 %까지 확대" 권고를 공간상 1.3까지로 줄인 것 — 판단). 레이아웃은 1.3에서도 넘치지 않게 여유를 둔다(5.8의 높이 계산은 1.0 기준이며 1.3에서는 팔 탭 그림을 200 px로 줄이는 분기가 필요하다).
- 줄 높이는 글자 크기의 1.3배를 기준으로 `ItemSpacing.y`를 정한다.
- 굵기는 Regular/Medium/SemiBold/Bold 네 개만 내장한다(가변 글꼴 1개로 대체 가능하나 ImGui의 stb/FreeType 로더는 가변 축을 고르지 못하므로 정적 글꼴을 쓴다 — 8절).

### 4.3 간격·모양

| 토큰 | 값 | 용도 |
|---|---|---|
| `sp1..sp6` | 4, 8, 12, 16, 24, 32 | 간격 격자. 모든 패딩·간격은 이 중 하나 |
| `gutter` | 16 | 화면 가장자리, 영역 사이 |
| `radiusS` | 6 | 배지, 작은 칩, 입력란 |
| `radiusM` | 10 | 버튼, 카드 |
| `radiusL` | 16 | 패널, 비상정지 |
| `radiusFull` | 999 | 원형 배지, 점 |
| `strokeHair` | 1 | 경계선 |
| `stroke` | 2 | 선택 경계, 링 바탕 |
| `strokeBold` | 4 | 진행 링, 주행 벡터 |
| `iconS/M/L` | 20, 24, 32 | 아이콘 글꼴 크기(글자 크기와 짝: fLabel·fBody·fTitle) |
| `touchMin` | 64 | 보조 터치 목표 한 변(간격 8 포함 시) |
| `touch` | 88 | 기본 터치 목표 한 변(≈7 mm) |
| `touchL` | 112 | 주요 터치 목표(속도, 팔 버튼) |
| `shadow` | 없음 | 그림자 대신 표면 단계 + 경계선. 떠 있는 것(토스트·팝업)만 `scrim` 8 px 외곽을 4단계 알파로 흉내 |

---

## 5. 레이아웃 재설계 (1920×1080)

구조는 유지한다(상단 상태 줄, 왼쪽 카메라, 가운데 탭 패널, 오른쪽 안전 열, 하단 조작 띠). 바뀌는 것은 비율·정렬·내용 배치다. 좌표는 px, 원점 왼쪽 위.

### 5.1 공통 틀

```
y 0    ┌────────────────────────────────────────────────────────────────────────┐
       │ 상단 상태 줄 h=56  [●연결 로봇이름] [차체 ○대기] [팔 ○대기] [조종권 ●나]   [배터리 ▮▮▮ 7.6V | Ally 99%] [ACK 31] [영상 5] │ [주행|팔|진단|설정] │
y 56   ├───────────────────────────────────────────────┬──────────────────┬─────┤
       │ 카메라 (16:9 또는 소스 비율, 가로 1140)        │ 탭 패널 w=508    │ 안전 │
       │ x 16..1156, y 72..713 (높이 641)                │ x 1172..1680     │ 열  │
       │  HUD: 모서리 4곳 + 중앙 십자선                   │                  │ w=224│
       │                                                 │                  │ x1696│
y 776  ├───────────────────────────────────────────────┴──────────────────┴─────┤
       │ 하단 조작 띠 h=304  [가상 스틱 r=100] [회전] │ [팔 켜기] [팔 버튼 묶음] [그리퍼] │ [패드 모니터] │
y 1080 └────────────────────────────────────────────────────────────────────────┘
```

- 상단 56(지금 70): 칩을 없애고 한 줄 상태 텍스트로 바꾸면 높이가 줄어든다.
- 본문 720(지금 760), 하단 304(지금 250): 하단 터치 목표를 112 px로 키우려면 2줄 × 112 + 간격이 필요하다.
- 가장자리 `gutter 16`을 네 변과 영역 사이에 똑같이 쓴다. 카메라·탭 패널·안전 열·하단 띠의 왼쪽 위 모서리가 같은 격자에 놓인다.
- 안전 열 224(지금 ≈211): 비상정지 192×192 + 좌우 16.

### 5.2 상단 상태 줄 (칩 대신 "상태 줄")

왼쪽 그룹(연결 상태, 왼쪽 정렬, 간격 24):
1. 연결 표시: `●` 8 px 점(ok/warn/danger/disabled) + 로봇 이름 또는 "연결 안 됨"(fLabel). 데모 실행이면 "데모 로봇".
2. 채널 두 개: "차체 ○ 대기", "팔 ● 활성" — 글자 text2, 상태 기호만 색.
3. 조종권: "조종권 나" / "다른 조종기"(danger 기호 ■ + 글자 danger).
4. 고장 보고됨(있을 때만): ■ danger 글자 "고장 — 새 세션 필요".

오른쪽 그룹(계기, 오른쪽 정렬, 고정폭 숫자는 JetBrains Mono 22 px = fLabel 크기, 라벨 fMicro):
- 로봇 배터리 `7.6 V` + 4칸 막대(낮음이면 warn), Ally `99 %` + 충전 아이콘, ACK `31` ms(중앙값만; p50/max는 진단 탭), 영상 `5` fps 또는 "영상 꺼짐", 패드 아이콘(없으면 danger + "패드 없음").
- 자리가 모자라면 **숫자 라벨을 아이콘으로 줄이는 2단계 축약**을 정의하고, 정보를 버리지 않는다(H2 해결).

맨 오른쪽: 모드 세그먼트(5.3). 상단 줄 바탕은 `bg1`, 아래 1 px `border1`.

### 5.3 모드 전환(탭)

- 세그먼트 컨트롤 4칸(주행·팔·진단·설정), 칸 112×40, 아이콘 20 + 글자 fLabel. 선택 칸은 `surface3` 바탕 + 아래 3 px `accent` 밑줄(색만이 아니라 위치로도 표시). 전환 애니메이션: 밑줄이 120 ms에 이동.
- 패드 힌트: 세그먼트 오른쪽 끝에 `≡` 배지 + "설정"(Menu 버튼은 설정만 토글하므로 그것만 표시). F1–F4는 설정 탭의 "키보드" 항목에서만 보여 준다.
- 모드 이름은 "조종"→"주행"으로 바꾸는 것을 제안한다(팔 탭도 조종이므로). 기능 변화는 없다. 문서(`CONTROLS_KO.md`)의 이름과 맞출지는 사용자가 결정한다.

### 5.4 카메라와 HUD

- 이미지는 영역에 **꽉 차게 맞추되(contain)** 레터박스 바탕은 `bg0`. 아래의 버튼 두 개는 없애고 HUD 오른쪽 아래에 아이콘 버튼 2개(64×64, 영상 켜기/끄기 Ⓧ, 크게 Ⓨ)로 옮긴다. 카메라 상태 문구("수신 중")는 영상이 정상일 때는 숨기고, 끊겼을 때만 중앙에 보인다.
- 가장자리 안전 여백 32 px(EBU R 95의 그래픽 안전 5 %는 1080p에서 좌우 96·상하 54 px이지만 이는 오버스캔 대비 값이고 7인치에서는 공간 손실이 크다. 손가락·베젤 가림을 고려한 32 px로 정했다 — 판단). 네 모서리 배치:
  - 왼쪽 위: 상태 단어 하나("중립" / "조종 중" / "비상정지") fTitle, 외곽선 글자. 비상정지일 때만 stop 색 바탕 띠.
  - 오른쪽 위: 팔 목표 `x 0.105 y 0.000 z 0.075` `pitch -59° 그리퍼 0.00` — **팔이 켜져 있을 때만**. JetBrains Mono 22 px, 외곽선 글자. 범위 끝이면 warn.
  - 왼쪽 아래: 주행 벡터 원(반지름 44, 2 px 흰 70 % 고리) + 명령 벡터(accent 4 px) + 회전 호(info 3 px). 지금 구조 유지, 색만 토큰화.
  - 오른쪽 아래: 아이콘 버튼 2개(위) + 속도 상한 `30 %` fNumL(아래). 저속이면 "저속" 꼬리표.
  - 중앙: 십자선(설정으로 켜고 끔) 외에는 아무것도 두지 않는다. 토스트는 **상단 중앙**으로 옮긴다(아래 중앙은 주행 벡터·버튼과 겹친다).
- 상단·하단 가장자리에 높이 96 px의 `scrim` 그라디언트(위는 0.35→0, 아래는 0→0.35)를 `AddRectFilledMultiColor`로 깐다. 밝은 영상에서 글자를 보장한다.

### 5.5 안전 열(항상 보임)

위에서 아래로, 모두 가로 192, 왼쪽 여백 16:
1. 비상정지 192×192(6.4), 아래 배지 줄: `Ⓑ` + "Space" fMicro text2.
2. 해제 길게 누르기 192×72(6.5), 아래 배지 줄: `⧉`형 View 배지(두 사각형 겹친 모양을 선으로 그림) + "0.8초".
3. 간격 24.
4. 속도 상한: 라벨 "속도 상한" fLabel + 현재 값 fNumL(예: `30 %`), 아래 세로 세그먼트 3칸(30/60/100) 각 192×64, 간격 8. 선택 칸은 accent 바탕 + accentInk 글자. 오른쪽 D-pad 배지 `↕` + "±10 %".
5. 저속 토글 192×64: 꺼짐 `surface2`, 켜짐 `surface3` + 왼쪽 4 px accent 띠 + "저속 켜짐". 배지 `RB`.
6. 남는 공간은 비워 둔다. 비상정지 주변이 비어 있을수록 잘 보인다(판단).

### 5.6 하단 조작 띠(터치 보조)

x 좌표는 왼쪽부터. 높이 304, 세로 가운데 정렬.
- 가상 스틱: 중심 (136, 928), 반지름 100. 바탕 `surface2`, 테두리 `border2` 2 px, 활성 시 accent. 손잡이 반지름 32.
- 회전 슬라이더: x 272..592, 트랙 높이 16, 손잡이 지름 56, 라벨 "회전" fLabel + 양 끝 `◀ ▶`은 글자 대신 Lucide `chevron-left/right`.
- 팔 묶음(x 640..1480): 왼쪽에 팔 켜기 토글 160×112(2줄 라벨 대신 "팔 켜기" + 아래 작은 글자 "LB 누른 채도 됨"; 켜짐이면 accent 띠 + 남은 시간 "자동 꺼짐 58초"는 10초 이하일 때만 warn으로 표시). 그 오른쪽에 **축별로 묶은** 버튼: `[앞/뒤]` 세로 짝, `[왼/오른]` 가로 짝, `[위/아래]` 세로 짝, `[손목↑/↓]` 세로 짝 — 각 112×112, 짝 사이 4 px, 묶음 사이 16 px. 라벨은 글자 + 화살표 아이콘. 지금의 4×2 격자보다 짝이 바로 보인다(판단).
- 그리퍼: "닫기 RT" / "열기 LT" 세로 짝 144×112, 아이콘(집게 벌림/오므림은 Lucide `grip`류가 없으므로 선 4개로 직접 그림).
- 패드 모니터(x 1560..1904): 스틱 2개(반지름 44) + 위에 LB/RB 배지(눌림이면 accent 채움), 아래 LT/RT 트리거 막대 120×8(채움 accent). 라벨을 원 중심 기준으로 대칭 정렬한다.

### 5.7 주행 탭 패널(508×720)

카드 3장, 세로 배치, 카드 패딩 16, 카드 사이 16:
1. **채널 카드**(높이 168): 2열(차체 | 팔) × 2행(상태 기호+글자, 마지막 응답). 큰 글자 fBody, 기호로 상태. "무장/자동 재무장/마지막 차단"은 진단 탭으로 이동.
2. **링크 카드**(높이 120): ACK 중앙값 `31` ms(fNumL) + 최대 `94` + 루프 `20.1` Hz 세 개를 가로 3칸 숫자 타일로. 패킷 수·오류는 진단 탭으로.
3. **세션 카드**(높이 208): 버튼 3개 가로(연결 바꾸기 / 세션 놓기·새 세션 / 연결 끊기) 각 148×64. "연결 끊기"만 danger 경계(`border` 2 px danger, 글자 danger) — 파괴적 동작 구분. 아래 설명 한 줄 fLabel text2.
4. 속도 슬라이더는 삭제(안전 열에 세그먼트가 있고 D-pad로 ±10 %). 남는 세로 공간은 비워 둔다.

### 5.8 팔 탭 패널(508×720, 스크롤 없음)

1. 제목 줄(40): "팔" fTitle + 상태 기호/글자("켜짐" ok / "꺼짐 — LB 또는 팔 켜기" text2).
2. 그림 2장(각 236×236, 사이 16): 위에서/옆에서. 격자선 `border1` 5 cm 간격, 작업공간 상자 `border2` 점선 대신 1 px 실선 + 안쪽 `surface2` 채움 12 % 알파(주황 제거). 점: 내 목표 accent ●, 로봇 적용 목표 ok ○, 로봇 현재 text2 ●(작게). 범례는 그림 아래 한 줄 fMicro에 기호와 함께.
3. 수치 카드(높이 152): x/y/z/pitch 4칸. 각 칸 = 라벨 fMicro + 값 fNum(고정폭) + 좌우 −/+ 버튼 56×56(지금 ImGui 기본 ±는 너무 작다). 그 아래 버튼 3개(숫자 목표로 / 지금 목표 불러오기 / 기본 자세) 각 156×56.
4. 그리퍼 슬라이더(높이 56): 트랙 16, 손잡이 56, 양 끝 라벨 "열림 0" / "닫힘 1".
5. 목표 표(높이 120): 3행(내 목표 / 로봇 적용 / 로봇 현재) × 5열, 고정폭 숫자, 첫 열 160 px에 줄바꿈 허용. 관절 6개는 표 아래 **한 줄 6칸 미니 막대**(각 72×8, 색 border2 바탕 + text2 채움, 값은 fMicro) — 지금의 노란 ProgressBar 대체.
합계 ≈ 40+236+24+152+56+120+40 = 668 + 카드 간격 5×8 = 708 < 720(글자 배율 1.0 기준). 스크롤이 사라진다(H5). 여유가 12 px뿐이므로 단계 6에서 실제 줄 높이로 재측정하고, 모자라면 그림을 220 px로 줄인다.

### 5.9 진단 탭·설정 탭

- 진단: 상단에 "조종 탭에서 옮겨 온" 패킷·루프·차단 사유 카드, 그 아래 접이식(ImGui `CollapsingHeader`를 쓰되 헤더를 카드 제목 스타일로 재정의) 로봇 상태 / 관절 직접 조종 / 전체 정지·해제 / 사건 기록. 로그 영역은 고정폭 글꼴 fNum 18 px, 송신 accent·수신 ok 대신 **화살표 기호 + text2**로(색 절약).
- 설정: 2열 폼. 왼쪽 라벨(fLabel text2) 오른쪽 컨트롤(높이 56). 슬라이더는 손잡이 지름 40. 주사율 30/60/120은 세그먼트. "앱 종료"는 danger 경계. 맨 아래 버전 줄 text3.

### 5.10 전체 화면 카메라 모드(Ⓨ)

- 카메라가 본문 전체(상단 줄 아래, 하단 띠 위)를 차지하고 안전 열은 그대로 남는다(현재 동작과 같음: `camera_big_`일 때 측면 열 유지). 하단 띠도 유지한다(터치 주행 보조). 즉 기능은 동일하고, HUD 텍스트 크기만 한 단계 올린다(fTitle→fDisplay 아님, fNumL→fNumXL는 속도만).
- 이 모드에서는 탭 패널의 정보가 보이지 않으므로, HUD 오른쪽 위에 팔 목표(팔 켜짐일 때), 오른쪽 아래에 속도·링크 ACK를 둔다.

### 5.11 연결 화면

- 왼쪽 절반(카메라 자리): 제목 "로봇 연결" fDisplay, 안내 한 줄 text2, 주소 입력란(높이 64, fNumL 고정폭) + "연결" 버튼(accent, 160×64) + "로봇 찾기"(surface2). 숫자판 3×4, 키 112×80, 간격 8, 아래 "지우기".
- 오른쪽(탭 패널 자리): "찾은 로봇" 제목 + **카드** 목록(각 476×112): 1행 이름 fBody, 2행 주소·포트·기기 ID 앞 8자 fNum text2, 오른쪽 끝 RTT 숫자 + 3칸 막대. 눌림 전체가 버튼. "최근 연결" 카드 1장(시계 아이콘). 맨 아래 패드 상태 줄(아이콘 + "연결됨/없음" + 안내).
- 안전 열은 연결 전에도 보이되 비상정지는 `disabled` 모양(6.4의 상태 D).

---

## 6. 컴포넌트 명세

공통: 모든 커스텀 위젯은 `ally::ui::widgets` 네임스페이스, `ImGui::InvisibleButton` + `ImDrawList`로 그린다. 상태 애니메이션 값은 `ImGui::GetStateStorage()`에 `ImGuiID`로 저장하거나 위젯 구조체를 호출자가 보관한다. `theme.hpp`의 토큰만 참조한다.

### 6.1 상태 표시(칩 대체) `StatusItem`

```cpp
enum class Status { kOk, kIdle, kWarn, kDanger, kOff };
// 기호 + 글자 한 묶음. 반환: 차지한 너비.
float StatusItem(const char* label, Status s, const char* value = nullptr);
```
- 기호 10×10: kOk ● `ok`, kIdle ○ `text2` 2 px, kWarn ▲ `warn`, kDanger ■ `danger`, kOff ○ `disabled`. 그리기: `AddCircleFilled`/`AddCircle`/`AddTriangleFilled`/`AddRectFilled`.
- 글자: label fLabel text2, value fNum text1. 간격 8. 바탕 없음. 높이 32.
- 상태 변화 시 기호가 150 ms 동안 1.3배로 커졌다 돌아온다(주목 유도, 7절).

### 6.2 패드 배지 `PadBadge`

```cpp
enum class PadKey { kA, kB, kX, kY, kLB, kRB, kLT, kRT, kView, kMenu, kDpadUD, kDpadLR, kLStick, kRStick };
ImVec2 PadBadge(ImDrawList* dl, ImVec2 pos, PadKey k, float h = 28, bool pressed = false);
```
- A/B/X/Y: 지름 h(기본 28, 안의 글자 fMicro 18이 XAG 101의 "글리프 안 글자도 최소 크기" 조건을 지키도록) 원, `surface3` 채움 + `border2` 1 px, 가운데 글자 fMicro Bold text1. 색상 관례(A 초록 등)는 쓰지 않는다 — 상표 연상과 상태색 혼동을 피한다(판단). pressed면 accent 채움 + accentInk 글자.
- LB/RB/LT/RT: 가로 h×1.6, 세로 h, `radiusS` 둥근 사각형, 글자 fMicro.
- View/Menu: 원 안에 선으로 그림(View: 사각형 2개 겹침 `AddRect` 2회, Menu: 가로선 3개 `AddLine`).
- D-pad: 원 안에 ↕ 또는 ↔ 화살표(`AddTriangleFilled` 2개 + `AddLine`).
- 스틱: 원 안에 작은 원 + "L"/"R" 글자.
- 라벨과 함께 쓰는 보조: `void HintRow(std::initializer_list<std::pair<PadKey,const char*>>)` — 배지 + 글자를 가로로 나열, 높이 32.

### 6.3 카드·패널 `BeginCard / EndCard`

```cpp
bool BeginCard(const char* id, const char* title, ImVec2 size, bool scroll = false);
void EndCard();
```
- `PushStyleColor(ImGuiCol_ChildBg, surface1)`, `PushStyleVar(ImGuiStyleVar_ChildRounding, radiusM)`, `ChildBorderSize 1` + `ImGuiCol_Border = border1`, `WindowPadding (16,16)`. 제목은 fTitle(28) text1, 아래 8 px 간격. `ImGuiChildFlags_Borders | AlwaysUseWindowPadding`.
- 표 스타일: `ImGuiCol_TableHeaderBg = surface1`(바탕과 동일), `TableBorderLight = border1`, `TableRowBgAlt = surface2` 40 % 알파. 머리글은 fMicro text2.

### 6.4 비상정지 버튼 `EstopButton`

```cpp
enum class EstopVisual { kReady, kPressed, kLatched, kDisabled };
bool EstopButton(const char* id, ImVec2 size, EstopVisual v, double now);
```
| 상태 | 모양 |
|---|---|
| kReady(연결됨, 정지 아님) | `stop` 채움, `radiusL`, 안쪽 8 px 들어간 자리에 흰 40 % 1.5 px 고리, 라벨 "비상\n정지" fDisplay 44 Bold 흰색, 라벨 아래 fMicro 흰 70 % "소프트웨어 정지". **노란 바탕·버섯 머리 도형은 쓰지 않는다** — 빨강-노랑 조합과 버섯 형태는 규격상 물리 비상정지 전용이고 화면 버튼은 그 등급이 아니다(2.4, `SAFETY_KO.md` "화면 비상정지는 소프트웨어 정지"). 명칭을 "정지"로 바꿀지는 사용자 결정(부록 B) |
| kPressed(터치 중) | 채움을 `#B01F24`로, 전체를 0.97배로 줄임(눌림 느낌, 80 ms) |
| kLatched(로봇이 비상정지 상태) | 채움 `#5A1416`(어두운 빨강), 테두리 3 px `stop`가 1 Hz로 알파 0.4↔1.0 호흡, 라벨 "정지됨" fTitle + 아래 "해제: 아래 버튼 길게" fMicro. 주변 안전 열 바탕에 `stop` 6 % 알파 |
| kDisabled(연결 안 됨) | 채움 없음, `border2` 2 px 테두리, 라벨 text3 "비상정지", 눌러도 아무 일 없음(지금도 `core_` 없으면 알림만) |
- 구현: `InvisibleButton` → `IsItemActive()`로 kPressed, `AddRectFilled` + `AddRect`(고리) + `AddText(font, size, ...)` 중앙 정렬. 호흡은 `0.7 + 0.3 * sin(now * 2π)`.
- 터치 즉시 발동(현재 동작 유지: `ImGui::Button` 반환 = release 시 발동이면 **press 시 발동으로 바꾸는 것이 안전상 더 낫다**. 이것은 기능 변경이므로 사용자 확인 후 결정. 디자인 계획은 어느 쪽이든 모양이 같다).

### 6.5 길게 누르기 해제 `HoldRingButton`

```cpp
bool HoldRingButton(const char* id, const char* label, ImVec2 size, double hold_s, float external_progress);
```
- 모양: `surface2` 바탕 + `border1`, 왼쪽에 지름 40 진행 링(바탕 `border2` 2 px, 진행 `warn` 4 px, `PathArcTo(center, 18, -π/2, -π/2 + 2π·frac)` + `PathStroke`), 오른쪽에 라벨 fBody. 누르는 동안 바탕 `surface3`, 라벨 text1.
- `external_progress`는 패드(View) 진행도 — 지금처럼 터치와 패드 중 큰 값을 링에 반영. 별도 `ProgressBar`는 없앤다.
- 완료 순간: 링이 꽉 차고 60 ms 동안 `ok`로 바뀐 뒤 사라진다. 실패(도중 놓음)는 링이 120 ms에 0으로 되감긴다.

### 6.6 세그먼트 컨트롤 `Segmented`

```cpp
int Segmented(const char* id, const char* const* items, int n, int current, ImVec2 item_size, bool vertical = false);
```
- 트랙 `surface1` + `border1`, 선택 칸 accent(세로형 안전 열) 또는 `surface3` + 밑줄 accent(가로형 모드 탭). 선택 칸 위치는 120 ms ease-out로 이동(`GetStateStorage`에 float 보관).
- 터치 최소 64 높이.

### 6.7 가상 스틱·트리거 표시

- `VirtualStick` 로직 유지, 색만: 바탕 `surface2`, 고리 `border2`(활성 accent), 십자 `border1`, 손잡이 `text2`(활성 accent). 손잡이 복귀는 즉시(안전: 손 떼면 중립이므로 시각도 즉시).
- `TriggerBar(ImVec2 pos, float w, float value, PadKey label)`: 트랙 8 px `border2`, 채움 accent, 왼쪽 배지.
- 패드 모니터 스틱 점: 연결 시 text1, 비연결 시 disabled.

### 6.8 팔 그림 `ArmPlot`

- 지금 좌표 변환 유지. 추가: 5 cm 격자(`AddLine` border1 1 px), 축 라벨 fMicro text3("x 앞 →", "y 왼 ↑"), 작업공간 상자(border2 1 px + surface2 12 % 채움), 점 크기 목표 8/적용 12 고리 3 px/현재 6. 끌기 중 손가락 위치에 accent 24 고리 + 가는 십자(`AddLine` 2개). 그림 모서리 `radiusM`, `PushClipRect` 유지.

### 6.9 토스트 `ToastStack`

- 위치: 카메라 영역 상단 중앙(가로 최대 720), 최신이 아래, 최대 3개 겹쳐 세로 8 px 간격. 높이 56, 바탕 `surface2` 92 % + `border1`, 왼쪽 4 px 종류 색 띠(ok/warn/danger/info), 아이콘 20(Lucide: check / alert-triangle / octagon-x / info), 글자 fBody text1.
- 들어올 때 y −16→0, 알파 0→1, 120 ms. 나갈 때 알파 1→0 150 ms. `notes_`는 이미 `deque<Note>`이므로 전부 그리면 된다.
- 비상정지 토스트만 danger 띠 + 아이콘, 사라질 때까지 호흡 없음(과하지 않게).

### 6.10 숫자 표시 `NumberTile`

```cpp
void NumberTile(const char* label, const char* value, const char* unit, ImVec2 size, Status s = Status::kOk);
```
- 라벨 fMicro text2 위, 값 fNumL(고정폭) text1 가운데, 단위 fMicro text3 오른쪽 아래. 상태가 kWarn/kDanger면 값만 그 색. 바탕 `surface2`, `radiusM`.
- 모든 변동 숫자는 고정폭 글꼴로 그려 레이아웃 흔들림을 없앤다.

### 6.11 카메라 HUD 글자 `OsdText`

```cpp
void OsdText(ImDrawList* dl, ImVec2 pos, ImFont* f, float size, ImU32 col, const char* text);
```
- 외곽선: 같은 글자를 (±1, 0), (0, ±1), (±1, ±1) 8방향에 `#000000` 70 %로 먼저 그리고 본 글자를 위에 그린다(`AddText(font, size, ...)` 9회). 120 Hz에서도 HUD 글자 수가 적어 비용이 작다(판단; 측정은 10절).
- 규칙: 모서리 안전 여백 24, 중앙 비움, 한 모서리에 최대 2줄, 숫자는 고정폭, 색은 text1/warn/danger만.

### 6.12 연결 카드 `RobotCard`

- 476×112, `surface2` + `border1`, 눌림 `surface3`. 1행: 로봇 아이콘 24 + 이름 fBody. 2행: 주소 · 포트 · ID 8자 fNum text2. 오른쪽: RTT 값 fNumL + 3칸 막대(≤30 ms 3칸 ok, ≤80 2칸, 그 이상 1칸 warn). 사진·실제 주소는 README용 스크린샷에서 데모 모드로 가린다(현재 코드가 이미 "데모 로봇"으로 숨김).

### 6.13 ImGui 스타일 변수(한 번에 적용, `theme.cpp`)

```cpp
void ApplyTheme(ImGuiStyle& st, float scale);
// WindowRounding 0, ChildRounding radiusM, FrameRounding radiusM, GrabRounding radiusFull, PopupRounding radiusM
// FramePadding (16,12), ItemSpacing (12,12), ItemInnerSpacing (8,8), CellPadding (12,8), ScrollbarSize 16, GrabMinSize 40
// FrameBorderSize 1, ChildBorderSize 1, WindowBorderSize 0
// Colors: WindowBg bg1, ChildBg surface1, FrameBg surface2, FrameBgHovered/Active surface3, Button surface2, ButtonHovered surface3, ButtonActive accent(+ Text accentInk via PushStyleColor in wrapper), Border border1, Text text1, TextDisabled disabled, SliderGrab accent, CheckMark accent, Header surface2, HeaderHovered surface3, TableHeaderBg surface1, TableBorderLight border1, PlotHistogram text2, ScrollbarBg bg1, ScrollbarGrab border2
```
- 기본 `ImGui::Button`은 `widgets::Button(label, size, Variant{kDefault,kPrimary,kDanger,kGhost}, PadKey hint)`로 감싼다: primary = accent 채움 + accentInk, danger = border 2 px danger + 글자 danger, ghost = 바탕 없음.

---

## 7. 움직임·피드백

120 Hz(8.3 ms/프레임) 기준. 모든 전환은 `io.DeltaTime` 기반 보간이며 프레임 수가 아니라 ms로 정의한다. 30/60 fps 상한에서도 같은 시간이 걸린다.

| 토큰 | ms | 곡선 | 용도 |
|---|---|---|---|
| `tFast` | 80 | ease-out | 버튼 눌림 색, 비상정지 0.97배 |
| `tBase` | 120 | ease-out | 세그먼트 이동, 토스트 입장, 상태 기호 커짐 |
| `tSlow` | 200 | ease-in-out | 탭 패널 내용 교차 페이드(알파만, 이동 없음) |
| `tRing` | 입력 길이 | 선형 | 길게 누르기 링(0.8 s), 되감기 120 ms |
| `tBreath` | 1000 주기 | sin | 비상정지 latched 테두리 |

- 진동과 화면의 짝: `HapticPlayer` 패턴이 재생되는 동안 같은 길이로 화면도 반응한다. 비상정지(강한 진동) ↔ 안전 열 바탕 `stop` 12 % 알파 플래시 2회(각 150 ms) 후 latched 호흡으로. 통신 끊김(두 번) ↔ 상단 연결 기호가 ▲ warn으로 바뀌며 2회 깜빡임. 범위 끝(짧게) ↔ HUD 팔 수치가 warn으로 150 ms. 거부 ↔ 팔 그림의 목표 점이 적용 점으로 되돌아가는 120 ms 이동. 서비스 성공/실패 ↔ 토스트 ok/danger.
- 비상정지 시각 효과의 한도: 전체 화면을 빨갛게 덮지 않는다. 카메라는 그대로 보여야 하고(상황 파악), 빨강은 버튼·상단 상태·HUD 왼쪽 위 단어에만 쓴다.
- 과한 것 금지: 글로우, 그림자 애니메이션, 숫자 카운트업, 배경 움직임.
- 스모크 테스트(기본 180프레임 ≈ 1.5 s @120 Hz)에서 스크린샷을 찍을 때 입장 애니메이션이 끝나 있도록, 모든 전환은 0.5 s 안에 끝난다.

---

## 8. 글꼴·아이콘

### 8.1 사실 확인(2026-11-14)

| 후보 | 라이선스 | 확인 출처 | 비고 |
|---|---|---|---|
| Pretendard | SIL OFL(저장소에 "SIL 오픈 폰트 라이선스… 글꼴 단독 판매를 제외한 모든 상업적 행위 및 수정, 재배포가 가능") | https://github.com/orioncactus/pretendard (열어 봄, v1.3.9 최신) | 9굵기 + 가변. OpenType `tnum`(고정폭 숫자) 제공, 기본 숫자가 비례인지 고정폭인지는 명시 없음(https://cactus.tistory.com/306 열어 봄). 파일 크기는 페이지에 표시되지 않아 내려받아 측정해야 함 |
| Noto Sans KR | SIL OFL(일반 지식, 이번에 페이지 미확인) | — | CJK 전체 포함이라 파일이 큼(수 MB 이상 예상, 미측정) |
| IBM Plex Sans KR | OFL로 알려짐 | https://github.com/IBM/plex/releases (열었으나 KR 자산·크기 미표시) | 미확인으로 둠 |
| JetBrains Mono | SIL OFL 1.1 | https://github.com/JetBrains/JetBrainsMono (열어 봄) | 고정폭. 숫자 표시용 |
| Material Symbols | Apache 2.0 | https://github.com/google/material-design-icons LICENSE(열어 봄) | 가변 TTF 3종 + `.codepoints` 파일 제공. 크기 미표시(가변 전체는 수 MB로 예상) |
| Lucide | ISC(Feather 유래분 MIT) | https://github.com/lucide-icons/lucide LICENSE(열어 봄) | 아이콘 글꼴 패키지(lucide-static) 존재 여부는 이 페이지에서 미확인 |
| Font Awesome Free | 페이지가 제목만 반환되어 미확인 | https://fontawesome.com/license/free | 일반적으로 아이콘 CC BY 4.0·글꼴 OFL·코드 MIT로 알려졌으나 이번에 확인 못함 |

### 8.2 ImGui 1.92.9b에서의 로딩(로컬 헤더·릴리스 노트로 확인)

- 1.92부터 글리프를 **동적으로** 로드한다. "No need to specify ranges, prebake etc."(v1.92.0 릴리스 노트). `GetGlyphRangesKorean()`은 레거시 백엔드용이며 DX11 백엔드(`ImGuiBackendFlags_RendererHasTextures` 지원)에서는 필요 없다. 한글 전체를 미리 굽지 않으므로 아틀라스 메모리도 작다.
- `PushFont(font, size)`로 같은 글꼴을 임의 크기로 그린다. `GetFontSize()` 값을 다시 넘기지 말 것(전역 배율 2중 적용 경고가 헤더에 있다). 크기 토큰은 `style.FontSizeBase` 기준의 **비배율 값**으로 보관한다.
- 내장: `AddFontFromMemoryTTF(data, size, 0.0f, &cfg)`에 `cfg.FontDataOwnedByAtlas = false`(exe 리소스는 수명이 프로세스 전체). 데이터는 `res/app.rc`에 `RCDATA`로 넣고 `FindResource/LoadResource/LockResource`로 포인터를 얻는다.
- 아이콘 병합: 두 번째 `ImFontConfig`에 `MergeMode = true`, `GlyphOffset.y`로 기준선 보정, 겹치는 코드포인트는 `GlyphExcludeRanges`로 제외. 코드포인트 상수는 Lucide `.codepoints`/Material `.codepoints`에서 생성하거나 juliettef/IconFontCppHeaders(이번에 페이지 미확인) 형식으로 헤더를 만든다.
- FreeType(`IMGUI_ENABLE_FREETYPE`)은 지금 꺼져 있다. 한글 24 px 이하에서 stb_truetype 힌팅 없는 렌더가 흐리면 FreeType을 켜는 것을 2단계 선택지로 둔다(빌드 의존 추가).
- 가변 글꼴의 축 선택은 ImGui 로더가 지원하지 않으므로 **정적 굵기 파일**을 쓴다.

### 8.3 최종 추천

- 본문: **Pretendard** Regular·Medium·SemiBold·Bold 4개(정적 OTF/TTF). 이유: OFL, 한글·라틴 균형, 9굵기 제공, 국내 UI에서 검증됨. Noto Sans KR보다 파일이 작을 것으로 예상(미측정). 내려받은 뒤 각 파일 크기를 `THIRD_PARTY_NOTICES.md`에 기록한다. 예산: 글꼴 전체 합 6 MB 이하를 목표(초과 시 Regular·Bold 2개로 줄임).
- 숫자: **JetBrains Mono** Regular·Medium 2개. 고정폭 숫자가 필요한 곳(HUD, 표, ACK, 속도)에 별도 `ImFont`로 쓴다. Pretendard의 `tnum`은 ImGui 로더가 OpenType 기능을 적용하지 않으므로 쓸 수 없다(판단: stb_truetype·ImGui FreeType 로더 모두 GSUB 기능 미적용).
- 아이콘: **Lucide** 아이콘 글꼴(ISC). 선 아이콘이라 2 px 선 위주의 이 UI와 어울린다. 필요한 아이콘은 20개 안팎(연결, 배터리, 충전, 카메라, 카메라 끔, 최대화, 패드, 경고, 정지, 체크, 정보, 설정, 시계, 로봇 대신 "box", 화살표 4방향, 회전, 집게는 직접 그림). 글꼴 파일을 못 구하면 Material Symbols Outlined(Apache 2.0)로 대체하되 가변 전체 파일 대신 subset 도구(pyftsubset)로 필요한 코드포인트만 남긴다(수백 KB 예상).
- 맑은 고딕 폴백: 내장 글꼴 로드 실패 시 지금 경로 유지.

---

## 9. 구현 계획

### 9.1 구조

```
src/app/ui/theme.hpp / theme.cpp      토큰(색·크기·간격·시간), ApplyTheme(), 색 유틸(WithAlpha, Mix)
src/app/ui/fonts.hpp / fonts.cpp      리소스에서 글꼴 로드, Fonts{regular, medium, semibold, bold, mono, monoMedium, icons}
src/app/ui/widgets.hpp / widgets.cpp  6절 위젯 전부(StatusItem, PadBadge, Card, EstopButton, HoldRingButton, Segmented, Button, NumberTile, OsdText, ToastStack, RobotCard, TriggerBar)
src/app/ui/anim.hpp                   Spring/Tween 보간 헬퍼(GetStateStorage 기반)
src/app/gui_app.cpp                   화면 조립만 남김(draw_* 함수는 유지하되 widgets 호출로 교체)
res/fonts/*.ttf|otf, res/app.rc       RCDATA 추가
res/icons/codepoints.hpp              아이콘 코드포인트 상수
```
- `gui_app.cpp`의 `kRed…kDim` 상수와 `chip()`·`hold_button()`은 widgets로 옮기고 삭제한다. 입력·안전 로직(`handle_input`, `handle_cues`, 데드맨, 길게 누르기 판정)은 **손대지 않는다**.
- 단위 시험(`test/unit_tests.cpp`)은 UI와 무관하므로 영향 없음. CMake에 `src/app/ui/*.cpp`와 폰트 리소스만 추가.

### 9.2 단계와 완료 기준

| 단계 | 작업 | 완료 기준(스크린샷) | 추정 |
|---|---|---|---|
| 0 | 기준선: 현재 빌드로 4탭 + 연결 화면 + 전체화면 스크린샷 저장(`docs/images/before/`) | 6장 보관 | 0.5 h |
| 1 | `theme`·`fonts`: 토큰, ApplyTheme, 글꼴 내장(Pretendard 4 + Mono 2 + Lucide), clear color 통일 | 기존 화면에 새 글꼴·색이 적용된 상태로 깨짐 없이 렌더. 한글·숫자·아이콘 글리프 표시 | 4 h |
| 2 | 상단 상태 줄: 칩 제거, StatusItem·숫자 계기·모드 세그먼트·Menu 배지 | 상단 56 px, 정상 상태에서 채도 높은 색이 점 하나뿐 | 4 h |
| 3 | 안전 열: EstopButton 4상태, HoldRingButton, 속도 세그먼트, 저속 토글, 배지 | 연결 전(disabled)·정상·비상정지(latched) 세 장 | 4 h |
| 4 | 카메라 HUD: contain 맞춤, 아래 버튼 제거→아이콘 버튼, 스크림, OsdText, 토스트 상단 중앙 | 영상 있음/없음/비상정지 세 장. 밝은 테스트 이미지 위에서 글자 판독 | 4 h |
| 5 | 하단 띠: 스틱·회전 재도색, 팔 버튼 축별 묶음 112 px, 그리퍼, 패드 모니터 대칭 | 모든 터치 목표 ≥ 88 px 확인 | 4 h |
| 6 | 주행·팔 탭: 카드화, 진단 정보 이동, 속도 슬라이더 삭제, 팔 탭 스크롤 제거, 수치 ± 56 px, 관절 미니 막대 | 팔 탭에 스크롤바 없음, 잘리는 글자 없음 | 6 h |
| 7 | 연결 화면: RobotCard, 숫자판 112×80, 입력란 고정폭 | 데모 모드 스크린샷에 주소 미노출 | 3 h |
| 8 | 진단·설정: 접이식 헤더 재정의, 로그 고정폭, 폼 2열 | 두 탭 스크린샷 | 3 h |
| 9 | 모션: anim 헬퍼, 세그먼트 이동, 토스트 입장, latched 호흡, 플래시 | 120 Hz에서 GPU 시간 변화 없음(진단 탭 fps 확인) | 3 h |
| 10 | 감사·문서: 대비 스크립트 재실행, 터치 크기 목록, README 이미지 교체, THIRD_PARTY_NOTICES 글꼴 추가 | 전후 비교 표 완성 | 2 h |

합계 약 38 h(판단, 1인 기준). 단계 1–4만 해도 "촌스러움"의 대부분이 사라진다(판단).

### 9.3 위험

- **글꼴 라이선스·크기**: 파일 크기를 측정하기 전에는 exe 증가량을 모른다. 정적 링크 exe이므로 10 MB 증가도 동작엔 문제없지만 저장소 용량·배포에 영향. OFL 글꼴은 이름 유지(Reserved Font Name) 규정만 지키면 되며, 수정 없이 내장하므로 문제없음(판단; 각 LICENSE 파일을 `third_party/fonts/`에 동봉).
- **ImGui 1.92 동적 글리프의 첫 프레임 비용**: 새 글자 크기가 처음 등장할 때 래스터라이즈가 일어난다. 크기 단계를 8개로 고정했으므로 몇 프레임 뒤 안정됨. 스모크 테스트 180프레임 안에 모두 등장하도록 각 탭을 한 번씩 그릴 필요는 없다(탭별 스크린샷은 `--tab`으로 따로 찍음).
- **OsdText 9회 그리기**: HUD 글자 ~10줄 × 9 = 90 `AddText`. 1080p에서 무시할 수준으로 예상(판단). 진단 탭 fps로 확인.
- **터치 목표 확대로 공간 부족**: 하단 띠 304 px로 늘리면 본문이 720 px. 팔 탭 5.8 계산상 674 px로 들어가지만 글꼴 교체로 줄 높이가 바뀌면 재측정 필요.
- **기능과 디자인의 경계**: 6.4의 "press 시 발동", 5.3의 "조종→주행" 이름, 속도 슬라이더 삭제(기능 중복 제거)는 사용자 결정 사항으로 표시했다. 결정 전에는 현재 동작을 유지한 채 모양만 바꾼다.
- **동시 편집**: 이 문서를 쓰는 동안 `gui_app.cpp`에 보안 수정이 들어갔다. 단계 1 시작 전에 최신 파일을 다시 읽는다.

---

## 10. 확인 방법

### 10.1 스크린샷 명령(Ally에서)

```
armpi-ally.exe --smoke-test --screenshot before_drive.bmp --demo-connect HOST --demo-ports BASE --tab drive
armpi-ally.exe --smoke-test --screenshot before_arm.bmp   --demo-connect HOST --demo-ports BASE --tab arm
armpi-ally.exe --smoke-test --screenshot before_diag.bmp  --demo-connect HOST --demo-ports BASE --tab diag
armpi-ally.exe --smoke-test --screenshot before_settings.bmp --demo-connect HOST --demo-ports BASE --tab settings
armpi-ally.exe --smoke-test --screenshot before_connect.bmp            (연결 없이 → 연결 화면)
```
HOST/BASE는 가짜 로봇(`mock_robot.py`) 값. 데모 모드는 주소를 "데모 로봇"으로 가리므로 결과 이미지를 그대로 문서에 쓸 수 있다. 전체화면 모드와 비상정지 latched 상태는 지금 옵션으로 만들 수 없으므로, 단계 3·4에서 `--demo-state estop|cambig` 같은 **시험 전용 옵션 추가**를 제안한다(기능 아님, 표시 상태만 강제).

### 10.2 전·후 비교 기준

| 항목 | 측정 방법 | 전(현재) | 목표 |
|---|---|---|---|
| 정상 상태에서 채도 높은 색 면적 | 스크린샷에서 ok/warn/danger/accent 계열 픽셀 비율(파이썬으로 HSV S>0.5 & V>0.5 계수) | 상단 칩·탭·버튼·속도 등 상당 | 비상정지 버튼 + 선택 표시 외 2 % 미만 |
| 글자 대비 | 4.1 표의 조합을 스크립트로 재계산 | 녹/노/주황 위 흰 글자 2.1–2.9 | 본문 ≥ 4.5, 큰 글자 ≥ 3.0 |
| 터치 목표 | 코드에서 `ImVec2` 크기 목록 추출(grep `ImVec2(S(`) | 44–56 px 다수 | 주요 ≥ 88, 보조 ≥ 64 |
| 팔 탭 스크롤 | 스크린샷에 스크롤바 유무 | 있음 | 없음 |
| 잘린 글자 | 스크린샷 육안 + 표 첫 열 너비 | 2곳 이상 | 0 |
| 속도 상한 표시 수 | 화면에서 세기 | 3 | 1(안전 열) + HUD 숫자 1 |
| 글자 크기 단계 수 | 코드 `PushFont(…, N)` 수집 | 22/26/28/30/34/40 산발 | 토큰 8단계만 |
| 모서리 반경 수 | 코드 수집 | 4종 | 3종(S/M/L) |
| 패드 힌트 양식 | 육안 | 괄호 글자 혼재 | PadBadge 단일 양식 |
| 숫자 흔들림 | ACK 값이 다른 스크린샷 2장에서 상단 줄 요소 x 좌표 비교 | 뒤 요소가 밀림 | 고정 |
| 프레임 시간 | 진단 탭 "화면 fps" 120 유지 여부(Ally, fps_cap 120) | 120 | 120 |

### 10.3 사람 확인(사용자)

- 손에 들고 팔을 뻗은 자세에서 상단 상태 줄이 읽히는지, 엄지로 비상정지·속도 세그먼트가 닿는지.
- 밝은 실내 영상 위에서 HUD 글자가 읽히는지.
- 비상정지 latched 호흡이 거슬리지 않는지(1 Hz, 알파 0.4–1.0).

---

## 11. 가장 중요한 디자인 결정 5개

1. **색은 상태와 선택에만** — 표면은 무채색 5단계, 정상 상태는 작은 ● 하나. 칩 12개를 상태 줄 1줄로 바꾼다.
2. **비상정지는 4가지 모양을 가진다**(준비·눌림·정지됨·비활성) — 항상 같은 자리, 192×192, 해제는 진행 링.
3. **패드 배지 한 양식** — 괄호 글자 대신 원/둥근 사각형 안의 글자를 직접 그린다. 상표 로고·색상 관례는 쓰지 않는다.
4. **글꼴 내장 + 고정폭 숫자** — Pretendard 4굵기와 JetBrains Mono, Lucide 아이콘. 8단계 크기 토큰만 쓴다.
5. **한 탭은 한 화면** — 조종 탭의 진단 정보는 진단으로, 속도는 안전 열 하나로, 팔 탭은 스크롤 없이 674 px 안에.

---

## 부록 A. 열어 본 자료(링크)

2절 표에 링크를 모두 넣었다. 직접 연 것만 다시 모으면 다음과 같다.

- 로봇 조종: Foxglove 문서 3편, QGroundControl Fly View 3편, Mission Planner 2편, Clearpath 2편, Vizanti README, roslibjs 튜토리얼, Webviz README, Spot User Guide R2.0 PDF(제3자 사본), DJI Mini 4 Pro 매뉴얼 PDF, dronesgator DJI Fly 가이드, Unitree 매뉴얼 p.28·제품 페이지, MacRumors Vector 리뷰.
- 게임패드·접근성: ASUS ROG Ally 팁 글, Microsoft Game Bar 위젯 UI, UWP Gamepad interactions, Designing for TV, XAG 101·102, XR-022, WCAG 2.2 Understanding 1.4.3·1.4.11·2.5.8, Android 터치 목표 도움말, Okabe & Ito, Carbon 색 사용.
- OSD: Betaflight OSD 탭 문서, Oscar Liang Betaflight OSD·WTFOS 가이드, EBU R 95 PDF, Wikipedia Safe area.
- ImGui: `third_party/imgui/imgui.h`(1.92.9b), v1.92.0 릴리스 노트, Gallery #6478·#7959·#8333 댓글(API), #707 댓글(API), ImThemes, hello_imgui, IconFontCppHeaders.
- HMI·안전: ISA101 위원회 페이지, Ignition HPHMI 기법, Control Design 2021·2023, Schneider FAQ, Blue Robotics Cockpit PR #830.
- 글꼴: Pretendard 저장소·릴리스·OpenType 기능 블로그, JetBrains Mono 저장소, Material Design Icons LICENSE·variablefont 목록, Lucide LICENSE, IBM Plex 릴리스(KR 자산 미표시).

## 부록 B. 미확인·사용자 결정 필요

미확인:
- 글꼴 파일 실제 크기(내려받아 측정). Pretendard·JetBrains Mono·Lucide 글꼴 파일 모두 페이지에 크기 표시 없음.
- Font Awesome Free 라이선스 페이지가 제목만 반환되어 확인 못함(Lucide 추천이므로 영향 없음). IBM Plex Sans KR 자산·라이선스 미확인.
- Ally X 패널의 실제 ppi·표시 영역 크기(제조사 미공개; 7" 16:9 계산값 약 315 ppi, 155×87 mm 사용).
- Lucide 아이콘 글꼴 파일(TTF) 배포 여부(저장소 LICENSE 페이지에서는 미확인). 없으면 Material Symbols subset.
- 규격 원문(ISO 13850, IEC 60204-1, ISA-101.01)은 미열람. 2차 자료 기준.

사용자 결정:
- 화면 정지 버튼의 터치 발동 시점(현재 `ImGui::Button` = 손을 뗄 때 → 누르는 순간으로 바꿀지). 기능 변경이므로 디자인 계획 밖.
- 버튼 명칭 "비상정지" 유지 vs "정지(소프트웨어)". 문서(`SAFETY_KO.md`, `CONTROLS_KO.md`)와 함께 바꿔야 한다.
- 탭 이름 "조종"→"주행".
- 조종 탭 속도 슬라이더 삭제(안전 열 세그먼트 + D-pad ±10 %로 통일).
- 글자 배율 설정(1.0–1.3) 추가 — 설정 항목이 하나 늘어난다.
- FreeType 사용 여부(한글 소형 글자 품질 확인 후).
- 시험 전용 `--demo-state estop|cambig` 옵션 추가(스크린샷용 표시 상태 강제).
