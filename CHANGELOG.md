# Changelog

형식: [Keep a Changelog](https://keepachangelog.com/ko/1.1.0/), 버전: SemVer.

## [0.2.0] - 2026-11-14

### 바뀜(디자인)
- 화면을 다시 그렸다(`docs/DESIGN_PLAN_v001.md`). 디자인 토큰(색 5단계 표면, 상태색, 글자 8단계, 간격 4/8 격자, 모서리 3종)과 위젯 모음(`src/app/ui_theme.*`, `ui_widgets.*`)을 따로 두었다.
- 상단의 칩 12개를 상태 줄 하나로 바꿨다. 상태는 색·모양·글자로 함께 보이고, 오른쪽에 계기(배터리·ACK·영상·패드)와 모드 전환(주행·팔·진단·설정)이 있다.
- 비상정지 버튼은 준비·정지됨·비활성 세 가지 모양이고, **누르는 순간 발동**한다(사용자 결정). 해제는 진행 링이 도는 길게 누르기다.
- 카메라 HUD: 화면을 꽉 채우고 가장자리를 어둡게 덮어 글자를 외곽선으로 그린다. 알림은 위쪽 가운데에 쌓인다. 영상 끄기·크게 보기는 아이콘 버튼이다.
- 하단: 팔 버튼을 축별 짝으로 묶었다(112 px). 패드 모니터와 게임패드 배지를 넣었다.
- 탭 이름 '조종'을 '주행'으로 바꿨다. 주행 탭의 중복 속도 슬라이더를 없앴다. 글자 크기 설정(1.0–1.3)을 추가했다.
- 글꼴을 내장했다: Pretendard, JetBrains Mono, Lucide 아이콘 부분 집합(THIRD_PARTY_NOTICES).
- 문서용 화면 옵션 `--demo-state estop|cambig`.

### 보안
- 보안 검토 반영(`docs/SECURITY_KO.md`).

## [0.1.0] - 2026-11-14

### 추가
- Winsock/POSIX 공용 소켓 계층, Protocol v3 UDP 링크(주고받은 패킷 기록), 로봇 검색(UDP 5002).
- `RobotCore`: 20 Hz 조종 루프, 입력 0.3초 끊김 시 중립, 팔 목표 램프(0.15 m/s · 0.6 rad/s · 0.8/s), 앱 쪽 도달 범위 보정과 로봇 보정 따라가기, 터치 팔 토글 60초 자동 꺼짐, 비상정지·해제, 세션 놓기·새 세션.
- rosbridge 클라이언트: `/armpi/status`, `/armpi/battery`, `/joint_states`, `/armpi/arm/target`, `/armpi_vr_bridge/state` 구독, 기본 자세·전체 정지·전체 해제 서비스, 관절 직접 조종 발행.
- 카메라: MJPEG 수신 + stb_image 해독 + D3D11 텍스처 + HUD.
- XInput 게임패드(런타임 로드)와 진동 패턴, Mapper(패드·키보드·터치 → 조종 입력).
- Win32 + D3D11 + Dear ImGui 화면: 상단 상태 칩, 카메라 HUD, 조종·팔·진단·설정 탭, 가상 스틱·회전 슬라이더·팔 버튼, 팔 위·옆 그림 목표 지정, IP 숫자판, 전체 화면, 화면 꺼짐 방지, Ally 배터리, 단일 실행.
- 콘솔 도구 `armpi-ally-cli`: `discover`, `selftest`, `pads`.
- 단위 시험 60항목, 가짜 로봇 종단 시험 21항목, CI(우분투 mingw 크로스 빌드 + 시험).
