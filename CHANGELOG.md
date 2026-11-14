# Changelog

형식: [Keep a Changelog](https://keepachangelog.com/ko/1.1.0/), 버전: SemVer.

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
