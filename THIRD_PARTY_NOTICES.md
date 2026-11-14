# Third-party notices

| 구성 요소 | 판 | 라이선스 | 위치 | 비고 |
|---|---|---|---|---|
| ArmPro_Cpp_ROS2 발췌(Protocol v3 클라이언트, 엄격 JSON, 팔 기구학·목표 보정) | 커밋 4ab02ad | Apache-2.0 | `third_party/armpi/` | 수정 없음. 파일 목록·원본 경로는 `third_party/armpi/NOTICE` |
| Dear ImGui (본체, Win32·DX11 백엔드) | v1.92.9b | MIT | `third_party/imgui/` | `LICENSE.txt` 동봉. 수정 없음 |
| stb_image | v2.30 (커밋 013ac3b) | MIT 또는 Public Domain(택1) | `third_party/stb/stb_image.h` | 수정 없음. JPEG·PNG만 켜서 씀 |
| Pretendard (Regular·SemiBold·Bold) | v1.3.9 | SIL OFL 1.1 | `third_party/fonts/pretendard/` | 본문 글꼴. 수정 없이 exe에 내장(4624 KB). LICENSE 동봉 |
| JetBrains Mono (Regular·Medium) | v2.304 | SIL OFL 1.1 | `third_party/fonts/jetbrains-mono/` | 숫자용 고정폭 글꼴. 내장(535 KB). OFL.txt 동봉 |
| Lucide 아이콘 글꼴 | lucide-static 1.64.0 | ISC | `third_party/fonts/lucide/` | 쓰는 아이콘 45개만 남긴 부분 집합(15 KB, fontTools pyftsubset). 목록 `used_icons.txt` |

- 내장 글꼴을 읽지 못하면 Windows의 맑은 고딕으로 대신 그린다(배포하지 않음).
- 시험용 가짜 로봇 `mock_robot.py`는 이 저장소에 넣지 않는다. 위 ArmPro_Cpp_ROS2 저장소의 `protocol_v3/mock_robot.py`를 경로로 가리켜 쓴다.
