# Third-party notices

| 구성 요소 | 판 | 라이선스 | 위치 | 비고 |
|---|---|---|---|---|
| ArmPro_Cpp_ROS2 발췌(Protocol v3 클라이언트, 엄격 JSON, 팔 기구학·목표 보정) | 커밋 4ab02ad | Apache-2.0 | `third_party/armpi/` | 수정 없음. 파일 목록·원본 경로는 `third_party/armpi/NOTICE` |
| Dear ImGui (본체, Win32·DX11 백엔드) | v1.92.9b | MIT | `third_party/imgui/` | `LICENSE.txt` 동봉. 수정 없음 |
| stb_image | v2.30 (커밋 013ac3b) | MIT 또는 Public Domain(택1) | `third_party/stb/stb_image.h` | 수정 없음. JPEG·PNG만 켜서 씀 |

- 한글 글꼴(맑은 고딕)은 배포하지 않는다. 실행할 때 Windows에 깔린 `%WINDIR%\Fonts\malgun.ttf`, `malgunbd.ttf`를 읽는다.
- 시험용 가짜 로봇 `mock_robot.py`는 이 저장소에 넣지 않는다. 위 ArmPro_Cpp_ROS2 저장소의 `protocol_v3/mock_robot.py`를 경로로 가리켜 쓴다.
