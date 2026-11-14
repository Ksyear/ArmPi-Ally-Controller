cmake_minimum_required(VERSION 3.16)
project(armpi_ally_controller VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Threads REQUIRED)

# Shared robot code copied from the robot repository (Apache-2.0, see NOTICE.md)
add_library(armpi_shared STATIC
  third_party/armpi/src/json.cpp
  third_party/armpi/src/protocol.cpp
  third_party/armpi/src/client_codec.cpp
  third_party/armpi/src/client.cpp
  third_party/armpi/src/arm_kinematics.cpp
  third_party/armpi/src/reachability.cpp)
target_include_directories(armpi_shared PUBLIC third_party/armpi/include)
target_compile_definitions(armpi_shared PUBLIC _USE_MATH_DEFINES)

# Platform-independent app core (sockets, protocol session, rosbridge, camera, gamepad, haptics)
add_library(ally_core STATIC
  src/common/util.cpp
  src/common/settings.cpp
  src/net/sock.cpp
  src/net/comm_log.cpp
  src/net/udp_link.cpp
  src/net/discovery.cpp
  src/net/http.cpp
  src/net/crypto.cpp
  src/net/websocket.cpp
  src/robot/rosbridge.cpp
  src/robot/camera.cpp
  src/robot/image.cpp
  src/robot/core.cpp
  src/input/pad.cpp
  src/input/mapper.cpp)
target_include_directories(ally_core PUBLIC src)
target_include_directories(ally_core SYSTEM PRIVATE third_party/stb)
target_link_libraries(ally_core PUBLIC armpi_shared Threads::Threads)

set(ALLY_WARNINGS -Wall -Wextra -Wno-unused-parameter)
target_compile_options(ally_core PRIVATE ${ALLY_WARNINGS})

if(WIN32)
  target_compile_definitions(ally_core PUBLIC _WIN32_WINNT=0x0A00 WINVER=0x0A00 WIN32_LEAN_AND_MEAN NOMINMAX
                                              UNICODE _UNICODE)
  target_link_libraries(ally_core PUBLIC ws2_32 iphlpapi)
  if(MINGW)
    # One self-contained exe: no libstdc++/libgcc/winpthread DLLs next to it.
    # ASLR (incl. high entropy) and DEP stated explicitly; stack protector + fortify for our code.
    set(ALLY_STATIC_LINK -static -static-libgcc -static-libstdc++ $<$<CONFIG:Release>:-s>
                         -Wl,--dynamicbase,--nxcompat,--high-entropy-va -fstack-protector-strong)
    set(ALLY_HARDEN -fstack-protector-strong -D_FORTIFY_SOURCE=2)
    target_compile_options(ally_core PRIVATE ${ALLY_HARDEN})
    target_compile_options(armpi_shared PRIVATE ${ALLY_HARDEN})
  endif()
endif()

add_executable(armpi-ally-cli src/app/cli_main.cpp)
target_link_libraries(armpi-ally-cli PRIVATE ally_core ${ALLY_STATIC_LINK})
target_compile_options(armpi-ally-cli PRIVATE ${ALLY_WARNINGS} ${ALLY_HARDEN})

if(WIN32 AND EXISTS ${CMAKE_SOURCE_DIR}/src/app/gui_main.cpp)
  add_library(imgui STATIC
    third_party/imgui/imgui.cpp
    third_party/imgui/imgui_draw.cpp
    third_party/imgui/imgui_tables.cpp
    third_party/imgui/imgui_widgets.cpp
    third_party/imgui/backends/imgui_impl_win32.cpp
    third_party/imgui/backends/imgui_impl_dx11.cpp)
  target_include_directories(imgui SYSTEM PUBLIC third_party/imgui third_party/imgui/backends)
  # We own the gamepad (XInput in src/input/pad.cpp); ImGui must not read it too.
  target_compile_definitions(imgui PUBLIC UNICODE _UNICODE IMGUI_IMPL_WIN32_DISABLE_GAMEPAD)
  add_executable(armpi-ally WIN32 src/app/gui_main.cpp src/app/gui_app.cpp res/app.rc)
  target_link_libraries(armpi-ally PRIVATE ally_core imgui d3d11 dxgi d3dcompiler dwmapi winmm shell32
                                           ${ALLY_STATIC_LINK})
  target_compile_options(armpi-ally PRIVATE ${ALLY_WARNINGS} ${ALLY_HARDEN})
endif()

enable_testing()
if(EXISTS ${CMAKE_SOURCE_DIR}/test/unit_tests.cpp)
  add_executable(ally_unit_tests test/unit_tests.cpp)
  target_link_libraries(ally_unit_tests PRIVATE ally_core ${ALLY_STATIC_LINK})
  add_test(NAME unit COMMAND ally_unit_tests)
endif()
