// WebSocket 핸드셰이크용 SHA-1·Base64 (보안 용도 아님: RFC 6455 Sec-WebSocket-Accept 확인에만 쓴다)
#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <string_view>

namespace ally::net {

std::array<uint8_t, 20> sha1(std::string_view data);
std::string base64_encode(std::string_view data);
std::string websocket_accept(const std::string& key);

}  // namespace ally::net
