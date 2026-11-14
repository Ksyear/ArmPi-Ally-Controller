#include "net/crypto.hpp"

namespace ally::net {

namespace {
inline uint32_t rol(uint32_t v, int n) { return (v << n) | (v >> (32 - n)); }
}  // namespace

std::array<uint8_t, 20> sha1(std::string_view data) {
  uint32_t h0 = 0x67452301, h1 = 0xEFCDAB89, h2 = 0x98BADCFE, h3 = 0x10325476, h4 = 0xC3D2E1F0;
  std::string msg(data);
  const uint64_t bit_len = static_cast<uint64_t>(data.size()) * 8;
  msg.push_back(static_cast<char>(0x80));
  while (msg.size() % 64 != 56) msg.push_back('\0');
  for (int i = 7; i >= 0; --i) msg.push_back(static_cast<char>((bit_len >> (8 * i)) & 0xFF));
  for (size_t off = 0; off < msg.size(); off += 64) {
    uint32_t w[80];
    for (int i = 0; i < 16; ++i) {
      w[i] = (static_cast<uint32_t>(static_cast<uint8_t>(msg[off + 4 * i])) << 24) |
             (static_cast<uint32_t>(static_cast<uint8_t>(msg[off + 4 * i + 1])) << 16) |
             (static_cast<uint32_t>(static_cast<uint8_t>(msg[off + 4 * i + 2])) << 8) |
             static_cast<uint32_t>(static_cast<uint8_t>(msg[off + 4 * i + 3]));
    }
    for (int i = 16; i < 80; ++i) w[i] = rol(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);
    uint32_t a = h0, b = h1, c = h2, d = h3, e = h4;
    for (int i = 0; i < 80; ++i) {
      uint32_t f, k;
      if (i < 20) { f = (b & c) | (~b & d); k = 0x5A827999; }
      else if (i < 40) { f = b ^ c ^ d; k = 0x6ED9EBA1; }
      else if (i < 60) { f = (b & c) | (b & d) | (c & d); k = 0x8F1BBCDC; }
      else { f = b ^ c ^ d; k = 0xCA62C1D6; }
      const uint32_t t = rol(a, 5) + f + e + k + w[i];
      e = d; d = c; c = rol(b, 30); b = a; a = t;
    }
    h0 += a; h1 += b; h2 += c; h3 += d; h4 += e;
  }
  std::array<uint8_t, 20> out{};
  const uint32_t hs[5] = {h0, h1, h2, h3, h4};
  for (int i = 0; i < 5; ++i)
    for (int k = 0; k < 4; ++k) out[static_cast<size_t>(4 * i + k)] = static_cast<uint8_t>(hs[i] >> (24 - 8 * k));
  return out;
}

std::string base64_encode(std::string_view data) {
  static const char* t = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string o;
  size_t i = 0;
  while (i + 2 < data.size()) {
    const uint32_t v = (static_cast<uint8_t>(data[i]) << 16) | (static_cast<uint8_t>(data[i + 1]) << 8) |
                       static_cast<uint8_t>(data[i + 2]);
    o.push_back(t[(v >> 18) & 63]);
    o.push_back(t[(v >> 12) & 63]);
    o.push_back(t[(v >> 6) & 63]);
    o.push_back(t[v & 63]);
    i += 3;
  }
  if (i + 1 == data.size()) {
    const uint32_t v = static_cast<uint8_t>(data[i]) << 16;
    o.push_back(t[(v >> 18) & 63]);
    o.push_back(t[(v >> 12) & 63]);
    o += "==";
  } else if (i + 2 == data.size()) {
    const uint32_t v = (static_cast<uint8_t>(data[i]) << 16) | (static_cast<uint8_t>(data[i + 1]) << 8);
    o.push_back(t[(v >> 18) & 63]);
    o.push_back(t[(v >> 12) & 63]);
    o.push_back(t[(v >> 6) & 63]);
    o.push_back('=');
  }
  return o;
}

std::string websocket_accept(const std::string& key) {
  const auto h = sha1(key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11");
  return base64_encode(std::string_view(reinterpret_cast<const char*>(h.data()), h.size()));
}

}  // namespace ally::net
