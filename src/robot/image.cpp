// stb_image implementation unit (JPEG + PNG only, no file I/O).
#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_JPEG
#define STBI_ONLY_PNG
#define STBI_NO_STDIO
#define STBI_FAILURE_USERMSG
#define STBI_MAX_DIMENSIONS 4096  // reject "pixel bombs" before allocating (robot camera is 640x480)
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wsign-compare"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#if !defined(__clang__)
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#endif
#endif
#include "stb_image.h"
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

#include "robot/camera.hpp"

namespace ally::robot {

bool decode_image(const std::string& bytes, int* w, int* h, std::vector<uint8_t>* rgba, std::string* err) {
  if (bytes.empty() || bytes.size() > kMaxFrameBytes) {
    if (err) *err = "영상 프레임 크기 오류";
    return false;
  }
  int comp = 0;
  // Read the header first: a few KB of PNG can declare a multi-gigabyte image.
  if (!stbi_info_from_memory(reinterpret_cast<const stbi_uc*>(bytes.data()), static_cast<int>(bytes.size()), w, h, &comp) ||
      *w <= 0 || *h <= 0 || static_cast<long long>(*w) * *h > kMaxFramePixels) {
    if (err) *err = "영상 프레임 치수 거부";
    return false;
  }
  stbi_uc* px = stbi_load_from_memory(reinterpret_cast<const stbi_uc*>(bytes.data()), static_cast<int>(bytes.size()),
                                      w, h, &comp, 4);
  if (!px) {
    if (err) *err = std::string("영상 해독 실패: ") + stbi_failure_reason();
    return false;
  }
  rgba->assign(px, px + static_cast<size_t>(*w) * static_cast<size_t>(*h) * 4);
  stbi_image_free(px);
  return true;
}

}  // namespace ally::robot
