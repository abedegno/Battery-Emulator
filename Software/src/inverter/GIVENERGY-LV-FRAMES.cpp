#include "GIVENERGY-LV-FRAMES.h"

namespace givenergy_lv {

uint16_t crc16(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; bit++) {
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
    }
  }
  return crc;
}

bool is_request(const uint8_t* frame) {
  if (frame[1] != 3 && frame[1] != 4 && frame[1] != 6) {
    return false;
  }
  return crc16(frame, 6) == (frame[6] | frame[7] << 8);
}

}  // namespace givenergy_lv
