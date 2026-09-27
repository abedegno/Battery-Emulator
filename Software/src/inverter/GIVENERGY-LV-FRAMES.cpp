#include "GIVENERGY-LV-FRAMES.h"

#include <string.h>

namespace givenergy_lv {

namespace {

void put16(uint8_t* p, uint16_t value) {
  p[0] = value >> 8;
  p[1] = value & 0xFF;
}

uint16_t get16(const uint8_t* p) {
  return static_cast<uint16_t>(p[0] << 8 | p[1]);
}

// Appends the CRC, low byte first, and returns the frame length.
size_t finish(uint8_t* out, size_t len) {
  const uint16_t crc = crc16(out, len);
  out[len] = crc & 0xFF;
  out[len + 1] = crc >> 8;
  return len + 2;
}

size_t exception(uint8_t* out, uint8_t device, uint8_t function, uint8_t code) {
  out[0] = device;
  out[1] = function | 0x80;
  out[2] = code;
  return finish(out, 3);
}

// The serial as the battery sends it: ASCII, padded with spaces.
void put_serial(uint8_t* p, size_t width, const char* serial) {
  memset(p, ' ', width);
  for (size_t i = 0; i < width && serial[i] != '\0'; i++) {
    p[i] = serial[i];
  }
}

void holding_registers(const Snapshot& s, uint16_t* r) {
  uint8_t serial[10];
  put_serial(serial, sizeof(serial), s.serial);
  r[0] = 0x0065;  // device marker
  r[1] = r[2] = r[3] = r[4] = 0xFFFF;
  for (int i = 0; i < 5; i++) {
    r[5 + i] = get16(serial + 2 * i);
  }
  r[10] = 0xFFFF;
  r[11] = s.capacity_Ah;
  r[12] = 0x0030;  // hardware revision
  r[13] = s.firmware;
  r[14] = r[15] = r[16] = 0;
  r[17] = s.clock_hash & 0xFFFF;
  r[18] = s.clock_hash >> 16;
  r[19] = s.status;
  r[20] = s.alarms;
  r[21] = s.soc_pct;
  r[22] = s.voltage_cV;
  r[23] = static_cast<uint16_t>(s.current_cA);
  r[24] = static_cast<uint16_t>(s.temperature_C);
  r[25] = s.limit_cA;
  r[26] = s.charge_limit_cA;
  r[27] = s.discharge_limit_cA;
}

}  // namespace

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

size_t build_reply(const uint8_t* request, const Snapshot& s, uint8_t* out) {
  const uint8_t device = request[0];
  const uint8_t function = request[1];
  const uint16_t start = get16(request + 2);
  const uint16_t count = get16(request + 4);
  if (device < kPackDevice || device > kLastDevice) {
    return 0;
  }
  const bool present = device == kPackDevice;

  switch (function) {
    case 3: {
      // The master battery only answers holding polls for itself.
      if (!present) {
        return 0;
      }
      if (count == 0 || start + count > kHoldingCount) {
        return exception(out, device, function, 2);
      }
      uint16_t registers[kHoldingCount];
      holding_registers(s, registers);
      out[0] = device;
      out[1] = function;
      out[2] = count * 2;
      for (uint16_t i = 0; i < count; i++) {
        put16(out + 3 + 2 * i, registers[start + i]);
      }
      return finish(out, 3 + count * 2);
    }
    default:
      return 0;
  }
}

}  // namespace givenergy_lv
