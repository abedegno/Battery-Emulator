#ifndef GIVENERGY_LV_FRAMES_H
#define GIVENERGY_LV_FRAMES_H

#include <stddef.h>
#include <stdint.h>

// Wire format of the GivEnergy LV battery bus as a G3 inverter polls it.
// Modbus RTU, except that FC4 replies echo the start address where standard
// Modbus has a byte count. Sources: bms-analysis docs/01 to docs/03, checked
// against a G3 capture (September 2026).
namespace givenergy_lv {

constexpr size_t kRequestLen = 8;

uint16_t crc16(const uint8_t* data, size_t len);

// True for an 8-byte FC3, FC4 or FC6 request with a valid CRC.
bool is_request(const uint8_t* frame);

}  // namespace givenergy_lv

#endif
