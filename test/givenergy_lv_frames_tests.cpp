#include <gtest/gtest.h>

#include <cstring>
#include <vector>

#include "../Software/src/inverter/GIVENERGY-LV-FRAMES.h"

// Wire tests for the GivEnergy LV battery protocol. Every captured frame here
// comes from my G3 (D0.316-A0.316) talking to a GivEnergy 8.2 kWh battery on
// BMS firmware 3020, 2026-09-27. The battery's serial is replaced with
// EM2026G001 and the CRC recomputed.

namespace {

std::vector<uint8_t> with_crc(std::vector<uint8_t> frame) {
  const uint16_t crc = givenergy_lv::crc16(frame.data(), frame.size());
  frame.push_back(crc & 0xFF);
  frame.push_back(crc >> 8);
  return frame;
}

// The inverter's HR0..HR27 poll to device 1, sent every ~245 ms.
const std::vector<uint8_t> kHrPoll = {0x01, 0x03, 0x00, 0x00, 0x00, 0x1C, 0x44, 0x03};

}  // namespace

TEST(GivEnergyLvFrames, CrcMatchesTheCapturedPoll) {
  // Modbus sends the CRC low byte first: 44 03 is 0x0344.
  EXPECT_EQ(givenergy_lv::crc16(kHrPoll.data(), 6), 0x0344);
}

TEST(GivEnergyLvFrames, RecognisesCapturedRequests) {
  EXPECT_TRUE(givenergy_lv::is_request(kHrPoll.data()));
  const uint8_t block2_device5[] = {0x05, 0x04, 0x00, 0x15, 0x00, 0x13, 0xA1, 0x87};
  EXPECT_TRUE(givenergy_lv::is_request(block2_device5));
}

TEST(GivEnergyLvFrames, RejectsBadCrcAndOtherFunctions) {
  std::vector<uint8_t> corrupted = kHrPoll;
  corrupted[5] = 0x1D;
  EXPECT_FALSE(givenergy_lv::is_request(corrupted.data()));
  // FC16 with a valid CRC: the battery never answers it.
  EXPECT_FALSE(givenergy_lv::is_request(with_crc({0x01, 0x10, 0x00, 0x00, 0x00, 0x01}).data()));
}
