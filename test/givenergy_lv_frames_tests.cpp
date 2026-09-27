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

// The values behind the captured replies below, in wire units.
givenergy_lv::Snapshot captured_snapshot() {
  givenergy_lv::Snapshot s{};
  std::strcpy(s.serial, "EM2026G001");
  s.firmware = 3020;
  s.capacity_Ah = 160;
  s.clock_hash = 0x3448A151;
  s.status = 0x00CF;
  s.alarms = 0;
  s.soc_pct = 100;
  s.voltage_cV = 5617;
  s.current_cA = -11;
  s.temperature_C = 26;
  s.limit_cA = 9000;
  s.charge_limit_cA = 320;  // the BMS cut charging to 3.20 A at the top of charge
  s.discharge_limit_cA = 8000;
  const int16_t sensors[5] = {260, 253, 253, 260, 259};
  std::memcpy(s.sensor_temps_dC, sensors, sizeof(sensors));
  s.cell_count = 16;
  s.cycles = 740;
  s.pack_voltage_mV = 56224;
  s.cell_sum_mV = 56155;
  s.current_mA = -105;
  s.full_capacity_cAh = 14766;
  s.design_capacity_cAh = 16000;
  s.remaining_cAh = 14256;
  s.block2_soc_pct = 97;
  s.block2_word28 = 0x0610;
  s.block2_word32 = 0;
  const uint16_t cells[16] = {3517, 3520, 3509, 3472, 3506, 3507, 3511, 3519,
                              3512, 3514, 3511, 3514, 3515, 3504, 3509, 3515};
  std::memcpy(s.cells_mV, cells, sizeof(cells));
  s.temp_max_dC = 260;
  s.temp_min_dC = 253;
  s.cell_max_mV = 3520;
  s.cell_min_mV = 3472;
  return s;
}

std::vector<uint8_t> reply_to(const std::vector<uint8_t>& request, const givenergy_lv::Snapshot& s) {
  uint8_t out[givenergy_lv::kMaxReplyLen];
  const size_t len = givenergy_lv::build_reply(request.data(), s, out);
  return std::vector<uint8_t>(out, out + len);
}

// Captured reply to kHrPoll.
const std::vector<uint8_t> kCapturedHr = {
    0x01, 0x03, 0x38, 0x00, 0x65, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x45, 0x4D, 0x32,
    0x30, 0x32, 0x36, 0x47, 0x30, 0x30, 0x31, 0xFF, 0xFF, 0x00, 0xA0, 0x00, 0x30, 0x0B, 0xCC, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0xA1, 0x51, 0x34, 0x48, 0x00, 0xCF, 0x00, 0x00, 0x00, 0x64, 0x15,
    0xF1, 0xFF, 0xF5, 0x00, 0x1A, 0x23, 0x28, 0x01, 0x40, 0x1F, 0x40, 0x9F, 0xE9,
};

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

TEST(GivEnergyLvFrames, FullHoldingPollMatchesCapture) {
  EXPECT_EQ(reply_to(kHrPoll, captured_snapshot()), kCapturedHr);
}

TEST(GivEnergyLvFrames, ShortPollReturnsHr17ToHr25) {
  // The ARM switches to this poll when HR109 is not 1.
  std::vector<uint8_t> expected = {0x01, 0x03, 18};
  expected.insert(expected.end(), kCapturedHr.begin() + 3 + 34, kCapturedHr.begin() + 3 + 52);
  EXPECT_EQ(reply_to(with_crc({0x01, 0x03, 0x00, 0x11, 0x00, 0x09}), captured_snapshot()), with_crc(expected));
}

TEST(GivEnergyLvFrames, PollPastHr27IsRejected) {
  EXPECT_EQ(reply_to(with_crc({0x01, 0x03, 0x00, 0x10, 0x00, 0x0D}), captured_snapshot()),
            with_crc({0x01, 0x83, 0x02}));
}

TEST(GivEnergyLvFrames, HoldingPollToAnEmptySlotGetsNoReply) {
  EXPECT_TRUE(reply_to(with_crc({0x02, 0x03, 0x00, 0x00, 0x00, 0x1C}), captured_snapshot()).empty());
}
