#include <gtest/gtest.h>

#include <deque>
#include <vector>

#include "../Software/src/datalayer/datalayer.h"
#include "../Software/src/devboard/utils/types.h"
#include "../Software/src/inverter/GIVENERGY-LV-FRAMES.h"
#include "../Software/src/inverter/GIVENERGY-LV-RS485.h"
#include "../Software/src/inverter/INVERTERS.h"

// Tests for the GivEnergy LV inverter module: how it frames requests out of
// the serial stream and when it replies. The wire format itself is covered by
// givenergy_lv_frames_tests.cpp.

namespace {

// A serial port the test feeds and reads back.
class FakeSerial : public HardwareSerial {
 public:
  std::deque<uint8_t> rx;
  std::vector<uint8_t> tx;

  int available() override { return static_cast<int>(rx.size()); }
  int read() override {
    if (rx.empty()) {
      return -1;
    }
    const uint8_t byte = rx.front();
    rx.pop_front();
    return byte;
  }
  size_t write(uint8_t byte) override {
    tx.push_back(byte);
    return 1;
  }
  size_t write(const uint8_t* buffer, size_t size) override {
    tx.insert(tx.end(), buffer, buffer + size);
    return size;
  }
  void feed(const std::vector<uint8_t>& bytes) { rx.insert(rx.end(), bytes.begin(), bytes.end()); }
};

std::vector<uint8_t> with_crc(std::vector<uint8_t> frame) {
  const uint16_t crc = givenergy_lv::crc16(frame.data(), frame.size());
  frame.push_back(crc & 0xFF);
  frame.push_back(crc >> 8);
  return frame;
}

const std::vector<uint8_t> kHrPoll = {0x01, 0x03, 0x00, 0x00, 0x00, 0x1C, 0x44, 0x03};

class GivEnergyLvInverter : public ::testing::Test {
 protected:
  void SetUp() override {
    datalayer = DataLayer();
    set_millis64(1000);
  }
};

}  // namespace

TEST_F(GivEnergyLvInverter, RepliesAfterTheTurnaround) {
  FakeSerial port;
  GivEnergyLvRs485Inverter inverter(port);
  port.feed(kHrPoll);
  inverter.receive();
  EXPECT_TRUE(port.tx.empty()) << "replied inside the 3.5-character turnaround";
  set_millis64(1004);
  inverter.receive();
  ASSERT_EQ(port.tx.size(), 61u);
  EXPECT_EQ(port.tx[0], 0x01);
  EXPECT_EQ(port.tx[1], 0x03);
  EXPECT_EQ(port.tx[2], 0x38);
}

TEST_F(GivEnergyLvInverter, FindsAPollAfterLineNoise) {
  FakeSerial port;
  GivEnergyLvRs485Inverter inverter(port);
  port.feed({0x00, 0xFF, 0x13, 0x01});
  port.feed(kHrPoll);
  inverter.receive();
  set_millis64(1004);
  inverter.receive();
  EXPECT_EQ(port.tx.size(), 61u);
}

TEST_F(GivEnergyLvInverter, IgnoresItsOwnWriteEchoButAnswersALaterRetry) {
  FakeSerial port;
  GivEnergyLvRs485Inverter inverter(port);
  const std::vector<uint8_t> write = with_crc({0x01, 0x06, 0x00, 0x02, 0x00, 0x01});
  port.feed(write);
  inverter.receive();
  set_millis64(1004);
  inverter.receive();
  ASSERT_EQ(port.tx, write);

  port.feed(write);  // the transceiver hearing our own reply
  set_millis64(1010);
  inverter.receive();
  set_millis64(1020);
  inverter.receive();
  EXPECT_EQ(port.tx.size(), 8u) << "answered its own echo";

  set_millis64(1200);  // the inverter genuinely writing again
  port.feed(write);
  inverter.receive();
  set_millis64(1204);
  inverter.receive();
  EXPECT_EQ(port.tx.size(), 16u);
}

TEST_F(GivEnergyLvInverter, DISABLED_ReplyUsesTheDatalayerFromTheLastUpdate) {
  FakeSerial port;
  GivEnergyLvRs485Inverter inverter(port);
  datalayer.aggregate.max_charge_current_dA = 500;  // 50.0 A
  inverter.update_values();
  port.feed(kHrPoll);
  inverter.receive();
  set_millis64(1004);
  inverter.receive();
  ASSERT_EQ(port.tx.size(), 61u);
  EXPECT_EQ(port.tx[3 + 52], 0x13);  // HR26 = 5000 (50.00 A)
  EXPECT_EQ(port.tx[3 + 53], 0x88);
}

TEST(GivEnergyLvRegistration, IsListedByName) {
  EXPECT_STREQ(name_for_inverter_type(InverterProtocolType::GivEnergyLV485), GivEnergyLvRs485Inverter::Name);
}
