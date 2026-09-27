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

TEST_F(GivEnergyLvInverter, StillWithholdsTheReplyThreeMillisecondsIn) {
  FakeSerial port;
  GivEnergyLvRs485Inverter inverter(port);
  port.feed(kHrPoll);
  inverter.receive();
  set_millis64(1003);
  inverter.receive();
  EXPECT_TRUE(port.tx.empty()) << "replied inside the 3.5-character turnaround";
  set_millis64(1004);
  inverter.receive();
  EXPECT_EQ(port.tx.size(), 61u);
}

TEST_F(GivEnergyLvInverter, AssemblesARequestSplitAcrossTwoReceiveCalls) {
  FakeSerial port;
  GivEnergyLvRs485Inverter inverter(port);
  port.feed({kHrPoll.begin(), kHrPoll.begin() + 3});
  inverter.receive();
  port.feed({kHrPoll.begin() + 3, kHrPoll.end()});
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

TEST_F(GivEnergyLvInverter, IgnoresAnEchoThatArrivesInTheSameCallAsTheSend) {
  FakeSerial port;
  GivEnergyLvRs485Inverter inverter(port);
  const std::vector<uint8_t> write = with_crc({0x01, 0x06, 0x00, 0x02, 0x00, 0x01});
  port.feed(write);
  inverter.receive();

  set_millis64(1004);
  port.feed(write);  // the echo is already on the wire before we send our reply this call
  inverter.receive();

  set_millis64(1010);
  inverter.receive();
  set_millis64(1020);
  inverter.receive();
  EXPECT_EQ(port.tx.size(), 8u) << "answered its own echo";
}

TEST_F(GivEnergyLvInverter, ReplyUsesTheDatalayerFromTheLastUpdate) {
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

TEST_F(GivEnergyLvInverter, LimitsGoToHr26ForChargeAndHr27ForDischarge) {
  datalayer.aggregate.max_charge_current_dA = 300;
  datalayer.aggregate.max_discharge_current_dA = 800;
  const givenergy_lv::Snapshot s = GivEnergyLvRs485Inverter::snapshot_from_datalayer();
  EXPECT_EQ(s.charge_limit_cA, 3000);
  EXPECT_EQ(s.discharge_limit_cA, 8000);
  EXPECT_EQ(s.limit_cA, 9000);
}

TEST_F(GivEnergyLvInverter, LimitsAreCappedAt90A) {
  datalayer.aggregate.max_charge_current_dA = 2000;  // a Seplos offering 200 A
  datalayer.aggregate.max_discharge_current_dA = 1500;
  const givenergy_lv::Snapshot s = GivEnergyLvRs485Inverter::snapshot_from_datalayer();
  EXPECT_EQ(s.charge_limit_cA, 9000);
  EXPECT_EQ(s.discharge_limit_cA, 9000);
}

TEST_F(GivEnergyLvInverter, FaultStopsChargeAndDischarge) {
  datalayer.aggregate.max_charge_current_dA = 500;
  datalayer.aggregate.max_discharge_current_dA = 500;
  datalayer.system.status.system_status = FAULT;
  const givenergy_lv::Snapshot s = GivEnergyLvRs485Inverter::snapshot_from_datalayer();
  EXPECT_EQ(s.charge_limit_cA, 0);
  EXPECT_EQ(s.discharge_limit_cA, 0);
}

TEST_F(GivEnergyLvInverter, StatusBitsFollowTheCurrent) {
  datalayer.aggregate.current_dA = -1;
  EXPECT_EQ(GivEnergyLvRs485Inverter::snapshot_from_datalayer().status, 0x00CF);
  datalayer.aggregate.current_dA = 50;
  EXPECT_EQ(GivEnergyLvRs485Inverter::snapshot_from_datalayer().status, 0x00CE);
  datalayer.aggregate.current_dA = 0;
  EXPECT_EQ(GivEnergyLvRs485Inverter::snapshot_from_datalayer().status, 0x00CD);
}

TEST_F(GivEnergyLvInverter, UnitsMatchTheWire) {
  datalayer.aggregate.voltage_dV = 562;
  datalayer.aggregate.current_dA = -123;
  datalayer.aggregate.reported_soc = 9700;
  datalayer.aggregate.reported_total_capacity_Wh = 32000;  // the Fogstar
  datalayer.aggregate.reported_remaining_capacity_Wh = 16000;
  const givenergy_lv::Snapshot s = GivEnergyLvRs485Inverter::snapshot_from_datalayer();
  EXPECT_EQ(s.voltage_cV, 5620);
  EXPECT_EQ(s.pack_voltage_mV, 56200);
  EXPECT_EQ(s.current_cA, -1230);
  EXPECT_EQ(s.current_mA, -12300);
  EXPECT_EQ(s.soc_pct, 97);
  EXPECT_EQ(s.block2_soc_pct, 97);
  EXPECT_EQ(s.capacity_Ah, 625);
  EXPECT_EQ(s.full_capacity_cAh, 62500);
  EXPECT_EQ(s.design_capacity_cAh, 62500);
  EXPECT_EQ(s.remaining_cAh, 31250);
  EXPECT_EQ(s.firmware, 3020);
  EXPECT_STREQ(s.serial, "EM2026G001");
}

TEST_F(GivEnergyLvInverter, TwoPacksClampTheBlock2Capacity) {
  datalayer.aggregate.reported_total_capacity_Wh = 64000;
  const givenergy_lv::Snapshot s = GivEnergyLvRs485Inverter::snapshot_from_datalayer();
  EXPECT_EQ(s.capacity_Ah, 1250);
  EXPECT_EQ(s.full_capacity_cAh, 65535);
}

TEST_F(GivEnergyLvInverter, SubZeroTemperaturesStayNegative) {
  datalayer.aggregate.temperature_max_dC = -30;
  datalayer.aggregate.temperature_min_dC = -52;
  const givenergy_lv::Snapshot s = GivEnergyLvRs485Inverter::snapshot_from_datalayer();
  EXPECT_EQ(s.temperature_C, -3);
  const int16_t expected[5] = {-30, -52, -52, -30, -41};
  for (int i = 0; i < 5; i++) {
    EXPECT_EQ(s.sensor_temps_dC[i], expected[i]) << "sensor " << i;
  }
  EXPECT_EQ(s.temp_max_dC, -30);
  EXPECT_EQ(s.temp_min_dC, -52);
}

TEST_F(GivEnergyLvInverter, ReportedCellsPassThrough) {
  for (int i = 0; i < 16; i++) {
    datalayer.battery.status.cell_voltages_mV[i] = 3300 + i;
  }
  const givenergy_lv::Snapshot s = GivEnergyLvRs485Inverter::snapshot_from_datalayer();
  EXPECT_EQ(s.cells_mV[0], 3300);
  EXPECT_EQ(s.cells_mV[15], 3315);
  EXPECT_EQ(s.cell_sum_mV, 16 * 3300 + 120);
}

TEST_F(GivEnergyLvInverter, MissingCellsAreFilledFromMinAndMax) {
  datalayer.aggregate.cell_max_voltage_mV = 3400;
  datalayer.aggregate.cell_min_voltage_mV = 3300;
  const givenergy_lv::Snapshot s = GivEnergyLvRs485Inverter::snapshot_from_datalayer();
  EXPECT_EQ(s.cells_mV[0], 3400);
  EXPECT_EQ(s.cells_mV[1], 3300);
  for (int i = 2; i < 16; i++) {
    EXPECT_EQ(s.cells_mV[i], 3350) << "cell " << i;
  }
  EXPECT_EQ(s.cell_sum_mV, 3400 + 3300 + 14 * 3350);
}

TEST_F(GivEnergyLvInverter, NoBatteryYetStillGivesInRangeValues) {
  datalayer.aggregate.cell_max_voltage_mV = 0;
  datalayer.aggregate.cell_min_voltage_mV = 0;
  const givenergy_lv::Snapshot s = GivEnergyLvRs485Inverter::snapshot_from_datalayer();
  for (int i = 0; i < 16; i++) {
    EXPECT_GE(s.cells_mV[i], 2200) << "cell " << i;
    EXPECT_LE(s.cells_mV[i], 3700) << "cell " << i;
  }
  EXPECT_EQ(s.charge_limit_cA, 0);
  EXPECT_EQ(s.discharge_limit_cA, 0);
  EXPECT_EQ(s.cell_count, 16);
}
