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
    // A real pack voltage, so update_values() marks the module ready to reply. Tests that care
    // about the not-ready state override this back to 0 before constructing the inverter.
    datalayer.battery.status.voltage_dV = 520;
    // A normal mid-charge cell voltage, below the taper (see ChargeTaper* tests below): without
    // this, DataLayer's cell_max_voltage_mV default of 3700 mV would taper HR26 to 0 in every
    // test here that doesn't set the aggregate cell voltages itself. cell_min_voltage_mV also
    // defaults to 3700 mV, which is above cell_max here and would leave every test with an
    // unrealistic inverted pack, so give it a matching normal value too.
    datalayer.aggregate.cell_max_voltage_mV = 3300;
    datalayer.aggregate.cell_min_voltage_mV = 3280;
  }
};

}  // namespace

TEST_F(GivEnergyLvInverter, RepliesAfterTheTurnaround) {
  FakeSerial port;
  GivEnergyLvRs485Inverter inverter(port);
  inverter.update_values();
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
  inverter.update_values();
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
  inverter.update_values();
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
  inverter.update_values();
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
  inverter.update_values();
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
  inverter.update_values();
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

TEST_F(GivEnergyLvInverter, StaysSilentUntilTheBatteryHasReported) {
  FakeSerial port;
  datalayer.battery.status.voltage_dV = 0;  // the battery hasn't decoded a pack voltage yet
  GivEnergyLvRs485Inverter inverter(port);
  inverter.update_values();
  port.feed(kHrPoll);
  inverter.receive();
  set_millis64(1004);
  inverter.receive();
  EXPECT_TRUE(port.tx.empty()) << "replied before the battery had reported a real voltage";

  datalayer.battery.status.voltage_dV = 520;
  datalayer.aggregate.voltage_dV = 520;
  inverter.update_values();
  port.feed(kHrPoll);
  set_millis64(1100);
  inverter.receive();
  set_millis64(1104);
  inverter.receive();
  ASSERT_EQ(port.tx.size(), 61u);
  EXPECT_EQ(port.tx[3 + 44], 0x14);  // HR22 = 5200 (52.00 V)
  EXPECT_EQ(port.tx[3 + 45], 0x50);
}

TEST_F(GivEnergyLvInverter, DiscardsRequestsHeardBeforeReady) {
  FakeSerial port;
  datalayer.battery.status.voltage_dV = 0;  // the battery hasn't decoded a pack voltage yet
  GivEnergyLvRs485Inverter inverter(port);
  port.feed(kHrPoll);
  inverter.receive();  // parsed and discarded while not ready: never queued as a pending reply

  datalayer.battery.status.voltage_dV = 520;
  inverter.update_values();
  set_millis64(1004);
  inverter.receive();
  set_millis64(1010);
  inverter.receive();
  EXPECT_TRUE(port.tx.empty()) << "answered a request that arrived before the battery was ready";
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

TEST_F(GivEnergyLvInverter, ChargeTaperFollowsTheHighestCell) {
  datalayer.aggregate.max_charge_current_dA = 1000;  // 100 A, capped to 9000 before any taper
  datalayer.aggregate.cell_max_voltage_mV = 3400;
  EXPECT_EQ(GivEnergyLvRs485Inverter::snapshot_from_datalayer().charge_limit_cA, 9000);
  datalayer.aggregate.cell_max_voltage_mV = 3450;
  EXPECT_EQ(GivEnergyLvRs485Inverter::snapshot_from_datalayer().charge_limit_cA, 9000);
  datalayer.aggregate.cell_max_voltage_mV = 3500;
  EXPECT_EQ(GivEnergyLvRs485Inverter::snapshot_from_datalayer().charge_limit_cA, 4650);
  datalayer.aggregate.cell_max_voltage_mV = 3549;
  EXPECT_EQ(GivEnergyLvRs485Inverter::snapshot_from_datalayer().charge_limit_cA, 387);
  datalayer.aggregate.cell_max_voltage_mV = 3550;
  EXPECT_EQ(GivEnergyLvRs485Inverter::snapshot_from_datalayer().charge_limit_cA, 300);
  datalayer.aggregate.cell_max_voltage_mV = 3599;
  EXPECT_EQ(GivEnergyLvRs485Inverter::snapshot_from_datalayer().charge_limit_cA, 300);
  datalayer.aggregate.cell_max_voltage_mV = 3600;
  EXPECT_EQ(GivEnergyLvRs485Inverter::snapshot_from_datalayer().charge_limit_cA, 0);
  datalayer.aggregate.cell_max_voltage_mV = 3650;
  EXPECT_EQ(GivEnergyLvRs485Inverter::snapshot_from_datalayer().charge_limit_cA, 0);
}

TEST_F(GivEnergyLvInverter, ChargeTaperNeverRaisesTheBatteryLimit) {
  datalayer.aggregate.max_charge_current_dA = 20;  // 2.0 A: well under even the untapered limit
  datalayer.aggregate.cell_max_voltage_mV = 3500;  // deep in the taper band, but that's moot here
  const givenergy_lv::Snapshot s = GivEnergyLvRs485Inverter::snapshot_from_datalayer();
  EXPECT_EQ(s.charge_limit_cA, 200);
}

TEST_F(GivEnergyLvInverter, ChargeTaperLeavesDischargeAlone) {
  datalayer.aggregate.cell_max_voltage_mV = 3600;  // taper would zero HR26, but not HR27
  datalayer.aggregate.max_discharge_current_dA = 800;
  const givenergy_lv::Snapshot s = GivEnergyLvRs485Inverter::snapshot_from_datalayer();
  EXPECT_EQ(s.discharge_limit_cA, 8000);
}

TEST_F(GivEnergyLvInverter, FaultStopsChargeAndDischarge) {
  datalayer.aggregate.max_charge_current_dA = 500;
  datalayer.aggregate.max_discharge_current_dA = 500;
  datalayer.system.status.system_status = FAULT;
  const givenergy_lv::Snapshot s = GivEnergyLvRs485Inverter::snapshot_from_datalayer();
  EXPECT_EQ(s.charge_limit_cA, 0);
  EXPECT_EQ(s.discharge_limit_cA, 0);
  EXPECT_EQ(s.limit_cA, 0);
}

TEST_F(GivEnergyLvInverter, OverVoltageAlarmOnFault) {
  datalayer.system.status.system_status = FAULT;
  const givenergy_lv::Snapshot s = GivEnergyLvRs485Inverter::snapshot_from_datalayer();
  EXPECT_EQ(s.alarms, 0x000C);
}

TEST_F(GivEnergyLvInverter, OverVoltageAlarmAtTheCellStop) {
  datalayer.aggregate.cell_max_voltage_mV = 3599;
  EXPECT_EQ(GivEnergyLvRs485Inverter::snapshot_from_datalayer().alarms & 0x0004, 0u);
  datalayer.aggregate.cell_max_voltage_mV = 3600;
  EXPECT_EQ(GivEnergyLvRs485Inverter::snapshot_from_datalayer().alarms & 0x0004, 0x0004u);
}

TEST_F(GivEnergyLvInverter, OverVoltageAlarmAtTheUserCeiling) {
  datalayer.battery_settings.user_set_voltage_limits_active = true;
  datalayer.battery_settings.max_user_set_charge_voltage_dV = 564;
  datalayer.aggregate.voltage_dV = 563;
  EXPECT_EQ(GivEnergyLvRs485Inverter::snapshot_from_datalayer().alarms & 0x0004, 0u);
  datalayer.aggregate.voltage_dV = 564;
  EXPECT_EQ(GivEnergyLvRs485Inverter::snapshot_from_datalayer().alarms & 0x0004, 0x0004u);

  datalayer.battery_settings.user_set_voltage_limits_active = false;
  datalayer.aggregate.voltage_dV = 570;
  EXPECT_EQ(GivEnergyLvRs485Inverter::snapshot_from_datalayer().alarms & 0x0004, 0u);
}

TEST_F(GivEnergyLvInverter, UnderVoltageAlarmAtTheLowCellStop) {
  datalayer.aggregate.cell_min_voltage_mV = 2901;
  EXPECT_EQ(GivEnergyLvRs485Inverter::snapshot_from_datalayer().alarms & 0x0008, 0u);
  datalayer.aggregate.cell_min_voltage_mV = 2900;
  EXPECT_EQ(GivEnergyLvRs485Inverter::snapshot_from_datalayer().alarms & 0x0008, 0x0008u);
}

TEST_F(GivEnergyLvInverter, NoAlarmsInNormalRunning) {
  datalayer.aggregate.cell_max_voltage_mV = 3300;
  datalayer.aggregate.cell_min_voltage_mV = 3280;
  datalayer.aggregate.voltage_dV = 530;
  EXPECT_EQ(GivEnergyLvRs485Inverter::snapshot_from_datalayer().alarms, 0u);
}

TEST_F(GivEnergyLvInverter, FaultAlarmBitsAppearOnTheWire) {
  FakeSerial port;
  GivEnergyLvRs485Inverter inverter(port);
  datalayer.system.status.system_status = FAULT;
  inverter.update_values();
  port.feed(kHrPoll);
  inverter.receive();
  set_millis64(1004);
  inverter.receive();
  ASSERT_EQ(port.tx.size(), 61u);
  EXPECT_EQ(port.tx[3 + 40], 0x00);  // HR20 = 0x000C: over- and under-voltage alarms
  EXPECT_EQ(port.tx[3 + 41], 0x0C);
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
  // The Growatt battery module computes Wh = Ah x measured pack voltage, so converting back
  // uses that same measured voltage (562 dV = 56.2 V), not a fixed nominal one: 32000 Wh * 1000
  // / 562 dV = 56939 cAh.
  EXPECT_EQ(s.capacity_Ah, 569);
  EXPECT_EQ(s.full_capacity_cAh, 56939);
  EXPECT_EQ(s.design_capacity_cAh, 56939);
  EXPECT_EQ(s.remaining_cAh, 28469);
  EXPECT_EQ(s.firmware, 3020);
  EXPECT_STREQ(s.serial, "EM2026G001");

  // A round-numbers pack (33750 Wh at 54.0 V) so the expected cAh is exact.
  datalayer.aggregate.voltage_dV = 540;
  datalayer.aggregate.reported_total_capacity_Wh = 33750;
  const givenergy_lv::Snapshot s2 = GivEnergyLvRs485Inverter::snapshot_from_datalayer();
  EXPECT_EQ(s2.full_capacity_cAh, 62500);
  EXPECT_EQ(s2.capacity_Ah, 625);
}

TEST_F(GivEnergyLvInverter, FullCapacityNoLongerClampsAt16Bits) {
  datalayer.aggregate.voltage_dV = 512;
  datalayer.aggregate.reported_total_capacity_Wh = 64000;  // 625 Ah at 51.2 V: two Fogstar packs
  const givenergy_lv::Snapshot s = GivEnergyLvRs485Inverter::snapshot_from_datalayer();
  EXPECT_EQ(s.capacity_Ah, 1250);
  EXPECT_EQ(s.full_capacity_cAh, 125000u);
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
    EXPECT_GT(s.cells_mV[i], 2200) << "cell " << i;
    EXPECT_LT(s.cells_mV[i], 3700) << "cell " << i;
  }
  EXPECT_EQ(s.charge_limit_cA, 0);
  EXPECT_EQ(s.discharge_limit_cA, 0);
  EXPECT_EQ(s.cell_count, 16);
}
