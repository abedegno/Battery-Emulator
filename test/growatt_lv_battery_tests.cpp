#include <gtest/gtest.h>

#include <Arduino.h>  // Emul: set_millis64() to control the test clock
#include <initializer_list>

#include "../Software/src/battery/GROWATT-LV-BATTERY.h"
#include "../Software/src/datalayer/datalayer.h"
#include "../Software/src/devboard/utils/types.h"

// Regression tests for the three Growatt LV gaps confirmed in issue #3034
// (github.com/dalathegreat/Battery-Emulator/issues/3034):
//  1. 0x311 freshness - only 0x311 refreshes CAN_battery_still_alive, and the
//     charge/discharge limits it carries must go to zero once it has been
//     missing for more than five 1 Hz query periods, independently of
//     whatever other frame IDs keep arriving.
//  2. setup() sets an LFP cell voltage ceiling/floor, so the generic cell
//     over/under-voltage checks in safety.cpp can actually trigger.
//  3. 0x319's own max/min cell voltage is used as a fallback for
//     cell_max_voltage_mV/cell_min_voltage_mV when no per-cell frame
//     (0x315-0x318) has populated a cell.

namespace {

// Builds an 8-byte CAN frame from a byte list, the same way issue #3034's own
// repro test did.
static CAN_frame Frame(uint32_t id, std::initializer_list<uint8_t> bytes) {
  CAN_frame f{};
  f.ID = id;
  f.DLC = 8;
  int i = 0;
  for (uint8_t b : bytes) {
    f.data.u8[i++] = b;
  }
  return f;
}

// 0x311: charge voltage (0.1V), charge current limit (0.1A), discharge
// current limit (0.1A), status word (bit 5 = discharge enable, bit 6 =
// charge enable, both in the low/second byte of the pair).
static CAN_frame Frame311(uint16_t ccl_dA, uint16_t dcl_dA, bool charge_en, bool discharge_en, uint16_t cv_dV = 576) {
  uint8_t status_low = (uint8_t)((discharge_en ? 0x20 : 0) | (charge_en ? 0x40 : 0));
  return Frame(0x311, {(uint8_t)(cv_dV >> 8), (uint8_t)(cv_dV & 0xFF), (uint8_t)(ccl_dA >> 8), (uint8_t)(ccl_dA & 0xFF),
                       (uint8_t)(dcl_dA >> 8), (uint8_t)(dcl_dA & 0xFF), 0x00, status_low});
}

// 0x313: pack voltage (0.01V), current (0.1A), temperature (0.1C), SOC, SOH.
// Values are arbitrary and only need to be present/well-formed - this frame
// stands in for "some other frame keeps arriving" in the staleness tests.
static CAN_frame Frame313() {
  return Frame(0x313, {0x14, 0xB4, 0x00, 0x00, 0x00, 0xC8, 0x50, 0x64});  // 53.00V, 0A, 20.0C, 80%, 100%
}

// Replicates check_can_component_alive()'s decrement (safety.cpp): only
// decrements while nonzero. check_can_component_alive() itself is static to
// safety.cpp and not callable from here.
static void DecrementAliveCounterLikeSafetyDoes() {
  if (datalayer.battery.status.CAN_battery_still_alive > 0) {
    datalayer.battery.status.CAN_battery_still_alive--;
  }
}

}  // namespace

// The #3034 repro: one 0x311 with 100A limits and both directions enabled,
// then only 0x313 once a second for 600 simulated seconds, with the
// aliveness counter decremented every second the way check_can_component_alive
// does. Before the fix, the counter never reached zero (no missing-event) and
// the 100A limits were published forever; after the fix, the counter is
// exhausted at 60s and the limits are zeroed once 0x311 is more than 5s old.
TEST(GrowattLvStaleness, LimitsAndAlivenessExpireWhen0x311StopsButOthersContinue) {
  datalayer = DataLayer();
  set_millis64(0);
  GrowattLvBattery b;

  b.handle_incoming_can_frame(Frame311(1000, 1000, true, true));
  b.update_values();
  ASSERT_EQ(datalayer.battery.status.max_charge_current_dA, 1000);
  ASSERT_EQ(datalayer.battery.status.max_discharge_current_dA, 1000);
  ASSERT_EQ(datalayer.battery.status.CAN_battery_still_alive, CAN_STILL_ALIVE);

  bool checked_5s_zero = false;
  for (int s = 1; s <= 600; s++) {
    set_millis64((uint64_t)s * 1000ULL);
    DecrementAliveCounterLikeSafetyDoes();
    b.handle_incoming_can_frame(Frame313());
    b.update_values();

    if (s == 6) {  // more than 5s (five missed 1Hz queries) since the only 0x311
      EXPECT_EQ(datalayer.battery.status.max_charge_current_dA, 0)
          << "limits must be zeroed once 0x311 is more than 5s stale";
      EXPECT_EQ(datalayer.battery.status.max_discharge_current_dA, 0);
      checked_5s_zero = true;
    }
  }
  EXPECT_TRUE(checked_5s_zero);

  EXPECT_EQ(datalayer.battery.status.CAN_battery_still_alive, 0)
      << "only 0x311 may refresh the aliveness counter - 0x313 alone must let it "
         "run out, which is what makes safety.cpp raise the battery-missing event";
  EXPECT_EQ(datalayer.battery.status.max_charge_current_dA, 0);
  EXPECT_EQ(datalayer.battery.status.max_discharge_current_dA, 0);
}

// A healthy pack sending a fresh 0x311 every second must keep both the
// limits and the aliveness counter exactly as they were - the fix must not
// introduce any staleness where there wasn't one before.
TEST(GrowattLvStaleness, FreshFrameEverySecondKeepsLimitsAndAlivenessFull) {
  datalayer = DataLayer();
  set_millis64(0);
  GrowattLvBattery b;

  b.handle_incoming_can_frame(Frame311(1000, 1000, true, true));
  b.update_values();
  ASSERT_EQ(datalayer.battery.status.max_charge_current_dA, 1000);

  for (int s = 1; s <= 600; s++) {
    set_millis64((uint64_t)s * 1000ULL);
    DecrementAliveCounterLikeSafetyDoes();
    b.handle_incoming_can_frame(Frame311(1000, 1000, true, true));
    b.update_values();
  }

  EXPECT_EQ(datalayer.battery.status.CAN_battery_still_alive, CAN_STILL_ALIVE);
  EXPECT_EQ(datalayer.battery.status.max_charge_current_dA, 1000);
  EXPECT_EQ(datalayer.battery.status.max_discharge_current_dA, 1000);
}

// Once 0x311 comes back after a stale period, the limits must return
// immediately - staleness is not a one-way latch.
TEST(GrowattLvStaleness, LimitsReturnOnceAFreshFrameArrivesAfterBeingStale) {
  datalayer = DataLayer();
  set_millis64(0);
  GrowattLvBattery b;

  b.handle_incoming_can_frame(Frame311(1000, 1000, true, true));
  b.update_values();
  ASSERT_EQ(datalayer.battery.status.max_charge_current_dA, 1000);

  set_millis64(6000);  // >5s since the only 0x311, no new frame
  b.update_values();
  ASSERT_EQ(datalayer.battery.status.max_charge_current_dA, 0) << "limits must be zero while 0x311 is stale";
  ASSERT_EQ(datalayer.battery.status.max_discharge_current_dA, 0);

  set_millis64(6500);
  b.handle_incoming_can_frame(Frame311(1000, 1000, true, true));
  b.update_values();
  EXPECT_EQ(datalayer.battery.status.max_charge_current_dA, 1000) << "a fresh 0x311 must restore the limits right away";
  EXPECT_EQ(datalayer.battery.status.max_discharge_current_dA, 1000);
}

// The staleness check is unsigned-subtraction based, so it must give the
// right answer across a millis() wrap (2^32 ms), not just look like elapsed
// time went backwards or forwards by four billion milliseconds.
TEST(GrowattLvStaleness, StalenessSurvivesAMillisWrap) {
  datalayer = DataLayer();
  GrowattLvBattery b;

  const uint64_t base = 0x100000000ULL - 500;  // 500ms before the wrap
  set_millis64(base);
  b.handle_incoming_can_frame(Frame311(1000, 1000, true, true));
  b.update_values();
  ASSERT_EQ(datalayer.battery.status.max_charge_current_dA, 1000);

  set_millis64(base + 1500);  // 1s after the wrap; ~1.5s really elapsed - still fresh
  b.update_values();
  EXPECT_EQ(datalayer.battery.status.max_charge_current_dA, 1000)
      << "a millis() wrap must not be mistaken for the limits going stale";

  set_millis64(base + 5600);  // ~5.6s really elapsed - now stale
  b.update_values();
  EXPECT_EQ(datalayer.battery.status.max_charge_current_dA, 0)
      << "elapsed time must still be measured correctly across the wrap";
  EXPECT_EQ(datalayer.battery.status.max_discharge_current_dA, 0);

  set_millis64(0);
}

// setup() must set an LFP cell voltage ceiling/floor - previously unset, so
// the cell over/under-voltage checks in safety.cpp could never trigger on
// this pack. Also pins down that nothing else setup() does (notably
// contactor-closing permission) changed alongside it.
TEST(GrowattLvBatterySetup, SetsLfpCellVoltageLimits) {
  datalayer = DataLayer();
  GrowattLvBattery b;

  b.setup();

  EXPECT_EQ(datalayer.battery.info.max_cell_voltage_mV, 3650);
  EXPECT_EQ(datalayer.battery.info.min_cell_voltage_mV, 2500);
  EXPECT_FALSE(datalayer.system.status.battery_allows_contactor_closing)
      << "setup() must still start with contactor closing disallowed, unchanged by this fix";
  EXPECT_EQ(datalayer.battery.status.max_charge_current_dA, 0)
      << "before any CAN frame arrives, limits must still be 0 as before";
  EXPECT_EQ(datalayer.battery.status.max_discharge_current_dA, 0);
}

// From protocol V1.03, 0x315-0x318 are optional and 0x319 carries the same
// highest/lowest cell voltage as a fallback (bytes 1-2 max, bytes 3-4 min).
// With no per-cell frame ever received, that fallback must populate
// cell_max_voltage_mV/cell_min_voltage_mV.
TEST(GrowattLvCellVoltageFallback, Frame0x319AloneSetsCellMaxAndMin) {
  datalayer = DataLayer();
  GrowattLvBattery b;

  // byte0 = 0 (unused here), bytes1-2 = 3412 mV max, bytes3-4 = 3398 mV min,
  // byte5/6 = max/min cell number, byte7 = protect pack ID (unused here).
  b.handle_incoming_can_frame(Frame(0x319, {0x00, 0x0D, 0x54, 0x0D, 0x46, 0x01, 0x10, 0x00}));
  b.update_values();

  EXPECT_EQ(datalayer.battery.status.cell_max_voltage_mV, 3412);
  EXPECT_EQ(datalayer.battery.status.cell_min_voltage_mV, 3398);
}

// Feeds the same 16 per-cell values for each test below: 3400 mV, except
// cell 5 at 3420 mV (the highest) and cell 12 at 3390 mV (the lowest).
static void feed_per_cell_frames(GrowattLvBattery& b) {
  b.handle_incoming_can_frame(Frame(0x315, {0x0D, 0x48, 0x0D, 0x48, 0x0D, 0x48, 0x0D, 0x48}));  // cells 1-4: 3400mV
  b.handle_incoming_can_frame(Frame(0x316, {0x0D, 0x5C, 0x0D, 0x48, 0x0D, 0x48, 0x0D, 0x48}));  // cell 5: 3420mV (max)
  b.handle_incoming_can_frame(Frame(0x317, {0x0D, 0x48, 0x0D, 0x48, 0x0D, 0x48, 0x0D, 0x3E}));  // cell 12: 3390mV (min)
  b.handle_incoming_can_frame(Frame(0x318, {0x0D, 0x48, 0x0D, 0x48, 0x0D, 0x48, 0x0D, 0x48}));  // cells 13-16: 3400mV
}

// In a parallel bank, 0x315-0x318 can only carry the master pack's 16 cells,
// while 0x319 counts every pack's cells. A slave pack's highest and lowest
// cell must not be hidden by the master's per-cell frames.
TEST(GrowattLvCellVoltageFallback, BankWide0x319WidensPerCellMaxAndMin) {
  datalayer = DataLayer();
  GrowattLvBattery b;

  // 0x319: bank max 3610 mV, bank min 3300 mV - both outside the master's cells.
  b.handle_incoming_can_frame(Frame(0x319, {0x00, 0x0E, 0x1A, 0x0C, 0xE4, 0x15, 0x20, 0x00}));
  feed_per_cell_frames(b);
  b.update_values();

  EXPECT_EQ(datalayer.battery.status.cell_max_voltage_mV, 3610);
  EXPECT_EQ(datalayer.battery.status.cell_min_voltage_mV, 3300);
}

// A single pack's 0x319 agrees with (or sits inside) its own per-cell data,
// so the per-cell max/min stand.
TEST(GrowattLvCellVoltageFallback, PerCellMaxAndMinStandWhen0x319IsInsideThem) {
  datalayer = DataLayer();
  GrowattLvBattery b;

  // 0x319: 3410 mV max, 3395 mV min - inside the per-cell range.
  b.handle_incoming_can_frame(Frame(0x319, {0x00, 0x0D, 0x52, 0x0D, 0x43, 0x05, 0x0C, 0x00}));
  feed_per_cell_frames(b);
  b.update_values();

  EXPECT_EQ(datalayer.battery.status.cell_max_voltage_mV, 3420);
  EXPECT_EQ(datalayer.battery.status.cell_min_voltage_mV, 3390);
}

// An implausible 0x319 max/min (here 0xFFFF / 0) is ignored, and the
// per-cell data is used on its own.
TEST(GrowattLvCellVoltageFallback, Implausible0x319IsIgnoredAlongsidePerCellFrames) {
  datalayer = DataLayer();
  GrowattLvBattery b;

  b.handle_incoming_can_frame(Frame(0x319, {0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00}));
  feed_per_cell_frames(b);
  b.update_values();

  EXPECT_EQ(datalayer.battery.status.cell_max_voltage_mV, 3420);
  EXPECT_EQ(datalayer.battery.status.cell_min_voltage_mV, 3390);
}

// number_of_cells was never set, so the web UI cell view, MQTT and ESP-NOW
// showed no cells even though 0x315-0x318 were arriving.
TEST(GrowattLvCellCount, PerCellFrameSetsSixteenCells) {
  datalayer = DataLayer();
  GrowattLvBattery b;
  b.setup();
  EXPECT_EQ(datalayer.battery.info.number_of_cells, 0);

  b.handle_incoming_can_frame(Frame(0x315, {0x0D, 0x48, 0x0D, 0x48, 0x0D, 0x48, 0x0D, 0x48}));

  EXPECT_EQ(datalayer.battery.info.number_of_cells, 16);
}

// A pack that sends only 0x319 (max/min, no per-cell frames) has no cell
// array to show, so the count stays 0.
TEST(GrowattLvCellCount, Frame0x319AloneLeavesCellCountAtZero) {
  datalayer = DataLayer();
  GrowattLvBattery b;
  b.setup();

  b.handle_incoming_can_frame(Frame(0x319, {0x00, 0x0D, 0x54, 0x0D, 0x46, 0x01, 0x10, 0x00}));
  b.update_values();

  EXPECT_EQ(datalayer.battery.info.number_of_cells, 0);
}

// ---- Protection causes (0x312) ----
//
// 0x312 bytes 0 and 1 are the protection bytes, bytes 2 and 3 the matching warnings (protocol
// V1.04, tables 1 to 4). The temperature protections in each direction and the discharge
// over-current protection are published as bms_protection, for inverter protocols that pass the
// cause of a stop on. Bit positions are from the document only: no capture shows them set.

static CAN_frame Frame312(uint8_t prot1, uint8_t prot2, uint8_t warn1 = 0, uint8_t warn2 = 0) {
  return Frame(0x312, {prot1, prot2, warn1, warn2, 0x01, 0x00, 0x00, 0x10});
}

static bool NoCause() {
  const DATALAYER_BMS_PROTECTION_TYPE& p = datalayer.battery.status.bms_protection;
  return !p.charge_over_temperature && !p.charge_under_temperature && !p.discharge_over_temperature &&
         !p.discharge_under_temperature && !p.discharge_over_current;
}

TEST(GrowattLvProtectionCauses, NoneBeforeAnyFrame) {
  datalayer = DataLayer();
  GrowattLvBattery b;
  b.update_values();
  EXPECT_TRUE(NoCause());
}

TEST(GrowattLvProtectionCauses, Prot1Bit7IsDischargeOverCurrent) {
  datalayer = DataLayer();
  GrowattLvBattery b;
  b.handle_incoming_can_frame(Frame312(0x80, 0x00));
  b.update_values();
  EXPECT_TRUE(datalayer.battery.status.bms_protection.discharge_over_current);
  EXPECT_FALSE(datalayer.battery.status.bms_protection.discharge_over_temperature);
}

TEST(GrowattLvProtectionCauses, Prot2Bit7IsDischargeOverTemperature) {
  datalayer = DataLayer();
  GrowattLvBattery b;
  b.handle_incoming_can_frame(Frame312(0x00, 0x80));
  b.update_values();
  const DATALAYER_BMS_PROTECTION_TYPE& p = datalayer.battery.status.bms_protection;
  EXPECT_TRUE(p.discharge_over_temperature);
  EXPECT_FALSE(p.charge_over_temperature || p.discharge_under_temperature || p.charge_under_temperature ||
               p.discharge_over_current);
}

TEST(GrowattLvProtectionCauses, Prot2Bit6IsChargeOverTemperature) {
  datalayer = DataLayer();
  GrowattLvBattery b;
  b.handle_incoming_can_frame(Frame312(0x00, 0x40));
  b.update_values();
  const DATALAYER_BMS_PROTECTION_TYPE& p = datalayer.battery.status.bms_protection;
  EXPECT_TRUE(p.charge_over_temperature);
  EXPECT_FALSE(p.discharge_over_temperature || p.discharge_under_temperature || p.charge_under_temperature ||
               p.discharge_over_current);
}

TEST(GrowattLvProtectionCauses, Prot2Bit5IsDischargeUnderTemperature) {
  datalayer = DataLayer();
  GrowattLvBattery b;
  b.handle_incoming_can_frame(Frame312(0x00, 0x20));
  b.update_values();
  const DATALAYER_BMS_PROTECTION_TYPE& p = datalayer.battery.status.bms_protection;
  EXPECT_TRUE(p.discharge_under_temperature);
  EXPECT_FALSE(p.charge_over_temperature || p.discharge_over_temperature || p.charge_under_temperature ||
               p.discharge_over_current);
}

TEST(GrowattLvProtectionCauses, Prot2Bit4IsChargeUnderTemperature) {
  datalayer = DataLayer();
  GrowattLvBattery b;
  b.handle_incoming_can_frame(Frame312(0x00, 0x10));
  b.update_values();
  const DATALAYER_BMS_PROTECTION_TYPE& p = datalayer.battery.status.bms_protection;
  EXPECT_TRUE(p.charge_under_temperature);
  EXPECT_FALSE(p.charge_over_temperature || p.discharge_over_temperature || p.discharge_under_temperature ||
               p.discharge_over_current);
}

// The other protection bits (charge over-current, short circuit, cell and pack voltages, soft
// start, system error, cell difference) and every warning bit leave the causes clear.
TEST(GrowattLvProtectionCauses, OtherProtectionsAndAllWarningsAreNotCauses) {
  datalayer = DataLayer();
  GrowattLvBattery b;
  b.handle_incoming_can_frame(Frame312(0x7F, 0x0F, 0xFF, 0xFF));
  b.update_values();
  EXPECT_TRUE(NoCause());
  EXPECT_FALSE(datalayer.system.status.battery_allows_contactor_closing);
}

TEST(GrowattLvProtectionCauses, CausesClearWhenTheProtectionEnds) {
  datalayer = DataLayer();
  GrowattLvBattery b;
  b.handle_incoming_can_frame(Frame312(0x80, 0xF0));
  b.update_values();
  const DATALAYER_BMS_PROTECTION_TYPE& p = datalayer.battery.status.bms_protection;
  EXPECT_TRUE(p.discharge_over_current && p.discharge_over_temperature && p.charge_over_temperature &&
              p.discharge_under_temperature && p.charge_under_temperature);
  b.handle_incoming_can_frame(Frame312(0x00, 0x00));
  b.update_values();
  EXPECT_TRUE(NoCause());
}

// ---- Freshness of the protection causes ----
//
// The causes expire like the limits: a 0x312 more than 5 s old no longer says why the pack is
// stopped, so they go back to clear ("no cause known"), even while 0x311 keeps the link alive.

TEST(GrowattLvProtectionCauses, CausesClearWhen0x312GoesStaleWhile0x311KeepsArriving) {
  datalayer = DataLayer();
  set_millis64(0);
  GrowattLvBattery b;

  b.handle_incoming_can_frame(Frame311(1000, 1000, true, true));
  b.handle_incoming_can_frame(Frame312(0x80, 0x40));
  b.update_values();
  ASSERT_TRUE(datalayer.battery.status.bms_protection.discharge_over_current);
  ASSERT_TRUE(datalayer.battery.status.bms_protection.charge_over_temperature);

  for (int s = 1; s <= 5; s++) {  // 5 s old is still fresh
    set_millis64((uint64_t)s * 1000ULL);
    b.handle_incoming_can_frame(Frame311(1000, 1000, true, true));
    b.update_values();
  }
  EXPECT_TRUE(datalayer.battery.status.bms_protection.discharge_over_current);
  EXPECT_TRUE(datalayer.battery.status.bms_protection.charge_over_temperature);

  set_millis64(6000);  // more than 5 s since the only 0x312
  b.handle_incoming_can_frame(Frame311(1000, 1000, true, true));
  b.update_values();
  EXPECT_TRUE(NoCause()) << "causes must clear once 0x312 is more than 5s stale";
  EXPECT_EQ(datalayer.battery.status.max_charge_current_dA, 1000) << "a fresh 0x311 keeps its limits";

  set_millis64(0);
}

TEST(GrowattLvProtectionCauses, CausesReturnOnceAFresh0x312Arrives) {
  datalayer = DataLayer();
  set_millis64(0);
  GrowattLvBattery b;

  b.handle_incoming_can_frame(Frame311(1000, 1000, true, true));
  b.handle_incoming_can_frame(Frame312(0x00, 0x10));
  b.update_values();
  ASSERT_TRUE(datalayer.battery.status.bms_protection.charge_under_temperature);

  set_millis64(6000);
  b.handle_incoming_can_frame(Frame311(1000, 1000, true, true));
  b.update_values();
  ASSERT_TRUE(NoCause());

  set_millis64(6500);
  b.handle_incoming_can_frame(Frame312(0x00, 0x10));
  b.update_values();
  EXPECT_TRUE(datalayer.battery.status.bms_protection.charge_under_temperature)
      << "a fresh 0x312 must restore the causes right away";

  set_millis64(0);
}

// When every frame stops, the limits go stale and the causes clear with them.
TEST(GrowattLvProtectionCauses, CausesClearWhenTheWholeLinkGoesQuiet) {
  datalayer = DataLayer();
  set_millis64(0);
  GrowattLvBattery b;

  b.handle_incoming_can_frame(Frame311(1000, 1000, true, true));
  b.handle_incoming_can_frame(Frame312(0x80, 0xF0));
  b.update_values();
  ASSERT_TRUE(datalayer.battery.status.bms_protection.discharge_over_temperature);

  set_millis64(6000);
  b.update_values();
  EXPECT_EQ(datalayer.battery.status.max_charge_current_dA, 0);
  EXPECT_TRUE(NoCause());

  set_millis64(0);
}

// A 0x312 that keeps arriving can't hold the causes up once 0x311 (the frame the reader treats as
// the link's heartbeat) has gone stale: the limits are zero for a lost link, not for those causes.
TEST(GrowattLvProtectionCauses, CausesClearWhenTheLimitsGoStaleEvenIf0x312KeepsArriving) {
  datalayer = DataLayer();
  set_millis64(0);
  GrowattLvBattery b;

  b.handle_incoming_can_frame(Frame311(1000, 1000, true, true));
  b.handle_incoming_can_frame(Frame312(0x80, 0x00));
  b.update_values();
  ASSERT_TRUE(datalayer.battery.status.bms_protection.discharge_over_current);

  for (int s = 1; s <= 6; s++) {
    set_millis64((uint64_t)s * 1000ULL);
    b.handle_incoming_can_frame(Frame312(0x80, 0x00));
    b.update_values();
  }
  EXPECT_EQ(datalayer.battery.status.max_discharge_current_dA, 0);
  EXPECT_TRUE(NoCause());

  set_millis64(0);
}

// setup() starts with no cause known, whatever an earlier run left in the datalayer.
TEST(GrowattLvProtectionCauses, SetupClearsTheCauses) {
  datalayer = DataLayer();
  datalayer.battery.status.bms_protection.charge_over_temperature = true;
  datalayer.battery.status.bms_protection.discharge_over_current = true;
  GrowattLvBattery b;
  b.setup();
  EXPECT_TRUE(NoCause());
}
// 0x319 byte 0, per Growatt's "BMS CAN-Bus protocol, low voltage" V1.04, Table 5:
// bit 7 charge enable, bit 6 discharge enable, bit 5 request force charge I,
// bit 4 request force charge II (NULL per the spec), bits 1-0 battery type.
// Bits 3 and 2 are not used.
static CAN_frame Frame319(uint8_t byte0) {
  return Frame(0x319, {byte0, 0x0D, 0x54, 0x0D, 0x46, 0x01, 0x10, 0x00});  // 3412 / 3398 mV
}

TEST(GrowattLv0x319Flags, EachBitIsReadFromItsTable5Position) {
  struct Case {
    uint8_t byte0;
    bool charge_en, discharge_en, force_chg_1, force_chg_2;
  };
  const Case cases[] = {
      {0x80, true, false, false, false},   // bit 7: charge enable
      {0x40, false, true, false, false},   // bit 6: discharge enable
      {0x20, false, false, true, false},   // bit 5: request force charge I
      {0x10, false, false, false, true},   // bit 4: request force charge II
      {0xC0, true, true, false, false},    // both directions enabled, mid SOC
      {0x60, false, true, true, false},    // nearly empty: discharge allowed, force charge requested
      {0x0C, false, false, false, false},  // bits 3 and 2 are unused
      {0x03, false, false, false, false},  // battery type only
  };
  for (const Case& c : cases) {
    datalayer = DataLayer();
    GrowattLvBattery b;
    b.handle_incoming_can_frame(Frame319(c.byte0));
    EXPECT_EQ(b.charge_enabled(), c.charge_en) << "byte0 0x" << std::hex << (int)c.byte0;
    EXPECT_EQ(b.discharge_enabled(), c.discharge_en) << "byte0 0x" << std::hex << (int)c.byte0;
    EXPECT_EQ(b.force_charge_1_requested(), c.force_chg_1) << "byte0 0x" << std::hex << (int)c.byte0;
    EXPECT_EQ(b.force_charge_2_requested(), c.force_chg_2) << "byte0 0x" << std::hex << (int)c.byte0;
  }
}

// Once 0x311 has arrived its status bits own the enable state: a later 0x319
// with different enable bits does not change it, but its force-charge request
// is still read.
TEST(GrowattLv0x319Flags, EnableBitsAreIgnoredOnce0x311HasArrived) {
  datalayer = DataLayer();
  set_millis64(0);
  GrowattLvBattery b;

  b.handle_incoming_can_frame(Frame311(1000, 1000, true, false));
  b.handle_incoming_can_frame(Frame319(0x60));  // discharge enable + force charge I, no charge enable
  b.update_values();

  EXPECT_TRUE(b.charge_enabled());
  EXPECT_FALSE(b.discharge_enabled());
  EXPECT_TRUE(b.force_charge_1_requested());
  EXPECT_EQ(datalayer.battery.status.max_charge_current_dA, 1000);
  EXPECT_EQ(datalayer.battery.status.max_discharge_current_dA, 0);
}

// Before 0x311 there are no limits yet, so the 0x319 enable bits cannot
// open either direction on their own.
TEST(GrowattLv0x319Flags, EnableBitsAloneDoNotPublishLimits) {
  datalayer = DataLayer();
  set_millis64(0);
  GrowattLvBattery b;

  b.handle_incoming_can_frame(Frame319(0xC0));
  b.update_values();

  EXPECT_TRUE(b.charge_enabled());
  EXPECT_TRUE(b.discharge_enabled());
  EXPECT_EQ(datalayer.battery.status.max_charge_current_dA, 0);
  EXPECT_EQ(datalayer.battery.status.max_discharge_current_dA, 0);
  EXPECT_FALSE(datalayer.system.status.battery_allows_contactor_closing);
}
