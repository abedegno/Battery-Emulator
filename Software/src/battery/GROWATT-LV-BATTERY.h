#ifndef GROWATT_LV_BATTERY_H
#define GROWATT_LV_BATTERY_H

#include "../datalayer/datalayer.h"
#include "CanBattery.h"

// Battery-facing integration for the Growatt low-voltage (48V) BMS CAN
// protocol - GBLI-series packs (e.g. GBLI6532), used stacked/parallel behind
// a Growatt hybrid inverter's PCS port.
//
// Role:
//  - This implementation behaves as the PCS (inverter) side of the link.
//  - It sends the PCS->BMS keepalive (0x301) at 1 Hz; the real hardware
//    enables charge/discharge within 2-4s of seeing this frame.
//  - It parses BMS->PCS frames (0x311-0x314, 0x315-0x318, 0x319) and
//    populates the datalayer.
//
// Byte layout and bit positions below were confirmed against a real
// GBLI6532 (2-pack system) paired with a genuine Growatt SPH6000 inverter,
// not inferred from the spec alone - see the capture-backed writeup this PR
// is based on (github.com/schmellic/growatt2solis). Notably:
//  - 0x311 byte 7 bits 5/6 are the charge/discharge-enable bits, and are
//    authoritative: a real idle->discharge->charge capture showed them
//    track live BMS state correctly. 0x319 byte 0 also carries charge
//    enable (bit 7) and discharge enable (bit 6), plus the force-charge
//    requests (bits 5 and 4), per the protocol's Table 5; its enable bits
//    are used only before the first 0x311 arrives.
//  - 0x312 byte 4 is the live parallel pack count, confirmed against a real
//    2-pack system reporting exactly 2x the single-pack current ceiling.
//  - 0x313 bytes 0-1 (pack voltage) are 0.01V, not 0.1V - confirmed by
//    plausible voltage range (53.2-53.4V) through a real charge/discharge
//    cycle; the 0.1V reading would imply ~530V.
//
// Only 0x311 refreshes CAN_battery_still_alive, and the charge/discharge
// limits it carries are forced to zero if it goes missing for more than
// LIMITS_STALE_MS while other frames keep arriving - a BMS emulating this
// protocol less faithfully than a real GBLI6532 (e.g. Seplos/Pace-based
// packs) could otherwise leave stale limits published forever with no
// missing-event ever firing (github.com/dalathegreat/Battery-Emulator/issues/3034).
// The protection causes published from 0x312 (bms_protection) expire the same
// way: they clear when 0x312 is more than LIMITS_STALE_MS old or the limits
// have gone stale. A 0x312 warning bit also raises its cause while a fresh
// 0x311 refuses that direction (see update_values()).
//
// From protocol V1.03, the per-cell frames 0x315-0x318 are optional and 0x319
// carries the highest/lowest cell voltage. In a parallel bank 0x319 counts
// every pack's cells, while 0x315-0x318 can only hold the master's 16, so the
// reported max/min is the wider of the two. Seplos BMSs (as in Fogstar's
// packs) don't send 0x315-0x318 at all in a parallel bank, only 0x319, which
// Seplos confirmed covers every pack; a standalone Seplos pack sends all 16
// cells on 0x315-0x318. setup() also sets an LFP cell
// voltage ceiling/floor (previously unset, so the generic cell over/under-
// voltage checks in safety.cpp could never trigger on this pack).
//
// Protocol reference:
//  Growatt BMS CAN-Bus protocol, low voltage V1.04. Standard (11-bit) CAN
//  IDs, 500 kbit/s, big-endian.
class GrowattLvBattery : public CanBattery {
 public:
  GrowattLvBattery() {}

  ~GrowattLvBattery() {}

  void setup(void) override;
  void handle_incoming_can_frame(CAN_frame rx_frame) override;
  void update_values() override;
  void transmit_can(unsigned long currentMillis) override;

  static constexpr const char* Name = "Growatt LV (GBLI-series) battery via CAN, 500kbit/s";

  // The parsed enable and force-charge flags, read-only
  bool charge_enabled() const { return charge_en; }
  bool discharge_enabled() const { return discharge_en; }
  bool force_charge_1_requested() const { return force_chg_1; }
  bool force_charge_2_requested() const { return force_chg_2; }

 private:
  // GBLI6532 datasheet: 5kW / 104.2A max charge/discharge per pack. A real
  // 2-pack system's own reported CCL/DCL matched 2x this value exactly, so
  // the ceiling below scales with the live pack count from 0x312 byte 4.
  static const int MAX_CURRENT_PER_PACK_dA = 1042;

  // Solis accepts 40.0-60.0V; GBLI6532 datasheet range is 48.0-57.6V.
  static const int MAX_PACK_VOLTAGE_DV = 576;
  static const int MIN_PACK_VOLTAGE_DV = 480;

  // GBLI6532 16S range is 48.0-57.6V; no generic cell ceiling/floor was set
  // before, so the cell over/under-voltage checks in safety.cpp could never
  // trigger on this LFP pack. Agreed with the driver's author in issue #3034.
  static const int MAX_CELL_VOLTAGE_MV = 3650;
  static const int MIN_CELL_VOLTAGE_MV = 2500;

  // 0x311 arrives at 1 Hz along with the rest of the frame set. Five missed
  // queries is generous slack for jitter while still reacting quickly once a
  // BMS has stopped updating the limits it advertises (issue #3034).
  // uint32_t (not unsigned long/millis()'s usual type) so the staleness check
  // in update_values() wraps at the same 2^32ms boundary as millis() itself
  // on every host, not just on a 32-bit "unsigned long" target.
  static const uint32_t LIMITS_STALE_MS = 5000;

  // Sanity bounds for 0x319's fallback max/min cell voltage (bytes 1-4):
  // reject anything outside a plausible Li-ion cell range instead of trusting
  // a garbage/zero reading.
  static const uint16_t CELL_MV_PLAUSIBLE_MIN = 1000;
  static const uint16_t CELL_MV_PLAUSIBLE_MAX = 5000;

  // 0x312 byte 0 (protection table 1): bit 7 discharge over-current, bit 6 charge over-current,
  // bit 5 discharge short circuit, bit 4 cell over-voltage, bit 3 cell under-voltage, bit 2 pack
  // over-voltage, bit 1 pack under-voltage, bit 0 soft start failed. Byte 1 (table 2): bit 7
  // discharge over-temperature, bit 6 charge over-temperature, bit 5 discharge under-temperature,
  // bit 4 charge under-temperature, bit 3 system error, bit 2 cell difference. Bytes 2 and 3 are
  // the matching warnings. Only the bits below are published (see update_values()).
  static const uint8_t PROT1_DISCHARGE_OVER_CURRENT = 0x80;
  static const uint8_t PROT2_DISCHARGE_OVER_TEMPERATURE = 0x80;
  static const uint8_t PROT2_CHARGE_OVER_TEMPERATURE = 0x40;
  static const uint8_t PROT2_DISCHARGE_UNDER_TEMPERATURE = 0x20;
  static const uint8_t PROT2_CHARGE_UNDER_TEMPERATURE = 0x10;
  // The matching warnings in bytes 2 (table 3) and 3 (table 4), at the same positions. A Seplos BMS
  // (decompiled 1.43l and 1.5d) sets these and never the protection bits above; a warning becomes a
  // cause only while the pack also refuses that direction (see update_values()). Byte 2 bit 6, the
  // charge over-current warning, has no cause to go to.
  static const uint8_t WARN1_DISCHARGE_OVER_CURRENT = 0x80;
  static const uint8_t WARN2_DISCHARGE_OVER_TEMPERATURE = 0x80;
  static const uint8_t WARN2_CHARGE_OVER_TEMPERATURE = 0x40;
  static const uint8_t WARN2_DISCHARGE_UNDER_TEMPERATURE = 0x20;
  static const uint8_t WARN2_CHARGE_UNDER_TEMPERATURE = 0x10;

  // 0x315-0x318 carry exactly 16 cell voltages (cells 1-16).
  static const uint8_t PER_CELL_FRAME_CELLS = 16;

  unsigned long previousMillis1000 = 0;

  // --- Outgoing (PCS -> BMS) ---
  // Real payload captured off a genuine Growatt SPH6000. Its semantics are
  // undocumented; static across an entire session (idle, discharge, charge,
  // cold boot) with no counter or checksum, and a real Growatt inverter
  // sending it unchanged is what gets a real GBLI6532 to enable within
  // seconds - see growatt2solis's own README/CLAUDE.md for the full
  // commissioning writeup this port is based on. NOTE: growatt2solis itself
  // shipped the protocol doc's *example* placeholder here
  // (0x11 0x22 0x33 0x44 0x55 0x66 0x77 0x88) instead of this real value
  // for its entire history until 2026-09-03, which was the actual root
  // cause of an ~594s enable-then-disable bug there - see growatt2solis
  // CLAUDE.md open question 8 if this ever needs re-deriving.
  CAN_frame BMS_301 = {.FD = false,
                       .ext_ID = false,
                       .DLC = 8,
                       .ID = 0x301,
                       .data = {0x0B, 0x16, 0x21, 0x2C, 0x37, 0x42, 0x4D, 0x58}};

  // --- Parsed battery state ---
  uint16_t cvl_dV = 0;
  uint16_t ccl_dA = 0;
  uint16_t dcl_dA = 0;
  uint16_t status_word = 0;
  bool have_311 = false;
  // uint32_t, not unsigned long: see LIMITS_STALE_MS above.
  uint32_t last_311_millis = 0;

  uint8_t prot1 = 0, prot2 = 0, warn1 = 0, warn2 = 0;
  uint8_t pack_count = 1;
  bool have_312 = false;
  // uint32_t for the same reason as last_311_millis. Ages the protection causes published from
  // prot1/prot2, against the same LIMITS_STALE_MS (0x312 comes at 1 Hz with 0x311).
  uint32_t last_312_millis = 0;

  uint16_t remaining_cAh = 0;  // 0.01Ah units
  uint16_t full_cAh = 0;       // 0.01Ah units
  bool have_314 = false;

  int16_t pack_v_cV = 0;   // 0.01V units
  int16_t current_dA = 0;  // 0.1A, + = charging
  int16_t temp_dC = 0;     // 0.1 degC
  uint8_t soc_pct = 0;
  uint8_t soh_pct = 100;
  bool have_313 = false;

  bool charge_en = false;
  bool discharge_en = false;
  bool force_chg_1 = false;  // 0x319 byte 0 bit 5: request force charge I
  bool force_chg_2 = false;  // 0x319 byte 0 bit 4: request force charge II (NULL per spec)

  // 0x319 bytes 1-4: fallback max/min cell voltage (1 mV), used only when no
  // per-cell frame (0x315-0x318) has populated a cell - see update_values().
  uint16_t max_cell_mV_319 = 0;
  uint16_t min_cell_mV_319 = 0;
};

#endif
