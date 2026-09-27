#include "GIVENERGY-LV-RS485.h"

#include <algorithm>
#include <cstring>

#include "../communication/rs485/comm_rs485.h"
#include "../datalayer/datalayer.h"
#include "../devboard/hal/hal.h"
#include "../devboard/utils/events.h"
#include "../devboard/utils/logging.h"
#include "INVERTERS.h"

namespace {

// HR17/HR18 hash the battery's clock; any value ticking once a second will do.
constexpr uint32_t kClockSeed = 0x389D0000;

// How long len bytes take on the wire at 9600 8N1: 10 bits per byte, rounded up.
uint32_t tx_ms(size_t len) {
  return static_cast<uint32_t>((len * 10 * 1000 + 9599) / 9600);
}

constexpr char kSerial[] = "EM2026G001";
constexpr uint16_t kFirmware = 3020;     // as my battery; 3011 or higher selects HR26/HR27 on a G3
constexpr uint16_t kLimitCap_cA = 9000;  // 90 A: the new circuit has a 100 A DC MCB
// The inverter accepts cell voltages strictly between 2200 and 3700 mV and silently drops
// anything outside that (docs/05), so clamp one step inside those bounds, not onto them.
constexpr uint16_t kCellMin_mV = 2201;
constexpr uint16_t kCellMax_mV = 3699;
constexpr int kCells = 16;

uint16_t clamp_cell(uint16_t mV) {
  return std::min(std::max(mV, kCellMin_mV), kCellMax_mV);
}

uint16_t limit_cA(uint16_t dA) {
  return static_cast<uint16_t>(std::min<uint32_t>(dA * 10u, kLimitCap_cA));
}

uint16_t saturate16(uint32_t value) {
  return static_cast<uint16_t>(std::min<uint32_t>(value, 0xFFFF));
}

// 16 LFP cells at 51.2 V nominal: 0.01 Ah = Wh * 100 / 51.2.
uint32_t wh_to_cAh(uint32_t wh) {
  return static_cast<uint32_t>(static_cast<uint64_t>(wh) * 1000 / 512);
}

// The pack's cell voltages, or, if any is missing, a spread from the
// aggregate min and max so the inverter still sees plausible cells.
void fill_cells(givenergy_lv::Snapshot& s) {
  const uint16_t* cells = datalayer.battery.status.cell_voltages_mV;
  const bool complete = std::all_of(cells, cells + kCells, [](uint16_t mV) { return mV != 0; });
  uint32_t sum = 0;
  for (int i = 0; i < kCells; i++) {
    uint16_t mV = cells[i];
    if (!complete) {
      mV = i == 0 ? s.cell_max_mV : i == 1 ? s.cell_min_mV : (s.cell_max_mV + s.cell_min_mV) / 2;
    }
    s.cells_mV[i] = clamp_cell(mV);
    sum += s.cells_mV[i];
  }
  s.cell_sum_mV = saturate16(sum);
}

}  // namespace

GivEnergyLvRs485Inverter::GivEnergyLvRs485Inverter(HardwareSerial& port)
    : port_(port), snapshot_(snapshot_from_datalayer()) {}

bool GivEnergyLvRs485Inverter::setup() {
  if (!rs485_begin(Name, port_, baud_rate(), SERIAL_8N1)) {
    logging.println("Failed to initialize RS485 pins!");
    return false;
  }
  return true;
}

void GivEnergyLvRs485Inverter::update_values() {
  snapshot_ = snapshot_from_datalayer();

  // datalayer.battery.status.voltage_dV is 0 until the battery module has decoded a real pack
  // voltage; datalayer.aggregate keeps a 370 V placeholder meanwhile (see datalayer.h). Once the
  // battery has reported, stay ready even if a later reading is 0 - the FAULT path already zeroes
  // the limits, so there's nothing unsafe left to hide.
  if (datalayer.battery.status.voltage_dV != 0) {
    ready_ = true;
  }

  if (incoming_message_counter_ > 0) {
    incoming_message_counter_--;
  }
  if (incoming_message_counter_ == 0) {
    set_event(EVENT_MODBUS_INVERTER_MISSING, 0);
  } else {
    clear_event(EVENT_MODBUS_INVERTER_MISSING);
  }
}

void GivEnergyLvRs485Inverter::receive() {
  const uint32_t now = millis();
  if (reply_len_ > 0) {
    if (now - request_ms_ < kTurnaroundMs) {
      return;
    }
    send_reply(now);
  }

  while (port_.available() > 0) {
    const int byte = port_.read();
    if (byte < 0) {
      break;
    }
    rx_[rx_len_++] = static_cast<uint8_t>(byte);
    if (rx_len_ < givenergy_lv::kRequestLen) {
      continue;
    }
    if (givenergy_lv::is_request(rx_)) {
      rx_len_ = 0;
      if (is_echo(now)) {
        continue;
      }
      handle_request(now);
      return;  // the reply goes out on a later call, after the turnaround
    }
    // Not a request (noise, or the tail of a frame): slide the window on a byte.
    std::memmove(rx_, rx_ + 1, givenergy_lv::kRequestLen - 1);
    rx_len_ = givenergy_lv::kRequestLen - 1;
  }
}

bool GivEnergyLvRs485Inverter::is_echo(uint32_t now_ms) const {
  return sent_write_echo_ && static_cast<int32_t>(echo_until_ms_ - now_ms) > 0 &&
         std::memcmp(rx_, sent_, givenergy_lv::kRequestLen) == 0;
}

void GivEnergyLvRs485Inverter::handle_request(uint32_t now_ms) {
  // A valid request means the inverter is alive on the bus even if we're not ready to answer it
  // yet: don't let a slow battery module get the inverter reported missing.
  incoming_message_counter_ = RS485_HEALTHY;
  if (!inverter_detected_) {
    inverter_detected_ = true;
    set_event(EVENT_MODBUS_INVERTER_DETECTED, 1);
  }
  if (rx_[1] == 6) {
    logging.printf("GivEnergy: inverter wrote %u to register %u of device %u\n",
                   static_cast<unsigned>(rx_[4] << 8 | rx_[5]), static_cast<unsigned>(rx_[2] << 8 | rx_[3]),
                   static_cast<unsigned>(rx_[0]));
  }
  if (!ready_) {
    return;  // no real snapshot yet: stay silent rather than answer with placeholder values
  }
  snapshot_.clock_hash = kClockSeed + now_ms / 1000;
  reply_len_ = givenergy_lv::build_reply(rx_, snapshot_, reply_);
  request_ms_ = now_ms;
}

void GivEnergyLvRs485Inverter::send_reply(uint32_t now_ms) {
  // No flush(): a 61-byte reply takes ~64 ms at 9600 baud, and blocking core_loop for that
  // long on every poll starves the CAN receive and trips EVENT_TASK_OVERRUN.
  port_.write(reply_, reply_len_);
  sent_write_echo_ = reply_len_ == givenergy_lv::kRequestLen && reply_[1] == 6;
  std::memcpy(sent_, reply_, givenergy_lv::kRequestLen);
  const uint32_t sent_end_ms = now_ms + tx_ms(reply_len_);
  echo_until_ms_ = sent_end_ms + kEchoWindowMs;
  reply_len_ = 0;
}

givenergy_lv::Snapshot GivEnergyLvRs485Inverter::snapshot_from_datalayer() {
  const DATALAYER_AGGREGATE_TYPE& agg = datalayer.aggregate;
  givenergy_lv::Snapshot s{};
  std::strcpy(s.serial, kSerial);
  s.firmware = kFirmware;

  const uint32_t full_cAh = wh_to_cAh(agg.reported_total_capacity_Wh);
  s.capacity_Ah = saturate16(full_cAh / 100);
  s.status = 0x00CC;  // bits 2, 3, 6 and 7, always set on my battery in normal running
  if (agg.current_dA <= 0) {
    s.status |= 0x01;
  }
  if (agg.current_dA != 0) {
    s.status |= 0x02;
  }
  s.soc_pct = agg.reported_soc / 100;
  s.voltage_cV = agg.voltage_dV * 10;
  s.current_cA = static_cast<int16_t>(std::min(std::max(agg.current_dA * 10, -32000), 32000));
  s.temperature_C = agg.temperature_max_dC / 10;
  s.limit_cA = kLimitCap_cA;
  if (datalayer.system.status.system_status != FAULT) {
    s.charge_limit_cA = limit_cA(agg.max_charge_current_dA);
    s.discharge_limit_cA = limit_cA(agg.max_discharge_current_dA);
  }

  const int16_t t_max = agg.temperature_max_dC;
  const int16_t t_min = agg.temperature_min_dC;
  const int16_t sensors[5] = {t_max, t_min, t_min, t_max, static_cast<int16_t>((t_max + t_min) / 2)};
  std::copy(sensors, sensors + 5, s.sensor_temps_dC);

  const uint8_t cells = datalayer.battery.info.number_of_cells;
  s.cell_count = (cells >= 1 && cells <= kCells) ? cells : kCells;
  s.pack_voltage_mV = saturate16(agg.voltage_dV * 100u);
  s.current_mA = agg.current_dA * 100;
  s.full_capacity_cAh = saturate16(full_cAh);
  s.design_capacity_cAh = s.full_capacity_cAh;
  s.remaining_cAh = saturate16(wh_to_cAh(agg.reported_remaining_capacity_Wh));
  s.block2_soc_pct = s.soc_pct;
  s.block2_word28 = 0x0E10;

  s.cell_max_mV = clamp_cell(agg.cell_max_voltage_mV);
  s.cell_min_mV = clamp_cell(agg.cell_min_voltage_mV);
  fill_cells(s);
  s.temp_max_dC = t_max;
  s.temp_min_dC = t_min;
  return s;
}
