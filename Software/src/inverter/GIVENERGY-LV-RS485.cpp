#include "GIVENERGY-LV-RS485.h"

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
  return givenergy_lv::Snapshot{};  // filled in by Task 6
}
