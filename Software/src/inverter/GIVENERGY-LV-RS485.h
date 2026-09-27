#ifndef GIVENERGY_LV_RS485_H
#define GIVENERGY_LV_RS485_H

#include "GIVENERGY-LV-FRAMES.h"
#include "Rs485InverterProtocol.h"

// Answers a GivEnergy LV inverter (tested against a G3 5 kW hybrid) as its
// battery: device 1 is the pack, devices 2 to 5 are empty slots.
class GivEnergyLvRs485Inverter : public Rs485InverterProtocol {
 public:
  explicit GivEnergyLvRs485Inverter(HardwareSerial& port = Serial2);
  const char* name() override { return Name; }
  bool setup() override;
  void receive() override;
  void update_values() override;
  static constexpr const char* Name = "GivEnergy LV battery via RS485";

  // Wire values for the current datalayer. Public for the unit tests.
  static givenergy_lv::Snapshot snapshot_from_datalayer();

 private:
  int baud_rate() override { return 9600; }
  bool is_echo(uint32_t now_ms) const;
  void handle_request(uint32_t now_ms);
  void send_reply(uint32_t now_ms);

  // Modbus RTU wants 3.5 character times (3.6 ms at 9600) before a reply.
  static constexpr uint32_t kTurnaroundMs = 4;
  // A frame identical to the write echo just sent, this soon after, is the transceiver hearing itself.
  static constexpr uint32_t kEchoWindowMs = 50;
  // As PYLON-LV-RS485: 12 updates of 1 s (about 12 s) without a request raise the missing event.
  static constexpr uint8_t RS485_HEALTHY = 12;

  HardwareSerial& port_;
  givenergy_lv::Snapshot snapshot_;
  uint8_t rx_[givenergy_lv::kRequestLen] = {};
  size_t rx_len_ = 0;
  uint8_t reply_[givenergy_lv::kMaxReplyLen] = {};
  size_t reply_len_ = 0;
  uint32_t request_ms_ = 0;
  uint8_t sent_[givenergy_lv::kRequestLen] = {};
  bool sent_write_echo_ = false;
  // Deadline past which a frame identical to our last write echo is a genuine retry, not the
  // transceiver hearing its own transmission: the end of that transmission plus kEchoWindowMs.
  uint32_t echo_until_ms_ = 0;
  uint8_t incoming_message_counter_ = RS485_HEALTHY;
  bool inverter_detected_ = false;
  // Set once update_values() has seen a real pack voltage from the battery module. Until then,
  // receive() parses and discards requests without replying, so we never tell the inverter a
  // DataLayer default (370 V) is the pack voltage. Sticky: once true, stays true.
  bool ready_ = false;
};

#endif
