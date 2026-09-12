#pragma once
#include <cstdint>

#include "partsim/Spill.h"

// The chain between cubes: ESP-NOW carrying spill packets from one beaker's open top to the next.
//
// Broadcast, not paired. A chain is configured by the user as an order, and the alternative --
// each cube storing its neighbour's MAC -- means pairing every cube with the next before the
// object does anything, and re-pairing whenever one is replaced. Broadcast plus a link id in the
// packet costs one byte and makes a cube's position in the chain a setting rather than a ceremony.
//
// Receive happens in a WiFi task callback, so it may NOT touch the simulation: everything it does
// is copy the bytes into a ring and return. Draining is the caller's job, on its own thread.
class EspNowLink {
 public:
  bool begin();
  bool ready() const { return ready_; }

  // Fire and forget. ESP-NOW is unacknowledged here by choice -- a spill packet is worth less than
  // the latency of retrying it, and the receiver's cumulative-count arithmetic makes up whatever
  // is lost (partsim::SpillReceiver). Returns false only if the radio refused the send outright.
  bool send(const uint8_t* data, int len);

  // Copies out one packet, oldest first. Returns its length, or 0 if the ring is empty.
  int poll(uint8_t* out, int cap);

  uint32_t sent() const { return sent_; }
  uint32_t received() const { return received_; }
  // Packets the ISR-side callback had nowhere to put. A chain that looks lossy should check this
  // before blaming the radio: it means the consumer is not draining, not that the air is bad.
  uint32_t overruns() const { return overruns_; }

 private:
  static void onRecv(const uint8_t* mac, const uint8_t* data, int len);

  static constexpr int kSlots = 8;
  struct Slot {
    uint8_t data[partsim::kSpillMaxPayload];
    volatile int len = 0;
  };
  static Slot s_ring[kSlots];
  static volatile uint32_t s_head, s_tail, s_overruns, s_received;

  bool ready_ = false;
  uint32_t sent_ = 0;
  uint32_t received_ = 0;
  uint32_t overruns_ = 0;
};
