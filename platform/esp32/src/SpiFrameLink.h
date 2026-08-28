#pragma once
#include "FrameLink.h"
#include "partsim/SimFrame.h"

// SPI carrier for SimFrame payloads. Master is the host, display nodes are devices.
//
// UNTESTED. Written against the ESP-IDF SPI master/slave drivers and it compiles, but no two boards
// have ever talked. Treat every timing and chunking choice here as a first guess.
//
// Broadcast, and its one hazard: the master asserts all three chip selects together so a single
// transaction reaches every node -- the payload is identical for all of them because any particle
// can light any face. The hazard is that an SPI slave drives MISO whenever its CS is low, so three
// slaves selected at once contend on that net every frame. The frame path never reads MISO, but the
// contention happens regardless of whether anyone listens.
//
// Fix is on the board, not here: a 100-330 ohm series resistor on each display node's MISO makes the
// contention current-limited and harmless. See docs/CUBE-PCB.md REQ-SPI-6. If that resistor is
// absent, set kBroadcast = false below and the master will send three separate transactions instead
// -- correct, and 75% of the 20 MHz budget rather than 25%.
class SpiMasterLink final : public FrameLink {
 public:
  static constexpr bool kBroadcast = true;

  bool begin(int sck, int mosi, int miso, const int cs[3], int nodes, int hz);
  bool send(const uint8_t* bytes, int len) override;
  int poll(uint8_t*, int) override { return 0; }  // the master never receives frames
  uint32_t sent() const override { return sent_; }
  uint32_t errors() const override { return errors_; }
  const char* name() const override { return kBroadcast ? "spi-host (broadcast)" : "spi-host"; }

 private:
  bool ready_ = false;
  int cs_[3] = {-1, -1, -1};
  int nodes_ = 0;
  uint32_t sent_ = 0;
  uint32_t errors_ = 0;
};

class SpiDisplayLink final : public FrameLink {
 public:
  bool begin(int sck, int mosi, int miso, int cs);
  bool send(const uint8_t*, int) override { return false; }  // a display node never sends frames
  int poll(uint8_t* buf, int cap) override;
  const uint8_t* pollDirect(int& len) override;
  uint32_t received() const override { return received_; }
  uint32_t errors() const override { return errors_; }
  const char* name() const override { return "spi-device"; }

 private:
  bool ready_ = false;
  bool queued_ = false;
  uint32_t received_ = 0;
  uint32_t errors_ = 0;
};
