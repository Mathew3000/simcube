#pragma once
#include <cstddef>
#include <cstdint>

// The frame transport, behind an interface.
//
// SimFrame already produces and consumes bytes without knowing how they travel, and three
// different carriers need to move them:
//
//   SPI, master as host, 20 MHz DMA      real hardware
//   a socket between two emulated boards QEMU (Milestone 3.5)
//   a JS array between module instances  the browser preview
//
// Without this seam each of those would grow its own copy of the protocol handling, which is
// exactly backwards for a format whose entire purpose is that both ends agree. So the protocol
// sits above the interface and the carrier below it.
class FrameLink {
 public:
  virtual ~FrameLink() = default;

  // Master side. Returns false if the link is not ready or the payload does not fit.
  virtual bool send(const uint8_t* bytes, int len) = 0;

  // Display side. Copies at most `cap` bytes of the next available frame into `buf` and returns
  // its length, or 0 if nothing has arrived. Never blocks: a display node that finds no frame
  // holds its previous one, which is the specified behaviour rather than a fallback.
  virtual int poll(uint8_t* buf, int cap) = 0;

  // Diagnostics, reported by the console. A link that silently drops is the failure this whole
  // design is trying to make visible.
  virtual uint32_t sent() const { return 0; }
  virtual uint32_t received() const { return 0; }
  virtual uint32_t errors() const { return 0; }
  virtual const char* name() const = 0;
};

// A link that carries nothing, so a single-board build and the QEMU environment can compile and
// run the same code paths without a peer. Returns 0 from poll() forever, which a display node
// treats as "hold the last frame" -- the same as a real link that has gone quiet.
class NullFrameLink final : public FrameLink {
 public:
  bool send(const uint8_t*, int) override { ++sent_; return true; }
  int poll(uint8_t*, int) override { return 0; }
  uint32_t sent() const override { return sent_; }
  const char* name() const override { return "null"; }

 private:
  uint32_t sent_ = 0;
};
