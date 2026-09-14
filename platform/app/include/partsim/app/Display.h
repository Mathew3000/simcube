#pragma once
#include <cstdint>

#include "partsim/ChainMap.h"
#include "partsim/Geometry.h"
#include "partsim/Renderer.h"

namespace partsim {
namespace app {

// Where resolved pixels go.
//
// This is the seam that makes a non-ESP solver board possible at all: the simulation master
// drives no panels, so a board that only solves needs an implementation of THIS -- which is
// nothing -- rather than a HUB75 DMA driver, which is one to two weeks of work per MCU family.
// See docs/W3-HANDOFF.md section 2.
//
// The interface is deliberately the one PanelDriver already had. ChainMap and the mount table
// stay in it because they are a property of how panels are physically arranged, not of the
// HUB75 protocol -- a different display technology in the same cube has the same mount problem.
class Display {
 public:
  virtual ~Display() = default;

  // Resolves each face out of the renderer's accumulation buffers and pushes a whole frame at
  // once. Partial frames read as tearing, which on a cube looks like the fluid breaking apart.
  virtual void present(const Renderer& r, const Geometry& g) = 0;

  // The orientation calibration pattern; see PanelDriver.h for what it draws and why.
  virtual void testPattern(const Geometry& g) = 0;

  virtual void setBrightness(uint8_t b) = 0;

  // False when there is no hardware behind this: a master node, the QEMU build, the host. The
  // benchmark uses it to decide whether a blit timing would mean anything.
  virtual bool ready() const = 0;

  virtual ChainMap& chain() = 0;
  const ChainMap& chain() const { return const_cast<Display*>(this)->chain(); }

  // Whether every face maps to horizontal runs, i.e. the fast blit path is available for all of
  // them. A property of the mount table rather than of the code, which is why it is reported at
  // boot rather than assumed.
  virtual bool allRunsHorizontal(const Geometry& g) const = 0;

  // The walk (MINI.md 6.3): lights exactly one physical pixel by its raw index in the display's
  // own serial addressing and blanks everything else; `index < 0` turns everything off. It
  // deliberately bypasses ChainMap and the renderer, because a fluid -- or even the orientation
  // pattern -- cannot tell a driver bug (wrong strip order, wrong serpentine) from a mount-table
  // bug, and this is the one tool that can.
  //
  // NOT pure virtual: a technology with no such per-pixel serial address (HUB75 addresses a whole
  // row at once) has nothing to implement, so the default is a no-op that reports false, and
  // PanelDriver/NullDisplay need no change at all.
  virtual bool lightOne(int index) { (void)index; return false; }
  // How many indices lightOne accepts, 0 where it is not implemented.
  virtual int lightCount() const { return 0; }
};

// A display that is not there. The master role, the QEMU environment and the host build all need
// the application to run with nothing attached, and each of them previously did it with its own
// #if around the call site.
class NullDisplay final : public Display {
 public:
  void present(const Renderer&, const Geometry&) override {}
  void testPattern(const Geometry&) override {}
  void setBrightness(uint8_t) override {}
  bool ready() const override { return false; }
  ChainMap& chain() override { return chain_; }
  bool allRunsHorizontal(const Geometry&) const override { return true; }

 private:
  ChainMap chain_;
};

}  // namespace app
}  // namespace partsim
