#pragma once
#include "partsim/FieldGrid.h"

namespace partsim {

// Scene presets as plain C++ tables, not parsed files: zero parsing code, zero RAM (they land
// in flash rodata on the ESP32), and identical on all three targets.
struct SceneDesc {
  const char* name;
  int waterCount;   // clamped to the volume's capacity at init
  int sandCount;
  int paletteIndex;
  int emitterCount;
  Emitter emitters[kMaxEmitters];
  float dwellSeconds;  // how long auto-cycle rests here
};

int sceneCount();
const SceneDesc& sceneAt(int i);

}  // namespace partsim
