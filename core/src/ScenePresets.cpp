#include "partsim/Scene.h"

namespace partsim {
namespace {

// Emitter positions are normalised (0..1 in the box) so a preset does not need to know the
// volume's size, and the same table works for a cube or a single panel.
const SceneDesc kScenes[] = {
    {"water tank", 3000, 0, 0, 0, {}, 14.0f},

    // Fire from a patch on the floor, burning continuously so it never needs relighting.
    {"campfire", 0, 0, 0, 1,
     {{0.5f, 0.06f, 0.5f, 4.5f, 0.55f}},
     16.0f},

    // Sand alone, so the heap and its angle are the whole picture.
    {"sand pile", 0, 1100, 0, 0, {}, 12.0f},

    // The interesting mixture: sand sinks through the water, which falls out of the density
    // constraint rather than being scripted.
    {"water and sand", 2000, 500, 0, 0, {}, 16.0f},

    // Water over a burner. There is no water/fire coupling -- no steam, no extinguishing -- so
    // this is two systems sharing a volume, which still reads well.
    {"kettle", 1800, 0, 0, 1,
     {{0.5f, 0.05f, 0.5f, 5.0f, 0.45f}},
     14.0f},

    {"neon tank", 2600, 0, 1, 0, {}, 12.0f},

    // Two burners on opposite sides, which makes tilt-driven leaning obvious.
    {"twin flames", 0, 0, 1, 2,
     {{0.28f, 0.06f, 0.5f, 3.2f, 0.5f}, {0.72f, 0.06f, 0.5f, 3.2f, 0.5f}},
     14.0f},
};

constexpr int kCount = (int)(sizeof(kScenes) / sizeof(kScenes[0]));

}  // namespace

int sceneCount() { return kCount; }

const SceneDesc& sceneAt(int i) {
  return kScenes[i < 0 ? 0 : (i >= kCount ? kCount - 1 : i)];
}

}  // namespace partsim
