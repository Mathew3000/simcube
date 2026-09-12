#include "partsim/Parallel.h"

namespace partsim {

Parallel& serialParallel() {
  static Parallel instance;
  return instance;
}

}  // namespace partsim
