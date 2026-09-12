#include "partsim/app/Role.h"

namespace partsim {
namespace app {

const char* roleName(Role r) {
  switch (r) {
    case Role::Display0: return "display0 (faces 0,2)";
    case Role::Display1: return "display1 (faces 1,5)";
    case Role::Display2: return "display2 (faces 3,4)";
    case Role::Master: return "master (physics, no panels)";
  }
  return "?";
}

bool roleDrivesPanels(Role r) { return r != Role::Master; }

RoleFaces facesFor(Role r) {
  switch (r) {
    case Role::Display0: return RoleFaces{{0, 2}, 2};
    case Role::Display1: return RoleFaces{{1, 5}, 2};
    case Role::Display2: return RoleFaces{{3, 4}, 2};
    case Role::Master: return RoleFaces{{0, 0}, 0};
  }
  return RoleFaces{{0, 0}, 0};
}

}  // namespace app
}  // namespace partsim
