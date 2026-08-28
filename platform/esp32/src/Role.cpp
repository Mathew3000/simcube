#include "Role.h"

#include <Arduino.h>

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

Role readRole(int pinA, int pinB) {
  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);
  // Let the pull-ups settle before sampling. An unconnected pin on a board that has just powered
  // up can read low for a few microseconds, which would silently pick the wrong role.
  delayMicroseconds(50);
  const int a = digitalRead(pinA) ? 1 : 0;
  const int b = digitalRead(pinB) ? 1 : 0;
  return (Role)((b << 1) | a);
}

RoleFaces facesFor(Role r) {
  switch (r) {
    case Role::Display0: return RoleFaces{{0, 2}, 2};
    case Role::Display1: return RoleFaces{{1, 5}, 2};
    case Role::Display2: return RoleFaces{{3, 4}, 2};
    case Role::Master: return RoleFaces{{0, 0}, 0};
  }
  return RoleFaces{{0, 0}, 0};
}
