#include "RoleStraps.h"

#include <Arduino.h>

using partsim::app::Role;

Role readRole(int pinA, int pinB) {
  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);
  // Let the pull-ups settle before sampling. An unconnected pin on a board that has just powered
  // up can read low for a few microseconds, which would silently pick the wrong role.
  delayMicroseconds(50);
  const int a = digitalRead(pinA) ? 1 : 0;
  const int b = digitalRead(pinB) ? 1 : 0;
  const Role r = (Role)((b << 1) | a);

#ifdef PARTSIM_PROFILE_ESP32_DISPLAY
  // In a DISPLAY build, unstrapped cannot mean master: the build has no master code in it, and
  // facesFor(Master) returns zero faces, which made PanelDriver::begin fail and print
  // "FATAL: HUB75 init failed -- check Pins.h against the wiring". That sends someone hunting a
  // wiring fault that does not exist, on their first evening with the boards.
  //
  // The build already says what this board is; the strap only chooses WHICH faces. So default to
  // the first display role and say so.
  if (r == Role::Master) return Role::Display0;
#endif
  return r;
}
