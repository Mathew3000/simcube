#pragma once
#include <cstdint>

// What this board is, read from strapping pins at boot.
//
// Every board runs the same image and the role is a fact about the wiring -- the same principle as
// the mount table and the IMU axis map.
//
// Four runtime roles fit exactly in two pins, and they fit only because STANDALONE IS NOT A ROLE:
// a single board driving all six faces is the `cube` build environment with its own capacity
// profile, decided at compile time. Trying to make it a fifth runtime code needed three pins for
// no benefit.
//
// The straps are pulled up and a jumper pulls to ground, so an UNSTRAPPED board reads 0b11 -- and
// that is deliberately the master. Forget the jumpers on every board and you get four masters:
// nothing lights up and every console says "master", which is immediately diagnosable. Had
// unstrapped meant display0 instead, four boards would fight over faces 0 and 2 while four faces
// stayed dark -- the same amount of broken, harder to read.
enum class Role : uint8_t {
  Display0 = 0,  // straps 0b00: faces 0 and 2  (-Z, -X)
  Display1 = 1,  // straps 0b01: faces 1 and 5  (+Z, +Y)
  Display2 = 2,  // straps 0b10: faces 3 and 4  (+X, -Y)
  Master = 3,    // unstrapped: physics, IMU, radio, no panels
};

// Face pairs per display role. Adjacent, never opposite, so each board's two HUB75 ribbons stay
// short instead of one crossing the cube's interior. See docs/CUBE-PCB.md section 2.1.
struct RoleFaces {
  int face[2];
  int count;  // 0 for the master, which drives nothing
};

const char* roleName(Role r);
bool roleDrivesPanels(Role r);
// Reads the two strap pins. Configures the pull-ups itself.
Role readRole(int pinA, int pinB);
RoleFaces facesFor(Role r);
