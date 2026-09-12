#pragma once
#include "partsim/app/Role.h"

// Reading the role off the strap pins: the hardware half of partsim/app/Role.h.
//
// It is here rather than in the application layer because it is two digitalReads and a settle
// delay -- i.e. entirely GPIO -- while what a role MEANS is a fact about the cube. Configures
// the pull-ups itself.
partsim::app::Role readRole(int pinA, int pinB);
