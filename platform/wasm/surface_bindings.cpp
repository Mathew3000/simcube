#include "partsim/SurfaceWater.h"
#include <cstdint>

namespace {
using namespace partsim;
SurfaceWater water[2];
Geometry geometry;
Vec3 down[2] = {{0,-1,0}, {0,-1,0}};
uint8_t pixels[2][6][64*64*4];
float basis[6][12];
float flow[2] = {};
int resolution = 32;
bool valid(int i) { return i >= 0 && i < 2; }
}

// A small independent WASM artifact. Linking partsim_core only pulls the
// SurfaceWater/Geometry objects, never the particle solver's static pools.
extern "C" {
int sw_init(int res) {
  if (res != 8 && res != 32 && res != 64) return 0;
  geometry = Geometry::cube(res, 32.0f/(float)res);
  if (geometry.count() != 6) return 0;
  resolution = res;
  for (int i = 0; i < 2; ++i) {
    water[i].reset(i == 0 ? 0.65f : 0.15f);
    down[i] = {0,-1,0};
    flow[i] = 0;
  }
  for (int i = 0; i < 6; ++i) {
    const Panel& p = geometry.at(i);
    const float b[] = {p.origin.x,p.origin.y,p.origin.z,p.u.x,p.u.y,p.u.z,
                       p.v.x,p.v.y,p.v.z,p.n.x,p.n.y,p.n.z};
    for (int j = 0; j < 12; ++j) basis[i][j] = b[j];
  }
  return 1;
}
int sw_resolution() { return resolution; }
uintptr_t sw_basis(int face) { return face >= 0 && face < 6 ? (uintptr_t)basis[face] : 0; }
uintptr_t sw_pixels(int i, int face) {
  return valid(i) && face >= 0 && face < 6 ? (uintptr_t)pixels[i][face] : 0;
}
void sw_orient(int i, float x, float y, float z, float w) {
  if (valid(i)) down[i] = rotateByConjugate({x,y,z,w}, {0,-1,0});
}
void sw_impulse(int i, float x, float y, float z) {
  if (valid(i)) water[i].impulse({x,y,z});
}
void sw_fill(int i, float v) { if (valid(i)) water[i].setVolume(v); }
float sw_volume(int i) { return valid(i) ? water[i].volume() : 0; }
float sw_flow(int i) { return valid(i) ? flow[i] : 0; }
int sw_state_bytes() { return (int)sizeof(SurfaceWater); }
void sw_step(float dt, int pouring) {
  if (!(dt > 0)) return;
  dt = pmin(dt, 0.1f);
  for (int i = 0; i < 2; ++i) {
    water[i].step(dt, down[i]);
    flow[i] = 0;
  }
  // Fixed identity ring, completely independent of positions in the preview.
  // Sequential transfers are intentional; this is an in-process demo, not
  // the radio protocol. Full receivers block outgoing transfer at the source.
  if (pouring) for (int i = 0; i < 2; ++i)
    flow[i] = water[i].pourTo(water[1-i], dt)/dt;
}
void sw_render() {
  for (int i = 0; i < 2; ++i) for (int face = 0; face < geometry.count(); ++face)
    water[i].renderPanel(geometry.at(face), 32.0f, pixels[i][face]);
}
}
