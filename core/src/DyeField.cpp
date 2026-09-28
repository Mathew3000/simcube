#include "partsim/DyeField.h"

namespace partsim {
namespace {
int index(int x, int y, int z) { return (z*DyeField::kSize+y)*DyeField::kSize+x; }
uint16_t pack(float c) { return (uint16_t)(pclamp(c,0.0f,1.0f)*65535.0f+0.5f); }
Vec3 inside(Vec3 p) {
  return {pclamp(p.x,-0.499f,0.499f), pclamp(p.y,-0.499f,0.499f),
          pclamp(p.z,-0.499f,0.499f)};
}
}

void DyeField::reset(DyeSample colour) {
  const uint16_t r = pack(colour.red), b = pack(colour.blue);
  for (auto& buffer : grid_) for (auto& c : buffer) { c[0] = r; c[1] = b; }
  for (auto& r : diffusionRemainder_) r[0] = r[1] = 0;
  for (auto& v : vortices_) v.strength = 0.0f;
  front_ = nextVortex_ = 0;
  jet_ = 0.0f;
}

DyeSample DyeField::cell(int x, int y, int z) const {
  const auto& c = grid_[front_][index(iclamp(x,0,kSize-1), iclamp(y,0,kSize-1),
                                   iclamp(z,0,kSize-1))];
  return {c[0]*(1.0f/65535.0f), c[1]*(1.0f/65535.0f)};
}

DyeSample DyeField::sample(Vec3 p) const { return sampleBuffer(front_,p); }

DyeSample DyeField::sampleBuffer(int buffer, Vec3 p) const {
  const float fx = pclamp((p.x+0.5f)*kSize-0.5f,0.0f,(float)(kSize-1));
  const float fy = pclamp((p.y+0.5f)*kSize-0.5f,0.0f,(float)(kSize-1));
  const float fz = pclamp((p.z+0.5f)*kSize-0.5f,0.0f,(float)(kSize-1));
  const int x = imin((int)fx,kSize-2), y = imin((int)fy,kSize-2), z = imin((int)fz,kSize-2);
  const float tx = fx-(float)x, ty = fy-(float)y, tz = fz-(float)z;
  DyeSample result{0,0};
  for (int dz = 0; dz < 2; ++dz) for (int dy = 0; dy < 2; ++dy)
    for (int dx = 0; dx < 2; ++dx) {
      const float w = (dx ? tx : 1.0f-tx)*(dy ? ty : 1.0f-ty)*(dz ? tz : 1.0f-tz);
      const auto& c = grid_[buffer][index(x+dx,y+dy,z+dz)];
      result.red += w*(float)c[0]; result.blue += w*(float)c[1];
    }
  result.red *= 1.0f/65535.0f; result.blue *= 1.0f/65535.0f;
  return result;
}

void DyeField::inject(Vec3 position, DyeSample colour, float amount, float radius) {
  if (!(amount > 0.0f)) return;
  radius = pclamp(radius,0.07f,0.3f);
  const float invR2 = 1.0f/(radius*radius);
  const float strength = pmin(1.0f, amount/(radius*radius*radius*1.5f));
  const int x0 = iclamp((int)((position.x-radius+0.5f)*kSize),0,kSize-1);
  const int y0 = iclamp((int)((position.y-radius+0.5f)*kSize),0,kSize-1);
  const int z0 = iclamp((int)((position.z-radius+0.5f)*kSize),0,kSize-1);
  const int x1 = iclamp((int)((position.x+radius+0.5f)*kSize),0,kSize-1);
  const int y1 = iclamp((int)((position.y+radius+0.5f)*kSize),0,kSize-1);
  const int z1 = iclamp((int)((position.z+radius+0.5f)*kSize),0,kSize-1);
  for (int z = z0; z <= z1; ++z) for (int y = y0; y <= y1; ++y)
    for (int x = x0; x <= x1; ++x) {
      const Vec3 p{((float)x+0.5f)/kSize-0.5f, ((float)y+0.5f)/kSize-0.5f,
                   ((float)z+0.5f)/kSize-0.5f};
      const float t = pmax(0.0f,1.0f-length2(p-position)*invR2);
      const float a = strength*t*t;
      auto& c = grid_[front_][index(x,y,z)];
      if (a > 0) diffusionRemainder_[index(x,y,z)][0] = diffusionRemainder_[index(x,y,z)][1] = 0;
      c[0] = pack((float)c[0]*(1.0f/65535.0f)*(1.0f-a)+colour.red*a);
      c[1] = pack((float)c[1]*(1.0f/65535.0f)*(1.0f-a)+colour.blue*a);
    }
}

void DyeField::stir(Vec3 position, Vec3 down, float strength) {
  down = normalize(down);
  Vec3 tangent = normalize(cross(down,pabs(down.z) < 0.8f ? Vec3{0,0,1} : Vec3{1,0,0}));
  const Vec3 axis = normalize(cross(down,tangent));
  for (int sign = -1; sign <= 1; sign += 2) {
    Vortex& v = vortices_[nextVortex_];
    nextVortex_ = (nextVortex_+1)%6;
    v.position = inside(position+down*0.12f+tangent*((float)sign*0.10f));
    v.axis = axis*(float)sign;
    v.strength = pclamp(strength*2.0f,0.0f,10.0f);
  }
  jetPosition_ = position; jetDown_ = down; jet_ = pmin(0.4f,strength*0.06f);
}

Vec3 DyeField::displacement(Vec3 p, float dt, Vec3 down, Vec3 bulkVelocity) const {
  Vec3 velocity = bulkVelocity*0.12f + cross(bulkVelocity,p)*0.5f;
  // A narrow descending inlet jet feeds the counter-rotating vortex pair.
  const Vec3 r = p-jetPosition_;
  const float along = dot(r,jetDown_);
  const Vec3 radial = r-jetDown_*along;
  const float jetWeight = pmax(0.0f,1.0f-length2(radial)*70.0f);
  if (along > -0.08f && along < 0.7f)
    velocity += jetDown_*(jet_*jetWeight*jetWeight);
  for (const auto& v : vortices_) {
    if (v.strength < 0.01f) continue;
    const Vec3 d = p-v.position;
    const float falloff = pmax(0.0f,1.0f-length2(d)*9.0f);
    velocity += cross(v.axis,d)*(v.strength*falloff*falloff);
  }
  // Gentle dye settling makes an injected plume continue after the pour.
  // This helper is evaluated at cell centres in both advection passes.
  const DyeSample dye = cell((int)((p.x+0.5f)*kSize),(int)((p.y+0.5f)*kSize),
                            (int)((p.z+0.5f)*kSize));
  velocity += down*((dye.red+dye.blue)*0.008f);
  Vec3 displacement = velocity*dt;
  const float maxD = pmax(pabs(displacement.x),pmax(pabs(displacement.y),pabs(displacement.z)));
  if (maxD > 0.75f/kSize) displacement *= (0.75f/kSize)/maxD;
  return displacement;
}

void DyeField::step(float dt, const SurfaceWater& water, Vec3 bulkVelocity, float agitation) {
  if (!(dt > 0.0f)) return;
  dt = pmin(dt,0.05f);
  const Vec3 down = -water.up();
  const int back = (front_+1)%3, corrected = (front_+2)%3;
  for (int z = 0; z < kSize; ++z) for (int y = 0; y < kSize; ++y)
    for (int x = 0; x < kSize; ++x) {
      Vec3 p{((float)x+0.5f)/kSize-0.5f, ((float)y+0.5f)/kSize-0.5f,
             ((float)z+0.5f)/kSize-0.5f};
      // Dry cells extend the nearest liquid colour. They are never rendered;
      // this avoids black voids when a moving surface wets a new grid cell.
      Vec3 source = p;
      const float depth = water.depth(p);
      if (depth < 0.0f) {
        for (int j = 0; j < 3; ++j)
          source = inside(source + down*pmax(0.0f,-water.depth(source)+0.01f));
      } else {
        const Vec3 displacement = this->displacement(p,dt,down,bulkVelocity);
        source = inside(p-displacement);
        if (water.depth(source) < 0.0f)
          source = inside(source+down*(-water.depth(source)+0.005f));
      }
      const DyeSample d = sample(source);
      grid_[back][index(x,y,z)][0] = pack(d.red);
      grid_[back][index(x,y,z)][1] = pack(d.blue);
    }
  // Limited MacCormack correction removes much of first-order advection's
  // numerical blur. Three fixed buffers keep both passes independent of cell
  // traversal order; local old-field bounds prevent overshoot and negative ink.
  for (int z = 0; z < kSize; ++z) for (int y = 0; y < kSize; ++y)
    for (int x = 0; x < kSize; ++x) {
      const int idx = index(x,y,z);
      const Vec3 p{((float)x+0.5f)/kSize-0.5f,((float)y+0.5f)/kSize-0.5f,
                   ((float)z+0.5f)/kSize-0.5f};
      if (water.depth(p) < 0) {
        grid_[corrected][idx][0] = grid_[back][idx][0];
        grid_[corrected][idx][1] = grid_[back][idx][1];
        continue;
      }
      const Vec3 delta = displacement(p,dt,down,bulkVelocity);
      Vec3 source = inside(p-delta);
      if (water.depth(source) < 0)
        source = inside(source+down*(-water.depth(source)+0.005f));
      const DyeSample reverse = sampleBuffer(back,inside(p+delta));
      const float reverseChannels[] = {reverse.red,reverse.blue};
      const int sx = iclamp((int)((source.x+0.5f)*kSize-0.5f),0,kSize-2);
      const int sy = iclamp((int)((source.y+0.5f)*kSize-0.5f),0,kSize-2);
      const int sz = iclamp((int)((source.z+0.5f)*kSize-0.5f),0,kSize-2);
      for (int c = 0; c < 2; ++c) {
        uint16_t lo = 65535, hi = 0;
        for (int dz = 0; dz < 2; ++dz) for (int dy = 0; dy < 2; ++dy)
          for (int dx = 0; dx < 2; ++dx) {
            const uint16_t v = grid_[front_][index(sx+dx,sy+dy,sz+dz)][c];
            if (v < lo) lo = v;
            if (v > hi) hi = v;
          }
        const float value = (float)grid_[back][idx][c]+0.5f*((float)grid_[front_][idx][c]-reverseChannels[c]*65535.0f);
        grid_[corrected][idx][c] = (uint16_t)(pclamp(value,(float)lo,(float)hi)+0.5f);
      }
    }
  front_ = corrected;
  diffuse(dt,water,agitation);
  for (auto& v : vortices_) {
    v.strength *= 1.0f-dt*0.18f;
    v.position = inside(v.position+down*(dt*0.035f));
  }
  jet_ *= 1.0f-dt*0.3f;
}

void DyeField::diffuse(float dt, const SurfaceWater& water, float agitation) {
  // Six-neighbour diffusion, with no flux through the walls or waterline.
  // Motion raises diffusivity; the nonzero floor keeps still water mixing.
  // lambda <= 1/6 makes this explicit stencil a convex combination.
  const float lambda = pmin(1.0f/6.0f,dt*(0.2f+4.6f*pclamp(agitation,0.0f,1.0f)));
  const int back = (front_+1)%3;
  const Vec3 up = water.up();
  const float strideDepth[] = {-up.x/kSize,-up.y/kSize,-up.z/kSize};
  const int strides[] = {1,kSize,kSize*kSize};
  uint32_t sum[2] = {};
  uint16_t lo[2] = {65535,65535}, hi[2] = {};
  int count = 0;
  for (int z = 0; z < kSize; ++z) for (int y = 0; y < kSize; ++y)
    for (int x = 0; x < kSize; ++x) {
      const int idx = index(x,y,z);
      const Vec3 p{((float)x+0.5f)/kSize-0.5f,((float)y+0.5f)/kSize-0.5f,
                   ((float)z+0.5f)/kSize-0.5f};
      const float depth = water.depth(p);
      const auto& old = grid_[front_][idx];
      auto& next = grid_[back][idx];
      if (depth < 0) {
        next[0] = old[0]; next[1] = old[1];
        diffusionRemainder_[idx][0] = diffusionRemainder_[idx][1] = 0;
        continue;
      }
      int laplacian[2] = {};
      const int coords[] = {x,y,z};
      for (int axis = 0; axis < 3; ++axis) for (int sign = -1; sign <= 1; sign += 2) {
        if (coords[axis]+sign < 0 || coords[axis]+sign >= kSize ||
            depth+(float)sign*strideDepth[axis] < 0) continue;
        const auto& neighbour = grid_[front_][idx+sign*strides[axis]];
        for (int c = 0; c < 2; ++c) laplacian[c] += (int)neighbour[c]-(int)old[c];
      }
      ++count;
      for (int c = 0; c < 2; ++c) {
        const float value = pclamp((float)old[c]+lambda*(float)laplacian[c]+
                            (float)diffusionRemainder_[idx][c]*(1.0f/128.0f),0.0f,65535.0f);
        next[c] = (uint16_t)(value+0.5f);
        const float error = (value-(float)next[c])*128.0f;
        diffusionRemainder_[idx][c] = (int8_t)(error+(error >= 0 ? 0.5f : -0.5f));
        sum[c] += next[c];
        if (next[c] < lo[c]) lo[c] = next[c];
        if (next[c] > hi[c]) hi[c] = next[c];
      }
    }
  front_ = back;
  if (!count) return;
  // Finish below 0.1% concentration contrast. This removes the final integer
  // precision floor, per channel, without averaging visible unmixed plumes.
  for (int c = 0; c < 2; ++c) if ((int)hi[c]-(int)lo[c] <= 64) {
    const uint16_t mean = (uint16_t)((sum[c]+(uint32_t)count/2u)/(uint32_t)count);
    for (int i = 0; i < kCells; ++i) {
      grid_[front_][i][c] = mean;
      diffusionRemainder_[i][c] = 0;
    }
  }
}
}  // namespace partsim
