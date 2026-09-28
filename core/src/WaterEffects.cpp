#include "partsim/WaterEffects.h"

namespace partsim {
namespace {
Vec3 limit(Vec3 p, float bounds) {
  return {pclamp(p.x,-bounds,bounds),pclamp(p.y,-bounds,bounds),pclamp(p.z,-bounds,bounds)};
}
Vec3 capped(Vec3 p, float magnitude) {
  const float n = length2(p);
  return n > magnitude*magnitude ? p*(magnitude*frsqrt(n)) : p;
}
uint8_t toByte(float v) { return (uint8_t)pclamp(v,0.0f,255.0f); }
}

void WaterEffects::reset(float volume, DyeSample colour) {
  surface_.reset(volume); dye_.reset(colour);
  for (auto& d : drops_) d.volume = 0;
  lastDown_ = {0,-1,0}; flow_ = {0,0,0};
  agitation_ = 0;
  dyeTime_ = splashCooldown_ = stirCooldown_ = 0;
  spawnSerial_ = 0; seeded_ = false;
}

int WaterEffects::dropletCount() const {
  int n = 0;
  for (const auto& d : drops_) if (d.volume > 0) ++n;
  return n;
}
float WaterEffects::airborneVolume() const {
  float v = 0;
  for (const auto& d : drops_) v += d.volume;
  return v;
}
float WaterEffects::volume() const { return surface_.volume()+airborneVolume(); }

Vec3 WaterEffects::inlet() const {
  // Fixed local inlet, offset so a plume has a visible path across the vessel.
  Vec3 p{0.19f,0.48f,0.12f};
  for (int i = 0; i < 4; ++i)
    p = limit(p-surface_.up()*(-surface_.depth(p)+0.035f),0.46f);
  return p;
}

void WaterEffects::addInk(DyeSample colour) {
  const Vec3 p = inlet();
  dye_.inject(p,colour,0.018f,0.12f);
  dye_.stir(p,-surface_.up(),4.5f);
}

void WaterEffects::splash(Vec3 direction, float strength) {
  if (surface_.volume() < 0.025f || surface_.volume() > 0.96f) return;
  // Intersect the water plane with all 12 box edges. These are real wet
  // corners/walls, so the splashes visibly detach from the waterline there.
  Vec3 points[12]; int n = 0;
  for (int a = 0; a < 8; ++a) for (int axis = 0; axis < 3; ++axis) {
    const int b = a^(1<<axis);
    if (b < a) continue;
    const Vec3 p{(a&1)?0.5f:-0.5f,(a&2)?0.5f:-0.5f,(a&4)?0.5f:-0.5f};
    const Vec3 q{(b&1)?0.5f:-0.5f,(b&2)?0.5f:-0.5f,(b&4)?0.5f:-0.5f};
    const float dp = surface_.depth(p), dq = surface_.depth(q);
    if ((dp >= 0) == (dq >= 0) || pabs(dp-dq) < 1e-6f) continue;
    points[n++] = p+(q-p)*(dp/(dp-dq));
  }
  if (!n) return;
  const int wanted = iclamp((int)(strength*3.0f),2,12);
  direction = capped(direction,1.0f);
  float removed = 0;
  int spawned = 0;
  for (auto& d : drops_) {
    if (spawned >= wanted) break;
    if (d.volume > 0) continue;
    // Cycle edge intersections for broad splashes, with a bias towards motion.
    const int first = (int)(spawnSerial_++ % (unsigned)n);
    const int other = (first+1)%n;
    const Vec3 edge = dot(direction,points[first]) > dot(direction,points[other]) ? points[first] : points[other];
    d.p = limit(edge,0.465f)+surface_.up()*0.035f;
    d.p = limit(d.p,0.49f);
    if (surface_.depth(d.p) > -0.008f) continue;
    d.dye = dye_.sample(edge);
    const float variety = (float)(spawnSerial_%5u)*0.05f;
    d.v = surface_.up()*(0.55f+strength*0.14f+variety)+direction*0.3f;
    d.volume = 0.00025f; d.age = 0;
    removed += d.volume; ++spawned;
  }
  if (removed > 0) surface_.setVolume(surface_.volume()-removed);
}

void WaterEffects::impulse(Vec3 accelerationG) {
  const Vec3 a = capped(accelerationG,4.0f);
  surface_.impulse(a);
  flow_ = capped(flow_+a*0.6f,2.0f);
  const float strength = length(a);
  agitation_ = pmin(1.0f,agitation_+strength*0.35f);
  if (strength > 0.5f) {
    splash(a,strength);
    dye_.stir(inlet(),-surface_.up(),pmin(5.0f,strength));
    splashCooldown_ = 0.12f;
  }
}

void WaterEffects::step(float dt, Vec3 down) {
  if (!(dt > 0)) return;
  dt = pmin(dt,0.1f);
  down = length2(down) > 1e-8f ? normalize(down) : lastDown_;
  const Vec3 change = down-lastDown_;
  const float speed = seeded_ ? length(change)/dt : 0.0f;
  // Accumulate angular travel, not a single-frame speed peak. Even gentle
  // sloshing stirs dye; the envelope persists through 30 Hz dye updates.
  agitation_ = pmin(1.0f,agitation_+speed*dt*1.5f);
  agitation_ *= pmax(0.0f,1.0f-dt*0.5f);
  if (seeded_) flow_ = capped(flow_-change*1.8f,2.0f);
  seeded_ = true; lastDown_ = down;
  splashCooldown_ -= dt; stirCooldown_ -= dt;
  // Actual orientation changes trigger droplets: not restricted to Shake.
  if (speed > 1.8f && splashCooldown_ <= 0) {
    const float strength = pclamp(speed*0.3f,0.7f,3.0f);
    splash(-change,strength);
    splashCooldown_ = 0.10f;
  }
  surface_.step(dt,down);
  const int substeps = (int)(dt*120.0f)+1;
  const float h = dt/(float)substeps;
  float returned = 0;
  for (auto& d : drops_) {
    if (d.volume <= 0) continue;
    for (int j = 0; j < substeps; ++j) {
      d.age += h; d.v += down*(2.6f*h); d.p += d.v*h;
      // The LED cube is a bounded container. Droplets rebound from walls;
      // outgoing volume is handled by the aggregate rim-flow path below.
      float* coords[] = {&d.p.x,&d.p.y,&d.p.z};
      float* speeds[] = {&d.v.x,&d.v.y,&d.v.z};
      for (int k = 0; k < 3; ++k) {
        if (*coords[k] > 0.485f) { *coords[k] = 0.485f; *speeds[k] = -pabs(*speeds[k])*0.45f; }
        if (*coords[k] < -0.485f) { *coords[k] = -0.485f; *speeds[k] = pabs(*speeds[k])*0.45f; }
      }
      if ((d.age > 0.06f && surface_.depth(d.p) > 0.0f) || d.age > 2.5f) {
        returned += d.volume;
        dye_.inject(d.p,d.dye,d.volume,0.075f);
        d.volume = 0; break;
      }
    }
  }
  if (returned > 0) surface_.setVolume(surface_.volume()+returned);
  flow_ *= pmax(0.0f,1.0f-dt*0.9f);
  dyeTime_ += dt;
  // 30 Hz dye; 120 Hz surface/drops. No dependence on panel resolution.
  while (dyeTime_ >= 1.0f/30.0f) {
    if (surface_.volume() > 1e-5f) dye_.step(1.0f/30.0f,surface_,flow_,agitation_);
    dyeTime_ -= 1.0f/30.0f;
  }
}

float WaterEffects::pourTo(WaterEffects& receiver, float dt) {
  if (&receiver == this || !(dt > 0)) return 0;
  const float amount = pmin(pmin(surface_.outflowRate()*pmin(dt,0.1f),surface_.volume()),
                            pmax(0.0f,1.0f-receiver.volume()));
  if (!(amount > 0)) return 0;
  const Vec3 up = surface_.up();
  const Vec3 lip{up.x >= 0 ? -0.46f : 0.46f,0.48f,up.z >= 0 ? -0.46f : 0.46f};
  const DyeSample colour = dye_.sample(lip);
  surface_.setVolume(surface_.volume()-amount);
  const bool empty = receiver.surface_.volume() < 1e-5f;
  receiver.surface_.setVolume(receiver.surface_.volume()+amount);
  if (empty) receiver.dye_.reset(colour);
  const Vec3 p = receiver.inlet();
  receiver.dye_.inject(p,colour,amount,0.13f);
  if (receiver.stirCooldown_ <= 0) {
    receiver.dye_.stir(p,-receiver.surface_.up(),4.0f);
    receiver.stirCooldown_ = 0.45f;
  }
  return amount;
}

void WaterEffects::renderPanel(const Panel& panel, float cubeSide, uint8_t* rgba) const {
  if (!rgba || !(cubeSide > 0)) return;
  constexpr int size = DyeField::kSize;
  uint8_t projection[size*size][3];
  const float invSide = 1.0f/cubeSide;
  const Vec3 corner = panel.origin*invSide;
  const Vec3 cu = panel.u*((float)panel.w*invSide/size);
  const Vec3 cv = panel.v*((float)panel.h*invSide/size);
  for (int y = 0; y < size; ++y) for (int x = 0; x < size; ++x) {
    Vec3 p = corner+cu*((float)x+0.5f)+cv*((float)y+0.5f)+panel.n*(0.5f/size);
    float red = 0.0f, blue = 0.0f, green = 0.0f;
    for (int z = 0; z < size; ++z, p += panel.n*(1.0f/size)) {
      if (surface_.depth(p) < 0) continue;
      const DyeSample d = dye_.cell((int)((p.x+0.5f)*size),(int)((p.y+0.5f)*size),
                                    (int)((p.z+0.5f)*size));
      // Maximum-intensity projection exposes internal wisps on opaque LEDs.
      // Front-to-back alpha compositing hid them behind the uniformly dyed
      // foreground. All channels still come from the same evolving 3D field.
      red = pmax(red,d.red); blue = pmax(blue,d.blue); green = pmax(green,d.green);
    }
    const float sum = pmax(red,pmax(green,blue));
    const float opacity = sum/(0.12f+sum);
    const float inv = sum > 1e-6f ? 1.0f/sum : 0.0f;
    const float r = red*235.0f*inv;
    const float g = green*235.0f*inv;
    const float b = blue*235.0f*inv;
    projection[y*size+x][0] = toByte(r*opacity+42.0f*(1.0f-opacity));
    projection[y*size+x][1] = toByte(g*opacity+57.0f*(1.0f-opacity));
    projection[y*size+x][2] = toByte(b*opacity+68.0f*(1.0f-opacity));
  }
  const Vec3 du = panel.u*invSide, dv = panel.v*invSide;
  const Vec3 start = corner+(du+dv)*0.5f;
  const float aa = pmax(0.001f,(pabs(dot(surface_.up(),du))+pabs(dot(surface_.up(),dv)))*0.65f);
  const float invAA = 1.0f/aa, invLine = 1.0f/(0.012f+aa);
  const float sx = (float)size/(float)panel.w, sy = (float)size/(float)panel.h;
  for (int y = 0; y < (int)panel.h; ++y) {
    const float fy = pclamp(((float)y+0.5f)*sy-0.5f,0.0f,(float)(size-1));
    const int iy = imin((int)fy,size-2); const float ty = fy-(float)iy;
    Vec3 p = start+dv*(float)y;
    for (int x = 0; x < (int)panel.w; ++x,p += du) {
      const float fx = pclamp(((float)x+0.5f)*sx-0.5f,0.0f,(float)(size-1));
      const int ix = imin((int)fx,size-2); const float tx = fx-(float)ix;
      const float depth = surface_.depth(p);
      float wet = pclamp(0.5f+depth*invAA,0.0f,1.0f);
      if (surface_.volume() <= 0) wet = 0;
      if (surface_.volume() >= 1) wet = 1;
      const float line = pmax(0.0f,1.0f-pabs(depth)*invLine)*wet;
      const int k = (y*(int)panel.w+x)*4;
      for (int c = 0; c < 3; ++c) {
        const float bottom = (float)projection[iy*size+ix][c]*(1.0f-tx)+(float)projection[iy*size+ix+1][c]*tx;
        const float top = (float)projection[(iy+1)*size+ix][c]*(1.0f-tx)+(float)projection[(iy+1)*size+ix+1][c]*tx;
        const float value = bottom*(1.0f-ty)+top*ty;
        rgba[k+c] = toByte((c == 2 ? 6.0f : 2.0f)*(1.0f-wet)+value*wet+line*48.0f);
      }
      rgba[k+3] = 255;
    }
  }
  // Rasterize only tiny local footprints. Neighbour interactions are unnecessary.
  for (const auto& d : drops_) {
    if (d.volume <= 0) continue;
    const Proj q = project(panel,d.p*cubeSide);
    const float distance = q.dist*invSide;
    if (distance < 0 || distance > 0.65f) continue;
    const float radius = pmax(1.1f,(float)panel.w*0.025f);
    const float invR2 = 1.0f/(radius*radius);
    const float fade = 1.0f-distance*0.8f;
    const int x0 = imax(0,(int)(q.s-radius)), x1 = imin((int)panel.w-1,(int)(q.s+radius+1));
    const int y0 = imax(0,(int)(q.t-radius)), y1 = imin((int)panel.h-1,(int)(q.t+radius+1));
    for (int y = y0; y <= y1; ++y) for (int x = x0; x <= x1; ++x) {
      const float dx = (float)x+0.5f-q.s, dy = (float)y+0.5f-q.t;
      const float a = pclamp(1.4f-(dx*dx+dy*dy)*invR2,0.0f,1.0f)*fade;
      const float colour[] = {45.0f+d.dye.red*200.0f+d.dye.blue*20.0f,
                              45.0f+d.dye.green*200.0f,140.0f+d.dye.blue*110.0f};
      const int k = (y*(int)panel.w+x)*4;
      for (int c = 0; c < 3; ++c) rgba[k+c] = toByte((float)rgba[k+c]*(1.0f-a)+colour[c]*a);
    }
  }
}
}  // namespace partsim
