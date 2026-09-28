#include "partsim/SurfaceWater.h"

namespace partsim {
namespace {

// Integrate max(a*x + b*y + c, 0) over the unit square. Clip its four
// corners to the positive half-plane, then integrate the affine function
// exactly over a triangle fan. Unlike a 3D uniform-distribution CDF this
// does not divide by tiny normal components near an axis-aligned pose.
struct Point { float x, y, h; };
float positiveIntegral(float a, float b, float c) {
  const Point square[4] = {{-0.5f, -0.5f, c - 0.5f*a - 0.5f*b},
                           {0.5f, -0.5f, c + 0.5f*a - 0.5f*b},
                           {0.5f, 0.5f, c + 0.5f*a + 0.5f*b},
                           {-0.5f, 0.5f, c - 0.5f*a + 0.5f*b}};
  Point polygon[8];
  int count = 0;
  for (int i = 0; i < 4; ++i) {
    const Point p = square[i], q = square[(i + 1) % 4];
    if (p.h > 0.0f) polygon[count++] = p;
    if ((p.h > 0.0f) != (q.h > 0.0f)) {
      const float t = p.h / (p.h - q.h);
      polygon[count++] = {p.x + (q.x-p.x)*t, p.y + (q.y-p.y)*t, 0.0f};
    }
  }
  float integral = 0.0f;
  for (int i = 1; i + 1 < count; ++i) {
    const Point p = polygon[0], q = polygon[i], r = polygon[i+1];
    const float twiceArea = (q.x-p.x)*(r.y-p.y) - (q.y-p.y)*(r.x-p.x);
    integral += twiceArea * (p.h + q.h + r.h) * (1.0f / 6.0f);
  }
  return integral;
}

Vec3 bounded(Vec3 v, float maxLength) {
  const float d2 = length2(v);
  return d2 > maxLength*maxLength ? v * (maxLength * frsqrt(d2)) : v;
}

uint8_t byte(float value) { return (uint8_t)pclamp(value, 0.0f, 255.0f); }

}  // namespace

void SurfaceWater::reset(float volume) {
  up_ = targetUp_ = {0, 1, 0};
  velocity_ = {0, 0, 0};
  ripple_ = rippleVelocity_ = phase_ = 0.0f;
  setVolume(volume);
}

void SurfaceWater::setVolume(float volume) {
  volume_ = pclamp(volume, 0.0f, 1.0f);
  solveLevel();
}

float SurfaceWater::volumeBelow(Vec3 up, float level) {
  // Cube symmetry lets us use absolute components and integrate along the
  // largest: this denominator is >= 1/sqrt(3) for a unit normal.
  float a = pabs(up.x), b = pabs(up.y), c = pabs(up.z);
  if (a > c) { const float t = c; c = a; a = t; }
  if (b > c) { const float t = c; c = b; b = t; }
  if (c < 1e-8f) return level >= 0.0f ? 1.0f : 0.0f;
  const float extent = 0.5f*(a+b+c);
  if (level <= -extent) return 0.0f;
  if (level >= extent) return 1.0f;
  const float inv = 1.0f/c;
  const float h = level*inv + 0.5f;
  // clamp(h,0,1) = max(h,0) - max(h-1,0).
  return pclamp(positiveIntegral(-a*inv, -b*inv, h) -
                positiveIntegral(-a*inv, -b*inv, h-1.0f), 0.0f, 1.0f);
}

void SurfaceWater::solveLevel() {
  const float extent = 0.5f*(pabs(up_.x)+pabs(up_.y)+pabs(up_.z));
  if (volume_ <= 0.0f) { level_ = -extent; return; }
  if (volume_ >= 1.0f) { level_ = extent; return; }
  float lo = -extent, hi = extent;
  for (int i = 0; i < 19; ++i) {
    const float mid = 0.5f*(lo+hi);
    if (volumeBelow(up_, mid) < volume_) lo = mid;
    else hi = mid;
  }
  level_ = 0.5f*(lo+hi);
}

void SurfaceWater::impulse(Vec3 accelerationG) {
  const Vec3 a = bounded(accelerationG, 4.0f);
  velocity_ += (a - up_*dot(a, up_)) * 1.4f;
  velocity_ = bounded(velocity_, 5.0f);
  rippleVelocity_ = pclamp(rippleVelocity_ + length2(a)*0.25f, -4.0f, 4.0f);
}

void SurfaceWater::step(float dt, Vec3 down, Vec3 accelerationG) {
  if (!(dt > 0.0f)) return;
  dt = pmin(dt, 0.1f);
  if (length2(down) > 1e-8f) targetUp_ = -down * frsqrt(length2(down));
  const Vec3 target = normalize(targetUp_ + bounded(accelerationG, 3.0f)*0.35f);
  const int steps = (int)(dt*120.0f) + 1;
  const float h = dt/(float)steps;
  for (int i = 0; i < steps; ++i) {
    // A spring in vector space, renormalized after integration. The explicit
    // antipodal case prevents a perfectly upside-down cube remaining stuck.
    Vec3 force = (target-up_)*48.0f - velocity_*5.5f;
    if (dot(target, up_) < -0.99f) {
      const Vec3 axis = pabs(up_.x) < 0.8f ? Vec3{1,0,0} : Vec3{0,0,1};
      force += normalize(cross(up_, axis))*12.0f;
    }
    velocity_ += force*h;
    up_ = normalize(up_ + velocity_*h);
    velocity_ -= up_*dot(velocity_, up_);
    rippleVelocity_ += (-100.0f*ripple_ - 4.0f*rippleVelocity_)*h;
    ripple_ += rippleVelocity_*h;
  }
  phase_ += dt*1.3f;
  if (phase_ > kTwoPi) phase_ -= kTwoPi;
  solveLevel();
}

float SurfaceWater::outflowRate() const {
  if (volume_ <= 0.0f) return 0.0f;
  // Lowest edge of the virtual +Y opening in the current surface frame.
  const float rim = 0.5f*(up_.y - pabs(up_.x) - pabs(up_.z));
  const float head = pmax(0.0f, level_-rim);
  // Stylized weir flow: head^(3/2). Zero head means no leak while upright.
  return 1.6f*head*psqrt(head);
}

float SurfaceWater::pourTo(SurfaceWater& receiver, float dt) {
  if (&receiver == this || !(dt > 0.0f)) return 0.0f;
  const float amount = pmin(pmin(outflowRate()*pmin(dt, 0.1f), volume_),
                            pmax(0.0f, 1.0f-receiver.volume_));
  if (!(amount > 0.0f)) return 0.0f;
  setVolume(volume_-amount);
  receiver.setVolume(receiver.volume_+amount);
  receiver.rippleVelocity_ = pmin(4.0f, receiver.rippleVelocity_+amount*20.0f);
  return amount;
}

void SurfaceWater::renderPanel(const Panel& panel, float cubeSide, uint8_t* rgba) const {
  if (!rgba || !(cubeSide > 0.0f)) return;
  const float invSide = 1.0f/cubeSide;
  const Vec3 du = panel.u*invSide, dv = panel.v*invSide;
  const Vec3 start = (panel.origin + (panel.u+panel.v)*0.5f)*invSide;
  const float aa = pmax(0.001f, (pabs(dot(up_,du))+pabs(dot(up_,dv)))*0.65f);
  const float invAA = 1.0f/aa;
  const float invLineWidth = 1.0f/(0.012f+aa);
  const float shimmerA = fsin(phase_), shimmerB = fcos(phase_*2.0f);
  for (int y = 0; y < (int)panel.h; ++y) {
    Vec3 p = start + dv*(float)y;
    for (int x = 0; x < (int)panel.w; ++x, p += du) {
      const float d = depth(p);
      float wet = pclamp(0.5f + d*invAA, 0.0f, 1.0f);
      if (volume_ <= 0.0f) wet = 0.0f;
      if (volume_ >= 1.0f) wet = 1.0f;
      const float line = pclamp(1.0f-pabs(d)*invLineWidth, 0.0f, 1.0f)*wet;
      const float shallow = 1.0f-pclamp(d*1.7f, 0.0f, 1.0f);
      // Low-order shared 3D lighting modes: continuous over face seams.
      // Ripples affect shading only, so they cannot invent apparent fill volume.
      const float pattern = (p.x*p.z*4.0f)*shimmerA +
                            (p.x*p.x-p.z*p.z)*shimmerB;
      const float light = pclamp(0.83f + pattern*(0.12f+ripple_*0.8f), 0.5f, 1.2f);
      const int k = (y*(int)panel.w+x)*4;
      rgba[k]   = byte(2.0f + wet*(5.0f+shallow*7.0f)*light + line*65.0f);
      rgba[k+1] = byte(3.0f + wet*(50.0f+shallow*52.0f)*light + line*85.0f);
      rgba[k+2] = byte(6.0f + wet*(110.0f+shallow*55.0f)*light + line*65.0f);
      rgba[k+3] = 255;
    }
  }
}

}  // namespace partsim
