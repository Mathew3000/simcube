// Mean LED duty cycle per scene -- the number that actually sets panel current, and therefore the
// pack size, the converter rating and the runtime requirement in docs/CUBE-PCB.md.
//
// Datasheet "max power" assumes a white screen. A fluid simulation is mostly dark, so using the
// datasheet figure directly would oversize the battery by roughly 3x. This measures the real thing
// from the resolved pixels, and must be re-run after any renderer change: the average-picture-level
// limiter (REQ-PWR-1) is sized against it.

#include <cstdio>
#include <initializer_list>
#include "partsim/Simulation.h"
using namespace partsim;
Simulation sim;
uint8_t face[kMaxPanelTexels * 4];

int main() {
  std::printf("scene                    meanR meanG meanB   duty%%   peak%%  lit%%\n");
  double worstDuty = 0.0; const char* worstName = "";
  for (int s = 0; s < sceneCount(); ++s) {
    sim.initScene(Simulation::kCube, s, 1);
    sim.setAutoCycle(false);
    for (int i = 0; i < 600; ++i) sim.stepFixed();   // settle, and let fire reach steady state
    sim.accumulate();

    double sum[3] = {0, 0, 0}; long n = 0, lit = 0; int peak = 0;
    for (int k = 0; k < sim.geometry().count(); ++k) {
      const Panel& p = sim.geometry().at(k);
      sim.renderer().resolve(k, face, 4);
      const long texels = (long)p.w * (long)p.h;
      for (long t = 0; t < texels; ++t) {
        const int r = face[t*4+0], g = face[t*4+1], b = face[t*4+2];
        sum[0]+=r; sum[1]+=g; sum[2]+=b; ++n;
        if (r+g+b > 12) ++lit;
        const int mx = r > g ? (r > b ? r : b) : (g > b ? g : b);
        if (mx > peak) peak = mx;
      }
    }
    const double mr=sum[0]/n, mg=sum[1]/n, mb=sum[2]/n;
    const double duty = (mr+mg+mb) / (3.0*255.0) * 100.0;
    if (duty > worstDuty) { worstDuty = duty; worstName = sceneAt(s).name; }
    std::printf("%-24s %5.1f %5.1f %5.1f   %5.2f   %5.1f  %4.1f\n",
                sceneAt(s).name, mr, mg, mb, duty, peak*100.0/255.0, lit*100.0/n);
  }
  std::printf("\nworst scene: %s at %.2f%% mean duty\n", worstName, worstDuty);
  return 0;
}
