#pragma once
#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>

// Direct access to the HUB75 library's DMA back buffer, so a whole row can be written without
// going through drawPixelRGB888 once per texel.
//
// WHY THIS EXISTS
//
// `drawPixelRGB888` costs 7.50 ms of the 11.10 ms blit at six 32x32 faces, and only 0.20 ms of
// that is the scattered addressing -- measured by rewriting the inner loop to write every texel to
// (0,0), which changed the blit by 0.20 ms. The cost is per-call work, and the largest single item
// is that the library recomputes
//
//     &fb->rowBits[y]->data[depth * fb->rowBits[y]->width]
//
// for every bitplane of every texel: a pointer, a vector, a shared_ptr, two more loads and a
// multiply, six times per pixel. Hoisting it to once per row per plane is the whole optimisation,
// and hoisting it requires the pointer it is computed from.
//
// `MatrixPanel_I2S_DMA::fb` is private and the library exposes no bulk-pixel entry point (the
// `hlineDMA` family takes a single colour for the whole span, which a fluid render never has).
// Vendoring a 40-file third-party library to add one accessor is a permanent maintenance cost for
// a five-line change, so instead this uses the explicit-instantiation idiom below.
//
// WHY IT IS NOT A HACK IN THE DANGEROUS SENSE
//
// [temp.spec]/6: "The usual access checking rules do not apply to names in a declaration of an
// explicit instantiation." So `&MatrixPanel_I2S_DMA::fb` is legal there, and the friend function
// injected by the template is the ordinary, portable way to carry the result out. GCC and Clang
// have both supported this since C++11; it is not a layout assumption and not undefined behaviour.
//
// What it IS is a dependency on a private member's NAME and TYPE. Both failure modes are loud:
//
//   * a renamed or retyped member is a compile error, not a wrong picture, and
//   * a changed buffer LAYOUT is caught at boot by PanelDriver::verifyFastBlit(), which writes
//     sample texels through the library, writes the same texels through this path, and compares
//     the raw DMA words. On any mismatch the driver keeps the per-texel path for good and says so
//     on the console.
//
// The library is pinned at ^3.0.11 and resolves to 3.0.15.

namespace panelfb {

// The explicit-instantiation accessor. `Rob<Tag, M>` injects `get(Tag)` returning the
// pointer-to-member M, which the instantiation below is allowed to name despite it being private.
template <class Tag, typename Tag::type M>
struct Rob {
  friend typename Tag::type get(Tag) { return M; }
};

struct FbTag {
  typedef frameStruct* MatrixPanel_I2S_DMA::*type;
  friend type get(FbTag);
};
template struct Rob<FbTag, &MatrixPanel_I2S_DMA::fb>;

// The frame the library is currently accepting writes into -- the BACK buffer under double
// buffering, which is what present() is filling. It changes on every flipDMABuffer(), so this is
// read per frame rather than cached.
inline frameStruct* backBuffer(MatrixPanel_I2S_DMA* p) { return p->*get(FbTag{}); }

// Whether the bit packing this file's caller reproduces is the one the library compiles.
//
// The caller reads `lumConvTab` directly, which is only the library's whole brightness
// compensation when the CIE table is enabled AND is already at the panel's bit depth. The other
// branches of cie_luts.h need a runtime shift-and-round that is not worth replicating, so the
// fast path simply switches itself off for them.
#if defined(NO_CIE1931) || !defined(LUT_NATIVE_BIT_DEPTH) || LUT_NATIVE_BIT_DEPTH == 0
#define PARTSIM_FAST_BLIT 0
#elif defined(ESP32_THE_ORIG)
// The original ESP32's I2S FIFO wants adjacent uint16s swapped within the row; the S3's LCD_CAM
// path does not. Every board this project targets is an S3, so rather than carry the swap, the
// fast path is compiled out where it would be needed.
#define PARTSIM_FAST_BLIT 0
#else
#define PARTSIM_FAST_BLIT 1
#endif

}  // namespace panelfb
