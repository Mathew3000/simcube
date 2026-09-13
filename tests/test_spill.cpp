#include "check.h"
#include "partsim/Spill.h"

using namespace partsim;

namespace {
const Aabb kBox{Vec3{-16.0f, -16.0f, -16.0f}, Vec3{16.0f, 16.0f, 16.0f}};

SpillParticle made(int i) {
  SpillParticle s{};
  s.pos = Vec3{-15.0f + (float)i, 12.0f, 3.5f - (float)i * 0.25f};
  s.vel = Vec3{0.5f * (float)i, -8.0f, 1.25f};
#if PARTSIM_ENABLE_CHROMA
  s.cr = (uint16_t)(i * 1000);
  s.cg = (uint16_t)(i * 37);
#endif
  return s;
}
}  // namespace

TEST(spill_packet_fits_esp_now) {
  // ESP-NOW's limit is 250 bytes and is not ours to change, so a full packet must fit it.
  const int full = kSpillHeaderBytes + kSpillMaxPerPacket * kSpillBytesPerParticle;
  std::printf("       header %d + %d x %d = %d B of the 250 ESP-NOW allows\n", kSpillHeaderBytes,
              kSpillMaxPerPacket, kSpillBytesPerParticle, full);
  CHECK(full <= kSpillMaxPayload);
  CHECK(kSpillMaxPerPacket > 0);
}

TEST(spill_round_trips_through_the_wire_format) {
  SpillParticle in[kSpillMaxPerPacket];
  for (int i = 0; i < kSpillMaxPerPacket; ++i) in[i] = made(i);

  uint8_t buf[kSpillMaxPayload];
  SpillHeader h;
  h.seq = 7;
  h.totalOut = 1234;
  const int n = encodeSpill(h, in, kSpillMaxPerPacket, kBox, buf, sizeof buf);
  CHECK(n > 0);

  SpillParticle out[kSpillMaxPerPacket];
  SpillHeader got;
  const int m = decodeSpill(buf, n, kBox, out, kSpillMaxPerPacket, got);
  CHECK(m == kSpillMaxPerPacket);
  CHECK(got.seq == 7);
  CHECK(got.totalOut == 1234);  // the field the shortfall arithmetic needs
  CHECK(got.count == kSpillMaxPerPacket);

  // Quantisation bounds, not equality: position is uint16 across a 32-unit box and velocity is a
  // byte, so the tolerance is a property of the format rather than a fudge.
  const float posTol = 32.0f / 65535.0f * 2.0f;
  for (int i = 0; i < m; ++i) {
    CHECK(length(out[i].pos - in[i].pos) < posTol * 2.0f);
    CHECK(length(out[i].vel - in[i].vel) < 0.5f);
#if PARTSIM_ENABLE_CHROMA
    CHECK(out[i].cr == in[i].cr);  // dye is carried exactly; it is what a chain is FOR
    CHECK(out[i].cg == in[i].cg);
#endif
  }
}

TEST(spill_rejects_a_damaged_packet_before_touching_state) {
  SpillParticle in[4];
  for (int i = 0; i < 4; ++i) in[i] = made(i);
  uint8_t buf[kSpillMaxPayload];
  SpillHeader h;
  const int n = encodeSpill(h, in, 4, kBox, buf, sizeof buf);
  CHECK(n > 0);

  SpillParticle out[4];
  SpillHeader got;
  CHECK(decodeSpill(buf, n, kBox, out, 4, got) == 4);

  // Every byte, one at a time: a flipped bit anywhere in the header or the body must be rejected,
  // not silently decoded into particles that then enter the fluid.
  int accepted = 0;
  for (int b = 0; b < n; ++b) {
    uint8_t copy[kSpillMaxPayload];
    for (int k = 0; k < n; ++k) copy[k] = buf[k];
    copy[b] ^= 0x40;
    if (decodeSpill(copy, n, kBox, out, 4, got) >= 0) ++accepted;
  }
  std::printf("       %d of %d single-bit corruptions accepted\n", accepted, n);
  // NONE. Version 1 covered the header only and accepted every one of the body's corruptions on
  // the reasoning that a wrong position is harmless -- which was wrong twice over: dye rides in
  // the body too, and a Fletcher-16 is blind to 0x00 <-> 0xFF, so seven of the header's own bytes
  // were substitutable as well. The CRC-16 covers the whole packet.
  CHECK(accepted == 0);

  CHECK(decodeSpill(buf, kSpillHeaderBytes - 1, kBox, out, 4, got) == -1);  // truncated
}

TEST(spill_rejects_every_single_byte_substitution_in_the_header) {
  // The corruption a Fletcher-16 cannot see. It sums modulo 255, where 0x00 and 0xFF are
  // congruent, so a zero byte could be substituted with 0xFF and the checksum did not move --
  // and the bytes that are zero in real traffic are the high halves of seq and totalOut and
  // `from` on cube 0, which is to say the addressing and the entire shortfall mechanism.
  SpillParticle in[2];
  for (int i = 0; i < 2; ++i) in[i] = made(i);
  uint8_t buf[kSpillMaxPayload];
  SpillHeader h;
  h.seq = 7;
  h.totalOut = 7;  // deliberately small, so the high bytes are zero as they are in real traffic
  const int n = encodeSpill(h, in, 2, kBox, buf, sizeof buf);
  CHECK(n > 0);

  SpillParticle out[2];
  SpillHeader got;
  int accepted = 0, tried = 0;
  for (int b = 0; b < kSpillHeaderBytes; ++b)
    for (int v = 0; v < 256; ++v) {
      if ((uint8_t)v == buf[b]) continue;
      uint8_t copy[kSpillMaxPayload];
      for (int k = 0; k < n; ++k) copy[k] = buf[k];
      copy[b] = (uint8_t)v;
      ++tried;
      if (decodeSpill(copy, n, kBox, out, 2, got) >= 0) ++accepted;
    }
  std::printf("       %d of %d single-byte header substitutions accepted\n", accepted, tried);
  CHECK(accepted == 0);
}

#if PARTSIM_ENABLE_CHROMA
TEST(spill_clamps_dye_that_would_create_colour_from_nothing) {
  // Defence in depth behind the checksum: the renderer resolves colour as a ratio whose
  // denominator is cr + cg + cb, with blue implied. A pair that breaks cr + cg <= kChromaOne makes
  // the implied blue negative, and the arrival brings dye into the ring that nobody poured.
  SpillParticle p{};
  p.pos = Vec3{0.0f, 0.0f, 0.0f};
  p.vel = Vec3{0.0f, 0.0f, 0.0f};
  p.cr = kChromaOne;
  p.cg = kChromaOne;  // twice the dye a particle can hold, encoded with a VALID checksum
  uint8_t buf[kSpillMaxPayload];
  SpillHeader h;
  const int n = encodeSpill(h, &p, 1, kBox, buf, sizeof buf);
  CHECK(n > 0);

  SpillParticle out[1];
  SpillHeader got;
  CHECK(decodeSpill(buf, n, kBox, out, 1, got) == 1);
  CHECK((uint32_t)out[0].cr + (uint32_t)out[0].cg <= (uint32_t)kChromaOne);
  CHECK(out[0].cr == kChromaOne);  // the first channel survives; the second gives way
  CHECK(out[0].cg == 0);
}
#endif

TEST(spill_receiver_will_not_manufacture_an_implausible_shortfall) {
  // One flipped byte in totalOut asked for 65 535 particles, and the chain would have created
  // them eighteen a frame for hours -- with the beaker looking plausible throughout, because it
  // pours out of its own top at roughly the rate it manufactures clones.
  SpillReceiver rx;
  SpillHeader h;
  h.totalOut = 10;
  CHECK(rx.note(h, 1) == 0);

  h.totalOut = 10u + (1u << 16);  // byte 2 of the counter, flipped once
  const uint32_t missed = rx.note(h, 1);
  std::printf("       implausible jump reported as %u (asked for %u)\n", missed, 1u << 16);
  CHECK(missed == kMaxShortfallPerPacket);
  CHECK(rx.resyncs() == 1u);

  // And a sender that RESTARTS counts backwards -- init() resets the cumulative counters. Left
  // alone, the receiver's own count stays ahead of the sender's for good and the shortfall
  // mechanism is silently disabled for the rest of the run.
  SpillReceiver r2;
  SpillHeader a;
  a.totalOut = 500;
  r2.note(a, 5);
  a.totalOut = 520;
  r2.note(a, 20);
  a.totalOut = 4;  // the far cube rebooted
  CHECK(r2.note(a, 4) == 0);
  CHECK(r2.resyncs() == 1u);
  // Back in step: the next genuine loss is reported normally rather than swallowed.
  a.totalOut = 12;
  CHECK(r2.note(a, 4) == 4u);
}

TEST(spill_receiver_counts_what_the_radio_lost) {
  SpillReceiver rx;
  SpillHeader h;

  // First packet ever seen: a receiver joining a running chain has not lost the history before it.
  h.totalOut = 100;
  CHECK(rx.note(h, 5) == 0);
  CHECK(rx.started());

  // Normal delivery: totalOut advances by exactly what arrived.
  h.totalOut = 105;
  CHECK(rx.note(h, 5) == 0);
  CHECK(rx.shortfall() == 0);

  // A packet of 6 is lost: the next one says 117 while only 6 more arrived.
  h.totalOut = 117;
  CHECK(rx.note(h, 6) == 6);
  CHECK(rx.shortfall() == 6);

  // The same shortfall must not be reported twice -- it is made up once and then forgotten.
  h.totalOut = 120;
  CHECK(rx.note(h, 3) == 0);
  CHECK(rx.shortfall() == 6);
  std::printf("       shortfall after one lost 6-particle packet: %u\n", rx.shortfall());
}

TEST(spill_queue_counts_what_it_drops) {
  // Overflow must be counted, not silent: in a closed ring a dropped particle is volume that never
  // returns, and beakers slowly emptying reads as a physics leak rather than as a full buffer.
  SpillQueue q;
  SpillParticle s = made(1);
  for (int i = 0; i < kMaxSpill + 5; ++i) q.push(s);
  CHECK(q.count == kMaxSpill);
  CHECK(q.totalOut == (uint32_t)(kMaxSpill + 5));
  CHECK(q.dropped == 5);
  q.clear();
  CHECK(q.count == 0);
  CHECK(q.totalOut == (uint32_t)(kMaxSpill + 5));  // cumulative: clear() is not a reset
}
