#include "EspNowLink.h"

#include <esp_now.h>
#include <esp_wifi.h>

#include <cstdio>
#include <cstring>

EspNowLink::Slot EspNowLink::s_ring[EspNowLink::kSlots];
volatile uint32_t EspNowLink::s_head = 0;
volatile uint32_t EspNowLink::s_tail = 0;
volatile uint32_t EspNowLink::s_overruns = 0;
volatile uint32_t EspNowLink::s_received = 0;
volatile uint8_t EspNowLink::s_lastPeer[6] = {0};

namespace {
// The broadcast address. Every cube in earshot hears every packet; which ones it acts on is the
// link id in the payload, not the MAC.
const uint8_t kBroadcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
}  // namespace

void EspNowLink::onRecv(const uint8_t* mac, const uint8_t* data, int len) {
  if (mac)
    for (int i = 0; i < 6; ++i) s_lastPeer[i] = mac[i];
  if (len <= 0 || len > (int)partsim::kSpillMaxPayload) return;
  const uint32_t head = s_head;
  const uint32_t next = (head + 1) % kSlots;
  if (next == s_tail) {  // ring full: drop the NEWEST
    ++s_overruns;
    return;
  }
  // Dropping the newest rather than overwriting the oldest is deliberate. The receiver's shortfall
  // arithmetic is built on a cumulative count that only ever advances, so losing a recent packet
  // is made up on the next one; overwriting an unread older packet would lose particles the
  // consumer already believes it is about to get.
  std::memcpy(s_ring[head].data, data, (size_t)len);
  s_ring[head].len = len;
  s_head = next;
  ++s_received;
}

bool EspNowLink::begin() {
  // esp_now_init() is called in setup() alongside WiFi, because the radio's cost is measured there
  // and bringing it up twice is not free. This only adds the peer and the callback.
  esp_now_peer_info_t peer = {};
  std::memcpy(peer.peer_addr, kBroadcast, 6);
  peer.channel = 0;  // current channel
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) return false;
  if (esp_now_register_recv_cb(onRecv) != ESP_OK) return false;
  ready_ = true;
  return true;
}

bool EspNowLink::send(const uint8_t* data, int len) {
  if (!ready_ || len <= 0 || len > (int)partsim::kSpillMaxPayload) return false;
  const esp_err_t e = esp_now_send(kBroadcast, data, (size_t)len);
  if (e != ESP_OK) {
    // COUNTED, because the first hardware run refused 14 of 39 packets during a fast pour and the
    // only evidence was a particle count that did not add up at the far end.
    ++failed_;
    lastErr_ = (int)e;
    return false;
  }
  ++sent_;
  return true;
}

const char* EspNowLink::diagnostic() const {
  uint8_t self[6] = {0};
  esp_wifi_get_mac(WIFI_IF_STA, self);
  uint8_t peer[6];
  for (int i = 0; i < 6; ++i) peer[i] = s_lastPeer[i];
  bool heardSelf = true;
  for (int i = 0; i < 6; ++i)
    if (peer[i] != self[i]) heardSelf = false;
  std::snprintf(diag_, sizeof diag_,
                "radio: refused %u (last err 0x%x), ring overruns %u, self %02x%02x%02x%02x%02x%02x"
                ", last peer %02x%02x%02x%02x%02x%02x%s",
                (unsigned)failed_, (unsigned)lastErr_, (unsigned)s_overruns, self[0], self[1],
                self[2], self[3], self[4], self[5], peer[0], peer[1], peer[2], peer[3], peer[4],
                peer[5], heardSelf ? "  <-- ITSELF" : "");
  return diag_;
}

int EspNowLink::poll(uint8_t* out, int cap) {
  overruns_ = s_overruns;
  received_ = s_received;
  const uint32_t tail = s_tail;
  if (tail == s_head) return 0;
  const int len = s_ring[tail].len;
  const int n = len < cap ? len : cap;
  std::memcpy(out, s_ring[tail].data, (size_t)n);
  s_tail = (tail + 1) % kSlots;
  return n;
}
