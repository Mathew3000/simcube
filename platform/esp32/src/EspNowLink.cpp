#include "EspNowLink.h"

#include <esp_now.h>
#include <esp_wifi.h>

#include <cstring>

EspNowLink::Slot EspNowLink::s_ring[EspNowLink::kSlots];
volatile uint32_t EspNowLink::s_head = 0;
volatile uint32_t EspNowLink::s_tail = 0;
volatile uint32_t EspNowLink::s_overruns = 0;
volatile uint32_t EspNowLink::s_received = 0;

namespace {
// The broadcast address. Every cube in earshot hears every packet; which ones it acts on is the
// link id in the payload, not the MAC.
const uint8_t kBroadcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
}  // namespace

void EspNowLink::onRecv(const uint8_t* mac, const uint8_t* data, int len) {
  (void)mac;
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
  if (esp_now_send(kBroadcast, data, (size_t)len) != ESP_OK) return false;
  ++sent_;
  return true;
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
