#include "SpiFrameLink.h"

#include <Arduino.h>
#include <driver/spi_master.h>
#include <driver/spi_slave.h>
#include <string.h>

using namespace partsim;

namespace {
constexpr spi_host_device_t kHost = SPI2_HOST;
spi_device_handle_t g_dev = nullptr;

// DMA-capable staging, one direction per role. Declaring both cost 24KB of a 230KB budget to a
// board that can only ever use one of them -- the same mistake RenderState exists to prevent, made
// again somewhere else.
#ifdef PARTSIM_PROFILE_ESP32_MASTER
DMA_ATTR uint8_t g_tx[frameMaxBytes()];
#endif
#ifdef PARTSIM_PROFILE_ESP32_DISPLAY
DMA_ATTR uint8_t g_rx[frameMaxBytes()];
#endif
}  // namespace

#ifdef PARTSIM_PROFILE_ESP32_MASTER
bool SpiMasterLink::begin(int sck, int mosi, int miso, const int cs[3], int nodes, int hz) {
  nodes_ = nodes;
  for (int i = 0; i < 3; ++i) cs_[i] = (i < nodes) ? cs[i] : -1;

  spi_bus_config_t bus = {};
  bus.mosi_io_num = mosi;
  bus.miso_io_num = miso;
  bus.sclk_io_num = sck;
  bus.quadwp_io_num = -1;
  bus.quadhd_io_num = -1;
  bus.max_transfer_sz = frameMaxBytes();
  if (spi_bus_initialize(kHost, &bus, SPI_DMA_CH_AUTO) != ESP_OK) return false;

  // CS is driven by GPIO, not by the driver, because the whole point is asserting several at once.
  spi_device_interface_config_t dev = {};
  dev.clock_speed_hz = hz;
  dev.mode = 0;
  dev.spics_io_num = -1;
  dev.queue_size = 2;
  if (spi_bus_add_device(kHost, &dev, &g_dev) != ESP_OK) return false;

  for (int i = 0; i < nodes_; ++i) {
    if (cs_[i] < 0) continue;
    pinMode(cs_[i], OUTPUT);
    digitalWrite(cs_[i], HIGH);  // idle high
  }
  ready_ = true;
  return true;
}

bool SpiMasterLink::send(const uint8_t* bytes, int len) {
  if (!ready_ || len <= 0 || len > (int)sizeof(g_tx)) return false;
  memcpy(g_tx, bytes, (size_t)len);

  spi_transaction_t t = {};
  t.length = (size_t)len * 8;
  t.tx_buffer = g_tx;
  t.rx_buffer = nullptr;

  auto one = [&](int lo, int hi) -> bool {
    for (int i = lo; i < hi; ++i)
      if (cs_[i] >= 0) digitalWrite(cs_[i], LOW);
    const esp_err_t e = spi_device_transmit(g_dev, &t);
    for (int i = lo; i < hi; ++i)
      if (cs_[i] >= 0) digitalWrite(cs_[i], HIGH);
    return e == ESP_OK;
  };

  bool ok;
  if (kBroadcast) {
    ok = one(0, nodes_);  // all chip selects together: one transaction for every node
  } else {
    ok = true;
    for (int i = 0; i < nodes_; ++i) ok = one(i, i + 1) && ok;
  }
  if (ok) ++sent_; else ++errors_;
  return ok;
}

#endif  // master

#ifdef PARTSIM_PROFILE_ESP32_DISPLAY
bool SpiDisplayLink::begin(int sck, int mosi, int miso, int cs) {
  spi_bus_config_t bus = {};
  bus.mosi_io_num = mosi;
  bus.miso_io_num = miso;
  bus.sclk_io_num = sck;
  bus.quadwp_io_num = -1;
  bus.quadhd_io_num = -1;
  bus.max_transfer_sz = frameMaxBytes();

  spi_slave_interface_config_t slv = {};
  slv.mode = 0;
  slv.spics_io_num = cs;
  slv.queue_size = 2;
  slv.flags = 0;
  if (spi_slave_initialize(kHost, &bus, &slv, SPI_DMA_CH_AUTO) != ESP_OK) return false;
  ready_ = true;
  return true;
}

const uint8_t* SpiDisplayLink::pollDirect(int& len) {
  len = 0;
  if (!ready_) return nullptr;

  static spi_slave_transaction_t t;
  if (!queued_) {
    memset(&t, 0, sizeof(t));
    t.length = (size_t)sizeof(g_rx) * 8;
    t.rx_buffer = g_rx;
    t.tx_buffer = nullptr;
    if (spi_slave_queue_trans(kHost, &t, 0) != ESP_OK) { ++errors_; return nullptr; }
    queued_ = true;
  }

  // Non-blocking. A display node that finds no frame holds its previous one, which is the
  // specified behaviour rather than a fallback.
  spi_slave_transaction_t* done = nullptr;
  if (spi_slave_get_trans_result(kHost, &done, 0) != ESP_OK) return nullptr;
  queued_ = false;

  const int got = (int)(done->trans_len / 8);
  if (got <= 0 || got > (int)sizeof(g_rx)) { ++errors_; return nullptr; }
  ++received_;
  len = got;
  return g_rx;
}

int SpiDisplayLink::poll(uint8_t* buf, int cap) {
  int len = 0;
  const uint8_t* p = pollDirect(len);
  if (!p || len > cap) return 0;
  memcpy(buf, p, (size_t)len);
  return len;
}
#endif  // display
