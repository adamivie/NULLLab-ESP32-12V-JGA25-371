#include "QuadratureEncoder.h"

#include <esp_log.h>

namespace {
constexpr const char* kTag = "encoder";
// Hardware limits are +/-32767; the driver accumulates past them when the
// limits are registered as watch points.
constexpr int kHighLimit = 30000;
constexpr int kLowLimit = -30000;
}  // namespace

QuadratureEncoder::~QuadratureEncoder() { release(); }

void QuadratureEncoder::release() {
  if (unit_ != nullptr) {
    pcnt_unit_stop(unit_);
    pcnt_unit_disable(unit_);
  }
  if (channelA_ != nullptr) pcnt_del_channel(channelA_);
  if (channelB_ != nullptr) pcnt_del_channel(channelB_);
  if (unit_ != nullptr) pcnt_del_unit(unit_);
  channelA_ = channelB_ = nullptr;
  unit_ = nullptr;
}

bool QuadratureEncoder::begin(uint32_t glitchFilterNs) {
  release();

  pcnt_unit_config_t unitConfig = {};
  unitConfig.low_limit = kLowLimit;
  unitConfig.high_limit = kHighLimit;
  unitConfig.flags.accum_count = 1;

  esp_err_t err = pcnt_new_unit(&unitConfig, &unit_);
  if (err != ESP_OK) {
    ESP_LOGE(kTag, "pcnt_new_unit failed (A=%d B=%d): %s", pinA_, pinB_, esp_err_to_name(err));
    unit_ = nullptr;
    return false;
  }

  // Standard 4x quadrature decoding (as in the ESP-IDF rotary encoder
  // example): each channel counts edges of one signal and uses the other as
  // the direction level.
  pcnt_chan_config_t configA = {};
  configA.edge_gpio_num = pinA_;
  configA.level_gpio_num = pinB_;
  pcnt_chan_config_t configB = {};
  configB.edge_gpio_num = pinB_;
  configB.level_gpio_num = pinA_;

  bool ok = true;
  if (glitchFilterNs > 0) {
    pcnt_glitch_filter_config_t filter = {};
    filter.max_glitch_ns = glitchFilterNs;
    ok &= pcnt_unit_set_glitch_filter(unit_, &filter) == ESP_OK;
  }
  ok &= pcnt_new_channel(unit_, &configA, &channelA_) == ESP_OK;
  ok &= pcnt_new_channel(unit_, &configB, &channelB_) == ESP_OK;
  if (ok) {
    ok &= pcnt_channel_set_edge_action(channelA_, PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                                       PCNT_CHANNEL_EDGE_ACTION_INCREASE) == ESP_OK;
    ok &= pcnt_channel_set_level_action(channelA_, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                        PCNT_CHANNEL_LEVEL_ACTION_INVERSE) == ESP_OK;
    ok &= pcnt_channel_set_edge_action(channelB_, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                       PCNT_CHANNEL_EDGE_ACTION_DECREASE) == ESP_OK;
    ok &= pcnt_channel_set_level_action(channelB_, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                        PCNT_CHANNEL_LEVEL_ACTION_INVERSE) == ESP_OK;
    ok &= pcnt_unit_add_watch_point(unit_, kHighLimit) == ESP_OK;
    ok &= pcnt_unit_add_watch_point(unit_, kLowLimit) == ESP_OK;
    ok &= pcnt_unit_enable(unit_) == ESP_OK;
    ok &= pcnt_unit_clear_count(unit_) == ESP_OK;
    ok &= pcnt_unit_start(unit_) == ESP_OK;
  }
  if (!ok) {
    ESP_LOGE(kTag, "PCNT setup failed (A=%d B=%d)", pinA_, pinB_);
    release();
    return false;
  }
  lastRaw_ = 0;
  accumulated_ = 0;
  return true;
}

int64_t QuadratureEncoder::count() {
  if (unit_ == nullptr) return 0;
  int raw = 0;
  if (pcnt_unit_get_count(unit_, &raw) != ESP_OK) raw = lastRaw_;
  // The driver's accumulated value is an int; take the difference with
  // unsigned arithmetic so it stays correct even if that int ever wraps.
  const int32_t delta = static_cast<int32_t>(static_cast<uint32_t>(raw) - static_cast<uint32_t>(lastRaw_));
  lastRaw_ = raw;
  accumulated_ += delta;
  return inverted_ ? -accumulated_ : accumulated_;
}

void QuadratureEncoder::reset() {
  if (unit_ != nullptr) pcnt_unit_clear_count(unit_);
  lastRaw_ = 0;
  accumulated_ = 0;
}
