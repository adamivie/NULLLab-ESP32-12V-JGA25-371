#include "LedEffects.h"

#include <math.h>

namespace {

constexpr float kTwoPi = 6.28318530718f;
constexpr float kIdleLevel = 0.25f;  // brightness of "moving only" effects at rest

float frac(float v) { return v - floorf(v); }
float clamp01(float v) { return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v); }

LedRgb fromHex(uint32_t c) {
  return {static_cast<uint8_t>(c >> 16), static_cast<uint8_t>(c >> 8), static_cast<uint8_t>(c)};
}

LedRgb scale(LedRgb c, float level) {
  level = clamp01(level);
  return {static_cast<uint8_t>(c.r * level + 0.5f), static_cast<uint8_t>(c.g * level + 0.5f),
          static_cast<uint8_t>(c.b * level + 0.5f)};
}

LedRgb mix(LedRgb a, LedRgb b, float t) {
  t = clamp01(t);
  return {static_cast<uint8_t>(a.r + (b.r - a.r) * t + 0.5f), static_cast<uint8_t>(a.g + (b.g - a.g) * t + 0.5f),
          static_cast<uint8_t>(a.b + (b.b - a.b) * t + 0.5f)};
}

LedRgb hue(float h) {  // fully saturated colour wheel, h in cycles
  const float x = frac(h) * 6.0f;
  const int sector = static_cast<int>(x);
  const float f = x - sector;
  const uint8_t up = static_cast<uint8_t>(255.0f * f + 0.5f), down = static_cast<uint8_t>(255.0f * (1.0f - f) + 0.5f);
  switch (sector) {
    case 0: return {255, up, 0};
    case 1: return {down, 255, 0};
    case 2: return {0, 255, up};
    case 3: return {0, down, 255};
    case 4: return {up, 0, 255};
    default: return {255, 0, down};
  }
}

struct PaletteDef {
  int count;
  uint32_t colors[4];
};
constexpr PaletteDef kPalettes[] = {
    {3, {0xFF0000, 0xFF5A00, 0xFFC800, 0}},         // Fire
    {3, {0x0018FF, 0x00C8FF, 0x00FFA0, 0}},         // Ocean
    {3, {0x00FF20, 0x90FF00, 0x006410, 0}},         // Forest
    {3, {0x8000FF, 0xFF0080, 0xFF6400, 0}},         // Sunset
    {4, {0xFF00C8, 0x00E0FF, 0xFFE000, 0x40FF00}},  // Party
    {3, {0xFFFFFF, 0x60C0FF, 0x0040FF, 0}},         // Ice
};

// Smoothly cycles through the palette; t in cycles.
LedRgb paletteColor(const LedConfig& config, float t) {
  uint32_t custom[2] = {config.color1, config.color2};
  const uint32_t* colors = custom;
  int count = 2;
  if (config.palette != LedPalette::Custom) {
    const PaletteDef& def = kPalettes[static_cast<int>(config.palette)];
    colors = def.colors;
    count = def.count;
  }
  const float x = frac(t) * count;
  const int i = static_cast<int>(x) % count;
  return mix(fromHex(colors[i]), fromHex(colors[(i + 1) % count]), x - floorf(x));
}

}  // namespace

bool ledConfigFromFields(const uint32_t f[kLedConfigFields], LedConfig* out) {
  if (f[0] >= static_cast<uint32_t>(LedMode::Count) || f[1] > 0xFFFFFF || f[2] > 0xFFFFFF ||
      f[3] >= static_cast<uint32_t>(LedPalette::Count) || f[4] < 1 || f[4] > 100 || f[5] < 1 || f[5] > 255 || f[6] > 1) {
    return false;
  }
  out->mode = static_cast<LedMode>(f[0]);
  out->color1 = f[1];
  out->color2 = f[2];
  out->palette = static_cast<LedPalette>(f[3]);
  out->speed = static_cast<uint8_t>(f[4]);
  out->brightness = static_cast<uint8_t>(f[5]);
  out->movingOnly = f[6] != 0;
  return true;
}

void ledConfigToFields(const LedConfig& c, uint32_t f[kLedConfigFields]) {
  f[0] = static_cast<uint32_t>(c.mode);
  f[1] = c.color1;
  f[2] = c.color2;
  f[3] = static_cast<uint32_t>(c.palette);
  f[4] = c.speed;
  f[5] = c.brightness;
  f[6] = c.movingOnly ? 1 : 0;
}

void LedClock::advance(const LedConfig& config, const LedMotion& motion, float dtS) {
  if (config.movingOnly && !motion.moving) return;  // frozen at rest
  // 0.2 .. 3 cycles per second from the speed setting.
  float rate = 0.2f + 2.8f * (static_cast<float>(config.speed) - 1.0f) / 99.0f;
  float direction = 1.0f;
  if (motion.moving) {
    // Faster driving = faster lights; spin follows the way the robot turns,
    // or runs backwards when reversing.
    const float effort = fmaxf(fabsf(motion.forward), fabsf(motion.turn));
    rate *= 0.5f + clamp01(effort);
    if (fabsf(motion.turn) > 0.15f) direction = motion.turn > 0.0f ? 1.0f : -1.0f;
    else if (motion.forward < 0.0f) direction = -1.0f;
  }
  phase += direction * rate * dtS;
  if (phase >= 1000.0f) phase -= 1000.0f;
  if (phase < 0.0f) phase += 1000.0f;
}

void renderLeds(const LedConfig& config, const LedMotion& motion, const LedClock& clock, LedRgb out[kLedRingSize]) {
  const LedRgb c1 = fromHex(config.color1), c2 = fromHex(config.color2);
  const bool active = motion.moving || !config.movingOnly;
  const float phase = clock.phase;

  for (int i = 0; i < kLedRingSize; ++i) {
    LedRgb c = {};
    switch (config.mode) {
      case LedMode::Solid:
        c = c1;
        break;
      case LedMode::Pulse:
        c = active ? scale(c1, 0.08f + 0.92f * (0.5f - 0.5f * cosf(kTwoPi * phase))) : scale(c1, kIdleLevel);
        break;
      case LedMode::Blink:
        c = active ? (frac(phase) < 0.5f ? c1 : c2) : scale(c1, kIdleLevel);
        break;
      case LedMode::SpinFade: {
        if (!active) {
          c = scale(c1, kIdleLevel);
          break;
        }
        // Distance behind the comet's head, 0..4 LEDs, measured against travel.
        const float behind = frac((phase * kLedRingSize - i) / kLedRingSize) * kLedRingSize;
        const float tail = clamp01(1.0f - behind / 2.5f);
        c = mix(scale(c2, 0.35f), c1, tail * tail);
        break;
      }
      case LedMode::SpinBlink:
        if (!active) c = scale(c1, kIdleLevel);
        else c = (static_cast<int>(frac(phase) * kLedRingSize) % kLedRingSize == i) ? c1 : c2;
        break;
      case LedMode::FrontBack: {
        const bool front = i < 2;
        float level = 0.6f;
        if (motion.moving && fabsf(motion.forward) > 0.05f) {
          const bool leading = (motion.forward > 0.0f) == front;
          level = leading ? 1.0f : 0.3f;
        }
        c = scale(front ? c1 : c2, level);
        break;
      }
      case LedMode::PaletteSweep:
        c = paletteColor(config, phase * 0.5f + i * 0.08f);
        if (!active) c = scale(c, 0.5f);
        break;
      case LedMode::Rainbow:
        c = hue(phase * 0.5f + static_cast<float>(i) / kLedRingSize);
        if (!active) c = scale(c, 0.5f);
        break;
      case LedMode::Status:
      case LedMode::Count:
        break;
    }
    out[i] = c;
  }
}
