#include <Wire.h>
#include "Adafruit_DRV2605.h"

Adafruit_DRV2605 drv;

// ===== HAPTIC SETTINGS =====
const int MAX_AMPLITUDE = 127;   // DRV2605 RTP max

// Ramp pulse shape: ON increases, OFF decreases
const int ON_START_MS  = 60;     // early: short pulse
const int ON_END_MS    = 160;    // late: longer pulse

const int OFF_START_MS = 900;    // early: long gap
const int OFF_END_MS   = 10;     // late: short gap (still pulsing)

// Timing
const float RAMP_S     = 45.0;   // ramp duration (seconds)
const float HOLD_MAX_S = 10.0;   // continuous vibration duration at end

// Smooth curve: slow change early, faster late (approach feel)
static float easeInCubic(float x) {
  if (x < 0) x = 0;
  if (x > 1) x = 1;
  return x * x * x;
}

static int lerpInt(int a, int b, float t) {
  return a + (int)((b - a) * t + 0.5f);
}

void setup() {
  Serial.begin(115200);

  if (!drv.begin()) {
    Serial.println("DRV2605 not found");
    while (1);
  }

  drv.selectLibrary(1);
  drv.setMode(DRV2605_MODE_REALTIME);
  drv.setRealtimeValue(0);

  Serial.println("Ramp ON↑ + OFF↓, then continuous hold");
}

void loop() {
  const float TOTAL_S = RAMP_S + HOLD_MAX_S;
  unsigned long start_ms = millis();
  unsigned long lastPrint = 0;

  while (true) {
    float t_s = (millis() - start_ms) / 1000.0f;
    if (t_s >= TOTAL_S) break;

    // ===== Final phase: continuous vibration =====
    if (t_s >= RAMP_S) {
      drv.setRealtimeValue(MAX_AMPLITUDE);
      if (millis() - lastPrint >= 500) {
        lastPrint = millis();
        Serial.print("t=");
        Serial.print(t_s, 1);
        Serial.println("s  CONTINUOUS (HOLD)");
      }
      delay(10); // small loop delay; keeps serial prints sane
      continue;
    }

    // ===== Ramp phase: smoothly change ON and OFF =====
    float u = t_s / RAMP_S;       // 0..1
    float e = easeInCubic(u);     // eased 0..1

    int on_ms  = lerpInt(ON_START_MS,  ON_END_MS,  e);
    int off_ms = lerpInt(OFF_START_MS, OFF_END_MS, e);

    // ON
    drv.setRealtimeValue(MAX_AMPLITUDE);
    delay(on_ms);

    // OFF
    drv.setRealtimeValue(0);
    delay(off_ms);

    // Debug print ~2x/second
    if (millis() - lastPrint >= 500) {
      lastPrint = millis();
      Serial.print("t=");
      Serial.print(t_s, 1);
      Serial.print("s  ON=");
      Serial.print(on_ms);
      Serial.print("ms  OFF=");
      Serial.print(off_ms);
      Serial.println("ms");
    }
  }

  // Stop at end
  drv.setRealtimeValue(0);
  Serial.println("Done.");
  while (1);
}
