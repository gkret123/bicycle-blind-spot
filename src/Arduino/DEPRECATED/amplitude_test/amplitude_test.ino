#include <Wire.h>
#include "Adafruit_DRV2605.h"

Adafruit_DRV2605 drv;

// Experiment parameters
const int AMP_MIN = 0;
const int AMP_MAX = 127;     // DRV2605 RTP max
const float RAMP_DURATION = 10.0; // seconds
const int PULSE_ON_MS = 120;
const int PULSE_PERIOD_MS = 600;

void setup() {
  Serial.begin(115200);

  if (!drv.begin()) {
    Serial.println("DRV2605 not found");
    while (1);
  }

  drv.selectLibrary(1);
  drv.setMode(DRV2605_MODE_REALTIME);

  Serial.println("Starting amplitude ramp test...");
}

void loop() {
  unsigned long start = millis();

  while (true) {
    float elapsed = (millis() - start) / 1000.0;

    if (elapsed > RAMP_DURATION) {
      drv.setRealtimeValue(0);
      Serial.println("Done.");
      while (1);  // Stop after one run
    }

    // Linear ramp 0 → 127
    float scalar = elapsed / RAMP_DURATION;
    int amplitude = AMP_MIN + scalar * (AMP_MAX - AMP_MIN);

    amplitude = constrain(amplitude, 0, 127);

    Serial.print("Amplitude: ");
    Serial.println(amplitude);

    // Pulse ON
    drv.setRealtimeValue(amplitude);
    delay(PULSE_ON_MS);

    // Pulse OFF
    drv.setRealtimeValue(0);
    delay(PULSE_PERIOD_MS - PULSE_ON_MS);
  }
}
