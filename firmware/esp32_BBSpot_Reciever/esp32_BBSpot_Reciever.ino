#include <Arduino.h>
#include <Wire.h>
#include <NimBLEDevice.h>
#include <Adafruit_DRV2605.h>

// =====================
// BLE IDs (match Python)
// =====================
static const char* DEVICE_NAME         = "BBSpot-XIAO";
static const char* SERVICE_UUID        = "12345678-1234-1234-1234-1234567890ab";
static const char* CHAR_UUID_RX_WRITE  = "12345678-1234-1234-1234-1234567890ac";
static const char* CHAR_UUID_TX_NOTIFY = "12345678-1234-1234-1234-1234567890ad";

// ===================== 
// XIAO ESP32-C3 I2C pins
// =====================
static const int I2C_SDA = 6; // D4 = GPIO6
static const int I2C_SCL = 7; // D5 = GPIO7

// =====================
// DRV2605
// =====================
Adafruit_DRV2605 drv;
static bool drv_ok = false;

// =====================
// Haptic pattern knobs
// =====================
const int MAX_AMPLITUDE = 127;

// Your chosen envelope endpoints (more continuous as urgency rises)
const int ON_START_MS  = 60;
const int ON_END_MS    = 160;

const int OFF_START_MS = 900;
const int OFF_END_MS   = 10;

// Smoothing / thresholds
const float MERGE_START_URGENCY = 0.85f; // start collapsing OFF->0
const float CONTINUOUS_TTR      = 0.05f; // very small TTR => continuous
const float SILENT_TTR          = 0.98f; // very large TTR => idle/silent

// Watchdog: if Pi stops sending, go silent
const uint32_t TTR_TIMEOUT_MS = 500;

// Idle heartbeat while waiting for connection (tiny pulse every 30s)
const uint32_t IDLE_PULSE_PERIOD_MS = 30000;
const uint16_t IDLE_PULSE_ON_MS     = 60;
const uint8_t  IDLE_PULSE_AMP       = 40;

// =====================
// BLE globals
// =====================
static NimBLECharacteristic* g_txChar = nullptr;
static bool g_connected = false;

// Latest received TTR byte (0..255)
static volatile uint8_t g_ttr_byte = 255;
static volatile bool g_have_ttr = false;
static volatile uint32_t g_last_ttr_ms = 0;

static uint32_t g_last_idle_pulse_ms = 0;

// =====================
// Helpers
// =====================
static float clamp01(float x) {
  if (x < 0) return 0;
  if (x > 1) return 1;
  return x;
}

static float smoothstep(float x) {
  x = clamp01(x);
  return x * x * (3.0f - 2.0f * x);
}

static float easeInCubic(float x) {
  x = clamp01(x);
  return x * x * x;
}

static int lerpInt(int a, int b, float t) {
  t = clamp01(t);
  return a + (int)((b - a) * t + 0.5f);
}

static float ttrFromByte(uint8_t b) {
  return ((float)b) / 255.0f;
}

static void setRtp(uint8_t v) {
  if (!drv_ok) return;
  drv.setMode(DRV2605_MODE_REALTIME);
  drv.setRealtimeValue(v);
}

static void stopVibration() {
  setRtp(0);
  if (drv_ok) drv.stop();
}

static void notifyText(const char* msg) {
  if (!g_txChar || !g_connected) return;
  g_txChar->setValue(msg);
  g_txChar->notify();
}

// =====================
// BLE callbacks
// =====================
class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer*, NimBLEConnInfo&) override {
    g_connected = true;
    Serial.println("BLE: central connected");
    notifyText("ack:connected");
  }

  void onDisconnect(NimBLEServer* server, NimBLEConnInfo&, int reason) override {
    g_connected = false;
    stopVibration();
    Serial.print("BLE: central disconnected reason=");
    Serial.println(reason);
    server->startAdvertising();
  }
};

class RxCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* ch, NimBLEConnInfo&) override {
    std::string v = ch->getValue();

    // Expect 1 byte TTR
    if (v.size() == 1) {
      g_ttr_byte = (uint8_t)v[0];
      g_have_ttr = true;
      g_last_ttr_ms = millis();
      return;
    }

    // Optional debug commands:
    // "stop" -> silence
    if (v == "stop") {
      g_ttr_byte = 255;
      g_have_ttr = true;
      g_last_ttr_ms = millis();
      stopVibration();
      notifyText("ack:stop");
      return;
    }

    notifyText("err:bad_payload");
  }
};

void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println("BBSpot: TTR-driven haptics + idle heartbeat");

  // I2C / DRV init
  Wire.begin(I2C_SDA, I2C_SCL);
  drv_ok = drv.begin();
  if (!drv_ok) {
    Serial.println("DRV2605 NOT found on I2C. Check wiring/power.");
  } else {
    Serial.println("DRV2605 found");
    drv.selectLibrary(1);
    setRtp(0);
  }

  // BLE init
  NimBLEDevice::init(DEVICE_NAME);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  NimBLEServer* server = NimBLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());
  server->advertiseOnDisconnect(true);

  NimBLEService* service = server->createService(SERVICE_UUID);

  g_txChar = service->createCharacteristic(CHAR_UUID_TX_NOTIFY, NIMBLE_PROPERTY::NOTIFY);

  NimBLECharacteristic* rxChar = service->createCharacteristic(
    CHAR_UUID_RX_WRITE,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
  );
  rxChar->setCallbacks(new RxCallbacks());

  service->start();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  NimBLEAdvertisementData advData;
  advData.setName(DEVICE_NAME);
  advData.setFlags(0x06);
  advData.addServiceUUID(NimBLEUUID(SERVICE_UUID));
  adv->setAdvertisementData(advData);

  NimBLEAdvertisementData scanResp;
  scanResp.setName(DEVICE_NAME);
  adv->setScanResponseData(scanResp);

  adv->start();
  Serial.println("BLE: advertising started");
}

void loop() {
  // ---------------------------------------------------------
  // A) Not connected: stay silent except a tiny heartbeat pulse
  // ---------------------------------------------------------
  if (!g_connected) {
    stopVibration();

    uint32_t now = millis();
    if (now - g_last_idle_pulse_ms >= IDLE_PULSE_PERIOD_MS) {
      g_last_idle_pulse_ms = now;

      // tiny “alive” blip
      setRtp(IDLE_PULSE_AMP);
      delay(IDLE_PULSE_ON_MS);
      setRtp(0);

      Serial.println("Idle heartbeat pulse (waiting for BLE)");
    }

    delay(10);
    return;
  }

  // ---------------------------------------------------------
  // B) Connected but no TTR received yet: silent
  // ---------------------------------------------------------
  if (!g_have_ttr) {
    stopVibration();
    delay(10);
    return;
  }

  // ---------------------------------------------------------
  // C) Watchdog: if updates stop, silence
  // ---------------------------------------------------------
  if ((millis() - g_last_ttr_ms) > TTR_TIMEOUT_MS) {
    stopVibration();
    delay(10);
    return;
  }

  // ---------------------------------------------------------
  // D) Use TTR to generate urgency-driven pattern
  // ---------------------------------------------------------
  float ttr = clamp01(ttrFromByte(g_ttr_byte));

  // high TTR means idle (this is your “interrupt buzzing” behavior)
  if (ttr >= SILENT_TTR) {
    stopVibration();
    delay(10);
    return;
  }

  // urgency grows as TTR shrinks
  float u = 1.0f - ttr;
  float e = easeInCubic(u);

  // very small TTR -> continuous
  if (ttr <= CONTINUOUS_TTR) {
    setRtp(MAX_AMPLITUDE);
    delay(10);
    return;
  }

  int on_ms  = lerpInt(ON_START_MS,  ON_END_MS,  e);
  int off_ms = lerpInt(OFF_START_MS, OFF_END_MS, e);

  // smoothly collapse OFF -> 0 near the end (less abrupt)
  if (u >= MERGE_START_URGENCY) {
    float m = (u - MERGE_START_URGENCY) / (1.0f - MERGE_START_URGENCY); // 0..1
    m = smoothstep(m);
    off_ms = lerpInt(off_ms, 0, m);
  }

  // Pulse ON
  setRtp(MAX_AMPLITUDE);
  delay(on_ms);

  // Pulse OFF (may be 0 near end)
  if (off_ms > 0) {
    setRtp(0);
    delay(off_ms);
  }
}
