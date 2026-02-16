// testing/bbspot_pulse_ttr/bbspot_pulse_ttr.ino
#include <Arduino.h>
#include <Wire.h>
#include <NimBLEDevice.h>
#include <Adafruit_DRV2605.h>

static const char* DEVICE_NAME = "BBSpot-XIAO";

// BLE UUIDs
static const char* SERVICE_UUID       = "12345678-1234-1234-1234-1234567890ab";
static const char* CHAR_UUID_RX_WRITE = "12345678-1234-1234-1234-1234567890ac";
static const char* CHAR_UUID_TX_NOTIFY= "12345678-1234-1234-1234-1234567890ad";

// XIAO ESP32-C3 I2C pins
static const int I2C_SDA = 6; // D4 = GPIO6
static const int I2C_SCL = 7; // D5 = GPIO7

Adafruit_DRV2605 drv;
static bool drv_ok = false;

static volatile uint8_t g_ttr_byte = 255; // 255 => TTR=1.0 (calm)
static NimBLECharacteristic* g_txChar = nullptr;
static bool g_connected = false;

// ===== Pattern knobs (your chosen endpoints) =====
const int MAX_AMPLITUDE = 127;

const int ON_START_MS  = 60;   // at TTR=1.0
const int ON_END_MS    = 160;  // at TTR~0.0

const int OFF_START_MS = 900;  // at TTR=1.0
const int OFF_END_MS   = 10;   // at TTR~0.0 (near continuous)

// Smoothly begin collapsing OFF to 0 when urgency is high
const float MERGE_START_URGENCY = 0.85f; // u >= 0.85 => OFF collapses to 0 smoothly

// Full continuous threshold
const float CONTINUOUS_TTR = 0.05f; // TTR <= 0.05 => continuous

// ===== Helpers =====
static float clamp01(float x) { return (x < 0) ? 0 : (x > 1) ? 1 : x; }

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

static float ttrFromByte(uint8_t b) { return ((float)b) / 255.0f; }

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

// ===== BLE callbacks =====
class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer*, NimBLEConnInfo&) override {
    g_connected = true;
    Serial.println("BLE: connected");
  }
  void onDisconnect(NimBLEServer* server, NimBLEConnInfo&, int reason) override {
    g_connected = false;
    Serial.print("BLE: disconnected reason=");
    Serial.println(reason);
    stopVibration();
    server->startAdvertising();
  }
};

class RxCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* ch, NimBLEConnInfo&) override {
    std::string v = ch->getValue();

    // Expect 1 byte: TTR scalar 0..255
    if (v.size() == 1) {
      g_ttr_byte = (uint8_t)v[0];
      return;
    }

    // Optional debug: "ttr:NN"
    if (v.rfind("ttr:", 0) == 0) {
      int n = atoi(v.c_str() + 4);
      if (n < 0) n = 0;
      if (n > 255) n = 255;
      g_ttr_byte = (uint8_t)n;
      return;
    }

    notifyText("err:bad_payload");
  }
};

void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println("BBSpot: DRV2605 pattern driven by TTR byte over BLE");

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
  // TTR scalar
  float ttr = clamp01(ttrFromByte(g_ttr_byte));

  // Urgency grows as TTR shrinks
  float u = 1.0f - ttr;

  // Nonlinear response (feels more “approach-like”)
  float e = easeInCubic(u);

  // Fully continuous when TTR is tiny
  if (ttr <= CONTINUOUS_TTR) {
    setRtp(MAX_AMPLITUDE);
    delay(10);
    return;
  }

  // Pulse shaping
  int on_ms  = lerpInt(ON_START_MS,  ON_END_MS,  e);
  int off_ms = lerpInt(OFF_START_MS, OFF_END_MS, e);

  // Smoothly collapse OFF to 0 as urgency approaches 1
  if (u >= MERGE_START_URGENCY) {
    float m = (u - MERGE_START_URGENCY) / (1.0f - MERGE_START_URGENCY); // 0..1
    m = smoothstep(m);
    off_ms = lerpInt(off_ms, 0, m);
  }

  // Pulse ON
  setRtp(MAX_AMPLITUDE);
  delay(on_ms);

  // Pulse OFF (may go to 0 near end)
  if (off_ms > 0) {
    setRtp(0);
    delay(off_ms);
  }
}
