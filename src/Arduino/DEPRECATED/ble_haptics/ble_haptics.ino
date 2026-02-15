/*
 * Unified ESP32 BLE + DRV2605 haptics receiver for BBSpot.
 *
 * Decision: use DRV2605 built-in driver (I2C) with Real-Time Playback (RTP),
 * not ESP32 raw PWM. This keeps motor drive/protection/envelope handling in
 * the DRV2605 and gives better consistency across ERM/LRA hardware.
 *
 * BLE RX supports:
 *   1) Compact 4-byte packets from Pi: [intensity, pattern, duration_ticks_10ms, side]
 *      - intensity: 0..255 (mapped to DRV RTP 0..127)
 *      - pattern:   0=off, 1=amplitude pulse, 2=rate pulse (handled similarly on device)
 *      - duration_ticks_10ms: pulse duration in 10 ms increments
 *      - side: 0=both/center, 1=left, 2=right (included for protocol compatibility)
 *
 *   2) Text commands for debugging:
 *      ping | buzz | stop | effect:NN
 */

#include <Arduino.h>
#include <Wire.h>
#include <NimBLEDevice.h>
#include <Adafruit_DRV2605.h>

static const char* DEVICE_NAME = "BBSpot-XIAO";

// BLE UUIDs
static const char* SERVICE_UUID        = "12345678-1234-1234-1234-1234567890ab";
static const char* CHAR_UUID_RX_WRITE  = "12345678-1234-1234-1234-1234567890ac";
static const char* CHAR_UUID_TX_NOTIFY = "12345678-1234-1234-1234-1234567890ad";

// XIAO ESP32-C3 I2C pins
static const int I2C_SDA = 6; // D4 = GPIO6
static const int I2C_SCL = 7; // D5 = GPIO7

// Optional canned effect for quick bring-up
static uint8_t DEFAULT_EFFECT = 47;

enum Pattern : uint8_t {
  PATTERN_OFF = 0,
  PATTERN_AMP = 1,
  PATTERN_RATE = 2,
};

enum Side : uint8_t {
  SIDE_BOTH = 0,
  SIDE_LEFT = 1,
  SIDE_RIGHT = 2,
};

static NimBLECharacteristic* g_txChar = nullptr;
static bool g_connected = false;

// DRV2605 state
Adafruit_DRV2605 drv;
static bool drv_ok = false;
static uint32_t pulse_until_ms = 0;


static uint8_t piIntensityToDrvRtp(uint8_t pi_intensity) {
  // Pi sends 0..255; DRV RTP expects 0..127.
  return (uint8_t)((pi_intensity * 127UL) / 255UL);
}

static void notifyText(const char* msg) {
  if (!g_txChar || !g_connected) return;
  g_txChar->setValue(msg);
  g_txChar->notify();
}

static void stopVibration() {
  if (!drv_ok) return;
  drv.setRealtimeValue(0);
  drv.stop();
  pulse_until_ms = 0;
}

static void playEffect(uint8_t effect) {
  if (!drv_ok) {
    Serial.println("DRV2605 not detected; cannot vibrate");
    notifyText("err:drv_missing");
    return;
  }

  // Temporarily use internal trigger mode for library effect playback.
  drv.setMode(DRV2605_MODE_INTTRIG);
  drv.setWaveform(0, effect);
  drv.setWaveform(1, 0);
  drv.go();
}

static void startRealtimePulse(uint8_t pi_intensity, uint32_t duration_ms) {
  if (!drv_ok) {
    notifyText("err:drv_missing");
    return;
  }

  // Ensure RTP mode for direct amplitude control.
  drv.setMode(DRV2605_MODE_REALTIME);

  const uint8_t rtp = piIntensityToDrvRtp(pi_intensity);
  drv.setRealtimeValue(rtp);
  pulse_until_ms = millis() + duration_ms;
}

class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* server, NimBLEConnInfo& connInfo) override {
    (void)server;
    (void)connInfo;
    g_connected = true;
    Serial.println("BLE: central connected");
  }

  void onDisconnect(NimBLEServer* server, NimBLEConnInfo& connInfo, int reason) override {
    (void)connInfo;
    g_connected = false;
    stopVibration();

    Serial.print("BLE: central disconnected, reason=");
    Serial.println(reason);

    server->startAdvertising();
    Serial.println("BLE: advertising restarted");
  }
};

class RxCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* characteristic, NimBLEConnInfo& connInfo) override {
    (void)connInfo;

    std::string val = characteristic->getValue();

    // Compact binary command path: [intensity, pattern, duration_10ms, side]
    if (val.size() == 4) {
      const uint8_t intensity = (uint8_t)val[0];
      const uint8_t pattern = (uint8_t)val[1];
      const uint8_t duration_ticks = (uint8_t)val[2];
      const uint8_t side = (uint8_t)val[3];

      const uint32_t duration_ms = (uint32_t)duration_ticks * 10UL;

      if (pattern == PATTERN_OFF || intensity == 0 || duration_ms == 0) {
        stopVibration();
      } else {
        // Current board controls a single DRV channel; side is accepted/proxied.
        (void)side;
        startRealtimePulse(intensity, duration_ms);
      }

      char ack[96];
      snprintf(ack, sizeof(ack), "rx:i=%u p=%u d=%lums s=%u", intensity, pattern, duration_ms, side);
      notifyText(ack);
      return;
    }

    // Text/debug command path
    Serial.print("BLE RX text: ");
    Serial.println(val.c_str());

    if (val == "ping") {
      notifyText("pong");
      return;
    }

    if (val == "buzz") {
      playEffect(DEFAULT_EFFECT);
      notifyText("ack:buzz");
      return;
    }

    if (val == "stop") {
      stopVibration();
      notifyText("ack:stop");
      return;
    }

    const std::string prefix = "effect:";
    if (val.rfind(prefix, 0) == 0) {
      int n = atoi(val.c_str() + prefix.size());
      if (n < 1 || n > 127) {
        notifyText("err:bad_effect");
        return;
      }
      playEffect((uint8_t)n);
      notifyText("ack:effect");
      return;
    }

    notifyText("err:unknown_cmd");
  }
};

void setup() {
  Serial.begin(115200);
  delay(1200);

  Serial.println("Boot: BLE + DRV2605 unified receiver");

  // I2C + DRV2605 init
  Wire.begin(I2C_SDA, I2C_SCL);

  drv_ok = drv.begin();
  if (!drv_ok) {
    Serial.println("DRV2605 NOT found on I2C 0x5A. Check wiring/power.");
  } else {
    Serial.println("DRV2605 found");
    drv.selectLibrary(1);
    drv.setMode(DRV2605_MODE_REALTIME);
    drv.setRealtimeValue(0);
  }

  // BLE init
  NimBLEDevice::init(DEVICE_NAME);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  NimBLEServer* server = NimBLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());
  server->advertiseOnDisconnect(true);

  NimBLEService* service = server->createService(SERVICE_UUID);

  g_txChar = service->createCharacteristic(
    CHAR_UUID_TX_NOTIFY,
    NIMBLE_PROPERTY::NOTIFY
  );

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
  Serial.println("Commands: compact-4B, ping, buzz, effect:47, stop");
}

void loop() {
  if (pulse_until_ms > 0 && millis() >= pulse_until_ms) {
    stopVibration();
  }
  delay(2);
}
