#include <Arduino.h>
#include <M5Unified.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <math.h>

#if __has_include("secrets.h")
#include "secrets.h"
#endif

namespace {

constexpr uint8_t HALL_PIN = 2;
constexpr uint32_t SERIAL_BAUD = 115200;

#ifndef DEVICE_AP_MODE
#define DEVICE_AP_MODE 1
#endif
#ifndef DEVICE_IP_LAST_OCTET
#define DEVICE_IP_LAST_OCTET 1
#endif
#ifndef OSC_ADDRESS_PREFIX
#define OSC_ADDRESS_PREFIX "/bike1"
#endif

constexpr bool WIFI_AP_MODE = DEVICE_AP_MODE != 0;
constexpr const char* AP_SSID = "katakishi-bike-sensor";
constexpr const char* AP_PASSWORD = "bike-sensor";
constexpr const char* OSC_PREFIX = OSC_ADDRESS_PREFIX;
const IPAddress DEVICE_IP(192, 168, 4, DEVICE_IP_LAST_OCTET);
const IPAddress WIFI_GATEWAY(192, 168, 4, 1);
const IPAddress WIFI_SUBNET(255, 255, 255, 0);
const IPAddress OSC_HOST(192, 168, 4, 10);
constexpr uint16_t OSC_PORT = 9000;
constexpr uint32_t WIFI_RECONNECT_INTERVAL_MS = 5000;

constexpr uint32_t IMU_INTERVAL_MS = 20;
constexpr uint32_t HALL_INTERVAL_MS = 2;
constexpr uint32_t SPEED_INTERVAL_MS = 1500;//ホールセンサー計測レート
constexpr uint32_t SERIAL_INTERVAL_MS = 50;
constexpr uint32_t OSC_INTERVAL_MS = 50;
constexpr uint32_t DISPLAY_INTERVAL_MS = 80;

constexpr float WHEEL_CIRCUMFERENCE_M = 2.0f;//タイヤ周長2m
constexpr float MPS_TO_KMH = 3.6f;
constexpr uint8_t HALL_INPUT_MODE = INPUT_PULLUP;
constexpr uint8_t HALL_ACTIVE_LEVEL = HIGH;
constexpr uint32_t HALL_DEBOUNCE_MS = 0.1;//ノイズ対策(ここを増やすと加速時に速度取得ができなくなる可能性あり)

constexpr float LEAN_CENTER_DEG = 5.0f;
constexpr float LEAN_SIDE_DEG = 10.0f;

struct SensorState {
  float roll = 0.0f;
  float pitch = 0.0f;
  int leanState = 0;
  float speedKmh = 0.0f;
  uint32_t pulseCount = 0;
  uint32_t speedWindowPulses = 0;
  uint8_t hallState = 0;
  bool hallActive = false;
  uint32_t updatedAtMs = 0;
};

//ハンドル操舵
class ImuReader {
 public:
  bool begin() {
    available_ = M5.Imu.isEnabled();
    return available_;
  }

  void update(SensorState& state) {
    if (!available_) {
      return;
    }

    if (!M5.Imu.update()) {
      return;
    }

    m5::imu_data_t data;
    M5.Imu.getImuData(&data);

    const float ax = data.accel.x;
    const float ay = data.accel.y;
    const float az = data.accel.z;
    const float frontBack = atan2f(ay, az) * 180.0f / PI;
    const float leftRight = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / PI;

    state.roll = leftRight;
    state.pitch = frontBack;
    state.leanState = classifyLean(leftRight, state.leanState);
    state.updatedAtMs = millis();
  }

 private:
  static int classifyLean(float roll, int current) {
    if (current == 0) {
      if (roll >= LEAN_SIDE_DEG) {
        return 1;
      }
      if (roll <= -LEAN_SIDE_DEG) {
        return -1;
      }
      return 0;
    }

    if (fabsf(roll) <= LEAN_CENTER_DEG) {
      return 0;
    }
    if (roll >= LEAN_SIDE_DEG) {
      return 1;
    }
    if (roll <= -LEAN_SIDE_DEG) {
      return -1;
    }
    return current;
  }

  bool available_ = false;
};

//車輪回転数
class WheelSensor {
 public:
  void begin() {
    pinMode(HALL_PIN, HALL_INPUT_MODE);
    const bool rawActive = digitalRead(HALL_PIN) == HALL_ACTIVE_LEVEL;
    candidateActive_ = rawActive;
    debouncedActive_ = rawActive;
    candidateChangedAtMs_ = millis();
    lastSpeedAtMs_ = millis();
  }

  void sample(SensorState& state) {
    const uint32_t now = millis();
    const bool rawActive = digitalRead(HALL_PIN) == HALL_ACTIVE_LEVEL;

    if (rawActive != candidateActive_) {
      candidateActive_ = rawActive;
      candidateChangedAtMs_ = now;
    }

    if (candidateActive_ != debouncedActive_ && now - candidateChangedAtMs_ >= HALL_DEBOUNCE_MS) {
      const bool wasActive = debouncedActive_;
      debouncedActive_ = candidateActive_;

      if (debouncedActive_ && !wasActive) {
        ++totalPulseCount_;
        ++windowPulseCount_;
      }
    }

    state.hallState = debouncedActive_ ? 1 : 0;
    state.hallActive = debouncedActive_;
    state.pulseCount = totalPulseCount_;
    state.speedWindowPulses = windowPulseCount_;
  }

  void updateSpeed(SensorState& state) {
    const uint32_t now = millis();
    const uint32_t elapsed = now - lastSpeedAtMs_;
    if (elapsed == 0) {
      return;
    }

    const uint32_t pulseDelta = windowPulseCount_;
    const float elapsedSeconds = static_cast<float>(elapsed) / 1000.0f;
    const float metersPerSecond = static_cast<float>(pulseDelta) * WHEEL_CIRCUMFERENCE_M / elapsedSeconds;
    state.speedKmh = metersPerSecond * MPS_TO_KMH;
    state.pulseCount = totalPulseCount_;
    state.speedWindowPulses = pulseDelta;

    windowPulseCount_ = 0;
    lastSpeedAtMs_ = now;
  }

 private:
  bool candidateActive_ = false;
  bool debouncedActive_ = false;
  uint32_t totalPulseCount_ = 0;
  uint32_t windowPulseCount_ = 0;
  uint32_t lastSpeedAtMs_ = 0;
  uint32_t candidateChangedAtMs_ = 0;
};

class SerialReporter {
 public:
  void begin() {
    Serial.begin(SERIAL_BAUD);
    delay(200);
    Serial.println("roll,lean_state,speed_kmh,speed_window_pulses,hall_state,hall_active");
  }

  void update(const SensorState& state) {
    Serial.printf("%.2f,%d,%.2f,%lu,%u,%d\n",
                  state.roll,
                  state.leanState,
                  state.speedKmh,
                  static_cast<unsigned long>(state.speedWindowPulses),
                  state.hallState,
                  state.hallActive ? 1 : 0);
  }
};

class OscReporter {
 public:
  void begin() {
    if (WIFI_AP_MODE) {
      startAccessPointMode();
    } else {
      startStationMode();
    }
    udp_.begin(0);
  }

  void update(const SensorState& state) {
    if (!ready()) {
      return;
    }

    sendTilt(state);
    sendWheel(state);
  }

 private:
  void startStationMode() {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.config(DEVICE_IP, WIFI_GATEWAY, WIFI_SUBNET);
    WiFi.begin(AP_SSID, AP_PASSWORD);
    lastWifiAttemptMs_ = millis();
  }

  void startAccessPointMode() {
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(DEVICE_IP, WIFI_GATEWAY, WIFI_SUBNET);
    WiFi.softAP(AP_SSID, AP_PASSWORD);
  }

  bool ready() {
    if (WIFI_AP_MODE) {
      return WiFi.softAPgetStationNum() > 0;
    }

    if (WiFi.status() == WL_CONNECTED) {
      printLocalIpOnce();
      return true;
    }

    wifiConnectedPrinted_ = false;
    const uint32_t now = millis();
    if (now - lastWifiAttemptMs_ >= WIFI_RECONNECT_INTERVAL_MS) {
      WiFi.disconnect();
      WiFi.config(DEVICE_IP, WIFI_GATEWAY, WIFI_SUBNET);
      WiFi.begin(AP_SSID, AP_PASSWORD);
      lastWifiAttemptMs_ = now;
    }
    return false;
  }

  void printLocalIpOnce() {
    if (wifiConnectedPrinted_) {
      return;
    }

    Serial.printf("WiFi connected ssid=%s local_ip=%s osc_target=%s:%u prefix=%s\n",
                  AP_SSID,
                  WiFi.localIP().toString().c_str(),
                  OSC_HOST.toString().c_str(),
                  OSC_PORT,
                  OSC_PREFIX);
    wifiConnectedPrinted_ = true;
  }

  static void appendPaddedString(uint8_t* buffer, size_t& size, const char* value) {
    while (*value != '\0') {
      buffer[size++] = static_cast<uint8_t>(*value++);
    }
    buffer[size++] = 0;
    while ((size % 4) != 0) {
      buffer[size++] = 0;
    }
  }

  static void appendU32(uint8_t* buffer, size_t& size, uint32_t value) {
    buffer[size++] = static_cast<uint8_t>((value >> 24) & 0xFF);
    buffer[size++] = static_cast<uint8_t>((value >> 16) & 0xFF);
    buffer[size++] = static_cast<uint8_t>((value >> 8) & 0xFF);
    buffer[size++] = static_cast<uint8_t>(value & 0xFF);
  }

  static void appendInt32(uint8_t* buffer, size_t& size, int32_t value) {
    appendU32(buffer, size, static_cast<uint32_t>(value));
  }

  static void appendFloat(uint8_t* buffer, size_t& size, float value) {
    uint32_t raw = 0;
    static_assert(sizeof(raw) == sizeof(value), "float must be 32-bit");
    memcpy(&raw, &value, sizeof(raw));
    appendU32(buffer, size, raw);
  }

  void sendTilt(const SensorState& state) {
    uint8_t buffer[64];
    size_t size = 0;
    char address[24];
    snprintf(address, sizeof(address), "%s/tilt", OSC_PREFIX);
    appendPaddedString(buffer, size, address);
    appendPaddedString(buffer, size, ",fi");
    appendFloat(buffer, size, state.roll);
    appendInt32(buffer, size, state.leanState);
    send(buffer, size);
  }

  void sendWheel(const SensorState& state) {
    uint8_t buffer[64];
    size_t size = 0;
    char address[24];
    snprintf(address, sizeof(address), "%s/wheel", OSC_PREFIX);
    appendPaddedString(buffer, size, address);
    appendPaddedString(buffer, size, ",fi");
    appendFloat(buffer, size, state.speedKmh);
    appendInt32(buffer, size, state.hallState);
    send(buffer, size);
  }

  void send(const uint8_t* buffer, size_t size) {
    udp_.beginPacket(OSC_HOST, OSC_PORT);
    udp_.write(buffer, size);
    udp_.endPacket();
  }

  WiFiUDP udp_;
  uint32_t lastWifiAttemptMs_ = 0;
  bool wifiConnectedPrinted_ = false;
};

class DisplayView {
 public:
  void begin() {
    auto& display = M5.Display;
    display.setRotation(0);
    display.setTextDatum(top_left);
    display.setTextSize(1);
    display.fillScreen(TFT_BLACK);
  }

  void update(const SensorState& state) {
    auto& display = M5.Display;
    const int width = display.width();
    const int height = display.height();
    const int centerX = width / 2;

    display.startWrite();
    display.fillScreen(TFT_BLACK);

    display.setTextColor(TFT_WHITE, TFT_BLACK);
    display.setCursor(4, 4);
    display.printf("ROLL %5.1f", state.roll);
    display.setCursor(4, 16);
    display.printf("KMH %6.1f", state.speedKmh);
    display.setCursor(4, 28);
    display.printf("HALL %1u", state.hallState);
    drawDeviceIp(display);

    const int horizonY = 70;
    display.drawLine(12, horizonY, width - 12, horizonY, TFT_DARKGREY);
    display.drawLine(centerX, horizonY - 22, centerX, horizonY + 22, TFT_DARKGREY);

    const float clampedRoll = constrain(state.roll, -35.0f, 35.0f);
    const int bubbleX = centerX + static_cast<int>((clampedRoll / 35.0f) * (width / 2 - 18));
    const uint16_t leanColor = state.leanState == 0 ? TFT_GREEN : TFT_ORANGE;
    display.fillCircle(bubbleX, horizonY, 7, leanColor);
    display.drawCircle(centerX, horizonY, 12, TFT_DARKGREY);

    const char* leanLabel = state.leanState < 0 ? "LEFT" : (state.leanState > 0 ? "RIGHT" : "CENTER");
    display.setTextDatum(middle_center);
    display.setTextColor(leanColor, TFT_BLACK);
    display.drawString(leanLabel, centerX, 96);
    display.setTextDatum(top_left);

    const int barX = 8;
    const int barY = height - 18;
    const int barW = width - 16;
    const int barH = 9;
    const int speedFill = constrain(static_cast<int>((state.speedKmh / 60.0f) * barW), 0, barW);
    display.drawRect(barX, barY, barW, barH, TFT_DARKGREY);
    display.fillRect(barX + 1, barY + 1, max(0, speedFill - 2), barH - 2, TFT_CYAN);

    if ((!WIFI_AP_MODE && WiFi.status() == WL_CONNECTED) ||
        (WIFI_AP_MODE && WiFi.softAPgetStationNum() > 0)) {
      display.fillCircle(width - 7, 7, 3, TFT_BLUE);
    }

    display.endWrite();
  }

 private:
  static IPAddress currentDeviceIp() {
    if (WIFI_AP_MODE) {
      return WiFi.softAPIP();
    }
    if (WiFi.status() == WL_CONNECTED) {
      return WiFi.localIP();
    }
    return DEVICE_IP;
  }

  static void drawDeviceIp(m5gfx::M5GFX& display) {
    const IPAddress ip = currentDeviceIp();
    display.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    display.setCursor(4, 40);
    display.printf("IP %u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
  }
};

SensorState sensorState;
ImuReader imuReader;
WheelSensor wheelSensor;
SerialReporter serialReporter;
OscReporter oscReporter;
DisplayView displayView;

uint32_t lastImuMs = 0;
uint32_t lastHallMs = 0;
uint32_t lastSpeedMs = 0;
uint32_t lastSerialMs = 0;
uint32_t lastOscMs = 0;
uint32_t lastDisplayMs = 0;

bool shouldRun(uint32_t& lastMs, uint32_t intervalMs, uint32_t now) {
  if (now - lastMs < intervalMs) {
    return false;
  }
  lastMs = now;
  return true;
}

}  // namespace

void setup() {
  auto config = M5.config();
  config.serial_baudrate = SERIAL_BAUD;
  M5.begin(config);

  serialReporter.begin();
  imuReader.begin();
  wheelSensor.begin();
  oscReporter.begin();
  displayView.begin();

  if (WIFI_AP_MODE) {
    Serial.printf("SoftAP SSID=%s password=%s ip=%s osc=%s:%u prefix=%s\n",
                  AP_SSID,
                  AP_PASSWORD,
                  WiFi.softAPIP().toString().c_str(),
                  OSC_HOST.toString().c_str(),
                  OSC_PORT,
                  OSC_PREFIX);
  } else {
    Serial.printf("WiFi STA static_ip=%s ssid=%s osc_target=%s:%u prefix=%s\n",
                  DEVICE_IP.toString().c_str(),
                  AP_SSID,
                  OSC_HOST.toString().c_str(),
                  OSC_PORT,
                  OSC_PREFIX);
  }
}

void loop() {
  M5.update();
  const uint32_t now = millis();

  if (shouldRun(lastHallMs, HALL_INTERVAL_MS, now)) {
    wheelSensor.sample(sensorState);
  }
  if (shouldRun(lastSpeedMs, SPEED_INTERVAL_MS, now)) {
    wheelSensor.updateSpeed(sensorState);
  }
  if (shouldRun(lastImuMs, IMU_INTERVAL_MS, now)) {
    imuReader.update(sensorState);
  }
  if (shouldRun(lastSerialMs, SERIAL_INTERVAL_MS, now)) {
    serialReporter.update(sensorState);
  }
  if (shouldRun(lastOscMs, OSC_INTERVAL_MS, now)) {
    oscReporter.update(sensorState);
  }
  if (shouldRun(lastDisplayMs, DISPLAY_INTERVAL_MS, now)) {
    displayView.update(sensorState);
  }
}
