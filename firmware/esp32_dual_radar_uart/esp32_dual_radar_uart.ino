/*
 * ESP32 dual blind-spot radar -> Jetson USB UART bridge.
 *
 * This example expects one DIGITAL DETECT output from each radar module.
 * Change pins and active level to match the exact radar datasheet.
 * USB Serial protocol: one JSON object per line at 115200 baud.
 */

#include <Arduino.h>

constexpr uint8_t LEFT_RADAR_PIN = 25;
constexpr uint8_t RIGHT_RADAR_PIN = 26;
constexpr bool RADAR_ACTIVE_HIGH = true;
constexpr uint32_t SEND_INTERVAL_MS = 50;  // 20 Hz
constexpr uint32_t DEBOUNCE_MS = 80;

struct DebouncedInput {
  uint8_t pin;
  bool stable = false;
  bool lastRaw = false;
  uint32_t changedAt = 0;

  bool update(uint32_t now) {
    bool raw = digitalRead(pin) == (RADAR_ACTIVE_HIGH ? HIGH : LOW);
    if (raw != lastRaw) {
      lastRaw = raw;
      changedAt = now;
    }
    if (now - changedAt >= DEBOUNCE_MS) stable = raw;
    return stable;
  }
};

DebouncedInput leftRadar{LEFT_RADAR_PIN};
DebouncedInput rightRadar{RIGHT_RADAR_PIN};
uint32_t sequenceNumber = 0;
uint32_t lastSend = 0;

void setup() {
  pinMode(LEFT_RADAR_PIN, INPUT_PULLDOWN);
  pinMode(RIGHT_RADAR_PIN, INPUT_PULLDOWN);
  Serial.begin(115200);
  delay(250);
}

void loop() {
  const uint32_t now = millis();
  const bool left = leftRadar.update(now);
  const bool right = rightRadar.update(now);
  if (now - lastSend < SEND_INTERVAL_MS) return;
  lastSend = now;

  Serial.print(F("{\"v\":1,\"seq\":"));
  Serial.print(sequenceNumber++);
  Serial.print(F(",\"left\":"));
  Serial.print(left ? F("true") : F("false"));
  Serial.print(F(",\"right\":"));
  Serial.print(right ? F("true") : F("false"));
  // Digital-output radar has presence only, so distance is explicitly null.
  Serial.println(F(",\"left_m\":null,\"right_m\":null}"));
}
