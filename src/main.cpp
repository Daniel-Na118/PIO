#include <Arduino.h>
#include <Adafruit_TinyUSB.h>

// Compile and flash separately at 400 and 800 Hz.
#ifndef SAMPLE_RATE_HZ
#define SAMPLE_RATE_HZ 400
#endif

constexpr uint8_t SENSOR_PIN = A1;
constexpr uint32_t PERIOD_US = 1000000UL / SAMPLE_RATE_HZ;
constexpr uint32_t CAPTURE_SECONDS = 10;
constexpr uint32_t SAMPLE_COUNT = SAMPLE_RATE_HZ * CAPTURE_SECONDS;
static_assert(SAMPLE_RATE_HZ == 400 || SAMPLE_RATE_HZ == 800,
              "This build supports the 400/800 Hz control only");

struct Sample {
  uint32_t timestamp_us;
  uint16_t adc;
};

// 64 kB at 800 Hz with typical 8-byte struct alignment. No USB prints or
// BLE activity while capturing.
Sample samples[SAMPLE_COUNT];

uint16_t read_sensor_adc() {
  (void)analogRead(SENSOR_PIN);
  delayMicroseconds(20);
  return static_cast<uint16_t>(analogRead(SENSOR_PIN));
}

void capture_and_send() {
  Serial.println("CAPTURE_BEGIN");
  Serial.flush();
  uint32_t next_us = micros() + PERIOD_US;
  uint32_t missed_slots = 0;
  for (uint32_t i = 0; i < SAMPLE_COUNT; ++i) {
    while (static_cast<int32_t>(micros() - next_us) < 0) {
      // Remain on the same sampling schedule across conversions.
    }
    uint32_t now = micros();
    if (now - next_us >= PERIOD_US) {
      uint32_t skipped = (now - next_us) / PERIOD_US;
      missed_slots += skipped;
      next_us += skipped * PERIOD_US;
    }
    samples[i].timestamp_us = micros();
    samples[i].adc = read_sensor_adc();
    next_us += PERIOD_US;
  }

  Serial.print("DATA,");
  Serial.print(SAMPLE_RATE_HZ);
  Serial.print(',');
  Serial.print(SAMPLE_COUNT);
  Serial.print(',');
  Serial.println(missed_slots);
  Serial.println("sample_index,device_time_us,adc_raw");
  for (uint32_t i = 0; i < SAMPLE_COUNT; ++i) {
    Serial.print(i);
    Serial.print(',');
    Serial.print(samples[i].timestamp_us);
    Serial.print(',');
    Serial.println(samples[i].adc);
  }
  Serial.println("END");
  Serial.flush();
}

void setup() {
  pinMode(SENSOR_PIN, INPUT);
  analogReadResolution(12);
  Serial.begin(115200);
}

void loop() {
  // START\n from the PC begins a capture. Incoming USB data is ignored during
  // capture, and the captured samples are sent only when acquisition ends.
  if (Serial.available() && Serial.read() == 'S') {
    while (Serial.available()) Serial.read();
    capture_and_send();
  }
}
