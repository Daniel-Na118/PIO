#include <Arduino.h>
#include <bluefruit.h>

const int SENSOR_PIN = A1;

const uint16_t SAMPLE_RATE_HZ = 500;
const uint32_t SAMPLE_INTERVAL_US =
    1000000UL / SAMPLE_RATE_HZ;

const uint8_t SAMPLES_PER_PACKET = 80;

// Packet size:
// 2 bytes sequence
// 4 bytes timestamp
// 80 * 2 bytes ADC
// = 166 bytes
struct __attribute__((packed)) RawPacket {
    uint16_t sequence;
    uint32_t timestamp_us;
    uint16_t samples[SAMPLES_PER_PACKET];
};

BLEService sensorService(
    "7e400001-b5a3-f393-e0a9-e50e24dcca9e"
);

BLECharacteristic rawCharacteristic(
    "7e400003-b5a3-f393-e0a9-e50e24dcca9e"
);

RawPacket packet;

uint8_t packetIndex = 0;
uint16_t sequenceNumber = 0;

uint32_t nextSampleUs = 0;

uint32_t samplesThisSecond = 0;
uint32_t sentPackets = 0;
uint32_t failedPackets = 0;
uint32_t lastStatsMs = 0;


void setupBLE() {

    Bluefruit.configPrphConn(
        247,
        6,
        8,
        1
    );

    Bluefruit.begin();

    Bluefruit.setName("GlassesML");

    sensorService.begin();

    rawCharacteristic.setProperties(
        CHR_PROPS_READ | CHR_PROPS_NOTIFY
    );

    rawCharacteristic.setPermission(
        SECMODE_OPEN,
        SECMODE_NO_ACCESS
    );

    rawCharacteristic.setFixedLen(
        sizeof(RawPacket)
    );

    rawCharacteristic.begin();

    Bluefruit.Advertising.addService(sensorService);
    Bluefruit.Advertising.addName();

    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.start(0);
}


void sendPacket() {

    if (!Bluefruit.connected())
        return;

    packet.sequence = sequenceNumber;

    bool success =
        rawCharacteristic.notify(
            (uint8_t *)&packet,
            sizeof(packet)
        );

    if (success) {
        sequenceNumber++;
        sentPackets++;
    } else {
        failedPackets++;
    }
}


void acquireSample() {

    if (packetIndex == 0) {
        packet.timestamp_us = micros();
    }

    packet.samples[packetIndex] =
        analogRead(SENSOR_PIN);

    packetIndex++;
    samplesThisSecond++;

    if (packetIndex >= SAMPLES_PER_PACKET) {

        sendPacket();

        packetIndex = 0;
    }
}


void setup() {

    Serial.begin(115200);
    delay(1500);

    pinMode(SENSOR_PIN, INPUT);

    setupBLE();

    nextSampleUs =
        micros() + SAMPLE_INTERVAL_US;

    Serial.println();
    Serial.println("============================");
    Serial.println("SINGLE-EAR ML SENSOR");
    Serial.println("============================");
    Serial.println("A0 = LEFT EAR");
    Serial.println("Sample rate = 500 Hz");
}


void loop() {

    uint32_t nowUs = micros();

    if ((int32_t)(nowUs - nextSampleUs) >= 0) {

        acquireSample();

        nextSampleUs += SAMPLE_INTERVAL_US;

        if ((int32_t)(micros() - nextSampleUs) >
            (int32_t)(SAMPLE_INTERVAL_US * 5)) {

            nextSampleUs =
                micros() + SAMPLE_INTERVAL_US;
        }
    }


    uint32_t nowMs = millis();

    if (nowMs - lastStatsMs >= 1000) {

        lastStatsMs += 1000;

        Serial.print("BLE=");
        Serial.print(
            Bluefruit.connected()
            ? "YES"
            : "NO"
        );

        Serial.print(" samples/s=");
        Serial.print(samplesThisSecond);

        Serial.print(" sent=");
        Serial.print(sentPackets);

        Serial.print(" failed=");
        Serial.println(failedPackets);

        samplesThisSecond = 0;
    }
}