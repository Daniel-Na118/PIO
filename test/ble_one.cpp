#include <Arduino.h>
#include <bluefruit.h>

// ============================================================
// CONFIGURATION
// ============================================================

const int SENSOR_PIN = A0;

const uint16_t SAMPLE_RATE_HZ = 500;
const uint32_t SAMPLE_INTERVAL_US =
    1000000UL / SAMPLE_RATE_HZ;

const uint8_t SAMPLES_PER_PACKET = 50;

// 2 sequence
// 4 timestamp
// 50 * 2 ADC
//
// Total = 106 bytes
struct __attribute__((packed)) RawPacket {
    uint16_t sequence;
    uint32_t timestamp_us;
    uint16_t samples[SAMPLES_PER_PACKET];
};

RawPacket packet;

uint8_t packetIndex = 0;
uint16_t sequenceNumber = 0;

// ============================================================
// BLE
// ============================================================

BLEService sensorService(
    "7e400001-b5a3-f393-e0a9-e50e24dcca9e"
);

BLECharacteristic rawCharacteristic(
    "7e400003-b5a3-f393-e0a9-e50e24dcca9e"
);

// ============================================================
// TIMING / STATS
// ============================================================

uint32_t nextSampleUs = 0;

uint32_t totalSamples = 0;

uint32_t attemptedPackets = 0;
uint32_t successfulPackets = 0;
uint32_t failedPackets = 0;

uint32_t statsSamples = 0;
uint32_t lastStatsMs = 0;


// ============================================================
// BLE SETUP
// ============================================================

void setupBLE() {

    /*
     * IMPORTANT:
     * configure before Bluefruit.begin()
     *
     * MTU = 247
     * larger BLE event
     * notification queue = 8
     */
    Bluefruit.configPrphConn(
        247,
        6,
        8,
        1
    );

    Bluefruit.begin();

    Bluefruit.setName("BodyMotionRaw");

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

    Bluefruit.Advertising.setInterval(
        32,
        244
    );

    Bluefruit.Advertising.start(0);
}


// ============================================================
// SEND PACKET
// ============================================================

void sendPacket() {

    if (!Bluefruit.connected()) {
        return;
    }

    packet.sequence = sequenceNumber;

    attemptedPackets++;

    bool success = rawCharacteristic.notify(
        (uint8_t*)&packet,
        sizeof(packet)
    );

    /*
     * Increment sequence ONLY if BLE actually accepted
     * the packet.
     */
    if (success) {

        successfulPackets++;
        sequenceNumber++;

    } else {

        failedPackets++;
    }
}


// ============================================================
// ACQUIRE SAMPLE
// ============================================================

void acquireSample() {

    if (packetIndex == 0) {
        packet.timestamp_us = micros();
    }

    packet.samples[packetIndex] =
        analogRead(SENSOR_PIN);

    packetIndex++;

    totalSamples++;
    statsSamples++;

    if (packetIndex >= SAMPLES_PER_PACKET) {

        sendPacket();

        packetIndex = 0;
    }
}


// ============================================================
// SETUP
// ============================================================

void setup() {

    Serial.begin(115200);
    delay(2000);

    pinMode(SENSOR_PIN, INPUT);

    setupBLE();

    nextSampleUs =
        micros() + SAMPLE_INTERVAL_US;

    Serial.println();
    Serial.println("==============================");
    Serial.println("RAW ADC BLE STREAM");
    Serial.println("==============================");

    Serial.print("Sample rate: ");
    Serial.println(SAMPLE_RATE_HZ);

    Serial.print("Samples / packet: ");
    Serial.println(SAMPLES_PER_PACKET);

    Serial.print("Packet bytes: ");
    Serial.println(sizeof(RawPacket));

    Serial.println();
}


// ============================================================
// LOOP
// ============================================================

void loop() {

    uint32_t nowUs = micros();

    if ((int32_t)(nowUs - nextSampleUs) >= 0) {

        acquireSample();

        nextSampleUs += SAMPLE_INTERVAL_US;

        // If badly delayed, resynchronize
        if ((int32_t)(micros() - nextSampleUs) >
            (int32_t)(SAMPLE_INTERVAL_US * 5)) {

            nextSampleUs =
                micros() + SAMPLE_INTERVAL_US;
        }
    }


    // Statistics

    uint32_t nowMs = millis();

    if (nowMs - lastStatsMs >= 1000) {

        lastStatsMs += 1000;

        Serial.print("connected=");
        Serial.print(
            Bluefruit.connected() ? "YES" : "NO"
        );

        Serial.print(" samples/s=");
        Serial.print(statsSamples);

        Serial.print(" attempted=");
        Serial.print(attemptedPackets);

        Serial.print(" sent=");
        Serial.print(successfulPackets);

        Serial.print(" failed=");
        Serial.println(failedPackets);

        statsSamples = 0;
    }
}