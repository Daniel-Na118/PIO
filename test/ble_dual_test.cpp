#include <Arduino.h>
#include <bluefruit.h>

// ============================================================
// SENSOR CONFIGURATION
// ============================================================

const int TEMPLE_PIN = A0;
const int EAR_PIN    = A1;

const uint16_t SAMPLE_RATE_HZ = 500;
const uint32_t SAMPLE_INTERVAL_US =
    1000000UL / SAMPLE_RATE_HZ;

// 40 timestamps worth of:
// temple, ear
//
// 40 pairs * 4 bytes = 160 bytes
// header = 6 bytes
// total = 166 bytes
const uint8_t SAMPLES_PER_PACKET = 40;


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
// PACKET
// ============================================================

struct __attribute__((packed)) DualChannelPacket {

    uint16_t sequence;

    // Timestamp of first temple/ear pair
    uint32_t timestamp_us;

    // Interleaved:
    //
    // temple[0], ear[0],
    // temple[1], ear[1],
    // ...
    uint16_t samples[SAMPLES_PER_PACKET * 2];
};

DualChannelPacket packet;

uint8_t packetIndex = 0;
uint16_t sequenceNumber = 0;


// ============================================================
// TIMING / STATISTICS
// ============================================================

uint32_t nextSampleUs = 0;

uint32_t totalSamplePairs = 0;

uint32_t attemptedPackets = 0;
uint32_t successfulPackets = 0;
uint32_t failedPackets = 0;

uint32_t statsSamples = 0;
uint32_t lastStatsMs = 0;


// ============================================================
// BLE SETUP
// ============================================================

void setupBLE() {

    // Configure before Bluefruit.begin()
    Bluefruit.configPrphConn(
        247,   // maximum MTU
        6,     // event length
        8,     // notification queue
        1      // write command queue
    );

    Bluefruit.begin();

    Bluefruit.setName("GlassesDual");

    sensorService.begin();

    rawCharacteristic.setProperties(
        CHR_PROPS_READ | CHR_PROPS_NOTIFY
    );

    rawCharacteristic.setPermission(
        SECMODE_OPEN,
        SECMODE_NO_ACCESS
    );

    rawCharacteristic.setFixedLen(
        sizeof(DualChannelPacket)
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
// SEND BLE PACKET
// ============================================================

void sendPacket() {

    if (!Bluefruit.connected()) {
        return;
    }

    packet.sequence = sequenceNumber;

    attemptedPackets++;

    bool success =
        rawCharacteristic.notify(
            (uint8_t*)&packet,
            sizeof(packet)
        );

    if (success) {

        successfulPackets++;
        sequenceNumber++;

    } else {

        failedPackets++;
    }
}


// ============================================================
// SAMPLE BOTH CONTACTS
// ============================================================

void acquireSamplePair() {

    if (packetIndex == 0) {
        packet.timestamp_us = micros();
    }

    uint16_t temple =
        analogRead(TEMPLE_PIN);

    uint16_t ear =
        analogRead(EAR_PIN);

    int base =
        packetIndex * 2;

    packet.samples[base] =
        temple;

    packet.samples[base + 1] =
        ear;

    packetIndex++;

    totalSamplePairs++;
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

    pinMode(TEMPLE_PIN, INPUT);
    pinMode(EAR_PIN, INPUT);

    setupBLE();

    nextSampleUs =
        micros() + SAMPLE_INTERVAL_US;

    Serial.println();
    Serial.println("============================");
    Serial.println("DUAL CONTACT BLE SENSOR");
    Serial.println("============================");

    Serial.print("Sampling: ");
    Serial.print(SAMPLE_RATE_HZ);
    Serial.println(" pairs/sec");

    Serial.println("A0 = TEMPLE");
    Serial.println("A1 = RIGHT EAR");

    Serial.print("Packet bytes: ");
    Serial.println(sizeof(DualChannelPacket));
}


// ============================================================
// LOOP
// ============================================================

void loop() {

    uint32_t nowUs =
        micros();

    if ((int32_t)(nowUs - nextSampleUs) >= 0) {

        acquireSamplePair();

        nextSampleUs +=
            SAMPLE_INTERVAL_US;

        if ((int32_t)(micros() - nextSampleUs) >
            (int32_t)(SAMPLE_INTERVAL_US * 5)) {

            nextSampleUs =
                micros() + SAMPLE_INTERVAL_US;
        }
    }


    // Once-per-second debug output

    uint32_t nowMs =
        millis();

    if (nowMs - lastStatsMs >= 1000) {

        lastStatsMs += 1000;

        Serial.print("BLE=");
        Serial.print(
            Bluefruit.connected()
            ? "YES"
            : "NO"
        );

        Serial.print(" pairs/s=");
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