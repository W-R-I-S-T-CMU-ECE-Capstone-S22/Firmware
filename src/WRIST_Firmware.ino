#include <MQTT.h>
#include <SparkFunMAX17043/SparkFunMAX17043.h>

#include "Adafruit_VL6180X.h"

// #define PRINT_DATA
// #define PRINT_BATT

#define ALPHA           0.5

// addresses of the sensors
#define LOX1_ADDRESS    0x30
#define LOX2_ADDRESS    0x31
#define LOX3_ADDRESS    0x32
#define LOX4_ADDRESS    0x33
#define LOX5_ADDRESS    0x34
#define LOX6_ADDRESS    0x35
// MAX17043_ADDRESS is 0x36, so we cannot use 0x36
#define LOX7_ADDRESS    0x37
#define LOX8_ADDRESS    0x38
#define LOX9_ADDRESS    0x39
#define LOX10_ADDRESS   0x3A

// shutdown pins
#define SHT_LOX1    D2
#define SHT_LOX2    D3
#define SHT_LOX3    D4
#define SHT_LOX4    D5
#define SHT_LOX5    D6
#define SHT_LOX6    A0
#define SHT_LOX7    A1
#define SHT_LOX8    A2
#define SHT_LOX9    A3
#define SHT_LOX10   A4

// MQTT defines
#define MQTT_HOST   "mqtt.eclipseprojects.io"
#define MQTT_PORT   1883
#define MQTT_NAME   ("wrist-watch" + String(Time.now()))

#define DATA_TOPIC  "wrist/data/sensors"
#define BATT_TOPIC  "wrist/batt/sensors"
#define BATT_TOPIC_ASK  "wrist/batt/ask"
#define CTRL_TOPIC  "wrist/ctrl/sensors"

// objects for the VL6180X
Adafruit_VL6180X lox1  = Adafruit_VL6180X();
Adafruit_VL6180X lox2  = Adafruit_VL6180X();
Adafruit_VL6180X lox3  = Adafruit_VL6180X();
Adafruit_VL6180X lox4  = Adafruit_VL6180X();
Adafruit_VL6180X lox5  = Adafruit_VL6180X();
Adafruit_VL6180X lox6  = Adafruit_VL6180X();
Adafruit_VL6180X lox7  = Adafruit_VL6180X();
Adafruit_VL6180X lox8  = Adafruit_VL6180X();
Adafruit_VL6180X lox9  = Adafruit_VL6180X();
Adafruit_VL6180X lox10 = Adafruit_VL6180X();

Adafruit_VL6180X *sensors[] = {&lox1, &lox2, &lox3, &lox4, &lox5, &lox6, &lox7, &lox8, &lox9, &lox10};

const uint8_t COUNT_SENSORS = sizeof(sensors) / sizeof(sensors[0]);

uint8_t sensor_idx = 0;
uint8_t sensor_ranges_prev[COUNT_SENSORS];
uint8_t sensor_status[COUNT_SENSORS];

// objects for MQTT
MQTT client(MQTT_HOST, MQTT_PORT, mqtt_callback);
char data[sizeof(uint32_t) + COUNT_SENSORS];

void mqtt_callback(char *topic, byte *payload, unsigned int length) {
    // Serial.println(topic);
    if (!strcmp(topic, BATT_TOPIC_ASK)) {
        double voltage = lipo.getVoltage();
        double soc = lipo.getSOC();

        String batt_data = "";
        uint32_t timestamp = Time.now();
        batt_data += String(timestamp) + ",";
        batt_data += String(voltage) + ",";
        batt_data += String(soc);

#ifdef PRINT_BATT
        Serial.print("Voltage: ");
        Serial.print(voltage);
        Serial.println(" V");

        Serial.print("Percentage: ");
        Serial.print(soc);
        Serial.println(" %");
        Serial.println();
#endif

        if (client.isConnected()) {
            client.publish(BATT_TOPIC, batt_data);
        }
    }
}

void set_sht_pins(byte state) {
    for (int pin = SHT_LOX1; pin <= SHT_LOX10; pin++) {
        digitalWrite(pin, state);
    }
}

void start_sensors() {
    // all reset
    set_sht_pins(LOW);
    delay(10);

    // all unreset
    set_sht_pins(HIGH);
    delay(10);

    // activating LOX1 and reseting LOX2
    digitalWrite(SHT_LOX1, HIGH);
    digitalWrite(SHT_LOX2, LOW);
    digitalWrite(SHT_LOX3, LOW);
    digitalWrite(SHT_LOX4, LOW);
    digitalWrite(SHT_LOX5, LOW);
    digitalWrite(SHT_LOX6, LOW);
    digitalWrite(SHT_LOX7, LOW);
    digitalWrite(SHT_LOX8, LOW);
    digitalWrite(SHT_LOX9, LOW);
    digitalWrite(SHT_LOX10, LOW);

    // initing LOX1
    if (!lox1.begin()) {
        Serial.println(F("Failed to boot first VL6180X"));
        while (1);
    }
    lox1.setAddress(LOX1_ADDRESS);
    delay(10);

    // activating LOX2
    digitalWrite(SHT_LOX2, HIGH);
    delay(10);

    // initing LOX2
    if (!lox2.begin()) {
        Serial.println(F("Failed to boot second VL6180X"));
        while (1);
    }
    lox2.setAddress(LOX2_ADDRESS);

    // activating LOX3
    digitalWrite(SHT_LOX3, HIGH);
    delay(10);

    // initing LOX3
    if (!lox3.begin()) {
        Serial.println(F("Failed to boot third VL6180X"));
        while (1);
    }
    lox3.setAddress(LOX3_ADDRESS);

    // activating LOX4
    digitalWrite(SHT_LOX4, HIGH);
    delay(10);

    // initing LOX4
    if (!lox4.begin()) {
        Serial.println(F("Failed to boot fourth VL6180X"));
        while (1);
    }
    lox4.setAddress(LOX4_ADDRESS);

    // activating LOX5
    digitalWrite(SHT_LOX5, HIGH);
    delay(10);

    // initing LOX5
    if (!lox5.begin()) {
        Serial.println(F("Failed to boot fifth VL6180X"));
        while (1);
    }
    lox5.setAddress(LOX5_ADDRESS);

    // activating LOX6
    digitalWrite(SHT_LOX6, HIGH);
    delay(10);

    // initing LOX6
    if (!lox6.begin()) {
        Serial.println(F("Failed to boot sixth VL6180X"));
        while (1);
    }
    lox6.setAddress(LOX6_ADDRESS);

    // activating LOX7
    digitalWrite(SHT_LOX7, HIGH);
    delay(10);

    // initing LOX7
    if (!lox7.begin()) {
        Serial.println(F("Failed to boot seventh VL6180X"));
        while (1);
    }
    lox7.setAddress(LOX7_ADDRESS);

    // activating LOX8
    digitalWrite(SHT_LOX8, HIGH);
    delay(10);

    // initing LOX8
    if (!lox8.begin()) {
        Serial.println(F("Failed to boot eigth VL6180X"));
        while (1);
    }
    lox8.setAddress(LOX8_ADDRESS);

    // activating LOX9
    digitalWrite(SHT_LOX9, HIGH);
    delay(10);

    // initing LOX9
    if (!lox9.begin()) {
        Serial.println(F("Failed to boot ninth VL6180X"));
        while (1);
    }
    lox9.setAddress(LOX9_ADDRESS);

    // activating LOX10
    digitalWrite(SHT_LOX10, HIGH);
    delay(10);

    // initing LOX10
    if (!lox10.begin()) {
        Serial.println(F("Failed to boot tenth VL6180X"));
        while (1);
    }
    lox10.setAddress(LOX10_ADDRESS);
}

void round_robin_read_sensors() {
    uint8_t range_lox = sensors[sensor_idx]->readRange();
    uint8_t status_lox = sensors[sensor_idx]->readRangeStatus();

    sensor_status[sensor_idx] = status_lox;
    if (status_lox == VL6180X_ERROR_NONE) {
        // apply EWMA filter
        uint8_t val = ALPHA * range_lox + (1.0 - ALPHA) * sensor_ranges_prev[sensor_idx];

        // save EWMA filtered value
        data[sizeof(uint32_t) + sensor_idx] = val;
        sensor_ranges_prev[sensor_idx] = val;
    }
    else {
        data[sizeof(uint32_t) + sensor_idx] = -1;
    }

    sensor_idx = (sensor_idx + 1) % COUNT_SENSORS;
}

void setup() {
    Serial.begin(115200);

    // wait until serial port opens for native USB devices
    while (!Serial) {
        delay(1);
    }

    Serial.println("Connecting to MQTT host...");
    client.connect(MQTT_NAME);
    while (!client.isConnected()) {
        Serial.println("Connecting to MQTT host...");
        client.connect(MQTT_NAME);
        delay(1000);
    }

    client.subscribe(BATT_TOPIC_ASK);

    pinMode(SHT_LOX1, OUTPUT);
    pinMode(SHT_LOX2, OUTPUT);
    pinMode(SHT_LOX3, OUTPUT);
    pinMode(SHT_LOX4, OUTPUT);
    pinMode(SHT_LOX5, OUTPUT);
    pinMode(SHT_LOX6, OUTPUT);
    pinMode(SHT_LOX7, OUTPUT);
    pinMode(SHT_LOX8, OUTPUT);
    pinMode(SHT_LOX9, OUTPUT);
    pinMode(SHT_LOX10, OUTPUT);

    Serial.println("Shutdown pins inited...");

    set_sht_pins(LOW);
    Serial.println("Starting...");
    start_sensors();

    // initialize the MAX17043 LiPo fuel gauge
    lipo.begin();
    // quick start restarts the MAX17043 in hopes of getting a more accurate guess for the SOC.
    lipo.quickStart();
    // set alert threshold to 20%.
    lipo.setThreshold(20);

    Serial.println("Ready!");
}

void loop() {
    round_robin_read_sensors();

    if (sensor_idx == COUNT_SENSORS - 1) {
#ifdef PRINT_DATA
        for (int i = 0; i < COUNT_SENSORS; i++) {
            if (sensor_status[i] == VL6180X_ERROR_NONE) Serial.print(data[i], DEC);
            else Serial.print("###");

            if (i != COUNT_SENSORS-1) Serial.print(" : ");
        }
        Serial.println();
#endif
        uint32_t timestamp = Time.now();
        *(uint32_t *)(&data) = timestamp;

        if (client.isConnected()) {
            client.publish(DATA_TOPIC, data);
        }
    }

    if (client.isConnected()) client.loop();
}
